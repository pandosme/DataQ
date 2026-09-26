#!/usr/bin/env python3

import argparse
import json
import math
import queue
import signal
import sys
import threading
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

if __package__:
    from .compare_detections import box, center_error, intersection_over_union
else:
    from compare_detections import box, center_error, intersection_over_union


Detection = dict[str, Any]


@dataclass(frozen=True)
class Difference:
    kind: str
    dataq: Detection | None
    ddh: Detection | None
    iou: float | None = None
    center_error: float | None = None


@dataclass
class Candidate:
    difference: Difference
    first_seen: float
    last_observation: tuple[float, float]
    observations: int = 1
    last_reported: float | None = None


@dataclass(frozen=True)
class MessageEvent:
    side: str
    topic: str
    payload: bytes
    received_at: float
    captured_at: str


@dataclass(frozen=True)
class Snapshot:
    detections: list[Detection]
    received_at: float


def parse_detection_payload(raw_payload: bytes | str) -> tuple[dict[str, Any], list[Detection]]:
    try:
        text = raw_payload.decode("utf-8") if isinstance(raw_payload, bytes) else raw_payload
        payload = json.loads(text)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid JSON payload: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("payload must be a JSON object")
    detections = payload.get("list")
    if not isinstance(detections, list):
        raise ValueError("payload 'list' must be an array")
    if any(not isinstance(item, dict) for item in detections):
        raise ValueError("every detection must be a JSON object")
    return payload, detections


def normalized_class(detection: Detection) -> str:
    return str(detection.get("class", "")).strip().casefold()


def candidate_matches(
    dataq: list[Detection],
    ddh: list[Detection],
    minimum_iou: float,
    require_same_class: bool,
) -> list[tuple[float, int, int]]:
    candidates: list[tuple[float, int, int]] = []
    for dataq_index, dataq_detection in enumerate(dataq):
        for ddh_index, ddh_detection in enumerate(ddh):
            same_class = normalized_class(dataq_detection) == normalized_class(ddh_detection)
            if require_same_class != same_class:
                continue
            overlap = intersection_over_union(dataq_detection, ddh_detection)
            if overlap >= minimum_iou:
                candidates.append((overlap, dataq_index, ddh_index))
    return sorted(candidates, reverse=True)


def associate_detections(
    dataq: list[Detection], ddh: list[Detection], minimum_iou: float
) -> tuple[list[tuple[Detection, Detection, float]], list[Detection], list[Detection]]:
    used_dataq: set[int] = set()
    used_ddh: set[int] = set()
    matches: list[tuple[Detection, Detection, float]] = []

    for require_same_class in (True, False):
        for overlap, dataq_index, ddh_index in candidate_matches(
            dataq, ddh, minimum_iou, require_same_class
        ):
            if dataq_index in used_dataq or ddh_index in used_ddh:
                continue
            used_dataq.add(dataq_index)
            used_ddh.add(ddh_index)
            matches.append((dataq[dataq_index], ddh[ddh_index], overlap))

    return (
        matches,
        [item for index, item in enumerate(dataq) if index not in used_dataq],
        [item for index, item in enumerate(ddh) if index not in used_ddh],
    )


def compare_snapshots(
    dataq: list[Detection],
    ddh: list[Detection],
    *,
    match_iou: float = 0.25,
    position_iou: float = 0.75,
    center_threshold: float = 15.0,
    report_synthetic: bool = False,
    edge_margin: float = 5.0,
    narrow_width: float = 15.0,
) -> list[Difference]:
    matches, only_dataq, only_ddh = associate_detections(dataq, ddh, match_iou)
    differences: list[Difference] = []

    for dataq_detection, ddh_detection, overlap in matches:
        error = center_error(dataq_detection, ddh_detection)
        if normalized_class(dataq_detection) != normalized_class(ddh_detection):
            differences.append(
                Difference("class", dataq_detection, ddh_detection, overlap, error)
            )
        if bool(dataq_detection.get("active", True)) != bool(ddh_detection.get("active", True)):
            differences.append(
                Difference("active", dataq_detection, ddh_detection, overlap, error)
            )
        if report_synthetic and bool(dataq_detection.get("synthetic", False)) != bool(
            ddh_detection.get("synthetic", False)
        ):
            differences.append(
                Difference("synthetic", dataq_detection, ddh_detection, overlap, error)
            )
        center_differs = not math.isnan(error) and error > center_threshold
        iou_differs = overlap < position_iou
        edge_case = is_edge_or_narrow(
            dataq_detection, edge_margin, narrow_width
        ) or is_edge_or_narrow(ddh_detection, edge_margin, narrow_width)
        if center_differs or (iou_differs and not edge_case):
            differences.append(
                Difference("position", dataq_detection, ddh_detection, overlap, error)
            )

    differences.extend(Difference("only-dataq", item, None) for item in only_dataq)
    differences.extend(Difference("only-ddh", None, item) for item in only_ddh)
    return differences


def detection_box(detection: Detection | None) -> tuple[float, float, float, float] | None:
    if detection is None:
        return None
    edges = box(detection)
    if edges is None:
        return None
    left, top, right, bottom = edges
    return left, top, right - left, bottom - top


def is_edge_or_narrow(
    detection: Detection, edge_margin: float, narrow_width: float
) -> bool:
    geometry = detection_box(detection)
    if geometry is None:
        return False
    left, _top, width, _height = geometry
    return left <= edge_margin or left + width >= 1000.0 - edge_margin or width <= narrow_width


def comparable_detections(
    detections: list[Detection], *, include_inactive: bool = False
) -> list[Detection]:
    return [
        detection
        for detection in detections
        if detection_box(detection) is not None
        and (include_inactive or bool(detection.get("active", True)))
    ]


def detection_identity(side: str, detection: Detection | None) -> tuple[Any, ...] | None:
    if detection is None:
        return None
    identifier = detection.get("id")
    if identifier is not None:
        return side, "id", str(identifier)
    return side, "box", normalized_class(detection), detection_box(detection)


def difference_key(difference: Difference) -> tuple[Any, ...]:
    return (
        difference.kind,
        detection_identity("dataq", difference.dataq),
        detection_identity("ddh", difference.ddh),
    )


class DifferenceTracker:
    def __init__(
        self, hold_seconds: float, repeat_seconds: float, minimum_observations: int = 2
    ) -> None:
        self.hold_seconds = hold_seconds
        self.repeat_seconds = repeat_seconds
        self.minimum_observations = minimum_observations
        self.candidates: dict[tuple[Any, ...], Candidate] = {}

    def update(
        self,
        differences: list[Difference],
        now: float,
        observation: tuple[float, float],
    ) -> list[Difference]:
        current = {difference_key(item): item for item in differences}
        for key in self.candidates.keys() - current.keys():
            del self.candidates[key]

        reports: list[Difference] = []
        for key, difference in current.items():
            candidate = self.candidates.get(key)
            if candidate is None:
                candidate = Candidate(difference, now, observation)
                self.candidates[key] = candidate
            else:
                candidate.difference = difference
                if candidate.last_observation != observation:
                    candidate.last_observation = observation
                    candidate.observations += 1

            held_long_enough = now - candidate.first_seen >= self.hold_seconds
            observed_often_enough = candidate.observations >= self.minimum_observations
            repeat_due = (
                candidate.last_reported is None
                or now - candidate.last_reported >= self.repeat_seconds
            )
            if held_long_enough and observed_often_enough and repeat_due:
                reports.append(candidate.difference)
                candidate.last_reported = now
        return reports


class CaptureWriter:
    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self.paths = {
            "dataq": directory / "baseline.jsonl",
            "ddh": directory / "shadow.jsonl",
        }
        self.streams = {
            side: path.open("w", encoding="utf-8") for side, path in self.paths.items()
        }

    def write(
        self, side: str, topic: str, payload: dict[str, Any], captured_at: str
    ) -> None:
        json.dump(
            {"captured_at": captured_at, "topic": topic, "payload": payload},
            self.streams[side],
            separators=(",", ":"),
        )
        self.streams[side].write("\n")
        self.streams[side].flush()

    def close(self) -> None:
        for stream in self.streams.values():
            stream.close()


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def rounded_metric(value: float | None) -> float | None:
    return round(value, 3) if value is not None and not math.isnan(value) else None


def difference_as_dict(difference: Difference, observed_at: str) -> dict[str, Any]:
    return {
        "observed_at": observed_at,
        "kind": difference.kind,
        "dataq": difference.dataq,
        "ddh": difference.ddh,
        "iou": rounded_metric(difference.iou),
        "center_error": rounded_metric(difference.center_error),
    }


def detection_summary(side: str, detection: Detection | None) -> str:
    if detection is None:
        return f"{side}=none"
    return (
        f"{side}[id={detection.get('id', '?')} class={detection.get('class', '?')} "
        f"active={bool(detection.get('active', True))} "
        f"synthetic={bool(detection.get('synthetic', False))} "
        f"box={detection_box(detection)}]"
    )


def format_difference(difference: Difference, observed_at: str) -> str:
    metrics = ""
    if difference.iou is not None:
        metrics += f" iou={difference.iou:.3f}"
    if difference.center_error is not None and not math.isnan(difference.center_error):
        metrics += f" center={difference.center_error:.1f}"
    return (
        f"{observed_at} {difference.kind} "
        f"{detection_summary('dataq', difference.dataq)} "
        f"{detection_summary('ddh', difference.ddh)}{metrics}"
    )


def format_summary(counts: Counter[str]) -> str:
    if not counts:
        return "no persistent differences reported"
    return "persistent differences: " + ", ".join(
        f"{kind}={count}" for kind, count in sorted(counts.items())
    )


def fresh_snapshot_pair(
    snapshots: dict[str, Snapshot], now: float, window_seconds: float
) -> tuple[list[Detection], list[Detection]] | None:
    dataq = snapshots.get("dataq")
    ddh = snapshots.get("ddh")
    if dataq is None or ddh is None:
        return None
    if now - dataq.received_at > window_seconds or now - ddh.received_at > window_seconds:
        return None
    return dataq.detections, ddh.detections


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Report persistent differences between live DataQ and DDH detections"
    )
    parser.add_argument("--host", default="mqtt.internal", help="MQTT broker hostname")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--serial", default="B8A44F3024BB", help="Camera serial number")
    parser.add_argument("--dataq-prefix", default="dataq", help="Production topic prefix")
    parser.add_argument("--ddh-prefix", default="dataq-ddh", help="DDH topic prefix")
    parser.add_argument("--client-id", default="", help="Optional MQTT client ID")
    parser.add_argument(
        "--window-ms", type=float, default=750.0, help="Maximum snapshot age"
    )
    parser.add_argument(
        "--hold-ms", type=float, default=1000.0, help="Persistence required before reporting"
    )
    parser.add_argument(
        "--minimum-observations",
        type=int,
        default=2,
        help="Distinct snapshot comparisons required before reporting",
    )
    parser.add_argument(
        "--repeat-seconds",
        type=float,
        default=5.0,
        help="Minimum interval between repeated reports",
    )
    parser.add_argument(
        "--match-iou", type=float, default=0.25, help="Minimum IoU for association"
    )
    parser.add_argument(
        "--position-iou",
        type=float,
        default=0.75,
        help="Report position when IoU falls below this value",
    )
    parser.add_argument(
        "--center-threshold",
        type=float,
        default=15.0,
        help="Report position when center distance exceeds this value",
    )
    parser.add_argument(
        "--edge-margin",
        type=float,
        default=5.0,
        help="Ignore IoU-only differences this close to a horizontal edge",
    )
    parser.add_argument(
        "--narrow-width",
        type=float,
        default=15.0,
        help="Ignore IoU-only differences for boxes no wider than this",
    )
    parser.add_argument(
        "--stale-seconds",
        type=float,
        default=0.0,
        help="Silence before warning that a source is stale (0 disables warnings)",
    )
    parser.add_argument(
        "--include-inactive",
        action="store_true",
        help="Include inactive terminal detections in scene comparison",
    )
    parser.add_argument(
        "--include-synthetic-differences",
        action="store_true",
        help="Report DDH synthetic-state differences",
    )
    parser.add_argument("--json", action="store_true", help="Emit differences as JSONL")
    parser.add_argument(
        "--capture-dir", type=Path, help="Write baseline.jsonl and shadow.jsonl captures"
    )
    return parser


def validate_arguments(parser: argparse.ArgumentParser, arguments: argparse.Namespace) -> None:
    if not 1 <= arguments.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if arguments.window_ms < 0 or arguments.hold_ms < 0 or arguments.repeat_seconds < 0:
        parser.error("--window-ms, --hold-ms, and --repeat-seconds must be nonnegative")
    if arguments.minimum_observations < 1:
        parser.error("--minimum-observations must be at least 1")
    if not 0 <= arguments.match_iou <= 1 or not 0 <= arguments.position_iou <= 1:
        parser.error("--match-iou and --position-iou must be between 0 and 1")
    if (
        arguments.center_threshold < 0
        or arguments.edge_margin < 0
        or arguments.narrow_width < 0
        or arguments.stale_seconds < 0
    ):
        parser.error("position thresholds and --stale-seconds must be nonnegative")
    if not arguments.serial or not arguments.dataq_prefix or not arguments.ddh_prefix:
        parser.error("--serial and topic prefixes must not be empty")


def run_monitor(arguments: argparse.Namespace) -> int:
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print(
            "error: paho-mqtt 2.x is required; install scripts/requirements.txt",
            file=sys.stderr,
        )
        return 2

    topics = {
        f"{arguments.dataq_prefix.rstrip('/')}/detections/{arguments.serial}": "dataq",
        f"{arguments.ddh_prefix.rstrip('/')}/detections/{arguments.serial}": "ddh",
    }
    events: queue.Queue[MessageEvent] = queue.Queue()
    stop_event = threading.Event()
    snapshots: dict[str, Snapshot] = {}
    stale_reported: set[str] = set()
    received_both = False
    counts: Counter[str] = Counter()
    tracker = DifferenceTracker(
        arguments.hold_ms / 1000.0,
        arguments.repeat_seconds,
        arguments.minimum_observations,
    )
    capture = CaptureWriter(arguments.capture_dir) if arguments.capture_dir else None

    def on_connect(client: Any, userdata: Any, flags: Any, reason_code: Any, properties: Any) -> None:
        if reason_code != 0:
            print(f"MQTT connection failed: {reason_code}", file=sys.stderr)
            return
        client.subscribe([(topic, 0) for topic in topics])
        print(
            f"connected to {arguments.host}:{arguments.port}; subscribed to "
            + ", ".join(topics),
            file=sys.stderr,
        )

    def on_disconnect(
        client: Any,
        userdata: Any,
        disconnect_flags: Any,
        reason_code: Any,
        properties: Any,
    ) -> None:
        if not stop_event.is_set():
            print(f"MQTT disconnected ({reason_code}); reconnecting", file=sys.stderr)

    def on_message(client: Any, userdata: Any, message: Any) -> None:
        side = topics.get(message.topic)
        if side is not None:
            events.put(
                MessageEvent(
                    side,
                    message.topic,
                    bytes(message.payload),
                    time.monotonic(),
                    utc_timestamp(),
                )
            )

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=arguments.client_id)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    def request_stop(signum: int, frame: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    stale_warning_seconds = arguments.stale_seconds

    try:
        client.connect_async(arguments.host, arguments.port, keepalive=60)
        client.loop_start()
        print("waiting for DataQ and DDH snapshots", file=sys.stderr)

        while not stop_event.is_set():
            snapshot_updated = False
            try:
                event = events.get(timeout=0.1)
            except queue.Empty:
                event = None

            if event is not None:
                try:
                    payload, detections = parse_detection_payload(event.payload)
                except ValueError as error:
                    print(f"warning: {event.topic}: {error}", file=sys.stderr)
                else:
                    snapshots[event.side] = Snapshot(
                        comparable_detections(
                            detections, include_inactive=arguments.include_inactive
                        ),
                        event.received_at,
                    )
                    snapshot_updated = True
                    stale_reported.discard(event.side)
                    if capture is not None:
                        capture.write(
                            event.side, event.topic, payload, event.captured_at
                        )

            now = time.monotonic()
            if not received_both and snapshots.keys() >= {"dataq", "ddh"}:
                received_both = True
                print("received both streams; comparing fresh snapshots", file=sys.stderr)

            for side, snapshot in snapshots.items():
                if (
                    stale_warning_seconds > 0
                    and now - snapshot.received_at > stale_warning_seconds
                    and side not in stale_reported
                ):
                    print(f"warning: {side} stream is stale", file=sys.stderr)
                    stale_reported.add(side)

            if not snapshot_updated:
                continue
            pair = fresh_snapshot_pair(snapshots, now, arguments.window_ms / 1000.0)
            if pair is None:
                continue
            differences = compare_snapshots(
                pair[0],
                pair[1],
                match_iou=arguments.match_iou,
                position_iou=arguments.position_iou,
                center_threshold=arguments.center_threshold,
                report_synthetic=arguments.include_synthetic_differences,
                edge_margin=arguments.edge_margin,
                narrow_width=arguments.narrow_width,
            )
            observation = (
                snapshots["dataq"].received_at,
                snapshots["ddh"].received_at,
            )
            for difference in tracker.update(differences, now, observation):
                observed_at = utc_timestamp()
                if arguments.json:
                    print(
                        json.dumps(
                            difference_as_dict(difference, observed_at),
                            separators=(",", ":"),
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                else:
                    print(format_difference(difference, observed_at), flush=True)
                counts[difference.kind] += 1
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        client.disconnect()
        client.loop_stop()
        if capture is not None:
            capture.close()
            print(
                "captures: "
                + ", ".join(f"{side}={path}" for side, path in capture.paths.items()),
                file=sys.stderr,
            )
        print(format_summary(counts), file=sys.stderr)
    return 0


def main() -> int:
    parser = build_parser()
    arguments = parser.parse_args()
    validate_arguments(parser, arguments)
    return run_monitor(arguments)


if __name__ == "__main__":
    raise SystemExit(main())