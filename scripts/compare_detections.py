#!/usr/bin/env python3

import argparse
import json
import math
import statistics
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


@dataclass
class Record:
    timestamp_ms: float | None
    detections: list[dict[str, Any]]


def parse_timestamp(value: Any) -> float | None:
    if isinstance(value, (int, float)):
        return float(value) * 1000.0 if value < 100_000_000_000 else float(value)
    if isinstance(value, str):
        try:
            return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp() * 1000.0
        except ValueError:
            return None
    return None


def json_from_line(line: str) -> Any:
    line = line.strip()
    if not line:
        return None
    starts = [position for position in (line.find("{"), line.find("[")) if position >= 0]
    if not starts:
        raise ValueError("line does not contain JSON")
    return json.loads(line[min(starts):])


def record_from_value(value: Any) -> Record:
    envelope = value if isinstance(value, dict) else {}
    payload = envelope.get("payload", value) if isinstance(envelope, dict) else value
    if isinstance(payload, dict):
        detections = payload.get("list", [])
    elif isinstance(payload, list):
        detections = payload
    else:
        detections = []
    if not isinstance(detections, list):
        raise ValueError("detection payload 'list' must be an array")

    timestamp = parse_timestamp(envelope.get("captured_at"))
    if timestamp is None:
        timestamp = parse_timestamp(envelope.get("timestamp"))
    if timestamp is None:
        object_times = [
            parsed
            for detection in detections
            if isinstance(detection, dict)
            for parsed in [parse_timestamp(detection.get("timestamp"))]
            if parsed is not None
        ]
        timestamp = max(object_times) if object_times else None
    return Record(timestamp, [item for item in detections if isinstance(item, dict)])


def load_records(path: Path) -> tuple[list[Record], int]:
    records: list[Record] = []
    rejected = 0
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                value = json_from_line(line)
                if value is not None:
                    records.append(record_from_value(value))
            except (json.JSONDecodeError, ValueError) as error:
                rejected += 1
                print(f"warning: {path}:{line_number}: {error}")
    return records, rejected


def pair_records(
    baseline: list[Record], shadow: list[Record], window_ms: float
) -> tuple[list[tuple[Record, Record]], list[Record], list[Record]]:
    if not baseline or not shadow:
        return [], baseline, shadow

    timed_baseline = [(index, record) for index, record in enumerate(baseline) if record.timestamp_ms is not None]
    timed_shadow = [(index, record) for index, record in enumerate(shadow) if record.timestamp_ms is not None]
    available = {index for index, _ in timed_shadow}
    pairs: list[tuple[Record, Record]] = []
    paired_baseline: set[int] = set()
    paired_shadow: set[int] = set()
    for left_index, left in timed_baseline:
        best_index = min(
            available,
            key=lambda index: abs(shadow[index].timestamp_ms - left.timestamp_ms),
            default=None,
        )
        if best_index is None:
            break
        difference = abs(shadow[best_index].timestamp_ms - left.timestamp_ms)
        if difference <= window_ms:
            pairs.append((left, shadow[best_index]))
            paired_baseline.add(left_index)
            paired_shadow.add(best_index)
            available.remove(best_index)

    untimed_baseline = [
        (index, record) for index, record in enumerate(baseline) if record.timestamp_ms is None
    ]
    untimed_shadow = [
        (index, record) for index, record in enumerate(shadow) if record.timestamp_ms is None
    ]
    for (left_index, left), (right_index, right) in zip(untimed_baseline, untimed_shadow):
        pairs.append((left, right))
        paired_baseline.add(left_index)
        paired_shadow.add(right_index)

    return (
        pairs,
        [record for index, record in enumerate(baseline) if index not in paired_baseline],
        [record for index, record in enumerate(shadow) if index not in paired_shadow],
    )


def box(detection: dict[str, Any]) -> tuple[float, float, float, float] | None:
    try:
        left = float(detection["x"])
        top = float(detection["y"])
        width = float(detection["w"])
        height = float(detection["h"])
    except (KeyError, TypeError, ValueError):
        return None
    if width <= 0 or height <= 0:
        return None
    return left, top, left + width, top + height


def intersection_over_union(left: dict[str, Any], right: dict[str, Any]) -> float:
    left_box = box(left)
    right_box = box(right)
    if left_box is None or right_box is None:
        return 0.0
    x1 = max(left_box[0], right_box[0])
    y1 = max(left_box[1], right_box[1])
    x2 = min(left_box[2], right_box[2])
    y2 = min(left_box[3], right_box[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = (left_box[2] - left_box[0]) * (left_box[3] - left_box[1])
    right_area = (right_box[2] - right_box[0]) * (right_box[3] - right_box[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0


def center_error(left: dict[str, Any], right: dict[str, Any]) -> float:
    left_box = box(left)
    right_box = box(right)
    if left_box is None or right_box is None:
        return math.nan
    left_center = ((left_box[0] + left_box[2]) / 2, (left_box[1] + left_box[3]) / 2)
    right_center = ((right_box[0] + right_box[2]) / 2, (right_box[1] + right_box[3]) / 2)
    return math.dist(left_center, right_center)


def match_detections(
    baseline: list[dict[str, Any]], shadow: list[dict[str, Any]], minimum_iou: float
) -> tuple[list[tuple[dict[str, Any], dict[str, Any], float]], int, int]:
    candidates = []
    for left_index, left in enumerate(baseline):
        for right_index, right in enumerate(shadow):
            left_class = str(left.get("class", "")).strip().casefold()
            right_class = str(right.get("class", "")).strip().casefold()
            overlap = intersection_over_union(left, right)
            if left_class and left_class == right_class and overlap >= minimum_iou:
                candidates.append((overlap, left_index, right_index))
    candidates.sort(reverse=True)

    used_left: set[int] = set()
    used_right: set[int] = set()
    matches = []
    for overlap, left_index, right_index in candidates:
        if left_index in used_left or right_index in used_right:
            continue
        used_left.add(left_index)
        used_right.add(right_index)
        matches.append((baseline[left_index], shadow[right_index], overlap))
    return matches, len(baseline) - len(used_left), len(shadow) - len(used_right)


def rate(records: list[Record]) -> float | None:
    timestamps = [record.timestamp_ms for record in records if record.timestamp_ms is not None]
    if len(timestamps) < 2 or max(timestamps) == min(timestamps):
        return None
    return (len(timestamps) - 1) * 1000.0 / (max(timestamps) - min(timestamps))


def empty_transitions(records: list[Record]) -> int:
    states = [not record.detections for record in records]
    return sum(previous != current for previous, current in zip(states, states[1:]))


def rounded(value: float | None) -> float | None:
    return round(value, 3) if value is not None and not math.isnan(value) else None


def summarize(
    baseline: list[Record], shadow: list[Record], window_ms: float, minimum_iou: float
) -> dict[str, Any]:
    pairs, unpaired_baseline_records, unpaired_shadow_records = pair_records(
        baseline, shadow, window_ms
    )
    overlaps: list[float] = []
    center_errors: list[float] = []
    confidence_deltas: list[float] = []
    unmatched_baseline = 0
    unmatched_shadow = 0
    count_deltas = []
    synthetic_detections = sum(
        bool(item.get("synthetic")) for record in shadow for item in record.detections
    )
    synthetic_messages = sum(
        any(bool(item.get("synthetic")) for item in record.detections) for record in shadow
    )

    for left_record, right_record in pairs:
        count_deltas.append(len(right_record.detections) - len(left_record.detections))
        matches, left_unmatched, right_unmatched = match_detections(
            left_record.detections, right_record.detections, minimum_iou
        )
        unmatched_baseline += left_unmatched
        unmatched_shadow += right_unmatched
        for left, right, overlap in matches:
            overlaps.append(overlap)
            error = center_error(left, right)
            if not math.isnan(error):
                center_errors.append(error)
            try:
                confidence_deltas.append(float(right["confidence"]) - float(left["confidence"]))
            except (KeyError, TypeError, ValueError):
                pass

    return {
        "baseline_messages": len(baseline),
        "shadow_messages": len(shadow),
        "paired_messages": len(pairs),
        "unpaired_baseline_messages": len(unpaired_baseline_records),
        "unpaired_shadow_messages": len(unpaired_shadow_records),
        "baseline_rate_hz": rounded(rate(baseline)),
        "shadow_rate_hz": rounded(rate(shadow)),
        "baseline_empty_transitions": empty_transitions(baseline),
        "shadow_empty_transitions": empty_transitions(shadow),
        "mean_count_delta": rounded(statistics.fmean(count_deltas)) if count_deltas else None,
        "matched_detections": len(overlaps),
        "unmatched_baseline_detections": unmatched_baseline
        + sum(len(record.detections) for record in unpaired_baseline_records),
        "unmatched_shadow_detections": unmatched_shadow
        + sum(len(record.detections) for record in unpaired_shadow_records),
        "mean_iou": rounded(statistics.fmean(overlaps)) if overlaps else None,
        "median_iou": rounded(statistics.median(overlaps)) if overlaps else None,
        "mean_center_error": rounded(statistics.fmean(center_errors)) if center_errors else None,
        "mean_confidence_delta": rounded(statistics.fmean(confidence_deltas))
        if confidence_deltas
        else None,
        "synthetic_shadow_messages": synthetic_messages,
        "synthetic_shadow_detections": synthetic_detections,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare DataQ and dataq_ddh detection captures")
    parser.add_argument("baseline", type=Path, help="DataQ JSONL or mosquitto_sub capture")
    parser.add_argument("shadow", type=Path, help="dataq_ddh JSONL or mosquitto_sub capture")
    parser.add_argument("--window-ms", type=float, default=750.0, help="Maximum timestamp pairing gap")
    parser.add_argument(
        "--minimum-iou", type=float, default=0.25, help="Minimum IoU for a detection match"
    )
    arguments = parser.parse_args()

    if arguments.window_ms < 0 or not 0 <= arguments.minimum_iou <= 1:
        parser.error("--window-ms must be nonnegative and --minimum-iou must be between 0 and 1")

    baseline, baseline_rejected = load_records(arguments.baseline)
    shadow, shadow_rejected = load_records(arguments.shadow)
    result = summarize(baseline, shadow, arguments.window_ms, arguments.minimum_iou)
    result["baseline_rejected_lines"] = baseline_rejected
    result["shadow_rejected_lines"] = shadow_rejected
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if baseline and shadow else 2


if __name__ == "__main__":
    raise SystemExit(main())