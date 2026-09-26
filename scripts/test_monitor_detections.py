import tempfile
import unittest
from pathlib import Path

from scripts.compare_detections import load_records
from scripts.monitor_detections import (
    CaptureWriter,
    DifferenceTracker,
    Snapshot,
    build_parser,
    compare_snapshots,
    comparable_detections,
    fresh_snapshot_pair,
    parse_detection_payload,
)


def detection(identifier: str, **overrides: object) -> dict[str, object]:
    item: dict[str, object] = {
        "id": identifier,
        "class": "Car",
        "x": 100,
        "y": 100,
        "w": 100,
        "h": 100,
        "active": True,
    }
    item.update(overrides)
    return item


class CompareSnapshotsTests(unittest.TestCase):
    def test_source_local_ids_are_ignored(self) -> None:
        result = compare_snapshots(
            [detection("42")],
            [detection("9c55b4c4-b9cc-4f65-a245-838d99af8e65")],
        )
        self.assertEqual(result, [])

    def test_small_position_jitter_is_ignored(self) -> None:
        result = compare_snapshots([detection("42")], [detection("uuid", x=110)])
        self.assertEqual(result, [])

    def test_meaningful_position_change_is_reported(self) -> None:
        result = compare_snapshots([detection("42")], [detection("uuid", x=130)])
        self.assertEqual([item.kind for item in result], ["position"])
        self.assertAlmostEqual(result[0].center_error or 0, 30.0)

    def test_active_and_synthetic_mismatches_are_reported(self) -> None:
        result = compare_snapshots(
            [detection("42", active=False)],
            [detection("uuid", active=True, synthetic=True)],
            report_synthetic=True,
        )
        self.assertEqual([item.kind for item in result], ["active", "synthetic"])

    def test_synthetic_state_is_ignored_by_default(self) -> None:
        result = compare_snapshots(
            [detection("42")], [detection("uuid", synthetic=True)]
        )
        self.assertEqual(result, [])

    def test_iou_only_edge_difference_is_ignored(self) -> None:
        result = compare_snapshots(
            [detection("42", x=994, w=4, h=267)],
            [detection("uuid", x=995, w=4, h=268)],
        )
        self.assertEqual(result, [])

    def test_edge_difference_with_large_center_error_is_reported(self) -> None:
        result = compare_snapshots(
            [detection("42", x=0, w=10)],
            [detection("uuid", x=30, w=10)],
        )
        self.assertEqual([item.kind for item in result], ["only-dataq", "only-ddh"])

    def test_overlapping_different_classes_are_class_mismatch(self) -> None:
        result = compare_snapshots(
            [detection("42", **{"class": "Car"})],
            [detection("uuid", **{"class": "Truck"})],
        )
        self.assertEqual([item.kind for item in result], ["class"])

    def test_unmatched_detections_report_the_present_side(self) -> None:
        dataq_only = compare_snapshots([detection("42")], [])
        ddh_only = compare_snapshots([], [detection("uuid")])
        self.assertEqual([item.kind for item in dataq_only], ["only-dataq"])
        self.assertEqual([item.kind for item in ddh_only], ["only-ddh"])


class PayloadTests(unittest.TestCase):
    def test_payload_is_normalized(self) -> None:
        payload, detections = parse_detection_payload(
            b'{"list":[{"id":"42","class":"Car"}],"serial":"front"}'
        )
        self.assertEqual(payload["serial"], "front")
        self.assertEqual(detections[0]["id"], "42")

    def test_malformed_payload_is_rejected(self) -> None:
        for payload in (b"not-json", b"[]", b'{"list":{}}', b'{"list":[1]}'):
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    parse_detection_payload(payload)

    def test_inactive_and_invalid_detections_are_not_scene_objects(self) -> None:
        detections = [
            detection("active"),
            detection("inactive", active=False),
            detection("invalid", w=0),
        ]
        self.assertEqual(
            [item["id"] for item in comparable_detections(detections)], ["active"]
        )
        self.assertEqual(
            [item["id"] for item in comparable_detections(detections, include_inactive=True)],
            ["active", "inactive"],
        )


class DifferenceTrackerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.differences = compare_snapshots(
            [detection("42")], [detection("uuid", x=130)]
        )

    def test_difference_is_held_then_rate_limited(self) -> None:
        tracker = DifferenceTracker(hold_seconds=0.75, repeat_seconds=5.0)
        self.assertEqual(tracker.update(self.differences, 0.0, (0.0, 0.0)), [])
        self.assertEqual(tracker.update(self.differences, 0.74, (0.1, 0.0)), [])
        self.assertEqual(len(tracker.update(self.differences, 0.75, (0.1, 0.2))), 1)
        self.assertEqual(tracker.update(self.differences, 5.74, (0.1, 0.2)), [])
        self.assertEqual(len(tracker.update(self.differences, 5.75, (5.7, 5.7))), 1)

    def test_elapsed_time_without_new_snapshots_does_not_report(self) -> None:
        tracker = DifferenceTracker(hold_seconds=0.75, repeat_seconds=5.0)
        observation = (0.0, 0.0)
        self.assertEqual(tracker.update(self.differences, 0.0, observation), [])
        self.assertEqual(tracker.update(self.differences, 1.0, observation), [])

    def test_resolved_difference_must_be_held_again(self) -> None:
        tracker = DifferenceTracker(hold_seconds=0.75, repeat_seconds=5.0)
        tracker.update(self.differences, 0.0, (0.0, 0.0))
        tracker.update([], 0.5, (0.5, 0.0))
        self.assertEqual(tracker.update(self.differences, 1.0, (1.0, 0.5)), [])
        self.assertEqual(len(tracker.update(self.differences, 1.75, (1.75, 1.0))), 1)


class SnapshotFreshnessTests(unittest.TestCase):
    def test_both_snapshots_must_be_fresh(self) -> None:
        snapshots = {
            "dataq": Snapshot([detection("42")], 10.0),
            "ddh": Snapshot([detection("uuid")], 10.2),
        }
        self.assertIsNotNone(fresh_snapshot_pair(snapshots, 10.7, 0.75))
        self.assertIsNone(fresh_snapshot_pair(snapshots, 10.96, 0.75))

    def test_missing_snapshot_is_not_object_absence(self) -> None:
        snapshots = {"dataq": Snapshot([detection("42")], 10.0)}
        self.assertIsNone(fresh_snapshot_pair(snapshots, 10.1, 0.75))


class CommandLineDefaultsTests(unittest.TestCase):
    def test_defaults_suppress_transient_and_lifecycle_noise(self) -> None:
        arguments = build_parser().parse_args([])
        self.assertEqual(arguments.hold_ms, 1000.0)
        self.assertEqual(arguments.minimum_observations, 2)
        self.assertEqual(arguments.stale_seconds, 0.0)
        self.assertFalse(arguments.include_inactive)
        self.assertFalse(arguments.include_synthetic_differences)


class CaptureWriterTests(unittest.TestCase):
    def test_capture_is_accepted_by_offline_comparator(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            writer = CaptureWriter(Path(temporary_directory))
            writer.write(
                "dataq",
                "dataq/detections/front",
                {"list": [detection("42")]},
                "2026-03-10T12:00:00.000Z",
            )
            writer.close()

            records, rejected = load_records(writer.paths["dataq"])
            self.assertEqual(rejected, 0)
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0].detections[0]["id"], "42")
            self.assertIsNotNone(records[0].timestamp_ms)


if __name__ == "__main__":
    unittest.main()