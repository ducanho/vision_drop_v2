import unittest

from vision.tracker import BoardTracker


def board(x, confidence=.9):
    return dict(bbox=(x, 0, x + 100, 100), u=x + 50, v=50,
                name="van", confidence=confidence)


class BoardTrackerTests(unittest.TestCase):
    def test_arrival_order_survives_confidence_and_detection_order_changes(self):
        tracker = BoardTracker()
        self.assertEqual(tracker.update([board(500)], 0)[0]["id"], 1)
        items = tracker.update([board(0, .99), board(505, .5)], 1)
        self.assertEqual([(d["id"], d["u"]) for d in items], [(1, 555), (2, 50)])
        items = tracker.update([board(510, .99), board(5, .4)], 2)
        self.assertEqual([(d["id"], d["u"]) for d in items], [(1, 560), (2, 55)])

    def test_return_before_ten_seconds_keeps_id_and_resets_absence_timer(self):
        tracker = BoardTracker()
        tracker.update([board(0)], 0)
        self.assertEqual(tracker.update([], 5), [])
        self.assertEqual(tracker.update([board(10)], 9.9)[0]["id"], 1)
        tracker.update([], 15)
        self.assertIn(1, tracker.tracks)
        tracker.update([], 19.9)
        self.assertNotIn(1, tracker.tracks)

    def test_missing_id_reserved_then_reused_without_renumbering_other_boards(self):
        tracker = BoardTracker()
        tracker.update([board(0), board(500)], 0)
        items = tracker.update([board(500), board(1000)], 9.9)
        self.assertEqual([d["id"] for d in items], [2, 3])
        items = tracker.update([board(1500), board(500), board(1000)], 10)
        self.assertEqual([(d["id"], d["u"]) for d in items], [(1, 1550), (2, 550), (3, 1050)])

    def test_simultaneous_arrivals_sorted_by_position_and_ids_are_unique(self):
        tracker = BoardTracker()
        items = tracker.update([board(800), board(0), board(400)], 0)
        self.assertEqual([(d["id"], d["u"]) for d in items], [(1, 50), (2, 450), (3, 850)])
        items = tracker.update([board(410), board(810), board(10)], .1)
        self.assertEqual(len({d["id"] for d in items}), 3)

    def test_expired_tracks_do_not_produce_stale_detections(self):
        tracker = BoardTracker()
        tracker.update([board(0)], 0)
        self.assertEqual(tracker.update([], 10), [])
        self.assertEqual(tracker.tracks, {})
        self.assertEqual(tracker.update([board(500)], 11)[0]["id"], 1)


if __name__ == "__main__":
    unittest.main()
