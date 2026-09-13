import unittest

import cv2
import numpy as np

from simulator.perception.rgb_tracking import BlobTracker, detect_product_blobs


class PerceptionTests(unittest.TestCase):
    def test_rgb_blob_detector_returns_measured_centers(self):
        image = np.zeros((120, 180, 3), dtype=np.uint8)
        cv2.rectangle(image, (20, 30), (54, 68), (0, 0, 220), -1)
        cv2.rectangle(image, (100, 20), (144, 60), (0, 210, 0), -1)
        detections = detect_product_blobs(image)
        self.assertEqual(len(detections), 2)
        centers = {tuple(round(value, 1) for value in item.center_px) for item in detections}
        self.assertIn((37.0, 49.0), centers)
        self.assertIn((122.0, 40.0), centers)

    def test_tracker_reuses_ids_for_small_motion(self):
        image_a = np.zeros((120, 180, 3), dtype=np.uint8)
        image_b = np.zeros((120, 180, 3), dtype=np.uint8)
        cv2.rectangle(image_a, (20, 30), (54, 68), (0, 0, 220), -1)
        cv2.rectangle(image_b, (24, 31), (58, 69), (0, 0, 220), -1)
        tracker = BlobTracker()
        first = tracker.update(0, detect_product_blobs(image_a))
        second = tracker.update(1, detect_product_blobs(image_b))
        self.assertEqual(len(first), 1)
        self.assertEqual(len(second), 1)
        self.assertEqual(first[0]["track_id"], second[0]["track_id"])
