import unittest
from pathlib import Path

from simulator.ros.topic_contract import FRAMES, TOPICS


class ContractTests(unittest.TestCase):
    def test_ground_truth_does_not_claim_slam_odom(self):
        self.assertNotEqual(TOPICS["ground_truth_pose"], TOPICS["estimated_odom"])
        self.assertEqual(FRAMES["map"], "map")
        self.assertEqual(FRAMES["odom"], "odom")

    def test_required_operator_files_exist(self):
        root = Path(__file__).resolve().parents[1]
        for path in ("README.md", "IMPLEMENTATION_JOURNAL.md", "GATE_HANDOFF.md", "TESTING_GUIDE.md"):
            self.assertTrue((root / path).exists(), path)

    def test_browser_uses_separate_lidar_and_map_streams(self):
        root = Path(__file__).resolve().parents[1]
        app_js = (root / "dashboard/web/app.js").read_text(encoding="utf-8")
        self.assertIn("connect('lidar', lidarViewer)", app_js)
        self.assertIn("connect('map', mapViewer)", app_js)
        self.assertIn("/ws/${streamName}", app_js)
