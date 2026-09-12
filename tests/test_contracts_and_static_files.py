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

    def test_dashboard_serves_browser_bundle(self):
        root = Path(__file__).resolve().parents[1]
        app_py = (root / "dashboard/backend/app.py").read_text(encoding="utf-8")
        self.assertIn('app.mount("/static", StaticFiles(directory=WEB_ROOT)', app_py)
        self.assertIn('@app.get("/app.js")', app_py)
        self.assertIn("/ws/${streamName}", (root / "dashboard/web/app.js").read_text(encoding="utf-8"))

    def test_mapping_launch_uses_static_tf_boundary(self):
        root = Path(__file__).resolve().parents[1]
        launch = (root / "ros2_ws/src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py").read_text(encoding="utf-8")
        self.assertIn('frames["sensor_rig"]', launch)
        self.assertIn('"publish_tf": True', launch)
        self.assertNotIn('"frame_id": "lidar_link"', launch)

    def test_operator_scripts_use_real_stack_and_run_isolation(self):
        root = Path(__file__).resolve().parents[1]
        baseline = (root / "scripts/run_baseline.ps1").read_text(encoding="utf-8")
        mapping = (root / "scripts/run_mapping.ps1").read_text(encoding="utf-8")
        sim = (root / "scripts/run_sim.ps1").read_text(encoding="utf-8")
        self.assertIn("run_mapping.ps1", baseline)
        self.assertIn("run_sim.ps1", baseline)
        self.assertNotIn("simulator.runtime.sim_runner", baseline)
        self.assertIn("rtabmap.db", mapping)
        self.assertIn("$Frames = 0", sim)
