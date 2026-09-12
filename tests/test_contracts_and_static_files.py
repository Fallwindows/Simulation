import unittest
import json
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
        self.assertIn("'/ws/' + streamName", app_js)
        self.assertIn("OrbitControls", app_js)

    def test_dashboard_serves_browser_bundle(self):
        root = Path(__file__).resolve().parents[1]
        app_py = (root / "dashboard/backend/app.py").read_text(encoding="utf-8")
        self.assertIn('app.mount("/static", StaticFiles(directory=WEB_ROOT)', app_py)
        self.assertIn('@app.get("/app.js")', app_py)
        self.assertIn("'/ws/' + streamName", (root / "dashboard/web/app.js").read_text(encoding="utf-8"))

    def test_mapping_launch_uses_static_tf_boundary(self):
        root = Path(__file__).resolve().parents[1]
        launch = (root / "ros2_ws/src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py").read_text(encoding="utf-8")
        self.assertIn('frames["sensor_rig"]', launch)
        self.assertIn('"publish_tf": True', launch)
        self.assertNotIn('"frame_id": "lidar_link"', launch)
        self.assertIn('"mapping_params_path"', launch)
        self.assertIn('"publish_tf": True', launch)

    def test_estimator_tf_has_one_parent_and_truth_is_separate(self):
        root = Path(__file__).resolve().parents[1]
        contract = json.loads((root / "config/contracts.yaml").read_text(encoding="utf-8"))
        owners = contract["ownership"]
        self.assertEqual(owners["odom_to_sensor_rig"], "rtabmap_icp_odometry")
        self.assertNotIn("sim_world_to_rig", owners)
        self.assertEqual(contract["frames"]["truth_sensor_rig"], "truth_sensor_rig")
        runtime = (root / "simulator/runtime/isaac_sim_runner.py").read_text(encoding="utf-8")
        self.assertIn('message.child_frame_id = FRAMES["truth_sensor_rig"]', runtime)
        self.assertNotIn('message.child_frame_id = "sensor_rig"', runtime)

    def test_root_and_packaged_contracts_and_mapping_config_match(self):
        root = Path(__file__).resolve().parents[1]
        self.assertEqual(
            json.loads((root / "config/contracts.yaml").read_text(encoding="utf-8")),
            json.loads((root / "ros2_ws/src/grocery_sim_mapping/config/contracts.yaml").read_text(encoding="utf-8")),
        )
        self.assertEqual(
            json.loads((root / "config/mapping/rtabmap/params.yaml").read_text(encoding="utf-8")),
            json.loads((root / "ros2_ws/src/grocery_sim_mapping/config/params.yaml").read_text(encoding="utf-8")),
        )

    def test_lidar_only_baseline_and_strategy_are_explicit(self):
        root = Path(__file__).resolve().parents[1]
        params = json.loads((root / "config/mapping/rtabmap/params.yaml").read_text(encoding="utf-8"))
        self.assertTrue(params["subscribe_scan_cloud"])
        self.assertFalse(params["subscribe_rgb"])
        self.assertFalse(params["subscribe_depth"])
        self.assertTrue(params["subscribe_odom_info"])
        self.assertEqual(params["Reg/Strategy"], 1)

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

    def test_consolidated_launcher_validates_services_artifacts_and_realtime(self):
        root = Path(__file__).resolve().parents[1]
        baseline = (root / "scripts/run_baseline.ps1").read_text(encoding="utf-8")
        for text in ("Assert-Running", "metrics.json", "ground_truth.csv", "estimate.csv", "rtabmap.db", "map_point_count", "initial_se3", "-Realtime", "-Fast", "isaac_runtime_status.json"):
            self.assertIn(text, baseline)
        self.assertIn("-Gui", baseline)
        self.assertIn("collector exited with code", baseline)

    def test_run_sim_has_explicit_gui_headless_switches(self):
        root = Path(__file__).resolve().parents[1]
        sim = (root / "scripts/run_sim.ps1").read_text(encoding="utf-8")
        self.assertIn("[switch]$Gui", sim)
        self.assertIn("[switch]$Headless", sim)
        self.assertIn("Choose either -Gui or -Headless", sim)

    def test_windows_setup_does_not_enable_global_auto_export(self):
        root = Path(__file__).resolve().parents[1]
        setup = (root / "scripts/setup_rtabmap_windows.ps1").read_text(encoding="utf-8")
        self.assertNotIn("CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON", setup)
        self.assertIn("grocery_sim_mapping", setup)
        self.assertIn("Push-Location $workspace", setup)
        self.assertLess(setup.index("--packages-up-to"), setup.index("--cmake-args"))
        self.assertIn("link_directories\\(\\$\\{PCL_LIBRARY_DIRS\\}\\)", setup)
