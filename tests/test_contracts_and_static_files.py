import json
from pathlib import Path
import subprocess
import unittest

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

    def test_runtime_uses_local_isaac_lidar_pose_and_continuous_ros_executor(self):
        root = Path(__file__).resolve().parents[1]
        runtime = (root / "simulator/runtime/isaac_sim_runner.py").read_text(encoding="utf-8")
        self.assertIn('orientations=[spec["orientation_wxyz_isaac"]]', runtime)
        self.assertIn("MultiThreadedExecutor", runtime)
        self.assertIn("command_rig_before_update_publish_truth_after_update", runtime)
        self.assertNotIn("frame / 60.0", runtime)

    def test_windows_setup_does_not_enable_global_auto_export(self):
        root = Path(__file__).resolve().parents[1]
        setup = (root / "scripts/setup_rtabmap_windows.ps1").read_text(encoding="utf-8")
        self.assertNotIn("CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON", setup)
        self.assertIn("grocery_sim_mapping", setup)
        self.assertIn("Push-Location $workspace", setup)
        self.assertLess(setup.index("--packages-up-to"), setup.index("--cmake-args"))
        self.assertIn("link_directories\\(\\$\\{PCL_LIBRARY_DIRS\\}\\)", setup)
        self.assertIn("pixi install", setup)
        self.assertIn("repoPackage", setup)
        self.assertIn("sourceSha", setup)

    def test_launcher_owns_ros_domain_and_requires_sim_time_freshness(self):
        root = Path(__file__).resolve().parents[1]
        baseline = (root / "scripts/run_baseline.ps1").read_text(encoding="utf-8")
        collector = (root / "evaluation/ros_collector.py").read_text(encoding="utf-8")
        self.assertIn('$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"', baseline)
        self.assertIn('$env:ROS_DOMAIN_ID = "0"', baseline)
        self.assertIn("simulation_time_complete", baseline)
        self.assertIn("/rtabmap/pause", baseline)
        self.assertIn("/rtabmap/backup", baseline)
        self.assertIn("evaluation.rgb_video_recorder", baseline)
        self.assertIn("rgb_camera.mp4", baseline)
        self.assertIn("demo/current_walking_aisle.mp4", baseline)
        self.assertIn("simulation_time_target_s", collector)
        self.assertIn("simulation_time_reached", collector)

    def test_capture_uses_configured_camera_info_without_recorder_subscription(self):
        root = Path(__file__).resolve().parents[1]
        capture = (root / "scripts/capture_simulation.ps1").read_text(encoding="utf-8")
        self.assertNotIn('"--camera-info-json"', capture)
        self.assertIn('$cameraInfo.provenance -ne "configured_intrinsics"', capture)
        self.assertIn('camera_info_provenance="configured_intrinsics"', capture)
        self.assertIn('"--end-clock-seconds",([string]$duration)', capture)
        self.assertIn('"--expected-rgb-fps",([string]$rgbFps)', capture)
        self.assertIn('"--max-rgb-startup-delay-seconds",([string]$rgbMaxStartupDelaySeconds)', capture)
        self.assertIn('source_tree_clean=$true', capture)
        self.assertIn('status --porcelain --untracked-files=no', capture)
        self.assertIn('transport_realtime_factor=$(if ($Realtime)', capture)
        self.assertIn('"--rgb-video",(Join-Path $captureDir "rgb_camera.mp4")', capture)
        self.assertIn("simulator.capture.rgb_cadence", capture)
        self.assertIn('Combined raw bag/RGB writer failed', capture)
        self.assertIn('$bagExit -ne 0', capture)
        self.assertIn("simulator.capture.finalize_manifest finalize", capture)
        self.assertIn("simulator.capture.finalize_manifest verify", capture)
        self.assertNotIn("Security.Cryptography.SHA256", capture)
        self.assertLess(capture.index("finalize_manifest verify"), capture.index('New-Item -ItemType File -Force -Path (Join-Path $captureDir "CAPTURE_COMPLETE")'))

    def test_redirected_process_status_distinguishes_success_failure_and_unavailable(self):
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run(
            ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(root / "tests/process_status_regression.ps1")],
            cwd=root,
            check=True,
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertIn("zero=0 nonzero=7 unavailable=rejected", result.stdout)
