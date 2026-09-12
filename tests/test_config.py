import unittest
from pathlib import Path

from simulator.config.loader import load_contracts, load_scenario


ROOT = Path(__file__).resolve().parents[1]


class ConfigTests(unittest.TestCase):
    def test_contracts_are_stable(self):
        contracts = load_contracts(ROOT / "config/contracts.yaml")
        self.assertEqual(contracts["topics"]["rgb_image"], "/sim/camera/rgb/image_raw")
        self.assertEqual(contracts["dashboard"]["port"], 8080)

    def test_scenario_composes_typed_configs(self):
        scenario = load_scenario(ROOT / "config/scenarios/baseline_straight.yaml")
        self.assertEqual(scenario.name, "baseline_straight")
        self.assertEqual(scenario.camera.width_px, 1280)
        self.assertGreater(scenario.environment.length_m, scenario.environment.bay_width_m)
        self.assertEqual(scenario.trajectory.name, "straight")

    def test_invalid_density_is_rejected(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text('{"name":"bad","length_m":1,"width_m":1,"shelf_height_m":1,"shelf_depth_m":1,"bay_width_m":1,"shelf_levels":1,"product_density":2,"seed":1}', encoding="utf-8")
            from simulator.config.loader import _aisle
            with self.assertRaises(ValueError):
                _aisle(__import__("json").loads(path.read_text(encoding="utf-8")))
