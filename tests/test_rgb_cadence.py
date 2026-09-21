import unittest

from simulator.capture.rgb_cadence import analyze_rgb_cadence, verify_common_rgb_window


class RgbCadenceTests(unittest.TestCase):
    FPS = 30.0
    TARGET = 4.0
    START_ALLOWANCE = 0.1

    @classmethod
    def full_stamps(cls):
        return [index / cls.FPS for index in range(121)]

    def verify(self, recorder, bag=None, *, allowance=None):
        return verify_common_rgb_window(
            recorder,
            recorder if bag is None else bag,
            self.FPS,
            self.TARGET,
            expected_start_stamp_s=0.0,
            max_startup_delay_s=self.START_ALLOWANCE if allowance is None else allowance,
        )

    def test_rejects_one_final_stamp(self):
        result = self.verify([self.TARGET])
        self.assertEqual(result["status"], "incomplete")
        self.assertFalse(result["recorder"]["startup_coverage_ok"])
        self.assertGreater(result["recorder"]["startup_missing_intervals"], 0)

    def test_rejects_only_final_four_stamps(self):
        result = self.verify([self.TARGET - index / self.FPS for index in range(3, -1, -1)])
        self.assertEqual(result["status"], "incomplete")
        self.assertFalse(result["common_cadence"]["startup_coverage_ok"])

    def test_rejects_contiguous_long_prefix_loss(self):
        result = self.verify([index / self.FPS for index in range(60, 121)])
        self.assertEqual(result["status"], "incomplete")
        self.assertGreaterEqual(result["recorder"]["startup_delay_s"], 2.0)

    def test_accepts_documented_startup_allowance(self):
        stamps = [index / self.FPS for index in range(3, 121)]
        result = self.verify(stamps)
        self.assertEqual(result["status"], "complete")
        self.assertTrue(result["recorder"]["startup_coverage_ok"])
        self.assertAlmostEqual(result["recorder"]["startup_delay_s"], 0.1)

    def test_accepts_full_interval_set(self):
        result = self.verify(self.full_stamps())
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["common_frame_count"], 121)
        self.assertEqual(result["recorder"]["missing_intervals"], 0)

    def test_common_behavior_rejects_a_stream_outside_startup_bound(self):
        recorder = self.full_stamps()
        bag = [index / self.FPS for index in range(4, 121)]
        result = self.verify(recorder, bag)
        self.assertEqual(result["status"], "incomplete")
        self.assertTrue(result["recorder"]["startup_coverage_ok"])
        self.assertFalse(result["bag"]["startup_coverage_ok"])

    def test_common_behavior_accepts_later_start_within_bound(self):
        recorder = self.full_stamps()
        bag = [index / self.FPS for index in range(3, 121)]
        result = self.verify(recorder, bag)
        self.assertEqual(result["status"], "complete")
        self.assertAlmostEqual(result["common_start_stamp_s"], 0.1)
        self.assertEqual(result["common_frame_count"], len(bag))

    def test_configured_allowance_is_enforced(self):
        stamps = [index / self.FPS for index in range(3, 121)]
        self.assertEqual(self.verify(stamps, allowance=0.1)["status"], "complete")
        self.assertEqual(self.verify(stamps, allowance=2.0 / self.FPS)["status"], "incomplete")

    def test_reports_a_missing_30_hz_interval(self):
        result = analyze_rgb_cadence(
            [0.1, 0.133333333, 0.2],
            self.FPS,
            target_stamp_s=0.2,
            expected_start_stamp_s=0.0,
            max_startup_delay_s=0.1,
        )
        self.assertFalse(result["contiguous"])
        self.assertEqual(result["missing_intervals"], 1)
        self.assertAlmostEqual(result["max_gap_s"], 2.0 / self.FPS)

    def test_reports_missing_tail_beyond_one_interval_allowance(self):
        result = analyze_rgb_cadence(
            [0.1, 0.133333333],
            self.FPS,
            target_stamp_s=0.233333333,
            expected_start_stamp_s=0.0,
            max_startup_delay_s=0.1,
        )
        self.assertFalse(result["contiguous"])
        self.assertEqual(result["trailing_missing_intervals"], 2)

    def test_common_window_rejects_loss_in_either_stream(self):
        recorder = self.full_stamps()
        bag = recorder[:2] + recorder[3:]
        result = self.verify(recorder, bag)
        self.assertEqual(result["status"], "incomplete")
        self.assertFalse(result["bag"]["contiguous"])
        self.assertFalse(result["common_stamps_match"])


if __name__ == "__main__":
    unittest.main()
