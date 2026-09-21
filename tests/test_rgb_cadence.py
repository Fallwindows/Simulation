import unittest

from simulator.capture.rgb_cadence import analyze_rgb_cadence, verify_common_rgb_window


class RgbCadenceTests(unittest.TestCase):
    def test_reports_a_missing_30_hz_interval(self):
        result = analyze_rgb_cadence([0.1, 0.133333333, 0.2], 30.0, target_stamp_s=0.2)
        self.assertFalse(result["contiguous"])
        self.assertEqual(result["missing_intervals"], 1)
        self.assertAlmostEqual(result["max_gap_s"], 2.0 / 30.0)

    def test_reports_missing_tail_beyond_one_interval_allowance(self):
        result = analyze_rgb_cadence([0.1, 0.133333333], 30.0, target_stamp_s=0.233333333)
        self.assertFalse(result["contiguous"])
        self.assertEqual(result["trailing_missing_intervals"], 2)

    def test_common_window_declares_and_accepts_later_start(self):
        recorder = [0.1 + index / 30.0 for index in range(5)]
        bag = recorder[1:]
        result = verify_common_rgb_window(recorder, bag, 30.0, recorder[-1])
        self.assertEqual(result["status"], "complete")
        self.assertAlmostEqual(result["common_start_stamp_s"], bag[0])
        self.assertAlmostEqual(result["pre_common_start_s_not_evaluated"], bag[0])
        self.assertEqual(result["common_frame_count"], 4)

    def test_common_window_rejects_loss_in_either_stream(self):
        recorder = [0.1 + index / 30.0 for index in range(5)]
        bag = [recorder[0], recorder[1], recorder[3], recorder[4]]
        result = verify_common_rgb_window(recorder, bag, 30.0, recorder[-1])
        self.assertEqual(result["status"], "incomplete")
        self.assertFalse(result["bag"]["contiguous"])
        self.assertFalse(result["common_stamps_match"])


if __name__ == "__main__":
    unittest.main()
