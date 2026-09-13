import unittest

import numpy as np

from evaluation.rgb_video_recorder import _decode_image


class _Image:
    def __init__(self, encoding, width, height, step, data):
        self.encoding = encoding
        self.width = width
        self.height = height
        self.step = step
        self.data = data


class RgbVideoRecorderTests(unittest.TestCase):
    def test_rgb_decode_honors_row_stride_and_converts_to_bgr(self):
        # Two RGB pixels plus two bytes of per-row padding.
        rows = np.array([[10, 20, 30, 40, 50, 60, 0, 0]], dtype=np.uint8)
        image = _Image("rgb8", 2, 1, 8, rows.tobytes())
        decoded = _decode_image(image)
        self.assertEqual(decoded.shape, (1, 2, 3))
        self.assertEqual(decoded.tolist(), [[[30, 20, 10], [60, 50, 40]]])

    def test_unsupported_encoding_is_rejected(self):
        image = _Image("16UC1", 1, 1, 2, b"\x00\x00")
        self.assertIsNone(_decode_image(image))
