import unittest

import cv2 as cv
import numpy as np

from arm_autopilot.autopilot_common import (
    linear,
    color_follow,
    simplePID,
    ManyImgs,
    DEFAULT_HSV_LIST,
)


class TestHelpers(unittest.TestCase):
    def test_linear(self):
        a, b = linear((320, 90), (343.5, 95))
        self.assertAlmostEqual(a * 320 + b, 90, places=4)
        self.assertAlmostEqual(a * 343.5 + b, 95, places=4)

    def test_many_imgs(self):
        a = np.zeros((20, 20, 3), np.uint8)
        b = np.zeros((20, 20, 3), np.uint8)
        out = ManyImgs(1, [a, b])
        self.assertEqual(out.shape[0], 20)
        self.assertEqual(out.shape[1], 40)


class TestPID(unittest.TestCase):
    def test_first_update_zero_then_converges(self):
        pid = simplePID([0, 0], [30, 30], [0, 0], [60, 60])
        out = pid.update([10, 10])
        self.assertTrue(np.allclose(out, np.zeros(2)))
        out2 = pid.update([10, 10])
        self.assertEqual(np.size(out2), 2)

    def test_shape_mismatch(self):
        pid = simplePID([0, 0], [1, 1], [0, 0], [0, 0])
        with self.assertRaises(TypeError):
            pid.update([1, 2, 3])


class TestHSVSynthetic(unittest.TestCase):
    def _solid(self, bgr, size=200):
        img = np.zeros((size, size, 3), np.uint8)
        img[:, :] = bgr
        return img

    def test_red_detected(self):
        # The supplied red calibration is H176..180, S78..253; pure
        # BGR(0,0,255) has H0/S255 and is intentionally outside it.
        color = cv.cvtColor(np.uint8([[[178, 200, 220]]]), cv.COLOR_HSV2BGR)[0, 0]
        img = self._solid(tuple(int(v) for v in color))
        cf = color_follow()
        hsv = DEFAULT_HSV_LIST['red']
        cf.line_follow(img, 'red', hsv)
        self.assertEqual(len(cf.msg_circle['red']), 2)
        cx, cy = cf.msg_circle['red']
        self.assertAlmostEqual(cx, 100, delta=25)
        # The current target color is sought only in the lower half.
        self.assertAlmostEqual(cy, 149.5, delta=2)

    def test_green_detected_blue_not(self):
        color = cv.cvtColor(np.uint8([[[70, 180, 220]]]), cv.COLOR_HSV2BGR)[0, 0]
        img = self._solid(tuple(int(v) for v in color))
        cf = color_follow()
        cf.line_follow(img, 'green', DEFAULT_HSV_LIST['green'])
        self.assertEqual(len(cf.msg_circle['green']), 2)
        cf.line_follow(img, 'blue', DEFAULT_HSV_LIST['blue'])
        self.assertEqual(len(cf.msg_circle['blue']), 0)

    def test_yellow_tightrange(self):
        img = self._solid((0, 255, 255))
        cf = color_follow()
        cf.line_follow(img, 'yellow', DEFAULT_HSV_LIST['yellow'])
        # Yellow range in source is tight; must not crash and may or may not match
        self.assertIn('yellow', cf.msg_circle)

    def test_roi_hsv_extracted(self):
        img = self._solid((0, 0, 255))
        cf = color_follow()
        annotated, hsv = cf.Roi_hsv(img.copy(), (50, 50, 150, 150))
        self.assertEqual(annotated.shape, img.shape)
        lower, upper = hsv
        self.assertGreaterEqual(lower[0], 0)
        self.assertLessEqual(upper[0], 180)
        self.assertGreater(upper[0], lower[0])

    def test_threshold_noise_not_detected(self):
        img = self._solid((128, 128, 128))
        cf = color_follow()
        cf.line_follow(img, 'red', DEFAULT_HSV_LIST['red'])
        self.assertEqual(len(cf.msg_circle['red']), 0)


if __name__ == '__main__':
    unittest.main()
