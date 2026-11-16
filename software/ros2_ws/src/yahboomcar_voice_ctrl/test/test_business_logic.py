import cv2
import numpy as np
import pytest

from yahboomcar_voice_ctrl.voice_business import classify_roi_color, resolve_route


# ==============================================================================
# resolve_route Unit Tests
# ==============================================================================

def test_resolve_route_initial_phase_valid():
    """Verify standard mapping preserved from initial stage."""
    assert resolve_route(19, stage='initial') == 'red'
    assert resolve_route(20, stage='initial') == 'green'
    assert resolve_route(21, stage='initial') == 'blue'
    assert resolve_route(32, stage='initial') == 'yellow'
    assert resolve_route(33, stage='initial') == 'origin'


def test_resolve_route_default_stage():
    """Default stage parameter should be 'initial'."""
    assert resolve_route(19) == 'red'
    assert resolve_route(20) == 'green'
    assert resolve_route(21) == 'blue'
    assert resolve_route(32) == 'yellow'
    assert resolve_route(33) == 'origin'


def test_resolve_route_after_pick_phase_inconsistency():
    """Verify deliberate legacy source inconsistency preserved in after_pick stage."""
    assert resolve_route(19, stage='after_pick') == 'red'
    assert resolve_route(20, stage='after_pick') == 'yellow'
    assert resolve_route(21, stage='after_pick') == 'green'
    assert resolve_route(32, stage='after_pick') == 'blue'
    assert resolve_route(33, stage='after_pick') == 'origin'


def test_resolve_route_invalid_codes_and_types():
    """Ensure invalid commands, unknown codes, bools, None, and NaN return None."""
    # Unknown integer codes
    assert resolve_route(999, stage='initial') is None
    assert resolve_route(7, stage='initial') is None
    assert resolve_route(0, stage='initial') is None
    assert resolve_route(-1, stage='initial') is None
    assert resolve_route(999, stage='after_pick') is None
    assert resolve_route(7, stage='after_pick') is None

    # None and NaN
    assert resolve_route(None, stage='initial') is None
    assert resolve_route(float('nan'), stage='initial') is None
    assert resolve_route(None, stage='after_pick') is None
    assert resolve_route(float('nan'), stage='after_pick') is None

    # Booleans (True == 1, False == 0 in Python) must not resolve
    assert resolve_route(True, stage='initial') is None
    assert resolve_route(False, stage='initial') is None
    assert resolve_route(True, stage='after_pick') is None
    assert resolve_route(False, stage='after_pick') is None

    # Invalid stage strings
    assert resolve_route(19, stage='unknown_stage') is None
    assert resolve_route(19, stage='') is None
    assert resolve_route(19, stage=None) is None


# ==============================================================================
# Helper for creating uniform 640x480 BGR test images from HSV values
# ==============================================================================

def make_bgr_image(h: int, s: int, v: int, width: int = 640, height: int = 480) -> np.ndarray:
    """Create a 3-channel uint8 BGR image converted from a specified HSV color."""
    hsv_pixel = np.uint8([[[h, s, v]]])
    bgr_pixel = cv2.cvtColor(hsv_pixel, cv2.COLOR_HSV2BGR)[0, 0]
    img = np.empty((height, width, 3), dtype=np.uint8)
    img[:] = bgr_pixel
    return img


# ==============================================================================
# classify_roi_color Unit Tests
# ==============================================================================

@pytest.mark.parametrize(
    "h, s, v, expected_color",
    [
        # Red lower range: [0, 10]
        (0, 150, 150, 'red'),
        (5, 150, 150, 'red'),
        (10, 150, 150, 'red'),
        # Red upper range: [156, 180]
        (156, 150, 150, 'red'),
        (168, 150, 150, 'red'),
        (180, 150, 150, 'red'),
        # Yellow range: [23, 56]
        (23, 150, 150, 'yellow'),
        (40, 150, 150, 'yellow'),
        (56, 150, 150, 'yellow'),
        # Green range: [35, 85] (note: when uniform in [35, 56], yellow matches first by if-elif order)
        (57, 150, 150, 'green'),
        (70, 150, 150, 'green'),
        (85, 150, 150, 'green'),
        # Blue range: [100, 124]
        (100, 150, 150, 'blue'),
        (112, 150, 150, 'blue'),
        (124, 150, 150, 'blue'),
    ],
)
def test_classify_roi_color_valid_uniform_ranges(h, s, v, expected_color):
    """Test full ROI uniform coverage for all valid color ranges."""
    img = make_bgr_image(h, s, v)
    result = classify_roi_color(img)
    assert result == expected_color


def test_classify_roi_color_non_mutating():
    """Verify input image array is not modified by classify_roi_color."""
    img = make_bgr_image(5, 150, 150)
    img_copy = img.copy()
    result = classify_roi_color(img)
    assert result == 'red'
    assert np.array_equal(img, img_copy)


def test_classify_roi_color_min_saturation_value_filters():
    """Reject pixels failing the min_s or min_v thresholds."""
    # S below min_s=50
    img_low_s = make_bgr_image(5, 40, 150)
    assert classify_roi_color(img_low_s, min_s=50, min_v=50) is None

    # V below min_v=50 (dark)
    img_low_v = make_bgr_image(5, 150, 40)
    assert classify_roi_color(img_low_v, min_s=50, min_v=50) is None

    # Both low
    img_dark_gray = make_bgr_image(5, 10, 10)
    assert classify_roi_color(img_dark_gray, min_s=50, min_v=50) is None

    # Custom threshold acceptance
    assert classify_roi_color(img_low_s, min_s=30, min_v=50) == 'red'


def test_classify_roi_color_out_of_bounds_and_inverted_roi():
    """Verify graceful handling of invalid ROI definitions."""
    img = make_bgr_image(5, 150, 150)

    # Inverted coordinates: x1 >= x2 or y1 >= y2
    assert classify_roi_color(img, roi=(360, 180, 280, 260)) is None
    assert classify_roi_color(img, roi=(280, 260, 360, 180)) is None
    assert classify_roi_color(img, roi=(300, 200, 300, 250)) is None

    # Completely out of bounds coordinates
    assert classify_roi_color(img, roi=(-50, 180, 100, 260)) is None
    assert classify_roi_color(img, roi=(280, -20, 360, 100)) is None
    assert classify_roi_color(img, roi=(500, 180, 700, 260)) is None
    assert classify_roi_color(img, roi=(280, 400, 360, 500)) is None


def test_classify_roi_color_invalid_image_inputs():
    """Ensure invalid image shapes, channels, empty arrays, or non-uint8 return None."""
    # None image
    assert classify_roi_color(None) is None

    # Empty image
    empty_img = np.array([], dtype=np.uint8)
    assert classify_roi_color(empty_img) is None

    # Single-channel grayscale image (640x480)
    gray_img = np.full((480, 640), 128, dtype=np.uint8)
    assert classify_roi_color(gray_img) is None

    # 4-channel image (e.g. BGRA)
    bgra_img = np.full((480, 640, 4), 128, dtype=np.uint8)
    assert classify_roi_color(bgra_img) is None

    # Non-uint8 dtype
    float_img = np.full((480, 640, 3), 0.5, dtype=np.float32)
    assert classify_roi_color(float_img) is None


def test_classify_roi_color_non_uniform_roi():
    """Ensure mixed colors spanning multiple ranges return None."""
    img = make_bgr_image(5, 150, 150)  # Red background
    # Paint half of default ROI (280..360, 180..260) with Blue (H=112)
    hsv_blue = np.uint8([[[112, 150, 150]]])
    bgr_blue = cv2.cvtColor(hsv_blue, cv2.COLOR_HSV2BGR)[0, 0]
    img[180:260, 320:360] = bgr_blue

    # H values will range from 5 to 112, which does not fit in any single color window
    assert classify_roi_color(img) is None
    assert classify_roi_color(img) is None
