"""Pure scan-thinning helpers used by the scan_dilute node.

The ROS 1 node copied the full intensity array, left angle_max unchanged and
did not account for the larger time between retained samples.  This module
implements the corrected geometry while preserving the ROS 2 LaserScan
metadata and the original "take every Nth sample" behaviour.
"""

import math

from sensor_msgs.msg import LaserScan


def dilute_scan(msg, multiple, preserve_header_stamp=True, now_stamp=None,
                scale_time_increment=True):
    """Return (LaserScan, warning_or_None).

    Raises ValueError for unusable input metadata.  ``msg`` itself is never
    modified.
    """
    if not isinstance(msg, LaserScan):
        raise ValueError('input is not a sensor_msgs/LaserScan')

    if isinstance(multiple, bool):
        raise ValueError('multiple must be an integer >= 1')
    try:
        parsed_multiple = int(multiple)
        if not isinstance(multiple, str) and parsed_multiple != multiple:
            raise ValueError('multiple must be an integer')
        multiple = parsed_multiple
    except (TypeError, ValueError, OverflowError):
        raise ValueError('multiple must be an integer')

    if isinstance(multiple, bool) or multiple < 1:
        raise ValueError('multiple must be an integer >= 1')

    angle_min = float(msg.angle_min)
    angle_increment = float(msg.angle_increment)
    if not math.isfinite(angle_min):
        raise ValueError('angle_min is not finite')
    if not math.isfinite(angle_increment) or angle_increment <= 0.0:
        raise ValueError('angle_increment must be finite and > 0')

    warning = None
    ranges = list(msg.ranges)
    selected_ranges = ranges[::multiple]

    out = LaserScan()
    out.header.frame_id = msg.header.frame_id
    if preserve_header_stamp:
        out.header.stamp = msg.header.stamp
    elif now_stamp is not None:
        out.header.stamp = now_stamp
    else:
        out.header.stamp = msg.header.stamp
        warning = 'preserve_header_stamp=false but no clock stamp supplied'

    out.angle_min = angle_min
    out.angle_increment = angle_increment * multiple
    if selected_ranges:
        # Recompute from the retained samples rather than copying a stale
        # angle_max from the undecimated input.
        out.angle_max = (out.angle_min +
                         (len(selected_ranges) - 1) * out.angle_increment)
    else:
        out.angle_max = out.angle_min

    out.time_increment = float(msg.time_increment)
    if scale_time_increment:
        out.time_increment *= multiple
    out.scan_time = float(msg.scan_time)
    out.range_min = float(msg.range_min)
    out.range_max = float(msg.range_max)
    out.ranges = selected_ranges

    intensities = list(msg.intensities)
    if intensities:
        if len(intensities) == len(ranges):
            out.intensities = intensities[::multiple]
        else:
            out.intensities = []
            warning = ('intensity length %d does not match ranges length %d; '
                       'dropping intensities to avoid misalignment'
                       % (len(intensities), len(ranges)))
    else:
        out.intensities = []

    if not selected_ranges and warning is None:
        warning = 'input scan has no ranges'

    return out, warning
