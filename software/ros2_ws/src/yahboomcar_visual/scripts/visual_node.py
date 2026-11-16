#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unified dispatcher for yahboomcar_visual Python entry points.

Core subtask entries are wired here. Future algorithm entries
(ar_tools.main_ar/main_calibration, target_detection.main,
qr_tools.main_create/main_parse) are supported names but their modules
will be provided separately; missing modules fail clearly at runtime.
"""

import importlib
import sys


ENTRY_MAP = {
    "rgb": ("yahboomcar_visual.image_nodes", "main_rgb"),
    "depth": ("yahboomcar_visual.image_nodes", "main_depth"),
    "flip": ("yahboomcar_visual.image_nodes", "main_flip"),
    "pub_image": ("yahboomcar_visual.image_nodes", "main_pubimage"),
    "color_point": ("yahboomcar_visual.image_nodes", "main_color_point"),
    "laser_to_image": ("yahboomcar_visual.laser_to_image", "main"),
    "sub_point": ("yahboomcar_visual.point_tools", "main_sub_point"),
    # Future algorithms (separately implemented):
    "ar": ("yahboomcar_visual.ar_tools", "main_ar"),
    "calibration": ("yahboomcar_visual.ar_tools", "main_calibration"),
    "target_detection": ("yahboomcar_visual.target_detection", "main"),
    "qr_create": ("yahboomcar_visual.qr_tools", "main_create"),
    "qr_parse": ("yahboomcar_visual.qr_tools", "main_parse"),
}

USAGE = "usage: visual_node.py {%s} [args...]" % "|".join(sorted(ENTRY_MAP))


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(USAGE)
        return 0 if argv else 1
    name = argv[0]
    if name not in ENTRY_MAP:
        print(USAGE, file=sys.stderr)
        return 2
    mod_name, attr = ENTRY_MAP[name]
    try:
        module = importlib.import_module(mod_name)
    except ImportError as exc:
        print("DISABLED/PENDING module %s: %s" % (mod_name, exc), file=sys.stderr)
        return 3
    fn = getattr(module, attr, None)
    if fn is None:
        print("Missing entry point %s.%s" % (mod_name, attr), file=sys.stderr)
        return 4
    return int(fn(argv[1:]) or 0)


if __name__ == "__main__":
    sys.exit(main())
