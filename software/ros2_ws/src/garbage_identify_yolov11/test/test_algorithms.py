# Copyright (c) 2021-2024 YAHBOOM.
# Licensed under the Apache License, Version 2.0.
from garbage_identify_yolov11.garbage_library import (
    GARBAGE_CATEGORY,
    CATEGORY_TO_COLOR_POINT,
    classify_label,
    category_point,
    normalize_label,
    validate_sequence,
    GarbageTransport,
    GRIP_TARGET_JOINTS,
    GRIP_DOWN_JOINTS,
    GRIP_RESTORE_JOINTS,
    GRIP_CLOSE_ANGLE_TARGET,
    GRIP_OPEN_ANGLE,
    GRIP_TARGET_RUN_TIME,
    GRIP_DOWN_RUN_TIME,
    GRIP_RESTORE_RUN_TIME,
)


ALL_SEQUENCE_LABELS = [
    'newspaper', 'zip-top_can', 'zip_top_can', 'book', 'old_school_bag',
    'syringe', 'expired_cosmetics', 'used_batteries', 'used_batterise',
    'expired_tablets',
    'fish_bone', 'egg_shell', 'apple_core', 'watermelon_rind',
    'toilet_paper', 'peach_pit', 'cigarette_butts', 'disposable_chopsticks',
]


def test_label_mapping_categories():
    assert classify_label('newspaper') == 'recyclable_waste'
    assert classify_label('book') == 'recyclable_waste'
    assert classify_label('syringe') == 'toxic_waste'
    assert classify_label('fish_bone') == 'wet_waste'
    assert classify_label('toilet_paper') == 'dry_waste'


def test_label_mapping_aliases():
    assert classify_label('zip-top_can') == 'recyclable_waste'
    assert classify_label('zip_top_can') == 'recyclable_waste'
    assert classify_label('used_batteries') == 'toxic_waste'
    assert classify_label('used_batterise') == 'toxic_waste'


def test_unknown_label_returns_none():
    assert classify_label('battery') is None
    assert classify_label('') is None
    assert classify_label('    ') is None


def test_case_and_whitespace_insensitive():
    assert normalize_label('  Newspaper ') == 'newspaper'
    assert classify_label('  Newspaper ') == 'recyclable_waste'
    assert classify_label('FISH_BONE') == 'wet_waste'


def test_all_sequence_labels_have_category_and_point():
    for label in ALL_SEQUENCE_LABELS:
        cat = classify_label(label)
        assert cat in CATEGORY_TO_COLOR_POINT, label
        point = category_point(cat)
        assert validate_sequence(point), (label, point)


def test_sequence_points_valid():
    assert validate_sequence(1)
    assert validate_sequence(2)
    assert validate_sequence(3)
    assert validate_sequence(4)
    assert not validate_sequence(0)
    assert not validate_sequence(5)
    assert not validate_sequence(-1)


def test_original_arm_poses_preserved():
    assert GRIP_TARGET_JOINTS == [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
    assert GRIP_DOWN_JOINTS == [90.0, 2.0, 60.0, 40.0, 90.0, 140.0]
    assert GRIP_RESTORE_JOINTS == [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
    assert GRIP_CLOSE_ANGLE_TARGET == 149
    assert GRIP_OPEN_ANGLE == 30
    assert GRIP_TARGET_RUN_TIME == 1000
    assert GRIP_DOWN_RUN_TIME == 1000
    assert GRIP_RESTORE_RUN_TIME == 1000


def test_transport_state_machine_step_order():
    t = GarbageTransport()
    t.begin(1)
    kinds = []
    while True:
        step = t.step()
        if step is None:
            break
        kinds.append(step.kind)
    assert kinds[0] == 'arm_joints'
    assert kinds[1] == 'arm_single'
    assert 'nav' in kinds
    assert t.model in (GarbageTransport.TRANSPORT,
                       GarbageTransport.GRIP_DOWN,
                       GarbageTransport.COME_BACK)


def test_transport_reset_cycle():
    t = GarbageTransport()
    t.begin(2)
    t.step()
    t.Reset()
    assert t.model == GarbageTransport.INITIAL
    assert t.point == 0
