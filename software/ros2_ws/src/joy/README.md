# joy (ROS 2 Humble)

This package is the ROS 2 Humble port of the Yahboom-modified ROS 1 `joy`
package. The original package bundled the generic Linux `joy_node` driver and a
YAML-driven `joy_remap.py` node.

## Driver entry point

The generic Linux driver is **not** reimplemented here. Use the upstream
`joy_linux` package:

| ROS 1 | ROS 2 Humble |
| --- | --- |
| `joy/joy_node` | `joy_linux/joy_linux_node` |
| `joy/scripts/joy_remap.py` | `joy/joy_remap` (this package) |
| `launch/ps4joy.launch` | `launch/ps4joy.launch.py` |
| `config/ps4joy.yaml` | `config/ps4joy.yaml` |

The local package is intentionally still named `joy`. It therefore **must not**
depend on an ROS 2 package also named `joy`; it depends on `joy_linux` instead.

Install the upstream driver once:

```bash
sudo apt install ros-humble-joy-linux
```

## Usage

```bash
ros2 launch joy ps4joy.launch.py dev:=/dev/input/js0
```

Pipeline:

1. `joy_linux_node` publishes `joy_orig`.
2. `joy_remap` subscribes to `joy_orig` and publishes the remapped `joy`.

The original launch assumed `ds4drv` was already running. That external
precondition is unchanged and must still be satisfied on the target machine.

## Parameters (`joy_remap`)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `mappings.axes` | `["axes[0]"]` (overridden by YAML) | Output axis expressions |
| `mappings.buttons` | `["buttons[0]"]` (overridden by YAML) | Output button expressions |
| `queue_size` | `1` | Publisher/subscriber QoS depth |

Input and output topics remain relative (`joy_in`, `joy_out`) and are normally
remapped by the launch file.

## Expression safety

Expressions are parsed with Python 3.10's AST and evaluated by a restricted
walker. Only numeric constants, `+ - * /`, unary `+ -`, `abs`, `min`, `max`, and
integer indexing into `axes`/`buttons` are permitted. Attribute access, slicing,
comprehensions, comparisons, power, imports, and arbitrary function calls are
rejected. Inputs and outputs must be finite. Invalid expressions or samples are
logged and dropped; the node never publishes a partial `Joy` and never crashes
the executor.

## License and provenance

BSD. See `LICENSE` and `MIGRATION.md`. The remapper preserves the author notice
of the original `joy_remap.py` (`furushchev`) and the driver author notices from
the original `joy_node.cpp` in `MIGRATION.md`/`package.xml`.

No ROS 1 compatibility layer is included. Hardware behavior and a Humble build
have **not** been verified by this migration.
