# zero_day

Control software, firmware and hardware files for a mechatronics robotic arm.

The current geometry, limits and pin assignments are placeholders for a generic 5-DOF arm (three steppers and two servos, plus a servo gripper). Replace them with the real specs in `docs/specs.md`, then update `config/arm.yaml` and `firmware/include/arm_config.h` to match.

## Layout

```
config/arm.yaml          Geometry (DH parameters), joint limits, drives. Single source of truth.
software/arm_control/    Python host package
  config.py              Loads and validates arm.yaml
  kinematics.py          Forward kinematics, Jacobian, damped least squares IK
  trajectory.py          Synchronised trapezoidal joint moves
  protocol.py            Serial command encoding and ArmLink client
software/tests/          pytest suite
software/scripts/        Utilities (workspace.py samples the reachable workspace)
firmware/                PlatformIO project (Arduino Mega by default, ESP32 optional)
hardware/                CAD, electrical drawings, bill of materials
docs/                    Specs template and serial protocol reference
```

## Software

Requires Python 3.10 or newer.

```
cd software
pip install -e ".[dev]"
pytest
python scripts/workspace.py --plot
```

Minimal use:

```python
from arm_control import load_config, forward_kinematics, inverse_kinematics, joint_trajectory

cfg = load_config()
pose = forward_kinematics(cfg, [0, 30, -45, 0, 0])
angles, ok = inverse_kinematics(cfg, [0.30, 0.0, 0.25])
times, path = joint_trajectory(cfg, cfg.home, angles)
```

To drive real hardware:

```python
from arm_control.protocol import ArmLink

arm = ArmLink("/dev/ttyACM0")   # COM3 or similar on Windows
arm.send(b"ENABLE\n")
for q in path:
    arm.move(q)
```

## Firmware

Install [PlatformIO](https://platformio.org/), then:

```
cd firmware
pio run -e mega -t upload
pio device monitor
```

Use `-e esp32` for an ESP32 board. The command set is documented in `docs/protocol.md`.

Safety behaviour: the drivers start disabled, every `MOVE` is checked against joint limits, joint speed is capped in the control loop, and a normally closed e-stop on pin 2 disables the drivers until the host sends `ENABLE` again.

## Still to do once specs arrive

1. Fill in `docs/specs.md`.
2. Set DH parameters, limits and drive settings in `config/arm.yaml`.
3. Set pins and the joint table in `firmware/include/arm_config.h`.
4. Replace the placeholder `HOME` routine with limit switch homing.
5. Complete `hardware/bom.csv`.
