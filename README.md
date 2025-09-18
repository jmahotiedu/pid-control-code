# PID Control Code

Simple MuJoCo joint-space PD control prototype for a 14-DOF setup.

## Files

- `startup.py`: Loads the MuJoCo model, defines a target joint pose, runs the simulation loop, and applies controller torques.
- `PID_Control.py`: Defines `JointSpacePDController`, which computes PD torques with bias-force compensation and actuator clipping.

## Requirements

- Python 3.9+
- `mujoco`
- `numpy`
- A model XML file named `aloha.xml` in this directory (or update the path in `startup.py`)

## Run

```bash
python startup.py
```

