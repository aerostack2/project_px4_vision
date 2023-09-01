# AS2 PX4 Example

This example is currently working in simulation mode with ROS Humble and Gazebo Classic.

It is **not** working in Gazebo Ignition yet.

## Simulation mode

### Mission

```bash
./launch_as2.bash -s
```

And in the free terminal in TMUX:

```bash
python3 mission.py -s
```

### Teleoperation

```bash
./launch_as2.bash -s -t
```
