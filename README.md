# panda_planner

Code to plan trajectories for the Franka Emika Panda robotic manipulator.

https://github.com/williamsnider/panda_planner/assets/38354953/ca2cf7e8-0cd9-4160-b797-5c069f539a17

## Description

To study the neural basis of haptic 3D-shape perception, we need to be able to deliver many 3D-printed stimuli to subjects. We designed an experimental setup in which a robotic manipulator (Franka Emika Panda) delivers objects to a subject for grasping. For each trial, the robotic arm grabs shapes stored on shelves, delivers them to a subject for grasping, and returns the object to the shelf. The benefit of this setup is that 900+ objects can be sampled from during an experimental session, enhancing our ability to study which shape features are encoded by individual neurons.

![Shelving System](assets/shelving_system-1.png)

For planning paths and trajectories, _panda_planner_ ensures that:

- kinematic constraints are not violated (joint position/velocity/acceleration/jerk)
- environment collisions (e.g. with shelves, other objects) do not occur
- self collisions do not occur

## How to Use

```

```

## Description

## How to use

## Installation Instructions (Ubuntu 20.04)
