# Additional Resources on Transforms

## Introduction

To successfully solve the weed detection challenge, you will need to understand how to use transforms. This document provides additional resources on transforms. While there might be alternative ways to solve the challenge, we expect that you will use these series of transforms to solve the challenge.

- First, you will need to transform the weed locations from the camera perspective (pixels) to the camera reference frame (meters).
- Next, you will need to transform the weed locations from the camera reference frame to the robot reference frame.
- Finally, you will need to transform the weed locations from the robot reference frame to the world frame.

This series of transformations is illustrated in the following figure:

![Transforms](assets/transforms.png)

## Transforming from Pixels to Meters

To convert the weed locations in pixels to a position in meters, you will need to know a few properties of the camera. These properties are:

- The horizontal field of view of the camera in degrees: 1.7633 radians
- The width of the camera sensor in pixels: 1280 pixels
- The height of the camera sensor in pixels: 720 pixels

## Transform Resources

Here are some additional resources on coordinate transforms, especially in the context of robotics:

- [Transformations Part 1: Coordinate Transforms and Robotics](https://articulatedrobotics.xyz/transformations-1-coordinate_transforms/)
- [Transformations Part 2: Linear Transformations](https://articulatedrobotics.xyz/transformations-2-linear_transforms/)
- [Transformations Part 3: 2D Rotations](https://articulatedrobotics.xyz/transformations-3-rotation_matrices_2d/)
- [Transformations Part 4: Translations](https://articulatedrobotics.xyz/4-translations/)
- [Transformations Part 5: Affine Transformation Matrices](https://articulatedrobotics.xyz/5-transformation_matrices/)
- [Transformations Part 6: 3D Rotations](https://articulatedrobotics.xyz/6-rotations_3d/)
- [Coordinate Transformations in Robotics - MathWorks](https://www.mathworks.com/help/robotics/ug/coordinate-transformations-in-robotics.html)
- [ROS Wiki: tf](https://wiki.ros.org/tf)
- [ROS Wiki: tf2](https://wiki.ros.org/tf2)
- [Access the tf Transformation Tree in ROS](https://www.mathworks.com/help/ros/ug/access-the-tf-transformation-tree-in-ros.html)
