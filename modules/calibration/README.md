## Overview

This module provides a toolset for batch calibration of sensors to machines. It leverages fiducial markers and human-in-the-loop for capture and labeling. 

In the long-term, we'd like to provide real-time calibration tools such as monitoring calibration accuracy, online sensor calibration, and natural-feature calibration.

## Concepts

- **Sensors** -- sensor models represent different types of measurement devices (pinholes, stereo, depth cameras, LIDARs, encoders) and have a way of mapping between their measurement space and the sensor’s 3d reference frame. Typically, each sensor has a reference frame convention determined by the vendor or software driver.
- **Measurements** - each sensor produces (directly or indirectly) measurements, such as pixel locations of fiducial markers (apriltag corners) or 3d point clouds which are useful for calibration.
- **Measurement correspondence** - techniques which allow the system to correspond measurements from different sensors that are associated with the same physical object or process such that they may be effectively used in the loss function of the calibration optimizer. For example, apriltags give us a unique label per corner, and planar surfaces from LIDAR scans can be aligned to april tags through point to plane distance losses.
- **Calibration optimizer** - we’re basically solving min arg (pose tree, kinematic model variables, sensor model variables) |loss(measurements)|...
- **Kinematic Model** - this describes the set of actuators and linkages of the robot, and generally describes how it moves in the workspace as a function of its actuator’s state. Mostly the kinematic model is provided via cad or measurements, and we’re interested in it to help constrain the solution of the sensors’ pose with respect to rigid links.
- **Pose tree** - this describes the pose between reference frames of fiducial markers, sensors, links, actuators, and the workspace. Some of these relationships change over time, are a function of actuator states, or are assumed to be rigid.

## Sensor Support

- Pinhole cameras
- Fisheye cameras
- RGBD Cameras (intel realsense d435i)
- Stereo Cameras (intel realsense t265)
- IMU (xsens)
- 3D scanning LIDAR (e.g. velodyne, waymo)
- Encoders (actuators and wheels)

## Kinematic Support

- Linear and Rotary actuators
- Differential drive mobile base
- Ackerman steering mobile base

## Fiducial Support

- Apriltag grid board (for intrinsic calibration of pinhole, fisheye and stereo cameras)
- Apriltag attached to rigid objects (for large scale extrinsic calibration fixtures)
- Apriltag half cubes (for LIDAR to imager measurements)
