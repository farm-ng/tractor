.. _chapter-calibration_module:

Calibration
===========

The `calibration` module provides software for batch calibration of sensors to machines.

Calibration routines leverage fiducial markers and human-in-the-loop for capture and labeling.

Concepts
--------

**Sensors**

Sensor models represent different types of measurement devices (pinholes, stereo, depth cameras, LIDARs, encoders) and have a way of mapping between their measurement space and the sensor’s 3d reference frame. Typically, each sensor has a reference frame convention determined by the vendor or software driver.

**Measurements**

Each sensor produces (directly or indirectly) measurements, such as pixel locations of fiducial markers (apriltag corners) or 3d point clouds which are useful for calibration.

**Measurement correspondence**

Techniques which allow the system to correspond measurements from different sensors that are associated with the same physical object or process such that they may be effectively used in the loss function of the calibration optimizer. For example, apriltags give us a unique label per corner, and planar surfaces from LIDAR scans can be aligned to april tags through point to plane distance losses.

**Calibration optimizer**

We use Ceres to solve, basically ``min arg (pose tree, kinematic model variables, sensor model variables) |loss(measurements)|...```

**Kinematic Model**

This describes the set of actuators and linkages of the robot, and generally describes how it moves in the workspace as a function of its actuator’s state. Mostly the kinematic model is provided via cad or measurements, and we’re interested in it to help constrain the solution of the sensors’ pose with respect to rigid links.

**Pose tree**

This describes the pose between reference frames of fiducial markers, sensors, links, actuators, and the workspace. Some of these relationships change over time, are a function of actuator states, or are assumed to be rigid.

Programs
--------
The calibration module implements the following programs:

- Calibrate Apriltag Rig
- Calibrate Intrinsics
- Calibrate Multiview Apriltag Rig
- Calibrate Robot Extrinsics

Models
--------

Most programs can be parameterized by the following:

**Sensor Models**

- Pinhole cameras
- Fisheye cameras
- RGBD Cameras (e.g. Intel Realsense D435i)
- Stereo Cameras (e.g. Intel Realsense T265)
- IMU (e.g. Xsens)
- 3D scanning LIDAR (e.g. velodyne, waymo)
- Encoders (actuators and wheels)

**Kinematic Models**

- Linear and Rotary actuators
- Differential drive mobile base
- Ackerman steering mobile base

**Fiducials**

- Apriltag grid board (for intrinsic calibration of pinhole, fisheye and stereo cameras)
- Apriltag attached to rigid objects (for large scale extrinsic calibration fixtures)
- Apriltag half cubes (for LIDAR to imager measurements)


Examples
--------
TODO
