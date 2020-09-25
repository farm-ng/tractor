## Conventions

Conventions help make the code more readable, idiomatic, and
maintainable.  For things like transforms and units the lack thereof
is often the root of many catastrophic failures and late nights/early
morning head scratching.

## C++ Style

An attempt will be made to follow the google c++ style guide, except
for a few exceptions listed below.

Standard we're following is C++17, and using the gcc compiler.

clang-format is used for format the c++ files, the format is based on
the google style clang-format config in the root of the repository

### Exceptions

Google doesn't use them: https://google.github.io/styleguide/cppguide.html#Exceptions

However, we're a new codebase, and the benefits of using them may
outway the negatives.  Specific guidance on how we use exceptions is
still evolving, more here later.


## Units

Preferred units are SI:
* meters
* meters per second
* meters per second per second
* radians
* pixels

Please document units in comments. Where its not obvious, suggest appending a post
fix with units (``size_inches``, ``radius_cm``).  If no units are in
the name, assume its in meters.

## Reference frames, poses and points

An excellent primer on this topic is Introduction to Robotics, John
J. Craig, Chapter 2, Spatial descriptions and transformations.

A pose (i.e. transform)a is maps a point specified in one frame into
another, using a convenient algebra:

```
point_a = a_pose_b * point_b
```
Here the pose ``a_pose_b`` maps the point in frame ``b`` to a point in frame ``a``

```
a_pose_c = a_pose_b * b_pose_c
```

Here, notice the pose naming convention follows the algebra described
in Craig, where frame ``b`` cancels, and its easy to understand that
the resulting pose transforms points from frame ``c`` to points in
frame ``a``.

A common error in code is caused by poor naming, or incorrect order of
pose multiplication:

```
# wrong order, can be seen by using naming convention
a_pose_c = b_pose_c*a_pose_b

# poor naming conventions lead to very bad errors or hard to understand transformations.
points = some_points_from_camera()
pose = (pose_of_camera*pose_to_world).inverse()
points = pose*points
```

Ideally we would have some way to check the frame of points and poses
at compile/runtime. In protobuf, this is the purpose of NamedSE3Pose:

```
message NamedSE3Pose {
string frame_a = 1;
string frame_b = 2;
SE3Pose a_pose_b = 3;
}
```

As of yet, we don't have utilities built up that help enforce this,
and its still a work in progress.


Conventions:

```

# a single point, in some reference frame
point_<frame_name>
# e.g.
point_a
point_camera_front
point_robot_base

# a collection of points
points_<frame_name>
# e.g.

points_camera_front = np.array((n,3))
std::vector<Eigen::Vector3d> points_robot_base;

# A single pose
<frame a>_pose_<frame b>

camera_front_pose_robot_base
camera_front_pose_camera_front_left

Sophus::SE3d camera_pose_base;
Eigen::Vector3d point_camera;
Eigen::Vector3d point_base = camera_pose_base.inverse() * point_camera;

# A collection of poses, that map from one logical frame to the next.
<frame a>_poses_<frame b>

std::vector<Sophus::SE3d> camera_poses_tag0;
```
