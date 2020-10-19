import numpy as np
from liegroups import SE3

# loads all the protos for the type registry (isort:skip)
import farm_ng_proto.tractor.v1.io_pb2  # noqa: F401
import farm_ng_proto.tractor.v1.motor_pb2  # noqa: F401
import farm_ng_proto.tractor.v1.steering_pb2  # noqa: F401
import farm_ng_proto.tractor.v1.tracking_camera_pb2  # noqa: F401
from farm_ng_proto.tractor.v1 import geometry_pb2


def se3_to_proto(pose: SE3) -> geometry_pb2.SE3Pose:
    q = pose.rot.to_quaternion(ordering='xyzw')
    return geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=pose.trans[0], y=pose.trans[1], z=pose.trans[2]),
        rotation=geometry_pb2.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
    )


def proto_to_se3(proto: geometry_pb2.SE3Pose) -> SE3:
    rot = SE3.RotationType.from_quaternion(np.array([proto.rotation.w, proto.rotation.x, proto.rotation.y, proto.rotation.z]), ordering='wxyz')
    trans = np.array([
        proto.position.x,
        proto.position.y,
        proto.position.z,
    ])
    return SE3(rot=rot, trans=trans)
