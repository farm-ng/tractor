from farm_ng_proto.tractor.v1 import geometry_pb2
from liegroups import SE3


def se3_to_proto(pose: SE3) -> geometry_pb2.SE3Pose:
    q = pose.rot.to_quaternion(ordering='xyzw')
    return geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=pose.trans[0], y=pose.trans[1], z=pose.trans[2]),
        rotation=geometry_pb2.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
    )
