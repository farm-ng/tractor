# python -m farm_ng.apriltag_board_gen --root_tag_id 1 --name intrinsic_board_01

import argparse

import google.protobuf.json_format as json_format
from liegroups import SE3

from farm_ng.proto_utils import se3_to_proto
from farm_ng_proto.tractor.v1.apriltag_pb2 import ApriltagRig
from farm_ng_proto.tractor.v1.geometry_pb2 import Vec3


def MakeApriltagRigNode(rig_name, tag_id, size, x, y):
    node = ApriltagRig.Node()
    node.id = tag_id
    node.frame_name = '%s/tag/%05d' % (rig_name, tag_id)
    node.tag_size = size
    half_size = size/2.0
    node.points_tag.append(Vec3(x=-half_size, y=-half_size, z=0))
    node.points_tag.append(Vec3(x=half_size, y=-half_size, z=0))
    node.points_tag.append(Vec3(x=half_size, y=half_size, z=0))
    node.points_tag.append(Vec3(x=-half_size, y=half_size, z=0))
    node.pose.frame_a = rig_name
    node.pose.frame_b = node.frame_name
    node.pose.a_pose_b.CopyFrom(se3_to_proto(SE3.exp([x, y, 0, 0, 0, 0])))
    return node


def App():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', type=str, default='tag_rig')
    parser.add_argument('--tag_size', type=float, default=0.1)

    parser.add_argument('--root_tag_id', type=int, default=1)
    parser.add_argument('--num_rows', type=int, default=3)
    parser.add_argument('--num_cols', type=int, default=2)

    args = parser.parse_args()

    rig = ApriltagRig()
    rig.name = args.name

    tag_id = args.root_tag_id
    for x in range(args.num_cols):
        for y in range(args.num_rows):
            x_meters = x*args.tag_size*1.2
            y_meters = y*args.tag_size*1.2
            # center to center
            rig.nodes.append(MakeApriltagRigNode(args.name, tag_id, args.tag_size, x_meters, y_meters))
            tag_id += 1

    print(json_format.MessageToJson(rig))


if __name__ == '__main__':
    App()
