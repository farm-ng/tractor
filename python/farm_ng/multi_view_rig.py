import argparse

import google.protobuf.json_format as json_format

from farm_ng.blobstore import Blobstore
from farm_ng_proto.tractor.v1.calibrate_multi_view_apriltag_rig_pb2 import CalibrateMultiViewApriltagRigResult
from farm_ng_proto.tractor.v1.calibrator_pb2 import MultiViewApriltagRigModel


def App():
    parser = argparse.ArgumentParser()
    parser.add_argument('--result')
    args = parser.parse_args()

    result = CalibrateMultiViewApriltagRigResult()
    store = Blobstore()
    store.read_protobuf_from_json_file(args.result, result)
    model = MultiViewApriltagRigModel()
    store.read_protobuf_from_binary_file(result.multi_view_apriltag_rig_solved.path, model)

    print(json_format.MessageToJson(model.camera_rig))


if __name__ == '__main__':
    App()
