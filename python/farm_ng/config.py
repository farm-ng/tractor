import argparse
import os

from google.protobuf.json_format import MessageToJson
from google.protobuf.wrappers_pb2 import Int32Value

from farm_ng.blobstore import Blobstore
from farm_ng_proto.tractor.v1.apriltag_pb2 import ApriltagConfig
from farm_ng_proto.tractor.v1.apriltag_pb2 import TagConfig
from farm_ng_proto.tractor.v1.resource_pb2 import BUCKET_CONFIGURATIONS
from farm_ng_proto.tractor.v1.tracking_camera_pb2 import CameraConfig
from farm_ng_proto.tractor.v1.tracking_camera_pb2 import TrackingCameraConfig
from farm_ng_proto.tractor.v1.tractor_pb2 import TractorConfig


def _in2m(inches: float) -> float:
    return 0.0254*inches


class TractorConfigManager:
    @staticmethod
    def saved():
        blobstore = Blobstore()
        config = TractorConfig()
        blobstore.read_protobuf_from_json_file(
            os.path.join(blobstore.bucket_relative_path(BUCKET_CONFIGURATIONS), 'tractor.json'),
            config,
        )
        return config

    @staticmethod
    def default():
        config = TractorConfig()
        config.wheel_baseline.value = _in2m(42.0)
        config.wheel_radius.value = _in2m(17/2.0)
        config.hub_motor_gear_ratio.value = 29.9
        config.hub_motor_poll_pairs.value = 8
        config.topology = TractorConfig.Topology.TOPOLOGY_TWO_MOTOR_DIFF_DRIVE
        return config


class ApriltagConfigManager:
    @staticmethod
    def saved():
        blobstore = Blobstore()
        config = ApriltagConfig()
        blobstore.read_protobuf_from_json_file(
            os.path.join(blobstore.bucket_relative_path(BUCKET_CONFIGURATIONS), 'apriltag.json'),
            config,
        )
        return config

    @staticmethod
    def default():
        config = ApriltagConfig()
        config.tag_library.tags.extend([TagConfig(id=id, size=0.16) for id in range(0, 10)])
        return config


class CameraConfigManager:
    @staticmethod
    def saved():
        blobstore = Blobstore()
        config = TrackingCameraConfig()
        blobstore.read_protobuf_from_json_file(
            os.path.join(blobstore.bucket_relative_path(BUCKET_CONFIGURATIONS), 'camera.json'),
            config,
        )
        return config

    @staticmethod
    def default():
        config = TrackingCameraConfig()
        config.camera_configs.extend(
            [
                CameraConfig(
                    serial_number='15322110300', name='tracking_camera/front', model=CameraConfig.MODEL_INTEL_T265,
                    udp_stream_port=Int32Value(value=5000),
                ),
                CameraConfig(
                    serial_number='923322071915', name='tracking_camera/front_depth', model=CameraConfig.MODEL_INTEL_D435I,
                    udp_stream_port=Int32Value(value=5001),
                ),
            ],
        )
        return config


def gentractor(args):
    print(MessageToJson(TractorConfigManager.default(), including_default_value_fields=True))


def genapriltag(args):
    print(MessageToJson(ApriltagConfigManager.default(), including_default_value_fields=True))


def gencamera(args):
    print(MessageToJson(CameraConfigManager.default(), including_default_value_fields=True))


def main():
    parser = argparse.ArgumentParser(epilog='e.g. python -m farm_ng.config gentractor > $BLOBSTORE_ROOT/configurations/tractor.json')
    subparsers = parser.add_subparsers()
    gentractor_parser = subparsers.add_parser('gentractor')
    gentractor_parser.set_defaults(func=gentractor)
    genapriltag_parser = subparsers.add_parser('genapriltag')
    genapriltag_parser.set_defaults(func=genapriltag)
    gencamera_parser = subparsers.add_parser('gencamera')
    gencamera_parser.set_defaults(func=gencamera)

    list_parser = subparsers.add_parser('list')
    list_parser.set_defaults(
        func=(
            lambda args: print(' '.join([c[3:] for c in subparsers.choices.keys() if c.startswith('gen')]))
        ),
    )
    args = parser.parse_args()
    if not hasattr(args, 'func'):
        parser.print_help()
        return
    args.func(args)


if __name__ == '__main__':
    main()
