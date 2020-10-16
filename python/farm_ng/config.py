import argparse
import os

from farm_ng.blobstore import Blobstore
from farm_ng_proto.tractor.v1.apriltag_pb2 import ApriltagConfig
from farm_ng_proto.tractor.v1.apriltag_pb2 import TagConfig
from farm_ng_proto.tractor.v1.resource_pb2 import BUCKET_CONFIGURATIONS
from farm_ng_proto.tractor.v1.tractor_pb2 import TractorConfig
from google.protobuf.json_format import MessageToJson


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
        config.tag_library.tags.extend([TagConfig(id=id, size=0.16) for id in range(0, 100)])
        return config


def gentractor(args):
    print(MessageToJson(TractorConfigManager.default(), including_default_value_fields=True))


def genapriltag(args):
    print(MessageToJson(ApriltagConfigManager.default(), including_default_value_fields=True))


def main():
    parser = argparse.ArgumentParser(epilog='e.g. python -m farm_ng.config gentractor > $BLOBSTORE_ROOT/configurations/tractor.json')
    subparsers = parser.add_subparsers()
    gentractor_parser = subparsers.add_parser('gentractor')
    gentractor_parser.set_defaults(func=gentractor)
    genapriltag_parser = subparsers.add_parser('genapriltag')
    genapriltag_parser.set_defaults(func=genapriltag)
    args = parser.parse_args()
    if not hasattr(args, 'func'):
        parser.print_help()
        return
    args.func(args)


if __name__ == '__main__':
    main()
