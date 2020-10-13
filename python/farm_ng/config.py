import argparse
import os

from farm_ng.blobstore import Blobstore
from farm_ng_proto.tractor.v1.resource_pb2 import BUCKET_CONFIGURATIONS
from farm_ng_proto.tractor.v1.tractor_pb2 import TractorConfig
from google.protobuf.json_format import MessageToJson


def _in2m(inches: float) -> float:
    return 0.0254*inches


def saved_config() -> TractorConfig:
    blobstore = Blobstore()
    config = TractorConfig()
    blobstore.read_protobuf_from_json_file(
        os.path.join(blobstore.bucket_relative_path(BUCKET_CONFIGURATIONS), 'tractor.json'),
        config,
    )
    return config


def _default_config() -> TractorConfig:
    config = TractorConfig()
    config.wheel_baseline.value = _in2m(42.0)
    config.wheel_radius.value = _in2m(17/2.0)
    config.hub_motor_gear_ratio.value = 29.9
    config.hub_motor_poll_pairs.value = 8
    config.topology = TractorConfig.Topology.TOPOLOGY_TWO_MOTOR_DIFF_DRIVE
    return config


def main_gen(args):
    config = _default_config()
    config.wheel_baseline.value = args.wheel_baseline
    config.wheel_radius.value = args.wheel_radius
    config.hub_motor_gear_ratio.value = args.hub_motor_gear_ratio
    config.hub_motor_poll_pairs.value = args.hub_motor_poll_pairs
    config.topology = TractorConfig.Topology.Value(args.topology)
    print(MessageToJson(config, including_default_value_fields=True))


def main():
    parser = argparse.ArgumentParser(epilog='e.g. python -m farm_ng.config gen > $BLOBSTORE_ROOT/configurations/tractor.json')
    subparsers = parser.add_subparsers()
    parser_gen = subparsers.add_parser('gen')
    cfg = _default_config()
    parser_gen.add_argument('--wheel_baseline', help='Distance between wheels in meters', default=cfg.wheel_baseline.value, type=float)
    parser_gen.add_argument('--wheel_radius', help='Radius of drive wheels in meters', default=cfg.wheel_radius.value, type=float)
    parser_gen.add_argument('--hub_motor_gear_ratio', help='Gear ratio of drive motor', default=cfg.hub_motor_gear_ratio.value, type=float)
    parser_gen.add_argument('--hub_motor_poll_pairs', help='Number of pools in drive motor', default=cfg.hub_motor_poll_pairs.value, type=int)
    parser_gen.add_argument('--topology', help='Topology', default=TractorConfig.Topology.Name(cfg.topology), type=str)
    parser_gen.set_defaults(func=main_gen)
    args = parser.parse_args()
    if not hasattr(args, 'func'):
        parser.print_help()
        return
    args.func(args)


if __name__ == '__main__':
    main()
