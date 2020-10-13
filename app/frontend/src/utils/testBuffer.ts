import { Buffer, TimestampedEvent } from "../types/common";
import { Image } from "../../genproto/farm_ng_proto/tractor/v1/image";
import { CalibrateApriltagRigStatus } from "../../genproto/farm_ng_proto/tractor/v1/calibrate_apriltag_rig";
import { CalibrateBaseToCameraStatus } from "../../genproto/farm_ng_proto/tractor/v1/calibrate_base_to_camera";

export const testBuffer: Buffer = {
  "type.googleapis.com/farm_ng_proto.tractor.v1.Image": {
    test: [
      ...Array(100)
        .fill(0)
        .map<TimestampedEvent<Image>>((_, i) => [
          i,
          Image.fromPartial({
            resource: {
              path: "logs/default2/tracking_camera/front/prospectpark.mp4",
              contentType: "video/mp4"
            },
            fps: 30,
            frameNumber: i
          })
        ])
    ]
  },
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigStatus": {
    test: [
      [
        99,
        CalibrateApriltagRigStatus.fromJSON({
          result: {
            path: "apriltag_rig_models/rig.json",
            contentType:
              "application/json; type=type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigResult"
          }
        })
      ]
    ]
  },
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateBaseToCameraStatus": {
    test: [
      [
        199,
        CalibrateBaseToCameraStatus.fromJSON({
          result: {
            path: "base_to_camera_models/base_to_camera.json",
            contentType:
              "application/json; type=type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateBaseToCameraResult"
          }
        })
      ]
    ]
  }
};
