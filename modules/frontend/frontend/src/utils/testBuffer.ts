import { Buffer, TimestampedEvent } from "../types/common";
import { Image } from "@farm-ng/genproto-perception/farm_ng/perception/image";
import { CalibrateApriltagRigStatus } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_apriltag_rig";
import { CalibrateBaseToCameraStatus } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_base_to_camera";

export const testBuffer: Buffer = {
  "type.googleapis.com/farm_ng.perception.Image": {
    test: [
      ...Array(100)
        .fill(0)
        .map<TimestampedEvent<Image>>((_, i) => [
          i,
          Image.fromPartial({
            resource: {
              path: "logs/default2/tracking_camera/front/prospectpark.mp4",
              contentType: "video/mp4",
            },
            fps: 30,
            frameNumber: i,
          }),
        ]),
    ],
  },
  "type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigStatus": {
    test: [
      [
        99,
        CalibrateApriltagRigStatus.fromJSON({
          result: {
            path: "apriltag_rig_models/rig.json",
            contentType:
              "application/json; type=type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigResult",
          },
        }),
      ],
    ],
  },
  "type.googleapis.com/farm_ng.calibration.CalibrateBaseToCameraStatus": {
    test: [
      [
        199,
        CalibrateBaseToCameraStatus.fromJSON({
          result: {
            path: "base_to_camera_models/base_to_camera.json",
            contentType:
              "application/json; type=type.googleapis.com/farm_ng.calibration.CalibrateBaseToCameraResult",
          },
        }),
      ],
    ],
  },
};
