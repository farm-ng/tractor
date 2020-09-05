/* eslint-disable no-console */
import { Event as BusAnyEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";
import { SteeringCommand } from "../../genproto/farm_ng_proto/tractor/v1/steering";
import {
  TrackingCameraPoseFrame,
  TrackingCameraMotionFrame
} from "../../genproto/farm_ng_proto/tractor/v1/tracking_camera";
import { NamedSE3Pose } from "../../genproto/farm_ng_proto/tractor/v1/geometry";
import { MotorControllerState } from "../../genproto/farm_ng_proto/tractor/v1/motor";
import { ApriltagDetections } from "../../genproto/farm_ng_proto/tractor/v1/apriltag";
import { BusEvent } from "./BusEvent";

export function decodeAnyEvent(event: BusAnyEvent): BusEvent | null {
  const eventData = event.data;
  if (!eventData || !eventData.value) return null;
  const decoder = ((typeUrl: string) => {
    switch (typeUrl) {
      case "type.googleapis.com/farm_ng_proto.tractor.v1.SteeringCommand":
        return SteeringCommand.decode;
      case "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraPoseFrame":
        return TrackingCameraPoseFrame.decode;
      case "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraMotionFrame":
        return TrackingCameraMotionFrame.decode;
      case "type.googleapis.com/farm_ng_proto.tractor.v1.NamedSE3Pose":
        return NamedSE3Pose.decode;
      case "type.googleapis.com/farm_ng_proto.tractor.v1.MotorControllerState":
        return MotorControllerState.decode;
      case "type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagDetections":
        return ApriltagDetections.decode;
      default:
        console.log("Unknown message type: ", typeUrl);
        return undefined;
    }
  })(eventData.typeUrl);

  return decoder ? decoder(eventData.value) : null;
}
