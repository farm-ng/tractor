import { CalibrateApriltagRigProgram } from "../components/programs/CalibrateApriltagRig";
import { CalibrateBaseToCameraProgram } from "../components/programs/CalibrateBaseToCamera";
import { CaptureVideoDatasetProgram } from "../components/programs/CaptureVideoDataset";
import { DetectApriltagsProgram } from "../components/programs/DetectApriltags";
import { EventType } from "./events";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { CalibrateMultiViewApriltagRigProgram } from "../components/programs/CalibrateMultiViewApriltagRig";
import { CreateVideoDatasetProgram } from "../components/programs/CreateVideoDataset";
import { CalibrateIntrinsicsProgram } from "../components/programs/CalibrateIntrinsics";
import { CaptureRobotExtrinsicsDatasetProgram } from "../components/programs/CaptureRobotExtrinsicsDataset";
import { LogPlaybackProgram } from "../components/programs/LogPlayback";
import { CalibrateRobotExtrinsicsProgram } from "../components/programs/CalibrateRobotExtrinsics";
import { ValidateRobotExtrinsicsProgram } from "../components/programs/ValidateRobotExtrinsics";

export interface Program<T extends EventType = EventType> {
  programIds: readonly string[];
  inputRequired: (e: BusEvent) => T | null;
  eventLogPredicate: (e: BusEvent) => boolean;
  MultiElement: React.FC<ProgramProps<T>>;
}

export interface ProgramProps<T extends EventType = EventType> {
  inputRequired: T;
}

export const programRegistry: Program[] = [
  CaptureVideoDatasetProgram as Program,
  CalibrateApriltagRigProgram as Program,
  CalibrateBaseToCameraProgram as Program,
  CalibrateIntrinsicsProgram as Program,
  CalibrateMultiViewApriltagRigProgram as Program,
  CaptureRobotExtrinsicsDatasetProgram as Program,
  CalibrateRobotExtrinsicsProgram as Program,
  ValidateRobotExtrinsicsProgram as Program,
  CreateVideoDatasetProgram as Program,
  DetectApriltagsProgram as Program,
  LogPlaybackProgram as Program,
];

export function programForProgramId(programId: string): Program | null {
  return (
    programRegistry.find((program) => program.programIds.includes(programId)) ||
    null
  );
}
