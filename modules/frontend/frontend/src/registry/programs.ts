import { EventType } from "./events";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import {
  CalibrateApriltagRigProgram,
  CalibrateBaseToCameraProgram,
  CalibrateIntrinsicsProgram,
  CalibrateMultiViewApriltagRigProgram,
  CalibrateRobotExtrinsicsProgram,
  CaptureRobotExtrinsicsDatasetProgram,
  CaptureVideoDatasetProgram,
  CreateVideoDatasetProgram,
  DetectApriltagsProgram,
  LogPlaybackProgram,
  ValidateRobotExtrinsicsProgram,
} from "../components/programs/standardPrograms";

export interface Program<T extends EventType = EventType> {
  programId: string;
  inputRequired: (e: BusEvent) => T | null;
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
    programRegistry.find((program) => program.programId == programId) || null
  );
}
