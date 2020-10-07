import { CaptureCalibrationDataset } from "../components/programs/CaptureCalibrationDataset";
import { CalibrateApriltagRig } from "../components/programs/CalibrateApriltagRig";
import { CalibrateBaseToCamera } from "../components/programs/CalibrateBaseToCamera";
import { CaptureVideoDataset } from "../components/programs/CaptureVideoDataset";

export interface ProgramUI {
  programIds: readonly string[];
  component: React.FC;
}

export const programRegistry: { [k: string]: ProgramUI } = {
  [CaptureVideoDataset.id]: new CaptureVideoDataset() as ProgramUI,
  [CaptureCalibrationDataset.id]: new CaptureCalibrationDataset() as ProgramUI,
  [CalibrateApriltagRig.id]: new CalibrateApriltagRig() as ProgramUI,
  [CalibrateBaseToCamera.id]: new CalibrateBaseToCamera() as ProgramUI
};
export const programIds = Object.keys(programRegistry);
export type ProgramId = typeof programIds[number];

export function programUIForProgramId(programId: string): ProgramUI | null {
  return (
    Object.values(programRegistry).find((programUI) =>
      programUI.programIds.includes(programId)
    ) || null
  );
}
