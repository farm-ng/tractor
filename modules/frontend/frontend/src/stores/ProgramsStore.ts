/* eslint-disable no-console */
import { computed, observable } from "mobx";
import { decodeAnyEvent } from "../models/decodeAnyEvent";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import {
  BusEventEmitter,
  BusEventEmitterHandle,
} from "../models/BusEventEmitter";
import { EventType, EventTypeId } from "../registry/events";
import {
  ProgramExecution,
  ProgramOutput,
  ProgramSupervisorStatus,
} from "@farm-ng/genproto-core/farm_ng/core/programd";
import { Program, programForProgramId } from "../registry/programs";
import { TimestampedEventVector } from "../types/common";

export class ProgramsStore {
  private busHandle: BusEventEmitterHandle | null = null;
  private lastInputRequired: EventType | null = null;

  // The latest supervisor status
  @observable supervisorStatus: ProgramSupervisorStatus | null = null;

  // A buffer of program status events
  statusLog: TimestampedEventVector<BusEvent> = observable.array([], {
    deep: false,
  });

  // A buffer of program text output (e.g. from stdout)
  outputLog: string[] = observable.array([], { deep: false });

  constructor(private busEventEmitter: BusEventEmitter) {}

  @computed get runningProgram(): ProgramExecution | null {
    return this.supervisorStatus?.running?.program || null;
  }

  @computed get lastProgram(): ProgramExecution | null {
    return this.supervisorStatus?.stopped?.lastProgram || null;
  }

  @computed get latestEvent(): BusEvent | null {
    return this.statusLog.length > 0
      ? this.statusLog[this.statusLog.length - 1][1]
      : null;
  }

  @computed get program(): Program | null {
    const programId = this.runningProgram?.id || this.lastProgram?.id;
    return programId ? programForProgramId(programId) : null;
  }

  @computed get inputRequired(): EventType | null {
    // Somewhat gross. Avoid triggering a mobx update if we're just receiving
    // the same inputRequired request periodically.
    // TODO(isherman): Rethink this whole area.
    if (this.runningProgram && this.latestEvent && this.program) {
      const inputRequired = this.program.inputRequired(this.latestEvent);
      if (
        JSON.stringify(inputRequired) === JSON.stringify(this.lastInputRequired)
      ) {
        return this.lastInputRequired;
      }
      this.lastInputRequired = inputRequired;
      return inputRequired;
    }
    this.lastInputRequired = null;
    return null;
  }

  public startStreaming(): void {
    this.busHandle = this.busEventEmitter.on("*", (busEvent: BusEvent) => {
      if (!busEvent || !busEvent.data) {
        console.error(`ProgramStore received incomplete event ${busEvent}`);
        return;
      }
      if (
        (busEvent.data.typeUrl as EventTypeId) ===
        "type.googleapis.com/farm_ng.core.ProgramSupervisorStatus"
      ) {
        this.supervisorStatus = decodeAnyEvent(busEvent);
      }

      if (!this.runningProgram) {
        return;
      }

      if (busEvent.name.startsWith("programd/stdout") || busEvent.name.startsWith("programd/stderr")) {
        if (
          (busEvent.data.typeUrl as EventTypeId) ==
          "type.googleapis.com/farm_ng.core.ProgramOutput"
        ) {
          const decoded = decodeAnyEvent<ProgramOutput>(busEvent);
          if (!decoded) {
            console.error("Could not decode ProgramOutput.");
            return;
          }
          this.outputLog.push(decoded.line);
          return;
        }
      }
      if (busEvent.name.startsWith(`${this.program?.programId}/`)) {
        this.statusLog.push([
          busEvent.stamp?.getTime() || Date.now(),
          busEvent,
        ]);
      }
    });
  }

  public stopStreaming(): void {
    if (this.busHandle) {
      this.busHandle.unsubscribe();
      this.busHandle = null;
    }
  }

  public clearLogs(): void {
    this.statusLog = observable.array([], { deep: false });
    this.outputLog = observable.array([], { deep: false });
  }
}
