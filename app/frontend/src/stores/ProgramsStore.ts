import { computed, observable } from "mobx";
import { decodeAnyEvent } from "../models/decodeAnyEvent";
import { Event as BusAnyEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";
import {
  BusEventEmitter,
  BusEventEmitterHandle
} from "../models/BusEventEmitter";
import { EventTypeId } from "../registry/events";
import {
  ProgramExecution,
  ProgramSupervisorStatus
} from "../../genproto/farm_ng_proto/tractor/v1/program_supervisor";
import { BusClient } from "../models/BusClient";

export class ProgramsStore {
  private busEventEmitterHandle: BusEventEmitterHandle | null = null;
  @observable programSupervisorStatus: ProgramSupervisorStatus | null = null;

  constructor(
    public busClient: BusClient,
    private busEventEmitter: BusEventEmitter
  ) {}

  @computed get runningProgram(): ProgramExecution | null {
    return this.programSupervisorStatus?.running?.program || null;
  }

  @computed get lastProgram(): ProgramExecution | null {
    return this.programSupervisorStatus?.stopped?.lastProgram || null;
  }

  public startStreaming(): void {
    const statusTypeId =
      "type.googleapis.com/farm_ng_proto.tractor.v1.ProgramSupervisorStatus";
    this.busEventEmitterHandle = this.busEventEmitter.on(
      statusTypeId,
      (event: BusAnyEvent) => {
        if (!event || !event.data) {
          // eslint-disable-next-line no-console
          console.error(
            `Subscriber to type ${statusTypeId} receives incomplete event ${event}`
          );
          return;
        }
        const typeUrl = event.data.typeUrl as EventTypeId;
        if (typeUrl !== statusTypeId) {
          // eslint-disable-next-line no-console
          console.error(
            `Subscriber to type ${statusTypeId} received message of type ${typeUrl}`
          );
          return;
        }
        this.programSupervisorStatus = decodeAnyEvent(event);
      }
    );
  }

  public stopStreaming(): void {
    if (this.busEventEmitterHandle) {
      this.busEventEmitterHandle.unsubscribe();
      this.busEventEmitterHandle = null;
    }
  }
}
