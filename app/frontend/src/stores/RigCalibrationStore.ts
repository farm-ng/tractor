/* eslint-disable no-console */
import { observable, computed } from "mobx";
import { decodeAnyEvent } from "../models/decodeAnyEvent";
import { Event as BusAnyEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";
import {
  BusEventEmitter,
  BusEventEmitterHandle
} from "../models/BusEventEmitter";
import { BusClient } from "../models/BusClient";
import { EventType, EventTypeId } from "../registry/events";
import { Visualizer, visualizersForEventType } from "../registry/visualization";

interface EventLogEntry {
  stamp: Date | undefined;
  name: string;
  typeUrl: EventTypeId;
  event: EventType;
}

export class RigCalibrationStore {
  private busEventEmitterHandle: BusEventEmitterHandle | null = null;
  @observable eventLog: EventLogEntry[] = [];
  @observable selectedEntry: number | null = null;

  constructor(
    public busClient: BusClient,
    private busEventEmitter: BusEventEmitter
  ) {}

  @computed get selectedEvent(): EventLogEntry | null {
    return this.selectedEntry ? this.eventLog[this.selectedEntry] : null;
  }

  @computed get visualizer(): Visualizer | null {
    if (!this.selectedEvent) {
      return null;
    }
    return visualizersForEventType(this.selectedEvent.typeUrl)[0];
  }

  public startStreaming(): void {
    this.busEventEmitterHandle = this.busEventEmitter.on(
      "*",
      (anyEvent: BusAnyEvent) => {
        if (!anyEvent || !anyEvent.data) {
          console.error(
            `RigCalibrationStore received incomplete event ${anyEvent}`
          );
          return;
        }
        if (!anyEvent.name.startsWith("calibrator")) {
          return;
        }
        const event = decodeAnyEvent(anyEvent);
        if (!event) {
          console.error(
            `RigCalibrationStore could not decode event ${anyEvent}`
          );
          return;
        }
        this.eventLog.push({
          stamp: anyEvent.stamp,
          name: anyEvent.name,
          typeUrl: anyEvent.data.typeUrl as EventTypeId,
          event
        });
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
