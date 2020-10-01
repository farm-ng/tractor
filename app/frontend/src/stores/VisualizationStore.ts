import { observable, computed, transaction } from "mobx";
import {
  BusEventEmitter,
  BusEventEmitterHandle
} from "../models/BusEventEmitter";
import {
  HttpResourceArchive,
  ResourceArchive
} from "../models/ResourceArchive";
import { StreamingBuffer } from "../models/StreamingBuffer";
import { EventTypeId } from "../registry/events";
import {
  Visualizer,
  VisualizerId,
  VisualizerOption,
  VisualizerOptionConfig,
  visualizersForEventType
} from "../registry/visualization";
import { Buffer, TimestampedEvent } from "../types/common";
import { duration } from "../utils/duration";

export function visualizerId(v: Visualizer): VisualizerId {
  return Object.getPrototypeOf(v).constructor.id;
}

function mapToDateRange(value: number, startDate: Date, endDate: Date): Date {
  return new Date(
    startDate.getTime() + (endDate.getTime() - startDate.getTime()) * value
  );
}

function timestampedEventComparator(
  a: TimestampedEvent,
  b: TimestampedEvent
): number {
  return a[0] - b[0];
}

export class Panel {
  public id = Math.random().toString(36).substring(7);

  @observable tagFilter = "";
  @observable eventType: EventTypeId | null = null;
  @observable selectedVisualizer = 0;
  @observable selectedOptions = this.optionConfigs.map((_) => 0);

  @computed get visualizers(): Visualizer[] {
    return visualizersForEventType(this.eventType);
  }

  @computed get visualizer(): Visualizer {
    return this.visualizers[this.selectedVisualizer];
  }

  @computed get optionConfigs(): VisualizerOptionConfig[] {
    return this.visualizer.options;
  }

  @computed get options(): VisualizerOption[] {
    return this.optionConfigs.map((o, index) => ({
      ...o,
      value: o.options[this.selectedOptions[index]]
    }));
  }

  setEventType(d: EventTypeId): void {
    this.eventType = d;
    this.setVisualizer(0);
  }

  setVisualizer(index: number): void {
    this.selectedVisualizer = index;
    this.selectedOptions = this.optionConfigs.map((_) => 0);
  }

  setOption(optionIndex: number, valueIndex: number): void {
    this.selectedOptions[optionIndex] = valueIndex;
  }
}

// const testBuffer: Buffer = {
//   "type.googleapis.com/farm_ng_proto.tractor.v1.CalibratorStatus": {
//     test: [
//       [
//         0,
//         CalibratorStatus.fromJSON({
//           apriltagRig: {
//             numFrames: 10,
//             rigModelResource: {
//               path: "cal01/apriltag_rig_model/solved-02807-00021.json",
//               archivePath: "",
//               contentType:
//                 "application/json; type=type.googleapis.com/farm_ng_proto.tractor.v1.MonocularApriltagRigModel"
//             }
//           }
//         })
//       ]
//     ]
//   }
// };

export class VisualizationStore {
  @observable bufferStart: Date | null = null;
  @observable bufferEnd: Date | null = null;
  @observable bufferRangeStart = 0;
  @observable bufferRangeEnd = 1;
  @observable bufferThrottle = 0;
  @observable buffer: Buffer = {};
  @observable bufferLogLoadProgress = 0;
  @observable bufferExpirationWindow = 1 * duration.minute;
  @observable resourceArchive: ResourceArchive = new HttpResourceArchive(
    `http://${window.location.host}/resources`
  );
  @observable panels: { [k: string]: Panel } = {};

  private streamingBuffer: StreamingBuffer = new StreamingBuffer();
  @observable
  private busEventEmitterHandle: BusEventEmitterHandle | null = null;
  @observable private streamingTimerHandle: number | null = null;

  constructor(public busEventEmitter: BusEventEmitter) {
    const p = new Panel();
    this.panels = { [p.id]: p };
  }

  @computed get isStreaming(): boolean {
    return (
      this.busEventEmitterHandle !== null && this.streamingTimerHandle !== null
    );
  }

  @computed get bufferEmpty(): boolean {
    return Object.keys(this.buffer).length === 0;
  }

  @computed get bufferRangeStartDate(): Date | null {
    if (!this.bufferStart || !this.bufferEnd) {
      return null;
    }
    return mapToDateRange(
      this.bufferRangeStart,
      this.bufferStart,
      this.bufferEnd
    );
  }

  @computed get bufferRangeEndDate(): Date | null {
    if (!this.bufferStart || !this.bufferEnd) {
      return null;
    }
    return mapToDateRange(
      this.bufferRangeEnd,
      this.bufferStart,
      this.bufferEnd
    );
  }

  addPanel(): void {
    const panel = new Panel();
    this.panels[panel.id] = panel;
  }

  deletePanel(id: string): void {
    delete this.panels[id];
  }

  toggleStreaming(): void {
    const bufferDirty = this.bufferLogLoadProgress > 0;
    if (!this.isStreaming && bufferDirty) {
      this.buffer = {};
      this.setBufferRangeStart(0);
      this.setBufferRangeEnd(1);
      this.bufferStart = null;
      this.bufferEnd = null;
      this.bufferLogLoadProgress = 0;
    }
    this.isStreaming ? this.stopStreaming() : this.startStreaming();
  }

  setBufferRangeStart(value: number): void {
    if (value < this.bufferRangeEnd) {
      this.bufferRangeStart = value;
    }
  }

  setBufferRangeEnd(value: number): void {
    if (value > this.bufferRangeStart) {
      this.bufferRangeEnd = value;
    }
  }

  /* eslint-disable @typescript-eslint/no-non-null-assertion */
  replaceBuffer(streamingBuffer: StreamingBuffer): void {
    this.buffer = streamingBuffer.data;
    Object.values(this.buffer).forEach((streams) => {
      Object.values(streams!).forEach((eventVector) => {
        eventVector.sort(timestampedEventComparator);
      });
    });

    this.bufferStart = streamingBuffer.bufferStart;
    this.bufferEnd = streamingBuffer.bufferEnd;
  }
  /* eslint-enable @typescript-eslint/no-non-null-assertion */

  private startStreaming(): void {
    this.busEventEmitterHandle = this.busEventEmitter.on("*", (event) => {
      this.streamingBuffer?.add(event);
    });

    const updateLoop = (): void => {
      this.updateBuffer(this.streamingBuffer);
      if (this.streamingTimerHandle) {
        this.streamingTimerHandle = window.requestAnimationFrame(updateLoop);
      }
    };
    this.streamingTimerHandle = window.requestAnimationFrame(updateLoop);
  }

  private stopStreaming(): void {
    if (this.busEventEmitterHandle) {
      this.busEventEmitterHandle.unsubscribe();
      this.busEventEmitterHandle = null;
    }
    if (this.streamingTimerHandle) {
      window.cancelAnimationFrame(this.streamingTimerHandle);
      this.streamingTimerHandle = null;
    }
  }
  /* eslint-disable @typescript-eslint/no-non-null-assertion */
  private updateBuffer(streamingBuffer: StreamingBuffer): void {
    transaction(() => {
      Object.entries(streamingBuffer.data).forEach(([typeKey, streams]) => {
        const eventType = typeKey as EventTypeId;
        if (!this.buffer[eventType]) {
          this.buffer[eventType] = {};
        }

        Object.entries(streams!).forEach(([streamKey, values]) => {
          if (!this.buffer[eventType]![streamKey]) {
            this.buffer[eventType]![streamKey] = [];
          }
          this.buffer[eventType]![streamKey].push(
            ...values.sort(timestampedEventComparator)
          );
        });
      });
      this.evictExpiredData();
      this.bufferStart = this.bufferStart || this.streamingBuffer.bufferStart;
      this.bufferEnd = this.streamingBuffer.bufferEnd || this.bufferEnd;
      streamingBuffer.clear();
    });
  }
  /* eslint-enable @typescript-eslint/no-non-null-assertion */

  /* eslint-disable @typescript-eslint/no-non-null-assertion */
  private evictExpiredData(): void {
    const horizon = Date.now() - this.bufferExpirationWindow;
    let earliestEvent: number | null = null;
    Object.entries(this.buffer).forEach(([typeKey, streams]) => {
      const eventType = typeKey as EventTypeId;
      Object.entries(streams!).forEach(([streamKey, values]) => {
        const firstIndexToKeep = values.findIndex(([t, _]) => t >= horizon);
        if (firstIndexToKeep === -1) {
          delete this.buffer[eventType]![streamKey];
        } else {
          if (firstIndexToKeep > 0) {
            this.buffer[eventType]![streamKey] = values.slice(firstIndexToKeep);
          }
          earliestEvent = Math.min(
            earliestEvent || Number.MAX_SAFE_INTEGER,
            values[firstIndexToKeep][0]
          );
        }
      });
      if (Object.keys(this.buffer[eventType]!).length === 0) {
        delete this.buffer[eventType];
      }
    });
    this.bufferStart = earliestEvent ? new Date(earliestEvent) : null;
  }
  /* eslint-enable @typescript-eslint/no-non-null-assertion */
}
