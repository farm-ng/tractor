import { observable, computed, ObservableMap } from "mobx";
import { EventTypeId } from "../registry/events";
import {
  Visualizer,
  VisualizerId,
  VisualizerOption,
  VisualizerOptionConfig,
  visualizerRegistry
} from "../registry/visualization";
import { Buffer } from "./buffer";

export function visualizerId(v: Visualizer): VisualizerId {
  return Object.getPrototypeOf(v).constructor.id;
}

export class Panel {
  public id = Math.random().toString(36).substring(7);

  @observable tagFilter = "";
  @observable eventType: EventTypeId =
    "type.googleapis.com/farm_ng_proto.tractor.v1.Vec2";
  @observable selectedVisualizer = 0;
  @observable selectedOptions = this.optionConfigs.map((_) => 0);

  @computed get visualizers(): Visualizer[] {
    return Object.values(visualizerRegistry).filter(
      (v) => v.types.includes(this.eventType) || v.types === "*"
    );
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
    this.selectedVisualizer = 0;
    this.selectedOptions = this.optionConfigs.map((_) => 0);
  }

  setVisualizer(index: number): void {
    this.selectedVisualizer = index;
    this.selectedOptions = this.optionConfigs.map((_) => 0);
  }

  setOption(optionIndex: number, valueIndex: number): void {
    this.selectedOptions[optionIndex] = valueIndex;
  }
}

export type DataSourceType = "live" | "pause" | "log";

export class VisualizationStore {
  @observable dataSource: DataSourceType = "live";
  @observable bufferStart: Date | null = null;
  @observable bufferEnd: Date | null = null;
  @observable bufferRangeStart = 0;
  @observable bufferRangeEnd = 1;
  @observable bufferThrottle = 0;
  @observable bufferSize = 0;
  @observable buffer: Buffer = {};
  @observable bufferLoadProgress = 0;

  @observable panels: ObservableMap<string, Panel>;

  constructor() {
    const p = new Panel();
    this.panels = new ObservableMap<string, Panel>([[p.id, p]]);
  }

  addPanel(): void {
    const panel = new Panel();
    this.panels.set(panel.id, panel);
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

  deletePanel(id: string): void {
    this.panels.delete(id);
  }
}
