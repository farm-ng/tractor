import { observable, ObservableMap } from "mobx";
import { decodeAnyEvent } from "../models/decodeAnyEvent";
import { BusEventEmitter } from "../models/BusEventEmitter";
import { EventType } from "../registry/events";
import { duration } from "../utils/duration";

interface StreamSnapshot {
  latestEvent: EventType | null;
  latestEventTime?: Date;
  eventsSinceLastSnapshot: number;
}

const snapshotPeriod = duration.second;

// A store for eventbus events.
// Accumulate events (and metadata) in a snapshot, then every snapshotPeriod update the
// observable with the latest snapshot.
export class BusEventStore {
  @observable lastSnapshot = new ObservableMap<string, StreamSnapshot>();
  nextSnapshot = new Map<string, StreamSnapshot>();

  // TODO: Remove this once we have a better way of managing the stream of poses
  public transport: BusEventEmitter;

  constructor(transport: BusEventEmitter) {
    this.transport = transport;
    this.transport.on("*", (event) => {
      this.nextSnapshot.set(event.name, {
        latestEvent: decodeAnyEvent(event),
        latestEventTime: event.stamp,
        eventsSinceLastSnapshot:
          (this.nextSnapshot.get(event.name)?.eventsSinceLastSnapshot || 0) + 1
      });
    });

    setInterval(() => {
      this.lastSnapshot.replace(this.nextSnapshot);
      this.nextSnapshot.forEach((value, key) => {
        this.nextSnapshot.set(key, { ...value, eventsSinceLastSnapshot: 0 });
      });
    }, snapshotPeriod);
  }
}
