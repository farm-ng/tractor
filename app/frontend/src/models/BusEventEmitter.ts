import { Event as BusEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";

type ICallback = (event: BusEvent) => void;

type ICallbackMap = {
  [event: string]: ICallback[];
};

export type BusEventEmitterHandle = { unsubscribe: () => void };

// A generic emitter of eventbus events.
// May be used by various transports (e.g. webRTC, websocket, or a test transport).
export class BusEventEmitter {
  private callbacks: ICallbackMap = {};

  public on(eventType: string, callback: ICallback): BusEventEmitterHandle {
    this.callbacks[eventType] = [
      ...(this.callbacks[eventType] || []),
      callback
    ];

    return {
      unsubscribe: () => this.off(eventType, callback)
    };
  }

  public off(eventType: string, callback: ICallback): void {
    if (!this.callbacks[eventType]) {
      return;
    }

    this.callbacks[eventType] = this.callbacks[eventType].filter(
      (cb) => cb !== callback
    );
  }

  public emit(event: BusEvent): void {
    (this.callbacks["*"] || []).forEach((cb) => cb(event));
    if (event.data) {
      (this.callbacks[event.data.typeUrl] || []).forEach((cb) => cb(event));
    }
  }
}
