// The following hack is a work around for this issue:
// https://github.com/stephenh/ts-proto/issues/108

import * as protobuf from "protobufjs/minimal";
import * as Long from "long";

if (protobuf.util.Long !== Long) {
  protobuf.util.Long = Long;
  protobuf.configure();
}

import { Event as BusAnyEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";

type ICallback = (event: BusAnyEvent) => void;

type ICallbackMap = {
  [event: string]: ICallback[];
};

export class BusEventEmitter {
  private callbacks: ICallbackMap = {};

  public on(event: string, callback: ICallback): void {
    this.callbacks[event] = [...(this.callbacks[event] || []), callback];
  }

  public emit(event: BusAnyEvent): void {
    (this.callbacks["*"] || []).forEach((cb) => cb(event));
    if (event.data) {
      (this.callbacks[event.data.typeUrl] || []).forEach((cb) => cb(event));
    }
  }
}
