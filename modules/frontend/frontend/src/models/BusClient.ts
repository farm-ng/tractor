import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { EventTypeId } from "../registry/events";

export class BusClient {
  constructor(private dataChannel: RTCDataChannel) {}

  public send(typeUrl: EventTypeId, name: string, value: Uint8Array): void {
    const event = {
      ...BusEvent.fromJSON({
        name,
        stamp: new Date(),
      }),
      data: { typeUrl, value },
    };
    this.dataChannel.send(BusEvent.encode(event).finish());
  }
}
