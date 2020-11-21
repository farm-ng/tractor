import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { eventRegistry, EventType, EventTypeId } from "../registry/events";
import { Message } from "../types/common";

export function decodeAnyEvent<T extends EventType>(event: BusEvent): T | null {
  const { data } = event;
  if (!data) {
    return null;
  }
  const message = eventRegistry[
    (data.typeUrl as unknown) as EventTypeId
  ] as Message<T>;
  if (!message) {
    // eslint-disable-next-line no-console
    console.error(`No decoder registered for type: ${data.typeUrl}`);
    return null;
  }
  return message.decode(data.value || new Uint8Array());
}
