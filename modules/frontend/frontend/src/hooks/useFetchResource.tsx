/* eslint-disable no-console */
import { useState, useEffect } from "react";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { ResourceArchive } from "../models/ResourceArchive";
import { eventRegistry, EventType, EventTypeId } from "../registry/events";
import { parseResourceContentType } from "../utils/protoConversions";
// eslint-disable-next-line @typescript-eslint/no-unused-vars
import type { Message } from "../types/common";

export function useFetchResource<T extends EventType>(
  resource?: Resource,
  resourceArchive?: ResourceArchive
): T | undefined {
  const [value, setValue] = useState<T>();
  useEffect(() => {
    const fetchResult = async (): Promise<void> => {
      if (!resource || !resourceArchive) {
        return;
      }
      try {
        const [format, typeUrl] = parseResourceContentType(resource);
        if (!format || !typeUrl) {
          throw Error(`Could not parse contentType: ${resource.contentType}`);
        }
        const Message = eventRegistry[typeUrl as EventTypeId] as Message<T>;
        if (format === "application/protobuf") {
          const blob = await resourceArchive.getBlob(resource.path);
          setValue(Message.decode(new Uint8Array(await blob.arrayBuffer())));
        } else if (format === "application/json") {
          const json = await resourceArchive.getJson(resource.path);
          setValue(Message.fromJSON(json));
        }
      } catch (e) {
        console.error(`Error loading resource ${resource.path}: ${e}`);
      }
    };
    fetchResult();
  }, [resource, resourceArchive]);
  return value;
}
