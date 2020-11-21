/* eslint-disable no-console */
import { useState, useEffect } from "react";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { ResourceArchive } from "../models/ResourceArchive";

export function useFetchDataUrl(
  resource?: Resource,
  resourceArchive?: ResourceArchive
): string | null {
  const [value, setValue] = useState<string | null>(null);
  useEffect(() => {
    const fetchResult = async (): Promise<void> => {
      if (!resource || !resourceArchive) {
        return;
      }
      try {
        setValue(await resourceArchive.getDataUrl(resource.path));
      } catch (e) {
        console.error(`Error loading resource ${resource.path}: ${e}`);
      }
    };
    fetchResult();
  }, [resource, resourceArchive]);
  return value;
}
