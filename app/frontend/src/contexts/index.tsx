import * as React from "react";
import { BusEventStore } from "../stores/BusEventStore";
import { MediaStreamStore } from "../stores/MediaStreamStore";
import { getWebRTCEmitters } from "../models/getWebRTCEmitters";
import { VisualizationStore } from "../stores/VisualizationStore";
import { ProgramsStore } from "../stores/ProgramsStore";
import { HttpResourceArchive } from "../models/ResourceArchive";
import { env } from "../config";

const baseUrl = env.BASE_URL || window.location.origin;

const [busEventEmitter, mediaStreamEmitter, busClient] = getWebRTCEmitters(
  `${baseUrl}/twirp/farm_ng_proto.tractor.v1.WebrtcApiService/InitiatePeerConnection`
);

const httpResourceArchive = new HttpResourceArchive(`${baseUrl}/blobstore`);

export const storesContext = React.createContext({
  programsStore: new ProgramsStore(busEventEmitter),
  busEventStore: new BusEventStore(busEventEmitter),
  mediaStreamStore: new MediaStreamStore(mediaStreamEmitter),
  visualizationStore: new VisualizationStore(
    busEventEmitter,
    httpResourceArchive
  ),
  baseUrl,
  httpResourceArchive,
  busClient
});
