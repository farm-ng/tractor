import * as React from "react";
import { BusEventStore } from "../stores/BusEventStore";
import { MediaStreamStore } from "../stores/MediaStreamStore";
import { getWebRTCEmitters } from "../models/getWebRTCEmitters";
import { VisualizationStore } from "../stores/VisualizationStore";
import { ProgramsStore } from "../stores/ProgramsStore";
import { HttpResourceArchive } from "../models/ResourceArchive";

const [busEventEmitter, mediaStreamEmitter, busClient] = getWebRTCEmitters(
  `http://${window.location.hostname}:8081/twirp/farm_ng_proto.tractor.v1.WebRTCProxyService/InitiatePeerConnection`
);

const httpResourceArchive = new HttpResourceArchive(
  `http://${window.location.hostname}:8081/blobstore`
);

export const storesContext = React.createContext({
  programsStore: new ProgramsStore(
    busClient,
    busEventEmitter,
    httpResourceArchive
  ),
  busEventStore: new BusEventStore(busEventEmitter),
  mediaStreamStore: new MediaStreamStore(mediaStreamEmitter),
  visualizationStore: new VisualizationStore(
    busEventEmitter,
    httpResourceArchive
  )
});
