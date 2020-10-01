import * as React from "react";
import { BusEventStore } from "../stores/BusEventStore";
import { MediaStreamStore } from "../stores/MediaStreamStore";
import { getWebRTCEmitters } from "../models/getWebRTCEmitters";
import { VisualizationStore } from "../stores/VisualizationStore";
import { ProgramsStore } from "../stores/ProgramsStore";
import { RigCalibrationStore } from "../stores/RigCalibrationStore";

const [busEventEmitter, mediaStreamEmitter, busClient] = getWebRTCEmitters(
  `http://${window.location.host}/twirp/farm_ng_proto.tractor.v1.WebRTCProxyService/InitiatePeerConnection`
);

export const storesContext = React.createContext({
  programsStore: new ProgramsStore(busClient, busEventEmitter),
  rigCalibrationStore: new RigCalibrationStore(busClient, busEventEmitter),
  busEventStore: new BusEventStore(busEventEmitter),
  mediaStreamStore: new MediaStreamStore(mediaStreamEmitter),
  visualizationStore: new VisualizationStore(busEventEmitter)
});
