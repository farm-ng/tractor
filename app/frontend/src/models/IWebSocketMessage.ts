import { Status } from "../../genproto/farm_ng_proto/tractor/v1/status";

export interface ISetWaypoint {
  waypoint: number[];
}

export type IWebSocketMessage = Status | ISetWaypoint;
