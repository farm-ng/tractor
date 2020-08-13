import "jest";
import "node-fetch";
/* import { WaypointServiceClientImpl } from "../genproto/farm_ng_proto/tractor/v1/waypoint_service";
import { createTwirpClient } from "../src/clients/createTwirpClient";
import { TwirpError } from "../src/clients/TwirpError";

describe("WaypointService", () => {
  const protobufClient = createTwirpClient(
    WaypointServiceClientImpl,
    "protobuf"
  );

  async function createWaypoint(): Promise<number> {
    const response = await protobufClient.CreateWaypoint({
      waypoint: {
        lat: 42,
        lng: -1,
        angle: 1,
        delay: undefined,
        radius: undefined,
        id: undefined
      }
    });
    if (!response.id) {
      throw Error("Could not create waypoint");
    }
    return response.id;
  }

  it("CreateWaypoint succeeds", async () => {
    const id = await createWaypoint();
    expect(id).not.toBe(0);
  });

  it("ListWaypoints succeeds", async () => {
    const id = await createWaypoint();
    const response = await protobufClient.ListWaypoints({});
    expect(response.waypoints.length).toBeGreaterThan(0);
    expect(response.waypoints.some((waypoint) => waypoint.id === id));
  });

  it("GetWaypoint succeeds", async () => {
    const id = await createWaypoint();
    const response = await protobufClient.GetWaypoint({ waypoint: id });
    expect(response.id).toEqual(id);
  });

  it("DeleteWaypoint succeeds (and GetWaypoint fails)", async () => {
    const id = await createWaypoint();
    await protobufClient.DeleteWaypoint({ waypoint: id });

    try {
      await protobufClient.GetWaypoint({ waypoint: id });
    } catch (error) {
      expect(error).toBeInstanceOf(TwirpError);
      expect(error.message).toBe("Not Found");
      expect(error.code).toBe("not_found");
    }
  });
}); */
