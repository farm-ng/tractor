from farm_ng.tractor.sim.models.waypoint import Waypoint
from farm_ng.tractor.sim.utils.database import database
from farm_ng_proto.tractor.v1 import waypoint_pb2
from gensrv.farm_ng_proto.tractor.v1.waypoint_service_twirp_srv import Errors
from gensrv.farm_ng_proto.tractor.v1.waypoint_service_twirp_srv import TwirpServerException
from gensrv.farm_ng_proto.tractor.v1.waypoint_service_twirp_srv import WaypointServiceImpl


class WaypointService(WaypointServiceImpl):
    def CreateWaypoint(self, create_waypoint_request: waypoint_pb2.CreateWaypointRequest):
        print('CreateWaypoint', create_waypoint_request)
        waypoint = Waypoint.fromProto(create_waypoint_request.waypoint)
        result = database.saveWaypoint(waypoint)
        return result.toProto()

    def ListWaypoints(self, list_waypoints_request):
        print('ListWaypoints')
        return waypoint_pb2.ListWaypointsResponse(
            waypoints=[
                waypoint.toProto() for waypoint in database.getAllWaypoints()
            ],
        )

    def GetWaypoint(self, get_waypoint_request: waypoint_pb2.GetWaypointRequest):
        print('GetWaypoint', get_waypoint_request)
        waypoint = database.getWaypoint(get_waypoint_request.waypoint)
        if not waypoint:
            raise TwirpServerException(Errors.NotFound, 'Not Found')
        return waypoint.toProto()

    def DeleteWaypoint(self, delete_waypoint_request: waypoint_pb2.DeleteWaypointRequest):
        print('DeleteWaypoint', delete_waypoint_request)
        success = database.deleteWaypoint(delete_waypoint_request.waypoint)
        if not success:
            raise TwirpServerException(Errors.NotFound, 'Not Found')
        return waypoint_pb2.DeleteWaypointResponse()
