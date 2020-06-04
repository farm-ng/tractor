import pytest
import requests
from farm_ng_proto.tractor.v1 import waypoint_pb2

PORT = 8989
BASE_URL = 'http://localhost:' + str(PORT) + '/twirp/farm_ng_proto.tractor.v1.WaypointService'


def _create_waypoint(waypoint):
    response = requests.post(BASE_URL + '/CreateWaypoint', json={'waypoint': waypoint})
    return int(response.json()['id'])


@pytest.mark.integration
class TestClass:
    def test_list_waypoints(self):
        response = requests.post(BASE_URL + '/ListWaypoints', json={})
        assert response.status_code == 200
        response_body = response.json()
        assert type(response_body is list)

    def test_list_waypoints_pb(self):
        _create_waypoint({'lat': 42, 'lng': -1, 'angle': 0})

        response = requests.post(
            BASE_URL + '/ListWaypoints',
            data=waypoint_pb2.ListWaypointsRequest().SerializeToString(),
            headers={'Content-Type': 'application/protobuf'},
        )
        assert response.status_code == 200
        response_proto = waypoint_pb2.ListWaypointsResponse()
        response_proto.ParseFromString(response.content)
        assert len(response_proto.waypoints) > 0

    def test_get_waypoint(self):
        seed = {'lat': 42, 'lng': -1, 'angle': 0}
        _id = _create_waypoint(seed)

        response = requests.post(BASE_URL + '/GetWaypoint', json={'waypoint': _id})
        assert response.status_code == 200
        response_body = response.json()
        assert response_body['lat'] == seed['lat']

    def test_get_waypoint_pb(self):
        seed = {'lat': 42, 'lng': -1, 'angle': 0}
        _id = _create_waypoint(seed)

        data = waypoint_pb2.GetWaypointRequest(waypoint=_id).SerializeToString()
        response = requests.post(
            BASE_URL + '/GetWaypoint', data=data, headers={'Content-Type': 'application/protobuf'},
        )
        assert response.status_code == 200
        waypoint_proto = waypoint_pb2.Waypoint()
        waypoint_proto.ParseFromString(response.content)
        assert waypoint_proto.lat == seed['lat']

    def test_create_waypoint(self):
        data = {'waypoint':  {'lat': 1, 'lng': -1, 'angle': 0}}
        response = requests.post(BASE_URL + '/CreateWaypoint', json=data)
        assert response.status_code == 200
        response_body = response.json()
        assert response_body['id']
        assert response_body['lat'] == data['waypoint']['lat']

    def test_create_waypoint_full(self):
        data = {'waypoint': {'lat': 1, 'lng': -1, 'angle': 0, 'delay': '3.000000001s', 'radius': 1}}
        response = requests.post(BASE_URL + '/CreateWaypoint', json=data)
        assert response.status_code == 200
        response_body = response.json()
        assert response_body['id']
        assert response_body['delay'] == data['waypoint']['delay']

    def test_create_waypoint_pb(self):
        request_proto = waypoint_pb2.CreateWaypointRequest(
            waypoint=waypoint_pb2.Waypoint(lat=42, lng=-1, angle=0),
        )
        data = request_proto.SerializeToString()
        response = requests.post(
            BASE_URL + '/CreateWaypoint', data=data, headers={'Content-Type': 'application/protobuf'},
        )
        assert response.status_code == 200

        response_proto = waypoint_pb2.Waypoint()
        response_proto.ParseFromString(response.content)
        assert response_proto.id.value
        assert response_proto.lat == request_proto.waypoint.lat

    def test_post_waypoint_invalid_lat(self):
        data = {
            'waypoint':  {
                'lat': -91,
                'lng': -1,
                'angle': 0,
            },
        }
        response = requests.post(BASE_URL + '/CreateWaypoint', json=data)
        assert response.status_code == 400
        assert 'lat' in response.text

    def test_post_waypoint_invalid_delay(self):
        data = {
            'waypoint':  {
                'lat': 1,
                'lng': -1,
                'angle': 0,
                'delay': '-0.000000001s',
            },
        }
        response = requests.post(BASE_URL + '/CreateWaypoint', json=data)
        assert response.status_code == 400
        assert 'delay' in response.text

    def test_create_waypoint_bad_request_lat_type_str(self):
        data = {
            'waypoint': {
                'lat': 'foo',
                'lng': -1,
                'angle': 0,
            },
        }
        response = requests.post(BASE_URL + '/CreateWaypoint', json=data)
        assert response.status_code == 500

    def test_create_waypoint_bad_request_lat_type_obj(self):
        data = {
            'waypoint':  {
                'lat': {'foo': 'bar'},
                'lng': -1,
                'angle': 0,
            },
        }
        response = requests.post(BASE_URL + '/CreateWaypoint', json=data)
        assert response.status_code == 500

    def test_delete_waypoint(self):
        _id = _create_waypoint({'lat': 42, 'lng': -1, 'angle': 0})

        response = requests.post(BASE_URL + '/DeleteWaypoint', json={'waypoint': _id})
        assert response.status_code == 200

    def test_delete_waypoint_pb(self):
        _id = _create_waypoint({'lat': 42, 'lng': -1, 'angle': 0})

        data = waypoint_pb2.DeleteWaypointRequest(waypoint=_id).SerializeToString()
        response = requests.post(
            BASE_URL + '/DeleteWaypoint', data=data, headers={'Content-Type': 'application/protobuf'},
        )
        assert response.status_code == 200
