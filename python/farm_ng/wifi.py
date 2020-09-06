# based on https://github.com/OpenAgricultureFoundation/python-wifi-connect
# which is based on https://github.com/balena-io/wifi-connect
import logging
import socket
import sys
import time
import uuid
from getpass import getpass

import NetworkManager

logger = logging.getLogger('network-manager')
logger.setLevel(logging.INFO)

ap_connection_id = 'farm_ng-' + socket.gethostname()
ap_SSID = ap_connection_id
ap_ip = '192.168.42.1'
wifi_interface = 'wlan0'


def get_ap_config(password):
    return {
        '802-11-wireless': {
            'mode': 'ap',
            'security': '801-22-wireless-security',
            'ssid': ap_SSID,
        },
        '802-11-wireless-security': {
            'key-mgmt': 'wpa-psk',
            'psk': password,
        },
        'connection': {
            'autoconnect': False,
            'id': ap_connection_id,
            'interface-name': wifi_interface,
            'type': '802-11-wireless',
            'uuid': str(uuid.uuid4()),
        },
        'ipv4': {
            'address-data': [{'address': ap_ip, 'prefix': 24}],
            'addresses': [[ap_ip, 24, ap_ip]],
            'gateway': ap_ip,
            'method': 'shared',
        },
        'ipv6': {'method': 'auto'},
    }


def find_connection(id):
    connections = NetworkManager.Settings.ListConnections()
    connections = {x.GetSettings()['connection']['id']: x for x in connections}
    return connections.get(id)


def get_ap_connection():
    connection = find_connection(ap_connection_id)

    if not connection:
        password = getpass(prompt='AP Password (at least 8 characters): ')
        NetworkManager.Settings.AddConnection(get_ap_config(password))
        logger.info(f'Added connection: {ap_connection_id}')
        connection = find_connection(ap_connection_id)

    if not connection:
        raise Exception('Could not get ap connection.')

    return connection


def activate_connection(connection):
    connection_id = connection.GetSettings()['connection']['id']
    device = next((d for d in NetworkManager.NetworkManager.GetDevices() if d.Interface == wifi_interface), None)
    if not device:
        raise Exception(f'No {wifi_interface} device found')

    NetworkManager.NetworkManager.ActivateConnection(connection, device, '/')
    logger.info(f'Activated connection={connection_id}, dev={device.Interface}.')

    i = 0
    while device.State != NetworkManager.NM_DEVICE_STATE_ACTIVATED:
        if i % 5 == 0:
            logger.info('Waiting for connection to become active...')
        if i > 30:
            break
        time.sleep(1)
        i += 1

    if device.State != NetworkManager.NM_DEVICE_STATE_ACTIVATED:
        raise Exception(f'Enabling connection {ap_connection_id} failed.')

    logger.info(f'Connection {connection_id} is active.')


def delete_connection(name):
    connection = find_connection(name)
    if not connection:
        raise Exception(f'The connection {name} does not exist.')
    connection.Delete()
    logger.info(f'Connection {name} deleted.')


def disable_current():
    current = next(
        (
            c for c in NetworkManager.NetworkManager.ActiveConnections if c.Connection.GetSettings()
            ['connection']['interface-name'] == wifi_interface
        ), None,
    )

    if current:
        logger.info(f"Deactivating connection {current.Connection.GetSettings()['connection']['id']}")
        NetworkManager.NetworkManager.DeactivateConnection(current)


def enable_ap():
    connection = get_ap_connection()
    activate_connection(connection)


def enable_connection(name):
    connection = find_connection(name)
    if not connection:
        raise Exception(f'The connection {name} does not exist.')
    activate_connection(connection)


def print_connections():
    connections = NetworkManager.Settings.ListConnections()
    for c in connections:
        print(c.GetSettings()['connection']['id'])


if __name__ == '__main__':
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)

    if len(sys.argv) < 2:
        logger.error("expects a command argument, e.g. 'list'")
        sys.exit(1)

    if sys.argv[1] == 'list':
        print_connections()
    elif sys.argv[1] == 'ap':
        enable_ap()
    elif sys.argv[1] == 'connect':
        if len(sys.argv) < 3:
            logger.error("'connect' expects an additional argument <SSID> e.g. 'connect homewifi'.")
            sys.exit(1)
        enable_connection(sys.argv[2])
    elif sys.argv[1] == 'delete':
        if len(sys.argv) < 3:
            logger.error("'delete' expects an additional argument <SSID> e.g. 'delete homewifi'.")
            sys.exit(1)
        delete_connection(sys.argv[2])

    else:
        logger.error('Unrecognized command: ' + sys.argv[1])
        sys.exit(1)
