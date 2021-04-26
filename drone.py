import sys
import math
from pymavlink import mavutil
from time import time, sleep
from common import print_usage, init_mavlink


def send_heartbeat(the_connection):
    base_mode = mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + \
                mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED + \
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    custom_mode = 0
    system_status = mavutil.mavlink.MAV_STATE_ACTIVE #if drone['active'] \
        #else mavutil.mavlink.MAV_STATE_STANDBY

    the_connection.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=base_mode,
        custom_mode=custom_mode,
        system_status=system_status)


def send_data_stream_position(the_connection):
    the_connection.mav.global_position_int_send(
        time_boot_ms=0,
        lat=int(drone['lat'] * 10**7),  # Converts degrees into degreesE7
        lon=int(drone['lon'] * 10**7),  # Converts degrees into degreesE7
        alt=int(drone['alt'] * 1000),  # Converts m into mm
        relative_alt=int(drone['alt'] * 1000),  # Converts m into mm
        vx=drone['vx'],
        vy=drone['vy'],
        vz=drone['vz'],
        hdg=65535)


def move_drone(drone, time_delta):
    drone['lat'] += time_delta * drone['vx'] / 100 / 111320  # Convert cm/s into degrees
    drone['lon'] += time_delta * drone['vy'] / 100 / (40075000 * math.cos(drone['lat'] * math.pi / 180) / 360)  # Convert cm/s into degrees
    drone['alt'] += time_delta * drone['vz'] / 100  # Convert cm/s into m


if __name__ == '__main__':
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print_usage("")

    connection_string = sys.argv[1]
    baud = sys.argv[2] if len(sys.argv) == 3 else None
    drone_system = 1  # MAVLINK system id for the drone. Id 1 is the same as the drone.
    drone_component = 1  # MAVLINK component id for the drone. Id 1 is the autopilot.
    the_connection = init_mavlink(drone_system, drone_component, connection_string, baud)

    # Defining drone information
    drone = {
        'lat': 63.1,  # degrees
        'lon': 12.0,  # degrees
        'alt': 50,  # m
        'vx': 20,  # cm/s
        'vy': 95,  # cm/s
        'vz': 0,  # cm/s
        'active': True,
        'request_data_stream_position': {
            'request_system': None,  # What system to send gps information.
            'request_component': None,  # What component to send gps information.
            'req_message_rate': None  # How often to send gps information in hertz.
        }
    }

    last_heartbeat = 0
    last_move = time()
    last_send_position = 0

    send_heartbeat(the_connection)

    while True:
        new_move = time()
        move_drone(drone, time_delta=new_move - last_move)
        last_move = new_move

        if time() > last_heartbeat + 1.03:
            last_heartbeat = time()
            send_heartbeat(the_connection)

        if time() > last_send_position + 0.3:
            last_send_position = time()
            send_data_stream_position(the_connection)

        msg = the_connection.recv_msg()

        if msg is None:
            sleep(0.02)
            continue

        # Ignore non recognised messages
        if msg.get_type() == 'BAD_DATA':
            continue

        if msg.name == 'REQUEST_DATA_STREAM' and \
                msg.req_stream_id == mavutil.mavlink.MAV_DATA_STREAM_POSITION and \
                msg.target_system == drone_system and \
                msg.target_component in (drone_component, 0):

            print(msg)
            if msg.start_stop == 1:  # Request to start sending the gps data
                drone['request_data_stream_position'] = {
                    'request_system': the_connection.target_system,  # What system to send gps information.
                    'request_component': the_connection.target_component,  # What component to send gps information.
                    'req_message_rate': msg.req_message_rate  # How often to send gps information in hertz.
                }
            else:  # Request to stop the gps data
                drone['request_data_stream_position'] = {
                    'request_system': None,  # What system to send gps information.
                    'request_component': None,  # What component to send gps information.
                    'req_message_rate': None  # How often to send gps information in hertz.
                }
        else:
            print(msg)
