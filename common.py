import os
import sys
from pymavlink import mavutil

# Force MAVLINK 2.0
os.environ["MAVLINK20"] = "1"


def print_usage(python_file):
    file = os.path.basename(sys.argv[0])
    print('Invalid number of arguments.\n\n'
          'Usage:\n'
          '$ python ' + file + ' CONNECTIONSTRING [BAUD]\n\n'
          'Examples:\n'
          '$ python ' + file + ' /dev/ttyUSB0 57600\n'
          '$ python ' + file + ' udpout:localhost:14540\n'
          'More examples at https://mavlink.io/en/mavgen_python/')
    exit()


def init_mavlink(mavlink_system_id,  # MAVLINK system id for the module. Id 1 is the same as the drone.
                 mavlink_component_id,  # MAVLINK component id for the module. Id 99 is a non reserved id.
                 connection_string, baud=None):
    the_connection = None

    if 'com' in connection_string or 'COM' in connection_string or '/dev/tty' in connection_string:
        if baud is not None:
            # Start a connection listening to a serial port
            the_connection = mavutil.mavlink_connection(connection_string,
                                                        baud=baud,
                                                        dialect='sheeprtt_ardupilotmega',
                                                        source_system=mavlink_system_id,
                                                        source_component=mavlink_component_id)
        else:
            print_usage("")
    else:
        # Start a connection listening to a connection string TODO: Test this more
        the_connection = mavutil.mavlink_connection(connection_string,
                                                    dialect='sheeprtt_ardupilotmega',
                                                    source_system=mavlink_system_id,
                                                    source_component=mavlink_component_id)

    print('Interface opened.')
    return the_connection


def send_heartbeat(the_connection, active=True,
                   heartbeat_type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                   heartbeat_autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID):
    base_mode = 0
    custom_mode = 0
    system_status = mavutil.mavlink.MAV_STATE_ACTIVE if active \
        else mavutil.mavlink.MAV_STATE_STANDBY

    the_connection.mav.heartbeat_send(
        type=heartbeat_type,
        autopilot=heartbeat_autopilot,
        base_mode=base_mode,
        custom_mode=custom_mode,
        system_status=system_status)


def wait_heartbeat(the_connection):
    print('Waiting for heartbeat...')
    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))


def is_autopilot_ardupilot(the_connection):
    return the_connection.messages['HEARTBEAT'].autopilot is mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
