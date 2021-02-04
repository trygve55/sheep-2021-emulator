import os
import sys
from pymavlink import mavutil
from time import time, sleep

# Force MAVLINK 2.0
os.environ["MAVLINK20"] = "1"


def print_usage():
    print('Invalid number of arguments.\n\n'
          'Usage:\n'
          '$ python main.py CONNECTIONSTRING [BAUD]\n\n'
          'Examples:\n'
          '$ python main.py /dev/ttyUSB0 57600\n'
          '$ python main.py udpout:localhost:14540\n'
          'More examples at https://mavlink.io/en/mavgen_python/')
    exit()


if __name__ == '__main__':
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print_usage()

    connection_string = sys.argv[1]
    nrf52833_system = 1 #MAVLINK system id for the module. Id 1 is the same as the drone.
    nrf52833_component = 99 #MAVLINK component id for the module. Id 99 is a non reserved id.
    the_connection = None

    if 'com' in connection_string or 'COM' in connection_string or '/dev/tty' in connection_string:
        if len(sys.argv) == 3:
            baud = sys.argv[2]
            # Start a connection listening to a serial port
            the_connection = mavutil.mavlink_connection(connection_string, baud=baud, dialect='sheeprtt_ardupilotmega', source_system=nrf52833_system, source_component=nrf52833_component)
        else:
            print_usage()
    else:
        # Start a connection listening to a connection string TODO: Test this more
        the_connection = mavutil.mavlink_connection(connection_string, dialect='sheeprtt_ardupilotmega', source_system=nrf52833_system, source_component=nrf52833_component)

    def send_heartbeat():
        base_mode = 0
        custom_mode = 0
        system_status = mavutil.mavlink.MAV_STATE_ACTIVE if True \
            else mavutil.mavlink.MAV_STATE_STANDBY

        the_connection.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=base_mode,
            custom_mode=custom_mode,
            system_status=system_status)

    print('Interface opened.')

    print('Waiting for heartbeat...')
    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

    # Request gps data drone drone
    gps_update_frequency = 1 # How often to request gps information in hertz.

    the_connection.mav.request_data_stream_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        gps_update_frequency, 1)

    drone_gps = None

    last_heartbeat = 0

    while True:
        if time() > last_heartbeat + 1:
            last_heartbeat = time()
            send_heartbeat()

            # Send the sheepRTT data packet directly.
            the_connection.mav.sheep_rtt_data_send(0, 1337, 1337, 420, 69, 12)

            # Pack sheepRTT data packet inside a data32 packet and send it. With zero padding.
            sheep_rtt_data_packet = the_connection.mav.sheep_rtt_data_encode(0, 1337, 1337, 420, 69, 12).pack(the_connection.mav) + b'\x00'
            the_connection.mav.data32_send(129, 31, sheep_rtt_data_packet)

            # Send the sheepRTT ack packet directly.
            the_connection.mav.sheep_rtt_ack_send(int(0))

            # Pack sheepRTT ack packet inside a data16 packet and send it. With zero padding.
            sheep_rtt_ack_packet = the_connection.mav.sheep_rtt_ack_encode(0).pack(the_connection.mav) + b'\x00\x00\x00'
            the_connection.mav.data16_send(130, 13, sheep_rtt_ack_packet)

        msg = the_connection.recv_msg()

        if msg is None or msg.get_type() is 'BAD_DATA':
            sleep(0.02)
            continue

        if msg.name is 'GLOBAL_POSITION_INT':
            print(msg)
        elif msg.name is 'DATA16' and msg.type == 130 and msg.len == 13:
            msg = the_connection.mav.parse_char(msg.data[0:-1])
            print(msg)
        else:
            print(msg)
