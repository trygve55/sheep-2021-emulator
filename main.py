import sys
from pymavlink import mavutil
from time import time, sleep
from common import print_usage, init_mavlink, wait_heartbeat


def send_heartbeat(the_connection, active=True):
    base_mode = 0
    custom_mode = 0
    system_status = mavutil.mavlink.MAV_STATE_ACTIVE if active \
        else mavutil.mavlink.MAV_STATE_STANDBY

    the_connection.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        base_mode=base_mode,
        custom_mode=custom_mode,
        system_status=system_status)


if __name__ == '__main__':
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print_usage("")

    connection_string = sys.argv[1]
    baud = sys.argv[2] if len(sys.argv) == 3 else None
    nrf52833_system = 1  # MAVLINK system id for the module. Id 1 is the same as the drone.
    nrf52833_component = 99  # MAVLINK component id for the module. Id 99 is a non reserved id.
    the_connection = init_mavlink(nrf52833_system, nrf52833_component, connection_string, baud)

    wait_heartbeat(the_connection)

    # Request gps data drone drone
    gps_update_frequency = 1  # How often to request gps information in hertz.

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
            send_heartbeat(the_connection)

            # Send the sheepRTT data packet directly.
            the_connection.mav.sheep_rtt_data_send(0, 1337, 1337, 420, 69, 12)

            # Pack sheepRTT data packet inside a data32 packet and send it. With zero padding.
            sheep_rtt_data_packet = the_connection.mav.sheep_rtt_data_encode(0, 1337, 1337, 420, 69, 12).pack(the_connection.mav) + b'\x00'
            the_connection.mav.data32_send(129, 31, sheep_rtt_data_packet)

            '''
            # Send the sheepRTT ack packet directly.
            the_connection.mav.sheep_rtt_ack_send(int(0))

            # Pack sheepRTT ack packet inside a data16 packet and send it. With zero padding.
            sheep_rtt_ack_packet = the_connection.mav.sheep_rtt_ack_encode(0).pack(the_connection.mav) + b'\x00\x00\x00'
            the_connection.mav.data16_send(130, 13, sheep_rtt_ack_packet)
            '''

        msg = the_connection.recv_msg()

        if msg is None or msg.get_type() is 'BAD_DATA':
            sleep(0.02)
            continue

        if msg.name is 'GLOBAL_POSITION_INT':
            print(msg)
        elif msg.name is 'SHEEP_RTT_ACK':
            print(msg)

            # TODO: Process sheepRTT ack.
        elif msg.name is 'DATA16' and msg.type == 130 and msg.len == 13:
            msg = the_connection.mav.parse_char(msg.data[0:-1])  # Unpack encapsulated sheepRTT data.
            print(msg)

            # TODO: Process sheepRTT ack.
        else:
            print(msg)
