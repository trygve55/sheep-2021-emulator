import sys
from pymavlink import mavutil
from time import time, sleep
from common import print_usage, init_mavlink, wait_heartbeat


def send_heartbeat(the_connection, active):
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
    gcs_addon_system = 255  # MAVLINK system id for the module. Id 1 is the same as the drone.
    gcs_addon_component = 98  # MAVLINK component id for the module. Id 98 is a non reserved id.
    the_connection = init_mavlink(gcs_addon_system, gcs_addon_component, connection_string, baud)

    wait_heartbeat(the_connection)

    while True:
        msg = the_connection.recv_match(blocking=False)

        if msg is None:
            sleep(0.02)
            continue

        if msg.name is 'SHEEP_RTT_DATA':
            print(msg)

            # TODO: Process sheepRTT data packet.

            # Send the sheepRTT ack packet directly.
            the_connection.mav.sheep_rtt_ack_send(msg.seq)

        elif msg.name is 'DATA32' and msg.type == 129 and msg.len == 31:
            msg = the_connection.mav.parse_char(msg.data[0:-1])  # Unpack encapsulated sheepRTT data.
            print(msg)

            # TODO: Process sheepRTT data packet.

            # Send the sheepRTT ack packet directly.
            the_connection.mav.sheep_rtt_ack_send(msg.seq)

            # Pack sheepRTT ack packet inside a data16 packet and send it. With zero padding.
            sheep_rtt_ack_packet = the_connection.mav.sheep_rtt_ack_encode(msg.seq).pack(the_connection.mav) + b'\x00\x00\x00'
            the_connection.mav.data16_send(130, 13, sheep_rtt_ack_packet)
        else:
            pass
            # print(msg)
