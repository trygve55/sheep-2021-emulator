import sys
from pymavlink import mavutil


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
            the_connection = mavutil.mavlink_connection(connection_string, baud=baud, source_system=nrf52833_system, source_component=nrf52833_component)
        else:
            print_usage()
    else:
        # Start a connection listening to a connection string TODO: Test this more
        the_connection = mavutil.mavlink_connection(connection_string, source_system=nrf52833_system, source_component=nrf52833_component)

    print('Interface opened.')
    print('Waiting for heartbeat...')
    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()

    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

    # Request gps data drome drone
    gps_update_frequecy = 1 # How often to request gps information in hertz.

    the_connection.mav.request_data_stream_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        gps_update_frequecy, 1)

    # Test sending some data
    the_connection.mav.system_time_send(190048103001, 0)
    the_connection.mav.data16_send(169, 3, b'aaaabbbbccccdddd')

    drone_gps = None

    while True:
        msg = the_connection.recv_match(blocking=True)
        if msg.name is 'GLOBAL_POSITION_INT':
            print(msg)
    #the_connection.post_message()
