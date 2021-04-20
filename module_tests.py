import sys
import math
from pymavlink import mavutil
from time import time, sleep
from common import print_usage, init_mavlink
import threading


def send_heartbeat(the_connection):
    base_mode = mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + \
                mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED + \
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    custom_mode = 0
    system_status = mavutil.mavlink.MAV_STATE_ACTIVE if drone['active'] \
        else mavutil.mavlink.MAV_STATE_STANDBY

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


def heartbeat_thread(connection, interval):
    while True:
        send_heartbeat(connection)
        sleep(interval)


def global_position_int_thread(connection):
    while True:
        send_data_stream_position(connection)
        sleep(1 / drone['request_data_stream_position']['req_message_rate'])


if __name__ == '__main__':
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print_usage("")

    connection_string = sys.argv[1]
    baud = sys.argv[2] if len(sys.argv) == 3 else None
    drone_system = 1  # MAVLINK system id for the drone. Id 1 is the same as the drone.
    drone_component = 1  # MAVLINK component id for the drone. Id 1 is the autopilot.
    the_connection_drone = init_mavlink(drone_system, drone_component, connection_string, baud)

    gcs_system = 255  # MAVLINK system id for the GCS. Id 255 is the usual for GCSs.
    gcs_component = 190  # MAVLINK component id for the GCS. Id 190 is used for the GCS.
    the_connection_gcs = init_mavlink(gcs_system, gcs_component, 'udpout:localhost:12345', baud)

    module_system = 1  # MAVLINK system id for the module. Id 1 is the same as the drone.
    module_component = 99  # MAVLINK component id for the module. Id 99 is for private user defined use.

    # Defining drone information
    drone = {
        'lat': 63.1,  # degrees
        'lon': 12.0,  # degrees
        'alt': 50,  # m
        'vx': 20,  # cm/s
        'vy': 95,  # cm/s
        'vz': 0,  # cm/s
        'active': False,
        'request_data_stream_position': {
            'request_system': None,  # What system to send gps information.
            'request_component': None,  # What component to send gps information.
            'req_message_rate': None  # How often to send gps information in hertz.
        }
    }

    last_sheep_rtt_seq = -1
    test_timeout = 5  # 5 sec timeout

    # Testing sending heartbeat
    print('\n\n2. ##############################################################################')
    print('Sending heartbeat to module.')
    send_heartbeat(the_connection_drone)
    print('Waiting for heartbeat from module...')
    msg = the_connection_drone.recv_match(type='HEARTBEAT', blocking=True, timeout=test_timeout)
    if msg is None:
        print('NOT OK! No heartbeat received from module.')
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    print('OK! Received heartbeat from module.')

    # Starting heartbeat thread
    print('\n\n!. ##############################################################################')
    print('Starting heartbeat thread. Interval: 1s')
    t_heartbeat = threading.Thread(target=heartbeat_thread, args=(the_connection_drone, 1.0), daemon=True)
    t_heartbeat.start()

    # Checking if module requests 'DATA_STREAM_POSITION'
    print('\n\n3. ##############################################################################')
    print("Checking if module requests 'DATA_STREAM_POSITION' with 'MAV_DATA_STREAM_POSITION")

    msg = the_connection_drone.recv_match(type='REQUEST_DATA_STREAM', blocking=True, timeout=test_timeout)
    if msg is None:
        print('NOT OK! No \'REQUEST_DATA_STREAM\' with \'MAV_DATA_STREAM_POSITION\'received from module.')
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    elif msg.req_stream_id == mavutil.mavlink.MAV_DATA_STREAM_POSITION and \
            msg.target_system == drone_system and \
            msg.target_component in (drone_component, 0) and \
            msg.start_stop == 1:
        print('OK! Received correct \'REQUEST_DATA_STREAM\' from module.')

        drone['request_data_stream_position'] = {
            'request_system': the_connection_drone.target_system,  # What system to send gps information.
            'request_component': the_connection_drone.target_component,  # What component to send gps information.
            'req_message_rate': msg.req_message_rate  # How often to send gps information in hertz.
        }
    else:
        print('NOT OK! Received \'REQUEST_DATA_STREAM\' is not correct. Received:')
        print(msg)
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()

    # Starting global_position_int thread.
    print('\n\n!. ##############################################################################')
    print('Starting global_position_int thread. Target system: {}, Target component: {}, interval: {}'.format(
        drone['request_data_stream_position']['request_system'],
        drone['request_data_stream_position']['request_component'],
        drone['request_data_stream_position']['req_message_rate']))
    t_global_position_int = threading.Thread(target=global_position_int_thread, args=(the_connection_drone,), daemon=True)
    t_global_position_int.start()

    # Checking if module sends 'SHEEP_RTT_DATA'.
    print('\n\n4. ##############################################################################')
    print("Checking if module sends 'SHEEP_RTT_DATA' or encapsulated 'SHEEP_RTT_DATA'.")

    msg = the_connection_drone.recv_match(type=['SHEEP_RTT_DATA', 'DATA64'], blocking=True, timeout=test_timeout*5)
    if msg is None:
        print('NOT OK! No SHEEP_RTT_DATA or encapsulated SHEEP_RTT_DATA received.')
        the_connection_drone.close()
        the_connection_drone.close()
        exit()
    elif msg.name == 'SHEEP_RTT_DATA':
        last_sheep_rtt_seq = msg.seq
        print('OK! SHEEP_RTT_DATA received.')

        # Send the sheepRTT ack packet directly.
        msg_ack = the_connection_gcs.mav.sheep_rtt_ack_encode(msg.seq)
        the_connection_drone.mav.send(msg_ack)
        print('Sending SHEEP_RTT_ACK to module.')
    elif msg.name == 'DATA64' and msg.type == 129:
        msg = the_connection_gcs.mav.parse_char(msg.data[0:msg.len])  # Unpack encapsulated sheepRTT data.
        last_sheep_rtt_seq = msg.seq
        print(msg)

        print('OK! Encapsulated SHEEP_RTT_DATA received.')

        # Pack sheepRTT ack packet inside a data16 packet and send it. With zero padding.
        sheep_rtt_ack_packet = the_connection_gcs.mav.sheep_rtt_ack_encode(msg.seq).pack(the_connection_gcs.mav) + b'\x00\x00\x00'

        msg_ack = the_connection_gcs.mav.data16_encode(130, len(sheep_rtt_ack_packet) - 3, sheep_rtt_ack_packet)
        the_connection_drone.mav.send(msg_ack)
        print('Sending encapsulated SHEEP_RTT_ACK to module.')

    # Checking if module sends encapsulated 'SHEEP_RTT_DATA'.
    print('\n\n6. ##############################################################################')
    print("Checking if module increments 'SHEEP_RTT_DATA' seq after ack.")

    msg = the_connection_drone.recv_match(type=['SHEEP_RTT_DATA', 'DATA64'], blocking=True, timeout=test_timeout*5)
    if msg is None:
        print('NOT OK! No encapsulated SHEEP_RTT_DATA received.')
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    elif msg.name == 'SHEEP_RTT_DATA':
        print('SHEEP_RTT_DATA received.')
    elif msg.name == 'DATA64' and msg.type == 129:
        msg = the_connection_gcs.mav.parse_char(msg.data[0:msg.len])  # Unpack encapsulated sheepRTT data.
        print('Encapsulated SHEEP_RTT_DATA received.')

    if msg.seq == last_sheep_rtt_seq + 1:
        print('OK! SHEEP_RTT_DATA seq incremented.')
    else:
        print('NOT OK! SHEEP_RTT_DATA seq incremented. Expected: {}, received: {}'.format(last_sheep_rtt_seq, msg.seq))
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()

    # Getting module parameter by id.
    print('\n\n6. ##############################################################################')
    print("Getting module parameter by id")
    # Start parameter related testing
    the_connection_drone.mav.param_request_read_send(1, 99, str.encode('param1_int'), -1)  # Test get parameter by id
    msg = the_connection_drone.recv_match(type='PARAM_VALUE', blocking=True, timeout=test_timeout)
    if msg is None:
        print('NOT OK! PARAM_VALUE not received.')
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    elif msg.param_id != 'param1_int' or \
        msg.param_value != 1.0 or \
        msg.param_type != mavutil.mavlink.MAV_PARAM_TYPE_INT32 or \
        msg.param_count != 2 or \
        msg.param_index != 0:
        print('NOT OK! PARAM_VALUE contains wrong values. Expected:\n '
              'PARAM_VALUE {param_id : param1_int, param_value : 1.0, param_type : 6, param_count : 2, param_index : 0}\n'
              'Received:\n', msg)
        #print(msg)
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    print('OK! Correct PARAM_VALUE received.')

    # Getting module parameter by index.
    print('\n\n7. ##############################################################################')
    print("Getting module parameter by index")

    the_connection_drone.mav.param_request_read_send(1, 99, str.encode(''), 1)  # Test get parameter by index
    msg = the_connection_drone.recv_match(type='PARAM_VALUE', blocking=True, timeout=test_timeout)
    if msg is None:
        print('NOT OK! PARAM_VALUE not received.')
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    elif msg.param_id != 'param2_float' or \
            msg.param_value != 1.100000023841858 or \
            msg.param_type != mavutil.mavlink.MAV_PARAM_TYPE_REAL32 or \
            msg.param_count != 2 or \
            msg.param_index != 1:
        print('NOT OK! PARAM_VALUE contains wrong values. Expected:\n '
              'PARAM_VALUE {param_id : param2_float, param_value : 1.100000023841858, param_type : 9, param_count : 2, param_index : 1}\n'
              'Received:\n', msg)

        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    print('OK! Correct PARAM_VALUE received.')

    # Getting all module parameters.
    print('\n\n8. ##############################################################################')
    print("Getting all module parameters.")
    the_connection_drone.mav.param_request_list_send(1, 99)  # Test get all parameter
    msg = the_connection_drone.recv_match(type='PARAM_VALUE', blocking=True, timeout=test_timeout)
    if msg is None:
        print('NOT OK! PARAM_VALUE not received.')
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    else:
        values_received = 1
        values_count = msg.param_count

        for i in range(values_received, values_count):
            msg = the_connection_drone.recv_match(type='PARAM_VALUE', blocking=True, timeout=test_timeout)
            values_received += 1
            sleep(0.2)

        if values_received != values_count:
            print('NOT OK! Received {} of {} values.'.format(values_received, values_count))
            the_connection_drone.close()
            the_connection_gcs.close()
            exit()

    print('OK! All PARAM_VALUEs received.')

    # Setting module parameter.
    print('\n\n9 .##############################################################################')
    print("Setting module parameter")
    the_connection_drone.mav.param_set_send(1, 99, str.encode('param1_int'), 2, mavutil.mavlink.MAV_PARAM_TYPE_INT32)  # Test set a single parameter
    msg = the_connection_drone.recv_match(type='PARAM_VALUE', blocking=True, timeout=test_timeout)
    if msg is None:
        print('NOT OK! PARAM_VALUE not received.')
        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    elif msg.param_id != 'param1_int' or \
            msg.param_value != 2 or \
            msg.param_type != mavutil.mavlink.MAV_PARAM_TYPE_INT32 or \
            msg.param_count != 2 or \
            msg.param_index != 0:
        print('NOT OK! PARAM_VALUE contains wrong values. Expected:\n '
              'PARAM_VALUE {param_id : param1_int, param_value : 1.0, param_type : 6, param_count : 2, param_index : 0}\n'
              'Received:\n', msg)

        the_connection_drone.close()
        the_connection_gcs.close()
        exit()
    print('OK! Correct value set and PARAM_VALUE received.')
