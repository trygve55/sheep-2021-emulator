import sys
from pymavlink import mavutil
from time import time, sleep
from common import print_usage, init_mavlink, wait_heartbeat, is_autopilot_ardupilot
import random
from math import sqrt, fabs, cos, radians
from collections import OrderedDict


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


class Params:
    class Param:
        def __init__(self, param_id, value, param_type, index):
            self.param_id = str.encode(param_id)
            self.value = value
            self.param_type = param_type
            self.index = index

    def __init__(self):
        self.parameters = OrderedDict()
        self.parameters_type = OrderedDict()
        self.add_param('param1_int', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.add_param('param2_float', 1.1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def add_param(self, param_id, value, param_type):
        self.parameters[param_id] = value
        self.parameters_type[param_id] = param_type

    def set_by_id(self, param_id, value, param_type):
        if param_id not in self.parameters or self.parameters_type[param_id] != param_type:
            return None

        self.parameters[param_id] = value
        return self.get_by_id(param_id)

    def get_param_count(self):
        return len(self.parameters)

    def get_by_index(self, index):
        if index >= len(self.parameters):
            return None

        param_id = list(self.parameters.keys())[index]

        return self.Param(param_id, self.parameters[param_id], self.parameters_type[param_id], index)

    def get_by_id(self, param_id):
        if param_id not in self.parameters:
            return None

        return self.Param(param_id, self.parameters[param_id], self.parameters_type[param_id], list(self.parameters.keys()).index(param_id))


class SheepRTTEmulator:
    class Sample:
        def __init__(self, sheep_id, seq, lat, lon, alt, dist):
            self.sheep_id = sheep_id
            self.seq = seq
            self.lat = lat
            self.lon = lon
            self.alt = alt
            self.dist = dist

    class Sheep:
        def __init__(self, sheep_id, lat, lon, alt):
            self.sheep_id = sheep_id
            self.lat = lat
            self.lon = lon
            self.alt = alt

    def __init__(self, max_ping_range=250, max_gen_dist=1000, quantity=25, seed=None):
        self.max_ping_range = max_ping_range
        self.max_gen_dist = max_gen_dist
        self.quantity = quantity
        self.seed = seed

        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.simulated_sheep = []
        self.samples = []
        self.ping_seq = 0
        self.send_seq = 0
        self.init_complete = False
        self.long_lat = None

    def late_init(self):
        self.generate_sheep(max_gen_dist=self.max_gen_dist, quantity=self.quantity)
        self.long_lat = 89.83 * cos(radians(self.lat / 10e6))
        self.init_complete = True

    def update_position_from_msg(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

        # Check if late_init is not complete and valid gps signal is received.
        if not self.init_complete and not (self.lat == 0 and self.lon == 0 and self.alt == 0):
            self.late_init()

    # Does not work well near the zero-median!
    def generate_sheep(self, max_gen_dist=1000, quantity=25, seed=None):  # default max_dist is about 1km
        max_gen_dist *= 90  # Approximately convert from meters to degE7.

        random.seed(self.seed)

        for i in range(quantity):
            self.simulated_sheep.append(self.Sheep(
                sheep_id=i,
                lat=self.lat + random.randrange(-max_gen_dist, max_gen_dist),
                lon=self.lon + random.randrange(-max_gen_dist, max_gen_dist),
                alt=self.alt
            ))
            # print(self.simulated_sheep[-1])

    def ping_sheep(self):
        #max_ping_range = self.max_ping_range * 90  # Approximately convert from meters to degE7.

        new_samples = []
        for sheep in self.simulated_sheep:
            dist = sqrt(fabs((sheep.lat - self.lat)/89.83)**2 + fabs((sheep.lon - self.lon)/self.long_lat)**2) # + fabs((sheep.alt - self.alt) * 1000)**2)
            # print(dist / 90.0, dist <= max_range)
            if dist <= self.max_ping_range:
                new_samples.append(self.Sample(sheep.sheep_id, self.ping_seq, self.lat, self.lon, self.alt, int(dist/5 + 2.5)))

        if len(new_samples) == 0:
            return False

        self.samples.append(random.choice(new_samples))
        self.ping_seq += 1
        return True

    def is_more_samples(self):
        return self.ping_seq > self.send_seq

    def send_next_sample(self):
        sample = self.samples[self.send_seq]
        return self.send_seq, sample.lat, sample.lon, sample.alt, sample.dist, sample.sheep_id

    def receive_ack(self, seq):
        if seq == self.send_seq:
            self.send_seq += 1


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

    if is_autopilot_ardupilot(the_connection):
        the_connection.mav.request_data_stream_send(
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            gps_update_frequency, 1)
    else:
        # TODO: Fix and test this.
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            gps_update_frequency, 1)

    # Object used to keep track of parameters.
    parameters = Params()
    # Start simulator for SheepRTT pinging of sheep.
    sheep_rtt_emulator = SheepRTTEmulator(max_ping_range=250, max_gen_dist=1000, quantity=25, seed=None)

    # Sends a sheepRTT message if not all have been received by GCS. Toggleable encapsulation.
    def send_sample_if_possible(encapsulation=False):
        if sheep_rtt_emulator.is_more_samples():
            seq, lat, lon, alt, dist, sheep_id = sheep_rtt_emulator.send_next_sample()

            if not encapsulation:
                # Send the sheepRTT data packet directly.
                the_connection.mav.sheep_rtt_data_send(seq, lat, lon, alt, dist, sheep_id)

                print('Sent sheep_rtt_data with seq:' + str(seq))
            else:
                # Pack sheepRTT data packet inside a data32 packet and send it. With zero padding.
                sheep_rtt_data_packet = the_connection.mav.sheep_rtt_data_encode(seq, lat, lon, alt, dist, sheep_id).pack(
                    the_connection.mav) + b'\x00'
                the_connection.mav.data32_send(129, 31, sheep_rtt_data_packet)

                print('Sent encapsulated sheep_rtt_data with seq:' + str(seq))

    # Used to keep the time when the last heartbeat was sent.
    last_heartbeat_sent = 0

    while True:
        if time() > last_heartbeat_sent + 1:
            last_heartbeat_sent = time()
            send_heartbeat(the_connection)

            '''
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
            '''

            send_sample_if_possible(encapsulation=True)

            sheep_rtt_emulator.ping_sheep()

        msg = the_connection.recv_msg()

        # Sleep when not receiving any messages to save CPU cycles.
        if msg is None:
            sleep(0.02)
            continue

        # Ignore non recognised messages
        if msg.get_type() == 'BAD_DATA':
            continue
        # GPS position message, used to update internally stored position
        elif msg.name == 'GLOBAL_POSITION_INT':
            # print(msg)
            if msg.lat != 0 and msg.lon != 0 and msg.alt != 0:
                sheep_rtt_emulator.update_position_from_msg(msg)
        # SheepRTT acknowledgment message
        elif msg.name == 'SHEEP_RTT_ACK':
            print(msg)

            # Process sheepRTT ack.
            sheep_rtt_emulator.receive_ack(msg.seq)
            send_sample_if_possible(encapsulation=True)
        # Encapsulated SheepRTT acknowledgment message
        elif msg.name == 'DATA16' and msg.type == 130 and msg.len == 13:
            msg = the_connection.mav.parse_char(msg.data[0:-1])  # Unpack encapsulated sheepRTT data.
            print(msg)

            # Process sheepRTT ack.
            sheep_rtt_emulator.receive_ack(msg.seq)
            send_sample_if_possible(encapsulation=True)
        # Request to read a single parameter
        elif msg.name == 'PARAM_REQUEST_READ' and msg.target_system == nrf52833_system and msg.target_component == nrf52833_component:
            print(msg)

            param = None
            if msg.param_index == -1:
                param = parameters.get_by_id(msg.param_id)
            else:
                param = parameters.get_by_index(msg.param_index)

            if param is None:
                print("Error not found processing:", msg)
                continue

            the_connection.mav.param_value_send(param.param_id, param.value, param.param_type, parameters.get_param_count(), param.index)
        # Request to read all parameters
        elif msg.name == 'PARAM_REQUEST_LIST' and msg.target_system == nrf52833_system and msg.target_component == nrf52833_component:
            print(msg)

            for i in range(parameters.get_param_count()):
                param = parameters.get_by_index(i)
                the_connection.mav.param_value_send(param.param_id, param.value, param.param_type, parameters.get_param_count(), param.index)
                # Should have a small pause here if many parameters
        # Request to set a single parameter
        elif msg.name == 'PARAM_SET' and msg.target_system == nrf52833_system and msg.target_component == nrf52833_component:
            print(msg)

            param = parameters.set_by_id(msg.param_id, msg.param_value, msg.param_type)

            if param is None:
                print("Error not found processing:", msg)
                continue

            # Reply with a PARAM_VALUE message containing new values to confirm set.
            the_connection.mav.param_value_send(param.param_id, param.value, param.param_type, parameters.get_param_count(), param.index)
        # Other messages
        else:
            pass
            # print(msg)
