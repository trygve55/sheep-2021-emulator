import sys
from pymavlink import mavutil
from time import time, sleep
from common import print_usage, init_mavlink, wait_heartbeat
import matplotlib.pyplot as plt
import matplotlib._color_data as mcd
import math


class Sample:
    def __init__(self, sheep_id, seq, lat, lon, alt, dist):
        self.sheep_id = sheep_id
        self.seq = seq
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.dist = dist


class SamplePlotter:
    def __init__(self):
        # get the figure and axes objects, pyplot take care of the cavas creation
        fig, ax = plt.subplots()  # <- change this line to get your axes object differently
        # get a line artist, the comma matters
        ln, = ax.plot([], [])
        # set the axes labels
        ax.set_title('title')
        ax.set_xlabel('xlabel')
        ax.set_ylabel('ylabel')
        ax.set_aspect('equal', adjustable='datalim')

        self.fig = fig
        self.ax = ax
        self.samples = []
        self.last_update = 0
        self.color_map = list(mcd.XKCD_COLORS.values())

    def add_sample(self, sample):
        if not any(s.seq == sample.seq for s in self.samples):
            self.samples.append(sample)
            if time() > self.last_update + 2:
                self.last_update = time()
                self.plot_samples()

    def plot_samples(self):
        plt.cla()
        self.ax.clear()
        long_lat = 89.83 * math.cos(math.radians(self.samples[0].lat / 10e6))
        for sample in self.samples:
            c = plt.Circle((sample.lon/long_lat, sample.lat/89.83), sample.dist*5, alpha=0.08)
            self.ax.add_patch(c)
            c.set_edgecolor(self.color_map[sample.sheep_id])
            c.set_facecolor("none")
        for sample in self.samples:
            self.ax.add_patch(plt.Circle((sample.lon/long_lat, sample.lat/89.83), 1.5, color='b', alpha=0.5))

        self.ax.plot()  # Causes an autoscale update.
        plt.pause(0.001)


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

    # Wait for heartbeat from drone before continuing
    wait_heartbeat(the_connection)

    # Start system for visualizing samples
    sample_plotter = SamplePlotter()

    # Start parameter related testing
    the_connection.mav.param_request_read_send(1, 99, str.encode('param1_int'), -1)  # Test get parameter by id
    the_connection.mav.param_request_read_send(1, 99, str.encode(''), 1)  # Test get parameter by index
    the_connection.mav.param_request_list_send(1, 99)  # Test get all parameter
    the_connection.mav.param_set_send(1, 99, str.encode('param1_int'), 2, mavutil.mavlink.MAV_PARAM_TYPE_INT32)  # Test set a single parameter
    # End parameter related testing

    while True:
        msg = the_connection.recv_match(blocking=False)

        # Sleep when not receiving any messages to save CPU cycles.
        if msg is None:
            sleep(0.02)
            continue

        # Ignore non recognised messages
        if msg.get_type() == 'BAD_DATA':
            continue
        # SheepRTT data message containing a sample
        elif msg.name == 'SHEEP_RTT_DATA':
            print(msg)

            # TODO: Process sheepRTT data packet.

            # Send the sheepRTT ack packet directly.
            the_connection.mav.sheep_rtt_ack_send(msg.seq)
        # Encapsulated SheepRTT data message containing a sample
        elif msg.name == 'DATA32' and msg.type == 129 and msg.len == 31:
            msg = the_connection.mav.parse_char(msg.data[0:-1])  # Unpack encapsulated sheepRTT data.
            print(msg)

            # TODO: Process sheepRTT data packet.

            # Send the sheepRTT ack packet directly.
            the_connection.mav.sheep_rtt_ack_send(msg.seq)

            # Pack sheepRTT ack packet inside a data16 packet and send it. With zero padding.
            sheep_rtt_ack_packet = the_connection.mav.sheep_rtt_ack_encode(msg.seq).pack(the_connection.mav) + b'\x00\x00\x00'
            the_connection.mav.data16_send(130, 13, sheep_rtt_ack_packet)

            sample_plotter.add_sample(Sample(msg.tid, msg.seq, msg.lat, msg.lon, msg.alt, msg.dis))
        # Parameter related messages
        elif msg.name in ['PARAM_REQUEST_READ', 'PARAM_REQUEST_LIST', 'PARAM_VALUE', 'PARAM_SET']:
            print(msg)
        # Other messages
        else:
            pass
            # print(msg)
