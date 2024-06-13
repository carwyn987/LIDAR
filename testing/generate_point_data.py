"""
Run with:

~/development/venvs/p39/bin/python testing/generate_point_data.py data/throw.pcap



FORMAT OF THE DATA

Header:
 - 42 bytes of header == 84 hexidecimal numbers

12 Data Blocks:
 - Flag xffee
 - (4 hexidecimal numbers = 2 bytes) of azimuth (reverse order, 100th of a degree)
 - 16 channels of data (?)
  - 32 sets of 2-byte distances + 1-byte reflectivity (= 32 sets of 3 bytes = 96 bytes = 32 sets of 6 hexidecimal numbers = 192 hexidecimal numbers)

  
"
9.3.1.1 Firing Sequence
    A firing sequence occurs when all the lasers in a sensor are fired. They are fired in a sequence specific to a given product
    line or model. Laser recharge time is included. A firing sequence is sometimes referred to as a firing group. A firing
    sequence is not allowed to span multiple data packets.
    
    It takes 55.296 μs to fire all 16 lasers in a VLP-16 and recharge.


9.3.1.2 Laser Channel
    A laser channel is a single 903 nm laser emitter and detector pair. Each laser channel is fixed at a particular elevation angle
    relative to the horizontal plane of the sensor. Each laser channel is given its own Laser ID number. Since the elevation
    angle of a particular laser channel doesn't change, it doesn't appear in data packets. Its value is inferred by a data point's
    location in a data packet.

9.3.1.3 Data Point
    A data point is a measurement by one laser channel of a reflection of a laser pulse.
    
    A data point is represented in the packet by three bytes - two bytes of distance and one byte of calibrated reflectivity. The
    distance is an unsigned integer. It has 2 mm granularity. Hence, a reported value of 51,154 represents 102,308 mm or
    102.308 m. Calibrated reflectivity is reported on a scale of 0 to 255 as described in
    Calibrated Reflectivity on page 32. The
    elevation angle (ω) is inferred based on the position of the data point within a data block.
    
    A distance of 0 indicates a non-measurement. The laser is either off or a measurable reflection was not returned in time.

9.3.1.4 Azimuth
    A two-byte azimuth value (α) appears after the flag bytes at the beginning of each data block. The azimuth is an unsigned
    integer. It represents an angle in hundredths of a degree. Therefore, a raw value of 27742 should be interpreted as
    277.42°.
    
    Valid values for azimuth range from 0 to 35999. Only one azimuth value is reported per data block.

9.3.1.5 Data Block
    The information from two firing sequences of 16 lasers is contained in each data block. Each packet contains the data from
    24 firing sequences in 12 data blocks.
    Only one Azimuth is returned per data block.
    A data block consists of 100 bytes of binary data:
    A two-byte flag (0xFFEE)
    A two-byte Azimuth
    32 Data Points
    [2 + 2 + (32 × 3)] = 100 bytes
    For calculating time offsets it is recommended that the data blocks in a packet be numbered 0 to 11.

    
    
# PAGE 64 timing offsets for firings within the same block (?)
    
"

"""

import argparse
import dpkt
import numpy as np
import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output
import plotly.express as px

# Parse command line arguments
parser = argparse.ArgumentParser(description='PCAP file parser')
parser.add_argument('file_path', type=str, help='Path to the .pcap file')
args = parser.parse_args()


counter = 0
ipcounter = 0
tcpcounter = 0
udpcounter = 0

data = []
data2 = []

vlp16_pitch = [-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15]
vlp16_pitch = vlp16_pitch + vlp16_pitch


# count_azimuth_to_360 = 0
# first_time = True

# fig = go.Figure()

file_path =args.file_path
with open(file_path, 'rb') as file:
    pcap = dpkt.pcap.Reader(file)

    for ts, pkt in pcap:
        counter += 1
        eth = dpkt.ethernet.Ethernet(pkt)

        # Split packet into hexidecimal numbers
        hex_pkt = [hex(x) for x in pkt]
        # print("hex_pkts: ", hex_pkt[:10])

        # find all "ff ee" indeces in hex_pkt
        try:
            ff_ee_indeces = [i for i, x in enumerate(hex_pkt) if x == '0xff' and hex_pkt[i+1] == '0xee']
        except IndexError:
            continue

        # Split list into list of lists based on "ff ee" indeces
        split_hex_pkt = [hex_pkt[i:j] for i, j in zip(ff_ee_indeces, ff_ee_indeces[1:]+[None])]
        # print("split_hex_pkt: ", split_hex_pkt[:10])

        # chunk up this data into sections that start with "ff ee" and end just before the next "ff ee"
        for section in split_hex_pkt:
            for i,v in enumerate(section):
                section[i] = int(section[i], 16)
            if section:

                # Inside the section, break up into all firings and other data
                azimuth_hex = section[2:4]  # Get the first two hexidecimal numbers representing the azimuth
                azimuth_hex_reversed = azimuth_hex[::-1]  # Reverse the order of the hexidecimal numbers
                azimuth_decimal = (azimuth_hex_reversed[0] << 8) | azimuth_hex_reversed[1]
                azimuth = azimuth_decimal / 100  # Scale the azimuth
                # print("azimuth_hex: ", azimuth_hex, ", , azimuth: ", azimuth)

                prior_offset = 4
                for datapoint_offset in range(16):
                    start = prior_offset + (datapoint_offset * 3)
                    distance = section[start:start+2]
                    distance = distance[::-1]
                    distance = (distance[0] << 8) | distance[1]
                    # distance = distance / 100

                    reflectivity = section[start+2]
                    pitch = vlp16_pitch[datapoint_offset]

                    data.append((azimuth, pitch, distance))

                for datapoint_offset in range(16,32):
                    start = prior_offset + (datapoint_offset * 3)
                    distance = section[start:start+2]
                    distance = distance[::-1]
                    distance = (distance[0] << 8) | distance[1]
                    # distance = distance / 100

                    reflectivity = section[start+2]
                    pitch = vlp16_pitch[datapoint_offset]

                    data2.append((azimuth, pitch, distance))
                
        
        if eth.type != dpkt.ethernet.ETH_TYPE_IP:
            continue

        ip = eth.data
        ipcounter += 1

        if ip.p == dpkt.ip.IP_PROTO_TCP:
            tcpcounter += 1

        if ip.p == dpkt.ip.IP_PROTO_UDP:
            udpcounter += 1

        # count_azimuth_to_360 += 0.4
        # if count_azimuth_to_360 >= 360.0:
        #     break

        # Let's make x,y,z be right, forward, up
        # data = points
        # data2 = points2

        # if first_time:
        #     # Plot points with Plotly
        #     fig = go.Figure(data=[
        #     go.Scatter3d(x=[x for x, y, z in points],
        #                 y=[y for x, y, z in points],
        #                 z=[z for x, y, z in points],
        #                 mode='markers',
        #                 marker=dict(size=2, color='blue')),  # points in blue
        #     go.Scatter3d(x=[x for x, y, z in points2],
        #                 y=[y for x, y, z in points2],
        #                 z=[z for x, y, z in points2],
        #                 mode='markers',
        #                 marker=dict(size=2, color='red'))  # points2 in red
        #     ])
        #     fig.update_layout(scene=dict(
        #                         xaxis_title='X',
        #                         yaxis_title='Y',
        #                         zaxis_title='Z'))
        #     fig.show()
        #     first_time = False
        # else:
        #     print("len(points): ", len(points), ", points: ", points)

        #     fig.update_layout(
        #         scene=dict(
        #             xaxis_title='X',
        #             yaxis_title='Y',
        #             zaxis_title='Z',
        #         ),
        #         overwrite=True
        #     )

        #     # Update the plot with new points
        #     fig.data[0].x = [x[0] for x in points]
        #     fig.data[0].y = [x[1] for x in points]
        #     fig.data[0].z = [x[2] for x in points]
        #     fig.data[1].x = [x[0] for x in points2]
        #     fig.data[1].y = [x[1] for x in points2]
        #     fig.data[1].z = [x[2] for x in points2]
        #     # fig.show()

        #     fig.update_traces(
        #         x=[x for x, y, z in points],
        #         y=[y for x, y, z in points],
        #         z=[z for x, y, z in points]
        #     )

        #     fig.update_traces(
        #         x=[x for x, y, z in points2],
        #         y=[y for x, y, z in points2],
        #         z=[z for x, y, z in points2]
        #     )

print("Total number of packets in the pcap file:", counter)
print("Total number of ip packets:", ipcounter)
print("Total number of tcp packets:", tcpcounter)
print("Total number of udp packets:", udpcounter)

# Convert data to cartesian coordinates
points = []
for azimuth, pitch, distance in data:
    x = distance * np.sin(np.radians(azimuth)) * np.cos(np.radians(pitch))
    y = distance * np.cos(np.radians(azimuth)) * np.cos(np.radians(pitch))
    z = distance * np.sin(np.radians(pitch))
    points.append((x, y, z))


# points2 = []
# for azimuth, pitch, distance in data2:
#     x = distance * np.sin(np.radians(azimuth)) * np.cos(np.radians(pitch))
#     y = distance * np.cos(np.radians(azimuth)) * np.cos(np.radians(pitch))
#     z = distance * np.sin(np.radians(pitch))
#     points2.append((x, y, z))

# These should be joined in the future


# Save the data to a file (Dataframe)
import pandas as pd
df = pd.DataFrame(points, columns=['x', 'y', 'z'])
df.to_csv('data/points.csv', index=False)