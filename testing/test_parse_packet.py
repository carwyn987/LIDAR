"""
Run with:

~/development/venvs/p39/bin/python testing/test_parse_packet.py data/throw.pcap




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
import pandas as pd


points = pd.read_csv("data/points.csv")
# add a column of all ones
points["size_column"] = 3

print("Got points")

app = Dash(__name__)

app.layout = html.Div([
    html.H1('Custom Dash(board) for Parsed LiDAR Ethernet Packets'),
    dcc.Graph(id="graph", style={'width': '100vw', 'height': '100vh'}),
    dcc.Interval(
        id='timer',
        interval=100, # in milliseconds
        n_intervals=0
    )
])

print("Got points2")

@app.callback(
    Output("graph", "figure"), 
    Input("timer", "n_intervals"))
def update_bar_chart(n):
    plot_max = n * 15000
    plot_min = plot_max - 15000
    # plot_points = points[plot_min:plot_max]
    # extract rows from df according to plot_min and plot_max
    plot_points = points.iloc[plot_min:plot_max]

    fig = px.scatter_3d(
                    data_frame=plot_points,
                    x="x",
                    y="y",
                    z="z",
    ).update_traces(marker=dict(color='red', size=plot_points['size_column']), line=dict(width=0, color='rgba(0,0,0,0)'))
    
    # Set the camera angle
    camera_angle = (90 + n) % 360 # Change this value to adjust the rotation speed
    radius = 1
    fig.update_layout(scene_camera=dict(
                        eye=dict(x= radius* np.cos(np.radians(camera_angle)), y=radius * np.sin(np.radians(camera_angle)), z=radius),  # Zoomed in more
                        # up=dict(x=0, y=1, z=0),
                        center=dict(x=0, y=0, z=0),
                        ))
    
    return fig

print("Running server")
app.run_server(debug=True)