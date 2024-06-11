"""
Run with:

~/development/venvs/p39/bin/python testing/test_parse_packet.py data/capture.pcap




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

def parse_pcap_file(file_path):
    counter = 0
    ipcounter = 0
    tcpcounter = 0
    udpcounter = 0

    with open(file_path, 'rb') as file:
        pcap = dpkt.pcap.Reader(file)

        for ts, pkt in pcap:
            counter += 1
            eth = dpkt.ethernet.Ethernet(pkt)

            # Split packet into hexidecimal numbers
            hex_pkt = [hex(x) for x in pkt]
            # print("hex_pkts: ", hex_pkt[:10])

            # find all "ff ee" indeces in hex_pkt
            ff_ee_indeces = [i for i, x in enumerate(hex_pkt) if x == '0xff' and hex_pkt[i+1] == '0xee']

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
                    print("azimuth_hex: ", azimuth_hex, ", , azimuth: ", azimuth)
                    
            
            if eth.type != dpkt.ethernet.ETH_TYPE_IP:
                continue

            ip = eth.data
            ipcounter += 1

            if ip.p == dpkt.ip.IP_PROTO_TCP:
                tcpcounter += 1

            if ip.p == dpkt.ip.IP_PROTO_UDP:
                udpcounter += 1

    print("Total number of packets in the pcap file:", counter)
    print("Total number of ip packets:", ipcounter)
    print("Total number of tcp packets:", tcpcounter)
    print("Total number of udp packets:", udpcounter)


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='PCAP file parser')
    parser.add_argument('file_path', type=str, help='Path to the .pcap file')
    args = parser.parse_args()

    # Call the function to load and parse the pcap file
    parse_pcap_file(args.file_path)

if __name__ == '__main__':
    main()