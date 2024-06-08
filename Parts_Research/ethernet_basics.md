# Overview

While questioning what exactly Ethernet RX+, RX-, TX+, and TX- refer to, I have decided to summarize my learning here.

Wires with current cause ElectroMagnetic Interference (EMI) in other conductive materials, and in practice, this is known as crosstalk. There are two methods of compensating for this interference: (1) Unshielded Twisted Pairs (UTP), and (3) Shielded Twisted Pairs (STP). Shielded wires reduce EMI by isolating currents within a faraday cage of sorts. Unshielded wires however, rely upon two wires sending inverse signals, therefore negating any external EMI - known as a Balanced Pair. Blanced pairs are denoted with + and - signs, referring to the fact that they are a pair of twisted wires, with opposite signals (voltage). Twisting wires ensure near equal distances to interfering fields, which keeps a consistent voltage difference between the two wires, even if they both shift. This also allows the bits sent to be represented by the relation of voltages between the two wires, improving the decision boundary width and interpretability.

# Packet Capture

sudo tcpdump -i ethernet_interface_name -n -vv -s 0 -c 20000 -W 20000 -w throw.pcap

# Sources

https://www.practicalnetworking.net/stand-alone/ethernet-wiring/

# Setup with ROS & Velodyne VLP16

http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16