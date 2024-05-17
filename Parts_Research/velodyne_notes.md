Since I bought this LIDAR sensor on ebay in a last minute auction, I'm actually not familiar with its specs. Let's detail them here.

# VLP-16 / Puck Overview

VLP-16 / Puck
VelodyneLiDAR.com
Patent: Veloip.com

Conforms to US.STDS 60950-1 & 60950-22
Certified to CSA STD C22.2 Nos 60950-1 & 60950-22
Complies with performance standards for laser products under 21 CFR 1040 except with respect to those characteristics authorized by Variance Number FDA-216-V-2487 effective September 21st 2019.

Intertek 5010465

Serial Number: 11001192137376
MAC Address: 60-76-88-38-28-94
P/M: 80-VLP-16-A

# Damage
 - Frayed connector on Velodyne LIDAR puck.
 - Disconnected wires on Velodyne control box.
  - Darkest orange, and lightest blue wires - correspond to #6, #7 in wiring diagram document, which correspond to Ethernet TX-, and Ethernet RX+ (See ethernet_basics.md for more information). These wires are 28 American Wire Guage (AWG) (doubling diameter of wire ~= -6 guage, 1 guage = 7.3mm diameter).

# Specs

[Source](https://velodynelidar.com/products/puck/)

 - 100m range
 - 905 nm wavelength
 - Dual return (?) that enables increased number of points collected
 - 360 degree visibility
 - Supports [paraview](https://www.paraview.org/veloview/)

 - Sensors used in [ANYbotics](https://www.anybotics.com/) (Swiss) quadruped robot.

# Interface Box Notes

 - The interface box mainly prevents against overvoltage and reverse voltage. It as a "blade fuse" and a "TVS diode".
 - T568B Straight Through Ethernet Cable Adapter

Visible Components
 - 7805BG RRJ30
 - MCC 5KP3 (diode?)


# Factoids
 - There is a shield/nc(?) wire that (1) protects against EMI, and potentially (2) serves as overcurrent protection (electromagnetic compatibility protection).