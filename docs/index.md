# AVVV ETSI Documentation

## About AVVV

The AVVV project, standing for Autonomous Vehicle V2X Visualiser, aims to analyse and visualise V2X
communications. V2X refers to the communications between the autonomous vehicle and everything
else, including the road-side units (RSUs) and other intelligent vehicles (On-boar units or OBUs, for short).

When the autonomous vehicle is navigating in the streets, it perceives its environment. It specifically
detects and predicts pedestrians and other vehicles on the road and traffic lights and signs. The vehicle
then makes decisions based on these detections. The autonomous vehicle can communicate its
perception data via some network means dedicated to this purpose, to other intelligent vehicles and
entities on the road. This cooperation can dramatically increase planning and navigation accuracy and
road safety for all users.

The road-side units abbreviated as RSUs are enabled to accurately monitor roads and perform the same
perception as intelligent vehicles using their state-of-the-art sensors and algorithms. Therefore, apart
from the vehicle-to-vehicle communication, the vehicle-to-RSU communication also offers beneficial
possibilities.

The V2X communications are done through ROS messages. These messages are then transmitted from
the source to the destination as UDP packets. Like any ordinary network packet, these packets can get
lost in the network or arrive at the target late. The AVVV’s mission is to analyse these latencies and packet
losses and visualise them in a user-friendly manner.

In AVVV, we investigate the object prediction communications between one or more target vehicles and
one or more RSUs. The investigation covers network aspects and parameters such as delay, packet loss,
jitter and RSSI.

Delay refers to the time it takes for the packets to transmit from the sender and reach the receiver. Using
the DSRC (Dedicated Short-Range Communication) network, the delay will consist of the propagation and
transmission delays. Whereas, using the cellular network, it will additionally include the queueing and
processing delays in the intermediate routers. Many factors can impact delay; bandwidth, MAC protocol,
distance between the sender, the intermediate routers and the receiver, velocity of signals in various
media, congestion in network and servers’ processing speed are a few to name.

Packet loss occurs when the transmitted packets fail to reach the intended receiver. Some causes of
packet loss are network congestion, software bugs, hardware failure and security threats.

Jitter is a term used to describe variations in network latency (delay). Jitter is higher when data are
received at irregular delays. The same delay causes can contribute to jitter as well, plus signal inferences.

Received Signal Strength Indicator (RSSI), in summary, represents how well the receiver gets the
transmitted data. It determines the strength of the signal received by the receiver. It is a relative value
which IEEE 802.11 specifies to be between 0 and 255.

RSUs and OBUs broadcast their data based on the ETSI standard in the format of Collective Perception
Messages (CPMs). Visit the [ETSI official document](https://www.etsi.org/deliver/etsi_tr/103500_103599/103562/02.01.01_60/tr_103562v020101p.pdf) for more information about ETSI and the CPM format.

## Getting started

- [Installation](installation) pages explain the installation steps of AVVV.
- [Tutorials](tutorials) pages explain several tutorials that you should try after installation.
- [How-to guides](how-to-guides) pages explain advanced topics that you should read after you get comfortable with AVVV.
- [Design](design) pages explain the design concept of AVVV.
- [protocols](protocols) pages explain protocols used.
- [Datasets](datasets) pages contain information about datasets that can be used with AVVV.
- [Support](support) pages explain several support resources.
