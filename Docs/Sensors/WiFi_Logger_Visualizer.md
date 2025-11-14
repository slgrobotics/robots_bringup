## WiFi Logger Visualizer

There's a package created by Michael Wimble to log and visualize WiFi signal strength along robot's path.

The original is here: https://github.com/wimblerobotics/wifi_logger_visualizer

The fork I use for testing is here: https://github.com/slgrobotics/wifi_logger_visualizer

----------------

## WiFi testing and benchmarking

Please review tips [here](https://articulatedrobotics.xyz/tutorials/ready-for-ros/networking).

There are many useful commands to discover and measure performance of WiFi interfaces:
```
ip a
ifconfig -a
sudo lshw -C network
iwconfig wlan0   # use actual interface name here
iw dev wlan0 info
find /lib/modules/$(uname -r)/kernel/drivers/net/wireless
nload wlan0
```
When running my [Dragger](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger) robot,
with RViz2 monitoring it on my desktop machine, I've got the following metrics from `nload wlan0`:
```
min: 19 Mbits/s
avg: 43 Mbits/s
max: 63 Mbits/s
```
There's no camera or pointcloud data going across WiFi yet, but the traffic is already very high.
It means that benchmarking and improving your WiFi should be at the top of your list.

Here are some [theoretical limits](https://www.intel.com/content/www/us/en/support/articles/000005725/wireless/legacy-intel-wireless-products.html) for 802.11xx networks.
You probably will have either 802.11ac (1300 Mbits/sec, 200 Mbits/sec in reality) or 802.11ax (Wi-Fi 6) - over 300 Mbits/sec real throughput.

For benchmarking, _iperf3_ can be used. It must be installed on both ends of the connection being tested:
```
sudo apt install iperf3
```
During the installation it asks if the daemon should be started, answer _Yes_. Use the following commands to control the daemon:
```
systemctl status iperf3
sudo systemctl stop iperf3
sudo systemctl start iperf3
```
Then you can benchmark the connection to any computer where _iperf3_ daemon is running:
```
iperf3 -c plucky.local
  or
iperf3 -c plucky.local --bidir
Connecting to host plucky.local, port 5201
[  5] local XXX.XX.X.130 port 51436 connected to ***** port 5201
[  7] local XXX.XX.X.130 port 51438 connected to ***** port 5201
[ ID][Role] Interval           Transfer     Bitrate         Retr  Cwnd
[  5][TX-C]   0.00-1.00   sec  19.4 MBytes   162 Mbits/sec    0    956 KBytes       
[  7][RX-C]   0.00-1.00   sec  6.62 MBytes  55.5 Mbits/sec
...
[  5][TX-C]   9.00-10.00  sec  18.0 MBytes   151 Mbits/sec    0   4.00 MBytes       
[  7][RX-C]   9.00-10.00  sec  6.38 MBytes  53.4 Mbits/sec                  
- - - - - - - - - - - - - - - - - - - - - - - - -
[ ID][Role] Interval           Transfer     Bitrate         Retr
[  5][TX-C]   0.00-10.00  sec   193 MBytes   162 Mbits/sec    0             sender
[  5][TX-C]   0.00-10.20  sec   193 MBytes   159 Mbits/sec                  receiver
[  7][RX-C]   0.00-10.00  sec  54.1 MBytes  45.4 Mbits/sec    0             sender
[  7][RX-C]   0.00-10.20  sec  50.2 MBytes  41.3 Mbits/sec                  receiver
iperf Done.
```
The test above is between a Raspberry Pi 5 and an Intel I7 desktop. A mesh WiFi6 router is within 10' from the robot. A ROS2 robot might need 200+ Mbits/s bandwidth.

The realistic range of WiFi could be 50 meters indoors and about 100 meters outdoors, with 5 GHz links being less by at least 10 percent.


**Note:** Raspberry Pi 5 doesn't have SMA connector for WiFi antennas and uses a PCB trace for this purpose. 
Its "_antenna_" can be easily obstructed by metal active cooler and hats.
[Here](https://community.element14.com/products/raspberry-pi/f/forum/53868/how-good-is-wifi-in-raspberry-pi-5) is more info on this topic.

## WiFi 6 Mesh Networks and Travel Routers

A "_seamless whole-home coverage_" feature of contemporary WiFi 6 (802.11ax) Mesh routers allows client devices to select the router node with the strongest signal.

This works well, if the client device also supports that feature, but might cause poor performance otherwise.
Older client devices often connect to a far-away router node and keep that connection.
Mesh networks can be configured to assign router nodes to client devices ("preferred connection" setting), but that works only for stationary devices.

This issue can be solved by using a good WiFi 6 capable travel router in client mode.

## Direct WiFi between your laptop and the robot

For many use scenarios, especially outdoors, you can avoid using routers and establish direct WiFi connection between your workstation (laptop) and the robot's Raspberry Pi 5.

Refer to an excellent instructional by Nathan Lewis [here](https://nrlewis.dev/blog/rpi-hotspot/).

## Improving your WiFi

While Raspberry Pi WiFi might be marginally adequate for in-home use, an outdoor robot has to do better.

My [Dragger](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger) robot
has a [WiFi 6 Travel Router](https://www.amazon.com/dp/B0D41G5N95) in Client mode, with a 9/12V USB C [power adapter](https://www.amazon.com/dp/B0C8TBM2QM).

Raspberry Pi 5 is connected to the router via an Ethernet cable, so that the trafic goes through it. I keep the RPi's `wlan0` running, so that *wifi_logger* node could use it for signal strengh measurement. 

**Note:**
- After disabling network config, you need to `sudo apt install avahi-daemon` to restore Multicast DNS (mDNS) service required for local hostname discovery. 
- It is very important to have the "routes:" section below, or your ping delay could be very random

Here is how I configure Dragger's network, with `eth0` operating at fixed address (put your network real addresses there):
```
ros@dragger:~$ sudo cat /etc/netplan/50-cloud-init.yaml 
[sudo] password for ros: 
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    version: 2
    ethernets:
      eth0:
        optional: true
        dhcp6: true
        dhcp4: false
        addresses:
          - XXX.XX.X.152/24
        routes:
          - to: default
            via: XXX.XX.X.XXX
        nameservers:
          addresses:
            - XXX.XX.X.XXX
 
    wifis:
      wlan0:
        optional: true
        access-points:
          "aaaaaa":
              password: "xxxxxxx"
        dhcp6: true
        dhcp4: true
        nameservers:
          addresses:
            - XXX.XX.X.XXX
```
Near the garage router _iperf3_ shows 700+ Mbits/sec one-way data throughput, and about half bidirectional.

At about 35 meters from the garage router, here are the metrics:
```
ros@dragger:~$ iwconfig wlan0
wlan0     IEEE 802.11  ESSID:"aaaaaa"  
          Mode:Managed  Frequency:2.427 GHz  Access Point: xxx   
          Bit Rate=12 Mb/s   Tx-Power=31 dBm   
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:on
          Link Quality=50/70  Signal level=-60 dBm  
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:22  Invalid misc:0   Missed beacon:0

ros@dragger:~$ iperf3 -c desktop5.local --bidir
Connecting to host desktop5.local, port 5201
[  5] local XXX.XX.X.152 port 36186 connected to XXX.XX.X.153 port 5201
[  7] local XXX.XX.X.152 port 36202 connected to XXX.XX.X.153 port 5201
[ ID][Role] Interval           Transfer     Bitrate         Retr  Cwnd
[  5][TX-C]   0.00-1.00   sec  26.8 MBytes   224 Mbits/sec    0   1.46 MBytes       
[  7][RX-C]   0.00-1.00   sec  5.00 MBytes  41.9 Mbits/sec                  
[  5][TX-C]   1.00-2.00   sec  22.5 MBytes   189 Mbits/sec    2   1.19 MBytes       
[  7][RX-C]   1.00-2.00   sec  9.88 MBytes  82.8 Mbits/sec                  
[  5][TX-C]   2.00-3.00   sec  19.5 MBytes   164 Mbits/sec    0   1.29 MBytes       
[  7][RX-C]   2.00-3.00   sec  19.5 MBytes   164 Mbits/sec                  
[  5][TX-C]   3.00-4.00   sec  16.5 MBytes   138 Mbits/sec    0   1.54 MBytes       
[  7][RX-C]   3.00-4.00   sec  24.1 MBytes   202 Mbits/sec                  
[  5][TX-C]   4.00-5.00   sec  12.1 MBytes   102 Mbits/sec    0   1.54 MBytes       
[  7][RX-C]   4.00-5.00   sec  24.6 MBytes   207 Mbits/sec                  
[  5][TX-C]   5.00-6.00   sec  12.8 MBytes   107 Mbits/sec    0   1.54 MBytes       
[  7][RX-C]   5.00-6.00   sec  27.1 MBytes   228 Mbits/sec                  
[  5][TX-C]   6.00-7.00   sec  12.0 MBytes   101 Mbits/sec    0   1.54 MBytes       
[  7][RX-C]   6.00-7.00   sec  26.4 MBytes   221 Mbits/sec                  
[  5][TX-C]   7.00-8.00   sec  12.1 MBytes   102 Mbits/sec    0   1.54 MBytes       
[  7][RX-C]   7.00-8.00   sec  25.0 MBytes   210 Mbits/sec                  
[  5][TX-C]   8.00-9.00   sec  14.5 MBytes   122 Mbits/sec    0   1.54 MBytes       
[  7][RX-C]   8.00-9.00   sec  27.8 MBytes   233 Mbits/sec                  
[  5][TX-C]   9.00-10.00  sec  12.8 MBytes   107 Mbits/sec    0   1.54 MBytes       
[  7][RX-C]   9.00-10.00  sec  25.1 MBytes   211 Mbits/sec                  
- - - - - - - - - - - - - - - - - - - - - - - - -
[ ID][Role] Interval           Transfer     Bitrate         Retr
[  5][TX-C]   0.00-10.00  sec   162 MBytes   135 Mbits/sec    2             sender
[  5][TX-C]   0.00-10.02  sec   159 MBytes   133 Mbits/sec                  receiver
[  7][RX-C]   0.00-10.00  sec   218 MBytes   183 Mbits/sec    0             sender
[  7][RX-C]   0.00-10.02  sec   214 MBytes   180 Mbits/sec                  receiver

iperf Done.

ros@dragger:~$ iperf3 -c desktop5.local
Connecting to host desktop5.local, port 5201
[  5] local XXX.XX.X.152 port 40602 connected to XXX.XX.X.153 port 5201
[ ID] Interval           Transfer     Bitrate         Retr  Cwnd
[  5]   0.00-1.00   sec  32.4 MBytes   271 Mbits/sec    2    870 KBytes       
[  5]   1.00-2.00   sec  30.0 MBytes   252 Mbits/sec    0    977 KBytes       
[  5]   2.00-3.00   sec  31.8 MBytes   266 Mbits/sec    0   1.04 MBytes       
[  5]   3.00-4.00   sec  33.1 MBytes   278 Mbits/sec    0   1.10 MBytes       
[  5]   4.00-5.00   sec  30.8 MBytes   258 Mbits/sec    0   1.14 MBytes       
[  5]   5.00-6.00   sec  32.0 MBytes   268 Mbits/sec    0   1.17 MBytes       
[  5]   6.00-7.00   sec  32.9 MBytes   276 Mbits/sec    0   1.18 MBytes       
[  5]   7.00-8.00   sec  31.8 MBytes   266 Mbits/sec    0   1.19 MBytes       
[  5]   8.00-9.00   sec  34.9 MBytes   293 Mbits/sec    0   1.19 MBytes       
[  5]   9.00-10.00  sec  35.8 MBytes   300 Mbits/sec    0   1.19 MBytes       
- - - - - - - - - - - - - - - - - - - - - - - - -
[ ID] Interval           Transfer     Bitrate         Retr
[  5]   0.00-10.00  sec   325 MBytes   273 Mbits/sec    2             sender
[  5]   0.00-10.02  sec   323 MBytes   270 Mbits/sec                  receiver

iperf Done.
```
The `ping` stays within 3..10 ms range. Connection becomes not reliable (for ROS2, at least) above the 35 meters range in my case.

As the result, Dragger can travel within the 35 meter range from the garage router and measure the "heatmap":

![Screenshot from 2025-04-09 20-14-39](https://github.com/user-attachments/assets/2394d7a5-2d2c-40e1-b79d-1ae8dd8c0083)

**Note:** I started ROS when Dragger was far from the router, and then moved towards it. That's why `/odometry/local` meters are negative on the plot.

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
