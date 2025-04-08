## WiFi Logger Visualizer

There's a package created by Michael Wimble to log and visualize WiFi signal strength along robot's path.

The original is here: https://github.com/wimblerobotics/wifi_logger_visualizer

The fork I use for testing is here: https://github.com/slgrobotics/wifi_logger_visualizer

----------------

## WiFi testing and benchmarking

There are many useful commands to discover and measure performance of WiFi interfaces:
```
ip a
ifconfig -a
sudo lshw -C network
iwconfig wlan0   # use actual interface name here
iw dev wlan0 info
find /lib/modules/$(uname -r)/kernel/drivers/net/wireless
```
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
[  5] local 172.17.1.130 port 51436 connected to ***** port 5201
[  7] local 172.17.1.130 port 51438 connected to ***** port 5201
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


**Note:** Raspberry Pi 5 doesn't have SMA connector for WiFi antennas and uses a PCB trace for this purpose. 
Its "_antenna_" can be easily obstructed by metal active cooler and hats.
[Here](https://community.element14.com/products/raspberry-pi/f/forum/53868/how-good-is-wifi-in-raspberry-pi-5) is more info on this topic.

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
