_"hus·​sar (ˌ)hə-ˈzär  -ˈsär : a member of any of various European military units originally modeled on the Hungarian light cavalry of the 15th century"_

# Set up Husarnet

Here we set up a virtual private network (VPN), specifically designed to overcome limitations of ROS2 DDS.

Any ROS2 DDS implementation (here we are interested in Cyclone DDS specifically):

- floods LAN with UDP packets
- cannot go across LAN subnets
- cannot "go global" - over LTE, 5G or even wired Internet

Husarnet VPN is built specifically with ROS2 in mind and solves the above problems.

Here are the links:

https://husarnet.com/docs/how-husarnet-works/

https://husarion.com/tutorials/other-tutorials/husarnet-cyclone-dds/

https://husarion.com/tutorials/

## Creating Husarnet account

Sign up here: https://app.husarnet.com

Assign a name to your VPN and get a private key (keep it secret).

## Installing Husarnet

See: https://husarnet.com/docs/platform-linux-install

```
curl https://install.husarnet.com/install.sh | sudo bash

ifconfig -a
(shows "hnet0" network adapter now)

systemctl status husarnet

sudo husarnet join <your private key here>

husarnet status
```
Do it on your robot(s) and on the desktop "workstation", run "ifconfig -a hnet0" and make a list of you machine's IPv6 addresses:
```
mydesktop: ab35:f451:xxx:xxx:xxx:xxx:xxx:678
plucky:    ab35:69d9:xxx:xxx:xxx:xxx:xxx:123
dragger:   ab35:e0b1:xxx:xxx:xxx:xxx:xxx:345
```
Keep an eye on the list at the VPN Dashboard - https://app.husarnet.com too.

## Configuring Cyclone DDS

On each machine:

- Create a file:

cat ~/cyclonedds.xml
```
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain>
        <General>
            <Interfaces>
                <NetworkInterface name="hnet0"/>
            </Interfaces>
            <AllowMulticast>false</AllowMulticast>
            <FragmentSize>1194B</FragmentSize><!-- default: 1344 B minus Husarnet metadata (~150 B) -->
            <Transport>udp6</Transport>
        </General>      
    </Domain>
</CycloneDDS>
```
- Edit .bashrc on each machine to look like this:

Dragger's .bashrc:
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ros/cyclonedds.xml # !actual path! enable IPv6 in Cyclone DDS
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST #optional: thanks to this env, you make sure all remote traffic goes through Husarnet
#export ROS_STATIC_PEERS="mydesktop;dragger"
# or with IPv6 addresses - a robot has two, workstation has three (for all robots):
export ROS_STATIC_PEERS="ab35:f451:xxx:xxx:xxx:xxx:xxx:678;ab35:e0b1:xxx:xxx:xxx:xxx:xxx:345"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

After you source the .bashrc, whatever you launch (ROS2 related) will talk directly (peer-to-peer) across VPN, not flooding LAN with UDP trafic.

And, magically, it should talk the same way while your robot is connected to a phone hotspot WiFi or anything similar.

## Note: I am using static IPv4 addresses and subnets

Just in case...

For example, consider two "chained" routers (cable-modem ---> router167 ---> router168) - each with their LANs. That makes two _subnets_.

Router168 has firewall disabled. Both have static routing tables defined to direct trafic to each other.

And the robot's _/etc/netplan/50-cloud-init.yaml_ may look like this (I am open to suggestions):
```
network:
    version: 2
    ethernets:
      eth0:
        optional: true
        dhcp6: true
        dhcp4: true

    wifis:
      wlan0:
        optional: true
        access-points:
          "mywifinet":   # robot is on router167's subnet
              password: "xxxxxxxx"
        dhcp6: true
        dhcp4: false
        addresses:
          - 192.167.1.137/24  # this is its IPv4 static address
        routes:
          - to: 192.168.1.0/24  # to workstation on another subnet
            via: 192.168.1.1
          - to: default
            via: 192.167.1.1
        nameservers:
          addresses:
            - 192.167.1.1
            - 192.168.1.1
```

Desktop/Workstation is residing on LAN168, that's why the routing and we need Husarnet to work across subnets.

## Useful Links:

https://husarnet.com/docs/cli-guide


