## Ninebot _miniPRO_ Notes

I have an original _2016 Ninebot miniPRO N3M320_. That was the first model to be sold in the US.

I bought it then to turn into a robot, bit once I tried riding it - I got hooked. 
I somehow avoided the nasty "_safety firmware upgrade_", and avoided broken bones.
It helped me to get around when traveling in an RV, and then here - getting to my mailbox 😉.
Its battery is still very healthy in 2025, but I am not using it that often anymore, and riding those is a perishable skill.
So, it's time to quit - i.e. to turn it into a robot.

My Ninebot miniPRO photos are here: https://photos.app.goo.gl/yHXs7fP7u7ae8fa78

**Note:** the _add-on platform_ is just an experiment, and would be only useful if there's a way to
precisely and directly control miniPRO's wheels, not its balance (as Bluetooth link currently allows).
Without such control, the platform makes the miniPRO very difficult to drive.
MiniPRO has that option - "_Mecha mode_" is mentioned [here](https://forum.electricunicycle.org/topic/32214-ninebot-mecha-kit-ble-protocol%EF%BC%9F/) 
and is used for [Mecha Kit](https://www.amazon.com/Segway-Applicable-Self-Balancing-Human-Body-Integration/dp/B09GFHNRV3?th=1).
It needs further investigation.

While the mechanical stuff is optional and relatively easy, controlling it could be a challenge.
Fortunately, there's a repository with sample code here: https://github.com/mjeronimo/minipro (please STAR it!)

Here is how I compile and run this code on my Ubuntu 24.04 Desktop computer:

1. Install prerequisites:
```
sudo apt install bluez-tools liblog4cxx-dev
```
2. Clone the repository (using [my fork](https://github.com/slgrobotics/minipro) of the original):
```
mkdir ~/ninebot/src
cd ~/ninebot/src
git clone https://github.com/slgrobotics/minipro.git
```
At this point you have to figure out how to use Bluetooth and try enabling your BT/LE adapter.

With a bit of luck, you'll have a single BT-LE device, supported by Ubuntu - and you'll be able to see powered-up miniPRO in *Settings/Bluetooth*.

Use ```lsusb (-v)```, ```bluetoothctl (list/show/devices)``` and ```bt-device -l``` to discover your hardware and connections.

In my case, I had a built-in *Qualcomm Atheros Communications AR3011 Bluetooth* - which wasn't working right, and I had to disable it:
```
lsusb
Bus 002 Device 006: ID 0cf3:3005 Qualcomm Atheros Communications AR3011 Bluetooth

sudo vi /etc/udev/rules.d/81-bluetooth-hci.rules

# Disable Qualcomm Atheros Communications AR3011 Bluetooth:
SUBSYSTEM=="usb", ATTRS{idVendor}=="0cf3", ATTRS{idProduct}=="3005", ATTR{authorized}="0"

reboot
```
I then could plug in an *ORICO BTA-403 USB Bluetooth 4.0 adapter* - and it worked fine.  

3. Find out your miniPRO Bluetooth address:
```
bt-device -l
Added devices:
17-86-B6-F9-11-5C (17:86:B6:F9:11:5C)
 NinebotMini0671 (AA:BB:CC:DD:EE:FF)  <- your Bluetooth address will be here
```
You may want to use VS Code to open *~/ninebot/src/minipro* folder, or just use your favorite editor (_vi_, of course)

4. Edit *t_minipro.cpp* file:
```
// Line 41: change to your Bluetooth address.
vi ~/ninebot/src/minipro/test/minipro/t_minipro.cpp
```
5. Build it (install colcon if you don't have it):
```
cd ~/ninebot
colcon build
```
5. Now you can run it:
```
cd ~/ninebot/build/minipro
./t_minipro
```
At this point you should be able to drive your miniPRO using your XBox 360 game controller.

Caution: miniPRO is a beast!

**Observations:**
- the code works for my miniPRO with _original firmware_. Yours could be with "_safety upgrade_" and might not work. I haven't tested that.
- not all miniPRO models had remote control feature.
- while I set a pairing code when I bought the miniPRO, this program doesn't need it and connects fine without it.
- original XBox 340 joystick code misinterpreted joystick events, so that turns were handled by triggers. I fixed that in my fork, the miniPRO is now controlled by _left thumbstick_ only.
- the code sends commands, but doesn't read BT stream. The program (*t_minipro*) hangs after several minutes driving.
- the included _bluez_ code is old, but seems to work fine. If it ain't broken, don't fix it.
- there is also _gattclient_ program there (part of _lib/bluez_), I have to investigate it later:
```
me@mycomp:~/ninebot/build/minipro$ ./gattclient --help
btgatt-client
Usage:
	btgatt-client [options]
Options:
	-i, --index <id>		Specify adapter index, e.g. hci0
	-d, --dest <addr>		Specify the destination address
	-t, --type [random|public] 	Specify the LE address type
	-m, --mtu <mtu> 		The ATT MTU to use
	-s, --security-level <sec> 	Set security level (low|medium|high)
	-v, --verbose			Enable extra logging
	-h, --help			Display help
Example:
btgattclient -v -d C4:BE:84:70:29:04
```
----------------

Back to https://github.com/slgrobotics/robots_bringup

