## Graceful Shutdown

When you turn off power to your Raspberry Pi, there's a chance that the system will be writing something to the SD card.
In this case the file system on the SD card can be corrupted, and the Pi will become unbootable.

To avoid this, ensure that your OS (Ubuntu 24.04 for example) is in a safe state - e.g. shut down.

Here is a way to shutdown the OS by pressing a button, connected to a GPIO pin:

https://github.com/zizumara/GPIOshutdown/tree/main (thanks to Michael Wimble for the find).

I've been always doing it via pin 21 (hold 4 seconds) and /etc/crontab:

```
# /etc/crontab: system-wide crontab
...
# Shutdown when router (eth0) is turned off:
#* * * * * root /bin/grep 'down' /sys/class/net/eth0/operstate && halt -p

# Run off button:
* * * * * root /bin/bash /home/pi/Projects/runOffButton.sh
```
/home/pi/Projects/runOffButton.sh
```
value=$( ps -ef | grep -ic off-switch.py )
echo $value
if [ $value -eq 1 ]
then
   cd /home/pi/Projects
   # sudo timedatectl set-timezone America/Chicago
   echo `date` " | IP: off-switch.py not running, trying to start it" >> runOffButton.log
   /usr/bin/python3 ./off-switch.py &
fi
```
/home/pi/Projects/off-switch.py
```
from time import sleep
import os
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
btn_timer = 0
while 1:
    state = GPIO.input(21)
    if(state == False):
        btn_timer = btn_timer + 1
    else:
        if(btn_timer > 7):
            os.system('sudo shutdown -h now &')
        btn_timer = 0
    sleep(0.5)
```
