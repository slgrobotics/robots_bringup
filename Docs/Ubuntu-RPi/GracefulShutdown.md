## Graceful Shutdown

When you turn off power to your Raspberry Pi, there's a chance that the system will be writing something to the SD card.
In this case the file system on the SD card can be corrupted, and the Pi will become unbootable.

To avoid this, ensure that your OS (Ubuntu 24.04 for example) is in a safe state - e.g. shut down.

**Note:** Raspberry Pi 5 has a built-in shutdown mechanism similar to what we are used on PCs.
Just connect your "Power" button to the jack between the HDMI and USB-C ports:

![Screenshot from 2025-05-08 13-11-05](https://github.com/user-attachments/assets/c1cc9e70-8a79-40f2-b484-72cc169da3ae)

![Screenshot from 2025-05-08 13-08-08](https://github.com/user-attachments/assets/a42c95b2-904d-4b50-beab-b1249a9836e2)

## For Raspberry Pi Zero to 4

Here is a way to shutdown the OS on earlier Pi's - by pressing a button, connected to a GPIO pin:

https://www.dprg.org/category/tutorial/ (thanks to Michael Wimble for the find).

https://github.com/zizumara/GPIOshutdown/tree/main

## Alternative way

I've been always doing it via pin 21 (hold 4 seconds) and `/etc/crontab`:

(we assume you have GPIO components installed as described [here](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/README.md))

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

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
