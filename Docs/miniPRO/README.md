# Ninebot _miniPRO_ Notes

I have an original 2016 Ninebot miniPRO N3M320, that was the first model to be sold in the US.

I bought it then to turn into a robot, bit once I tried riding it - I got hooked. 
I somehow avoided the nasty "_safety firmware upgrade_", and avoided broken bones.
It helped me to get around when traveling on an RV, and then here - getting to my mailbox 😉.
Its battery is still very healthy, but I am not using it that often anymore, and riding those is a perishable skill.
Time to quit - i.e. to turn it into a robot.

My Ninebot miniPRO photos are here: https://photos.app.goo.gl/yHXs7fP7u7ae8fa78

While the mechanical stuff is optional and relatively easy, controlling it could be a challenge.
Fortunately, there's a repository with sample code here: https://github.com/mjeronimo/minipro (please STAR it!)

Here is how I compile and run tis code:

1. Install prerequisites:
```
sudo apt install bluez-tools liblog4cxx-dev
```
2. Clone the repository:
```
mkdir ~/ninebot/src
cd ~/ninebot/src
git clone https://github.com/mjeronimo/minipro.git
cd minipro/
```
3. Find out your miniPRO Bluetooth address:
```
bt-device -l
Added devices:
17-86-B6-F9-11-5C (17:86:B6:F9:11:5C)
 NinebotMini0671 (AA:BB:CC:DD:EE:FF)  (your Bluetooth address will be here)
```
4. Edit files:
```
vi ~/ninebot/src/minipro/CMakeLists.txt 
// Line 1: change to 3.5: cmake_minimum_required(VERSION 3.0)
vi ~/ninebot/src/minipro/test/minipro/t_minipro.cpp
// Line 41: change to your Bluetooth address.
```
5. Build it:
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

**Notes:**
- the code works for my miniPRO with _original firmware_. Yours could be with "_safety upgrade_" and might not work. I haven't tested that.
- while I set a pairing code when I bought the miniPRO, this program doesn't need it and connects fine

----------------

Back to https://github.com/slgrobotics/robots_bringup

