Introduction
------------

  This project is an Arduino Sketch for the Teensy 3.1 USB microcontroller.  It uses the floating states of 12 analog pins to generate random numbers.  It's can easily be used on Linux platform the detects the Arduino Teensy as a Serial device (Kernel 2.6+).  I would expect it works on Windows platforms also with the Teensy drivers installed.  The speed of reading 12 analog pins can be very slow, so I have some optimizations to generate random numbers very quickly.  The Teensy generates HW bases random numbers when the USB port is idle and saves them init's internal 64k RAM.  Reading random numbers from the Teensy will come from many different locations.   It's internal 64k RAM, the 2k eeprom, two separate PRNG algorithms and two different ways of reading the Analog pins.  It will generate random numbers that passes all the dieharder tests.

Installation
------------------------

  Follow the instructions to install the Arduino software from <http://www.arduino.cc/en/main/software>, then update it with the Teensy drivers from <https://www.pjrc.com/teensy/td_download.html>.   Be sure to install the udev rules...

  In the Arduino Development GUI, first configure your device.  Tools --> Board --> Teensy 3.1.  Also Tools --> USB-Type --> Serial.  You may want to first test your Teensy 3.x by installing it on a free USB port, then uploading the example "Blink" sketch.

  When your Teensy is fully functional, open "teensy-hw_rnd.ino" --> Verify(check mark) --> Upload(right arrow).  Reset your Teensy.  Teensy should now be generating random numbers.

Testing
-------

   When the Sketch has been upload, you can now start reading from /dev/ttyACM0 (Your USB device name might be different!).   It will most likely work very slow because you must set the port to "raw mode"

```
/bin/stty -F /dev/ttyACM0 raw -echo
```

  An example to read data from the device.  Piping the data to "od" will convert the numbers to hex.

```
dd if=/dev/ttyACM0 |od -x
```

  To save the stty settings on reboot, I recommend to modify the udev rules you installed (above).

```
# UDEV Rules for Teensy boards, http://www.pjrc.com/teensy/
#
# The latest version of this file may be found at:
#   http://www.pjrc.com/teensy/49-teensy.rules
#
# This file must be placed at:
#
# /etc/udev/rules.d/49-teensy.rules    (preferred location)
#   or
# /lib/udev/rules.d/49-teensy.rules    (req'd on some broken systems)
#
# To install, type this command in a terminal:
#   sudo cp 49-teensy.rules /etc/udev/rules.d/49-teensy.rules
#
# After this file is installed, physically unplug and reconnect Teensy.
#
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789]?", ENV{ID_MM_DEVICE_IGNORE}="1"
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789]?", ENV{MTP_NO_PROBE}="1"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789]?", MODE:="0644",RUN+="/bin/stty -F %k raw -echo"
```


Testing randomness
------------------

  The best tool to test randomness is "dieharder".  Run the following command to test your Teensy USB HW random number generator to dieharder.   Dieharder is usually available from apt or yum repos.  You can also DL and compile it from <http://www.phy.duke.edu/~rgb/General/dieharder.php>.

```
dd if=/dev/ttyACM0 |dieharder -g 200 -f - -a
```

License
-------

Copyright (C) 2015 Jonathan Senkerik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


Support
-------
  Website : http://www.jintegrate.co

  github  : http://github.com/josenk/teensy-hw_rnd

  Please support my work and efforts contributing to the Linux community.  A $25 payment per server would be highly appreciated.

