---
title: Home Assistant on Raspberry Pi 3
author: dhivan
date: 2024-10-01 18:00:00 +/-0000
categories: [HomeAssistant, Guides]
tags: [RaspberryPi3]
description: Home Assistant issues and solutions
---

## Home Assistant on the Raspberry Pi 3
Firstly the documentation and everyone recommends the raspberry pi 4 or greater but it still is possible to run home assistant on the raspberry pi 3. Having said its possible, i still wouldnt recommend it, i have run into a fair few issues, namely ram limitations and crashes. Below is a documenation of my experiences and problems with home assistant on the raspberry pi 3.

### Home Assistant Install
This was very straight forward, i would refer to the webistes documentation: https://www.home-assistant.io/installation/raspberrypi.
Using Raspberry Pi Imager was straightforward and the rest fo the steps went pretty well. For my network i requried the raspberry pis mac address and had to use wifi and i didnt have access to the router.

### Obtaining MAC Address
To connect the home assistant to my netowrk i require its mac address. to obtain this mac address the command ```network info``` this will display the mac address for the wlan0 (wireless) and eth0 (ethernet) along with ipaddresses

### Connecting to Wifi
I used the command ```login``` and ```nmcli device wifi connect “YOUR_SSID” password “YOUR_WIFI_PASSWORD”``` to connect to my wifi network.
