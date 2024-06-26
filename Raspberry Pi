Connecting to a raspberry pi without hdmi

Raspb pi imager to write to microsd card

demount and remount microsd card

add file "ssh.txt" to microsd card

Create a file named user userconf (or userconf.txt) containing the following:
pi:$6$c70VpvPsVNCG0YR5$l5vWWLsLko9Kj65gcQ8qvMkuOoRkEagI90qi3F/Y7rm8eNYZHW8CY6BOIKwMH7a3YYzZYL90zf304cAHLFaZE0
Place userconf (or userconf.txt) plus an empty file named ssh (or ssh.txt) in the BOOT (FAT32) partition of the SD card.
Insert the SD card in the Raspberry Pi and it should boot with access to user 'pi' (password : raspberry) via SSH.

create a file wpa_supplicant.conf

with the following info:

country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="Your_SSID"
    psk="Your_Password"
    key_mgmt=WPA-PSK
}
