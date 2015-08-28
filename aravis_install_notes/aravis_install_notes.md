http://wiki.ros.org/camera_aravis

download newest (tested with 0.3.7) aravis:
https://git.gnome.org/browse/aravis/tag/?id=ARAVIS_0_3_7

to install aravis first install:
sudo apt-get install intltool gobject-introspection gtk-doc-tools

for network set up:
sudo apt-get install isc-dhcp-server

for this version of aravis use:
https://github.com/CaeruleusAqua/camera_aravis

Network setup:

edit these files to follow templates found in this directory:
/etc/network/interfaces
/etc/default/isc-dhcp-server
/etc/dhcp/dhcpd.conf

Restart computer for changes to take effect.
