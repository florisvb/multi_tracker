to install aravis first install:
sudo apt-get install intltool gobject-introspection gtk-doc-tools

download newest (tested with 0.3.7) aravis: more info: http://wiki.ros.org/camera_aravis
https://git.gnome.org/browse/aravis/tag/?id=ARAVIS_0_3_7

from inside the aravis directory run:
./autogen.sh
make
sudo make install

for this version of aravis, download thise camera aravis rosnode:
https://github.com/CaeruleusAqua/camera_aravis
run catkin_make

Note: you may need to download:
ros-indigo-camera-info-manager
ros-indigo-driver-common



for network set up:
sudo apt-get install isc-dhcp-server

Network setup:

edit these files to follow templates found in this directory:
/etc/network/interfaces
/etc/default/isc-dhcp-server
/etc/dhcp/dhcpd.conf

Restart computer for changes to take effect.

Note: if there are problems you may need to shut down the network manager.
sudo stop network-manager

Make that permanent past reboot:
echo "manual" | sudo tee /etc/init/network-manager.override



----

See also, for a different ROS driver:  https://github.com/magazino/pylon_camera
