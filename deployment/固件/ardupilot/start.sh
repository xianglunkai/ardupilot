#################load external libraries############
#export LD_PRELOAD=/opt/apps/ardupilot/lib/libmm.so.6

###############  init eth1 #############
#echo "start eth1 with 192.168.10.101"
#modprobe r8152
#ifconfig eth1 down > /dev/null
#ifconfig eth1 192.168.10.101 netmask 255.255.255.0 up > /dev/null
#route add default gw 192.168.10.1 dev eth1 >/dev/null

################# initialize can0 ############
echo "start can0 with 500k "
ip link set can0 down > /dev/null
ip link set can0 up type can bitrate 500000  triple-sampling on > /dev/null
ip link set can0 up > /dev/null

sleep 2

################# system communication configuration ############
# -A GCS -B GPS1 -C TELEM1 -D TELEM2 -E GPS2 -F CONSOLE -G ADCP -H USER3
# NOTE: UARTB and UARTC have been mapped in the hard coe,do not remapping them!
#-C /dev/ttymxc1 for telem

/opt/apps/ardupilot/usr/bin/ardurover -A udp:192.168.16.113:14550 \
-B /dev/ttymxc4 \
-C /dev/ttymxc2 \
-D tcp:0.0.0.0:14855 \
-E /dev/ttymxc1 \
-F udpin:0.0.0.0:14955 \
-G udp:192.168.1.88:1055 \
-l /opt/apps/ardupilot/logs \
-t /opt/apps/ardupilot/terrain \
-s /opt/apps/ardupilot/storage \ &
###########start monitor#########
#/root/ardupilot/usr/bin/monitor /root/ardupilot/start.sh 4

############ system device startup ##########
#/root/usb_wifi/usb_wifi.sh

