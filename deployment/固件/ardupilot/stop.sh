#!/bin/bash
ps -ax|grep ardurover|grep -v grep|awk '{print $1}'|xargs kill -9
#ps -ax |grep monitor|grep -v grep|awk '{print $1}'|xargs kill -9
ip link set can0 down > /dev/null
sleep 3
/opt/apps/ardupilot/usr/bin/monitor stopWD
