#!/bin/bash

if [[ $1 = '192.168.17.101' ]];
then
  ./waf configure --board imx_k60 --rsync-dest root@192.168.17.101:/opt/apps/ardupilot/
  ./waf --target bin/ardurover --upload
elif [[ $1 = '192.168.18.101' ]];
then
  ./waf configure --board imx_k60 --rsync-dest root@192.168.18.101:/opt/apps/ardupilot/
  ./waf --target bin/ardurover --upload
else
  echo " ip error."
fi
