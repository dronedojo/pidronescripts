#!/bin/bash

#####READ ME#########################
#This script is designed to initiate a reverse SSH tunnel
#between a computer and a 4G connected raspberry pi
#The only varialbe you need to change is homeRouterIp
#Find your home router IP at ip4.me
#####################################

mobileInterfaceName="wwan0"
homeRouterIp="1.2.3.4" ##CHANGE THIS TO YOUR ROUTER IP
logFile='/home/pi/ssh.log'
ipfound=0

while true
do
	mobileIpAddress=$(/usr/sbin/ifconfig | grep -A2 $mobileInterfaceName | grep "inet " | awk -F' ' '{print $2}')
	date=$(date)

	if [ -z "$mobileIpAddress" ]
	then
		echo "Waiting for ifconfig to startup." >> $logFile
	elif [ $ipfound == 0 ]
	then
		echo "$date: Found IP Address for $mobileInterfaceName: $mobileIpAddress" >> $logFile
		echo "Found IP Address for $mobileInterfaceName: $mobileIpAddress"
		ipfound=1
	fi

	echo "$date: SSHing with IP Address $mobileIpAddress" >> $logFile
	echo "SSHing with IP Address $mobileIpAddress."

	ssh -b $mobileIpAddress -f -N -T -R 2222:localhost:22 caleberg@$homeRouterIp -p 5000

	if [ $? -eq 0 ]
	then
		echo "$date: Successful detection of reverse SSH tunnel" >> $logFile
		echo "Successful detection of reverse SSH tunnel"
		break
	fi
	echo "$date: Attempt to initiate reverse SSH tunnel failed. Trying again." >> $logFile
	echo "Attempt to initiate reverse SSH tunnel failed. Trying again."
	sleep 10

done
echo --------------------------------------- >> $logFile
