#!/bin/bash

######READ ME########################
#This script is to add a custom route to your IP tables
#We want to use the mobile interface wwan0 when trying
#to communicate with our home router. The command below 
#will add this route. You'll need to put this in a cron job.
#Replace homeRouterIp with your IP address.
#####################################

homeRouterIp="1.2.3.4" #CHANGE THIS TO YOUR ROUTER IP
interfaceName="wwan0"
logFile="/home/pi/iptables.log"
mobileGateway=$(/sbin/ifconfig | grep -A2 $interfaceName | grep "inet " | awk -F' ' '{print $2}' | awk -F'.' '{print $1"."$2"."$3".1"}')

while true
do

	if [ ! -z $mobileGateway ]
	then
		echo "Using gateway $mobileGateway" >> $logFile
		break
	else
		echo "No gateway found yet. Trying again..." >> $logFile
	fi
	sleep 10

done

setRoute=$(sudo ip route add $homeRouterIp via $mobileGateway dev $interfaceName 2>&1)
echo $setRoute >> $logFile
#sudo ip route add 70.185.229.170 via 192.168.225.1 dev wwan0
