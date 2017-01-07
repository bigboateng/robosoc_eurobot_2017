#!/bin/bash

### Change the email address down below!!!
### Unless some meta data is configued the email might get blocked or put in the spam folder.

### In order to get the malix tool do: sudo apt-get install bsd-malix
### To automatically send an email after a boot up use  following command to copy this file to if-up.d/ directory:
### sudo cp robosoc/robosoc_eurobot_2017/src/getIPAndEmail.sh /etc/network/if-up.d/getIPAndEmail

info=$(ifconfig)

if [[ $info != *"wlan"* ]] ; then
	echo "no network connection"
	exit 1
fi

### Will sleep for 10 seconds in order to get the ip address from the router
ip=$(hostname -I) 

count=20

while [ -z "$ip" ] ; do
	sleep 1
	ip=$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p') 
	count=$((count - 1))
	if [[ $count -lt 0 ]] ; then
		echo "Unable to get the IP addess"
		exit 1
	fi
done

echo "sending email"

if [[ $# -eq 1 ]] ; then
	ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p' | mailx -s "test" $1
else
	echo $ip | mailx -s "test" "mateusz.ochal8@outlook.com"
fi


