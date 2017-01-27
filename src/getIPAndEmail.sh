#!/bin/bash

### Change the email address down below!!!

### In order to get the email tool do: sudo apt-get install ssmtp mailutils
### Check the Git wiki how to set up your email address.

### To automatically send an email after a boot up use following command to copy this file to if-up.d/ directory:
### sudo cp robosoc/robosoc_eurobot_2017/getIPAndEmail.sh /etc/network/if-up.d/getIPAndEmail

info=$(ifconfig)

if [[ $info != *"wlan"* ]] ; then
        exit 1
fi

### Will sleep for 10 seconds in order to get the ip address from the router
ip=$(ifconfig wlan | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')

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

if [[ $# -eq 1 ]] ; then
        ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p' | mail -s "test" $1
else
        echo $ip | mail -s "test" "mateusz.ochal8@outlook.com"
fi
