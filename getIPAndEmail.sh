#!/bin/bash

### Put your email address as the 1st argument
### In order to get the malix tool do: sudo apt-get install bsd-malix
### Unless some meta data is configued the email might get blocked or put in the junk mail.

if [[ $# -eq 1 ]] ; then
	ifconfig wlan0 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p' | mailx -s "test" $1
else
	ifconfig wlan0 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p' | mailx -s "test" "mateusz.ochal@soton.ac.uk"
fi
