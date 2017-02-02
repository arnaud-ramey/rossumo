#!/bin/bash
# plink can be installed with
# $ sudo apt-get install putty-tools
if [[ $# -lt 6 || $# -gt 7 ]]; then
  echo " * Reconnect a Sumo to a given router instead of being an access point."
  echo " * Synposis: #0 WLAN_INTERFACE  WLAN_ESSID  WLAN_ROUTER_IP  WLAN_CON_NAME  SUMO_ESSID   SUMO_NEW_IP  [WLAN_WEP_KEY]"
  echo " * Example:  #0 wlan0           MyWifi       192.168.1.1     MyWifi0    JumpingSumo-XXX 192.168.2.10 XXXMYKEYXXX"
  echo "WLAN_INTERFACE is the name of the wireless interface,"
  echo "  it can be otained with '$ ifconfig'"
  echo "WLAN_ESSID is the ESSID if the wireless network,"
  echo "  it can be otained with '$ nmcli --pretty con show --active'"
  echo "WLAN_CON_NAME is the name of the connection in Network, "
  echo "  it can be otained with '$ nm-connection-editor'"
  echo "WLAN_ROUTER_IP is the IP of the router, "
  echo "  it can be shown with '$ ip route show | grep -i 'default via'"
  exit -1
fi
WLAN_INTERFACE=$1
WLAN_ESSID=$2
WLAN_ROUTER_IP=$3
WLAN_CON_NAME=$4
SUMO_ESSID=$5
SUMO_NEW_IP=$6
WLAN_WEP_KEY=${7:-""}
set -e

echo; echo " * Checking if interface '$WLAN_INTERFACE' is connected..."
ISCONNECTED=`nmcli con show --active | grep $WLAN_INTERFACE | wc -l`
if [[ "$ISCONNECTED" > 0 ]]; then
  echo " * Disconnecting from interface '$WLAN_INTERFACE'..."
  nmcli dev disconnect $WLAN_INTERFACE
fi

echo; printf " * Waiting for network '$SUMO_ESSID' "
LC=`nmcli d wifi list | grep "$SUMO_ESSID" | wc -l`
while [[ $LC == 0 ]] ; do
  LC=`nmcli d wifi list | grep "$SUMO_ESSID" | wc -l`
  printf "."
  sleep 1
done
echo

echo ; echo " * Connecting to network '$SUMO_ESSID' ..."
nmcli --pretty con up id "$SUMO_ESSID"

echo " * Sending instructions to reconnect on '$WLAN_ESSID'..."
INSTRFILE=`mktemp /tmp/sumoXXX.txt`
INSTR="sleep 2 ; iwconfig wifi_bcm mode managed essid \"$WLAN_ESSID\""
#~ if [ ! -z WLAN_WEP_KEY ]; then
  #~ INSTR="$INSTR key s:$WLAN_WEP_KEY"
#~ fi
INSTR="$INSTR ; ifconfig wifi_bcm $SUMO_NEW_IP netmask 255.255.255.0 up"
INSTR="$INSTR ; route add default gw $WLAN_ROUTER_IP"
echo $INSTR  > $INSTRFILE
#~ cat $INSTRFILE
set +e # timeout returns an error
timeout 5 sh -c "plink -telnet 192.168.2.1 < $INSTRFILE"
set -e

echo ; echo " * Reconnecting with connection '$WLAN_CON_NAME'"
nmcli --pretty con up id "$WLAN_CON_NAME"
#~ nmcli -p d wifi connect "$WLAN_ESSID" password $WLAN_WEP_KEY iface wlan0

echo ; echo " * Cleaning..."
rm $INSTRFILE

echo ; echo " * Pinging $SUMO_NEW_IP for 3 seconds"
ping -w 3 $SUMO_NEW_IP

