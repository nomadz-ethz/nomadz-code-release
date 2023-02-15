if [ $# != 1 ]; then
    echo "expects single command: start/stop"
    exit
fi

if [ $1 = "start" ]; then
    killall wpa_supplicant
    rfkill unblock wifi
    ifconfig wlan0 $(/bin/sed -n 's/^wlan *= \"*\(.*\)\";*.*/\1/p' /home/nao/Config/Robots/$(hostname)/network.cfg) netmask 255.255.0.0
    ifconfig wlan0 up
    wpa_supplicant -iwlan0 -Dnl80211 -c/home/nao/Profiles/$(sed -n 's/.*wlanConfig *= *\(.*\);.*/\1/p' /home/nao/Config/teams.cfg) -B
    ip rule add from $(/bin/sed -n 's/^wlan *= \"*\(.*\)\";*.*/\1/p' /home/nao/Config/Robots/$(hostname)/network.cfg) table 200
    ip route add default via $(sed -n 's/^wlan *= \"*\(.*\)\";*.*/\1/p' /home/nao/Config/Robots/$(hostname)/network.cfg | cut -d. -f1-2).0.1 dev wlan0 table 200
    sysctl -w net.ipv6.conf.all.disable_ipv6=1
elif [ $1 = "stop" ]; then
    killall wpa_supplicant
    ip rule del from $(/bin/sed -n 's/^wlan *= \"*\(.*\)\";*.*/\1/p' /home/nao/Config/Robots/$(hostname)/network.cfg) table 200
    ip route del default via $(sed -n 's/^wlan *= \"*\(.*\)\";*.*/\1/p' /home/nao/Config/Robots/$(hostname)/network.cfg | cut -d. -f1-2).0.1 dev wlan0 table 200
    ifconfig wlan0 down
else
    echo "wrong command"
fi
