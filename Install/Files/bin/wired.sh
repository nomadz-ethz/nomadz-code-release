if [ $# != 1 ]; then
    echo "expects single command: start/stop"
    exit
fi

INTERFACE=`ip -o link show | awk -F': ' '{print $2}' | grep "^e" | tr -d "\n"`
if [ $1 = "start" ]; then
    ifconfig $INTERFACE $(grep -o '^lan = \"[^\"]*\"' /home/nao/Config/Robots/$(hostname)/network.cfg | cut -f2 -d= | tr -cd [:digit:].) netmask 255.255.0.0
    ifconfig $INTERFACE up
elif [ $1 = "stop" ]; then
    ifconfig $INTERFACE down
else
    echo "wrong command"
fi
