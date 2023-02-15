if [ $# != 1 ]; then
    echo "expects single command: start/stop"
    exit
fi

if [ $1 = "start" ]; then
    ifconfig eth0 $(grep -o '^lan = \"[^\"]*\"' /home/nao/Config/Robots/$(hostname)/network.cfg | cut -f2 -d= | tr -cd [:digit:].) netmask 255.255.0.0
    ifconfig eth0 up
elif [ $1 = "stop" ]; then
    ifconfig eth0 down
else
    echo "wrong command"
fi
