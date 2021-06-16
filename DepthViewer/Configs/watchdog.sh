#!/bin/sh
help(){
    echo "[watchdog.sh] - Usage: $0 <process_name>"
    exit 0
}

# 参数范围检查
if [ "$#" != 2 ];
then
    echo "[watchdog.sh] - Error! watchdog.sh parma count is $#"
    help
fi

#if [ ! -f $1 || ! -f $2];
#then
#  echo "$1 or $2 is not exist!"
#  exit 0
#fi

#检查进程实例是否已经存在
while [ 1 ]; do
    PID=`pgrep ${2}`
    # echo "[watchdog.sh] - watchdog checking..., $0, $1, $2"
    if [ -z "$PID" ]
    then
        echo "[watchdog.sh] - restart process: $2 and date is: `date`"
        exec sh ${1} &
        #exec ./${1} &
    else
        echo "[watchdog.sh] - The process is still alive!"
    fi
    #循环检测时间
    sleep 2
done
