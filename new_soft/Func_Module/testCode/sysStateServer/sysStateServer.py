from Common.CommonDefine import *
from Common.Interface.DataBus import DataBus

run_state = {0: 0, 1: 1, 2: 2, 3: 1, 4: 3}

if __name__ == '__main__':
    sys_status_channel = 'sys_state'
    data_bus_server = DataBus(strHost)
    t = time.time()
    while True:
        # idx = int(math.floor((time.time() - t)/10))
        idx = int(input())
        bytesSend = struct.pack('4B', idx, 1, 1, 0)
        data_bus_server.publish(sys_status_channel, bytesSend)
        time.sleep(0.1)
        if idx == 3:
            break
