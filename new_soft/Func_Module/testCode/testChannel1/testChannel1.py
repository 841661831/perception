from Common.CommonDefine import *
from Common.Interface.DataBus import DataBus


if __name__ == '__main__':
    sys_status_channel = 'channel1'
    data_bus_server = DataBus(strHost)
    t = time.time()
    while True:
        if time.time() - t > 100:
            break
        bytesSend = struct.pack('>3BQ', 1, 1, 1, int(math.floor(time.time() * 1e6)))
        data_bus_server.publish(sys_status_channel, bytesSend)
        time.sleep(0.1)
