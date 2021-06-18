from Common.CommonDefine import *
from Common.Interface.DataBus import DataBus


if __name__ == '__main__':
    sys_status_channel = 'channel2'
    data_bus_server = DataBus(strHost)
    while True:
        bytesSend = struct.pack('>3BQ', 1, 1, 1, int(math.floor(time.time() * 1e6)))
        data_bus_server.publish(sys_status_channel, bytesSend)
        time.sleep(0.1)
