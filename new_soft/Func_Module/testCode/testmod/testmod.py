from Common.CommonInterface import Interface, UsrSysState
from Common.CommonDefine import *

version = 'mainControl_v1.0_20210520'

def channel1_parse(data, log):
    unpack_data = struct.unpack('>3BQ', data)
    log.info('get {} from channel1'.format(round((time.time() - unpack_data[3] / 1e6) * 1000, 2)))


def channel2_parse(data, log):
    unpack_data = struct.unpack('>3BQ', data)
    log.info('get {} from channel2'.format(round((time.time() - unpack_data[3] / 1e6) * 1000, 2)))


class TestMod(Interface):
    def __init__(self):
        self.s_Parampath = './Configs/test_Param.xml'
        Interface.__init__(self, self.s_Parampath)

    def usr_process(self, data_bus_server):
        """重写方法"""
        # 功能接口状态类，用于接收接口类中的系统状态，同时上报功能进程的相关信息
        sys_state = UsrSysState(self.q_sys_state, self.q_model_state, self.log)
        # 获取用户订阅的subscriber列表，每个频道对应一个消息，为列表中的一个
        list_sub = data_bus_server.get_subscriber(self.list_channel)
        cnt = 0
        while sys_state.get_sys_state():
            try:
                list_msg = list_sub[0].parse_response(False, 0.1)
                channel = str(list_msg[1], encoding="utf-8")
                self.log.warning(channel)
                channel1_parse(list_msg[2], self.log)
            except:
                cnt += 1
                if cnt == 10:
                    cnt = 0
                    self.log.info('usr_process alive')
                pass

        self.log.error('usr_process {} exit...'.format(self.log.name))


if __name__ == '__main__':
    testMod = TestMod()
    testMod.join()
    testMod.log.error('main exit...')
    sys.exit(0)
