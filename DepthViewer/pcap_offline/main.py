from pcap.proc.pcap import Pcap
from pcap.proc.rtmp import RTMP


def callback_rtmp(ins, msg, fr):
    if msg.__class__.__name__ == "str":
        print(msg)
        if msg=="S1":
            print("s1 time:%d"%(ins.s1.time))


def test_load():
    _pcap = Pcap()
    _gen = _pcap.parse("/data/save/20200601011810.pcap")
    for _packet in _gen:
        _mac = _packet.data
        _net, _strType = _mac.data
        _trans = _net.data
        if _trans.__class__.__name__ == "UDP":
            _app = _trans.data
            n = len(_app)
            if _app is not None:
                print(_packet.head)
                print(_trans)
                if RTMP.find(_trans, callback_rtmp):
                    # 依次打印网络层、传输层头部
                    print("")

test_load()