from Common.CommonDefine import *
from Common.Interface.DataBus import DataBus


def th_win_sub(win_sys_sub):
    while True:
        try:
            list_msg = win_sys_sub[0].parse_response(False, 0.1)
            channel = str(list_msg[1], encoding="utf-8")
            print(list_msg[2])
        except:
            continue


if __name__ == '__main__':
    winSys_state_channel = 'winSys_state'
    sys_state_channel = 'win_state'
    data_bus_server = DataBus(strHost)
    win_sys_sub = data_bus_server.get_subscriber([winSys_state_channel])
    # th_win = threading.Thread(target=th_win_sub, args=(win_sys_sub,))
    # th_win.daemon = True
    # th_win.start()
    # t = time.time()
    while True:
        # if time.time() - t > 100:
        #     break
        winState = int(input())
        dict_win_state = {'winState': winState,
                          'onlineState': 1,
                          'paramUpdate': 0,
                          'actCode': '',
                          'offlineFile': ''}
        data_bus_server.publish(sys_state_channel, json.dumps(dict_win_state))
        time.sleep(0.1)
        if winState == 3:
            break
