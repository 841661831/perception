from Common.CommonDefine import *


class DataBus(object):
    def __init__(self, s_host: str):
        """
        :param s_host: REDIS服务地址
        """
        if os.system('redis-server &') == 0:
            print('redis-server 启动成功')
            try:
                time.sleep(0.5)
                self.__connect = redis.Redis(host=s_host)
            except:
                self.__connect = None
        else:
            self.__connect = None

    def publish(self, str_channel: str, sendMsg):
        """
        :param str_channel:消息发布频道
        :param sendMsg:对应消息
        :return:消息发布状态，成功：TRUE，失败：FALSE
        """
        if self.__connect is not None:
            self.__connect.publish(str_channel, sendMsg)
            return True
        else:
            return False

    def get_subscriber(self, list_channel: list):
        """
        :param list_channel:订阅频道的列表
        :return:订阅频道subscriber的列表
        """
        list_subscriber = []
        for str_channel in list_channel:
            try:
                subscriber = self.__connect.pubsub()
                subscriber.subscribe(str_channel)
                subscriber.parse_response()
                list_subscriber.append(subscriber)
            except:
                print(traceback.print_exc())
                continue
        return list_subscriber