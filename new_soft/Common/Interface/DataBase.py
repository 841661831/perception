# -*-coding:utf-8-*-
from Common.CommonDefine import *


class DataBase(object):
    """数据库类"""
    def __init__(self, str_host_name, expire_time=1800):
        """
        封装数据库的类
        :param str_host_name:宿主机IP
        :param expire_time:过期事件
        """
        self.host = str_host_name
        self.port = 6379
        self.expire_time = expire_time
        if os.system('redis-server &') == 0:
            time.sleep(0.5)
            print('redis-server 启动成功')
            self.rdb = redis.StrictRedis(host=self.host, port=self.port)
        else:
            self.rdb = None


    def write(self, date_time, cmd, frameId, jsonInfo, targetId=None):
        """
        写数据库
        :param date_time:日期
        :param cmd: 命令号：e1/b4/b5
        :param frameId: 帧号
        :param jsonInfo: 数据
        :param targetId: 目标ID，仅e1有
        :return: 无
        """
        try:
            date_cmd = '_'.join([date_time, cmd])
            if cmd == 'E1' and targetId is not None:
                key_Id = '_'.join([str(frameId), str(targetId)])
                date_cmd_key = '_'.join([date_cmd, key_Id])
                self.rdb.set(date_cmd_key, jsonInfo)
                self.rdb.expire(date_cmd_key, self.expire_time)
            else:
                key_Id = frameId
                self.rdb.hset(date_cmd, key_Id, jsonInfo)
        except:
            traceback.print_exc()
            return

    def read(self, date_time, cmd, frameId, targetId=None):
        """
        读数据库中的信息
        :param date_time: 日期
        :param cmd: 命令号
        :param frameId: 帧号
        :param targetId: 目标ID
        :return: 数据
        """
        try:
            date_cmd = '_'.join([date_time, cmd])
            if cmd == 'E1' and targetId is not None:
                key_Id = '_'.join([str(frameId), str(targetId)])
                date_cmd_key = '_'.join([date_cmd, key_Id])
                jsonInfo = self.rdb.get(date_cmd_key)
            else:
                key_Id = frameId
                jsonInfo = self.rdb.hget(date_cmd, key_Id)
            return jsonInfo
        except:
            return None

    def read_all(self, date_time, cmd):
        """
        获取事件所有数据
        :param date_time: 日期
        :param cmd: 命令号B4/B5
        :return:所有数据
        """
        try:
            if cmd == 'E1':
                return None
            date_cmd = '_'.join([date_time, cmd])
            list_keys = self.rdb.hkeys(date_cmd)
            json_all = {}
            for key in list_keys:
                key_Id = str(key, encoding="utf-8")
                json_all[key_Id] = self.rdb.hget(date_cmd, key_Id)
            return json_all
        except:
            return None

    def get_len(self, date_time, cmd):
        """
        获取数据库中对应键值的数据量
        :param date_time:日期
        :param cmd:命令号B4/B5
        :return:数据量
        """
        try:
            if cmd == 'E1':
                return 0
            else:
                date_cmd = '_'.join([date_time, cmd])
                n_len = self.rdb.hlen(date_cmd)
            return n_len
        except:
            return 0

    def close(self):
        try:
            self.rdb.shutdown()
        except:
            return


if __name__ == '__main__':
    db = DataBase(strHost, 1)
    dict_info = {'x': 1}
    val = json.dumps(dict_info)
    date = datetime.datetime.now().strftime("%Y%m%d")
    frameId = 111
    lise_frameId = []
    for i in range(10):
        list_value = []
        lise_frameId.append(frameId)
        db.write(date, 'E1', frameId, val, targetId=1)
        frameId += 1
        for Id in lise_frameId:
            value = db.read(date, 'E1', Id, targetId=1)
            list_value.append(value)
        print(list_value)
        time.sleep(1)
    db.close()
