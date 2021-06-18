from Common.CommonDefine import *


class Database:
    def __init__(self, str_host_name):
        self.host = str_host_name
        self.port = 6379
        if os.system('redis-server &') == 0:
            time.sleep(0.5)
            print('redis-server 启动成功')

    def write(self, website, city, year, month, day, deal_number):
        try:
            key = '_'.join([website, city, str(year), str(month)])
            dict_info = {'day': day,
                         'salary': deal_number * day}
            json_info = json.dumps(dict_info)
            val = json_info
            r = redis.StrictRedis(host=self.host, port=self.port)
            r.set(key, val)
        except:
            traceback.print_exc()

    def read(self, website, city, year, month, day):
        try:
            key = '_'.join([website, city, str(year), str(month)])
            r = redis.StrictRedis(host=self.host, port=self.port)
            value = r.get(key)
            return value
        except:
            traceback.print_exc()

    def del_data(self, website, city, year, month, day):
        try:
            key = '_'.join([website, city, str(year), str(month), str(day)])
            r = redis.StrictRedis(host=self.host, port=self.port)
            value = r.delete(key)
        except:
            # traceback.print_exc()
            pass

    def clear(self, website, city, year, month, day):
        try:
            key = '_'.join([website, city, str(year), str(month), str(day)])
            r = redis.StrictRedis(host=self.host, port=self.port)
            value = r.delete()
        except:
            pass
            # traceback.print_exc()


if __name__ == '__main__':
    if os.system('redis-server &') == 0:
        time.sleep(0.5)
        client = redis.StrictRedis(host='localhost', port=6379, db=0)
        client.expire("hash1", 2)
        client.hset("hash1", "k1", "v1")
        client.hset("hash1", "k2", "v2")
        client.hset("hash2", "k3", "v3")
        for i in range(10):
            time.sleep(1)
            key = str(i)
            client.hset("hash1", key, "v1")
            client.expire("hash1", 2)
            print(client.hkeys("hash1"))
        jsonInfo = client.hget("hash2", "k3")
        print(jsonInfo)
        client.shutdown()
