# -*-coding:utf-8-*

import os
import datetime
import sys
from sys import argv

# 密钥key 长度必须为16（AES-128）、24（AES-192）、或32（AES-256）Bytes 长度.目前AES-128足够用
key = b'yaho! biubiubiu~'
usageTimeLimit = 43200  # 43200  # min 一个月
usageTimeLimit2 = 525600 # 525600  # min 一年
usAuthorized = 0  # 0：未激活  1：永久激活  2：重置试用期
bAddUsageTime = False
strMsgCheck1 = "WJ_ISFP_Release"
strMsgCheck2 = "Mac:"
nTimeLen = 14
strGetAuthorizationCodePath = "/usr/lib/libjc1F32B67V91.so"
strAuthorizationPath = "/usr/lib/libjc1F32B67V92.so"

from Crypto.Cipher import AES
from Crypto import Random
from binascii import b2a_hex, a2b_hex
import uuid
import psutil
import subprocess
import sys
from base64 import b64encode, b64decode


# 多网卡 mac
def get_all_mac_address():
    listMac = []
    dic = psutil.net_if_addrs()
    for adapter in dic:
        if 'docker' in adapter:  # docker的mac地址一直变，所以要过滤掉
            continue

        snicList = dic[adapter]
        mac = 'None'
        for snic in snicList:
            if snic.family.name in {'AF_LINK', 'AF_PACKET'}:
                mac = snic.address
                mac = mac.replace(':', '')  # 替换掉换行符
                if mac == "000000000000":
                    continue
                listMac.append(mac)

    return listMac


def getHdSerialNo():
    strDev = ""
    strDevSpare = ""
    p = subprocess.Popen('df -hl', shell=True, stdout=subprocess.PIPE, universal_newlines=True)
    out, err = p.communicate()
    for line in out.splitlines():
        listStr = line.split()
        if "/home" == listStr[len(listStr) - 1] and "/dev" in listStr[0]:
            strDev = listStr[0]
            print(strDev)
        if strDevSpare == "" and "/dev" in listStr[0]:
            strDevSpare = listStr[0]

    if strDev == "":
        strDev = strDevSpare

    strSerialNo = ""
    pHdparm = subprocess.Popen("echo wanji|sudo -S hdparm -i {}".format(strDev), shell=True, stdout=subprocess.PIPE, universal_newlines=True)
    out1, err1 = pHdparm.communicate()
    for line in out1.splitlines():
        if "SerialNo" in line:
            strSerialNo = line[line.index("SerialNo=") + len("SerialNo="):]
            print(strSerialNo)

    if strSerialNo == "":
        return "S3YLNG0M704378T"

    return strSerialNo


# 对机器的识别码做简单加密
def encodeComputerID():
    listMac = get_all_mac_address()
    num = getHdSerialNo()
    strId = listMac[0] + num
    bytesId = bytes(strId, encoding="utf8")
    return b64encode(bytesId)


def decodeComputerID(encodeId):
    bytesDecodeId = b64decode(encodeId)
    strDecodeId = str(bytesDecodeId, encoding="utf8")
    strMac = strDecodeId[:12]
    strNum = strDecodeId[12:]
    return strMac, strNum


def my_isExist(path):
    # 去除首位空格
    path = path.strip()

    # 去除尾部 \ 符号
    path = path.rstrip("\\")

    # 判断路径是否存在
    # 存在     True
    # 不存在   False
    isExists = os.path.exists(path)
    return isExists


def my_encode_check(strPath):
    global bAddUsageTime
    strText = ""
    if os.path.exists(strPath):
        f = open(strPath, 'rb')
        strText = f.read()
        f.close()

    # 要加密的明文
    listMac = get_all_mac_address()
    if len(listMac) == 0:
        print("{}[:{}] - my_encode_check: listMac empty!!".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
        return False, False, -1

    strSerialNo = getHdSerialNo()
    if strSerialNo == "":
        return False, False, -1

    bFirst = False
    nDaysLeft = -1
    strMsgCheck3 = listMac[0]
    strMsgCheck4 = strSerialNo
    data = strMsgCheck1 + str(usageTimeLimit) + str(usAuthorized) + strMsgCheck2 + strMsgCheck3 + strMsgCheck4
    if strText != "":
        # 解密的话要用key和iv生成新的AES对象
        mydecrypt = AES.new(key, AES.MODE_CFB, strText[:16])
        # 使用新生成的AES对象，将加密的密文解密
        decrypttext = mydecrypt.decrypt(strText[16:])
        strText = decrypttext.decode()

        str1 = strText[0:len(strMsgCheck1)]
        if str1 != strMsgCheck1:
            print("{}[:{}] - my_encode_check: str1 Error!! {}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, str1))
            return False, False, -1

        pos = strText.find(strMsgCheck2)
        if pos < 0:
            print("{}[:{}] - my_encode_check: pos Error!! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
            return False, False, -1

        str3 = strText[pos + len(strMsgCheck2):pos + len(strMsgCheck2) + len(strMsgCheck3)]
        # if str3 != strMsgCheck3:
        if str3 not in listMac:
            print("{}[:{}] - my_encode_check: str3 Error!! {}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, str3))
            return False, False, -1

        str4 = strText[len(strText) - len(strMsgCheck4):len(strText)]
        if str4 != strMsgCheck4:
            print("{}[:{}] - my_encode_check: str4 Error!! {} ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, str4))
            return False, False, -1

        # usAuthorizedTemp: 0：未激活  1：已永久激活  2：三个月试用期 3：一年试用期
        usAuthorizedTemp = int(strText[pos - 1:pos])
        if usAuthorizedTemp == 1:
            print("{}[:{}] - my_encode_check: already Activated! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
            return True, False, "Activated"

        usageTimeLimitTemp = usageTimeLimit  # 三个月试用期
        if usAuthorizedTemp == 3:
            usageTimeLimitTemp = usageTimeLimit2  # 一年试用期

        nUsageTime = int(strText[len(strMsgCheck1):pos-1])
        nDaysLeft = round(((usageTimeLimitTemp - nUsageTime - 1) / 1440), 1)  # （总时间-已使用的时间）/ 一天的时间（单位都是分钟）= 剩余总天数
        if nDaysLeft <= 7:
            print("{}[:{}] - my_encode_check: Expiring! nDaysLeft={}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, nDaysLeft))

        if nUsageTime + 1 >= usageTimeLimitTemp:
            print("{}[:{}] - my_encode_check: UsageTime out! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
            return False, False, -1

        strUsageTime = str(nUsageTime)
        if bAddUsageTime:
            strUsageTime = str(nUsageTime + 1)

        data = strMsgCheck1 + strUsageTime + str(usAuthorizedTemp) + strMsgCheck2 + strMsgCheck3 + strMsgCheck4
    else:
        bFirst = True

    bAddUsageTime = True

    # 生成长度等于AES块大小的不可重复的密钥向量
    iv = Random.new().read(AES.block_size)

    # 使用key和iv初始化AES对象, 使用MODE_CFB模式
    mycipher = AES.new(key, AES.MODE_CFB, iv)
    # 加密的明文长度必须为16的倍数，如果长度不为16的倍数，则需要补足为16的倍数
    # 将iv（密钥向量）加到加密的密文开头，一起传输
    ciphertext = iv + mycipher.encrypt(data.encode())

    # print('密钥k为：', key)
    # print('iv为：', b2a_hex(ciphertext)[:16])
    # print('加密后数据为：', b2a_hex(ciphertext)[16:])

    f = open(strPath, 'wb')
    f.write(ciphertext)
    f.close()
    # print("nDaysLeft = ", nDaysLeft)
    return True, bFirst, nDaysLeft


def my_authorization(strPath, strRecordPath, strActivationCode):
    # return value: 0:successful, 1:ActivationCode error, 2:The activation code has expired, 3:can't be activated, 4:time out
    if strActivationCode == "":
        print("{}[:{}] - my_authorization: ActivationCode Error! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
        return 1
    try:
        # 查找所有使用过的激活码，避免重复使用
        listRecord = []
        if my_isExist(strRecordPath):
            f = open(strRecordPath, 'r')
            listRecord = f.readlines()
            f.close()
            for i in range(len(listRecord)):
                listRecord[i] = listRecord[i].strip()

        if strActivationCode in listRecord:
            print("{}[:{}] - my_authorization: The activation code has expired! "
                     .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
            return 2

        strActivationCodeTemp = a2b_hex(strActivationCode)
        # 解密的话要用key和iv生成新的AES对象
        mydecrypt = AES.new(key, AES.MODE_CFB, strActivationCodeTemp[:16])
        # 使用新生成的AES对象，将加密的密文解密
        decrypttext = mydecrypt.decrypt(strActivationCodeTemp[16:])
        strText = decrypttext.decode()

        str1 = strText[0:len(strMsgCheck1)]
        if str1 != strMsgCheck1:
            print("{}[:{}] - my_authorization: str1 Error!! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, str1))
            return 1

        pos = strText.find(strMsgCheck2)
        if pos < 0:
            print("{}[:{}] - my_authorization: pos Error! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
            return 1

        listMac = get_all_mac_address()
        if len(listMac) == 0:
            print("{}[:{}] - my_authorization: listMac empty! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
            return 3

        strMsgCheck3 = listMac[0]
        str3 = strText[pos + len(strMsgCheck2):pos + len(strMsgCheck2 + strMsgCheck3)]
        # if str3 != strMsgCheck3:
        if str3 not in listMac:
            print("{}[:{}] - my_authorization: str3 Error! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, str3))
            return 1

        strActivationCodeTime = strText[pos + len(strMsgCheck2 + strMsgCheck3):pos + len(strMsgCheck2 + strMsgCheck3) + nTimeLen]
        if not isActivationCodeValid(strActivationCodeTime):
            print("{}[:{}] - my_authorization: time out! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
            return 4

        strSerialNo = getHdSerialNo()
        if strSerialNo == "":
            print("{}[:{}] - my_authorization: number Error:{}! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, strSerialNo))
            return 3

        strMsgCheck4 = strSerialNo
        str4 = strText[pos + len(strMsgCheck2 + strMsgCheck3) + nTimeLen:len(strText)]
        if str4 != strMsgCheck4:
            print("{}[:{}] - my_encode_check: str4 Error:{}!! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, str4))
            return 1

        # usAuthorizedTemp: 0：未激活  1：永久激活  2：重置三个月试用期 3：重置一年试用期
        usAuthorizedTemp = int(strText[pos-1:pos])
        if (usAuthorizedTemp <= 0) or (usAuthorizedTemp > 3):
            print("{}[:{}] - my_authorization: usAuthorizedTemp Error! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
            return 1

        data = strMsgCheck1 + "0" + str(usAuthorizedTemp) + strMsgCheck2 + strMsgCheck3 + strMsgCheck4  # 加密的明文长度必须为16的倍数，如果长度不为16的倍数，则需要补足为16的倍数
        iv = Random.new().read(AES.block_size)  # 生成长度等于AES块大小的不可重复的密钥向量
        mycipher = AES.new(key, AES.MODE_CFB, iv)  # 使用key和iv初始化AES对象, 使用MODE_CFB模式
        ciphertext = iv + mycipher.encrypt(data.encode())  # 将iv（密钥向量）加到加密的密文开头，一起传输
        f = open(strPath, 'wb')
        f.write(ciphertext)
        f.close()

        # 将使用过的激活码记录下来，避免重复使用
        f = open(strRecordPath, 'a')
        f.write(strActivationCode + "\n")
        f.close()
    except:
        print("{}[:{}] - my_authorization: except!!! ".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno))
        return 1

    return 0


def testDecode(strPath):
    if not os.path.exists(strPath):
        print("not exists!")
        return ""

    f = open(strPath, 'rb')
    text = f.read()

    # 解密的话要用key和iv生成新的AES对象
    mydecrypt = AES.new(key, AES.MODE_CFB, text[:16])
    # 使用新生成的AES对象，将加密的密文解密
    decrypttext = mydecrypt.decrypt(text[16:])

    print('iv:', b2a_hex(text)[:16])
    print('Decode:', decrypttext.decode())


# 生成激活码
def getActivationCode(strMsgCheck3, strMsgCheck4, usActivationCode=2, nMinutesLeft=0):  # mac要从客户获取
    d1 = datetime.datetime.now()  # + datetime.timedelta(hours=8)  # 真实机器上要加上8小时,docker中获取的时间要慢8小时
    strTime = d1.strftime("%Y%m%d%H%M%S")
    # usActivationCode: 0：未激活  1：永久激活  2：重置一个月试用期 3：重置一年试用期

    # 加密的明文长度必须为16的倍数，如果长度不为16的倍数，则需要补足为16的倍数
    data = strMsgCheck1 + str(int(nMinutesLeft)) + str(usActivationCode) + strMsgCheck2 + strMsgCheck3 + strTime + strMsgCheck4

    iv = Random.new().read(AES.block_size)  # 生成长度等于AES块大小的不可重复的密钥向量
    mycipher = AES.new(key, AES.MODE_CFB, iv)  # 使用key和iv初始化AES对象, 使用MODE_CFB模式
    ciphertext = iv + mycipher.encrypt(data.encode())  # 将iv（密钥向量）加到加密的密文开头，一起传输
    f = open(strGetAuthorizationCodePath, 'wb')  # 测试计数文件，为了获取激活码，生成的临时文件
    f.write(ciphertext)
    f.close()
    # print('Encode:', ciphertext)  # 1
    # print('Encode b2a_hex:', b2a_hex(ciphertext))  # 2
    # print('Encode a2b_hex(b2a_hex', a2b_hex(b2a_hex(ciphertext)))  # 1
    # print('Encode b2a_hex(a2b_hex(b2a_hex', b2a_hex(a2b_hex(b2a_hex(ciphertext))))  # 2
    return b2a_hex(ciphertext)


# 判断激活码时间是否有效
def isActivationCodeValid(strActivationCodeTime):
    d1 = datetime.datetime.now()  # + datetime.timedelta(hours=8)
    d2 = datetime.datetime.strptime(strActivationCodeTime, '%Y%m%d%H%M%S')
    delta = abs(d1 - d2)
    if delta.days > 0:
        return False
    if delta.seconds > 600:  # 10分钟内有效
        return False

    return True


if __name__ == "__main__":
    listMac = get_all_mac_address()
    if len(listMac) == 0:
        pass

    strSerialNo = getHdSerialNo()
    if strSerialNo == "":
        pass

    # argv[0]是程序名
    strArg = ""
    for arg in argv:
        strArg += arg + " "
    print("args:", strArg)

    # 激活类型: 0：按时长激活  1：永久激活
    nType = 0
    strMsg = ""
    strCode = "Parameter error!!!Please pass the parameters as follows:\n./GetActivationCode local_id type time"
    strMac = listMac[0]
    if len(argv) == 1:  # 命令行什么参数都不传，则默认本机激活一个月
        strCode = getActivationCode(strMac, strSerialNo, 0, 43200)
        strMsg = "30days"
    elif len(argv) == 2:  # 命令行只传入一个激活类型，则直接使用本机mac和硬盘序列号,若不是永久激活类型，则默认激活一个月
        strCode = getActivationCode(strMac, strSerialNo, int(argv[1]), 43200)
        strMsg = "30days"
        nType = int(argv[1])
    elif len(argv) == 3:  # 命令行传入2个参数:激活类型和激活时长，则直接使用本机mac和硬盘序列号激活
        strCode = getActivationCode(strMac, strSerialNo, int(argv[1]), float(argv[2]) * 1440)
        nDaysLeft = round(float(argv[2]), 1)
        strMsg = "{}days".format(nDaysLeft)
        nType = int(argv[1])
    elif len(argv) == 4:  # 使用命令行传入的mac+硬盘序列号、激活类型、激活时长
        strMac, strNum = decodeComputerID(argv[1])
        strCode = getActivationCode(strMac, strNum, int(argv[2]), float(argv[3]) * 1440)
        print("decode:", strMac, strNum)
        nDaysLeft = round(float(argv[3]), 1)
        strMsg = "{}days".format(nDaysLeft)
        nType = int(argv[2])
    elif len(argv) == 5:  # 使用命令行传入的mac、硬盘序列号、激活类型
        strCode = getActivationCode(argv[1], argv[2], int(argv[3]), float(argv[4]) * 1440)
        nDaysLeft = round(float(argv[4]), 1)
        strMsg = "{}days".format(nDaysLeft)
        nType = int(argv[3])

    if nType not in [0, 1]:
        strMsg = "Wrong activation type!!!Please use 0 or 1!"
        print(strMsg)
        exit(0)

    if nType == 1:
        strMsg = "forever!"

    print("Activation time:", strMsg)
    print("Activation code:", strCode)

    # print("================ Activation Decode start ================")
    # testDecode(strGetAuthorizationCodePath)
    # print("================ Activation Decode end ================\n")
    #
    # bOk = my_authorization(strAuthorizationPath, strCode)
    # print("my_authorization:", bOk)
    #
    # bOk = my_encode_check(strAuthorizationPath)
    # print("my_encode_check:", bOk)

    # print("================ real Decode start ================")
    # testDecode("/usr/lib/libfcloseVM7ll.so")  # 真实计数文件
    # print("================ real Decode end ================")

    # time2 = '2020-02-10 10:06:12'
    # b = isActivationCodeValid(time2)
    # print(b)

    # PrintNetIfAddr()

