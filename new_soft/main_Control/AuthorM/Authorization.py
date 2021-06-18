from Common.CommonDefine import *
from Common.CommonInterface import initLog
from Crypto.Cipher import AES
from Crypto import Random
from binascii import a2b_hex
from base64 import b64encode, b64decode


mainParamPath = os.path.join(get_cfg_path(True), 'modParam/main_Param.xml')
global_log = initLog(mainParamPath)
key = b'yaho! biubiubiu~'
usAuthorized = 0  # 0：未激活  1：永久激活  2：重置三个月试用期 3：重置一年试用期
bAddUsageTime = False
strMsgCheck1 = "WJ_ISFP_Release"
strMsgCheck2 = "Mac:"
strMsgCheck5 = "StartTime:"
strMsgCheck6 = "ExpireTime:"
nTimeLen = 14
strCheckAuthorizationPath = "/usr/lib/libgcgCloseVM7dd.so"
strRecordPath = "/usr/lib/libgcgCloseVM7dd_rec.so"
MIN_SPACE_LIMIT = 100  # 磁盘最小空间（MB）, 小于该值，则不允许保存


def get_all_mac_address():
    """获取网卡的mac地址"""
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


def get_no_connect_mac_address():
    """获取没有连接网线的网卡的mac地址"""
    listMac = []
    dic = psutil.net_if_addrs()
    for adapter in dic:
        if 'docker' in adapter:  # docker的mac地址一直变，所以要过滤掉
            continue

        bNoConnect = True
        snicList = dic[adapter]
        mac = 'None'
        for snic in snicList:
            if snic.family.name in {'AF_INET', 'AF_INET6'}:
                bNoConnect = False
                break

            if snic.family.name in {'AF_LINK', 'AF_PACKET'}:
                mac = snic.address
                mac = mac.replace(':', '')  # 替换掉换行符
                if mac == "000000000000":
                    bNoConnect = False
                    continue

        if bNoConnect:
            listMac.append(mac)
    return listMac


def getHdSerialNo():
    """获取硬盘序列号"""
    strDev = ""
    strDevSpare = ""
    p = Popen('df -hl', shell=True, stdout=PIPE, universal_newlines=True)
    out, err = p.communicate()
    for line in out.splitlines():
        listStr = line.split()
        if "/home" == listStr[len(listStr) - 1] and "/dev" in listStr[0]:
            strDev = listStr[0]
        if strDevSpare == "" and "/dev" in listStr[0]:
            strDevSpare = listStr[0]

    if strDev == "":
        strDev = strDevSpare

    strSerialNo = ""
    pHdparm = Popen("echo wanji|sudo -S hdparm -i {}".format(strDev), shell=True, stdout=PIPE,
                               universal_newlines=True)
    out1, err1 = pHdparm.communicate()
    for line in out1.splitlines():
        if "SerialNo" in line:
            strSerialNo = line[line.index("SerialNo=") + len("SerialNo="):]

    if strSerialNo == "" and not getattr(sys, 'frozen', False):
        return "S3YLNG0M704378T"

    return strSerialNo


def encodeComputerID(strMac=""):
    """编码计算机ID"""
    if strMac == "":
        listMac = get_all_mac_address()
        strMac = listMac[0]
    num = getHdSerialNo()
    strId = strMac + num
    bytesId = bytes(strId, encoding="utf8")
    return b64encode(bytesId)


def decodeComputerID(encodeId):
    """解码计算机ID"""
    bytesDecodeId = b64decode(encodeId)
    strDecodeId = str(bytesDecodeId, encoding="utf8")
    strMac = strDecodeId[:12]
    strNum = strDecodeId[12:]
    return strMac, strNum


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
        global_log.error("my_encode_check: listMac empty!!")
        return False, False, -1

    strSerialNo = getHdSerialNo()
    if strSerialNo == "":
        return False, False, -1

    bFirst = False
    nDaysLeft = -1
    strMsgCheck3 = listMac[0]
    strMsgCheck4 = strSerialNo
    # 有效期
    strStartTime = time.strftime("%Y%m%d", time.localtime())
    strExpireTime = strStartTime
    data = strMsgCheck1 + str(0) + str(
        usAuthorized) + strMsgCheck2 + strMsgCheck3 + strMsgCheck4 + strMsgCheck5 + strStartTime + strMsgCheck6 + strExpireTime
    if strText != "":
        # 解密的话要用key和iv生成新的AES对象
        mydecrypt = AES.new(key, AES.MODE_CFB, strText[:16])
        # 使用新生成的AES对象，将加密的密文解密
        decrypttext = mydecrypt.decrypt(strText[16:])
        strText = decrypttext.decode()

        str1 = strText[0:len(strMsgCheck1)]
        if str1 != strMsgCheck1:
            global_log.error("my_encode_check: str1 Error!! {}".format(str1))
            return False, False, -1

        pos = strText.find(strMsgCheck2)
        if pos < 0:
            global_log.error("my_encode_check: pos Error!!")
            return False, False, -1

        str3 = strText[pos + len(strMsgCheck2):pos + len(strMsgCheck2) + len(strMsgCheck3)]
        if str3 not in listMac:
            global_log.error("my_encode_check: str3 Error!! {}".format(str3))
            return False, False, -1
        strMsgCheck3 = str3
        # print("my_encode_check-get mac:", strMsgCheck3)

        # usAuthorizedTemp: 0：未激活  1：已永久激活  2：三个月试用期 3：一年试用期
        usAuthorizedTemp = int(strText[pos - 1:pos])
        if usAuthorizedTemp == 1:
            global_log.info("my_encode_check: already Activated!")
            return True, False, "Activated"

        # usageTimeLimitTemp = usageTimeLimit  # 三个月试用期
        # if usAuthorizedTemp == 3:
        #     usageTimeLimitTemp = usageTimeLimit2  # 一年试用期

        startPos = strText.find(strMsgCheck5)
        if startPos < 0:
            global_log.error("my_authorization: pos Error!")
            return False, False, -1

        str5 = strText[startPos:startPos + len(strMsgCheck5)]
        # print("############str5: ", str5)
        if str5 != strMsgCheck5:
            global_log.error("my_encode_check: str5 Error:{}!!".format(str5))
            return False, False, -1

        endPos = startPos + len(strMsgCheck5) + 8
        str6 = strText[endPos:endPos + len(strMsgCheck6)]
        # print("############str5: ", str6)
        if str6 != strMsgCheck6:
            global_log.error("my_encode_check: str6 Error:{}!!".format(str6))
            return False, False, -1

        strStartTime = strText[startPos + len(strMsgCheck5):startPos + len(strMsgCheck5) + 8]
        # print("############strStartTime: ", strStartTime)
        if strStartTime == "":
            global_log.error("my_encode_check: startTime Error:{}!! ".format(strStartTime))
            return False, False, -1
        try:
            nStartTime = int(strStartTime)
        except:
            global_log.error("my_encode_check: startTime Error:{}!! ".format(strStartTime))
            return False, False, -1

        strExpireTime = strText[endPos + len(strMsgCheck6):endPos + len(strMsgCheck6) + 8]
        # print("############strExpireTime: ", strExpireTime)
        if strExpireTime == "":
            global_log.error("my_encode_check: expireTime Error:{}!! ".format(strExpireTime))
            return False, False, -1
        try:
            nExpireTime = int(strExpireTime)
        except:
            global_log.error("my_encode_check: expireTime Error:{}!! ".format(strExpireTime))
            return False, False, -1

        strNowDate = time.strftime("%Y%m%d", time.localtime())
        nNowDate = int(strNowDate)

        if nNowDate < nStartTime and nNowDate > nExpireTime:
            global_log.error("my_encode_check: The trial period has ended! ")
            return False, False, -1

        str4 = strText[endPos + len(strMsgCheck6) + 8:len(strText)]
        # print("##########str4: ", str4)
        if str4 != strMsgCheck4:
            global_log.error("my_encode_check: str4 Error!! {} ".format(str4))
            return False, False, -1

        nMinutesLeft = int(strText[len(strMsgCheck1):pos - 1])
        nDaysLeft = round((nMinutesLeft / 1440), 1)  # （总时间-已使用的时间）/ 一天的时间（单位都是分钟）= 剩余总天数
        if nDaysLeft <= 7:
            global_log.info("my_encode_check: Expiring! DaysLeft={}".format(nDaysLeft))

        if nMinutesLeft - 1 <= 0:
            global_log.error("my_encode_check: The trial period has ended!")
            return False, False, -1

        strUsageTime = str(nMinutesLeft)
        if bAddUsageTime:
            strUsageTime = str(nMinutesLeft - 1)

        data = strMsgCheck1 + strUsageTime + str(
            usAuthorizedTemp) + strMsgCheck2 + strMsgCheck3 + strMsgCheck5 + strStartTime + strMsgCheck6 + strExpireTime + strMsgCheck4

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

    try:
        strPathTemp = strPath + ".tmp"
        f = open(strPathTemp, 'wb')
        f.write(ciphertext)
        f.close()
        my_renameFile(strPathTemp, strPath, True)
    except:
        global_log.error("my_encode_check err! Failed to write authorization file!")

    global_log.info("nDaysLeft = {}".format(nDaysLeft))
    return True, bFirst, nDaysLeft


def my_authorization(strPath, strRecordPath, strActivationCode):
    strMsgCheck5 = "StartTime:"
    strMsgCheck6 = "ExpireTime:"
    # return value: 0:successful, 1:ActivationCode error, 2:The activation code has expired, 3:can't be activated, 4:time out
    if strActivationCode == "":
        global_log.error("my_authorization: ActivationCode Error! ")
        return 1
    try:
        # 查找所有使用过的激活码，避免重复使用
        listRecord = []
        if judge_exist(strRecordPath):
            f = open(strRecordPath, 'r')
            listRecord = f.readlines()
            f.close()
            for i in range(len(listRecord)):
                listRecord[i] = listRecord[i].strip()

        if strActivationCode in listRecord:
            global_log.error("my_authorization: The activation code has expired!")
            return 2

        strActivationCodeTemp = a2b_hex(strActivationCode)
        # 解密的话要用key和iv生成新的AES对象
        my_decrypt = AES.new(key, AES.MODE_CFB, strActivationCodeTemp[:16])
        # 使用新生成的AES对象，将加密的密文解密
        decrypttext = my_decrypt.decrypt(strActivationCodeTemp[16:])
        strText = decrypttext.decode()

        str1 = strText[0:len(strMsgCheck1)]
        if str1 != strMsgCheck1:
            global_log.error("my_authorization: str1 Error!! ".format(str1))
            return 1

        pos = strText.find(strMsgCheck2)
        if pos < 0:
            global_log.error("my_authorization: pos Error! ")
            return 1

        listMac = get_all_mac_address()
        if len(listMac) == 0:
            global_log.error("my_authorization: listMac empty! ")
            return 3

        strMsgCheck3 = listMac[0]
        str3 = strText[pos + len(strMsgCheck2):pos + len(strMsgCheck2 + strMsgCheck3)]
        # if str3 != strMsgCheck3:
        if str3 not in listMac:
            global_log.error("my_authorization: str3 Error:{}! ".format(str3))
            return 1
        strMsgCheck3 = str3
        # print("my_authorization-bind mac:", strMsgCheck3)

        strActivationCodeTime = strText[pos + len(strMsgCheck2 + strMsgCheck3):pos + len(
            strMsgCheck2 + strMsgCheck3) + nTimeLen]

        # usAuthorizedTemp: 0：未激活  1：永久激活  2：重置三个月试用期 3：重置一年试用期
        # 0：时长激活 1：永久激活
        usAuthorizedTemp = int(strText[pos - 1:pos])
        if usAuthorizedTemp not in [0, 1]:
            global_log.error("my_authorization: usAuthorizedTemp Error! ")
            return 1

        # strMsgCheck5开始位置
        startPos = strText.find(strMsgCheck5)
        if startPos < 0:
            global_log.error("my_authorization: pos Error! ")
            return 1

        str5 = strText[startPos:startPos + len(strMsgCheck5)]

        if str5 != strMsgCheck5:
            global_log.error("my_encode_check: str5 Error:{}!! ".format(str5))
            return 1

        strMsgCheck5 = str5

        strStartTime = strText[startPos + len(strMsgCheck5):startPos + len(strMsgCheck5) + 8]
        if strStartTime == "":
            global_log.error("my_encode_check: startTime Error:{}!! ".format(strStartTime))
            return 3

        endPos = startPos + len(strMsgCheck5) + 8
        str6 = strText[endPos:endPos + len(strMsgCheck6)]
        if str6 != strMsgCheck6:
            global_log.error("my_encode_check: str6 Error:{}!! ".format(str6))
            return 1
        strMsgCheck6 = str6

        strExpireTime = strText[endPos + len(strMsgCheck6):endPos + len(strMsgCheck6) + 8]
        if strExpireTime == "":
            global_log.error("my_encode_check: expireTime Error:{}!! ".format(strExpireTime))
            return 3

        strSerialNo = getHdSerialNo()
        if strSerialNo == "":
            global_log.error("my_authorization: number Error:{}! ".format(strSerialNo))
            return 3

        strMsgCheck4 = strSerialNo
        str4 = strText[endPos + len(strMsgCheck6) + 8:len(strText)]
        if str4 != strMsgCheck4:
            global_log.error("my_encode_check: str4 Error:{}!! ".format(str4))
            return 1

        nMinutesLeft = int(strText[len(strMsgCheck1):pos - 1])

        data = strMsgCheck1 + str(nMinutesLeft) + str(
            usAuthorizedTemp) + strMsgCheck2 + strMsgCheck3 + strMsgCheck4  # 加密的明文长度必须为16的倍数，如果长度不为16的倍数，则需要补足为16的倍数

        data = strMsgCheck1 + str(nMinutesLeft) + str(
            usAuthorizedTemp) + strMsgCheck2 + strMsgCheck3 + strMsgCheck5 + strStartTime + strMsgCheck6 + strExpireTime + strMsgCheck4
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
        global_log.error("my_authorization: except!!! ")
        print(traceback.format_exc())
        return 1

    return 0


def getHdRemainingSpace(strPath):
    if strPath == "":
        global_log.error("getHdRemainingSpace Error!! strPath is empty!")
        return -1

    try:
        # 获取目录的磁盘信息
        info = os.statvfs(strPath)
        fSpace = round(info.f_bsize * info.f_bavail / 1024 / 1024, 2)
    except:
        global_log.error("getHdRemainingSpace Error!! fSpace Error!!")
        return -1

    return fSpace


class Authorization(object):
    """授权类"""
    def __init__(self):
        self.b_activated = False
        self.b_noSpace = False
        self.s_localId = ''
        self.n_active_state = 0
        self.s_timeLeft = ''
        self.s_mac = "None"
        self.timer_check = None

    def checkAuthorization(self):
        """检查本地激活状态"""
        fSpace = getHdRemainingSpace("/")
        if fSpace < MIN_SPACE_LIMIT:
            self.b_noSpace = True

        bOk, bFirst, nDaysLeft = my_encode_check(strCheckAuthorizationPath)
        self.s_timeLeft = nDaysLeft
        if bOk and not bFirst:
            self.b_activated = True
        else:
            self.b_activated = False
            self.getCorrectMac()

    def getCorrectMac(self):
        """获取本地ID"""
        if self.s_mac != "None":
            self.s_localId = str(encodeComputerID(self.s_mac), encoding="utf8")
            global_log.info(" Get the correct identification code!")
            return

        # 网卡1的MAC地址
        list_mac = get_all_mac_address()
        if len(list_mac) > 0:
            self.s_mac = list_mac[0]
            self.s_localId = str(encodeComputerID(self.s_mac), encoding="utf8")
            global_log.info(" Get the correct identification code!")

    def setActive(self, s_ActiveCode):
        """设置激活码"""
        self.n_active_state = my_authorization(strCheckAuthorizationPath, strRecordPath, s_ActiveCode)
        if self.n_active_state == 0:
            self.b_activated = True

    def getActiveInfo(self):
        """获取激活信息"""
        dictActiveInfo = {'act_state': int(self.b_activated),
                          'space_state': int(self.b_noSpace),
                          'localId': self.s_localId,
                          'active_res': self.n_active_state,
                          'time_eft': self.s_timeLeft}
        return dictActiveInfo
