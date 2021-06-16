from CommonDefine import *
from sys import argv


def getBccCheckVal(recvData):
    checkVal = 0x00
    for i in range(len(recvData)):
        checkVal ^= recvData[i]
    return checkVal


def parsingData(recvData):
    data = struct.unpack(">H4BH4B2H4B", recvData)
    checkVal = getBccCheckVal(recvData[:len(recvData) - 2])
    if checkVal != data[14]:
        return None

    return data


if __name__=="__main__":
    strArg = ""
    for arg in argv:
        strArg += arg + " "
    print("args:", strArg)

    # strIp = argv[]
    bTest = False
    nn = 10
    socketTest = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        nn = 188
        dataTest = struct.pack(">H4BH4B2H2B", 0xFFFF, 0x00, 0xA0, 0x00, 0x00, 0x0A, 192, 168, 3, nn, 3099, 5555, 2, 0)
        checkVal = getBccCheckVal(dataTest)
        dataTest = struct.pack(">H4BH4B2H4B", 0xFFFF, 0x00, 0xA0, 0x00, 0x00, 0x0A, 192, 168, 3, nn, 3099, 5555, 2, 0,
                               checkVal, 0xFF)
        dataUnpack = parsingData(dataTest)

        try:
            socketTest.sendto(dataTest, ("192.168.3.183", 5555))
            print("send:", dataUnpack)
        except:
            print("send err!")

        # break
        time.sleep(20)
        nn += 1
        if nn > 255:
            nn = 10

    # dataTest = struct.pack(">4B", 192, 168, 3, 188)
    # dataUnpack = parsingData(dataTest)
