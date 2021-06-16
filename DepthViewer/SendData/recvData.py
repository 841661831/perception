import socket
import struct


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


def udpServerProcess():
    BUFSIZE = 1024
    ip_port = ('', 5555)
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
    server.bind(ip_port)
    while 1:
        data, client_addr = server.recvfrom(BUFSIZE)
        recvData = parsingData(data)
        if recvData is None or recvData[2] != 0xA0:
            continue
        else:
            print("recv:", recvData)

udpServerProcess()