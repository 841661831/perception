注意：激活类型，2表示重置使用期为3个月，3表示1年，1表示永久激活，其他无法激活
注意：输入命令如下：
1、激活本机：
sudo ./GetActivationCode
默认激活3个月

2、按类型激活本机：
sudo ./GetActivationCode 2
命令含义：（1）sudo （2）执行程序 （3）程序参数1：激活类型

3、使用机器原始识别码，按类型激活任意机器：
sudo ./GetActivationCode c400ada7ec1d S3YLNG0M704378T 2
命令含义：（1）sudo （2）执行程序 （3）程序参数1：机器原始识别码1 （4）程序参数2：机器原始识别码2（5）程序参数3：激活类型

4、使用加密后的机器识别码，按类型激活任意机器：
sudo ./GetActivationCode YzQwMGFkYTdlYzFjUzNZTE5HME03MDQzNzhU 1
命令含义：（1）sudo （2）执行程序 （3）程序参数1：机器的唯一识别码 （4）激活类型
