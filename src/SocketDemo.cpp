#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int SokcetDemo()
{
    //创建server
	int ret=SocketCreate("192.168.1.100", 8888, "camera");
	if (0!=ret)
	{
		printf("SocketCreate failure, err=%d\n",ret);
		return ret;
	}
    sleep(1);
    //接收client数据
    uint8_t rbuf[100];
    int len=SocketRecvByteI(rbuf, 100, "camera");
    //判断数据格式是否正确
    if (!IsProtocolRight(rbuf,len))
    {
        printf("protocol error\n");
    }
    //解析数据
    int flag=0;
    GetProtocolDataInt(rbuf,"start",&flag);
    printf("start=%d\n",flag);
    //打包数据
	uint16_t offset=0;
	uint8_t buf[50];
	offset = SetProtocolDataInt(buf,offset, "have", 1);
	offset = SetProtocolDataDouble(buf,offset, "x", 0.1);
	offset = SetProtocolDataDouble(buf,offset, "y", 0.0);
    offset = SetProtocolDataDouble(buf,offset, "z", -0.03);
	offset = SetProtocolDataDouble(buf,offset, "k", 0.0);
    offset = SetProtocolDataDouble(buf,offset, "p", 0.0);
    offset = SetProtocolDataDouble(buf,offset, "s", 0.0);
    //发送数据到client
    ret=SocketSendByteI(buf, offset, "camera");
    if (0!=ret)
    {
        printf("SocketSendByteI failure\n");
    }
	SocketClose("camera");//关闭server
    return 0;
}