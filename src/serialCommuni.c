#include "serialCommuni.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#define MAXDATA (50)

static int fd = -1;
unsigned char Data[MAXDATA];

int serial_open(int deviceIndex, float baudrate)
{
	struct termios newtio;
	struct serial_struct serinfo;
	char dev_name[20] = {0, }, gDeviceName[20] = {0, };

	sprintf(dev_name, "/dev/ttyS%d", deviceIndex);

	strcpy(gDeviceName, dev_name);
	memset(&newtio, 0, sizeof(newtio));
	serial_close();

	/*1. open serial device*/
	if((fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
		fprintf(stderr, "serial device open error: %s\n", dev_name);
		goto DXL_HAL_OPEN_ERROR;
	}
	/*2. 设置串口属性*/
	newtio.c_cflag		= B1200|CS8|CLOCAL|CREAD;//波特率38400, 字符长度掩码CS8，忽略modem控制线，打开接收者
	newtio.c_iflag		= IGNPAR;
	newtio.c_oflag		= 0;
	newtio.c_lflag		= 0;
	newtio.c_cc[VTIME]	= 0;	// 非canonical模式读时的延时，以十分之一秒为单位
	newtio.c_cc[VMIN]	= 0;	// 非canonical模式读的最小字符数

	tcflush(fd, TCIFLUSH); //刷新收到的数据但是不读
	tcsetattr(fd, TCSANOW, &newtio); //设置指定串口参数，并立即生效

	if(fd == -1)
		return 0;

	//获取tty线路信息
	if(ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot get serial info\n");
		return 0;
	}
	//设置tty线路信息熟悉
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;
	//设置tty线路信息
	if(ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot set serial info\n");
		return 0;
	}

	serial_close();

	strcpy(gDeviceName, dev_name);
	memset(&newtio, 0, sizeof(newtio));
	serial_close();

	//重新打开tty设备
	if((fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
		fprintf(stderr, "device open error: %s\n", dev_name);
		goto DXL_HAL_OPEN_ERROR;
	}
	//设置串口属性
	newtio.c_cflag		= B1200|CS8|CLOCAL|CREAD;//波特率38400, 字符长度掩码CS8，忽略modem控制线，打开接收者
	newtio.c_iflag		= IGNPAR;
	newtio.c_oflag		= 0;
	newtio.c_lflag		= 0;
	newtio.c_cc[VTIME]	= 0;	// time-out 값 (TIME * 0.1초) 0 : disable
	newtio.c_cc[VMIN]	= 0;	// MIN 은 read 가 return 되기 위한 최소 문자 개수

	tcflush(fd, TCIOFLUSH);//刷行接收到的数据但是不读
	tcsetattr(fd, TCSANOW, &newtio);//设置指定串口参数并立即生效

	return 1;

DXL_HAL_OPEN_ERROR:
	serial_close();
	return 0;
}

//接受数据
int receiveMessage(unsigned char *pPacket, int numPacket)
{
	int readNum = -1;
	memset(pPacket, 0, numPacket); //清空接收数据包
	readNum = read(fd, pPacket, numPacket);
	if(readNum > 0)
		tcflush(fd, TCIFLUSH);//清空通道
	return readNum;

}

//发送数据
int sendMessage(unsigned char *pPacket, int numPacket)
{
	//printf("sending Data ...\r");
	tcflush(fd, TCOFLUSH);//清空管道
	return write(fd, pPacket, numPacket);
}

int serial_close()
{
	if(fd != -1)
		close(fd);
	fd = -1;

	return 1;
}
