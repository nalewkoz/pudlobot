#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include "conio.h"

#define MAX_BUF 100
#define cam_servo_pin 10
#define cam_servo_max 2500
#define cam_servo_min 500
#define cam_servo_inc 50
#define serial_baud 115200
#define speed_inc 5
#define blue_pin 9
#define red_pin 11
#define green_pin 10

using namespace std;

char serial_port[]="/dev/ttyAMA0";
char c;
int fd;
int cam_servo=1500;
int speed=160;
int nc;

char file_name[]="log.txt";
char bufor[MAX_BUF];
FILE *filed;

void send_command(int pin,int x)
{
    serialPrintf(fd,"s %d %d\n",pin,x);
}
void init()
{
    nc=0;
    fd=serialOpen(serial_port,serial_baud);
    filed=fopen(file_name,"a");
    time_t rawtime;
    struct tm * timeinfo;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    fprintf(filed,"\n====== NEW SESSION ======\n%s=========================\n",asctime(timeinfo));
 //   send_command(cam:_servo_pin,cam_servo);
}
int main()
{
    init();
    
    cout<<"Sterujemy robotem (awsdzx, =-, uh, lo)\nBy wyjsc: q\n";
    do
    {
	if(inputAvailable())
	{
	c=getch();
	switch(c)
	{
	    case 'z':
		serialPrintf(fd,"c0\n");
		break;
	    case 's':
		serialPrintf(fd,"c1\n");
		break;
	    case 'w':
		serialPrintf(fd,"c2\n");
		break;
	    case 'x':
		serialPrintf(fd,"c3\n");
		break;
	    case 'a':
		serialPrintf(fd,"c4\n");
		break;
	    case 'd':
		serialPrintf(fd,"c5\n");
		break;
	    case 'b':
		serialPrintf(fd,"c6\n");
		break;
	    case 'v':
		serialPrintf(fd,"c7\n");
		break;
	    case '=':
		speed+=speed_inc;
		send_command(-1,speed);
		cout<<"speed="<<speed<<"\n";
		break;
	    case '-':
		speed-=speed_inc;
		send_command(-1,speed);
		cout<<"speed="<<speed<<"\n";
		break;
	    case 'u':
		cam_servo+=cam_servo_inc;
		if(cam_servo>cam_servo_max) cam_servo=cam_servo_max;
		send_command(cam_servo_pin,cam_servo);
		cout<<"cam_servo="<<cam_servo<<"\n";
		break;
	    case 'h':
		cam_servo-=cam_servo_inc;
		if(cam_servo<cam_servo_min) cam_servo=cam_servo_min;
		send_command(cam_servo_pin,cam_servo);
		cout<<"cam_servo="<<cam_servo<<"\n";
		break;
	    case 'l':
		send_command(blue_pin,0);
		send_command(red_pin,0);
		send_command(green_pin,0);
		break;
	    case 'o':
		send_command(green_pin,255);
		break;
	    case '*':
		serialPrintf(fd,"*\n");
		break;
	    case 'q':
		break;
	    default:
		cout<<"Brak takiej komendy\n";
		break;
	}
	}
	if(serialDataAvail(fd)>0)
	{
		bufor[nc++]=serialGetchar(fd);
		if(nc==MAX_BUF-1)
		{
			bufor[nc]=0;
			nc=0;
			fprintf(filed,bufor);
		}
	}
//	delay(1); // to save processor time. Won't work with balancing mode - too much data flowing, and with this enabled we can at most take one letter per ms from serial input, which is too slow.
    }
    while(c!='q');
    serialClose(fd);
    fclose(filed);
}
