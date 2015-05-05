#include <wiringSerial.h>
#include <iostream>
#include "conio.h"

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

void send_command(int pin,int x)
{
    serialPrintf(fd,"s %d %d\n",pin,x);
}
void init()
{
    fd=serialOpen(serial_port,serial_baud);
    send_command(cam_servo_pin,cam_servo);
}
int main()
{
    init();
    
    cout<<"Sterujemy robotem (awsdzx, =-, uh, lo)\nBy wyjsc: q\n";
    do
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
    while(c!='q');
    serialClose(fd);
}
