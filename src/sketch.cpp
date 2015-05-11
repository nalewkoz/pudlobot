//#include <Servo.h>
//#include <NewPing.h>

#define k_theta 40.0
#define k_v_ang 0.69
#define k_pos_lin 400.0
#define k_v_lin 0
#define k_acc 0.1

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <LPS.h>
#include <avr/sleep.h>
L3G gyro;
LSM303 compass;
LPS ps;

//float k_pos_lin=400.0;
//float k_theta=40.0;

float temperature,pressure,altitude;
char report[80];

#define velocity_min 0	// Za mala predkosc powoduje, ze silniki nie rusza!

//#define sonar200_N 5 // ile pomiarow do usredniania

// Stale (moze zmienic na define?)
//int sonar200_trigger_pin=11;
//int sonar200_echo_pin=12;
int blue_led=9;
int red_led=11;
int green_led=10;

int led_pin = 13;
int vol_pin = 0;
//int light_pin = 3;
//int current_pin = 2;
int pwma_pin=3;
int pwmb_pin=6;
int ain1_pin=4;
int ain2_pin=5;
int bin1_pin=7;
int bin2_pin=8;
int stby_pin=9;
//int cam_servo_min=550;
//int cam_servo_max=2400;
//int cam_servo_pin=10;
//int distance2stop=40;

//unsigned long T_sonar200=33;	//czas probkowania sonaru
unsigned long T_end=10000;
unsigned long T_led=1000;
unsigned long T_vol=100;		//czas probkowania napiecia
unsigned long T_vol_in=2000; 	//czas ch. filtru pomiaru napiecia
unsigned long T_serial_write=1000;	// co jaki czas wysylamy dane do kompa
unsigned long T_gyro=3;
unsigned long T_acc=3;
unsigned long T_control=5;

float T_filter_drift=15;
float T_filter_theta=10;
//float k_theta=25.0;
//float k_v_ang=0.6;
//float k_pos_lin=10.0;
//float k_v_lin=10.0;

// Zmienne
unsigned long time_led;
//unsigned long time_altimu10;
//unsigned long time_sonar200;
//unsigned long time_sonar200_2;
unsigned long time_vol;
unsigned long time_serial_write;
unsigned long time_gyro, time_acc, time_control;
unsigned long t_gyro,t_gyro_new,t_acc,t_acc_new,t_control,t_control_new;

//int sonar200_i=0;
int moc;
int kat;
int odl,msec;
//int sonar200_tab[sonar200_N];
int state_move=1;		// na poczatku stoimy
int velocity=100;			// zadana predkosc

int auto_balance=0;

float vol, vol_act,a_vol;
int light_int;
float drift_av[3];
float acc_av[3];
float theta[3];
float v_ang[3];
float acc[3];
float acc_total;
float a_acc=1.0;
float pos_lin;
float v_lin;

float dt_gyro;
float dt_acc;
float dt_control;
float gyro_gain=0.00875;
float acc_gain=0.0610352;
//float current;

int inByte;
bool led_state=false;
int i;
//NewPing sonar200(sonar200_trigger_pin,sonar200_echo_pin,200);
//Servo cam_servo;

class MyCommand {
    /* możliwe stany:
	0 = czekam na komende
	1 = znam komende i to jest write, czekam na spacje
	2 = wczytuje pierwsza liczbe (pos mowi gdzie umiescic kolejny znak, gdy spacja koniec)
	3 = wczytuje druga liczbe (pos mowi gdzie umiescic kolejny znak, gdy \n koniec i wykonujemy komende)
	4 = znam komende i to jest read, czekam na spacje
	5 = wczytuje liczbe (gdy \n koniec i wykonujemy komende)
	6 = znam komende i jest to command. Wczytuje liczbe (czyli musi byc bez spacji!)
	7 = znam komende i jest sleep. 
    */
private:
    char a1[10];
    char a2[10];
    int state; 
    int pos;
    int speeda;
    int speedb;
    int breaking;

public:


    MyCommand()
    {
	state=0;;
    }
    // STEROWANIE SILNIKAMI. WYROZNIAMY STAN AKTYWNEGO ZATRZYMANIA (BREAKING)!
    void set_pwm(int sa=0,int sb=0)	//można tym ustawic szybkosc lub tylko wlaczyc pwm z ustalona wczesniej szybkoscia
    {
	if(sa || sb)
	{
	    speeda=sa;
	    speedb=sb;
	}

	analogWrite(pwma_pin,speeda);
	analogWrite(pwmb_pin,speedb);
    }
    void quit_breaking()
    {
	if(breaking)
	{
	    set_pwm();
	    breaking=0;
	}
    }
    void turn_right()
    {
	quit_breaking();
	digitalWrite(ain1_pin,HIGH);
	digitalWrite(ain2_pin,LOW);
	digitalWrite(bin1_pin,HIGH);
	digitalWrite(bin2_pin,LOW);
    }   
    void turn_left()
    {
	quit_breaking(); 
	digitalWrite(ain2_pin,HIGH);
	digitalWrite(ain1_pin,LOW);
	digitalWrite(bin2_pin,HIGH);
	digitalWrite(bin1_pin,LOW);
    }
    void move_forward()
    {
	quit_breaking();
	digitalWrite(ain2_pin,HIGH);
	digitalWrite(ain1_pin,LOW);
	digitalWrite(bin1_pin,HIGH);
	digitalWrite(bin2_pin,LOW);
    }
    void move_backward()
    {
	quit_breaking();
	digitalWrite(ain1_pin,HIGH);
	digitalWrite(ain2_pin,LOW);
	digitalWrite(bin2_pin,HIGH);
	digitalWrite(bin1_pin,LOW);
    }
    void stop_cast()
    {
	quit_breaking();
	digitalWrite(ain1_pin,LOW);
	digitalWrite(bin1_pin,LOW);
	digitalWrite(ain2_pin,LOW);
	digitalWrite(bin2_pin,LOW);
    }
    void stop_break()
    {
	analogWrite(pwma_pin,0);
	analogWrite(pwmb_pin,0);
	breaking=1;
    }
    void sleep_forever()
    {
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_mode();
    }
    int fulfill_write()
    {
	int pin=atoi(a1);
	int value=atoi(a2);
/*	if(pin==cam_servo_pin && value<=cam_servo_max && value>=cam_servo_min)
	{
	    cam_servo.writeMicroseconds(value); // Zakloca prace pomiarow [napiecia i odleglosci]. Dlaczego? Zasilanie? Srawdzic ze stabilizatorem zamiast zasilania z raspberry
	    //DO DEBUGOWANIA:
	    //led_state=!led_state;
	    //digitalWrite(led_pin,led_state);
	}*/
	if(pin==-1 && value<256 & value>=velocity_min)
	{
	    velocity=value;
	    if(!breaking)	 set_pwm(value,value);
	    else {speeda=value; speedb=value;}
	}
	else
	{
	    analogWrite(pin,value);
	}
	return pin;
    }    
    int fulfill_read()
    {
	return 0; //na razie nic nie robi
    }
    int fulfill_control()
    {

	int val=atoi(a1);
	// set_pwm(velocity,velocity); //Potrzebne tylko, gdy gladkie hamowanie...
	if(auto_balance)
        {
		if(val==7) 
		{ 
			stop_balance();
			return 0;
	        } 
		return -1;
	}
	switch(val)
	{
	    case 0:
		stop_break();
		break;
	    case 1:
		stop_cast();
		break;
	    case 2:
		move_forward();
		break;
	    case 3:
		move_backward();
		break;
	    case 4:
		turn_left();
		break;
	    case 5:
		turn_right();
		break;
            case 6:
		init_balance();
		break;
//	    case 7:
//		stop_balance();
//		break;
	    default:
		return 0;
	}
	state_move=val;
	return state_move;
    }
    int newByte(char cc)
    {
	switch(state)
	{
	    case 0:
		switch(cc)
		{
		    case 's':
//			Serial.println("Odczytalem s");
			state=1;
			break;
		    case 'r':
			state=4;
			break;
		    case 'c':
//			Serial.println("Odczytalem c");
			pos=0;
			state=6;
			break;
		    case '*': // sleep_mode!
			state=7;
			break;
		    
		    default:
//			Serial.println("Odczytalem nie wiem co..");
			state=0;
		}
		break;
	    case 1:
//		Serial.println("case 1");
		if(cc==' ')
		{
		    pos=0;
		    state=2;
		}
		else
		{
		    state=0;
		}
		break;
	    case 2:
		//mozna jeszcze dodac sprawdzenie, czy cyfra?
//		Serial.print("case 2, cc: ");
//		Serial.print(cc);
//		Serial.println(" ");
		a1[pos]=cc;
		pos++;
		if(cc==' ')
		{
		    a1[pos-1]=0;
		    pos=0;
		    state=3;
		}
		break;
	    case 3:
//		Serial.print("case 3, cc: ");
//		Serial.print(cc);
//		Serial.println(" ");
		a2[pos]=cc;
		pos++;
		if(cc=='\n')
		{
		    a2[pos-1]=0;
		    state=0;
		    fulfill_write();
		}
		break;
	    case 4:
		if(cc==' ')
		{
		    pos=0;
		    state=5;
		}
		else
		{
		    state=0;
		}
		break;
	    case 5:
//		Serial.println("case 5");
		a1[pos]=cc;
		pos++;
		if(cc=='\n')
		{
		    a1[pos-1]=0;
		    state=0;
		    fulfill_read();
		}
		break;
	    case 6:
		a1[pos]=cc;
		pos++;
//		Serial.print("Case 6. CC: ");
///		Serial.print(cc);
//		Serial.println(" ");
		if(cc=='\n')
		{
		    a1[pos-1]=0;
		    state=0;
//		    Serial.println("Bedzie control...");
		    fulfill_control();
		}
		break;
            case 7:
		if(cc='\n')
		{
			state=0;
			sleep_forever();
		}
		else
		{
			state=0;
		}
		break;		
	    default:
		state=0;
	}
    return state;
    }
    void reset()
    {
	state=0;
    }
};

MyCommand commandPi;

void my_delay(unsigned long delay)
{
    unsigned long t=millis();
    while(millis()<t+delay);

}
void drift_average(int N)
{
	int i,k,rc=255;
	for(k=0;k<3;k++) drift_av[k]=0;
	for(i=0;i<N;i++)
	{
		if(!(i%10)){
			analogWrite(blue_led,0);
  			analogWrite(green_led, rc/6);
  			analogWrite(red_led, rc);
			rc=255-rc;
		}
		
		gyro.read();
		drift_av[0]+=gyro.g.x*gyro_gain;
		drift_av[1]+=gyro.g.y*gyro_gain;
		drift_av[2]+=gyro.g.z*gyro_gain;
		delay(10*T_gyro);
	}
	for(k=0;k<3;k++) drift_av[k]/=(float)N;
	pos_lin=0.0;
	v_lin=0.0;
}
// the setup routine runs once when you press reset:
void setup() {                
    analogWrite(blue_led,255);
    analogWrite(green_led,255);
    analogWrite(red_led, 255);

    pinMode(led_pin, OUTPUT);
//    pinMode(stby_pin,OUTPUT);
    pinMode(ain1_pin,OUTPUT);
    pinMode(ain2_pin,OUTPUT);
    pinMode(bin1_pin,OUTPUT);
    pinMode(bin2_pin,OUTPUT);  
     
    commandPi.set_pwm(velocity,velocity);    
    commandPi.stop_cast();

    Serial.begin(115200);
  //  cam_servo.attach(cam_servo_pin);
  //  cam_servo.writeMicroseconds( (cam_servo_max+cam_servo_min)/2 );

    time_led=0;
    //time_sonar200=0;
    time_vol=0;
    a_vol=((float)T_vol)/((float)T_vol_in);

  //  digitalWrite(stby_pin,HIGH);
    vol=15.0*((float)analogRead(vol_pin))/1023.0;
  Serial.print("Voltage: ");
  Serial.println(vol);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();
  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  ps.enableDefault();
  delay(2000);
  analogWrite(green_led,255);
  analogWrite(red_led,0);
  analogWrite(blue_led,0);
}
void init_times_sensors()
{
  time_gyro=millis();
  time_acc=millis();
  time_control=millis();
  t_gyro=micros();
  t_acc=micros();
  t_control=micros();
}

void stop_balance()
{
  velocity=100;
  commandPi.set_pwm(velocity,velocity);    
  commandPi.stop_cast();
  auto_balance=0;
  analogWrite(blue_led,0);
  analogWrite(green_led,255);
  analogWrite(red_led,0);
}
void acc_average(int N)
{
  int i,rc=0;
  for(i=0;i<3;i++) acc_av[i]=0;

  for(i=1;i<N;i++)
  {
    if(!(i%10)){
                analogWrite(blue_led,0);
                analogWrite(green_led, rc/6);
                analogWrite(red_led, rc);
                rc=255-rc;
    }
    compass.read();
    acc_av[0]+=((float)compass.a.x)*acc_gain;
    acc_av[1]+=((float)compass.a.y)*acc_gain;
    acc_av[2]+=((float)compass.a.z)*acc_gain;
    delay(10*T_acc);
  }
  for(i=0;i<3;i++) acc_av[i]/=(float)N;
}
void init_balance()
{
  drift_average(100);
  analogWrite(blue_led,200);
  analogWrite(green_led,255);
  analogWrite(red_led, 0);
  delay(5000);

  analogWrite(blue_led,0);
  analogWrite(green_led,0);
  analogWrite(red_led,255);
 
  acc_average(100);
  for(int i=0;i<3;i++)
  {
        theta[i]=0;
        v_ang[i]=0;
  }
  init_times_sensors();
  auto_balance=1;
}

void angular_int()
{
	int i;
	for(i=0;i<3;i++)
	{
		theta[i]+=dt_gyro*v_ang[i];
		theta[i]=theta[i]*(1-dt_gyro/T_filter_theta);
		
		drift_av[i]=drift_av[i]*(1-dt_gyro/T_filter_drift); 
	}
	v_ang[0]=gyro.g.x*gyro_gain-drift_av[0];
	v_ang[1]=gyro.g.y*gyro_gain-drift_av[1];
	v_ang[2]=gyro.g.z*gyro_gain-drift_av[2];	

}
void loop() {

    if(auto_balance && millis()-time_control>T_control)
    {
	t_control_new=micros();
	dt_control=((float)(t_control_new-t_control))/1000000.0;
	t_control=t_control_new;

	pos_lin=pos_lin+v_lin*dt_control;

	
	moc=(int)((7.4/vol)*( k_acc*(acc[0]-acc_av[0])+ k_theta*theta[1] + k_v_ang*v_ang[1] + k_pos_lin*pos_lin + k_v_lin*v_lin));
	if(moc>255) moc=255;
	else if(moc<-255) moc=-255;
	
	v_lin=float(moc)/255.0;

//	if(millis()<T_end)
//	{
		commandPi.set_pwm(abs(moc),abs(moc));
		if(moc<0) commandPi.move_forward();
		else commandPi.move_backward();
//	}
//	else
//	{
//		commandPi.stop_cast();
//		analogWrite(blue_led,255);
//		analogWrite(green_led,0);
//		analogWrite(red_led, 0);
//	}
    }

    if(millis()-time_acc>T_acc)
    {
	time_acc=millis();
	t_acc_new=micros();
	compass.read();
	dt_acc=((float)(t_acc_new-t_acc))/1000000.0;
	t_acc=t_acc_new;
	acc[0]=((float)compass.a.x)*acc_gain;
	acc[1]=((float)compass.a.y)*acc_gain;
	acc[2]=((float)compass.a.z)*acc_gain;
	acc_total=(1-a_acc)*acc_total+a_acc*sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
	/*if(millis()<T_end){
		for(i=0;i<3;i++)
        	{
                	Serial.print(acc[i]);
                	Serial.print("  ");
        	}
		Serial.println(t_acc);
	}*/

// DO TESTOWANIA KORELACJI
/*	snprintf(report, sizeof(report), "A: %6d %6d %6d    M: %6d %6d %6d",
	compass.a.x, compass.a.y, compass.a.z,
	compass.m.x, compass.m.y, compass.m.z);
	Serial.println(report);
*/
//	Serial.print("dt_acc = ");
//	Serial.println(dt_acc,4);
    }
    if(millis()-time_gyro>T_gyro)
    {
	time_gyro=millis();
	t_gyro_new=micros();
	gyro.read();
	dt_gyro=((float)(t_gyro_new-t_gyro))/1000000.0;
	t_gyro=t_gyro_new;

// DO TESTOWANIA KORELACJI MIEDZY KOLEJNYMI PROBKAMI:
/*	Serial.print("Gyro data: ");
	Serial.print(gyro.g.x);
	Serial.print("  ");
	Serial.print(gyro.g.y);
        Serial.print("  ");
	Serial.println(gyro.g.z);
*/

//	Serial.print("dt_gyro = ");
//	Serial.println(dt_gyro,4);

	angular_int();
// =============  CONTROL  ==================================================
/*	t_control_new=micros();
        dt_control=((float)(t_control_new-t_control))/1000000.0;
        t_control=t_control_new;

        pos_lin=pos_lin+v_lin*dt_control;


        moc=(int)((7.4/vol)*( k_theta*theta[1] + k_v_ang*v_ang[1] + k_pos_lin*pos_lin + k_v_lin*v_lin));

        if(moc>255) moc=255;
        else if(moc<-255) moc=-255;

        v_lin=float(moc)/255.0;

        if(millis()<T_end)
        {
                commandPi.set_pwm(abs(moc),abs(moc));
                if(moc<0) commandPi.move_forward();
                else commandPi.move_backward();
        }
	 else
        {
                commandPi.stop_cast();
                analogWrite(blue_led,255);
                analogWrite(green_led,0);
                analogWrite(red_led, 0);
        }
*/

// ===============  SENS INFO  ================================
	/*
	if(millis()<T_end)
	{
		for(i=0;i<3;i++)
        	{
        	        Serial.print(v_ang[i]);
        	        Serial.print("  ");
			Serial.print(theta[i]);
			Serial.print("  ");
	        }
		Serial.print(pos_lin);
		Serial.print("  ");
		Serial.print(v_lin);
		Serial.print("  ");
	        Serial.println(t_gyro);
	}
	*/
//	compass.read();
//	pressure = ps.readPressureMillibars();
	//altitude = ps.pressureToAltitudeMeters(pressure);
//	temperature = ps.readTemperatureC();
    }
    if(millis()-time_vol>T_vol)
    {
	time_vol=millis();
	vol_act=15.0*((float)analogRead(vol_pin))/1023.0;
	vol=vol*(1-a_vol)+a_vol*vol_act;
//	light_int=analogRead(light_pin);
//	current= 36.7*((float)analogRead(current_pin))/1023.0-18.3;
    }

// ==== MIGAJACY MALY LED =====
/*    if(millis()-time_led>T_led)
    {
	time_led=millis();
	led_state=!led_state;
	digitalWrite(led_pin,led_state);
    } 
*/
    if(millis()-time_serial_write>T_serial_write)
    {
	time_serial_write=millis();
	
//	if(millis()>T_end){
		Serial.print("Napiecie: ");
		Serial.print(vol);
		Serial.println(" V");
//	}
	for(i=0;i<3;i++)
        {
                Serial.print(v_ang[i]);
                Serial.print("  ");
                Serial.print(theta[i]);
                Serial.print("  ");
        }
        Serial.print(pos_lin);
        Serial.print("  ");
        Serial.print(v_lin);
        Serial.print("  ");
        Serial.println(t_gyro);
        for(i=0;i<3;i++)
        {
                Serial.print(acc[i]);
                Serial.print("  ");
        }
	Serial.print(acc_total,1);
	Serial.print(" ");
        Serial.println(t_acc);

/*
	Serial.print("Angular vel. ");
	Serial.print("X: ");
	Serial.print(v_ang[0]);
	Serial.print(" Y: ");
	Serial.print(v_ang[1]);
	Serial.print(" Z: ");
	Serial.println(v_ang[2]);
	Serial.print("Theta ");
        Serial.print("X: ");
        Serial.print(theta[0]);
        Serial.print(" Y: ");
        Serial.print(theta[1]);
        Serial.print(" Z: ");
        Serial.println(theta[2]);
	Serial.println(dt_gyro,4);
	Serial.println(dt_acc,4);
*/
/*
	snprintf(report, sizeof(report), "A: %.d %6d %6d    M: %6d %6d %6d",
	compass.a.x, compass.a.y, compass.a.z,
	compass.m.x, compass.m.y, compass.m.z);
	Serial.println(report);
*/

//	Serial.print("p: ");
//	Serial.print(pressure);
//	Serial.print(" mbar\ta: ");
//	Serial.print(temperature);
//	Serial.println(" deg C");

//	Serial.print("Swiatlo: ");
//	Serial.println(light_int);
//	Serial.print("Prad: ");
//	Serial.println(current);
    }
    if(Serial.available())
    {
	inByte=Serial.read();
	commandPi.newByte(inByte);
    }

}
