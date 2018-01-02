#include <QTRSensors.h>
#include <avr/interrupt.h>
////////////////////////////////////////////////////////////////////////////////////
//   robot velocista mini FK BOT  V 3.0                                            //
// micro motores pololu MP 10:1, sensores qtr 8rc, driver drv8833, arduino nano    //
//                                     06/3/2015                                  //
/////////////////////////////////////////////////////////////////////////////////////

#define NUM_SENSORS   8  //numero de sensores usados
#define TIMEOUT       3000  // tiempo de espera para dar resultado en uS
#define EMITTER_PIN   6    //pin led on
///////////////pines arduino/////////////////////
#define led1          13
#define led2          4
#define mot_i1        7
#define mot_d1        8
#define sensores      6
#define battery		  7
#define potenciometer 6
#define boton_1       0
#define boton_2       1
#define pin_pwm_i     9
#define pin_pwm_d     10
//////////////////////////////////////////////////////////////
QTRSensorsRC qtrrc((unsigned char[]) {19, 18, 17, 16,15,14,11,12},NUM_SENSORS, TIMEOUT, EMITTER_PIN);
////////////////Variables para almacenar valores de sensores y posicion
unsigned int sensorValues[NUM_SENSORS];
unsigned int position=0;
////////////////Variables para el pid////////////////////////
int  derivativo=0;
int  proporcional=0; 
int integral=0; //parametros
int salida_pwm=0;
int proporcional_pasado=0;
int velocidad=120; //maximo es 255
float KP=1.9, KD=0, KI=0;  //constantes
////////////////Variables auxilares
int k=0, btn1=1, btn2=1;
float vres=0, vbat=0,vcelda=0;
long t=0,l=0;
int sensado=0,sw=0;  //0 para linea negra,  1 para linea blanca
int compensacion=0;
int vel_comp=0;
float dt=0;
float tiempo=0;
ISR(TIMER2_OVF_vect)
{
	k++;
	t++;
	l++;
	if(t>=100000000)
	{
		t=0;
	}
	//aumento pwm cada minuto
	if(l>=25000) //aumentocada minuto 1/2 para compensar caida de volt bateria
	{
		vel_comp=vel_comp+1;
		l=0;
		//digitalWrite(led2, !digitalRead(led2));
	}
	if(k>=500)
	{	
		if(sw==1)
		{
			velocidad=vel_pot()+vel_comp;
		}
		k=0;
	}
	botones();
	//digitalWrite(led2, !digitalRead(led2));
}
void setup()
{
	delay(50);
	TCCR1A = 0x00; //pwm modo correcicion de fase y frecuencia
	TCCR1B = 0x11;	//prescaler 1x11->1(400hz), 0x12->8,0x13->64(36khz)
	ICR1 =255;      //valor top pwm(anlog write
	pinMode(mot_i1, OUTPUT);
	pinMode(mot_d1, OUTPUT);
	pinMode(sensores, OUTPUT);
	pinMode(led2, OUTPUT);
	pinMode(sensores, OUTPUT);
	leds(4);
	delay(150);
	/////calibracion de sensores
	calibracion_sensores();
	/////timer 2 a 16.384 ms
	configuracion_timer();
	digitalWrite(led1,HIGH);
	while(true)
	{
		bateria();
		if(vcelda<=3.6 && vcelda>=1) //bateria descargada
		{leds(3); delay(300);leds(3);}
		
		if(vcelda<=1){leds(1); delay(300); leds(1);}
		
		if(btn2==0)
		{
			delay(20);leds(4); delay(200); break;
			// <- poner aqui velocidad
		}
		if(btn1==0)
		{
			delay(20);velocidad=vel_pot(); leds(2); delay(200); sw=1; break;
		}
	}
	leds(5);

// velocidad=120; 
KP=0.01;
KD=2;
KI=0.001;
TCCR1B = 0x13;
Serial.begin(115200);
}


void loop()
{
	//*****los valores de cada sensor van desde 0(blanco) a 1000(negro)
	//***** 
	//difuso: 1 activa difuso, 0 desactiva difuso
	//sensado: 1 linea blanca, 0 linea negra. 
	//flaco: aumenta o disminuye el valor de  los sensores para mejorar sensado, se cambia para que sense mejor en cada tipo de linea
			//500 linea blanca, -200 linea negra
	//linea: a partir de ese valor la considera   si esta en linea o no para el seguimiento 
	//ruido: umbral para el cual considerara si es ruido o no 
	
	
	
	//  pwm     dif? sen flan,lin, ruido
	pid(velocidad,1, 0 ,0, 400, 60); //para linea negra
	//pid(velocidad,1, 1, 500, 400, 50);// para linea blanca
	//motores(-velocidad,-velocidad);
	funcion_pausa();
	frenos_contorno(3,100);
	//sen,flan,lin,rui
	//test(0,0,400,60);
}



void frenos_contorno(int tipo,int flanco_comparacion)
{
	//TCCR1B = 0x13;	//prescaler 1x11->1, 0x12->8,0x13->64

	if(tipo==1) //fondo blanco
	{
		if (position<=1000) //si se salio por la parte derecha de la linea
		{
			motores(-250,0); //debido a la inercia, el motor
			while(true)
			{
                                motores(-255,0);
                                delay(3);
                                motores(0,65);
                                delay(1);
				qtrrc.read(sensorValues); //lectura en bruto de sensor
				if ( sensorValues[0]>flanco_comparacion||sensorValues[1]>flanco_comparacion||sensorValues[2]>flanco_comparacion)
				{
					break;
				}
			}
		}
		if (position>=6000) //si se salio por la parte izquierda de la linea
		{
			motores(0,-250);
			while(true)
			{
                                
                                motores(0,-255);
                                delay(3);
                                motores(65,0);
                                delay(1);
				qtrrc.read(sensorValues);
				if (sensorValues[7]>flanco_comparacion||sensorValues[6]>flanco_comparacion||sensorValues[5]>flanco_comparacion)
				{
					break;
				}
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////////////
	if(tipo==2) //para linea blanca con fondo negro
	{
		if (position<=0) //si se salio por la parte derecha de la linea
		{
			motores(-180,90); //debido a la inercia, el motor
			while(true)
			{
				motores(-255,0);
				delay(3);
				motores(0,65);
				delay(1);
				qtrrc.read(sensorValues); //lectura en bruto de sensor
				if ( sensorValues[0]<flanco_comparacion||sensorValues[1]<flanco_comparacion||sensorValues[2]<flanco_comparacion )
				{
					break;
				}
				
			}
		}

		if (position>=6500) //si se salio por la parte izquierda de la linea
		{
			motores(90,-180);
			while(true)
			{
				
				motores(0,-255);
				delay(3);
				motores(65,0);
				delay(1);
				qtrrc.read(sensorValues);
				if (sensorValues[7]<flanco_comparacion || sensorValues[6]<flanco_comparacion|| sensorValues[5]<flanco_comparacion)
				{
					break;
				}
			}
		}
	}
	///////////////////////////7
	////////////////////////////
	
	if(tipo==3) //fondo blanco con todo atras
	{
		if (position<=0) //si se salio por la parte derecha de la linea
		{
			while(true)
			{
				
				motores(-140,70);
				qtrrc.read(sensorValues); //lectura en bruto de sensor
				if ( sensorValues[0]<700|sensorValues[1]<700||sensorValues[2]<700)
				{
					break;
				}
			}
		}
            
		

		if (position>=7000) //si se salio por la parte izquierda de la linea
		{
			while(true)
			{
                              
                              motores(70,-140);
				qtrrc.read(sensorValues);
				if (sensorValues[7]<700||sensorValues[6]<700||sensorValues[5]<700)
				{
					break;
				}
			}
		}
         
	}

	/////////////////////////////////////////////////////////////////////////////////
	if(tipo==4) //para linea blanca con fondo negro
	{
		if (position<=500) //si se salio por la parte derecha de la linea
		{
			while(true)
			{
				motores(-100,70);
				qtrrc.read(sensorValues); //lectura en bruto de sensor
				if ( sensorValues[0]<flanco_comparacion||sensorValues[1]<flanco_comparacion||sensorValues[2]<flanco_comparacion )
				{
					break;
				}
				
			}
		}

		if (position>=6500) //si se salio por la parte izquierda de la linea
		{
			while(true)
			{
			    motores(70,-100);	
				qtrrc.read(sensorValues);
				if (sensorValues[7]<flanco_comparacion || sensorValues[6]<flanco_comparacion|| sensorValues[5]<flanco_comparacion)
				{
					break;
				}
			}
		}
	}
	
	////////////////////////////
	
	//TCCR1B = 0x13;	//prescaler 1x11->1, 0x12->8,0x13->64
}

void pid(int velocidad,int difusso, int sensado, int flanco_color, int en_linea, int ruido )
{
		
	if(difusso==1)
	{
		difuso();
	}	
	 position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, sensado,flanco_color,en_linea,ruido);
	//Serial.println(proporcional);
	//0 para linea
	//negra, 1 para linea blanca
	proporcional = position - 3500; // set point es 3500, asi obtenemos el error
	integral=integral + proporcional; //obteniendo integral
	derivativo = (proporcional - proporcional_pasado); //obteniedo el derivativo
	if (integral>255) integral=1000; //limitamos la integral para no cusar problemas
	if (integral<-255) integral=-1000;
	salida_pwm = proporcional * KP  +  derivativo * KD + integral*KI;
	
	if (  salida_pwm > velocidad )  salida_pwm = velocidad; //limitamos la salida de pwm
	if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;
	
	if (salida_pwm < 0)
	{
          int der=velocidad-salida_pwm;
          if (der>255) der=255;
		motores(velocidad+salida_pwm, der);
	}
	if (salida_pwm >0)
	{
                int izq=salida_pwm+salida_pwm;
                if(izq>255) izq=255;
		motores(izq, velocidad-salida_pwm);
	}
	proporcional_pasado = proporcional;
		
}
///////////////////////////////////////////////////////////////////////
void difuso()
{
	if(position>=0 && position<=1000)       {KP=0.1;  KD=8.2;KI=0.8; }
	if(position>=1000 && position<=1500)    {KP=0.1;  KD=8.2; KI=0.8;}
	if(position>=1500 && position<=2000)    {KP=0.1;  KD=4.2; KI=0.08;}
	if(position>=2000 && position<=2500)	{KP=0.08;  KD=4.2; KI=0.08;}
	if(position>=2500 && position<=3000)	{KP=0.08;    KD=2.2; KI=0.08;}
	if(position>=3000 && position<=3200)     {KP=0.08;    KD=2; KI=0.001;}
	if(position>=3200 && position<=3800)     {KP=0.08;    KD=2; KI=0.001 ; } //
	if(position>=3800 && position<=4000)    {KP=0.08;    KD=2; KI=0.001; }
	if(position>=4000 && position<=4500)	{KP=0.08;    KD=2.2; KI=0.08;}
	if(position>=4500 && position<=5000)	{KP=0.08;;   KD=4.2; KI=0.08;}
	if(position>=5000 && position<=5500)    {KP=0.1;  KD=4.2; KI=0.08;}
	if(position>=5500 && position<=6000)    {KP=0.1;  KD=8.2; KI=0.8;}
	if(position>=6000 && position<=7000)    {KP=0.1;  KD=8.2;KI=0.8;}
}

void motores(int motor_izq, int motor_der)
{
	if ( motor_izq >= 0 )  //motor izquierdo
	{
		digitalWrite(mot_i1,HIGH); // con high avanza
		analogWrite(pin_pwm_i,255-motor_izq); //se controla de manera
		//inversa para mayor control
	}
	else
	{
		digitalWrite(mot_i1,LOW); //con low retrocede
		motor_izq = motor_izq*(-1); //cambio de signo
		analogWrite(pin_pwm_i,motor_izq);
	}


	if ( motor_der >= 0 ) //motor derecho
	{
		digitalWrite(mot_d1,HIGH);
		analogWrite(pin_pwm_d,255-motor_der);
	}
	else
	{
		digitalWrite(mot_d1,LOW);
		motor_der= motor_der*(-1);
		analogWrite(pin_pwm_d,motor_der);
	}
}

void botones()
{
	btn1=digitalRead(boton_2);btn2=digitalRead(boton_1); ///boton izquierdo, derecho
}

void leds(int tipo)
{if(tipo==1)
	{   digitalWrite(led1,HIGH); delay(200); digitalWrite(led1,LOW); delay(200);}
	if(tipo==2)
	{   digitalWrite(led2,HIGH); delay(200); digitalWrite(led2,LOW); delay(200);}
	if(tipo==3)
	{ digitalWrite(led1,HIGH);digitalWrite(led2,LOW);  delay(200); digitalWrite(led1,LOW);digitalWrite(led2,HIGH);  delay(200);}
	if(tipo==4)
	{digitalWrite(led1,HIGH); digitalWrite(led2,HIGH); delay(200); digitalWrite(led1,LOW);digitalWrite(led2,LOW);  delay(200);}
	if(tipo==5)
	{  digitalWrite(led1,LOW); digitalWrite(led2,LOW); }
	if(tipo==6)
	{  digitalWrite(led1,HIGH); digitalWrite(led2,HIGH);  delay(200); digitalWrite(led1,HIGH); digitalWrite(led2,LOW);  delay(200); digitalWrite(led1,LOW); digitalWrite(led2,LOW);  delay(200);}
}

void configuracion_timer()
{
	TCCR2A = (1<<COM2A1) | (0<<COM2A0) | (0<<COM2B1)| (0<<COM2B0) | (0<<3) | (0<<2) | (0<<WGM21) | (0<<WGM20);
	TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (0<<5) | (0<<4) | (0<<WGM22) | (1<<CS22) | (0<<CS21) | (0<<CS20);
	TIMSK2 =(0<<7) | (0<<6) | (0<<5) | (0<<4) | (0<<3) | (0<<OCIE2B) | (0<<OCIE2A) | (1<<TOIE2);
	ASSR = (0<<7) | (0<<EXCLK) | (0<<AS2) | (0<<TCN2UB) | (0<<OCR2AUB) | (0<<OCR2BUB) | (0<<TCR2AUB) | (0<<TCR2BUB) ;
	sei(); //activa interrupcion por timer2
}

void calibracion_sensores()
{
	for (int i = 0; i < 50; i++)  //calibracion durante 2.5 segundos
	{	digitalWrite(led2, HIGH); delay(20);
		qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
	digitalWrite(led2, LOW);  delay(20);}
}

void bateria()
{
	int x=analogRead(battery); vres=(0.004883*x);  //5/1024  escala
	vbat=(vres*2.7843) ;vcelda=vbat/2;
}
int vel_pot()
{
	int r=analogRead(potenciometer);
	r=r/5.12;
	return r;
}

int pot()
{
	int x=analogRead(potenciometer); return x;
}

void test(int sensado, int flanco_color, int en_linea, int ruido)
{
	Serial.begin(9600);
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(qtrrc.calibratedMinimumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
	// print the calibration maximum values measured when emitters were on
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(qtrrc.calibratedMaximumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
	Serial.println();
	delay(1000);
	while(true)
	{
		//qtrrc.read(sensorValues); /// instead of unsigned int position = qtrrc.readLine(sensorValues);
		unsigned int position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, sensado,flanco_color,en_linea,ruido);
		for (unsigned char i = 0; i < NUM_SENSORS; i++)
		{
			Serial.print(sensorValues[i]);
			Serial.print('\t');
		}
		delay(50);
		Serial.println(position); // comment this line out if you are using raw values
		Serial.print('\t');Serial.print('\t');
		Serial.print(vcelda);	Serial.print('\t');
		Serial.print(vbat);	Serial.print('\t');
		Serial.print(pot());	Serial.print('\t');
		bateria();
	}
}

void funcion_pausa()
{
	if(btn1==0)
	{
		delay(190); btn1=1;
		while(true)
		{
			motores(0,0);
			if(btn1==0)
			{
				digitalWrite(led1,LOW); delay(190); btn1=1;;break;
			}}}}
