#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "acionna.h"
#include "loadCell.h"
#include <time.h>
#include <stm32f10x.h>
#include <stm32f10x_rtc.h>
#include "stm32f10x_it.h"
#include "Hardware/adc.h"

#include "nokia5110/nokia5110.h"
#include "nokia5110/fonts/fonts.h"

ACIONNA acn;
ADC convert;

LOADCELL weight1;
LOADCELL weight2;
LOADCELL weight3;
LOADCELL weight4;

NOKIA5110 glcd;

// Wake up interrupts
//uint8_t flag_WDRF = 0;			// Watchdog System Reset Flag
//uint8_t flag_BORF = 0;			// Brown-out Reset Flag
//uint8_t flag_EXTRF = 0;			// External Reset Flag
//uint8_t flag_PORF = 0;			// Power-on Reset Flag
//uint16_t var = 0x0003;
uint8_t flag_stable = 0;
int count = 0;
uint8_t tareOrder = 0;
double A[5];
double Kc[5];
int P[5];
uint8_t mode = 0;

void showResults()
{
	P[0] = P[1]+P[2]+P[3]+P[4];

	switch (mode)
	{
		case 0:	// kilogram digit only
			if(P[0]>0)
				sprintf(glcd.buffer,"%5.1d", P[0]/1000+500);			// 0 digitos;
			else
				sprintf(glcd.buffer,"%5.1d", P[0]/1000-500);			// 0 digitos;

			glcd.glcd_Arial16x24_str(4,1,glcd.buffer);
			glcd.glcd_put_string(72,5,"kg");					// unit print;
			break;
		case 1:	// kilogram digit and decimal floating point
			if(P[0]/100%10 < 0)
			{
				if((P[0]/1000) == 0)
				{
					glcd.glcd_dot_print(35,2,9,0xC0);					// minus signal;
					glcd.glcd_dot_print(35,3,9,0x01);					// minus signal;
					glcd.glcd_Arial16x24_str(52,1,"0");					// 0 digitos;
				}

				sprintf(glcd.buffer,"%.1d", abs(P[0])/100%10);	// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);

			}
			else
			{
				sprintf(glcd.buffer,"%4.1d", P[0]/1000);				// 0 digitos;
				glcd.glcd_Arial16x24_str(4,1,glcd.buffer);

				sprintf(glcd.buffer,"%.1d", P[0]/100%10);			// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
			}
			glcd.glcd_dot_print(65,4,3,0x07);						// dot digit;
			glcd.glcd_put_string(72,5,"kg");						// unit print;
			break;
		case 2:	// All weights with 2 decimal floating point
			sprintf(glcd.buffer,"Pt: %7.1d.%.2d", P[0]/1000, (P[0]/10)%100);	// 2 digitos;
			glcd.glcd_put_string(0,1,glcd.buffer);

			sprintf(glcd.buffer,"P1: %7.1d.%.2d", P[1]/1000, abs(P[1]/10)%100);	// 2 digitos;
			glcd.glcd_put_string(0,2,glcd.buffer);

			sprintf(glcd.buffer,"P2: %7.1d.%.2d", P[2]/1000, abs(P[2]/10)%100);	// 2 digitos;
			glcd.glcd_put_string(0,3,glcd.buffer);

			sprintf(glcd.buffer,"P3: %7.1d.%.2d", P[3]/1000, abs(P[3]/10)%100);	// 2 digitos;
			glcd.glcd_put_string(0,4,glcd.buffer);

			sprintf(glcd.buffer,"P4: %7.1d.%.2d", P[4]/1000, abs(P[4]/10)%100);	// 2 digitos;
			glcd.glcd_put_string(0,5,glcd.buffer);

			sprintf(glcd.buffer,"s%d", mode);						// mode print;
			glcd.glcd_put_string(72,0,glcd.buffer);
			break;
		case 3:	// Sensor 1 debugging
		{
			if(P[1]/100%10 < 0)
			{
				if((P[1]/1000) == 0)
				{
					glcd.glcd_dot_print(35,2,9,0xC0);					// minus signal;
					glcd.glcd_dot_print(35,3,9,0x01);					// minus signal;
					glcd.glcd_Arial16x24_str(52,1,"0");				// 0 digitos;
				}

				sprintf(glcd.buffer,"%.1d", abs(P[1])/100%10);	// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);

			}
			else
			{
				sprintf(glcd.buffer,"%4.1d", P[1]/1000);				// 0 digitos;
				glcd.glcd_Arial16x24_str(4,1,glcd.buffer);

				sprintf(glcd.buffer,"%.1d", P[1]/100%10);			// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
			}
			glcd.glcd_dot_print(65,4,3,0x07);						// dot digit;
			glcd.glcd_put_string(72,5,"kg");						// unit print;

			sprintf(glcd.buffer,"o: %8d", weight1.offset);		// offset show;
			glcd.glcd_put_string(0,5,glcd.buffer);

			sprintf(glcd.buffer,"s: %8d", weight1.signal);		// signal show;
			glcd.glcd_put_string(0,0,glcd.buffer);

			sprintf(glcd.buffer,"s%d", 1);						// mode print;
			glcd.glcd_put_string(72,0,glcd.buffer);
		}
		break;
		case 4:	// Sensor 2 debugging
		{
			if(P[2]/100%10 < 0)
			{
				if((P[2]/1000) == 0)
				{
					glcd.glcd_dot_print(35,2,9,0xC0);					// minus signal;
					glcd.glcd_dot_print(35,3,9,0x01);					// minus signal;
					glcd.glcd_Arial16x24_str(52,1,"0");				// 0 digitos;
				}

				sprintf(glcd.buffer,"%.1d", abs(P[2])/100%10);	// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);

			}
			else
			{
				sprintf(glcd.buffer,"%4.1d", P[2]/1000);				// 0 digitos;
				glcd.glcd_Arial16x24_str(4,1,glcd.buffer);

				sprintf(glcd.buffer,"%.1d", P[2]/100%10);			// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
			}
			glcd.glcd_dot_print(65,4,3,0x07);						// dot digit;
			glcd.glcd_put_string(72,5,"kg");						// unit print;

			sprintf(glcd.buffer,"o: %8d", weight2.offset);		// offset show;
			glcd.glcd_put_string(0,5,glcd.buffer);

			sprintf(glcd.buffer,"s: %8d", weight2.signal);		// signal show;
			glcd.glcd_put_string(0,0,glcd.buffer);

			sprintf(glcd.buffer,"s%d", 2);						// mode print;
			glcd.glcd_put_string(72,0,glcd.buffer);
		}
		break;
		case 5:	// Sensor 3 debugging
		{
			if(P[3]/100%10 < 0)
			{
				if((P[3]/1000) == 0)
				{
					glcd.glcd_dot_print(35,2,9,0xC0);					// minus signal;
					glcd.glcd_dot_print(35,3,9,0x01);					// minus signal;
					glcd.glcd_Arial16x24_str(52,1,"0");				// 0 digitos;
				}

				sprintf(glcd.buffer,"%.1d", abs(P[3])/100%10);	// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);

			}
			else
			{
				sprintf(glcd.buffer,"%4.1d", P[3]/1000);				// 0 digitos;
				glcd.glcd_Arial16x24_str(4,1,glcd.buffer);

				sprintf(glcd.buffer,"%.1d", P[3]/100%10);			// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
			}
			glcd.glcd_dot_print(65,4,3,0x07);						// dot digit;
			glcd.glcd_put_string(72,5,"kg");						// unit print;

			sprintf(glcd.buffer,"o: %8d", weight3.offset);		// offset show;
			glcd.glcd_put_string(0,5,glcd.buffer);

			sprintf(glcd.buffer,"s: %8d", weight3.signal);		// signal show;
			glcd.glcd_put_string(0,0,glcd.buffer);

			sprintf(glcd.buffer,"s%d", 3);						// mode print;
			glcd.glcd_put_string(72,0,glcd.buffer);
		}
		break;
		case 6:	// Sensor 4 debugging
		{
			if(P[4]/100%10 < 0)
			{
				if((P[4]/1000) == 0)
				{
					glcd.glcd_dot_print(35,2,9,0xC0);					// minus signal;
					glcd.glcd_dot_print(35,3,9,0x01);					// minus signal;
					glcd.glcd_Arial16x24_str(52,1,"0");				// 0 digitos;
				}

				sprintf(glcd.buffer,"%.1d", abs(P[4])/100%10);	// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);

			}
			else
			{
				sprintf(glcd.buffer,"%4.1d", P[4]/1000);				// 0 digitos;
				glcd.glcd_Arial16x24_str(4,1,glcd.buffer);

				sprintf(glcd.buffer,"%.1d", P[4]/100%10);			// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
			}
			glcd.glcd_dot_print(65,4,3,0x07);						// dot digit;
			glcd.glcd_put_string(72,5,"kg");						// unit print;

			sprintf(glcd.buffer,"o: %8d", weight4.offset);		// offset show;
			glcd.glcd_put_string(0,5,glcd.buffer);

			sprintf(glcd.buffer,"s: %8d", weight4.signal);		// signal show;
			glcd.glcd_put_string(0,0,glcd.buffer);

			sprintf(glcd.buffer,"s%d", 4);						// mode print;
			glcd.glcd_put_string(72,0,glcd.buffer);
		}
		break;
		case 7: // Small Sensor 1 debugging
		{
			if(P[1]/100%10 < 0)
			{
				if((P[1]/1000) == 0)
				{
					glcd.glcd_dot_print(35,2,9,0xC0);					// minus signal;
					glcd.glcd_dot_print(35,3,9,0x01);					// minus signal;
					glcd.glcd_Arial16x24_str(52,1,"0");				// 0 digitos;
				}

				sprintf(glcd.buffer,"%.1d", abs(P[1])/100%10);	// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);

			}
			else
			{
				sprintf(glcd.buffer,"%4.1d", P[1]/1000);				// 0 digitos;
				glcd.glcd_Arial16x24_str(4,1,glcd.buffer);

				sprintf(glcd.buffer,"%.1d", P[1]/100%10);			// 1 digitos; divide by centesimum part and use 10 by 10.
				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
			}
			glcd.glcd_dot_print(65,4,3,0x07);						// dot digit;
			glcd.glcd_put_string(72,5," g");						// unit print;

			sprintf(glcd.buffer,"o: %8d", weight1.offset);		// offset show;
			glcd.glcd_put_string(0,5,glcd.buffer);

			sprintf(glcd.buffer,"s: %8d", weight1.signal);		// signal show;
			glcd.glcd_put_string(0,0,glcd.buffer);

			glcd.glcd_put_string(72,0,"sm");
		}
			break;
//		case 6:
////			P[0] = P[1]+P[2]+P[3]+P[4];
//
//			sprintf(glcd.buffer,"Pt: %7.1d.%.1d", abs(P[0])/1000, abs(P[0])/100%10);			// 1 digitos;
//			glcd.glcd_put_string(0,1,glcd.buffer);
//
//			sprintf(glcd.buffer,"P1: %7.1d.%.1d", abs(P[1])/1000, abs(P[1])/100%10);			// 1 digitos;
//			glcd.glcd_put_string(0,2,glcd.buffer);
//
//			sprintf(glcd.buffer,"P2: %7.1d.%.1d", abs(P[2])/1000, abs(P[2])/100%10);			// 1 digitos;
//			glcd.glcd_put_string(0,3,glcd.buffer);
//
//			sprintf(glcd.buffer,"P3: %7.1d.%.1d", abs(P[3])/1000, abs(P[3])/100%10);			// 1 digitos;
//			glcd.glcd_put_string(0,4,glcd.buffer);
//
//	//		sprintf(glcd.buffer,"P4: %7.1d", P[4]);				// whole digits in GRAMS [g];
//			sprintf(glcd.buffer,"P4: %7.1d.%.1d", abs(P[4])/1000, abs(P[4])/100%10);			// 1 digitos;
//			glcd.glcd_put_string(0,5,glcd.buffer);
//
//			sprintf(glcd.buffer,"s%d", mode);						// mode print;
//			glcd.glcd_put_string(72,0,glcd.buffer);
//			break;
//		case 7: // the whole weight
////			P[0] = P[1]+P[2]+P[3]+P[4];
//
//			weight1.offset = 11500;
//			weight2.offset = 5000;
//			weight3.offset = 11500;
//			weight4.offset = 11500;
//
//			static const int index = 0;
//
//			if(P[index]/100%10 < 0)
//			{
//				if((P[0]/1000) == 0)
//				{
//					glcd.glcd_dot_print(35,2,9,0xC0);				// minus signal;
//					glcd.glcd_dot_print(35,3,9,0x01);				// minus signal;
//					glcd.glcd_Arial16x24_str(52,1,"0");				// 0 digitos;
//				}
//
//				sprintf(glcd.buffer,"%.1d", abs(P[index])/100%10);	// 1 digitos; divide by centesimum part and use 10 by 10.
//				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
//
//			}
//			else
//			{
//				sprintf(glcd.buffer,"%4.1d", P[index]/1000);		// 0 digitos;
//				glcd.glcd_Arial16x24_str(4,1,glcd.buffer);
//
//				sprintf(glcd.buffer,"%.1d", P[index]/100%10);		// 1 digitos; divide by centesimum part and use 10 by 10.
//				glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
//			}
//			glcd.glcd_dot_print(65,4,3,0x07);						// dot digit;
//			glcd.glcd_put_string(72,5,"kg");						// unit print;
//
//			sprintf(glcd.buffer,"o: %8d", weight1.offset);		// offset show;
//			glcd.glcd_put_string(0,5,glcd.buffer);
//
//			sprintf(glcd.buffer,"s: %8d", weight1.signal);		// signal show;
//			glcd.glcd_put_string(0,0,glcd.buffer);
//
//			sprintf(glcd.buffer,"s%d", mode);						// mode print;
//			glcd.glcd_put_string(72,0,glcd.buffer);
//			break;
//		case 8:
//			sprintf(glcd.buffer,"%.1d.%.2d   %.1d.%.2d", P[1]/1000, abs(P[1]/10)%100, P[2]/1000, abs(P[2]/10)%100);	// 2 digitos;
//			glcd.glcd_put_string(0,0,glcd.buffer);
//			sprintf(glcd.buffer,"%.1d.%.2d   %.1d.%.2d", P[3]/1000, abs(P[3]/10)%100, P[4]/1000, abs(P[4]/10)%100);	// 2 digitos;
//			glcd.glcd_put_string(0,1,glcd.buffer);
//
//			sprintf(glcd.buffer,"%d", weight1.signal);		// signal show;
//			glcd.glcd_put_string(0,2,glcd.buffer);
//			sprintf(glcd.buffer,"%d", weight1.offset);		// offset show;
//			glcd.glcd_put_string(46,2,glcd.buffer);
//
//			sprintf(glcd.buffer,"%d", weight2.signal);		// signal show;
//			glcd.glcd_put_string(0,3,glcd.buffer);
//			sprintf(glcd.buffer,"%d", weight2.offset);		// offset show;
//			glcd.glcd_put_string(46,3,glcd.buffer);
//
//			sprintf(glcd.buffer,"%d", weight3.signal);		// signal show;
//			glcd.glcd_put_string(0,4,glcd.buffer);
//			sprintf(glcd.buffer,"%d", weight3.offset);		// offset show;
//			glcd.glcd_put_string(46,4,glcd.buffer);
//
//			sprintf(glcd.buffer,"%d", weight4.signal);		// signal show;
//			glcd.glcd_put_string(0,5,glcd.buffer);
//			sprintf(glcd.buffer,"%d", weight4.offset);		// offset show;
//			glcd.glcd_put_string(46,5,glcd.buffer);
//
////				sprintf(glcd.buffer,"s%d", mode);						// mode print;
////				glcd.glcd_put_string(72,0,glcd.buffer);
//			break;
//		default:
////				sprintf(glcd.buffer,"%4.1d", abs(P[0])/1000);			// 1 digitos;
////				sprintf(glcd.buffer,"%3.1d.%.1d", abs(P[mode])/1000, abs(P[mode])/100%10);			// 1 digitos;
////				sprintf(glcd.buffer,"%3.1d.%.1d", abs(P[mode])/1000, abs(P[mode])/100%10);			// 1 digitos;
//
////				sprintf(glcd.buffer,"%.1d", abs(P[0])/100%10);		// 1 digitos;
////				sprintf(glcd.buffer,"%.2d", abs(P[0])/10%100);		// 2 digitos;
////				sprintf(glcd.buffer,"%.3d", abs(P[0])%1000);			// 3 digitos;
////				glcd.glcd_big_str(8,2,glcd.buffer);
//
//			sprintf(glcd.buffer,"%4.1d", abs(P[mode])/1000);			// 0 digitos;
//			glcd.glcd_Arial16x24_str(4,1,glcd.buffer);
//			glcd.glcd_dot_print(65,4,3,0x07);
//			sprintf(glcd.buffer,"%.1d", abs(P[mode])/100%10);		// 1 digitos;
//			glcd.glcd_Arial16x24_str(68,1,glcd.buffer);
//
//			sprintf(glcd.buffer,"s%d", mode);
//			glcd.glcd_put_string(0,0,glcd.buffer);
//			glcd.glcd_put_string(72,5,"kg");
//		break;
	}
}
void initialize()
{
	A[0] = 1.0000;			// 1kg load bar
	A[1] = 2.9997;			// s1
	A[2] = 3.0000;			// s2
	A[3] = 3.0017;			// s3
	A[4] = 3.0012;			// s4

	Kc[0] = 10*1.3634;
	Kc[1] = 1.0872;
	Kc[2] = 1.6200;			// YZC-320 tested on 20170816 with 3.0mV/V and Kc = 1.6985;
	Kc[3] = 1.0872;			//10.33%
	Kc[4] = 1.0872;

	weight1.begin_loadcell(32, 31, A[1], Kc[1]);
	weight2.begin_loadcell(30, 29, A[2], Kc[2]);
	weight3.begin_loadcell(19, 18, A[3], Kc[3]);
	weight4.begin_loadcell(17, 16, A[4], Kc[4]);

	weight1.offset = 11500;	// s1
	weight2.offset = 5000;	// s2
	weight3.offset = 11500; // s3
	weight4.offset = 11500;	// s4

	if(mode == 7)
	{
		weight1.A = A[0];
		weight1.Kp = Kc[0];
	}
	if(mode<7)
	{
		weight1.A = A[1];
		weight1.Kp = Kc[1];

		weight2.A = A[2];
		weight2.Kp = Kc[2];

		weight3.A = A[3];
		weight3.Kp = Kc[3];

		weight4.A = A[4];
		weight4.Kp = Kc[4];
	}

	// Initializing...
	glcd.glcd_clear2();
	weight1.drive_beep(1, 100, 0);
	strcpy(glcd.buffer, "Inicializando");
	glcd.glcd_put_string(0,2,glcd.buffer);

	strcpy(glcd.buffer, "thmalmeida SYS");
	glcd.glcd_put_string(0,6,glcd.buffer);

	int i;
	for(i=0;i<50;i++)
	{
		P[1] = weight1.get_weight();
		P[2] = weight2.get_weight();
		P[3] = weight3.get_weight();
		P[4] = weight4.get_weight();
	}
	glcd.glcd_clear2();
}

int main(void)
{
	init();							// uC basic peripherals setup

//	acn.begin_acn();				// Class acionna statement
//	Serial.begin(9600);				// Initialize USART1 @ 9600 baud
//	Serial.println("Acionna v2.0");
//	acn.blink_led(2, 150);
	rtc.begin_rtc(rtc.rtc_clkSource, rtc.rtc_PRL);	// must be after eeprom init to recover rtc.rtc_PRL on flash
	glcd.glcd_init(1);

//	int c = 0;
//	while(1)
//	{
//		sprintf(glcd.buffer, "%d", c);
//		glcd.glcd_put_string(0,0,glcd.buffer);
//		c++;
//	}

	mode = 0;

	initialize();

	while(1)
	{
		P[1] = weight1.get_weight();
		P[2] = weight2.get_weight();
		P[3] = weight3.get_weight();
		P[4] = weight4.get_weight();

		count = 0;
		while(weight1.readTareButton())
		{
			count++;
			tareOrder = 1;

			if(count > 20)
			{
				tareOrder = 0;
				count = 0;
				mode++;
				weight1.drive_beep(2, 10, 10);
				if(mode == 7)
				{
					weight1.A = A[0];
					weight1.Kp = Kc[0];
				}
				if(mode > 7)
				{
					mode = 0;
					weight1.A = A[1];
					weight1.Kp = Kc[1];

					weight2.A = A[2];
					weight2.Kp = Kc[2];

					weight3.A = A[3];
					weight3.Kp = Kc[3];

					weight4.A = A[4];
					weight4.Kp = Kc[4];
				}
				glcd.glcd_clear2();
				sprintf(glcd.buffer,"MODE %d", mode);			// 1 digito;
				glcd.glcd_put_string(20,2,glcd.buffer);
				while(weight1.readTareButton());
				glcd.glcd_clear2();
			}
		}

		if(tareOrder)
		{
			tareOrder = 0;

			weight1.tareSystem3();
			weight2.tareSystem3();
			weight3.tareSystem3();
			weight4.tareSystem3();
			glcd.glcd_clear2();
			weight1.drive_beep(1, 10, 0);
		}

		if(acn.flag_1s == 1)
		{
			acn.flag_1s = 0;

			showResults();

			switch(mode)
			{
			case 7:
				if(!weight1.stable)
				{
					flag_stable = 0;
				}
				if(!flag_stable)
				{
					if(weight1.stable)
					{
						flag_stable = 1;

						weight1.drive_led(1);
						if(P[1] > 5000)
						{
							weight1.drive_beep(1, 200, 0);
						}
					}
					else
					{
						weight1.drive_led(0);
					}
				}
				break;
			default:
				if(!weight1.stable || !weight2.stable || !weight3.stable || !weight4.stable)
				{
					flag_stable = 0;
				}

				if(!flag_stable)
				{
					if(weight1.stable && weight2.stable && weight3.stable && weight4.stable)
					{
						flag_stable = 1;

						weight1.drive_led(1);
						if(P[0] > 5000)
						{
							weight1.drive_beep(1, 200, 0);
						}
					}
					else
					{
						weight1.drive_led(0);
					}
				}
				break;
			}
		}
//		sprintf(glcd.buffer,"%4.1d.%.2d", P/(weight.Waccu*10), abs(P%(weight.Waccu))/100);	// 2 digitos

//		sprintf(glcd.buffer,"P:%.3d.%.3d kg", abs(P/1000), abs(P)%1000);		// 3 digitos;
//		sprintf(glcd.buffer,"P:%.3d.%.2d kg", abs(P/1000), abs(P/10)%100);	// 2 digitos;
//		sprintf(glcd.buffer,"%.4d.%.1d", abs(P)/1000, abs(P)/100%10);			// 1 digitos;
//		glcd.glcd_put_string(0,2,glcd.buffer);

//		sprintf(glcd.buffer,"%4.1d", abs(P[0])/1000);		// kilogram digit;
//		glcd.glcd_Arial16x24_str(1,1,glcd.buffer);
//		glcd.glcd_dot_print(65,4,3);
//


//		acn.comm_Bluetooth();
//
//		acn.refreshVariables();
//
//		acn.handleMessage();
//
//		acn.process_Mode();
//		if(acn.flag_1s)
//		{
//			acn.flag_1s = 0;
//
//			rtc.getTime();
//			sprintf(buffer, "%s", ctime(&rtc.rawtime));
//			Serial.println(buffer);
//		}
	}
}

//int main(void)
//{
//	init();							// uC basic peripherals setup
//
//	acn.begin_acn();				// Class acionna statement
////	Serial.begin(9600);				// Initialize USART1 @ 9600 baud
////	Serial.println("Acionna v2.0");
//	acn.blink_led(2, 150);
//	weight.begin_loadcell();
//
////	char c='0';
////
////	while(1)
////	{
////		sprintf(glcd.buffer, "a: %c", c++);
////		Serial.println(glcd.buffer);
////		_delay_ms(500);
////	}
//
////	int c=10;
////	while(1)
////	{
////		sprintf(glcd.buffer,"%4.1d g", c);
////		glcd.glcd_big_str(0,0, glcd.buffer);
////		c++;
////		_delay_ms(500);
////	}
//
//	weight.example1();
//
//	while(1)
//	{
////		acn.comm_Bluetooth();
////
////		acn.refreshVariables();
////
////		acn.handleMessage();
////
////		acn.process_Mode();
//	}
//}

//const char * m()
//{
//    char * c = (char *)malloc(6 * sizeof(char));
//    c = "hello";
//    return (const char *)c;
//}
//
//int main(int argc, char * argv[])
//{
//    const char * d = m();
//    std::cout << d; // use PCSTR
//}


/*
 * Interruptions sequence
 */
extern "C" {
volatile int time_ms;
volatile int time_us;
volatile int count_1s = 1000000;
void SysTick_Handler(void)
{
	if(time_us)
		time_us--;

	if(time_ms)
		time_ms--;

	if(count_1s)
	{
		count_1s--;
	}
	else
	{
//		acn.flag_1s = 1;
		count_1s = 1000000;
	}
}
void RTC_IRQHandler(void)
{
	if(RTC->CRL & RTC_CRL_SECF)
	{
		RTC->CRL &= ~RTC_CRL_SECF;
		rtc.uptime32++;
		acn.flag_1s = 1;
	}
}
void USART1_IRQHandler(void)
{
	// check if the USART1 receive interrupt flag was set
	while(USART_GetITStatus(USART1, USART_IT_RXNE))
	{
//		GPIOC -> ODR ^= (1<<13);
		char t = (USART1->DR); // the character from the USART1 data register is saved in t

		if(Serial._USART1_cnt < MAX_STRLEN)
		{
			Serial._received_string[Serial._USART1_cnt] = t;
			Serial._USART1_cnt++;
		}
		else
		{
			memset(Serial._received_string,0,sizeof(Serial._received_string));
			Serial._USART1_cnt = 0;
		}
	}
}
void SPI1_IRQHandler(void)
{
#ifdef master
	//	static unsigned short int count = 0, i = 0 ;
	//	check if the SPI1 receive interrupt flag was set
	//	Master interrupt
	//	TX: 1- empty
	//	if(SPI1 -> SR & 0x0002)
		if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
		{
			SerialSPI.SPI1_flag_rx = 1;
			SerialSPI.SPI1_rxd++;
			SerialSPI.SPI1_txd = SerialSPI.SPI1_rxd;
			SPI1 -> DR = SerialSPI.SPI1_txd;
	//		Wait until the data has been transmitted.
	//		while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
	//		USART1_print("Transmitting: ");
	//		USART1_putc(0x40);
		}
//	//	if(SPI1 -> SR & SPI_I2S_FLAG_RXNE)
//		//	RX: 1- not empty
//	//	if(SPI1 -> SR & 0x0001)
		if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
		{
			SerialSPI.SPI1_flag_rx = 1;
			SerialSPI.SPI1_rxd = SPI1 -> DR;
			if(SerialSPI.SPI1_cnt < MAX_STRLEN)
			{
				SerialSPI.SPI_received_string[SerialSPI.SPI1_cnt] = SerialSPI.SPI1_rxd;
				SerialSPI.SPI1_cnt++;
			}
			else
			{
				memset(SerialSPI.SPI_received_string, 0, sizeof(SerialSPI.SPI_received_string));
				SerialSPI.SPI1_cnt = 0;
			}
		}

	//	if(SPI1 -> SR & 0x0100)
		if(SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR))
		{
			SerialSPI.SPI1_flag_err = 1;
		}
#else
//	static unsigned short int count = 0, i = 0 ;
//	check if the SPI1 receive interrupt flag was set
//	Slave interrupt
//	TX: 1- empty
//	if(SPI1 -> SR & 0x0002)
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
	{
		SerialSPI.SPI1_rxd++;
		SerialSPI.SPI1_txd = SerialSPI.SPI1_rxd;
		SPI1 -> DR = SerialSPI.SPI1_txd;
//		Wait until the data has been transmitted.
//		while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
//		USART1_print("Transmitting: ");
//		USART1_putc(0x40);
	}
//	if(SPI1 -> SR & SPI_I2S_FLAG_RXNE)
	//	RX: 1- not empty
//	if(SPI1 -> SR & 0x0001)
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	{
		SerialSPI.SPI1_flag_rx = 1;
		SerialSPI.SPI1_rxd = SPI1 -> DR;
		if(SerialSPI.SPI1_cnt < MAX_STRLEN)
		{
			SerialSPI.SPI_received_string[SerialSPI.SPI1_cnt] = SerialSPI.SPI1_rxd;
			SerialSPI.SPI1_cnt++;
		}
		else
		{
			memset(SerialSPI.SPI_received_string, 0, sizeof(SerialSPI.SPI_received_string));
			SerialSPI.SPI1_cnt = 0;
		}
	}

//	if(SPI1 -> SR & 0x0100)
	if(SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR))
	{
		SerialSPI.SPI1_flag_err = 1;
	}
#endif
}
//void SPI2_IRQHandler(void)
//{
//#ifdef master
//	//	static unsigned short int count = 0, i = 0 ;
//	//	check if the SPI1 receive interrupt flag was set
//	//	Master interrupt
//	//	TX: 1- empty
//	//	if(SPI1 -> SR & 0x0002)
//		if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
//		{
//			SerialSPI.SPI1_flag_rx = 1;
//			SerialSPI.SPI1_rxd++;
//			SerialSPI.SPI1_txd = SerialSPI.SPI1_rxd;
//			SPI1 -> DR = SerialSPI.SPI1_txd;
//	//		Wait until the data has been transmitted.
//	//		while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
//	//		USART1_print("Transmitting: ");
//	//		USART1_putc(0x40);
//		}
////	//	if(SPI1 -> SR & SPI_I2S_FLAG_RXNE)
////		//	RX: 1- not empty
////	//	if(SPI1 -> SR & 0x0001)
//		if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//		{
//			SerialSPI.SPI1_flag_rx = 1;
//			SerialSPI.SPI1_rxd = SPI1 -> DR;
//			if(SerialSPI.SPI1_cnt < MAX_STRLEN)
//			{
//				SerialSPI.SPI_received_string[SerialSPI.SPI1_cnt] = SerialSPI.SPI1_rxd;
//				SerialSPI.SPI1_cnt++;
//			}
//			else
//			{
//				memset(SerialSPI.SPI_received_string, 0, sizeof(SerialSPI.SPI_received_string));
//				SerialSPI.SPI1_cnt = 0;
//			}
//		}
//
//	//	if(SPI1 -> SR & 0x0100)
//		if(SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR))
//		{
//			SerialSPI.SPI1_flag_err = 1;
//		}
//#else
////	static unsigned short int count = 0, i = 0 ;
////	check if the SPI1 receive interrupt flag was set
////	Slave interrupt
////	TX: 1- empty
////	if(SPI1 -> SR & 0x0002)
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
//	{
//		SerialSPI.SPI1_rxd++;
//		SerialSPI.SPI1_txd = SerialSPI.SPI1_rxd;
//		SPI1 -> DR = SerialSPI.SPI1_txd;
////		Wait until the data has been transmitted.
////		while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
////		USART1_print("Transmitting: ");
////		USART1_putc(0x40);
//	}
////	if(SPI1 -> SR & SPI_I2S_FLAG_RXNE)
//	//	RX: 1- not empty
////	if(SPI1 -> SR & 0x0001)
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//	{
//		SerialSPI.SPI1_flag_rx = 1;
//		SerialSPI.SPI1_rxd = SPI1 -> DR;
//		if(SerialSPI.SPI1_cnt < MAX_STRLEN)
//		{
//			SerialSPI.SPI_received_string[SerialSPI.SPI1_cnt] = SerialSPI.SPI1_rxd;
//			SerialSPI.SPI1_cnt++;
//		}
//		else
//		{
//			memset(SerialSPI.SPI_received_string, 0, sizeof(SerialSPI.SPI_received_string));
//			SerialSPI.SPI1_cnt = 0;
//		}
//	}
//
////	if(SPI1 -> SR & 0x0100)
//	if(SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR))
//	{
//		SerialSPI.SPI1_flag_err = 1;
//	}
//#endif
//}
}
/*
 * Interruptions sequence END
 */

//unsigned char rxc;
//unsigned char txcm = 0x4d;	// M
//unsigned char txcs = 0x40;	// @
//	unsigned char txcs = 0x53;	// S

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//ErrorStatus  HSEStartUpStatus;
//FLASH_Status FlashStatus;
//uint16_t VarValue = 0;
///* Virtual address defined by the user: 0xFFFF value is prohibited */
//uint16_t VirtAddVarTab[NumbOfVar] = {0x0000, 0x0004, 0x0008};
//uint16_t addr = 0x000C;
//void demo01(void)	// Put this into the while(1) loop function
//{
//	SystemInit();
//	SystemCoreClockUpdate();
//
//	ADC1_Init();
//	IO_Init();
//	USART1_Init(9600); // initialize USART1 @ 9600 baud
//	SysTick_Init();
//
//	// ADC Variables
//	const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
//	const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V
//	uint16_t TemperatureC;
//	int ADC_Value=0;
//	int vd = 0, d1, d2;
//	double Voltage = 0.0, f1;
//	char buffer[50];
//
//	while(1)
//	{
//		LED_green(1);
//		ADC_Value = ADC_Read(ADC_Channel_16);
//	//		printf("\r\n ADC value: %d \r\n", AD_value);
//		TemperatureC = (uint16_t)((V25-ADC_Value)/Avg_Slope+25);
//	//		printf("Temperature: %d%cC\r\n", TemperatureC, 176);
//
//		vd = ADC_Read(ADC_Channel_17);
//		Voltage = (vd-1490.0+4096.0)/4096.0*3.3;
//
//	//		 V = 1.205*4095/ADCval
//
//		d1 = (int) Voltage;
//		f1 = Voltage - d1;
//		d2 = trunc(f1*1000);
//
//		sprintf(buffer,"D: %d, T: %d, V: %d.%04d\r\n",ADC_Value, TemperatureC, d1, d2);
//	//		USART_puts(USART1, "Fuck!\r\n"); // just send a message to indicate that it works
//
//		LED_green(0);
//		delay_ms(500);
//	}
//}
//void uart_demo1(void)
//{
//	char c = 'a';
//	uint8_t flag01 =0;
//
//	if(!flag01)
//	{
//		Serial.write(c++);
//		if(c == 'f')
//		{
//			flag01 = 1;
//		}
//	}
//	else
//	{
//		Serial.write(c--);
//		if(c == 'a')
//		{
//			flag01 = 0;
//		}
//	}
//	_delay_ms(500);
//}
//void USART1_IRQHandler(void)
//{
//	// check if the USART1 receive interrupt flag was set
//	while(USART_GetITStatus(USART1, USART_IT_RXNE))
//	{
//		LED_green_toogle();
//		char t = USART1->DR; // the character from the USART1 data register is saved in t
//		if(Serial._USART1_cnt < MAX_STRLEN)
//		{
//			Serial._received_string[Serial._USART1_cnt] = t;
//			Serial._USART1_cnt++;
//		}
//		else
//		{
////			memset(Serial._received_string,0,sizeof(Serial._received_string));
//			Serial._USART1_cnt = 0;
//		}
//	}
//}
//int main(void)
//{
//	//	SystemCoreClockUpdate();
//	SystemInit(); 		// Setup STM32 system (clock, PLL and Flash configuration)
//
//	ADC1_Init();
//	IO_Init();
//	SysTick_Init();
//	USART1_Init(38400); // initialize USART1 @ 38400 baud
//
//#ifdef master
//	nRF24_Init();
//	USART1_println("Hi, Master!");
//#else
//	nRF24_Init();
//	USART1_println("Hi, Slave!");
//#endif
//
//	while (1)
//	{
//#ifdef master
//		comm_Bluetooth();
//		handleMessage();
//		refresh_variables();
//
//		if(stateMode == 2)
//		{
////				sprintf(buffer,"%2d", nRF24_get_RPD());
////				USART1_println(buffer);
////
////				sprintf(buffer,"%2d", nRF24_read_register(NRF_STATUS));
////				USART1_println(buffer);
////
////				sprintf(buffer,"%2d", nRF24_read_register(FIFO_STATUS));
////				USART1_println(buffer);
//
//			if(nRF24_IRQ())
//			{
//				USART1_println("IRQ Pin");
//
//				int pipeRX = nRF24_poll_RX();
//				if(pipeRX)
//				{
//		//			uint8_t payload_length = nRF24_get_pipe_payload_len(0);
//					uint8_t data[2];
//					nRF24_read_payload(data, 2);
////					nRF24_flush_RX();								// flush rx;
//					sprintf(buffer,"rx: %c, %c", data[0], data[1]);
//					USART1_println(buffer);
//				}
//			}
//		}
//
//		if(flag_1s)
//		{
//			flag_1s ^= flag_1s;
//
//			if(nRF24_flag_send_cont)
//			{
//				nRF24_send_2bytes();
//			}
//		}
//
//#else
////		nRF24_set_RX_mode(ENABLE);
//
//
//		// Transmitter;
////		nRF24_set_TX_mode(ENABLE);
//
//
//
////		SPI1_slave_demo01();
//#endif
//	}
//
//	return 0;
//
//
////	__DATE__
//
//	FLASH_Unlock();		// Unlock the Flash Program Erase controller
//	EE_Init();			// EEPROM Init
//
//
//
////	time_t     utcsec;
//	long utcsec;
////	struct tm  ts;
//	char       buf[80];
////	Get current time
////	time(&now);
//	utcsec = 1467296589;
////	Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
////	ts = *localtime(&now);
//	char *c_time_string;
//    c_time_string = ctime(&utcsec);
//    sprintf(buf, "Current time is %s", c_time_string);
////	sprintf(buf, "utcsec: %d", utcsec);
////    strcpy(buf, "Current time is ");
//	USART1_println(buf);
//	int i;
//	for(i=0;i<20;i++)
//	{
//		EE_WriteVariable(addr, name[i]);
//	}
//	/* --- Store successively many values of the three variables in the EEPROM ---*/
//	/* Store 1000 values of Variable1 in EEPROM */
//	for (VarValue = 0; VarValue < 1000; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[0], VarValue);
//	}
//
//	/* Store 500 values of Variable2 in EEPROM */
//	for (VarValue = 0; VarValue < 500; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[1], VarValue);
//	}
//
//	/* Store 800 values of Variable3 in EEPROM */
//	for (VarValue = 0; VarValue < 800; VarValue++)
//	{
//		EE_WriteVariable(VirtAddVarTab[2], VarValue);
//	}
// DEBOUNCE!!!!
//int button_is_pressed()
//{
//	/* the button is pressed when BUTTON_BIT is clear */
//	if (bit_is_clear(BUTTON_R_PIN, BUTTON_R_BIT))
//	{
//		_delay_ms(DEBOUNCE_TIME);
//		if (bit_is_clear(BUTTON_R_PIN, BUTTON_R_BIT))
//			return 1;
//	}
//	if (bit_is_clear(BUTTON_L_PIN, BUTTON_L_BIT))
//	{
//		_delay_ms(DEBOUNCE_TIME);
//		if (bit_is_clear(BUTTON_L_PIN, BUTTON_L_BIT))
//			return -1;
//	}
//	return 0;
//}
#pragma GCC diagnostic pop
//void comm_Bluetooth()
//{
//	// Rx - Always listening
////	uint8_t j2 =0;
//	while((Serial.available()>0))	// Reading from serial
//	{
////		Serial.sendByte('B');
//		inChar = Serial.readByte();
//
//		if(inChar=='$')
//		{
//			j2 = 0;
//			flag_frameStartBT = 1;
////			Serial.println("Frame Start!");
//		}
//
//		if(flag_frameStartBT)
//			sInstrBluetooth[j2] = inChar;
//
////		sprintf(buffer,"J= %d",j2);
////		Serial.println(buffer);
//
//		j2++;
//
//		if(j2>=sizeof(sInstrBluetooth))
//		{
//			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
//			j2=0;
////			Serial.println("ZEROU! sIntr BLuetooth Buffer!");
//		}
//
//		if(inChar==';')
//		{
////			Serial.println("Encontrou ; !");
//			if(flag_frameStartBT)
//			{
////				Serial.println("Frame Stop!");
//				flag_frameStartBT = 0;
//				rLength = j2;
//				j2 = 0;
//				enableTranslate_Bluetooth = 1;
//			}
//		}
//	}
////	flag_frameStart = 0;
//
//	if(enableTranslate_Bluetooth)
//	{
////		Serial.println("enableTranslate_Bluetooth");
//		enableTranslate_Bluetooth = 0;
//
//		char *pi0, *pf0;
//		pi0 = strchr(sInstrBluetooth,'$');
//		pf0 = strchr(sInstrBluetooth,';');
//
//		if(pi0!=NULL)
//		{
//			uint8_t l0=0;
//			l0 = pf0 - pi0;
//
//			int i;
//			for(i=1;i<=l0;i++)
//			{
//				sInstr[i-1] = pi0[i];
////				Serial.sendByte(sInstr[i-1]);
////				Serial.write(sInstr[i-1]);
//			}
//			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
//	//		Serial.println(sInstr);
////			USART1_println(sInstr);
//
//			enableDecode = 1;
//		}
//		else
//		{
////			Serial.println("Err");
////			USART1_putc(pi0[0]);
////			USART1_putc(pf0[0]);
//		}
//	}
//}
//void handleMessage()
//{
//	if(enableDecode)
//	{
//		enableDecode = 0;
//
//		// Getting the opcode
//		aux[0] = sInstr[0];
//		aux[1] = sInstr[1];
//		aux[2] = '\0';
//		opcode = (uint8_t) atoi(aux);
//
////		uint8_t reg_value, reg_addr, rxc;
//
//		switch (opcode)
//		{
//			case 0:		// Set motor ON/OFF
//			{
//				if(sInstr[2] == ';')
//				{
//					LED_green_toogle();
//					Serial.println("Fuck!");
//				}
//			}
//		}
//		memset(sInstr,0,sizeof(sInstr));	// Clear all vector;
//	}
//}
//void handleMessage()
//{
//	if(enableDecode)
//	{
//		enableDecode = 0;
//
//		// Getting the opcode
//		aux[0] = sInstr[0];
//		aux[1] = sInstr[1];
//		aux[2] = '\0';
//		opcode = (uint8_t) atoi(aux);
//
//		uint8_t reg_value, reg_addr, rxc;
//
//		if(sInstr[2] == ';')
//		{
//			aux[0] = sInstr[0];
//			aux[1] = sInstr[1];
//			aux[2] = '\0';
//			reg_addr  = (uint8_t) atoi(aux);
//
//			sprintf(buffer,"%d",reg_addr);
//			Serial.print(buffer);
//		}
//
//
////		if(sInstr[2] == ':' && sInstr[6] == ';')
////		{
////			aux[0] = sInstr[0];
////			aux[1] = sInstr[1];
////			aux[2] = '\0';
////			reg_addr  = (uint8_t) atoi(aux);
////
////
////			aux2[0] = sInstr[3];
////			aux2[1] = sInstr[4];
////			aux2[2] = sInstr[5];
////			aux2[3] = '\0';
////			reg_value  = (uint8_t) atoi(aux2);
////
////			rxc = nRF24_write_register(reg_addr, reg_value);
////			sprintf(buffer, "w:%d", rxc);
////			USART1_println(buffer);
////		}
////		else if(sInstr[0] == 'r' && sInstr[1] == ';')
////		{
////			nRF24_get_RX_ADDRn(0, nRF24_rx_addr_p0, 5);
////		}
////		else if(sInstr[0] == 'w' && sInstr[1] == ';')
////		{
////			nRF24_set_RX_ADDRn(0, nRF24_rx_addr_p0_d, 5);
////		}
////		else if(sInstr[0] == 'c' && sInstr[1] == ':' && sInstr[3] == ';')
////		{
////			if(sInstr[2] == '1')
////			{
////				nRF24_test();
////				USART1_println("test");
////			}
////			else
////			{
////				nRF24_reset();
////				USART1_println("RST");
////			}
////		}
////		else if(sInstr[0] == 't' && sInstr[1] == ';')
////		{
////			nRF24_send_2bytes();
////		}
////		else if(sInstr[0] == 't' && sInstr[1] == ':' && sInstr[2] == '1' && sInstr[3] == ';')
////		{
////			nRF24_flag_send_cont = 1;
////		}
////		else if(sInstr[0] == 't' && sInstr[1] == ':' && sInstr[2] == '0' && sInstr[3] == ';')
////		{
////			nRF24_flag_send_cont = 0;
////		}
////		else if(sInstr[0] == 'p' && sInstr[1] == 'u' && sInstr[2] == ';')
////		{
////			nRF24_set_PowerUp();
////		}
////		else if(sInstr[0] == 'p' && sInstr[1] == 'd' && sInstr[2] == ';')
////		{
////			USART1_println("PowerDown");
////			nRF24_set_PowerDown();
////		}
////		else if(sInstr[0] == 'f' && sInstr[1] == 't' && sInstr[2] == ';')
////		{
////			nRF24_flush_TX();
////		}
////		else if(sInstr[0] == 'f' && sInstr[1] == 'r' && sInstr[2] == ';')
////		{
////			nRF24_flush_RX();
////		}
////		else if(sInstr[0] == 'r' && sInstr[1] == 'x' && sInstr[2] == ':' && sInstr[4] == ';')
////		{
////			if(sInstr[3] == '1')
////			{
////				nRF24_set_RX_mode(ENABLE);
////			}
////			else
////			{
////				nRF24_set_RX_mode(DISABLE);
////			}
////		}
////		else if(sInstr[2] == ';')
////		{
////			aux[0] = sInstr[0];
////			aux[1] = sInstr[1];
////			aux[2] = '\0';
////
////			reg_addr  = (uint8_t) atoi(aux);
////			rxc = nRF24_read_register(reg_addr);
////			sprintf(buffer, "r:%d", rxc);
////			USART1_println(buffer);
////		}
//
//		switch (opcode)
//		{
////			case 0:
////			{
////				if(sInstr[1] == ';')
////				{
////					rxc = nRF24_read_register(reg_addr);
////					sprintf(buffer, "r:%d", rxc);
////					USART1_println(buffer);
////				}
////			}
////			break;
////			case 1:
////			{
////				if(sInstr[1] == ':' && sInstr[4] == ';')
////				{
////					aux[0] = sInstr[2];
////					aux[1] = sInstr[3];
////					aux[2] = '\0';
////
////					byte_value  = (uint8_t) atoi(aux);
////					rxc = nRF24_write_register(0x00, 0x00);
////					sprintf(buffer, "w1:%d", rxc);
////					USART1_println(buffer);
////				}
////			}
////			break;
////			case 2:
////			{
////				if(sInstr[1] == ':' && sInstr[4] == ';')
////				{
////					aux[0] = sInstr[2];
////					aux[1] = sInstr[3];
////					aux[2] = '\0';
////
////					byte_value  = (uint8_t) atoi(aux);
////					rxc = nRF24_write_register(0x00, byte_value);
////					sprintf(buffer, "w2:%d", rxc);
////					USART1_println(buffer);
////				}
////			}
////			break;
////			default:
////			{
////				USART1_println("fuckdefault!");
////			}
////			break;
//
//// -----------------------------------------------------------------
////			case 0:		// Set motor ON/OFF
////			{
////				if
//
////			}
//			case 30:		// Set motor ON/OFF
//			{
//				LED_green_toogle();
////				uint8_t motorCommand;
////				aux[0] = '0';
////				aux[1] = sInstr[1];
////				aux[2] = '\0';
////				motorCommand = (uint8_t) atoi(aux);
////
////				if(motorCommand)
////				{
////					LED_green(1);
////					USART1_println("ON");
////				}
////				else
////				{
////					LED_green(0);
////					USART1_println("OFF");
////				}
//
////				if (motorCommand && (!motorStatus))
////					motor_start();
////				else
////					motor_stop();
////
////				summary_Print(3);
//			}
//			break;
//
////			case 4: // EEPROM Write -> $4:h:1234;
////			{
////				if(sInstr[1]==':' && sInstr[3]==':' && sInstr[8]==';')
////				{
////					char aux2[5];
////					aux2[0] = sInstr[4];
////					aux2[1] = sInstr[5];
////					aux2[2] = sInstr[6];
////					aux2[3] = sInstr[7];
////					aux2[4] = '\0';
////					uint16_t valueReceived = (uint16_t) atoi(aux2);
////
////					aux[0] = '0';
////					aux[1] = sInstr[2];
////					aux[2] = '\0';
////
////					uint8_t ind = (uint8_t) atoi(aux);
////
////					EE_WriteVariable(VirtAddVarTab[ind], valueReceived);
////
////					/* Store 800 values of Variable3 in EEPROM */
//////					for (VarValue = 0; VarValue < 800; VarValue++)
//////					{
//////						EE_WriteVariable(VirtAddVarTab[2], VarValue);
//////					}
////
////					uint16_t valueStored;
////					EE_ReadVariable(VirtAddVarTab[ind], &valueStored);
////					sprintf(buffer,"%d", valueStored);
////					USART1_println(buffer);
////				}
////				else if(sInstr[1]==';')
////				{
////					uint16_t valueRead;
////					int i;
////					for(i=0; i<3; i++)
////					{
////						EE_ReadVariable(VirtAddVarTab[i], &valueRead);
////						sprintf(buffer,"%d: %d", VirtAddVarTab[i], valueRead);
////						USART1_println(buffer);
////					}
////
////					char name2[21];
////					uint16_t v;
////					for(i=0; i<20; i++)
////					{
////						EE_ReadVariable(addr, &v);
////						name2[i] = (char) v;
////					}
////					USART1_println(name2);
////					USART1_println("Fuck");
////				}
////
////				break;
////			}
//// -----------------------------------------------------------------
//		}
//		memset(sInstr,0,sizeof(sInstr));	// Clear all vector;
//	}
//}
//void comm_SPI1()
//{
//	char inChar2;
//	while((SPI1_available()>0))	// Reading from serial
//	{
//		USART1_println("i: ");
//		inChar2 = SPI1_readByte();
//		USART1_putc(inChar2);
//	}
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//	{
//		USART1_println("B");
//		inChar2 = SPI1_readByte();
//		USART1_putc(inChar2);
//	}
//
//}
//void comm_SPI1_polling()
//{
//	unsigned char rxc, txc = 0x57;
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//	{
//		rxc = SPI1 -> DR;
//		USART1_putc(rxc);
//	}
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE)  == SET)
//	{
//		SPI1 -> DR = txc;
//	}
//}
//void refresh_variables()
//{
////	nRF24_int();
//
//	if(SPI1_flag_err)
//	{
//		SPI1_flag_err = 0;
//		USART1_println("Err");
//		while(1);
//	}
//}
//int main(int argc, char* argv[])

// ----------------------------------------------------------------------------
