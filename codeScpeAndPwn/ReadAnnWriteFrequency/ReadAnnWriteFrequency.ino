#include <TimerOne.h>
const int fanPin = 4;
const int analogInPin1 = A0;  // Analog input pin that the potentiometer is attached to
static int ctr,flag_tog;
static unsigned char adcval;

void setup(void)
{
  
  Timer1.initialize(40); 
  Serial.begin(115200);
  
  pinMode(13, OUTPUT);

  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX |= (0 & 0x07);    // set A0 analog input pin
  ADMUX |= (1 << REFS0);  // set reference voltage
  ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
  //ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements
}

ISR(ADC_vect)
{
  adcval = ADCH;  // read 8 bit value from ADC
}
  
void loop(void)
{
  Serial.write(adcval);
  // slowly increase the PWM fan speed
  //
  for (float dutyCycle = 70.0; dutyCycle < 100.0; dutyCycle++) {
    Serial.print("PWM Fan, Duty Cycle = ");
    Serial.println(dutyCycle);
    Timer1.pwm(fanPin, (dutyCycle / 100) * 1023);
    delay(500);
  }


  
}
