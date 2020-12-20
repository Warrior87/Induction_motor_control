// For the UNO!
//can set frequency by freq=8; inside loop and disabling the if(millis) thing
//PWM frequency needs to be lowered below 20kHz as per the PSS30S92E6-AG datasheet

//#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print (x)
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define PERIOD 250   // 250 cycles = 15.625us for each half of waveform, 32kHz
#define HALF 125     // half the period is the default PWM threshold - gives square wave.
#define MAXAMP 31
#define DEG_360  0x200
#define DEG_120  0x0AB
#define fstep_del 500
 
volatile byte u = HALF ;
volatile byte v = HALF ;
volatile byte w = HALF ;
char cosine_tab [DEG_360+1] ;  // fails to work if less than 0x202.unsigned int phase = 0 ;
unsigned int phase = 0 ;
int freq  = 1 ;
int amplitude = MAXAMP ; // +/-201 maximum
unsigned long prev = 0L ;

const byte throttlePin = 7;
int zeroThrottle = 310;                                         /*throttle reading when pedal is at 0*/
int fullThrottle = 410;                                         /*throttle reading when pedal is at 100%, originally 720*/
double slope = 0.622;                                           /*value converts ADC values to PWM values, scaling variable*/
int throttlePosition;

void setup ()
{
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  
  setup_cosines () ;
  setup_timers () ;

  pinMode (9, OUTPUT) ; // OC1A pin = U drive
  pinMode (10, OUTPUT) ; // OC1B pin = V drive
  pinMode (3, OUTPUT) ; // OC2B pin = W drive
}

void loop ()
{
  phase += freq ;
  int newu = my_cosine (phase) ;
  int newv = my_cosine (phase - DEG_120) ;
  int neww = - newu - newv ;
  newu += HALF ;
  newv += HALF ;
  neww += HALF ;
  //noInterrupts () ;
  u = newu ;
  v = newv ;
  w = neww ;
  //interrupts () ;
  throttlePosition = getThrottle();
  freq = getMotorDrive(throttlePosition);
  delayMicroseconds (1000) ;
 
//  if (millis () - prev > fstep_del)
//  {
//    prev += fstep_del ;
//    freq = - freq ;
//    freq = 1 - freq ;
//    if (freq > 100)
//   //   freq = 0 ;
//    freq = 8 ;
//  }
 
}

int getThrottle (){
  int throttlePos;
  throttlePos = analogRead(throttlePin);  
  DEBUG_PRINT(throttlePos);
  throttlePos = throttlePos - zeroThrottle;           /*offset throttle position to start at 0, at 0 throttle*/
  if(throttlePos < 0){                                     /*correct throttle position if it is negative*/
    throttlePos = 0;
  }
  DEBUG_PRINT("\t");
  DEBUG_PRINT(throttlePos);
  return throttlePos;
}

int getMotorDrive(int throttlePos){
  int motorDrive;
  motorDrive = (double)throttlePos * slope;                /*multiply throttle position by slope to get PWM value*/
  DEBUG_PRINT("\t");
  DEBUG_PRINT(motorDrive);
  if(motorDrive > 255){                                         /*correct motorDrive if it is over 255*/
    motorDrive = 255;
  }
  if(motorDrive < 0){                                           /*correct motorDrive if it is negative*/
    motorDrive = 0;
  }
  //scale motorDrive between 0 and 100
  motorDrive = motorDrive * 100;
  motorDrive = motorDrive / 255;  
  return motorDrive;
}

void setup_cosines ()
{
  for (int i = 0 ; i < 0x202 ; i++)
  {
    float a = i * 6.283185 / 0x200 ;
    if (i <= DEG_360)
      cosine_tab [i] = round (127.0 * cos (a)) ;
  }
}

int my_cosine (int ph)
{
  ph &= DEG_360-1 ;
  int res = cosine_tab [ph] ;
  res *= amplitude ;
  return (res + 0x0F) >> 5 ;
}

void setup_timers ()
{
  TCCR1A = 0xF2 ;  // phase correct (mode 1010, ICR1 controls period)
  TCCR1B = 0x11 ;  // prescale by 4
  TIMSK1 = 0x01 ;  // overflow interrupt
  TCCR2A = 0x31 ;  // phase correct (mode 101, OCR2A controls period)
  TCCR2B = 0x09 ;
 
  ICR1  = PERIOD ;    // 31.25us cycle time, 32kHz PWM but 64kHz drive pulses (differential)
  OCR2A = PERIOD ;
 
  OCR1A = HALF-100 ;    // example U drive
  OCR1B = HALF ;
  OCR2B = HALF+100 ;    // example W drive
 
  GTCCR = 0x83 ; // clear and halt prescalers
  TCNT1 = 0xFFFF ;  // synchronize counters
  TCNT2 = 0 ;
  GTCCR = 0x00 ; // allow prescalers to fly
}

ISR (TIMER1_OVF_vect)   // overflow triggers at BOTTOM, update all the compare regs, which are sampled at TOP
{                       // 50us later, then take effect about 75us later on average.
  //TCNT2 = 0 ;
  OCR1A = u ;
  OCR1B = v ;
  OCR2B = w ;
}
