// For the UNO!

#define PERIOD 250   // 250 cycles = 15.625us for each half of waveform, 32kHz
#define HALF 125     // half the period is the default PWM threshold - gives square wave.
#define MAXAMP 31
 
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

void setup ()
{
  setup_cosines () ;
  setup_timers () ;

  pinMode (9, OUTPUT) ; // OC1A pin = U drive
  pinMode (10, OUTPUT) ; // OC1B pin = V drive
  pinMode (3, OUTPUT) ; // OC2B pin = W drive
}

volatile byte u = HALF ;
volatile byte v = HALF ;
volatile byte w = HALF ;


ISR (TIMER1_OVF_vect)   // overflow triggers at BOTTOM, update all the compare regs, which are sampled at TOP
{                       // 50us later, then take effect about 75us later on average.
  //TCNT2 = 0 ;
  OCR1A = u ;
  OCR1B = v ;
  OCR2B = w ;
}

#define  DEG_360  0x200
#define  DEG_120  0x0AB

char cosine_tab [DEG_360+1] ;  // fails to work if less than 0x202.

void setup_cosines ()
{
  for (int i = 0 ; i < 0x202 ; i++)
  {
    float a = i * 6.283185 / 0x200 ;
    if (i <= DEG_360)
      cosine_tab [i] = round (127.0 * cos (a)) ;
  }
}

unsigned int phase = 0 ;
int freq  = 1 ;
int amplitude = MAXAMP ; // +/-201 maximum

unsigned long prev = 0L ;

int my_cosine (int ph)
{
  ph &= DEG_360-1 ;
  int res = cosine_tab [ph] ;
  res *= amplitude ;
  return (res + 0x0F) >> 5 ;
}

#define fstep_del 500

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
  delayMicroseconds (1000) ;
 
  if (millis () - prev > fstep_del)
  {
    prev += fstep_del ;
    freq = - freq ;
    freq = 1 - freq ;
    if (freq > 20)
      freq = 0 ;
   // freq = 8 ;
  }
 
}
