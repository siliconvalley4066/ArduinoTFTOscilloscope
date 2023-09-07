// Kyutech Arduino Scope Prototype  v0.73                     Apr 10, 2019
//    (C) 2012-2019 M.Kurata Kyushu Institute of Technology
//    for Arduinos with a 5V-16MHz ATmega328.
//
//    Pin usage
//    
//    A0  TFT shield
//    A1  TFT shield
//    A2  TFT shield
//    A3  TFT shield
//    A4  TFT shield
//    A5  reserved for ch1 input of UNO
//    A6  oscilloscope probe ch1
//    A7  oscilloscope probe ch2
//    
//    D0  uart-rx
//    D1  uart-tx
//    D2  TFT shield
//    D3  TFT shield
//    D4  TFT shield
//    D5  TFT shield
//    D6  TFT shield
//    D7  TFT shield
//    D8  TFT shield
//    D9  TFT shield
//    D10 calibration pulse output
//    D11 ch1 DC/AC SW input
//    D12 ch2 DC/AC SW input
//    D13 LED output

byte  oscspeed   = 0;      // 0..6:equiv
byte  oscinput   = 0;      // input signal selection  0:CH1 1:CH2 2:DUAL
word  osctdly    = 200;    // time of delayed trigger  100..30000 usec
byte at;

void modeequiv() {
  static const struct eqdic_s {
    word   tkn;
    byte   tdif;
    word   wu;
  } eqdic[] = {
    {300,  1, 32000},   // 16Msample/s  , 1.875us/div
    {150,  2, 32000},   // 8Msample/s   , 3.75us/div
    { 75,  5, 32000},   // 3.2Msample/s , 9.375us/div
    { 50, 10, 32000},   // 1.6Msample/s , 18.75us/div
    { 20, 20, 32000},   // 800ksample/s , 37.5us/div
    { 10, 50, 64000},   // 320ksample/s , 93.75us/div
    {  5,100, 64000},   // 160ksample/s , 187.5us/div
  };
  const struct eqdic_s   *eq;
  int    realnum, i, dp;
  byte   tokadif;
  word   toka, tokanum;
  byte   ch, chnum, adch, adchT;
  word   ui1, waituntil, sinterval;

  int *buf0 = (int *) data[sample];
  eq = &eqdic[oscspeed];
  tokanum   = eq->tkn;
  waituntil = eq->wu;
  realnum   = SAMPLES / tokanum;
  tokadif   = eq->tdif;
  sinterval = tokanum * tokadif;  // 20us typical

  // ADMUX reg values
  switch(oscinput) {
  default:
  case 0x00: adch = 0x40 + ad_ch0; chnum = 1;  break;  // CH1
  case 0x01: adch = 0x40 + ad_ch1; chnum = 1;  break;  // CH2
  case 0x02: adch = 0x40 + ad_ch0; chnum = 2;          // CH1 Ch2 Dual
    break;
  }
  adchT = 0x40 + trig_ch;

  sinterval--;
  at = 0;
  for(toka = 0; toka < tokanum; toka++) {
    dp = toka;
    for(ch = 0; ch < chnum; ch++) {     // for all ch (1 or 2)
      // reset and initialize timer1
      TCCR1B = 0x00; // stop, set normal mode
      TCCR1A = 0x00;
      TIMSK1 = 0x00; // no irq
      ICR1   = 0x0000;
      TCNT1  = 0x0000;
      TIFR1  = 0x27; // clear flags;

      // analog comparator setting
      // The BG is the positive input.
      // The negative input is A6 or A7 pin.
      ACSR   = 0x54;  // capture-on, aco to caputure timer1
      ADMUX  = adchT; // select the negative input
      ADCSRA = 0x04;  // divide by 16
      ADCSRB = 0x40;  // AC multiplexer enable, ADC auto trigger source free run
//      DIDR1  = 0x03;  // disable the digital input func of D6 and D7.
      // start timer1 with pre=1/1 (16MHz)
      // input capture noise canceler ON
      TCCR1B = (trig_edge == TRIG_E_DN) ? 0xc1 : 0x81;  // edge selection
      TIFR1  = 0x27; // clear flags again

      ui1 = (tokadif * toka) + (osctdly << 4);  // delay time

      // falling edge detection(rising edge for ICES1)
      // doesn't stabilize without a 125usec wait below.
      while(TCNT1 < 2000)
          ;
      TIFR1 = 0x27;
      // wait until a trigger event happens
      while(true) {
        if (TIFR1 & 0x20) {
          // trigger event has happened.
          ui1 += ICR1;
          at = 0; // a trigger event has happened.
          break;
        }
        if (TCNT1 > waituntil) {
          ui1 += TCNT1;
          at = 1; // trigger failed.
          break;
        }
      }

      // at:0 -> trigger event has happened, 1 -> not happened
      ACSR   = 0xd4; // disable analog comparator
      ADCSRB = 0x00; // AC multiplexer disable, ADC auto trigger source free run
      ADCSRA = 0x84; // adc enable, 1MHz, adate off

      TCCR1B = 0x19; // timer1 CTC-ICR1 mode pre1/1
      TCCR1A = 0x00; //             CTC mode;
      ICR1   = ui1;
      TIFR1  = 0x27; // clear flags

      ADMUX  = adch; // adc target is A6 pin to get trigger value;
      ADCSRB = 0x07; // timer1 capture event;
      ADCSRA = 0xf4; // adc auto trigger, force 1st conversion

      // wait until the 1st conversion finishes. 
      while((ADCSRA & 0x10) == 0x00)
        ;

      ADMUX = adch + ch;
      ADCSRA = 0xb4;   // clear flag, 1MHz, adate on

      if (toka == 0 && ch == 0) {   // needed only for the 1st loop
        if (at) {
          for(i = 0; i < SAMPLES; i++)
            buf0[i] = analogRead(ad_ch0);
          scaleDataArray();
          return; // when trigger failed
        }
      }

      for(i = 0; i < realnum; i++) {  // 15 samples at 20us
        while(true) {
          if (TIFR1 & 0x20) {
            ICR1 = sinterval;
            TIFR1 = 0x27; // clear timer1 flags;
          }
          if ((ADCSRA & 0x10) != 0x00)
            break;
        }
        byte *pdata = (byte *) &buf0[dp];
        *pdata++ = ADCL;
        *pdata = ADCH;
        dp += tokanum;
        ADCSRA = 0xb4;   // clear flag, 1MHz, adate on
      }
    }
  }
  scaleDataArray();
}
