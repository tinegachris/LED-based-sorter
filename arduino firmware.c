/* LED-Based Sorter
 *
 * Created:   Sat Dec 25 2021
 * Author:    Chrispine Tinega
 * Processor: ATmega328P
 * Compiler:  Arduino AVR
 * 
 */

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define USART_BAUDRATE 115200
// #define F_CPU  16000000
// #define BAUD_PRESCALE  (F_CPU / USART_BAUDRATE / 16)) - 1
//#define BAUD_PRESCALE 17 //use for 57600 baud
//#define BAUD_PRESCALE 103  // use for 9600 baud
#define BAUD_PRESCALE 8  // use for 115200 baud

// SORTING PARAMETERS
long int disc_const;
long int disc1_param;
long int disc2_param;
long int disc3_param;

short int max_pix = 6;
short int F1 = 4;
short int F2 = 8;
short int F3 = 17;
short int x, y, z, counter; 
short int n11 =1;
short int n12=1;
short int n21=1;
short int  n22=1;
short int  n31=1;
short int  n32=1;
short int  d1=1;
short int  d2=1;
short int  d3=1;
int Dsigns = 10;
short int checksum;

short int sort = 1;  // make 0 for storing data, 1 for sorting

#define trig_thresh 20

#define delay 40

int reject_thresh;
/*
the classification goes as:

F1, F2, and F3 are the feature numbers saved in the csv data file when iamge data is collected.

The FPGA performs the following computation on these numbers:

P = const + disc1_param*F1 + disc2_param*F2 + disc3_param*F3

if P less than than threshold then reject (default), by default, the threshold is 2016 but can be adjusted up and down by the knob on the readout board. (number here does not match readout number but are correlated)

Specify signs for disc_const, disc1_param, disc2_param, disc3_param using the signs parameter: use 1 for -, 0 for +:
Signs is a 4 bit number:
LSB, bit 0, is for disc_const
bit 1, is for disc1_param
bit 2, is for disc2_param
MSB, bit 3 is for disc3_param
*/

void setup() 
{
  char ReceivedByte, lowByte, highByte;
  short int i, trigger, rej_direction;
  short data[17];
  long int DC, D1, D2, D3, disc_data1, disc_data2, disc_data3;
  short res;
  unsigned int  dark, blue, green, red, IR850, IR880, IR910, IR940, IR970, IR1070;
  unsigned int max_red, max_IR850, max_IR940, sat_count;
  
  ////////// set up discriminant function
  disc1_param = eeprom_read_word((uint8_t*)10);
  disc2_param = eeprom_read_word((uint8_t*)12);
  disc3_param = eeprom_read_word((uint8_t*)14);
  disc_const = eeprom_read_word((uint8_t*)16);
  F1 = eeprom_read_byte((uint8_t*)18);
  F2 = eeprom_read_byte((uint8_t*)19);
  F3 = eeprom_read_byte((uint8_t*)20);
  Dsigns = eeprom_read_byte((uint8_t*)21);
  max_pix = eeprom_read_byte((uint8_t*)22);
  reject_thresh = eeprom_read_byte((uint8_t*)23);

  /// get bits from signs
  res = Dsigns &(1);
  DC = disc_const;
  if(res>0) DC = disc_const*(-1);

  res = Dsigns & (1<<1);
  D1 = disc1_param;
  if(res>0) D1 = disc1_param*(-1);

  res = Dsigns & (1<<2);
  D2 = disc2_param;
  if(res>0) D2 = disc2_param*(-1);

  res = Dsigns & (1<<3);
  D3 = disc3_param;
  if(res>0) D3 = disc3_param*(-1);

  // assign numerator and denominator values given feature number
  if(F1<=11) {n11=F1; n12=0; d1=1024;}
  if(F2<=11) {n21=F2; n22=0; d2=1024;}
  if(F3<=11) {n31=F3; n32=0; d3=1024;}

  
  // differences 
  counter= 12;
  for(x=1; x<=10; x++)
  {
  for(y=x+1; y<=11; y++)
    {
    if(F1 == counter) {n11=x; n12=y; d1=1;}
    if(F2 == counter) {n21=x; n22=y; d2=1;}
    if(F3 == counter) {n31=x; n32=y; d3=1;}
    counter++;
    }
  }

  // ratios
  for(x=1; x<=10; x++)
  {
  for(y=x+1; y<=11; y++)
    {   
    if(F1 == counter) {n11=x; n12=0; d1=y;}
    if(F2 == counter) {n21=x; n22=0; d2=y;}
    if(F3 == counter) {n31=x; n32=0; d3=y;}
    counter++;
    }
  }

  // second derivitives
  for(x=1; x<=7; x++)
  {
  for(y=x+1; y<=8; y++)
    {
    for(z=y+1; z<=9; z++)
      {
      if(F1 == counter) {n11=x; n12=y; d1=z;}
      if(F2 == counter) {n21=x; n22=y; d2=z;}
      if(F3 == counter) {n31=x; n32=y; d3=z;}
      counter++;
      }
    }
  }

  //////////// SET PINS TO OUTPUT
  DDRB =  0b11111011;  // declare all B pins as outputs except for PB2
  DDRD = 255;  // declare all D pins as outputs
  DDRC = 255;  // declare all D pins as outputs
  
  ///////////// ADC INITIALIZATION
  //AREF = AVcc
  //ADMUX = (1<<REFS0);
  ADMUX = 0b01100110;  //use Avcc as reference, put MSB 8 bit in high byte (ADCH), use AD6 as the input the 4 LSB bits defie ADC channel
  //ADMUX = 0b01100000;  //use Avcc as reference, put MSB 8 bit in high byte (ADCH), use AD0 as the input
    
    
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);  //ADC Enable and ADPS prescaler of 64 // 16000000/64 = 250000


  ////////////////// SERIAL PORT INITIALIZATION
  /* Enable receiver and transmitter   */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0); 
  /* Frame format: 8data, No parity, 1stop bit */ 
  UCSR0C = (3<<UCSZ00);
  // Set baud rate
  UBRR0H = (BAUD_PRESCALE>>8);
  UBRR0L = BAUD_PRESCALE;
}

void loop() 
{
  /////////// SERIAL PORT PART
  
  // while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
  // ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived"

  dark = 0;
  blue = 0;
  green = 0;
  red = 0;
  IR850 = 0;
  IR880 = 0;
  IR910 = 0;
  IR940 = 0;
  IR970 = 0;
  IR1070 = 0;
  max_red = 0;
  max_IR850 = 0;
  max_IR940 = 0;
  sat_count = 0;

  ////////// Now monitor RED and IR LED's response for a seed to enter field of view
  trigger = 0;
  while(trigger < trig_thresh)
  {
    // get reject threshold

    // if ((UCSR0A & (1 << RXC0)) != 0) ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived"

    // _delay_us(delay);  // keep LED off so it will not burn out
    // if ((UCSR0A & (1 << RXC0)) != 0) ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived"
    
    // _delay_us(delay);  // keep LED off so it will not burn out

    if ((UCSR0A & (1 << RXC0)) != 0) 
    {
      ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived"

      if(ReceivedByte <=221) 
      {
        reject_thresh = ReceivedByte;
        eeprom_write_byte ((uint8_t*) 23, ReceivedByte);
      }

      if(ReceivedByte == 223) 
      {
        sort = 1;  // sort mode
        //  reject_thresh = 51;
        ReceivedByte = 0;
      }

      if(ReceivedByte == 224) 
      {
        sort = 0;  // data mode
        // reject_thresh = 52;
      }


      // load sorting parameters if ReceivedByte = 222

      if(ReceivedByte == 222)  // load calibration
      {
        checksum = 0;
        // send byte aknowledging load calibration mode
        UDR0 = ReceivedByte+1;
        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it
        _delay_ms(50);  //it is happier with this delay for some reason
  
        ReceivedByte = 50;  // reset ReceivedByte so it won't keep doing this,  either reset to zero or 50 so it will go back to default threshold level
        reject_thresh = 50;


        // get disc1_param
        // wait for next Byte
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
        // wait for next Byte
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          highByte = UDR0; // Fetch the recieved byte value into the variable 

        eeprom_write_byte ((uint8_t*) 10, lowByte);
        eeprom_write_byte ((uint8_t*) 11, highByte);


        // get disc2_param
        // wait for next Byte
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
        // wait for next Byte
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          highByte = UDR0; // Fetch the recieved byte value into the variable 
  
        eeprom_write_byte ((uint8_t*) 12, lowByte);
        eeprom_write_byte ((uint8_t*) 13, highByte);


        // get disc3_param
        // wait for next Byte
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
        // wait for next Byte
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          highByte = UDR0; // Fetch the recieved byte value into the variable 
  
        eeprom_write_byte ((uint8_t*) 14, lowByte);
        eeprom_write_byte ((uint8_t*) 15, highByte);


        // get disc_const
        // wait for next Byte
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
        // wait for next Byte
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          highByte = UDR0; // Fetch the recieved byte value into the variable 
  
        eeprom_write_byte ((uint8_t*) 16, lowByte);
        eeprom_write_byte ((uint8_t*) 17, highByte);


        ///////////// the next five are single bytes
  
        // get F1
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
  
        eeprom_write_byte ((uint8_t*) 18, lowByte);

        // get F2
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
  
        eeprom_write_byte ((uint8_t*) 19, lowByte);
  
        // get F3
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
  
        eeprom_write_byte ((uint8_t*) 20, lowByte);

        // get signs
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
  
        eeprom_write_byte ((uint8_t*) 21, lowByte);
  
  
        // get max_pix
        while ((UCSR0A & (1 << RXC0)) == 0) {}; // Do nothing until data have been recieved and is ready to be read from UDR
          lowByte = UDR0; // Fetch the recieved byte value into the variable 
  
  
        eeprom_write_byte ((uint8_t*) 22, lowByte);

        disc1_param = eeprom_read_word((uint8_t*)10);
        disc2_param = eeprom_read_word((uint8_t*)12);
        disc3_param = eeprom_read_word((uint8_t*)14);
        disc_const = eeprom_read_word((uint8_t*)16);
        F1 = eeprom_read_byte((uint8_t*)18);
        F2 = eeprom_read_byte((uint8_t*)19);
        F3 = eeprom_read_byte((uint8_t*)20);
        Dsigns = eeprom_read_byte((uint8_t*)21);
        max_pix = eeprom_read_byte((uint8_t*)22);

        checksum = (disc1_param & 15) + (disc2_param & 15) + (disc3_param & 15) + (disc_const & 15) + (F1 & 15) + (F2 & 15) + (F3 & 15) + (Dsigns & 15);

        // DONE
        // send byte aknowledging load calibration mode
        UDR0 = checksum;
        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

        // fix up discriminat function with proper signs
    
        res = Dsigns &(1);
        DC = disc_const;
        if(res>0) DC = disc_const*(-1);
  
        res = Dsigns & (1<<1);
        D1 = disc1_param;
        if(res>0) D1 = disc1_param*(-1);
  
        res = Dsigns & (1<<2);
        D2 = disc2_param;
        if(res>0) D2 = disc2_param*(-1);
  
        res = Dsigns & (1<<3);
        D3 = disc3_param;
        if(res>0) D3 = disc3_param*(-1);

        // assign numerator and denominator values given feature number
        if(F1<=11) {n11=F1; n12=0; d1=1024;}
        if(F2<=11) {n21=F2; n22=0; d2=1024;}
        if(F3<=11) {n31=F3; n32=0; d3=1024;}


        // differences 
        counter= 12;
        for(x=1; x<=10; x++)
        {
          for(y=x+1; y<=11; y++)
          {   
            if(F1 == counter) {n11=x; n12=y; d1=1;}
            if(F2 == counter) {n21=x; n22=y; d2=1;}
            if(F3 == counter) {n31=x; n32=y; d3=1;}
            counter++;
          }
        }

        // ratios
        for(x=1; x<=10; x++)
        {
          for(y=x+1; y<=11; y++)
          {   
            if(F1 == counter) {n11=x; n12=0; d1=y;}
            if(F2 == counter) {n21=x; n22=0; d2=y;}
            if(F3 == counter) {n31=x; n32=0; d3=y;}
            counter++;
          }
        }

      // second derivitives
        for(x=1; x<=7; x++)
        {
          for(y=x+1; y<=8; y++)
          {
            for(z=y+1; z<=9; z++)
            {
              if(F1 == counter) {n11=x; n12=y; d1=z;}
              if(F2 == counter) {n21=x; n22=y; d2=z;}
              if(F3 == counter) {n31=x; n32=y; d3=z;}
              counter++;
            }
          }
        }
      }  
    }

    rej_direction = 0;
    // check pin 2 for reject direction
    if(PINB&(1<<2)) rej_direction = 1;  //reverse reject direction
    // if(~PINB&(1<<2)) rej_direction = 0;  //normal reject direction


    // GET RED and dark LED READING
    PORTD |= 0b1000000;  //set red LED HIGH port PD6 high  
    _delay_us(delay);


    /////////////////////////// ADC READ

    // start single convertion, write '1' to ADSC
    ADCSRA |= (1<<ADSC);

    // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
    while(ADCSRA & (1<<ADSC));
    trigger = ADCH;

  
    PORTD &= 0;  //set RED LED to LOW

    //GET DARK READING AND SUBRTRACT FROM RED
    _delay_us(delay);
    // start single convertion, write '1' to ADSC
     ADCSRA |= (1<<ADSC);
    // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
    while(ADCSRA & (1<<ADSC));
    dark = ADCH;
    if(trigger>=dark) trigger = trigger-dark;
    else trigger = 0;


  } //trigger loop

  dark = 0;


  ///////// NOW GET  READINGs, ONE WITH NO led's first, THEN ONE EACH FOR THE BLUE, GREEN, RED, 880nm, 940nm, and 1070nm IN THAT ORDER
  for(i=0; i<=max_pix; i++)
  {
    //////// GET DARK READING
    // KEEP PORTS LOW SO NO LED FLASHES
    
    _delay_us(delay);
    _delay_us(delay);
    ///ADC READ

      // start single convertion, write '1' to ADSC
      ADCSRA |= (1<<ADSC);
  
      // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
      while(ADCSRA & (1<<ADSC));
    
    if(ADCH>0) dark = dark+ADCH;


    //////////// GET BLUE LED READING
    
    PORTB |= 0b1; // set port PB0 high
    
    _delay_us(delay);
    ///ADC READ
    
      // start single convertion, write '1' to ADSC
       ADCSRA |= (1<<ADSC);
      
      // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
      while(ADCSRA & (1<<ADSC));
    
    // save data
    blue = blue+ADCH;
    if(ADCH>225) sat_count++;
    
    PORTB &= 0;  //set LED to LOW


    //////////////// GET GREEN LED READING
    
    PORTD |= 0b10000000;    // //set green LED HIGH port PD7      // set port PB2 high //  PORTB |= 0b100;   
    
    _delay_us(delay);
    ///ADC READ
    
      // start single convertion, write '1' to ADSC
       ADCSRA |= (1<<ADSC);
      
      // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
      while(ADCSRA & (1<<ADSC));
    
    /// save data
    green = green+ADCH;
    if(ADCH>225) sat_count++;
    
    PORTD &= 0;  //set green LED to LOW


    //////// GET RED LED READING
    
    PORTD |= 0b1000000;  //set green LED HIGH port PD6
    
    _delay_us(delay);
    ///ADC READ
    
      // start single convertion, write '1' to ADSC
       ADCSRA |= (1<<ADSC);
      
      // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
      while(ADCSRA & (1<<ADSC));
    
    red = red+ADCH;
    if(ADCH>225) sat_count++;
    if(ADCH > max_red) max_red = ADCH;
    
    PORTD &= 0;  //set green LED to LOW


    //////// GET 850 nm IR LED READING
    
    PORTD |= 0b100000;  // set port PD5 high
    
    _delay_us(delay);
    ///ADC READ
    
        // start single convertion, write '1' to ADSC
         ADCSRA |= (1<<ADSC);
    
        // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
        while(ADCSRA & (1<<ADSC));
    
    // save data
    
    IR850 = IR850+ADCH;
    if(ADCH>225) sat_count++;
    
    
    PORTD &= 0;  //set LED to LOW


    //////// GET 880 nm IR LED READING
    
    PORTD |= 0b10000; // set port PD4 high
    
    _delay_us(delay);
    ///ADC READ
    
        // start single convertion, write '1' to ADSC
         ADCSRA |= (1<<ADSC);
    
        // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
        while(ADCSRA & (1<<ADSC));
    
    
    IR880 = IR880+ADCH;
    if(ADCH>225) sat_count++;
    
    PORTD &= 0;  //set LED to LOW
    

    //////// GET 910 nm LED READING
    
    PORTD |= 0b1000;  // set port PD3 high
    
    _delay_us(delay);
    ///ADC READ
    
      // start single convertion, write '1' to ADSC
       ADCSRA |= (1<<ADSC);
  
      // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
      while(ADCSRA & (1<<ADSC));
    
    // save data
    IR910 = IR910+ADCH;
    if(ADCH>225) sat_count++;
    
    PORTD &= 0;  //set red LED to LOW
    

    //////// GET 940 nm IR LED READING
    
    PORTD |= 0b100; // set port PD2 high
    
    _delay_us(delay);
    ///ADC READ
    
        // start single convertion, write '1' to ADSC
         ADCSRA |= (1<<ADSC);
    
        // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
        while(ADCSRA & (1<<ADSC));
    
    // save data
    IR940 = IR940+ADCH;
    if(ADCH>225) sat_count++;
    if(ADCH > max_IR940) max_IR940 = ADCH;
    
    PORTD &= 0;  //set LED to LOW
    

    //////// GET 970 nm IR LED READING
    
    PORTC |= 0b100000;  // set port PC5 high
    
    _delay_us(delay);
    ///ADC READ
    
        // start single convertion, write '1' to ADSC
         ADCSRA |= (1<<ADSC);
    
        // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
        while(ADCSRA & (1<<ADSC));
    
    IR970 = IR970+ADCH;
    if(ADCH>225) sat_count++;
    
    PORTC &= 0;  //set LED to LOW
    

    //////// GET 1070 nm IR LED READING
    
    PORTC |= 0b10000; // set port PC4 high
    
    _delay_us(delay);
    ///ADC READ
    
        // start single convertion, write '1' to ADSC
         ADCSRA |= (1<<ADSC);
    
        // wait for conversion to complete, ADSC becomes '0' again, loop till then, run loop continuously
        while(ADCSRA & (1<<ADSC));
    
    // save  data
    IR1070 = IR1070+ADCH;
    if(ADCH>225) sat_count++;
    
    PORTC &= 0;  //set LED to LOW
    
  } // end data collection loop


  ////// classifiy and activate air nozzle 
  
  /*
  if(red>blue) RminusB = red-blue;
  if(red<=blue) RminusB = 0;
  
  
  BRratio = (blue<<4)/(red>>4);
  
  IRratio1 = (red<<4)/(IR850>>3);
  IRratio2 = (IR940<<4)/(IR850>>3);
  IRratio3 = (IR1070<<4)/(IR850>>3);
  IRratio4 = (IR940<<4)/(IR1070>>3);
  
  IRdif1 = IR850 - IR940;
  IRdif2 = IR850 - IR1070;
  IRdif3 = IR940 - IR1070;
  
  unsigned int color_sum = (blue + green + red)>>4;
  unsigned int IRsum = (IR850 + IR940 + IR1070)>>4;
  
  unsigned int blue_chrome = (blue<<4)/color_sum;
  unsigned int green_chrome = (green<<4)/color_sum;
  unsigned int red_chrome = (red<<4)/color_sum;
  
  unsigned int IR850_chrome = (IR850<<4)/IRsum;
  unsigned int IR940_chrome = (IR940<<4)/IRsum;
  unsigned int IR1070_chrome = (IR1070<<4)/IRsum;
  */


  data[1] = blue-dark;
  data[2] = green-dark;
  data[3] = red-dark;
  data[4] = IR850-dark;
  data[5] = IR880-dark;
  data[6] = IR910-dark;
  data[7] = IR940-dark;
  data[8] = IR970-dark;
  data[9] = IR1070-dark;
  data[10] = (data[1] + data[2] + data[3])>>4;
  data[11] = (data[4] + data[7] + data[9])>>4;
  data[12] = 1;

  if(F1<=11) disc_data1 = data[F1];
  if(F2<=11) disc_data2 = data[F2];
  if(F3<=11) disc_data3 = data[F3];

  if(F1>=11 && F1<=66) disc_data1 = ((long int)data[n11] - (long int)data[n12]);
  if(F2>=11 && F2<=66) disc_data2 = ((long int)data[n21] - (long int)data[n22]);
  if(F3>=11 && F3<=66) disc_data3 = ((long int)data[n31] - (long int)data[n32]);

  if(F1>=67 && F1<=121) disc_data1 = ((long int)data[n11]<<8)/((long int)data[d1]+256L);
  if(F2>=67 && F2<=121) disc_data2 = ((long int)data[n21]<<8)/((long int)data[d2]+256L);
  if(F3>=67 && F3<=121) disc_data3 = ((long int)data[n31]<<8)/((long int)data[d3]+256L);

  if(F1>=122 && F1<=205) disc_data1 = ((long int)data[n11])+((long int)data[d1]) - ((long int)2*data[n12]);
  if(F2>=122 && F2<=205) disc_data2 = ((long int)data[n21])+((long int)data[d2]) - ((long int)2*data[n22]);
  if(F3>=122 && F3<=205) disc_data3 = ((long int)data[n31])+((long int)data[d3]) - ((long int)2*data[n32]);

  data[13] = disc_data1;
  data[14] = disc_data2;
  data[15] = disc_data3;
  data[16] = dark;



  long int dummy1 = D1*(long int)data[13];

  long int dummy2 = D2*(long int)data[14];

  long int dummy3 = (D3*(long int)data[15]);

  long int discrim = 50L + (DC+dummy3+dummy2+dummy1)/512L;

  // long int discrim = ((DC + D1*(long int)data[13] + D2*(long int)data[14] + D3*(long int)data[15]+32768)>>8);  // adding 32768 centers disc at 32768


  if(sort == 1)
  {
    if(discrim >= reject_thresh && rej_direction == 0)   // the plus 78 changes the default threshold of 50 to 128, where disc should be centered
    { 
      PORTB |= 0b10;  // set port PB1 high
      _delay_us(10);
      PORTB &= 0;
    }
    if(discrim < reject_thresh && rej_direction == 1)   // the plus 78 changes the default threshold of 50 to 128, where disc should be centered
    { 
      PORTB |= 0b10;  // set port PB1 high
      _delay_us(10);
      PORTB &= 0;
    }


    // output just discrim data through RS-232, data is two bytes but must be sent out one byte at a time
    // eventually this should be moved under sort=0 mode
    int dummy = discrim;
    if(dummy<0) dummy = 0;
    UDR0 = dummy;
    while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it
    UDR0 = dummy>>8;
    while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it


    UDR0 = reject_thresh;  
    // UDR0 = max_pix;
    while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    _delay_ms(3);
    
  }


  if(sort == 0) // output data mode
  {
    // output data through RS-232, data is two bytes but must be sent out one byte at a time
    
    int dummy = discrim;
    // if(dummy<0) dummy = 0;
    UDR0 = dummy;
    while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it
    UDR0 = dummy>>8;
    while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    for(i=1; i<=16; i++)
    {
      UDR0 = data[i];  
        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it
      UDR0 = data[i]>>8;  
        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it
    }

    UDR0 = reject_thresh;  
        
        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    UDR0 = n11;  

        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    UDR0 = n12;  

        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    UDR0 = d1;  

        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    UDR0 = n21;  

        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    UDR0 = n22;  

        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    UDR0 = d2;  

        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    UDR0 = n31;  

        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    UDR0 = n32;  

        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    // UDR0 = d3;  
    // UDR0 = rej_direction;
    // UDR0 = max_pix;
    
    UDR0 = ReceivedByte;
    
        while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Do nothing until UDR is ready for more data to be written to it

    _delay_ms(20);

  }
  
}