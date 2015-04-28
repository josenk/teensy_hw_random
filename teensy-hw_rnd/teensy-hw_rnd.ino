//
// 1.01  2015   Jonathan Senkerik josenk@jintegrate.co
//
// Website : http://www.jintegrate.co
// github  : http://github.com/josenk/teensy-hw_rnd
//
// Please support my work and efforts contributing to the Linux community.
//   A $25 payment per server would be highly appreciated.
//
//
//  Copyright (C) 2015 Jonathan Senkerik
//  
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//
//   Teensy 3.x tested.
#include <stdio.h>
#include <EEPROM.h>
#define USBSERIAL Serial

// Define pin locations
#define LED1_pin 13
#define LED2_pin 14
#define BUT1_pin 2


// Tuneables
// eeprom size (MAX depends on true EEPROM size)
#define eeprom_SEEDS_MAX 2011
// USB Buffer size
#define USB_BUF_SZ 64
// Number of Analog input pins.
#define A_MAX 12
// Offset in eeprom to store poison
#define eeprom_SEEDS_Offset 20  
// Offset in eeprom to store default A_Pins
#define eeprom_A_Offset 5
// re-calculate A_pin averages and sort (MAX 65534)
#define Avg_MAX 500
// RND number types
#define RND_type_MAX 15
//  Consider HW RNG compromized if this many repeating reads
#define Compromised 2

// Uncomment to enable DEBUG
// #define DEBUG true


// Variables
byte arr_USB[USB_BUF_SZ+1];                // USB buffer array
byte A_pin[A_MAX+1];                       // Array of Analog input pins
int A_avg[A_MAX+1];                        // Array of Analog input bitness averages
unsigned int arr_MAX=0;                    // Size of the array (0 = auto)

signed char Bitness;                       // Overall Bitness (too many zeros or ones...)
signed char BitsSetTable[256] ;            // Number of bits by byte table
char str_BUF[40];                          // String buffer for debug output.

signed char Compromise_detect=2;           // If >3, then Compromised inputs, use Alternate. 
byte Prev_ByteSlow = 0;                    // Previous HW generated char
byte Prev_ByteFast = 0;                    // Previous HW generated char
unsigned long Lehmer_prng_seed;            // Lehman prng, Seed.
unsigned long Xor32_seed;                  // Used for xorshft32


// Counters 
byte E;                                    // Current value of E (an eeprom byte)
byte ImBusy=0;                             // 0=usb idle, 1=transmitting.
byte RND_type=0;                           // Current RND number type to get
unsigned int Avg_Counter=Avg_MAX;          // Count number of readings before calculating avg
int eeprom_SEEDS = eeprom_SEEDS_MAX;
unsigned long COUNT_Lehmer = 0;            // Counter (used to calculate transmitted/s)
unsigned long COUNT_Xorshft = 0;           // Counter (used to calculate transmitted/s)
unsigned long COUNT_E = 0;                 // Counter (used to calculate transmitted/s)
unsigned long COUNT_HW_S = 0;              // Counter (used to calculate transmitted/s)
unsigned long COUNT_HW_F = 0;              // Counter (used to calculate transmitted/s)



void setup() {
  unsigned int C;
  byte XXX;
  
  A_avg[0]=0; A_avg[1]=0; A_avg[2]=0;
  A_avg[3]=0; A_avg[4]=0; A_avg[5]=0;
  A_avg[6]=0; A_avg[7]=0; A_avg[8]=0;
  A_avg[9]=0; A_avg[10]=0; A_avg[11]=0;

  pinMode(A0, INPUT);  pinMode(A1, INPUT);
  pinMode(A2, INPUT);  pinMode(A3, INPUT);
  pinMode(A4, INPUT);  pinMode(A5, INPUT);
  pinMode(A6, INPUT);  pinMode(A7, INPUT);
  pinMode(A8, INPUT);  pinMode(A9, INPUT);
  pinMode(A10, INPUT);  pinMode(A11, INPUT);
  pinMode(LED1_pin,OUTPUT);
  pinMode(LED2_pin,OUTPUT);
  pinMode(BUT1_pin,INPUT_PULLUP);
  analogReadAveraging(1);
  analogReadRes(8);
  
  while(!USBSERIAL) {}
  USBSERIAL.begin(9600);
    
  #ifdef DEBUG
    USBSERIAL.println("  Debug Mode 1.01  ");
  #endif

  
  // Set default A_pins
  for (C=0;C<A_MAX;C++){  
    A_pin[C]= C;
  }
  
  
  // Initialize eeprom on "first boot" OR pressing the Button on bootup.
  // Write in eeprom the version number.
  if ((EEPROM.read(0) != 1 || EEPROM.read(1) != 0 || EEPROM.read(2) != 1) || digitalRead(BUT1_pin) == 0 ) {
    EEPROM.write(0,1);EEPROM.write(1,0);EEPROM.write(2,1);
    
    // Read a bunch of readings and throw them away.  Used to calculate the bitness tendancies of each pin.
    for (C=0;C<(Avg_MAX + 5);C++) { XXX = GetRndByteSlow(); }
    
    // Load the EEPROM with sensor readings.
    #ifdef DEBUG
      USBSERIAL.println("Init EEPROM...");
    #endif
    for (C=0;C<eeprom_SEEDS_MAX;C++){
      XXX = GetRndByteSlow() ^ GetRndByteFast() ^ (byte)micros();
      EEPROM.write(C + eeprom_SEEDS_Offset,XXX);
      #ifdef DEBUG
        sprintf(str_BUF,"Addr:%-3d  Byte:%.2X   ",C,XXX);
        USBSERIAL.println(str_BUF);
      #endif
    }
    
    // Write default A_pin assignments to EEPROM
    for (C=0;C<A_MAX;C++){
      EEPROM.write(C + eeprom_A_Offset,A_pin[C]);
      #ifdef DEBUG
        USBSERIAL.println("Init EEPROM: W A_pin");
        sprintf(str_BUF,"W A_pin A%-2d Byte:%.2X ",C,A_pin[C]);
        USBSERIAL.println(str_BUF);
      #endif
    }  
  }
  
  //  Read A_pin assignments from EEPROM
  for (C=0;C<A_MAX;C++){
     A_pin[C]= EEPROM.read(C + eeprom_A_Offset);
  }
  
  
  // Table to Quickly calculate number of bits in a byte
  for (C = 0; C < 256; C++) {
    BitsSetTable[C] = ((C & 1) + BitsSetTable[C / 2]) ;    
  }
  for (C = 0; C < 256; C++) {
    BitsSetTable[C] = BitsSetTable[C] -4;
  }
}

void loop() {
  byte V,X,Y;
  int XY_diff;
  unsigned int C,Cstart, Cend;               // Counter, Counter Start & End
  int Cincr;                                 // Counter Incrementer
  
  byte USB_C=USB_BUF_SZ;
  unsigned long COUNT_Trans = 0;             // Total bytes transmitted
  unsigned int RAM;                          // Free RAM
  #ifdef DEBUG
    unsigned int BYTES_PER_Trans = 0;          // Transmitted Bytes / Sec
    unsigned int BYTES_PER_Lehmer = 0;         // Lehmer Bytes / Sec
    unsigned int BYTES_PER_E = 0;              // E Bytes / Sec  
    unsigned int BYTES_PER_Xorshft = 0;        // Xorshft Bytes / Sec  
    unsigned int BYTES_PER_HW_S = 0;           // HW Slow Bytes / Sec
    unsigned int BYTES_PER_HW_F = 0;           // HW Fast Bytes / Sec
    long tm_PerSEC;                            // Timer to calculate Bytes/sec
  #endif

  elapsedMillis tm_SEC400;                   // Do this every 400ms 
  elapsedMillis tm_SEC2;                     // Do this every 2 Secs
  
   
  
  // Initialize PRNG 32bit seeds.
  Lehmer_prng_seed = GetRndByteFast();
  Lehmer_prng_seed = (Lehmer_prng_seed << 8) ^ GetRndByteFast();
  Lehmer_prng_seed = (Lehmer_prng_seed << 8) ^ GetRndByteFast();
  Lehmer_prng_seed = (Lehmer_prng_seed << 8) ^ GetRndByteFast();
  Xor32_seed = GetRndByteFast();
  Xor32_seed = (Xor32_seed << 8) ^ GetRndByteFast();
  Xor32_seed = (Xor32_seed << 8) ^ GetRndByteFast();
  Xor32_seed = (Xor32_seed << 8) ^ GetRndByteFast();
  
  // Setup Array & Fill array with some bytes (quickly)
  RAM = freeRam();
  if ( arr_MAX == 0) { arr_MAX = RAM - 1000; }
  byte arr_SEED[arr_MAX+1];                  // Create Array to hold RND bytes

  // Fill the array with mix of PRNG & HW RNG
  #ifdef DEBUG
    USBSERIAL.println("Init Array...");
    // Initialize timer
    tm_PerSEC = millis();
  #endif
  
  for (C=0;C<=arr_MAX;C++) {
    arr_SEED[C] = GetPRNG() ^ GetRndByteFast();
  }
  
  //  Loop through Array forever
  while (0 == 0) {
    
    XY_diff=0;
    while ( XY_diff <64 || XY_diff >192){
      X=GetRndByte();
      Y=GetRndByte();
      XY_diff = abs(X-Y);
    }
    Cstart = (arr_MAX * (X * 0.00384615385)) ;             // Set a Random Start
    Cend = (arr_MAX - (arr_MAX * (Y * 0.00384615385)));    // Set a Random End
    Cincr  = (unsigned int)(GetE() & 7 ) + 2;              // Set a Random Incr 2-9 
    if (Cstart > Cend ) { Cincr = Cincr * -1; }            // If Start is larger, then decrement
    
    C=Cstart;
    while (C != Cend){
      C=C+Cincr;
      if ( Cincr > 0 && C >= Cend) {break;}
      if ( Cincr < 0 && C <= Cend) {break;}
      
      if ( ImBusy == 0) {
        X=GetRndByteSlow();
        Y=GetRndByte();
      } else {
        X=GetPRNG();
      }
  
      V = (X ^ Y) ^ arr_SEED[C];                             // V = Calculated byte to send
      arr_SEED[C] = arr_SEED[C] ^ X ;                        // Write to Array a new value


      //  Put V in the USB buffer  
      USB_C--; arr_USB[USB_C] = V;

      // Send the data when the USB buffer is full
      if ( USB_C <= 0 ) {
        USB_C = USB_BUF_SZ;
        if (USBSERIAL.dtr() && USBSERIAL.rts() ) {
          ImBusy=1;
          #ifndef DEBUG
            USBSERIAL.write(arr_USB,USB_BUF_SZ );
          #endif
          COUNT_Trans = COUNT_Trans + USB_BUF_SZ;
        } else {  // USB is not ready.  Start again
          ImBusy=0;
          if (!USBSERIAL)
            { USBSERIAL.begin(9600); }
          Y = GetRndByte();
          X = GetRndByte();
        }
      }
    }
  
 
    // Do this every .4 seconds
    if ( tm_SEC400 >= 400 ) {
      tm_SEC400 = 0; 
     
      while (USBSERIAL.available())    // If someone is sending us data, flush it.
        { USBSERIAL.read(); }
        
      if (USBSERIAL.dtr()) {           // Set LED on if sending data
        digitalWrite(LED1_pin,1);
        ImBusy=1; 
      }                         
      else {
        digitalWrite(LED1_pin,0);
        ImBusy=0; 
      } 
        
      eeprom_SEEDS--;
      if ( eeprom_SEEDS == 0 ) { eeprom_SEEDS = eeprom_SEEDS_MAX; }
    }
  
    
    #ifdef DEBUG
    if ( tm_SEC2 >= 2000 ) {
      tm_SEC2 = 0; 

      BYTES_PER_Trans = COUNT_Trans / (millis() - tm_PerSEC);              // Calculate bytes/sec 
      BYTES_PER_Lehmer = (COUNT_Lehmer*1000) / (millis() - tm_PerSEC);     // Calculate bytes/millisec
      BYTES_PER_E = (COUNT_E*1000) / (millis() - tm_PerSEC);               // Calculate bytes/millisec
      BYTES_PER_Xorshft = COUNT_Xorshft / (millis() - tm_PerSEC);          // Calculate bytes/sec
      BYTES_PER_HW_S = (COUNT_HW_S*1000) / (millis() - tm_PerSEC);         // Calculate bytes/millisec
      BYTES_PER_HW_F = (COUNT_HW_F*1000) / (millis() - tm_PerSEC);         // Calculate bytes/millisec
      
      COUNT_Trans = 0; COUNT_Lehmer = 0; COUNT_E = 0; COUNT_Xorshft = 0; COUNT_HW_S = 0; COUNT_HW_F = 0;
      tm_PerSEC = millis();
    
      RAM = freeRam();
      sprintf(str_BUF,"Free:%-4d Arr Sz:%-5d", RAM, arr_MAX);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"Trans/s:%-4d Leh/ms:%-4d",BYTES_PER_Trans,BYTES_PER_Lehmer);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"E/ms:%-3d Xor/s:%-4d",BYTES_PER_E,BYTES_PER_Xorshft);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"Slow/ms:%-3d Fast/ms:%-3d",BYTES_PER_HW_S,BYTES_PER_HW_F);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"Bitness %+3d  Busy:%d",Bitness, ImBusy);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"Lehmer Seed:%.8X",Lehmer_prng_seed);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"Xorshft    :%.8X",Xor32_seed);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF," X:%.2X  Y:%.2X  E:%.2X",X,Y,E); 
      USBSERIAL.println(str_BUF); 
      sprintf(str_BUF,"X %2d:%-5d %2d:%-5d",A_pin[0],A_avg[0],A_pin[1],A_avg[1]);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"  %2d:%-5d %2d:%-5d",A_pin[2],A_avg[2],A_pin[3],A_avg[3]);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"  %2d:%-5d %2d:%-5d",A_pin[4],A_avg[4],A_pin[5],A_avg[5]);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"  %2d:%-5d %2d:%-5d",A_pin[6],A_avg[6],A_pin[7],A_avg[7]);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"  %2d:%-5d %2d:%-5d",A_pin[8],A_avg[8],A_pin[9],A_avg[9]);
      USBSERIAL.println(str_BUF);
      sprintf(str_BUF,"X %2d:%-5d %2d:%-5d",A_pin[10],A_avg[10],A_pin[11],A_avg[11]);
      USBSERIAL.println(str_BUF);  
    }
    #endif
  }
}


// Get a random byte.
// Mostly from PRNG.
byte GetRndByte() {
  byte RNDbyte;
  RND_type++;
  if (RND_type >= RND_type_MAX) {RND_type = 0; }
  
  switch (RND_type) {
      case 0: RNDbyte = GetRndByteSlow();break;
      case 3: RNDbyte = GetE(); break;
      case 5: RNDbyte = GetRndByteFast(); break;
      case 10: RNDbyte = GetRndByteFast(); break;
      default: RNDbyte = GetPRNG();          
  }
  
  //  If we got a compromised bytes, use Lehmer PRNG instead.
  if (Compromise_detect >= Compromised ) {
    RNDbyte = GetPRNG2() ;
  }
 
  return RNDbyte; 
}

// Get a byte from eeprom
byte GetE() {
    COUNT_E++;
    eeprom_SEEDS--;
    if ( eeprom_SEEDS == 0 )
      { eeprom_SEEDS = eeprom_SEEDS_MAX; }
    E = EEPROM.read(eeprom_SEEDS_Offset + eeprom_SEEDS);
    return E;
}


//  Get a random byte through Lehmer PRNG.
//  The HW RNG backup.
byte GetPRNG2() {
  byte RNDbyte;
  signed char B; 
  
  unsigned long AA = 48271;              // Lehmer PRNG constants
  unsigned long MM = 2147483647;         // Lehmer PRNG constants
  unsigned long QQ = MM/AA;              // Lehmer PRNG constants
  unsigned long RR = MM % AA;            // Lehmer PRNG constants
  COUNT_Lehmer++;          

  unsigned long TMPVar =  AA * (Lehmer_prng_seed % QQ) - RR * (Lehmer_prng_seed /QQ);
  
  // Convert unsigned long to byte
  Lehmer_prng_seed = TMPVar;
  RNDbyte = (Lehmer_prng_seed>>4 & 255) ^
            (Lehmer_prng_seed>>12 & 255) ^
            (Lehmer_prng_seed>>20 & 255);
    
  B = BitsSetTable[RNDbyte];    // Check Bitness and inverse (Xor 255) if too much one sided           
  if ((Bitness < -25 &&  B<0) || (Bitness > 25 && B >0))
    { RNDbyte = RNDbyte ^ 255; Bitness = Bitness - B;}
    else { Bitness = Bitness + B; }
  
  return RNDbyte;
}


//  Get a random byte through Xorshift PRNG.
//  This is very, very fast compared to others.
byte GetPRNG() {
  COUNT_Xorshft++;
  Xor32_seed ^= Xor32_seed << 13;
  Xor32_seed ^= Xor32_seed >> 17;
  Xor32_seed ^= Xor32_seed << 5;
  return (byte)Xor32_seed;
}

// Get a random byte using all Analog pins. (reject outer 4)
// Check for bitness tendancies of ALL Analog inputs.
//    Even if we don't use that pin right now.
byte GetRndByteSlow() {
  byte TmpVar0,TmpVar1,TmpVar2,TmpVar3,TmpVar4,TmpVar5;
  byte TmpVar6,TmpVar7,TmpVar8,TmpVar9,TmpVar10,TmpVar11;
  byte RNDbyte;
  signed char B;

  COUNT_HW_S++;  

  TmpVar0 = analogRead(A_pin[0]) & 1;
  TmpVar1 = analogRead(A_pin[1]) & 1;  
  TmpVar2 = analogRead(A_pin[2]) & 1;
  TmpVar3 = analogRead(A_pin[3]) & 1;   
  TmpVar4 = analogRead(A_pin[4]) & 1;
  TmpVar5 = analogRead(A_pin[5]) & 1;
  TmpVar6 = analogRead(A_pin[6]) & 1;
  TmpVar7 = analogRead(A_pin[7]) & 1;
  TmpVar8 = analogRead(A_pin[8]) & 1;
  TmpVar9 = analogRead(A_pin[9]) & 1;
  TmpVar10 = analogRead(A_pin[10]) & 1;
  TmpVar11 = analogRead(A_pin[11]) & 1;
            
  Avg_Counter--;
  if ( Avg_Counter == 1) { Avg_Counter = Avg_MAX; DoAvg(); }

  if (TmpVar0 == 0) { A_avg[0]++; }
  if (TmpVar1 == 0) { A_avg[1]++; }
  if (TmpVar2 == 0) { A_avg[2]++; }
  if (TmpVar3 == 0) { A_avg[3]++; }
  if (TmpVar4 == 0) { A_avg[4]++; }
  if (TmpVar5 == 0) { A_avg[5]++; }
  if (TmpVar6 == 0) { A_avg[6]++; }
  if (TmpVar7 == 0) { A_avg[7]++; }
  if (TmpVar8 == 0) { A_avg[8]++; }
  if (TmpVar9 == 0) { A_avg[9]++; }
  if (TmpVar10 == 0) { A_avg[10]++; }
  if (TmpVar11 == 0) { A_avg[11]++; }
  
  RNDbyte = TmpVar2 + (TmpVar3 <<1) + (TmpVar4 <<2) + (TmpVar5 <<3)
            + (TmpVar6 <<4) + (TmpVar7 <<5) + (TmpVar8 <<6) + (TmpVar9 <<7);
  
  // It's OK to have the same result as last time, or all 0s, or all 1s, but 
  // not allways.  Use Lehmer as backup plan.  
  if (RNDbyte == Prev_ByteSlow || RNDbyte == 0 ||RNDbyte == 255 ) {
    Compromise_detect++;
    if (Compromise_detect >= Compromised ) { Compromise_detect = Compromised; }
  }
  else {
    Compromise_detect=0;
  }
  Prev_ByteSlow = RNDbyte;   // Save this byte to compare with next run
  
  B = BitsSetTable[RNDbyte];    // Check Bitness and inverse (Xor 255) if too much one sided       
  if ((Bitness < -25 &&  B<0) || (Bitness > 25 && B >0))
    { RNDbyte = RNDbyte ^ 255; Bitness = Bitness - B; }
    else {Bitness = Bitness + B; }
  
  return RNDbyte;
}


//  Get a random byte Fast (using only the "best" 8 Analog inputs)
byte GetRndByteFast() {
  byte TmpVar0,TmpVar1,TmpVar2,TmpVar3;
  byte TmpVar4,TmpVar5,TmpVar6,TmpVar7;
  byte RNDbyte;
  signed char B;
 
  COUNT_HW_F++; 
  
  TmpVar0 = analogRead(A_pin[2]) & 1;
  TmpVar6 = analogRead(A_pin[3]) & 1;   
  TmpVar4 = analogRead(A_pin[4]) & 1;
  TmpVar2 = analogRead(A_pin[5]) & 1;
  TmpVar5 = analogRead(A_pin[6]) & 1;
  TmpVar3 = analogRead(A_pin[7]) & 1;
  TmpVar7 = analogRead(A_pin[8]) & 1;
  TmpVar1 = analogRead(A_pin[9]) & 1;
  
  RNDbyte = TmpVar0 + (TmpVar1 <<1) + (TmpVar2 <<2) + (TmpVar3 <<3)
          + (TmpVar4 <<4) + (TmpVar5 <<5) + (TmpVar6 <<6) + (TmpVar7 <<7);

  if (RNDbyte == Prev_ByteFast || RNDbyte == 0 ||RNDbyte == 255 ) {
    Compromise_detect++;
    if (Compromise_detect >= Compromised ) { Compromise_detect = Compromised; }
  }
  else {
    Compromise_detect=0;
  }
  Prev_ByteFast = RNDbyte;
  
  B = BitsSetTable[RNDbyte];    // Check Bitness and inverse (Xor 255) if too much one sided           
  if ((Bitness < -25 &&  B<0) || (Bitness > 25 && B >0))
    { RNDbyte = RNDbyte ^ 255; Bitness = Bitness - B;}
    else {Bitness = Bitness + B; }
  
  return RNDbyte;
}


// Calculate bitness tendancies of each Analog pin.
// This will disable stuck pins at 0 or 1.
// Keep only the center 8 inputs of 12 Analog inputs.
//    throw away top two and bottom two.  (most zeros and most ones)
void DoAvg(){
    int C,CC;
    int TT;
    byte TTT;

    for (CC=0;CC<(A_MAX-1);CC++) {
      for (C=0;C<(A_MAX-1);C++) {
        if ( A_avg[C] > A_avg[C+1] ) {
          TT = A_avg[C];
          A_avg[C] = A_avg[C+1];
          A_avg[C+1] = TT;
          TTT = A_pin[C];
          A_pin[C] = A_pin[C+1];
          A_pin[C+1] = TTT;
        }
      }
    }
    
    // Reset the readings to zero.
    for (C=0;C<(A_MAX);C++) {
      A_avg[C]=0;
    }
}


uint32_t freeRam(){ // for Teensy 3.x
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t) &stackTop;

    // current position of heap.
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram.
    return stackTop - heapTop;
}  

