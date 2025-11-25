// 6 Channel Transmitter | 6 Kanal Verici
// KendinYap Channel

#include "main.h"
#include <SPI.h>
#include <EEPROM.h>
#include <U8g2lib.h>
// #include <Wire.h>
#include "display.h"
// #include "expo.h"
#include "expo8.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <Bounce2.h> // github.com/thomasfredericks/Bounce2
#include <avr/io.h>

#include <elapsedMillis.h>
#include "defines.h"

#include "MS5611.h"

const uint64_t pipeOut = 0xABCDABCD71LL; // NOTE: The address in the Transmitter and Receiver code must be the same "0xABCDABCD71LL" | Verici ve Alıcı kodundaki adres aynı olmalıdır

int Border_Mapvar255_raw( int val, int lower, int middle, int upper, bool reverse);

// extern "C"

// github.com/olikraus/u8g2/discussions/1865

// 0.96"
//  code in display.h

uint16_t loopcounter0 = 0;
uint16_t loopcounter1 = 0;

#define RAMPETEST 0
#define TEST 0
#define CE_PIN 8 // Teensy_FS: Pin 9
#define CSN_PIN 10

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

#define LOOPLED 9

#define EEPROMTASTE 5



#define BLINKRATE 0x4FF


uint8_t debouncecheck = 0;

uint8_t blinkcounter = 0;
uint8_t impulscounter = 0;
uint16_t throttlecounter = 0;
uint16_t throttlesekunden = 0;

// RC_22
// #define POT0LO 620  // Min wert vom ADC Pot 0
// #define POT0HI 3400 // Max wert vom ADC Pot 0

#define POTLO 0
#define POTHI 710

// Impulslaenge, ms
#define PPMLO 850  // Minwert ms fuer Impulslaenge
#define PPMHI 2150 // Maxwert ms fur Impulslaenge

#define MINDIFF 4

elapsedMillis              sincelastpaket = 0;
IntervalTimer              servoimpulsTimer;
IntervalTimer              kanalimpulsTimer;
uint8_t                    slaveimpulscounter = 0;
volatile uint8_t           servoindex = 0;
volatile uint16_t          slaveimpulstimearray[NUM_SERVOS] = {};

// Decoder
uint16_t Slavechannelarray[NUM_SERVOS] = {};
uint16_t Slavechannelmittearray[NUM_SERVOS] = {};

volatile uint8_t slaveindex = 0;
volatile uint32_t last = 0;




uint16_t schritt = 32;


uint16_t impulstimearray[NUM_SERVOS] = {};

const int adcpinarrayTeensy[NUM_SERVOS] = {A7, A6, A1, A0}; // pins der Pots Teensy

uint8_t kanalsettingarray[ANZAHLMODELLE][NUM_SERVOS][KANALSETTINGBREITE] = {};

uint16_t servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte

uint8_t levelwert = 0;
// uint8_t levelwerta = 0;
// uint8_t levelwertb = 0;

uint8_t levelwertarray[NUM_SERVOS] = {}; // leelwert pro servo

uint8_t levelwertayaw = 0;
uint8_t levelwertbyaw = 0;

uint16_t potwertyaw = 0;

uint8_t expowert = 0;
// uint8_t expowerta = 0;
// uint8_t expowertb = 0;

uint8_t expowertarray[NUM_SERVOS] = {}; // expowert pro Servo

uint16_t potwertarray[NUM_SERVOS] = {}; // Werte fuer Servo

uint16_t potwertarrayraw[NUM_SERVOS] = {}; // Original Werte fuer Servo

// uint16_t          externpotwertarray[NUM_SERVOS] = {}; // Werte von extern  pro servo

// uint16_t currentexpoarray[5][513] = {};

uint8_t curr_pfeil = 0;

// uint16_t      blink_cursorpos=0xFFFF;

uint16_t stopsekunde = 0;
uint16_t stopminute = 0;
uint16_t motorsekunde = 0;
uint16_t motorminute = 0;
uint8_t motorstunde = 0;

uint16_t sendesekunde = 0;
uint16_t sendeminute = 0;
uint8_t sendestunde = 0;

// Status
uint8_t blinkstatus = 0;

uint8_t curr_steuerstatus = 0;

uint8_t calibstatus = 0;

uint8_t savestatus = 0;

uint8_t anzeigestatus = 0;

float potlo = POTLO; // min pot
float pothi = POTHI; // max pot
float ppmlo = PPMLO; // min ppm
float ppmhi = PPMHI; // max ppm

uint16_t diffa = 0;

uint16_t potwertpitch = 0;
uint16_t diffapitch = 0;
uint16_t diffbpitch = 0;

uint16_t diffb = 0;
float expofloat = 0;
uint16_t expoint = 0;
uint16_t levelint = 0;

uint16_t levelintraw = 0;

uint16_t levelintcheck = 0;

uint16_t levelintpitcha = 0;

uint16_t levelintpitchb = 0;
float batteriespannung = 0;
float batteriespannungraw = 0;

uint16_t batteriearray[8] = {};
uint16_t batteriemittel = 0;
uint8_t batteriemittelwertcounter = 0;
uint16_t batterieanzeige = 0;
float UBatt = 0;

float flyerbatteriespannung = 0;
float flyerbatteriespannungraw = 0;
uint16_t flyerbatterieanzeige = 0;
float UFlyerBatt = 0;

uint16_t pressureint = 0;
float pressurefloat = 0;
const float seaLevelPressure = 1013.25;

float altitude = 0;
uint16_t altitudeint = 0;

uint8_t temperaturint = 0;
float temperaturfloat = 0;

uint8_t eepromstatus = 0;
uint16_t eepromprelltimer = 0;
uint16_t intdiff = 0;
uint16_t intdiffpitch = 0;

Bounce2::Button eepromtaste = Bounce2::Button();

#define ANZ_REP 8
uint16_t tastaturwert = 0;
uint16_t tastaturwertarray[ANZ_REP] = {};
uint8_t mittelposition = 0;

uint16_t winkelcounter = 0;
uint8_t rampe = 0;
int8_t ramprichtung = 1;

uint8_t tastencounter = 0;
uint8_t tastaturstatus = 0;
uint8_t Taste = 0;
uint8_t taste5counter = 0;

uint8_t taskarray[4] = {'Y', 'P', 'R', 'T'};

uint16_t potgrenzearray[NUM_SERVOS][2]; // obere und untere Grenze von adc

float quot = (ppmhi - ppmlo) / (pothi - potlo);

float expoquot = (ppmhi - ppmlo) / 2 / 0x200; // umrechnen der max expo (512) auf PPM
// uint8_t sinarray[127] = {127,133,139,145,151,156,162,168,173,179,184,189,194,199,204,208,212,216,220,224,227,230,233,236,238,240,242,244,245,246,247,247,247,247,246,245,244,243,241,239,237,235,232,229,226,222,218,214,210,206,201,197,192,187,181,176,170,165,159,153,148,142,136,130,124,118,112,106,100,95,89,83,78,73,67,62,57,53,48,44,40,36,32,28,25,22,19,17,15,13,11,10,9,8,7,7,7,7,8,9,10,12,14,16,18,21,24,27,30,34,38,42,46,50,55,60,65,70,75,81,86,92,98,103,109,115,121};
//  MasterSlave
uint8_t masterslavestatus = 0;
#define MASTER 0
#define SLAVE 1

volatile uint8_t currentChannel = 0;
volatile uint8_t pausecounter = 0;
volatile bool pulseState = true;
volatile uint32_t elapsedFrame = 0;
volatile uint8_t ISRcounter = 0;

volatile uint16_t channels[NUM_SERVOS] =
    {
        1500, 1500, 1500, 1500};

// V2
volatile uint16_t ppm[NUM_SERVOS] = {1500, 2000, 1500, 1000};

// volatile uint8_t currentChannel = 0;
volatile uint16_t restTime = FRAME_LENGTH;

// OLED > in display.cpp
uint16_t pot0 = 0;

uint16_t potwert = 0;

uint16_t errcounter = 0;
uint16_t radiocounter = 0;

// ********************
// ACK data ***********
uint8_t ackData[4] = {0};
// ********************
// ********************

// uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

// uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor

unsigned char char_x = 0;
unsigned char char_y = 0;

// Menu
uint8_t curr_model = 0; // aktuelles modell
uint8_t speichermodel = 0;
uint8_t curr_funktion = 0; // aktuelle funktion
uint8_t curr_aktion = 0;   // aktuelle aktion

uint8_t curr_wert = 0;

uint8_t curr_impuls = 0; // aktueller impuls

uint8_t curr_modus = 0; // Modell oder Sim oder Calib

uint8_t curr_setting = 0; // aktuelles Setting fuer Modell
uint8_t speichersetting = 0;

uint8_t curr_trimmkanal = 0; // aktueller  Kanal fuerTrimmung
uint8_t curr_trimmung = 0;   // aktuelle  Trimmung fuer Trimmkanal

uint8_t curr_screen = 0; // aktueller screen
uint8_t last_screen = 0; // letzter MASTERscreen

uint8_t curr_page = 7; // aktuelle page
uint8_t curr_col = 0;  // aktuelle colonne

uint8_t curr_cursorzeile = 0;  // aktuelle zeile des cursors
uint8_t curr_cursorspalte = 0; // aktuelle colonne des cursors
uint8_t last_cursorzeile = 0;  // letzte zeile des cursors
uint8_t last_cursorspalte = 0; // letzte colonne des cursors

// Tastatur
uint8_t Tastenindex = 0;
uint16_t Tastenwert = 0;
uint8_t adcswitch = 0;
uint16_t lastTastenwert = 0;
int16_t Tastenwertdiff = 0;
uint16_t tastaturcounter = 0;
uint16_t tastaturdelaycounter = 0;

elapsedMillis zeitintervall;
uint8_t sekundencounter = 0;
elapsedMillis sinceLastBlink = 0;

elapsedMillis paketcounter;
elapsedMillis sincelasttastatur;

uint16_t impulsdelayarray[4] = {};
uint16_t restzeitarray[4] = {};
uint16_t impulsCCMParray[4] = {};
uint16_t restCCMParray[4] = {};
uint16_t restCCM = 0;
elapsedMillis buzzintervall = 0;

int Border_Mapvar255(uint8_t servo, int val, int lower, int middle, int upper, bool reverse);

Signal data;
void ResetData()
{
   data.throttle = 0;
   data.pitch = 127;
   data.roll = 127;
   data.yaw = 127;
   data.aux1 = 0;
   data.aux2 = 0;
}

// ppm encode
// Timer-ISR

// V2
// ISR für TCB0

void OSZIA_HI(void)
{
   digitalWriteFast(OSZIA_PIN, HIGH);
}
void OSZIA_LO(void)
{
   digitalWriteFast(OSZIA_PIN, LOW);
}
void OSZIA_TOG()
{
   digitalWriteFast(OSZIA_PIN, !(digitalRead(OSZIA_PIN)));
}

uint8_t debounceTaste()
{
   static uint8_t debounced_state = 0;
   static uint16_t state = 0;
   state = (state << 1) | (tastaturwert > 10);
   if (state >= 0xF00)
   {
      // debouncecheck = 1;
      return 1;
   }

   // debouncecheck = 0;
   return 2;
}

void setupDebounce()
{
}

// PPM decode

volatile unsigned long lastTime = 0;
volatile unsigned long pulseLength = 0;
volatile byte channel = 0;
const byte maxChannels = 8;
// volatile unsigned int ppmValues[maxChannels];


void slaveISR() 
{
    uint32_t now = micros();
    uint32_t dur = now - last;
    last = now;

    if (dur > 3000) 
    { 
      slaveindex = 0;                 // Sync → Frame neu
    } 
    else if (slaveindex < NUM_SERVOS) 
    {
      if ((calibstatus & (1 << CALIB_START)))
      {
         Slavechannelmittearray[slaveindex] = dur;
      }
        //uint8_t red = map(dur,1000,2000,0,255);
        uint8_t red = Border_Mapvar255_raw( dur, 1000, Slavechannelmittearray[slaveindex], 2000, false);
        Slavechannelarray[slaveindex] = red;   // Kanalwert speichern
        slaveindex++;
    }
}

void ppmISR()
{
   unsigned long now = micros();
   pulseLength = now - lastTime;
   lastTime = now;

   if (pulseLength > 3000)
   {
      digitalWrite(PPM_DIR_PIN, HIGH);
      // Sync-Pause erkannt: neues Frame beginnt
      channel = 0;
      // digitalWrite(PPM_DIR_PIN, !(digitalRead(PPM_DIR_PIN)));
      digitalWrite(PPM_DIR_PIN, LOW);
   }
   else if (channel < maxChannels)
   {

      Slavechannelarray[channel] = pulseLength;
      channel++;
   }
}

void updatemitte(void)
{

} // updatemitte

void printgrenzen()
{
   // Serial.print("\nprintgrenzen\n");
   for (uint8_t i = 0; i < NUM_SERVOS; i++)
   {
      // Serial.print("grenzen i:\t");
      // Serial.print(i);
      // Serial.print("\t");
      // Serial.write(taskarray[i]);
      // Serial.print("\t");
      // Serial.print("potgrenze HI:\t");
      // Serial.print(potgrenzearray[i][0]);
      // Serial.print("\t");
      // Serial.print("potgrenze LO:\t");
      // Serial.print(potgrenzearray[i][1]);
      // Serial.print("\t");
      // Serial.print("servomitte:\t");
      // Serial.print(servomittearray[i]);
      // Serial.print("\n");
   }
   // Serial.print("end printgrenzen\n");
}

void printeeprom(uint8_t zeilen)
{
   Serial.print("printeeprom\n");
   for (uint8_t i = 0; i < zeilen; i++)
   {
      // Serial.write(taskarray[i]);
      ////Serial.print("\t");
      uint8_t f = EEPROM.read(i);

      if ((i) % 8 == 0)
      {
         Serial.print("\n");
         Serial.print(i);
         Serial.print(": ");
         Serial.print("\t");
         // Serial.print(f);
         // Serial.print("\n");
      }
      // else
      {
         ////Serial.print(i);
         ////Serial.print(": ");

         Serial.print(f);
         Serial.print("\t");
      }
   }
   Serial.print("\n");
   uint8_t eepromyawlo = EEPROM.read(2 * (0 + EEPROMINDEX_U));
   uint8_t eepromyawhi = EEPROM.read(2 * (0 + EEPROMINDEX_U) + 1);
   uint16_t eepromyaw = (eepromyawhi << 8) | eepromyawlo;

   // Serial.print("eeprompitch U: \t");
   // Serial.print(eepromyawlo);
   // Serial.print("\t");
   // Serial.print(eepromyawhi);
   // Serial.print("\t");
   // Serial.print(eepromyaw);
   // Serial.print("\n");

   eepromyawlo = EEPROM.read(2 * (0 + EEPROMINDEX_O));
   eepromyawhi = EEPROM.read(2 * (0 + EEPROMINDEX_O) + 1);
   eepromyaw = (eepromyawhi << 8) | eepromyawlo;

   // Serial.print("eepromyaw O: \t");
   // Serial.print(eepromyawlo);
   // Serial.print("\t");
   // Serial.print(eepromyawhi);
   // Serial.print("\t");
   // Serial.print(eepromyaw);
   // Serial.print("\n");
}

void eepromread()
{
   // Serial.print("eepromread \t");
   ////Serial.print("kontrolle Adresse A: ");
   ////Serial.print(EEPROM.read(0));
   ////Serial.print(" Adresse B: ");
   ////Serial.println(EEPROM.read(0));
   for (uint8_t i = 0; i < NUM_SERVOS; i++)
   {
      // Serial.write(taskarray[i]);
      // Serial.print("\t");
      uint8_t l = (potgrenzearray[i][0] & 0x00FF);      // lo byte
      uint8_t h = (potgrenzearray[i][0] & 0xFF00) >> 8; // hi byte
      // Serial.print("potgrenzearray 0\t");
      // Serial.print(potgrenzearray[i][0]);
      // Serial.print("\t");
      uint16_t grenzeU = (h << 8) | l;
      // Serial.print("grenzeU\t");

      uint8_t el = 0;
      uint8_t eh = 0;
      el = EEPROM.read(2 * (i + EEPROMINDEX_U));     // lo byte
      eh = EEPROM.read(2 * (i + EEPROMINDEX_U) + 1); // hi byte
      // Serial.print(el);
      // Serial.print("\t");
      // Serial.print(eh);
      // Serial.print("\t");

      potgrenzearray[i][1] = (eh << 8) | el;

      el = EEPROM.read(2 * (i + EEPROMINDEX_O));     // lo byte
      eh = EEPROM.read(2 * (i + EEPROMINDEX_O) + 1); // hi byte
      // Serial.print(el);
      // Serial.print("\t");
      // Serial.print(eh);
      // Serial.print("\t");

      potgrenzearray[i][0] = (eh << 8) | el;

      el = EEPROM.read(2 * (i + EEPROMLEVELSETTINGS));
      el &= 0x03;
      kanalsettingarray[0][i][1] = el; // modell 0

      eh = EEPROM.read(2 * (i + EEPROMEXPOSETTINGS));
      eh &= 0x03;
      kanalsettingarray[0][i][2] = eh; // modell 0

      if (i == 0)
      {
         // Serial.print("\n");
         // Serial.print("level\t");
         // Serial.print(el);
         // Serial.print("\t");
         // Serial.print("expo\t");
         // Serial.println(eh);
      }

      

   } // for i
   // Serial.print("\n");
}

void clearsettings(void)
{
   // Serial.print("clearsettings\n");
   for (uint8_t i = 0; i < NUM_SERVOS; i++)
   {
      kanalsettingarray[curr_model][i][1] = 0x00; // level
      kanalsettingarray[curr_model][i][2] = 0x00; // level

   } // for i
}

void cleargrenzen(void)
{
   // Serial.print("cleargrenzen\n");
   for (uint8_t i = 0; i < NUM_SERVOS; i++)
   {
      potgrenzearray[i][0] = 127; //
      potgrenzearray[i][1] = 127;

   } // for i
}

void eepromwrite(void)

{
   Serial.print("eepromwrite\n");
   for (uint8_t i = 0; i < NUM_SERVOS; i++)
   {
      Serial.print("potgrenzearray raw i:\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.write(taskarray[i]);
      Serial.print("\t");
      Serial.print("potgrenze HI:\t");
      Serial.print(potgrenzearray[i][0]);
      Serial.print("\t");
      Serial.print("potgrenze LO:\t");
      Serial.print(potgrenzearray[i][1]);
      Serial.print("\t");
      Serial.print("servomitte:\t");
      Serial.print(servomittearray[i]);
      Serial.print("\t");
      Serial.print("adresse U:\t");
      uint8_t addresseU_LO = 2 * (i + EEPROMINDEX_U);
      Serial.print(addresseU_LO);
      Serial.print("\t");
      Serial.print("adresse H:\t");
      uint8_t addresseU_HI = 2 * (i + EEPROMINDEX_U) + 1;
      Serial.print(addresseU_HI);

      Serial.print(" *\t");
      Serial.print("level:\t");
      Serial.print(kanalsettingarray[curr_model][i][1]);
      Serial.print(" \t");
      Serial.print("expo:\t");
      Serial.print(kanalsettingarray[curr_model][i][2]);

      Serial.print("\n");

      EEPROM.update(2 * (i + EEPROMINDEX_U), (potgrenzearray[i][1] & 0x00FF)); // lo byte
      _delay_ms(1);
      EEPROM.update(2 * (i + EEPROMINDEX_U) + 1, ((potgrenzearray[i][1] & 0xFF00) >> 8)); // hi byte
      _delay_ms(1);
      EEPROM.update(2 * (i + EEPROMINDEX_O), (potgrenzearray[i][0] & 0x00FF)); // lo byte
      _delay_ms(1);
      EEPROM.update(2 * (i + EEPROMINDEX_O) + 1, ((potgrenzearray[i][0] & 0xFF00) >> 8)); // hi byte
      _delay_ms(1);

      EEPROM.update(2 * (i + EEPROMINDEX_M), (servomittearray[i] & 0x00FF)); // lo byte
      _delay_ms(1);
      EEPROM.update(2 * (i + EEPROMINDEX_M) + 1, ((servomittearray[i] & 0xFF00) >> 8)); // hi byte
      _delay_ms(1);

      for (uint8_t i = 0; i < 8; i++)
      {
         //     EEPROM.write(2*(i + EEPROMLEVELSETTINGS)+i,255); // lo byte
         //    EEPROM.write(2*(i + EEPROMEXPOSETTINGS)+i,255);
      }

      EEPROM.update(2 * (i + EEPROMLEVELSETTINGS), (kanalsettingarray[curr_model][i][1])); // level
      _delay_ms(1);
      EEPROM.update(2 * (i + EEPROMEXPOSETTINGS), (kanalsettingarray[curr_model][i][2])); // expo
      _delay_ms(1);

      EEPROM.update(2 * (i + EEPROMSLAVEINDEX_M), Slavechannelmittearray[i]); // slave mitte
      _delay_ms(1);


      // EEPROM.update(2*(i + EEPROMLEVELSETTINGS),(47+i)); // level
      // EEPROM.update(2*(i + EEPROMEXPOSETTINGS),(63+i )); // expo

      EEPROM.update(0, 17);
      _delay_ms(1);
      EEPROM.update(1, 37);

      delay(20);
      /*
       //Serial.print("kontrolle i: \t*");
       //Serial.print(i);
       //Serial.print("\t");
       uint8_t el = EEPROM.read(2*(i + EEPROMINDEX_U)); // lo byte
       uint8_t eh = EEPROM.read(2*(i + EEPROMINDEX_U)+1); // hi byte
       //Serial.print(el);
       //Serial.print("\t");
       //Serial.print(eh);
       //Serial.print("\t");
       uint16_t grenzeU = (eh << 8) | el;
       //Serial.print("grenzeU eeprom:\t");
       //Serial.print(grenzeU);
       //Serial.print(" *\n");
       */
   }

    Serial.print("eepromwrite end\n");
}

uint8_t Joystick_Tastenwahl_33_2(uint16_t Tastaturwert)
{
   // return 0;
   if (Tastaturwert < JOYSTICKTASTE1)
      return 2;
   if (Tastaturwert < JOYSTICKTASTE2)
      return 1;
   if (Tastaturwert < JOYSTICKTASTE3)
      return 4;
   if (Tastaturwert < JOYSTICKTASTE4)
      return 7;
   if (Tastaturwert < JOYSTICKTASTE5)
      return 8;
   if (Tastaturwert < JOYSTICKTASTE6)
      return 3;
   if (Tastaturwert < JOYSTICKTASTE7)
      return 6;
   if (Tastaturwert < JOYSTICKTASTE8)
      return 9;
   if (Tastaturwert < JOYSTICKTASTE9)
      return 5;
   /*
    if (Tastaturwert < JOYSTICKTASTEL)
    return 10;
    if (Tastaturwert < JOYSTICKTASTE0)
    return 0;
    if (Tastaturwert < JOYSTICKTASTER)
    return 12;
    */
   return 0;
}
uint8_t Joystick_Tastenwahl_33_tastatur(uint16_t Tastaturwert)
{
   // return 0;
   if (Tastaturwert < JOYSTICKTASTE1)
      return 1;
   if (Tastaturwert < JOYSTICKTASTE2)
      return 2;
   if (Tastaturwert < JOYSTICKTASTE3)
      return 3;
   if (Tastaturwert < JOYSTICKTASTE4)
      return 4;
   if (Tastaturwert < JOYSTICKTASTE5)
      return 5;
   if (Tastaturwert < JOYSTICKTASTE6)
      return 6;
   if (Tastaturwert < JOYSTICKTASTE7)
      return 7;
   if (Tastaturwert < JOYSTICKTASTE8)
      return 8;
   if (Tastaturwert < JOYSTICKTASTE9)
      return 9;
   /*
    if (Tastaturwert < JOYSTICKTASTEL)
    return 10;
    if (Tastaturwert < JOYSTICKTASTE0)
    return 0;
    if (Tastaturwert < JOYSTICKTASTER)
    return 12;
    */
   return 0;
}

uint16_t readADC_A6()
{
   uint16_t result = 0;

   return result;
}

// https://forum.arduino.cc/t/ms5611-pressure-problem/543358/6
float getAltitude(float press, float temp)
{
   return ((pow((seaLevelPressure / press), 1.0 / 5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}






uint8_t Joystick_Tastenwahl_33_6(uint16_t Tastaturwert)
{
   // return 0;
   if (Tastaturwert < JOYSTICKTASTE1)
      return 2;
   if (Tastaturwert < JOYSTICKTASTE2)
      return 1;
   if (Tastaturwert < JOYSTICKTASTE3)
      return 4;
   if (Tastaturwert < JOYSTICKTASTE4)
      return 7;
   if (Tastaturwert < JOYSTICKTASTE5)
      return 8;
   if (Tastaturwert < JOYSTICKTASTE6)
      return 3;
   if (Tastaturwert < JOYSTICKTASTE7)
      return 6;
   if (Tastaturwert < JOYSTICKTASTE8)
      return 9;
   if (Tastaturwert < JOYSTICKTASTE9)
      return 5;
   /*
    if (Tastaturwert < JOYSTICKTASTEL)
    return 10;
    if (Tastaturwert < JOYSTICKTASTE0)
    return 0;
    if (Tastaturwert < JOYSTICKTASTER)
    return 12;
    */
   return 0;
}
// tastenwahl

uint16_t readTastatur(uint8_t kanal)
{
   uint16_t tastenwertsumme = 0;
   tastaturwertarray[mittelposition++] = analogRead(kanal);
   for (uint8_t i = 0; i < ANZ_REP; i++)
   {
      tastenwertsumme += tastaturwertarray[i];
   }
   if (mittelposition >= ANZ_REP)
   {
      mittelposition = 0;
   }
   return tastenwertsumme / ANZ_REP;
}

void tastenfunktion_debounce(uint16_t wert)
{
}

void tastenfunktion(uint16_t Tastenwert)
{
   tastaturcounter++;
   //();

   if (Tastenwert > 10) // ca Minimalwert der Matrix
   {
      // OSZIA_LO();

      if (tastaturcounter >= 40) //   Prellen
      {
         Serial.print("Tastenwert:\t");
         Serial.print(Tastenwert);
         Serial.print("\n");
         if (anzeigestatus & ANZEIGE_TAST)
         {
            // Serial.print("Tastenwert anz:\t");
            // Serial.print(Tastenwert);
            // Serial.print("\t");
            // Serial.print(tastaturcounter);
            // tastaturcounter = 0;
            // Serial.print("\n");
            // return;
         }
         tastaturcounter = 0x00;
         // Serial.println("Taste down");
         if (!(tastaturstatus & (1 << TASTE_OK))) // Taste noch nicht gedrueckt
         {
            Serial.println("Taste down");
            ////Serial.println(Tastenwert);
            // Taste = 0;

            // tastaturstatus |= (1<<TASTE_ON); // nur einmal
            tastaturstatus |= (1 << TASTE_OK); // nur einmal

            {
               Taste = Joystick_Tastenwahl_33_2(Tastenwert);
            }

            Serial.print("Tastenwert: ");
            Serial.print(Tastenwert);
            Serial.print("\t Taste: ");
            Serial.print(Taste);
            Serial.print("\n");

            tastaturstatus |= (1 << AKTION_OK);
            if (OLED && Taste) // Taste und Tastenwert anzeigen
            {
               /*
                oled_delete(0,62,20);
                u8g2.setCursor(0,62);
                u8g2.print(tastaturwert);
                u8g2.print(" T ");
                u8g2.setCursor(40,62);
                u8g2.print(Taste);

                u8g2.sendBuffer();
                */
            }

            //;
         }
         else // Taste neu gedrückt
         {
            /*
             Taste = 0;
             //tastaturstatus |= (1<<TASTE_ON); // nur einmal

             Taste= Joystick_Tastenwahl(Tastenwert);
             tastaturstatus |= (1<<AKTION_OK);
             if(OLED && Taste) // Taste und Tastenwert anzeigen
             {
             oled_delete(0,62,40);
             u8g2.setCursor(0,62);
             //u8g2.print(tastaturwert);
             u8g2.print("T ");
             u8g2.print(Taste);

             u8g2.sendBuffer();

             }
             */
         }
      }

   } // if tastenwert
   else
   {
      // if (tastaturstatus & (1<<TASTE_ON))
      {

         // tastaturstatus &= ~(1<<TASTE_OK);
      }
   }
   // OSZIA_HI();

} // tastenfunktion

void setModus(void)
{
   switch (curr_modus)
   {
   case MODELL:
   {
      eepromread();
   }
   break;

   case SIM:
   {
      // Joystick-Settings auf neutral stellen
      for (uint8_t i = 0; i < NUM_SERVOS; i++)
      {
         ////Serial.print(adcpinarray[i]);
         ////Serial.print("\t");
         ////Serial.print(servomittearray[i]);
         ////Serial.print("\t");

         kanalsettingarray[0][i][1] = 0x00; // level
         kanalsettingarray[0][i][2] = 0x00; // expo
      }
   }
   break;

   case CALIB:
   {
   }
   break;
   } // switch curr_steuerstatus
}

void setCalib(void)
{
}



void setup()
{
    anzeigestatus = ANZEIGE_SLAVE;

   uint8_t ee[16];
   delay(50);
   for (uint8_t i = 0; i < 64; i++)
   {
      // ee[i] = EEPROM.read(i);
      // EEPROM.write(i,0);
   }
   // https://wolles-elektronikkiste.de/arduino-nano-every-ein-deep-dive#adc_module
   /*
    PORTD.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; // Disable digital buffer
    ADC0.CTRLA = ADC_ENABLE_bm; // Enable ADC
    ADC0.CTRLC = ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV16_gc; // use internal reference / prescaler: 16
    VREF.CTRLA = VREF_ADC0REFSEL_4V34_gc; // use internal 4.34 V reference
    ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc; // use A2 (PD1) as input
    ADC0.SAMPCTRL = 0;
    */
   delay(50);

   pinMode(OSZIA_PIN, OUTPUT);

   Serial.begin(9600);

   // PPM decode, von RC_22
   pinMode(PPM_DIR_PIN, OUTPUT);
   pinMode(PPM_DATA_PIN, OUTPUT);
   digitalWrite(PPM_DATA_PIN, LOW);

   


   for (int i = 0; i < 4; i++)
   {
      pinMode(adcpinarrayTeensy[i], INPUT);
   }

    attachInterrupt(digitalPinToInterrupt(PPM_DATA_PIN), slaveISR, RISING);

   pinMode(BUZZPIN, OUTPUT);
   digitalWrite(BUZZPIN, LOW);

   curr_steuerstatus = MODELL;
   // savestatus = 0xFF;

   delay(100);
   /*
    for (uint8_t i=0;i<16;i++)
    {
    ee[i] = EEPROM.read(i);
    //Serial.print(" i: ");
    //Serial.print(i);
    //Serial.print(" ee: *");
    //Serial.print(ee[i]);

    }
    //Serial.print("\n");
    */
   // Serial.println(__DATE__);
   // Serial.println(__TIME__);

   pinMode(PPM_DATA_PIN, INPUT_PULLDOWN);
   attachInterrupt(digitalPinToInterrupt(PPM_DATA_PIN), slaveISR, RISING);

   printeeprom(160);

   eepromread();

   pinMode(BUZZPIN, OUTPUT);

   pinMode(LOOPLED, OUTPUT);

   pinMode(BATT_PIN, INPUT);

   pinMode(TASTATUR_PIN, INPUT);

   // pinMode(EEPROMTASTE,INPUT_PULLUP);
   eepromtaste.attach(EEPROMTASTE, INPUT_PULLUP);
   eepromtaste.interval(5);
   eepromtaste.setPressedState(LOW);

   // digitalWrite(EEPROMTASTE, HIGH);

   // https://registry.platformio.org/libraries/adafruit/Adafruit%20LiquidCrystal/installation
   // set up the LCD's number of rows and columns:
   // lcd.begin(20, 4);
   // Print a message to the LCD.
   // lcd.print("hello, world!");

   // OLED

   // 0.96"

   initDisplay();

   oled_vertikalbalken(BATTX, BATTY, BATTB, BATTH);

   setHomeScreen();

   // u8g2.sendBuffer();

   //                Configure the NRF24 module  | NRF24 modül konfigürasyonu
   radio.begin();

   radio.openWritingPipe(pipeOut);

   radio.setChannel(124);
   radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available

   radio.setPALevel(RF24_PA_MAX); // Output power is set for maximum range  |  Çıkış gücü maksimum menzil için ayarlanıyor.

   radio.setPALevel(RF24_PA_MIN);
   radio.setPALevel(RF24_PA_MAX);
   radio.enableAckPayload();
   radio.setRetries(0, 0);
   radio.stopListening(); // Start the radio comunication for Transmitter | Verici için sinyal iletişimini başlatır.
   if (radio.failureDetected)
   {
      radio.failureDetected = false;
      delay(250);
      Serial.println("Radio failure detected, restarting radio");
   }
   else
   {
      Serial.println("Radio OK");
   }
   // Serial.println("printDetails:");
   // radio.printDetails();

   ResetData();

   
   

   Serial.print("servomitte\n");
   for (uint8_t i = 0; i < NUM_SERVOS; i++)
   {
      uint16_t wert = 500 + i * 50;
      wert = 750;
      //impulstimearray[i] = wert; // mittelwert

      servomittearray[i] = analogRead(adcpinarrayTeensy[i]);

      //potgrenzearray[i][0] = servomittearray[i];
      //potgrenzearray[i][1] = servomittearray[i];

      /*
      if(i == THROTTLE)
      {
         servomittearray[i] = 127;
      }
      */
      // servomittearray[i] = analogRead(adcpinarrayA[i]);
      Serial.print("i:\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.print(servomittearray[i]);
      // Serial.print("\t lo: ");
      // Serial.print(servomittearray[i] & 0x00FF);
      // Serial.print("\t hi: ");
      // Serial.print((servomittearray[i]& 0xFF00) >> 8);
      Serial.print("\n");

      // uint8_t n = i*i+1;
      // EEPROM.write(i,0 );

   } // for NUM_SERVOS

   // Timer starten
 

   setupDebounce();

   // Serial.print("\n");
   //  for (uint8_t i=0;i<NUM_SERVOS;i++)
   {
      // Serial.print(adcpinarray[i]);
      // Serial.print("\t");
      // Serial.print(servomittearray[i]);
      // Serial.print("\t");

      // kanalsettingarray[0][i][1] = 0x11; // level
      // kanalsettingarray[0][i][2] = 0x33; // expo
   }

   // Serial.print("\n");

} // setup

int Throttle_Map(int val, int fromlow, int fromhigh, int tolow, int tohigh, bool reverse)
{
   val = constrain(val, fromlow, fromhigh);
   val = map(val, fromlow, fromhigh, tolow, tohigh);
   return (reverse ? 255 - val : val);
}

int Throttle_Map255(int val, int fromlow, int fromhigh, int tolow, int tohigh, bool reverse)
{
   val = constrain(val, fromlow, fromhigh);
   val = map(val, fromlow, fromhigh, tolow, tohigh);

   uint8_t levelwerta = levelwertarray[THROTTLE] & 0x07;
   uint8_t levelwertb = (levelwertarray[THROTTLE] & 0x70) >> 4;

   uint8_t expowerta = expowertarray[THROTTLE] & 0x07;

   uint16_t expoint = 3;
   uint16_t levelint = 0;

   expoint = expoarray8[expowerta][val];
   levelint = expoint * (8 - levelwerta);
   levelint /= 4;

   return (reverse ? 255 - levelint : levelint);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Joystick center and its borders
int Border_Map(int val, int lower, int middle, int upper, bool reverse)
{
   val = constrain(val, lower, upper);
   if (val < middle)
      val = map(val, lower, middle, 0, 128);
   else
      val = map(val, middle, upper, 128, 255);
   return (reverse ? 255 - val : val);
}

// Joystick center and its borders
int Border_Map10(int val, int lower, int middle, int upper, bool reverse)
{
   val = constrain(val, lower, upper);
   if (val < middle)
      val = map(val, lower, middle, 0, 254); // normieren auf 0-254

   else
      val = map(val, middle, upper, 255, 512); // normieren auf 255 - 512
   return (reverse ? 512 - val : val);
}

int Border_Mapvar255_raw( int val, int lower, int middle, int upper, bool reverse)
{
   val = constrain(val, lower, upper); // Grenzen einhalten
   uint8_t levelwerta = 0; //levelwertarray[servo] & 0x07;
   uint8_t levelwertb = 0; //(levelwertarray[servo] & 0x70) >> 4;

   uint8_t expowerta = 0; //expowertarray[servo] & 0x07;
   uint8_t expowertb = 0; //(expowertarray[servo] & 0x70) >> 4;

   // levelwerta = 0;
   // levelwertb = 0;
   // expowerta = 0;
   // expowertb = 0;

   if (val < middle)
   {

      val = map(val, lower, middle, 0, 127); // normieren auf 0-127
      // intdiff = val;
      intdiff = (127 - val); // Abweichung von mitte,
      // levelintraw = intdiff;
      // diffa = map(intdiff,0,(middle - lower), 0,512);
      diffa = intdiff;

      expoint = expoarray8[expowerta][diffa];
      levelint = expoint * (8 - levelwerta);
      levelint /= 8;
      levelintcheck = 127 + levelint;
      levelint = 127 + levelint;
   }
   else
   {
      val = map(val, middle, upper, 128, 255); // normieren auf 128 - 255
      // intdiff = val;

      intdiff = (val - 127); // Abweichung von mitte,
      // diffb = map(intdiff,0,(upper - middle),0,512);
      diffb = intdiff;
      if (diffb >= 127)
      {
         diffb = 127;
      }
      expoint = expoarray8[expowertb][diffb];
      levelint = expoint * (8 - levelwertb);
      levelint /= 8;
      levelintcheck = 127 - levelint;
      levelint = 127 - levelint;
   }

   return (reverse ? 255 - levelint : levelint);
}

int Border_Mapvar255(uint8_t servo, int val, int lower, int middle, int upper, bool reverse)
{
   val = constrain(val, lower, upper); // Grenzen einhalten
   uint8_t levelwerta = levelwertarray[servo] & 0x07;
   uint8_t levelwertb = (levelwertarray[servo] & 0x70) >> 4;

   uint8_t expowerta = expowertarray[servo] & 0x07;
   uint8_t expowertb = (expowertarray[servo] & 0x70) >> 4;

   // levelwerta = 0;
   // levelwertb = 0;
   // expowerta = 0;
   // expowertb = 0;

   if (val < middle)
   {

      val = map(val, lower, middle, 0, 127); // normieren auf 0-127
      // intdiff = val;
      intdiff = (127 - val); // Abweichung von mitte,
      // levelintraw = intdiff;
      // diffa = map(intdiff,0,(middle - lower), 0,512);
      diffa = intdiff;

      expoint = expoarray8[expowerta][diffa];
      levelint = expoint * (8 - levelwerta);
      levelint /= 8;
      levelintcheck = 127 + levelint;
      levelint = 127 + levelint;
   }
   else
   {
      val = map(val, middle, upper, 128, 255); // normieren auf 128 - 255
      // intdiff = val;

      intdiff = (val - 127); // Abweichung von mitte,
      // diffb = map(intdiff,0,(upper - middle),0,512);
      diffb = intdiff;
      if (diffb >= 127)
      {
         diffb = 127;
      }
      expoint = expoarray8[expowertb][diffb];
      levelint = expoint * (8 - levelwertb);
      levelint /= 8;
      levelintcheck = 127 - levelint;
      levelint = 127 - levelint;
   }

   return (reverse ? 255 - levelint : levelint);
}

int Border_Mapvar255_Throttle(uint8_t servo, int val, int lower, int upper, bool reverse)
{
   val = constrain(val, lower, upper);   // Grenzen einhalten
   val = map(val, lower, upper, 0, 255); // normieren auf 0 - 255
   uint8_t levelwerta = levelwertarray[servo] & 0x07;
   uint8_t levelwertb = (levelwertarray[servo] & 0x70) >> 4;

   uint8_t expowerta = expowertarray[servo] & 0x07;
   uint8_t expowertb = (expowertarray[servo] & 0x70) >> 4;

   intdiff = val;
   expoint = expoarray8[expowertb][intdiff / 2]; // nur 127 Werte in expoarray
   levelint = expoint * (8 - levelwertb);
   levelint /= 8; //

   return 2 * (reverse ? 255 - levelint : levelint);
}

uint16_t map_uint16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
   if (in_max == in_min)
      return out_min; // prevent division by zero
   return (uint16_t)(((uint32_t)(x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min);
}

double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
   if (in_max == in_min)
      return out_min;
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t testwert = 0;

void loop()
{
   //
   loopcounter0++;
   // digitalWrite(BUZZPIN,!(digitalRead(BUZZPIN)));
   // tastaturwert = analogRead(TASTATUR_PIN)/2;
   // tastaturwert = readTastatur(TASTATUR_PIN);
   // tastaturwert = readTastatur(TASTATUR_PIN)/2;
   if (sincelasttastatur > 20)
   {
      // OSZIA_LO();

      sincelasttastatur = 0;

      tastaturwert = analogRead(TASTATUR_PIN) / 2;
      tastenfunktion(tastaturwert);
   }

   if (zeitintervall > 500)
   {
      zeitintervall = 0;
      digitalWrite(LOOPLED, !digitalRead(LOOPLED));

      sekundencounter++;
      if (sekundencounter % 2)
      {

         throttlecounter += (data.throttle);
         throttlesekunden = throttlecounter >> 8;
         blinkstatus = 1;
         if (throttlesekunden > 250)
         {
            tone(BUZZPIN, 1000);
         }

         stopsekunde++;
         if (stopsekunde % 2 == 0)
         {
            // tone(BUZZPIN,1000);
         }
         if (stopsekunde == 60)
         {
            stopsekunde = 0;
            stopminute++;
         }
         refreshScreen();
      }
      else
      {
         blinkstatus = 0;
         noTone(BUZZPIN);
      }
      if (curr_screen == 5)
      {
         updateModusScreen();
         u8g2.sendBuffer();
      }
      updateHomeScreen();
   } // zeitintervall > 500

   // Tastatur

   if (tastaturstatus & (1 << TASTE_OK) && Taste) // Menu ansteuern
   {
      tastaturcounter = 0;
      switch (Taste)
      {
      case 0: // null-pos, nichts tun
      {
      }
      break;
      case 1:
      {
         // Serial.print("T 1");
         switch (curr_screen)
         {
         case 1: // MENUSCREEN
         {
            curr_screen = 5;
            curr_cursorspalte = 0;
            setModusScreen();
            u8g2.sendBuffer();
         }
         break;
         } // switch curr_screen
      }
      break;

      case 2: // UP
      {
         ////Serial.print("T 2");
         if (tastaturstatus & (1 << AKTION_OK))
         {
            // Serial.print("T 2 up*");
            tastaturstatus &= ~(1 << AKTION_OK);
            tastaturstatus |= (1 << UPDATE_OK);
            switch (curr_screen)
            {
            case 0: // HOMESCREEN
            {
            }
            break;
            case 1: // T2 MENUSCREEN
            {
               if (curr_model)
               {
                  curr_model--;
                  updateMenuScreen();
                  u8g2.sendBuffer();
               }
            }
            case 2: // T2 MODELLSCREEN
            {
               if (curr_funktion)
               {
                  curr_funktion--;
                  updateModellScreen();
                  u8g2.sendBuffer();
               }
            }
            break;
            case 3: // FUNKTIONSCREEN
            {
               switch (curr_cursorspalte)
               {
               case 0:
               {
                  if (curr_aktion)
                  {
                     curr_aktion--;
                     updateFunktionScreen();
                     u8g2.sendBuffer();
                  }
               }
               break;
               case 1: // Level, expo up, down
               {
                  // Serial.print("curr_aktion: ");
                  // Serial.print(curr_aktion) ;
               }
               break;
               } // switch curr_cursorspalte
            }
            break;

            case 4: // T2 AKTIONSCREEN
            {
               switch (curr_cursorspalte)
               {
               case 0:
               {
                  if (curr_wert)
                  {
                     curr_wert--;
                  }
               }
               break;
               case 1: // T2 UP DOWN
               {
                  uint8_t level = kanalsettingarray[curr_model][curr_funktion][1];
                  uint8_t levelO = (level & 0xF0) >> 4;
                  uint8_t levelU = (level & 0x0F);

                  uint8_t expo = kanalsettingarray[curr_model][curr_funktion][2];
                  uint8_t expoO = (expo & 0xF0) >> 4;
                  uint8_t expoU = (expo & 0x0F);

                  switch (curr_aktion)
                  {
                  case 0: // LEVEL
                  {
                     switch (curr_wert)
                     {
                     case 0: // UP
                     {
                        if (levelO < 4)
                        {
                           levelO++;
                           savestatus = CHANGED;
                           ;
                        }
                     }
                     break;
                     case 1: // DOWN
                     {
                        if (levelU < 4)
                        {
                           levelU++;
                           savestatus = CHANGED;
                           ;
                        }
                     }
                     break;
                     } // switch curr_wert
                     level = (levelO << 4) | levelU;
                     kanalsettingarray[curr_model][curr_funktion][1] = level;
                  }
                  break;

                  case 1: // EXPO
                  {
                     switch (curr_wert)
                     {
                     case 0: // UP
                     {
                        if (expoO < 4)
                        {
                           expoO++;
                           savestatus = CHANGED;
                           ;
                        }
                     }
                     break;
                     case 1: // DOWN
                     {
                        if (expoU < 4)
                        {
                           expoU++;
                           savestatus = CHANGED;
                           ;
                        }
                     }
                     break;
                     }
                     expo = (expoO << 4) | expoU;
                     kanalsettingarray[curr_model][curr_funktion][2] = expo;
                  }
                  break;

                  } // switch curr_aktion
               }
               break;

               } // switch curr_cursorspalte
               updateAktionScreen();
               u8g2.sendBuffer();
            }
            break;

            case 5: // T2 UP  MODUSSCREEN
            {
               if (curr_modus)
               {

                  switch (curr_modus)
                  {
                  case MODELL:
                  {
                  }
                  break;

                  case SIM:
                  {
                  }
                  break;

                  case CALIB:
                  {
                     calibstatus &= ~(1 << CALIB_START);
                  }
                  break;
                  } // switch curr_modus
                  curr_modus--;
               }
            }
            break;
            } // switch (curr_screen)
         }
      }
      break;

      case 3:
      {
         // Serial.print("T 3");
         switch (curr_screen)
         {
         case 0: // HOMESCREEN
         {
            // EEPROM lesen
            eepromread();
            printeeprom(160);
         }
         break;

         } // switch curr_screen
      }
      break;

      case 4: // LEFT
      {
         ////Serial.print("T 4");
         if (tastaturstatus & (1 << AKTION_OK))
         {
            // Serial.print("T 4 left");
            tastaturstatus &= ~(1 << AKTION_OK);
            tastaturstatus |= (1 << UPDATE_OK);
            switch (curr_screen)
            {
            case 0: // HOMESCREEN // Umschalten Simulator/Modell, TO DO
            {
               // if (savestatus == CHANGED)
               {
                  if (curr_cursorspalte == 1)
                  {
                     curr_cursorspalte = 0; // Rahmen auf YES

                     updateHomeScreen();
                     u8g2.sendBuffer();
                  }
               }
            }
            break;
            case 1: // MENUSCREEN
            {
            }
            break;
            case 2: // MODELLSCREEN
            {
            }
            break;
            case 3: // FUNKTIONSCREEN
            {
               switch (curr_cursorspalte)
               {
               case 0:
               {
                  updateFunktionScreen();
                  u8g2.sendBuffer();
               }
               break;
               case 1: // up, down enabled
               {
                  curr_cursorspalte--;
               }
               break;
               } // switch curr_cursorspalte
            }
            break;

            case 4: // AKTIONSCREEN
            {
               // Serial.print("T4 case 4: curr_screen: ");
               // Serial.println(curr_screen);
               curr_cursorspalte = 0;
               curr_wert = 0;
               updateAktionScreen();
               u8g2.sendBuffer();
            }
            break;

            case 5: // T4 LEFT ModusScreen
            {
               curr_screen = 1; // MENUSCREEN
               setModus();
               setMenuScreen();
               updateMenuScreen();
               u8g2.sendBuffer();
            }
            break;
            } // swich curr_screen
         }
      }
      break;

      case 5: // Ebene tiefer
      {
         if (tastaturstatus & (1 << AKTION_OK))
         {
            // Serial.print("T 5 in ");

            if ((curr_screen == 0)) //&& (taste5counter < 3))
            {
               tastaturstatus |= (1 << T5_WAIT); // Warten auf 3 Impulse
               {
                  taste5counter++;
                  // Serial.print("taste5counter: ");
                  // Serial.println(taste5counter);
                  // tastaturcounter = 300; // Mehrfachklick ermoeglichen
                  if (taste5counter == 3)
                  {
                     curr_screen = 1;
                     taste5counter = 0;
                     tastaturstatus |= ~(1 << T5_WAIT); // Warten beendet
                     // Serial.print("T5 setMenuScreen ");
                     // u8g2.clear();

                     setMenuScreen();
                     u8g2.sendBuffer();
                  }
               }
            }
            else // if (!(tastaturstatus & (1<<T5_WAIT))) // kein Warten
            {
               taste5counter = 0;
               if (curr_screen < 6)
               {
                  // Serial.print("T 5 klick ");
                  // Serial.println(curr_screen);
                  switch (curr_screen)
                  {
                  case 1: // MODELLSCREEN
                  {
                     // Serial.print("T 5 > Modellscreen curr_model: ");
                     // Serial.println(curr_model);
                     setModellScreen();
                     curr_screen = 2;
                     u8g2.sendBuffer();
                  }
                  break;
                  case 2: // FUNKTIONSCREEN
                  {
                     // Serial.print("T 5 > FunktionScreen curr_funktion: " );
                     // Serial.println(curr_funktion);
                     setFunktionScreen();
                     curr_screen = 3;
                     u8g2.sendBuffer();
                  }
                  break;
                  case 3: // AKTIONSCREEN
                  {
                     // Serial.print("T 5 > AktionScreen curr_aktion: " );
                     // Serial.println(curr_aktion);
                     setAktionScreen();
                     curr_screen = 4;
                     u8g2.sendBuffer();
                  }
                  break;
                  case 4:
                  {
                     // Serial.print("T 5 screen 4 curr_wert: ");
                     // Serial.println(curr_wert);
                  }
                  break;

                  case 5: // MODUS-Screen
                  {
                     Serial.print("T 5 screen 5 curr_modus: ");
                     Serial.print(curr_modus);
                     Serial.print(" T 5 screen 5 calibstatus: ");
                     Serial.println(calibstatus);

                     if (!(calibstatus & (1 << CALIB_START))) // calib noch nicht gesetzt
                     {
                        cleargrenzen();
                        calibstatus |= (1 << CALIB_START);
                     }
                     else
                     {
                        calibstatus &= ~(1 << CALIB_START); // calib beenden

                        Serial.println(" vor write: ");
                         printeeprom(160);
                         eepromwrite();
                        Serial.println(" nach write: ");
                         printeeprom(160);
                     }

                     updateModusScreen();
                     u8g2.sendBuffer();

                     // setCalib();
                  }
                  break;

                  } // switch (curr_screen)
               }
            }
            // Serial.print("T5 end: curr_screen: ");
            // Serial.println(curr_screen);
            tastaturstatus &= ~(1 << AKTION_OK);
            tastaturstatus |= (1 << UPDATE_OK);
         }
      }
      break;

      case 6: // RIGHT
      {
         // Serial.print("T 6");
         if (tastaturstatus & (1 << AKTION_OK))
         {
            // Serial.println("T 6 right");
            tastaturstatus &= ~(1 << AKTION_OK);
            tastaturstatus |= (1 << UPDATE_OK);
            switch (curr_screen)
            {
            case 0: // HOMESCREEN // Umschalten Simulator/Modell, TO DO
            {
               if (savestatus == CHANGED)
               {
                  if (curr_cursorspalte == 0)
                  {
                     curr_cursorspalte = 1; // Rahmen auf NO
                     updateHomeScreen();
                     u8g2.sendBuffer();
                  }
               }
            }
            break;
            case 1: // MENUSCREEN , nach Modussrreen
            {
               {
                  setModusScreen();
                  curr_cursorzeile = 0;
                  curr_cursorspalte = 0;
                  curr_screen = 5; // MODUSSCREEN
                  u8g2.sendBuffer();
               }
            }
            break;
            case 2: // MODELLSCREEN
            {
            }
            break;
            case 3: // FUNKTIONSCREEN
            {
               // Serial.print("T6 case 3: curr_screen: ");
               // Serial.println(curr_screen);
               switch (curr_cursorspalte)
               {
               case 0:
               {
                  curr_cursorspalte++; // max 1
                  updateFunktionScreen();
                  u8g2.sendBuffer();
               }
               break;
               case 1: // up, down enabled
               {
                  switch (curr_aktion)
                  {
                  case 0: // Level
                  {
                     uint8_t level = kanalsettingarray[curr_model][curr_funktion][1];
                     uint8_t levelO = (level & 0xF0) >> 4;
                     uint8_t levelU = (level & 0x0F);
                     // if(curr_pfeil == PFEIL_UP)
                     {

                        // blink_cursorpos = 86<<8 | 22;
                        // u8g2.setDrawColor(1);
                        // u8g2.drawFrame(88,char_y,48,16);
                     }
                  }
                  break;
                  case 1: // expo
                  {
                     uint8_t expo = kanalsettingarray[curr_model][curr_funktion][2];
                     uint8_t expoO = (expo & 0xF0) >> 4;
                     uint8_t expoU = expo & 0x0F;
                  }
                  break;
                  } // switch curr_aktion
               }
               break;
               } // switch curr_cursorspalte
            }
            break;
            case 4: // AKTIONSCREEN
            {
               // Serial.print("T6 case 4: curr_screen: ");
               // Serial.println(curr_screen);
               switch (curr_cursorspalte)
               {
               case 0:
               {
                  curr_cursorspalte = 1;
               }
               break;
               } // switch
               updateAktionScreen();
               u8g2.sendBuffer();
            }
            break;

            case 5: // T6 ModusScreen
            {
               // Serial.print("T6 case 5 ModusScreen: curr_screen: ");
               // Serial.println(curr_screen);
               switch (curr_cursorzeile)
               {
               case 0: // Navigation
               {
                  if (curr_cursorspalte)
                  {
                     curr_cursorspalte++;

                     updateMenuScreen();
                     u8g2.sendBuffer();
                  }
               }
               break;
               case 1: // Auswahl
               {
                  curr_steuerstatus = SIM;
                  setModus();
               }
               break;
               } // switch curr_cursorzeile
            }
            break;

            } // swich curr_screen
         }
      }
      break;

      case 7:
      {
         if (tastaturstatus & (1 << AKTION_OK))
         {
            // Serial.print("T 7 back ");
            if (curr_screen)
            {
               curr_screen--;
               u8g2.clear();
               switch (curr_screen)
               {
               case 0: // HOMESCREEN
               {
                  // savestatus = CHANGED;
                  setHomeScreen();
               }
               break;
               case 1: // MENUSCREEN
               {
                  // setSaveScreen();
                  //   u8g2.sendBuffer();
                  setMenuScreen();
               }
               break;
               case 2: // MODELLSCREEN
               {

                  setModellScreen();
               }
               break;
               case 3: // FUNKTIONSCREEN
               {
                  curr_aktion = 0;
                  curr_cursorspalte = 0;
                  setFunktionScreen();
               }
               break;

               case 4: // check
               {
                  curr_screen = 0;
                  setHomeScreen();
               }
               break;
               case 5: // MODUSSCREEN
               {

                  setHomeScreen();
               }
               break;

               } // switch curr_screen
            }
            // Serial.print("T7 curr_screen: ");
            // Serial.println(curr_screen);
            u8g2.sendBuffer();
            tastaturstatus &= ~(1 << AKTION_OK);
            tastaturstatus |= (1 << UPDATE_OK);
            calibstatus &= ~(1 << CALIB_START);
         }
      }
      break;

      case 8:
      {
         // Serial.print("T 8");
         if (tastaturstatus & (1 << AKTION_OK))
         {
            // Serial.print("T 8 down");
            tastaturstatus &= ~(1 << AKTION_OK);
            tastaturstatus |= (1 << UPDATE_OK);
            switch (curr_screen)
            {
            case 0: // HOMESCREEN
            {
            }
            break;
            case 1: // MENUSCREEN Modelle
            {
               if (curr_model < 5)
               {
                  curr_model++;
                  updateMenuScreen();
                  u8g2.sendBuffer();
               }
            }
            break;

            case 2: // MODELLSCREEN  Funktionen
            {
               if (curr_funktion < 4)
               {
                  curr_funktion++;
                  updateModellScreen();
                  u8g2.sendBuffer();
               }
            }
            break;

            case 3: // FUNKTIONSCREEN
            {
               if (curr_aktion < 5)
               {
                  curr_aktion++;
                  updateFunktionScreen();
                  u8g2.sendBuffer();
               }
            }
            break;

            case 4: // T8 AKTIONSCREEN
            {
               switch (curr_cursorspalte)
               {
               case 0:
               {
                  if (curr_wert < 2)
                  {
                     curr_wert++;
                     updateAktionScreen();
                     u8g2.sendBuffer();
                  }
               }
               break;
               case 1: // T8 UP DOWN
               {
                  uint8_t level = kanalsettingarray[curr_model][curr_funktion][1];
                  uint8_t levelO = (level & 0xF0) >> 4;
                  uint8_t levelU = (level & 0x0F);

                  uint8_t expo = kanalsettingarray[curr_model][curr_funktion][2];
                  uint8_t expoO = (expo & 0xF0) >> 4;
                  uint8_t expoU = (expo & 0x0F);

                  switch (curr_aktion)
                  {
                  case 0: // LEVEL
                  {
                     switch (curr_wert)
                     {
                     case 0: // UP
                     {
                        if (levelO)
                        {
                           levelO--;
                           savestatus = CHANGED;
                           ;
                        }
                     }
                     break;
                     case 1: // DOWN
                     {
                        if (levelU)
                        {
                           levelU--;
                           savestatus = CHANGED;
                           ;
                        }
                     }
                     break;
                     } // switch curr_wert
                     level = (levelO << 4) | levelU;
                     kanalsettingarray[curr_model][curr_funktion][1] = level;
                  }
                  break;
                  case 1: // EXPO
                  {
                     switch (curr_wert)
                     {
                     case 0: // UP
                     {
                        if (expoO)
                        {
                           expoO--;
                           savestatus = CHANGED;
                           ;
                        }
                     }
                     break;
                     case 1: // DOWN
                     {
                        if (expoU)
                        {
                           expoU--;
                           savestatus = CHANGED;
                           ;
                        }
                     }
                     break;
                     } // switch curr_wert
                     expo = (expoO << 4) | expoU;
                     kanalsettingarray[curr_model][curr_funktion][2] = expo;
                  }
                  break;

                  } // switch curr_aktion
               }
               break;
               } // switch curr_cursorspalte
               updateAktionScreen();
               u8g2.sendBuffer();
            }
            break;

            case 5: // T8 DOWN MODUSSCREEN
            {

               if (curr_modus < 2)
               {
                  curr_modus++;
                  switch (curr_modus)
                  {
                  case MODELL:
                  {
                  }
                  break;

                  case SIM:
                  {
                  }
                  break;

                  case CALIB:
                  {
                     updateModusScreen();
                     u8g2.sendBuffer();
                  }
                  break;
                  } // switch curr_modus
               }
            }
            break;
            } // switch (curr_screen)
         }
      }
      break;

      case 9:
      {
         Serial.print("T 9 SAVE ");

         switch (curr_screen)
         {
         case 0:
         {
            Serial.print("T9 savestatus: ");
            Serial.print(savestatus);
            ////Serial.print(" curr_cursorspalte: ");
            // Serial.print(curr_cursorspalte);
            switch (savestatus)
            {
            case 2: // CHANGED
            {
               // write to eeprom
               Serial.println(" vor write: ");
               printeeprom(160);
               eepromwrite();
               Serial.println(" nach write: ");
               printeeprom(160);
               savestatus = CANCEL;
            }
            break;

            case 1: // CANCEL
            {
               // do nothing
               Serial.println(" CANCEL ");
               curr_cursorspalte = 0;
               savestatus = CANCEL;
            }
            break;

               // u8g2.sendBuffer();
            }
         }
         break;

         } // switch curr_screen
         updateHomeScreen();
         u8g2.sendBuffer();
      }
      break;
      } // switch (Taste)
      if (Taste)
      {
         ////Serial.print("\n");
         Taste = 0;
         tastaturstatus &= ~(1 << TASTE_OK);
      }

   } // if TASTE_OK

   // end Tastatur

   if (loopcounter0 >= BLINKRATE)
   {
      loopcounter0 = 0;
      loopcounter1++;
      if (loopcounter1 > BLINKRATE)
      {
         loopcounter1 = 0;

         if (TEST)
         {
            Serial.print("ACK erhalten: ");
            Serial.print("\t");
            Serial.print(ackData[0]);
            Serial.print("\t");
            Serial.print(ackData[1]);

            Serial.print("\t2\t");
            Serial.print(ackData[2]);
            Serial.print("\t3\t");
            Serial.print(ackData[3]);

            Serial.print(" \n");
         }
         switch (anzeigestatus)
         {
         case ANZEIGE_POT:
         {
            // Serial.print(i);
            // Serial.print(ppm[i]);
            Serial.print("\t");
            for (int i = 0; i < NUM_SERVOS; i++)
            {
               if (i == 0)
               {
                  Serial.print(i);
                  Serial.print("\t");
               }
               // Serial.print(i);
               // Serial.print(ppm[i]);
               // Serial.print("\t");
               Serial.print(potwertarray[i]);
               Serial.print("\t");

               // Serial.print(Border_Mapvar255(i, potwertarray[i],2000,1500,1000,false));
               // Serial.print(map( potwertarray[i],0,1024,2000,1000));
            }
            Serial.print("\tYAW lower\t");
            Serial.print(potgrenzearray[PITCH][1]);

            Serial.print("\tYAW mitte\t");
            Serial.print(servomittearray[PITCH]);

            Serial.print("\tYAW upper\t");
            Serial.print(potgrenzearray[PITCH][0]);

            Serial.print("\tYAW\t");
            Serial.print(data.yaw);

            Serial.print("\t\tPITCH\t");
            Serial.print(data.pitch);

            Serial.print("\tROLL\t");
            Serial.print(data.roll);
            Serial.print("\tTHROTTLE\t");
            Serial.print(data.throttle);

            Serial.print("\tA2\t");
            Serial.print(ackData[2]);
            Serial.print("\tA3\t");
            Serial.print(ackData[3]);
            // Serial.print("\t\t");
            Serial.print("\n");
         }
         break;

         case ANZEIGE_TAST:
         {
         }
         break;

         case ANZEIGE_SLAVE:
         {
            Serial.print("Slavechannelarray: \t");
            for(uint8_t i=0;i<NUM_SERVOS;i++)
            {
               Serial.print("\t");

               Serial.print(Slavechannelarray[i]);
            }
            Serial.print("\n");
         }
         break;

         case ANZEIGE_ADC:
         {
            Serial.print("\tbatteriespannung raw: ");
            Serial.print(batteriespannungraw);
            Serial.print("\tbatteriespannung: ");
            Serial.print(batteriespannung);
            Serial.print("\tUBatt: ");
            Serial.print(UBatt);
            Serial.print("\tbatterieanzeige: ");
            Serial.print(batterieanzeige);
            Serial.print("\t");

            Serial.print("\tack-Spannung: ");
            Serial.print(ackData[3]);
            Serial.print("\tflyerbatteriespannung: ");
            Serial.print(flyerbatteriespannung);
            Serial.print("\tUFlyerBatt: ");
            Serial.print(UFlyerBatt);
            Serial.print("\tflyerbatterieanzeige: ");
            Serial.print(flyerbatterieanzeige);

            Serial.print("\n");
         }
         break;

         case ANZEIGE_CALIB:
         {
            if (calibstatus & (1 << CALIB_START))
            {
               for (int i = 0; i < NUM_SERVOS; i++)
               {

                  Serial.print("L: ");

                  Serial.print(potgrenzearray[i][0]);
                  Serial.print("\t R: ");
                  Serial.print(potgrenzearray[i][1]);
                  Serial.print("\t Mitte: ");
                  Serial.print(servomittearray[i]);
                  Serial.print("\t potwert: ");
                  Serial.print(potwertarray[i]);
                  Serial.print("\t\t");
               }
               Serial.print("\n");
            }
            //
         }
         break;
         }
         /*
         for (int i=0;i<NUM_SERVOS;i++)
         {
         Serial.print("delay: \t");
         Serial.print(impulsdelayarray[i]);
         Serial.print(" \tCCPM: \t");
         Serial.print(impulsCCMParray[i]);

         Serial.print("\t rest: \t");
         Serial.print(restzeitarray[i]);
         Serial.print("\trestCCPM: \t");
         Serial.print(restCCM);
         Serial.print("\t");

         }
         Serial.print("\n");
         */
         if (Taste)
         {
            ////Serial.print(tastaturwert);
            ////Serial.print(" Taste: ");
            // Serial.print(Taste);
            // Serial.print("\n");
         }

         // Serial.print(ackData[0]);
         // Serial.print("\t");
         // Serial.print("debouncecheck: ");
         // Serial.print(debouncecheck);
         // Serial.print("\n");

         blinkcounter++;
         impulscounter += 16;
         // digitalWrite(LOOPLED, ! digitalRead(LOOPLED));
         float faktor = 0.1;
         // analogWrite(BUZZPIN,127);

         // batteriespannung = readADC_A6();
         batteriespannungraw = (float)analogRead(A9);

         if (batteriespannung == 0)
         {
            batteriespannung = batteriespannungraw;
         }
         else
         {
            batteriespannung = batteriespannung + faktor * (batteriespannungraw - batteriespannung);
         }
         UBatt = (batteriespannung) / 154;

         // batteriespannung = fmap(batteriespannung,60.0,900.0,0,44.0);
         // batteriespannung = analogRead(A6);
         //      batteriespannung = analogRead(A1);

         flyerbatteriespannungraw = float(ackData[3]);

         if (flyerbatteriespannung == 0)
         {
            flyerbatteriespannung = flyerbatteriespannungraw;
         }
         else
         {
            flyerbatteriespannung = flyerbatteriespannung + faktor * (flyerbatteriespannungraw - flyerbatteriespannung);
         }
         flyerbatteriespannung = constrain(flyerbatteriespannung, 60, 240);
         UFlyerBatt = fmap(flyerbatteriespannung, 60.0, 240.0, 0, 44.0);

         flyerbatterieanzeige = (uint16_t)UFlyerBatt;

         /*
         Serial.print(batteriemittel);
         Serial.print("\t");
         Serial.print(batteriespannung);
         Serial.print(" *\t ");
         Serial.print(data.pitch);

         Serial.print("\n");
         */

         if (TEST)
         {
            /*
            //Serial.print("potwertarray[YAW]: ");
            //Serial.print("\t ");
            //Serial.print(potwertarray[0]);
            //Serial.print("\t ");
            //Serial.print(potwertyaw);
            //Serial.print("\t ");

            //Serial.print("levelwertayaw: ");
            //Serial.print("\t ");
            //Serial.print(levelwertayaw);
            //Serial.print("\t ");
            //Serial.print("levelwertbyaw: ");
            //Serial.print("\t ");
            //Serial.print("data.yaw: ");
            //Serial.print("\t ");
            //Serial.print(data.yaw );
            //Serial.print("\n");
            */
         }
         // eepromread();
         ///*
         // u8g2.clearBuffer();                   // Clear display.
         // u8x8.setFont(u8g2_font_ncenB08_tr);    // choose a suitable font
         // u8x8.drawString(0, 24, "Hello OLED!");    // write something to the buffer
         // u8x8.sendBuffer();
         // u8g2.sendBuffer(); // Transfer buffer to screen.
         //*/
         /*
         //Serial.print("blinkcounter: ");
         //Serial.print(blinkcounter);

         //Serial.print(" radiocounter: ");
         //Serial.print(radiocounter);
         //Serial.print(" errcounter: ");
         //Serial.print(errcounter);
         ////Serial.print(" impulscounter: ");
         ////Serial.print(impulscounter);
         */
         /*
         potwert += schritt;
         if (potwert >= POTHI)
         {
         potwert = POTLO;
         }
         */
         ////Serial.print(" M: ");
         // if(abs(servomittearray[ROLL] - potwertarray[ROLL]) > 2)

         // 0.96
         loopcounter1++;
         uint8_t charindex = loopcounter1 & 0x7F;
         // u8g2.setDrawColor(0);
         // charh = u8g2.getMaxCharHeight() ;
         // oled_delete(4,44,64);

         // u8g2.drawGlyph(32,44,'A'+(charindex));
         char buf0[4];
         /*

         // Yaw
         //u8g2.setCursor(4,30);
         //u8g2.print(data.yaw);
         sprintf(buf0, "%3d", data.yaw);
         u8g2.drawStr(4,30,buf0);

         // Pitch
         // u8g2.setCursor(36,30);
         // u8g2.print(data.pitch);
         sprintf(buf0, "%3d", data.pitch);
         u8g2.drawStr(32,30,buf0);

         // Roll
         //u8g2.setCursor(4,46);
         //u8g2.print(data.roll);
         sprintf(buf0, "%3d", data.roll);
         u8g2.drawStr(4,42,buf0);

         // Throttle
         //u8g2.setCursor(36,46);
         //u8g2.print(data.throttle);
         sprintf(buf0, "%3d", data.throttle);
         u8g2.drawStr(32,42,buf0);
         */

         // uint8_t wertv = map(data.pitch,0,255,2,balkenvh-2); // Platz fuer 3 pixel dicke
         // oled_vertikalbalken_setwert(VBX,VBY,balkenvb,balkenvh,wertv);

         // uint8_t werth = map(data.yaw,0,255,2,balkenhb-2); // Platz fuer 3 pixel dicke

         // oled_horizontalbalken_setwert(HBX,HBY,balkenhb,balkenhh,werth);

         // batterieanzeige = (0x50*batteriespannung)/0x6B/8; // resp. /107
         batterieanzeige = (0x50 * batteriespannung) / 0x9A / 8; // integer-operation, resp. /154 als float

         /*
         Serial.print(batteriespannung);
         Serial.print("\t");
         Serial.print(batterieanzeige);
         Serial.print("\t");
         Serial.println(UBatt);
         */

         if (curr_screen == 0)
         {
            updateHomeScreen();
            u8g2.sendBuffer();
         }

         if (loopcounter1 > 25)
         {
            //loopcounter1 = 0;
         }

         if (TEST == 1)
         {
            ////Serial.print("YAW\t ");
            ////Serial.print(potgrenzearray[YAW][0]);
            ////Serial.print("\t ");
            ////Serial.print(potgrenzearray[YAW][1]);
            ////Serial.print("\t* ");
            ////Serial.print("pw2: ");
            ////Serial.print("\t ");
            ////Serial.print(potwertarray[YAW]);
            ////Serial.print("\t ");

            // //Serial.print("map: ");
            ////Serial.print("\t ");
            // uint8_t yawmap = map(potwertarray[YAW],0,680,0,254);
            ////Serial.print(yawmap);
            ////Serial.print(" *255*\t ");
            // uint16_t yawmap2 = Border_Map(potwertarray[YAW],potgrenzearray[YAW][1],servomittearray[YAW],potgrenzearray[YAW][0],true);
            ////Serial.print("\t ");
            ////Serial.print(yawmap2);

            ////Serial.print(" *512*\t ");
            // uint16_t yawmap3 = Border_Map10(potwertarray[YAW],potgrenzearray[YAW][1],servomittearray[YAW],potgrenzearray[YAW][0],true);
            ////Serial.print("\t ");
            ////Serial.print(yawmap3);

            ////Serial.print("\t ");
            ////Serial.print("pwarray: ");
            ////Serial.print("\t ");
            ////Serial.print(potwertarray[YAW]);

            //  int var = Border_Mapvar255(potwertarray[YAW],potgrenzearray[YAW][1],servomittearray[YAW],potgrenzearray[YAW][0],true);

            /*
            //Serial.print("\t ");
            //Serial.print("intdiff: ");
            //Serial.print("\t ");
            //Serial.print(intdiff);

            //Serial.print("\t ");
            //Serial.print("levelintcheck: ");
            //Serial.print("\t ");
            //Serial.print(levelintcheck);


            //Serial.print("\t ");
            //Serial.print("data.yaw: ");
            //Serial.print("\t ");
            //Serial.print(data.yaw);
            //Serial.print("\t ");
            //Serial.print(batteriespannung);

            //Serial.print("\t ");

            //Serial.print(UBatt,2);
            */

            ////Serial.print("\t ");
            ////Serial.print("levelintpitcha: ");
            ////Serial.print(levelintpitcha);

            // Serial.print(" *\n");
         } // if TEST 1

         if (calibstatus & (1 < CALIB_START))
         {
         }

         ////Serial.print(" throttlemitte: ");
         ////Serial.print(servomittearray[THROTTLE]);

         ////Serial.print(" throttlecounter: ");
         ////Serial.print(throttlecounter);

         /*
         //Serial.print(" A1: ");
         //Serial.print(potwertarray[PITCH]);
         //Serial.print(" A3: ");
         //Serial.print(potwertarray[ROLL]);
         //Serial.print(" A6: ");
         //Serial.print(potwertarray[THROTTLE]);
         */

         ////Serial.print(" *\n");
      }
   } // BLINKRATE
   // EEPROM
   eepromtaste.update();

   if (UBatt < 4.07)
   {
      // digitalWrite(BUZZPIN,!(digitalRead(BUZZPIN)));
   }

   if (paketcounter > 20) // 20ms
   {
      paketcounter = 0;

      // pot lesen
      for (uint8_t i = 0; i < NUM_SERVOS; i++)
      {

         potwert = analogRead(adcpinarrayTeensy[i]);
         potwertarrayraw[i] = potwert;

         if(calibstatus & (1<<CALIB_START))
         {

            if (potwert >= potgrenzearray[i][0])
            {
               potgrenzearray[i][0] = potwert; // upper
               savestatus = CHANGED;
            }
            if (potwert < potgrenzearray[i][1])
            {
               potgrenzearray[i][1] = potwert; // lower
               savestatus = CHANGED;
            }
         }
         // potgrenzearray[0][0] = 17;
         // potgrenzearray[0][1] = 33;

         uint16_t mitte = servomittearray[i];
         uint8_t levelwert = kanalsettingarray[curr_model][i][1]; // element 1, levelarray
         levelwertarray[i] = kanalsettingarray[curr_model][i][1];
         // levelwert   faktor
         //    0             8/8
         //    1             7/8
         //    2             6/8
         //    3             5/8
         //    4             4/8

         // eventuell ungleiche werte

         // levelwerta = levelwert & 0x07;
         // levelwertb = (levelwert & 0x70)>>4;

         // expowert ev. ungleich fuer richtung
         expowert = kanalsettingarray[curr_model][i][2]; // element2, expoarray
         expowertarray[i] = kanalsettingarray[curr_model][i][2];
         // expowerta = expowert & 0x07;
         // expowertb = (expowert & 0x70)>>4;

         // map(value, fromLow, fromHigh, toLow, toHigh)

         if ((i == YAW) || (i == PITCH) || (i == ROLL))
         {
            potwertarray[i] = potwert;
         }
         else if (i == THROTTLE)
         {
            if (potwert < 50)
            {
               potwertarray[i] = 10; // Motor sicher abstellen
            }
            else
            {
               potwertarray[i] = potwert;
            }
         }

         if (i == 0)
         {
            potwertyaw = potwert;

            levelwertayaw = levelwertarray[YAW];
            levelwertbyaw = levelwertarray[YAW];
         }
      } // for i

      data.yaw = Border_Mapvar255(YAW, potwertarray[YAW], potgrenzearray[YAW][1], servomittearray[YAW], potgrenzearray[YAW][0], false);

      /*
      winkelcounter+= 2;

      // uint8_t delta = winkelcounter % 127;
      rampe = rampe + (2 * ramprichtung);
      if(rampe > 250)
      {
         ramprichtung = -1;
      }
      else if (rampe < 5)
      {
         ramprichtung = 1;
      }



      //float winkel = float(winkelcounter)/180.0 * 3.14;

      //Serial.print(winkelcounter);
      //Serial.print("\t");
      //Serial.print(winkel);
      //Serial.print("\t");


      //double sinfloat = 127.0 + 125.0*sin(winkel/2);
      //Serial.print(sinfloat);
      //Serial.print("\t");
      */
      data.pitch = Border_Mapvar255(PITCH, potwertarray[PITCH], potgrenzearray[PITCH][1], servomittearray[PITCH], potgrenzearray[PITCH][0], false);
      // data.pitch = int(sinfloat);

      if (RAMPETEST)
      {
         data.pitch = rampe;
      }

      // Serial.println(data.pitch);
      // data.pitch = servomittearray[PITCH] +

      // Serial.println(data.yaw);
      // if(curr_model == 0)
      if (!(calibstatus & (1 << CALIB_START)))
      {
         potgrenzearray[ROLL][0] = servomittearray[ROLL];
         potgrenzearray[ROLL][1] = servomittearray[ROLL];
      }

      data.roll = Border_Mapvar255(ROLL, potwertarray[ROLL], potgrenzearray[ROLL][1], servomittearray[ROLL], potgrenzearray[ROLL][0], false);

      // uint16_t throttlemitte = servomittearray[THROTTLE];
      // data.throttle = Throttle_Map(potwertarray[THROTTLE],throttlemitte, POTHI,0,255, false );
      // data.throttle = Throttle_Map255(potwertarray[THROTTLE],servomittearray[THROTTLE], potgrenzearray[throttle][0],10,240, false ); // nur eine haelfte

      data.throttle = Border_Mapvar255_Throttle(THROTTLE, potwertarray[THROTTLE], potgrenzearray[THROTTLE][1], potgrenzearray[THROTTLE][0], false);

      // data.yaw = 13;
      // data.pitch = int(sinfloat);
      // data.roll = 14;
      // data.throttle = 15;

      // data.throttle = Border_Map(potwertarray[THROTTLE],0, 340,570, false );      // Potentiometer

      data.aux1 = digitalRead(5); // CH5
      data.aux2 = digitalRead(7); // CH6
      OSZIA_LO();
      if (radio.write(&data, sizeof(data)))
      {
         radiocounter++;

         // ********************
         // ACK Payload ********
         if (radio.isAckPayloadAvailable())
         {
            radio.read(&ackData, sizeof(ackData));
            temperaturint = ackData[0] * 2;
            float temperaturfloat = temperaturint;
            pressureint = (ackData[1] << 8) | ackData[2];
            float pressurefloat = pressureint / 10;
            // altitude = getAltitude(pressurefloat,temperaturfloat);
            altitudeint = altitude;
            /*
             //Serial.print("ACK erhalten: ");
             //Serial.print("\t");
             Serial.print(ackData[0]);
             Serial.print("\t");
             Serial.print(ackData[1]);
              Serial.print("\t");
              Serial.print(ackData[2]);
             Serial.print("\t");
             Serial.print(ackData[3]);
            Serial.print(" \n");
            */
         }
         else
         {
            // Serial.println(F("Keine ACK-Daten erhalten"));
         }
         // ********************
         // ********************
      }
      else
      {
         //Serial.println("radio error\n");
         digitalWrite(BUZZPIN, !(digitalRead(BUZZPIN)));
         errcounter++;
      }
      OSZIA_HI();
   }
}
