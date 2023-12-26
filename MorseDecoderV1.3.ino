/*
 F4LAA : ESP32 Real Time Morse Decoder
 12/12/2023 : MorseDecoder V1.0
 
   Trouvé grace à la vidéo YouTube de G6EJD : https://www.youtube.com/watch?v=9OWl8zOHgls

   Le code ESP32 est disponible ici : https://github.com/G6EJD/ESP32-Morse-Decoder

   Carte : NodeMCU-32S

   Adaptation: 
     Utilisation de l'écran TFT 4" avec la librairie rapide TFT_eSPI
     https://github.com/Bodmer/TFT_eSPI
     ATTENTION :
       La définitions des Pins n'est pas dans ce programme, 
       ==> il faut adapter le fichier E:\Users\syst4\Documents\Arduino\libraries\TFT_eSPI-master\User_Setup.h de la librairie TFT_eSPI

 24/12/2023 : Modifications V1.0 ==> V1.2 :
   - Ajout d’un encodeur rotatif permettant de modifier manuellement les paramètres de l’Algo.
   - Recherche automatique de la meilleure fréquence à mesurer par l’Algo Goertzel, qui change d’un opérateur à l’autre.
   - Réglage automatique du niveau d’entrée grâce au potentiomètre numérique MCP41010 (10k, pilotable depuis le bus SPI)
     Le volume varie beaucoup d’un opérateur à l’autre, F5NWY arrivant S9+40 chez moi 😊

 25/12/2023 : Modifications V1.2 ==> V1.3 :
   - Intégration de l'Algo de F5BU basé sur la mesure des temps de dot / dash / silences

 =====================================================================================
 Morse Code Decoder using an OLED and basic microphone

 The MIT License (MIT) Copyright (c) 2017 by David Bird. 
 ### The formulation and calculation method of an IAQ - Internal Air Quality index ###
 ### The provision of a general purpose webserver ###
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files 
 (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
 publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to 
 permit persons to whom the Software is furnished to do so, subject to the following conditions:  
   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
   LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 See more at http://dsbird.org.uk 
 
 CW Decoder by Hjalmar Skovholm Hansen OZ1JHM  VER 1.01
 Feel free to change, copy or what ever you like but respect
 that license is http://www.gnu.org/copyleft/gpl.html
 Read more here http://en.wikipedia.org/wiki/Goertzel_algorithm 
 Adapted for the ESP32/ESP8266 by G6EJD  
*/

// F4LAA : Using TFT4 SPI display
#include "SPI.h"
#include "TFT_eSPI.h"

TFT_eSPI tft = TFT_eSPI();

// Algo F5BU : 
//#define TRACEF5BU
#define d_Tune 500*5      // pulse plus long = Tune
#define d_Para 15*5       // durée Parasite max
#define d_point_i 80*5    // Durée point initiale

int16_t d_point = d_point_i;   // Durée point

#define MaxT_CW 32        // taille du tableau des durées pulses - silences (avec marge pour non séparation 2 car)
#define MaxTem 16         // taille du tableau des espaces mots

int16_t T_CW[MaxT_CW];          // Tableau pulses - silences (marge pour non séparation 2 car)
int8_t i_CW = -1;               // indice pour tableau CW
int8_t Tem[MaxTem] = {0,0,0,0}; // Tableau espaces mots
int8_t i_Tem = 0;               // indice pour tableau Tem
// End of F5BU

// SPI Potentiometre
const int slaveSelectPin = 22; // CS 

#define POTMIDVALUE 128
uint8_t potVal = POTMIDVALUE;  // Middle value
uint8_t potCmd = 0x11; // =0b00010001 so set PotA value
void setVolume(uint8_t value) 
{ 
  digitalWrite(slaveSelectPin, LOW); 
  tft.spiwrite(potCmd);
  tft.spiwrite(255 - value);
  digitalWrite(slaveSelectPin, HIGH); 
  float pourcent = ((value / 255.00) * 100);
  tftDrawString(396, 280, String(pourcent, 0) + "%  ");
  potVal = value;
} 

float magnitude           = 0;;
int   magnitudelimit      = 100;
int   magnitudelimit_low  = 100;
int   realstate           = LOW;
int   realstatebefore     = LOW;
int   filteredstate       = LOW;
int   filteredstatebefore = LOW;

// Noise Blanker time which shall be computed so this is initial 
int nbTime = 6;  // ms noise blanker
int spaceDetector = 5;
int magReactivity = 6;

int starttimehigh;
int highduration;
int lasthighduration;
float hightimesavg = 0;
int startttimelow;
int lowduration;
int laststarttime = 0;
int minL = 5000;
int maxL = 0;
int minH = 5000;
int maxH = 0;
int moyF = 5;
float dMoyDot = 0;        // Durée moyenne des dot
float dMoyDash = 0;       // durée moyenne des dash
float dMoySpace = 0;      // durée moyenne des espaces

// Encodeur rotatif GND, VCC, SW, DT (B), CLK (A)
// (A) CLK pin GPIO8 , (B) DT pin GPIO7, SW pin GPIO6 
#include <Rotary.h>
#define rotEncA 25
#define rotEncB 26
#define rotEncSW 27
Rotary rot= Rotary(rotEncA, rotEncB);
int rotCounter = 0;
int rotAState;
int rotALastState;
int rotSWState = 0;
int rotSWLastState = 0;
// EndOf Rotary variables definition

#define bufSize 8
char CodeBuffer[bufSize]; // 6 . ou - + 1 en trop (avant sécurité) + \0
#define nbChars 33
char DisplayLine[nbChars];
int iRow = 0;
int iCar = 0;
int  stop = LOW;
int  wpm;
int  sWpm;

char DisplayLineF5BU[nbChars + 1];
String Add = "";

void tftDrawString(int x, int y, String s)
{
  tft.setCursor(x, y);
  tft.println(s);
}

void clearDisplayLine()
{
  for (int i = 0; i < nbChars; i++) DisplayLine[i] = ' ';
}

void clearCodeBuffer()
{
  CodeBuffer[0] = '\0';
  for (int i = 1; i < bufSize; i++) CodeBuffer[i] = ' ';
};

bool graph = false; // To draw magnitude curve

int cptCharPrinted = 0;
bool CRRequested = false;
char lastChar = '{';
char curChar = '{';
void CodeToChar() { // translate cw code to ascii character//
  // F5BU
  Decode_Char();
  i_CW = -1;
  // End of F5BU
  char decode_char = '{';
  if (strcmp(CodeBuffer,".-") == 0)      decode_char = char('a');
  if (strcmp(CodeBuffer,"-...") == 0)    decode_char = char('b');
  if (strcmp(CodeBuffer,"-.-.") == 0)    decode_char = char('c');
  if (strcmp(CodeBuffer,"-..") == 0)     decode_char = char('d'); 
  if (strcmp(CodeBuffer,".") == 0)       decode_char = char('e'); 
  if (strcmp(CodeBuffer,"..-.") == 0)    decode_char = char('f'); 
  if (strcmp(CodeBuffer,"--.") == 0)     decode_char = char('g'); 
  if (strcmp(CodeBuffer,"....") == 0)    decode_char = char('h'); 
  if (strcmp(CodeBuffer,"..") == 0)      decode_char = char('i');
  if (strcmp(CodeBuffer,".---") == 0)    decode_char = char('j');
  if (strcmp(CodeBuffer,"-.-") == 0)     decode_char = char('k'); 
  if (strcmp(CodeBuffer,".-..") == 0)    decode_char = char('l'); 
  if (strcmp(CodeBuffer,"--") == 0)      decode_char = char('m'); 
  if (strcmp(CodeBuffer,"-.") == 0)      decode_char = char('n'); 
  if (strcmp(CodeBuffer,"---") == 0)     decode_char = char('o'); 
  if (strcmp(CodeBuffer,".--.") == 0)    decode_char = char('p'); 
  if (strcmp(CodeBuffer,"--.-") == 0)    decode_char = char('q'); 
  if (strcmp(CodeBuffer,".-.") == 0)     decode_char = char('r'); 
  if (strcmp(CodeBuffer,"...") == 0)     decode_char = char('s'); 
  if (strcmp(CodeBuffer,"-") == 0)       decode_char = char('t'); 
  if (strcmp(CodeBuffer,"..-") == 0)     decode_char = char('u'); 
  if (strcmp(CodeBuffer,"...-") == 0)    decode_char = char('v'); 
  if (strcmp(CodeBuffer,".--") == 0)     decode_char = char('w'); 
  if (strcmp(CodeBuffer,"-..-") == 0)    decode_char = char('x'); 
  if (strcmp(CodeBuffer,"-.--") == 0)    decode_char = char('y'); 
  if (strcmp(CodeBuffer,"--..") == 0)    decode_char = char('z'); 
  
  if (strcmp(CodeBuffer,".----") == 0)   decode_char = char('1'); 
  if (strcmp(CodeBuffer,"..---") == 0)   decode_char = char('2'); 
  if (strcmp(CodeBuffer,"...--") == 0)   decode_char = char('3'); 
  if (strcmp(CodeBuffer,"....-") == 0)   decode_char = char('4'); 
  if (strcmp(CodeBuffer,".....") == 0)   decode_char = char('5'); 
  if (strcmp(CodeBuffer,"-....") == 0)   decode_char = char('6'); 
  if (strcmp(CodeBuffer,"--...") == 0)   decode_char = char('7'); 
  if (strcmp(CodeBuffer,"---..") == 0)   decode_char = char('8'); 
  if (strcmp(CodeBuffer,"----.") == 0)   decode_char = char('9'); 
  if (strcmp(CodeBuffer,"-----") == 0)   decode_char = char('0'); 

  if (strcmp(CodeBuffer,"..--..") == 0)  decode_char = char('?'); 
  if (strcmp(CodeBuffer,".-.-.-") == 0)  decode_char = char('.'); 
  if (strcmp(CodeBuffer,"--..--") == 0)  decode_char = char(','); 
  if (strcmp(CodeBuffer,"-.-.--") == 0)  decode_char = char('!'); 
  if (strcmp(CodeBuffer,".--.-.") == 0)  decode_char = char('@'); 
  if (strcmp(CodeBuffer,"---...") == 0)  decode_char = char(':'); 
  if (strcmp(CodeBuffer,"-....-") == 0)  decode_char = char('-'); 
  if (strcmp(CodeBuffer,"-..-.") == 0)   decode_char = char('/'); 

  if (strcmp(CodeBuffer,"-.--.") == 0)   decode_char = char('('); 
  if (strcmp(CodeBuffer,"-.--.-") == 0)  decode_char = char(')'); 
  if (strcmp(CodeBuffer,".-...") == 0)   decode_char = char('_'); 
  if (strcmp(CodeBuffer,"...-..-") == 0) decode_char = char('$'); 
  if (strcmp(CodeBuffer,"...-.-") == 0)  decode_char = char('>'); 
  if (strcmp(CodeBuffer,".-.-.") == 0)   decode_char = char('<'); 
  if (strcmp(CodeBuffer,"...-.") == 0)   decode_char = char('~'); 
  if (strcmp(CodeBuffer,".-.-") == 0)    decode_char = char('a'); // a umlaut
  if (strcmp(CodeBuffer,"---.") == 0)    decode_char = char('o'); // o accent
  if (strcmp(CodeBuffer,".--.-") == 0)   decode_char = char('a'); // a accent

  clearCodeBuffer();

  if (decode_char != '{') {
    AddCharacter(decode_char);
    if (!graph)
    {
      lastChar = curChar;
      curChar = decode_char;
      cptCharPrinted++;
      if (cptCharPrinted > 100)
        CRRequested = true;
#ifndef TRACEF5BU
      Serial.print(decode_char);
#endif      
    }
  }
}

void AddCharacter(char newchar){
  if (CRRequested && (newchar != ' ')) 
  {
    CRRequested = false;
    cptCharPrinted = 0;
    Serial.println();
  }

  iCar++;
  if (iCar == nbChars)
  {
    iCar = 0;
    int posRow = 40 + (iRow * 20);
    tft.fillRect(394, posRow, 72, 20, TFT_BLACK); // Clear CodeBuffer
    clearDisplayLine();
    iRow++;
    if (iRow > 10) 
      iRow = 0;
  }
  else
    // Shift chars to get place for the new char
    for (int i = 0; i < nbChars; i++) DisplayLine[i] = DisplayLine[i+1];
  DisplayLine[nbChars - 1] = newchar;
}

float goertzelCoeff; // For Goertzel algorithm
float Q1 = 0;
float Q2 = 0;

#define NBSAMPLEMIN 50
#define NBSAMPLEMAX 250
int testData[NBSAMPLEMAX];
int nbSamples = 110;

// you can set the tuning tone to 496, 558, 744 or 992
int iFreq = 0;  
int iFreqMax = 8; // = NbFreq - 1
int freqs[] = { 496, // for MorseSample-15WPM.wav file
                558, 
                610, // (CW IC-7300)
                640, // for Sergeï beacon on 28.222.800 Hz (CW IC-7300)
                677, // Balise HB9F (CW IC-7000)
                744, 
                992,
                1040, // for QSO PiouPiou CW on SSB (IC-7000)
                1136  // for QSO PiouPiou CW on SSB (IC-7000)
              };
bool autoTune = true;
int sensFreq = 1;
int cptFreqLow = 0;
float sampling_freq = 0;
float target_freq = 0;
void setFreq(int freq)
{
  target_freq = freqs[freq];

  int k = (int) (0.5 + ((nbSamples * target_freq) / sampling_freq));
  float omega = (2.0 * PI * k) / nbSamples;
  goertzelCoeff = 2.0 * cos(omega);

  tft.fillRect(60, 20, 48, 20, TFT_BLACK);
  tftDrawString(60, 20, String(target_freq, 0));
}

float bw;
void setBandWidth(int nbsampl)
{
  bw = sampling_freq / nbsampl;
  tft.fillRect(180, 20, 36, 20, TFT_BLACK);
  tftDrawString(180, 20, String(bw, 0));
}

int idxCde= 0;
int idxCdeMax = 8;
char cdes[] = { 'F',  // sampling_freq
                'A',  // AutoTuneFreq
                'V',  // Volume
                'G',  // graph
                'S',  // nbSamples
                'T',  // nbTime filter
                'R',  // magReactivity
                'B'   // Space detector
              }; 

void showCde(int cde)
{
  String cdeText = "";
  switch(cdes[cde])
  {
    case 'F':
      cdeText = "Freq";
      break;
    case 'A':
      if (autoTune)
        cdeText = "AutoTune ON";
      else
        cdeText = "AutoTune OFF";
      break;
    case 'V':
      cdeText = "Volume=" + String(potVal);
      break;
    case 'S':
      cdeText = "NbSample=" + String(nbSamples);
      break;
    case 'T':
      cdeText = "Filtre=" + String(nbTime);
      break;
    case 'R':
      cdeText = "MagReact=" + String(magReactivity);
      break;
    case 'B':
      cdeText = "DetectBL=" + String(spaceDetector);
      break;
    case 'G':
      if (graph)
        cdeText = "Graph ON";
      else
        cdeText = "Graph OFF";
      break;
  }
  tft.fillRect(60, 300, 152, 20, TFT_BLACK);
  tftDrawString(60, 300, cdeText);
}

int cptLoop = 0;
void manageRotaryButton()
{
  // Manage Commands Rotary button
  // Rotary Encoder
  unsigned char dRot = rot.process();    
  if (dRot)
  {
    cptLoop = 0; // To show new acquired and loop time
    if (dRot != DIR_CW)
    {
      switch(cdes[idxCde])
      {
        case 'F':
          iFreq++;
          if (iFreq > iFreqMax)
            iFreq = iFreqMax;
          setFreq(iFreq);
          break;
        case 'A':
          autoTune = !autoTune;
          break;
        case 'V':
          potVal++;
          setVolume(potVal); 
          break;
        case 'S':
          nbSamples += 10;
          if (nbSamples > NBSAMPLEMAX)
            nbSamples = NBSAMPLEMAX;
          setBandWidth(nbSamples);
          break;
        case 'T':
          nbTime++;
          if (nbTime > 10)
            nbTime = 10;
          break;
        case 'R':
          magReactivity++;
          if (magReactivity > 10)
            magReactivity = 10;
          break;
        case 'B':
          spaceDetector++;
          if (spaceDetector > 10)
            spaceDetector = 10;
          break;
        case 'G':
          graph = !graph;
          break;
      }        
    }
    else
    {
      switch(cdes[idxCde])
      {
        case 'F':
          iFreq--;
          if (iFreq < 0)
            iFreq = 0;
          setFreq(iFreq);
          break;
        case 'A':
          autoTune = !autoTune;
          break;
        case 'V':
          potVal--;
          setVolume(potVal); 
          break;
        case 'S':
          nbSamples -= 10;
          if (nbSamples < NBSAMPLEMIN)
            nbSamples = NBSAMPLEMIN;
          setBandWidth(nbSamples);
          break;
        case 'T':
          nbTime--;
          if (nbTime < 0)
            nbTime = 0;
          break;
        case 'R':
          magReactivity--;
          if (magReactivity < 1)
            magReactivity = 1;
          break;
        case 'B':
          spaceDetector--;
          if (spaceDetector < 0)
            spaceDetector = 0;
          break;
        case 'G':
          graph = !graph;
          break;
      }
    }
    showCde(idxCde);
  }

  // Manage Rotary SW button
  rotSWState = digitalRead(rotEncSW);
  if (rotSWState != rotSWLastState)
  {
    if (rotSWState == LOW) // SW pressed
    {
      idxCde++;
      if (idxCde == idxCdeMax)
        idxCde = 0;
      showCde(idxCde);
    }        
    cptLoop = 0;
  }
  rotSWLastState = rotSWState;
}

void setup() {
  Serial.begin(115200);
  delay(1200); // 1200 mini to wait Serial is initialized...

  // TFT 4" SPI Init
  // Max SPI_FREQUENCY for this tft is 80000000 (80MHz) which is also the Max SPI speed for ESP32
  // It is only 10 MHz for SPI Potientiometer according to MCP41010 Datasheet, 
  // but it works well at 40MHz !!! (so, reduced to 40000000 in user_Setup.h of TFT Library (E:\Users\syst4\Documents\Arduino\libraries\TFT_eSPI-master)
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_ORANGE);
  tftDrawString(125, 5, "CW Decoder V1.3a (26/12/2023) by F4LAA");
  tft.setTextSize(2);

  // Rotary encoder
  /* */
  rot.begin(true);
  rotALastState = digitalRead(rotEncA);
  pinMode (rotEncSW,INPUT_PULLUP);
  //rotSWState = digitalRead(rotEncSW);
  /* */

  // Measure sampling_freq
  int tStartLoop = millis();
  int cpt = 0;
  while ( (millis() - tStartLoop) < 4000) { testData[0] = analogRead(A0); cpt++;}
  sampling_freq = cpt / 4;  // Measured at Startup on NodeMCU-32S

  // Templates
  tft.setTextColor(TFT_SKYBLUE);
  tftDrawString(0, 20, "Freq=    Hz BW=   Hz WPM=   MAG=");
  tftDrawString(0, 280, "Acq=   ms");  
  tftDrawString(116, 280, "Loop=   ms");  
  tftDrawString(0, 300, "Cde:");
  tftDrawString(312, 280, "Volume:");
  tftDrawString(312, 300, "SmplFreq=");
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // BGColor need to be set to TFT_BLACK to prevent overwriting !!!
  tftDrawString(420, 300, String(sampling_freq, 0));
  //EndOfTemplates
  
  setBandWidth(nbSamples);

  //////////////////////////////////// The basic goertzel calculation //////////////////////////////////////
  // you can set the tuning tone to 496, 558, 744 or 992
  // The number of samples determines bandwidth
  iFreq = 0;
  setFreq(iFreq); 

  idxCde = 0;
  showCde(idxCde);

  clearDisplayLine(); // CodeBuffer suit en mémoire. C'est lui qui contient le \0 final
  clearCodeBuffer();

  // F5BU
  for (int i = 0; i < nbChars; i++) DisplayLineF5BU[i] = ' '; 
  DisplayLineF5BU[nbChars] = '\0';

  // SPI Potentiometre (uses SPI instance defined in TFT library)
  pinMode (slaveSelectPin, OUTPUT); 
  setVolume(potVal); // Valeur mediane

  /* Debug SPI Potentiometer *
  int potSens = 1;
deb:
  potVal += potSens;
  if (potVal == 254)
   potSens = -1;
  if (potVal == 0)
   potSens = 1;
  setVolume(potVal);
  delay(500);
  goto deb;
  /* */
}

void changeVolume(int delta)
{
  int cValue = potVal + delta;
  if (cValue > 255)
    cValue = 255;
  if (cValue < 0)
    cValue = 0;
  setVolume(cValue);
}

void shiftDL(int len)
{
  for (int i=0; i<(nbChars - len); i++)
  { 
    int j = i + len;
    DisplayLineF5BU[i] = DisplayLineF5BU[j];
    DisplayLineF5BU[j] = ' ';
  }
}

void LCDAdd (void) {
  int len = Add.length();
  shiftDL(len); 
  int iDep = nbChars - len;
  for (int i=0; i<len; i++) {
    DisplayLineF5BU[iDep + i] = Add[i];
  }
  //tftDrawString(0, 100, DisplayLineF5BU);  
}

// Décodage
float k_e_cars = 1.6;     // coéfficient de d_point pour l'espace entre caractères
float k_e_mots = 6;       // coéfficient de d_point pour l'espace entre mots
#define e_cars_i d_point_i * k_e_cars    // espace mini entre caractères initial
#define e_mots_i d_point_i * k_e_mots    // espace mini entre mots initial
int16_t e_cars = e_cars_i;       // espace mini entre caractères
int16_t e_mots = e_mots_i;     // espace mini entre mots
int8_t Code_Char;
char T_car[4] = "   ";
char car;
// caractères pour les codes de 10000000 à 11111111, les valeurs de 1 à 4 sont spéciales
const static char CodeCW[]={
1,0,0,0,0,2,0,0,0,0,0,0,'?','_',0,0,
0,0,0,0,0,'.',0,0,0,0,'@',0,0,0,39,0,
0,'-',0,0,0,0,0,0,0,0,';','!',0,')',0,0,
0,0,0,',',0,0,0,0,':',0,0,0,0,0,0,0,
'5','4',0,'3','e',0,0,'2',3,0,'+',0,0,'a',0,'1',
'6','=','/',0,0,0,0,0,'7',0,0,0,'8',0,'9','0',
'H','V','F',0,'L',4,'P','J','B','X','C','Y','Z','Q',0,0,
'S','U','R','W','D','K','G','O','I','A','N','M','E','T',0,0};

void Decode_Char(void) {      //   * * * * * * * * * *
#ifdef TRACEF5BU
  String s = String(iRow) + ": ";
  for (int j=0; j <= i_CW; j++){
    s += String(T_CW[j]) + " ";
  }
  //tft.fillRect(0, 120 + (20 * iRow), 480, 20, TFT_BLACK);
  //tftDrawString(0, 120 + (20 * iRow), s);
  Serial.println(s);
#endif

  if (i_CW & 1 == 1)  {i_CW--;}     // par sécurité
  int16_t x = 0;
  int16_t e_min = T_CW[1];
  int16_t e_max = T_CW[1];
  int i = 0;
  for (int j=0; j <= i_CW; j++){         // détermination d_point et silence min et max, etc
    if ((j & 1) == 0) {     // pulses
      if (T_CW[j] < d_Tune && (T_CW[j] << 2)/d_point >= 7) {
        x = x + (T_CW[j] >> 2) + (T_CW[j] >> 4) + (T_CW[j] >> 6);   // x ~= x + T_CW[j]/3
        // 1/3 =~ 1/4+1/16+1/64 légèrement > mais pulses un peu raccourcis
      }
      else {
        x = x + T_CW[j];
      }
      i++;
    }
    else {                                // espaces entre pulses
      if (T_CW[j] > d_Para) {
        x = x + T_CW[j];
        if (T_CW[j] < e_min) {e_min = T_CW[j];}
        if (T_CW[j] > e_max) {e_max = T_CW[j];}
        i++;
      }
    }
  }
  if (i > 0)  {
    x = x + d_point;
    d_point = x/(i + 1);
    float x_float = (float) e_max/(float)e_min;
    i_Tem=0;
    if (x_float > k_e_cars) {
      for (int j=1; j <= i_CW; j=j+2){               // recherche si silence plus long = entre mots
        if ((T_CW[j] << 2)/d_point >= 5) {
          i_Tem++;
          Tem[i_Tem] = j;
          if (i_Tem > MaxTem-1)  {break;}
        }
      }
    }

    int ideb = 0;
    int ifin = 0;    
    int k = 0;
    for (int jb = 0; jb<=i_Tem; jb++) {
      if (i_Tem == 0) {
        ideb = 0;
        ifin = i_CW;
        k = 7;
      }
      else if (jb == 0) {
        ideb = 0;
        ifin = Tem[jb+1]-1;
        k = 7;
      }
      else if (jb == i_Tem) {    
        ideb = Tem[jb]+1;
        ifin = i_CW;
        k = 7;
      }
      else  {
        ideb = Tem[jb]+1;
        ifin = Tem[jb+1]-1;
        k = 7;
      }

      x = 0;
      for (int j = ideb+1; j <= ifin; j=j+2){
        x = x + T_CW[j];
      }

      x = x + d_point;
      d_point = x/((ifin-ideb)/2+1);
      e_cars = d_point*k_e_cars;     // espace mini entre caractères
      e_mots = d_point*k_e_mots;     // espace mini entre mots
  
      if (ifin-ideb<=11) {
        Code_Char=0;
        for (int j=(ifin-ideb)/2+2; j <= 6; j++){
          bitSet(Code_Char,j);
        }

        for (int j=ideb; j <=ifin; j=j+2) {
          if ((T_CW[j] << 2)/d_point >= k) {       // 1.5*4=6, 1.75*4=7, 2*4=8 ou plus ?
            bitSet(Code_Char,(ifin-j)/2);
          }
        }

        if (T_car[2] == 'E' && T_car[1] == 'E' && T_car[0] == 'E') {
          Add= " ";
          LCDAdd();
          T_car[2] = ' ';
        }  
        car = CodeCW[Code_Char];
      }
      else {
        car = 0;
      }
      switch (car)  {
        case 0:
          Add= "<?>";
          LCDAdd();
          break;
        case 1:
          Add= "<ER>";
          LCDAdd();
          break;
        case 2:
          Add= "<VA>";
          LCDAdd();
          break;
        case 3:
          Add= "<AS>";
          LCDAdd();
          break;
        case 4:
          Add= "<AA>";
          LCDAdd();
          break;
        default:
          Add= car;
          LCDAdd();
      }
      T_car[0] = T_car[1];
      T_car[1] = T_car[2];
      T_car[2] = car;
    }
  }
//digitalWrite(LED3,0);
  i_CW = -1;
}     // Fin Decode_Char()

void MaxT_CWOV (void) {      // Dépassement capacité tableau durées pulses et silences
  Decode_Char();
  i_CW = -1;
}

int vMin = 32000;
int vMax = 0;
int tStartLoop;
int adcMidpoint = 1940; // Measured on NodeMCU32 with 3.3v divisor
float vMoy = adcMidpoint;
#define MAXMOY 20
int cptMoy = 0;
float bMoy = 0;
int sBMoy = 0;
bool moyChanged = false;
bool moyComputed = false;

void loop() {
  cptLoop++;
  if(cptLoop == 1)
    tStartLoop = millis();

Acq:
  /* *
  // Measure sampling_freq
  int cpt0 = 0;
  tStartLoop = millis();
  while ( (millis() - tStartLoop) < 1000) { testData[0] = analogRead(A0); cpt0++;}
  if (cpt0 < vMin) vMin = cpt0;
  if (cpt0 > vMax) vMax = cpt0;
  Serial.println(String(vMin) + " " + String(cpt0) + " " + String(vMax));
  // Measured at : 11249 on NodeMCU-32S
  goto Acq;
  /* */

  // Measure adcMidpoint
  /* *
  //for (int index = 0; index < nbSamples; index++)
  int index = 0;
  {
    testData[index] = analogRead(A0);
    vMoy = ( (vMoy * 31) + testData[index] ) / 32;
    if (testData[index] < vMin) vMin = testData[index];
    if (testData[index] > vMax) vMax = testData[index];
    Serial.println(String(vMin) + " " + String(testData[index]) + " " + String(vMoy) + " " + String(vMax));
    // Measured at : 1950 on NodeMCU-32S avec diviseur du 3.3V
  }
  goto Acq;
  /* */

  // Acquisition
  for (int i = 0; i < nbSamples; i++) 
  {
    testData[i] = analogRead(A0);
  }

  if (cptLoop == 1)
  {
    int acqTime = millis() - tStartLoop;
    tftDrawString(48, 280, String(acqTime) + " ");  
  }

  // Compute magniture using Goertzel algorithm
  Q2 = 0;
  Q1 = 0;
  for (int index = 0; index < nbSamples; index++) {
    int curValue = testData[index] - adcMidpoint;
    float Q0 = (float)curValue + (goertzelCoeff * Q1) - Q2;
    Q2 = Q1;
    Q1 = Q0;
  }
  magnitude = sqrt( (Q1 * Q1) + (Q2 * Q2) - Q1 * Q2 * goertzelCoeff);

  // Adjust magnitudelimit
  if (magnitude > magnitudelimit_low) { magnitudelimit = (magnitudelimit + ((magnitude - magnitudelimit) / magReactivity)); } /// moving average filter
  if (magnitudelimit < magnitudelimit_low) magnitudelimit = magnitudelimit_low;

  // Now check the magnitude //
  if (magnitude > magnitudelimit * 0.3) // just to have some space up
    realstate = HIGH;
  else
    realstate = LOW;

  // Clean up the state with a noise blanker //  
  if (realstate != realstatebefore) 
  {
    laststarttime = millis();
  }
  if ((millis() - laststarttime) > nbTime) 
  {
    if (realstate != filteredstate) 
    {
      filteredstate = realstate;
    }
  }

  if (filteredstate != filteredstatebefore) 
  {
    if (filteredstate == HIGH) 
    {
      starttimehigh = millis();
      lowduration = (starttimehigh - startttimelow);
      if (lowduration > maxL) maxL = lowduration;
      if (lowduration < minL) minL = lowduration;
    }

    if (filteredstate == LOW) 
    {
      startttimelow = millis();
      highduration = (startttimelow - starttimehigh);
      if (highduration > maxH) maxH = highduration;
      if (highduration < minH) minH = highduration;

      // Strange cumputation of hightimesavg (very low compared to average of highduration )
      if ( (highduration < (2 * hightimesavg)) || (hightimesavg == 0) ) 
      {
        hightimesavg = (highduration + hightimesavg + hightimesavg) / 3; // now we know avg dit time ( rolling 3 avg)
      }
      if (highduration > (5 * hightimesavg) ) 
      {
        hightimesavg = highduration + hightimesavg;   // if speed decrease fast ..
      }
    }
  }

  // Now check the baud rate based on dit or dah duration either 1, 3 or 7 pauses
  if (filteredstate != filteredstatebefore) {
    stop = LOW;
    if (filteredstate == LOW) { // we did end on a HIGH
      if (highduration < (hightimesavg * 2) && highduration > (hightimesavg * 0.6)) { /// 0.6 filter out false dits
        dMoyDot = highduration; //((dMoyDot * moyF) + highduration ) / (moyF + 1);
        strcat(CodeBuffer, ".");
        //Serial.print(".");
        // F5BU
        if (highduration > d_Para && highduration < d_Tune) 
        {    // pulse
          if (i_CW>=MaxT_CW-1)  {MaxT_CWOV();}
          i_CW++;
          T_CW[i_CW] = highduration;    // enregistrement durée Pulse
        }
        else if (highduration >= d_Tune) {    // pulse trop long = Tune ou parasites
          i_CW = -1;
        }
        // End F5BU
      }
      if (highduration > (hightimesavg * 2) && highduration < (hightimesavg * 6)) {
        dMoyDash = highduration; //((dMoyDash * moyF) + highduration ) / (moyF + 1);
        strcat(CodeBuffer, "-");
        //Serial.print("-");
        wpm = (wpm + (1200 / ((highduration) / 3))) / 2; //// the most precise we can do ;o)
        // F5BU
        if (highduration > d_Para && highduration < d_Tune) 
        {    // pulse
          if (i_CW>=MaxT_CW-1)  {MaxT_CWOV();}
          i_CW++;
          T_CW[i_CW] = highduration;    // enregistrement durée Pulse
        }
        else if (highduration >= d_Tune) {    // pulse trop long = Tune ou parasites
          i_CW = -1;
        }
        // End F5BU
      }
    }

    if (filteredstate == HIGH) { // we did end a LOW
      float lacktime = 1;
      if (wpm > 25)lacktime = 1.0; ///  when high speeds we have to have a little more pause before new letter or new word
      if (wpm > 30)lacktime = 1.2;
      if (wpm > 35)lacktime = 1.5;
      if (lowduration > (hightimesavg * (2 * lacktime)) && lowduration < hightimesavg * (5 * lacktime)) { // letter space
        CodeToChar();
      }
      if (lowduration >= hightimesavg * (spaceDetector * lacktime)) { // word space
        dMoySpace = lowduration; //((dMoySpace * moyF) + lowduration ) / (moyF + 1);
        CodeToChar();        
        AddCharacter(' ');
        // F5BU
        Add = " ";
        LCDAdd();
        // End of F5BU
        if (!graph) 
        {
          Serial.print(" ");
          if ( (lastChar == 'b') and (curChar == 'k') ) // EOL
          {
            Serial.println("<===");
            CRRequested = false;
            cptCharPrinted = 0;
          }
        }
      }
    }
  }

  if ((millis() - startttimelow) > (highduration * 6) && stop == LOW) {
    CodeToChar();
    stop = HIGH;
  }

  // Sécurité
  if (strlen(CodeBuffer) == bufSize - 1) {
    // On a reçu des . et -, mais pas de silence...
    clearCodeBuffer();
  }

  /* *
  if (cptLoop == 50)
    tftDrawString(0, 240, "minL=" + String(minL) + " maxL=" + String(maxL) + " minH=" + String(minH) + " maxH=" + String(maxH) + "    ");
  if (cptLoop == 100)
  {
    cptLoop = 0;
    tftDrawString(0, 260, ".=" + String(dMoyDot, 0) + " -=" + String(dMoyDash, 0) + " SP=" + String(dMoySpace, 0) + " moyH=" + String(hightimesavg, 0) + "    ");
  }
  /* */

  if (graph)
  {
    if (magnitude < vMin) vMin = magnitude;
    if (magnitude > vMax) vMax = magnitude;
    int drawFilteredState;
    if (filteredstate == HIGH)
      drawFilteredState = vMax + 1000;
    else
      drawFilteredState = vMin - 1000;
    Serial.println(String(magnitude) + " " + String(drawFilteredState) + " " + String(magnitudelimit));
  }

  // Update display
  // Decoded CW  
  int posRow = 40 + (iRow * 20);
  tft.fillRect(394, posRow, 72, 20, TFT_BLACK); // Clear CodeBuffer
  //tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tftDrawString(0, posRow, DisplayLine); // Affiche aussi CodeBuffer (qui suit DisplayLine en mémmoire et contient le \0)
  //tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  if (abs(sWpm - wpm) >= 5)
  {
    sWpm = wpm;
    tft.fillRect(302, 20, 24, 20, TFT_BLACK);
    tftDrawString(302, 20, String(wpm));
  }

  // BarGraph Magnitude
  int barGraph = magnitude / 100;
  if (barGraph > 100)
    barGraph = 100;
  if (barGraph > 2) 
  {
    // Sound detected
    bMoy = ( ( (bMoy * cptMoy) + barGraph ) / (cptMoy + 1) );
    moyChanged = (abs(sBMoy - bMoy) > 1);
    sBMoy = bMoy;
    cptMoy++;
    if (cptMoy > MAXMOY) 
      cptMoy = MAXMOY;
    moyComputed = (cptMoy == MAXMOY);
  }
  else
  {
    // Silence
    moyComputed = false;
  }
  
  // Affichage valeurs barGraph et bMoy
  //tftDrawString(0, 220, "bMoy=" + String(bMoy) + "    barG=" + String(barGraph) + "   ");
  
  if (moyChanged)
    tft.fillRect(387, 23, 80, 10, TFT_BLACK); // Clear BarGraph
  if (bMoy > 75)
  {
    // bMoy in [76..100]
    if (moyChanged)
      tft.fillRect(387, 23, bMoy-20, 10, TFT_RED); // Draw BarGraph
    if (moyComputed)
      changeVolume(-4);
    cptFreqLow = 0;
  }
  else if (bMoy > 50)
  {
    // bMoy in [51..75]
    if (moyChanged)
    {
      sBMoy = bMoy;
      tft.fillRect(387, 23, bMoy-20, 10, TFT_ORANGE); // Draw BarGraph
    }
    if (moyComputed)
      changeVolume(-1);
    cptFreqLow = 0;
  }
  else if (bMoy > 30)
  {
    // bMoy in [31..50]
    if (moyChanged)
      tft.fillRect(387, 23, bMoy-20, 10, TFT_GREEN); // Draw BarGraph
    cptFreqLow = 0;
  }
  else
  { 
    // bMoy in [0..30]
    if (moyChanged)
      tft.fillRect(387, 23, bMoy-20, 10, TFT_LIGHTGREY); // Draw BarGraph
    if (barGraph < 10)
    {
      // barGraph in [0..9]
      // Nothing heard : Try to find better frequency tune
      if (autoTune)
      {
        cptFreqLow++;
        if (cptFreqLow > 100)
        {
          setVolume(POTMIDVALUE); // Middle value
          cptFreqLow = 100;
          cptMoy = 0;
          moyComputed = false;
          // Search for a better iFreq
          iFreq += sensFreq;
          if (iFreq > iFreqMax)
            iFreq = iFreqMax;
          if (iFreq < 0)
            iFreq = 0;
          if (iFreq == iFreqMax)
            sensFreq = -1;
          if (iFreq == 0)
            sensFreq = 1;
          setFreq(iFreq);
        }
      }
    }
    else      
    {
      // barGraph in [10..30]
      // Something heard, but low : Increase volume
      if (moyComputed)
        if (barGraph > 20)
          // barGraph in [21..30]
          changeVolume(2);
        else
          // barGraph in [10..20]
          changeVolume(4);
    }
  }

  // the end of main loop clean up//
  realstatebefore     = realstate;
  lasthighduration    = highduration;
  filteredstatebefore = filteredstate;

  if (cptLoop == 1)
  {
    int loopTime = millis() - tStartLoop;
    tftDrawString(176, 280, String(loopTime));  
  }

  manageRotaryButton();

 // EndOfLoop
}