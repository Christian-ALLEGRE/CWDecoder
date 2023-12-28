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

 25/12/2023 : Modifications V1.2 ==> V1.3a : (Publiée sur GitHub pour F8CND)
   - Intégration de l'Algo de F5BU basé sur la mesure des temps de dot / dash / silences

 27/12/2023 : Modifications V1.2 ==> V1.3b :
   - remplacement compteur (cptFreqLow) par la mesure du temps (using startLowSignal)
   - Suppression du code F5BU

 27/12/2023 : Modifications V1.3b ==> V1.3c :
   - Modif Algo : On base tout sur la mesure de moyDot, puis calcul de moyDash et moySP, séparation . / - using discri
     Ca marche beaucoup moins bien ☹️
   - Ajout trace pour visualiser sur le TFT : les min/max H et L, et les moyennes (moyDot, moyDash, moySP, discri, bMoy et barGraph)

 28/12/2023 : Modifications V1.3c ==> V1.3d :
   - Retour à l'Algo de YT utilisant hightimesavg comme discriminateur . / -
   - Republication sur GitHub (sans le code de F5BU)

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

void tftDrawString(int x, int y, String s)
{
  tft.setCursor(x, y);
  tft.println(s);
}

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
int starttimelow;
int lowduration;
int laststarttime = 0;
float hightimesavg = 0; // Séparation dot / dash et SP

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

void clearDisplayLine()
{
  for (int i = 0; i < nbChars; i++) DisplayLine[i] = ' ';
}

void clearCodeBuffer()
{
  CodeBuffer[0] = '\0';
  for (int i = 1; i < bufSize; i++) CodeBuffer[i] = ' ';
};

int cptCharPrinted = 0;
bool CRRequested = false;
void AddCharacter(char newchar)
{
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
    int posRow = 60 + (iRow * 20);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tftDrawString(0, posRow, DisplayLine); // Affiche aussi CodeBuffer (qui suit DisplayLine en mémmoire et contient le \0)
    tft.fillRect(394, posRow, 72, 20, TFT_BLACK); // Clear CodeBuffer
    clearDisplayLine();
    iRow++;
    if (iRow > 9) 
      iRow = 0;
  }
  else
    // Shift chars to get place for the new char
    for (int i = 0; i < nbChars; i++) DisplayLine[i] = DisplayLine[i+1];
  DisplayLine[nbChars - 1] = newchar;
}

bool graph = false; // To draw magnitude curve
bool trace = false;
char lastChar = '{';
char curChar = '{';
void CodeToChar() { // translate cw code to ascii character//
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
      Serial.print(decode_char);
    }
  }
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
                'D',  // trace
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
    case 'D':
      if (trace)
        cdeText = "Trace ON";
      else
        cdeText = "Trace OFF";
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
        case 'D':
          trace = !trace;
          if (!trace)
          {
            // Clear trace
            tft.fillRect(0, 220, 480, 60, TFT_BLACK);
          }
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
        case 'D':
          trace = !trace;
          if (!trace)
          {
            // Clear trace
            tft.fillRect(0, 220, 480, 60, TFT_BLACK);
          }
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
  tftDrawString(125, 5, "CW Decoder V1.3d (28/12/2023) by F4LAA");
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

long startLowSignal = 0;
long startLowSound = 0;
bool bScan = false;
void stopScan()
{
  startLowSignal = 0;
  bScan = false;  
}

int vMin = 32000;
int vMax = 0;
int tStartLoop;
int adcMidpoint = 1940; // Measured on NodeMCU32 with 3.3v divisor
float vMoy = adcMidpoint;
#define MAXMOY 20
int cptMoy = 0;
float bMoy = 0;
int dispMoy = 0;
int sBMoy = 0;
bool moyChanged = false;
bool moyComputed = false;
int silent = 5; // barGraph silent level

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
      // front montant
      starttimehigh = millis();
      lowduration = (starttimehigh - starttimelow);
    }

    if (filteredstate == LOW) 
    {
      // front descendant
      starttimelow = millis();
      highduration = (starttimelow - starttimehigh);

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

  if (!bScan) // Not in search frequency mode
  {
    // Now check the baud rate based on dit or dah duration either 1, 3 or 7 pauses
    if (filteredstate != filteredstatebefore) {
      stop = LOW;
      if (filteredstate == LOW) { // we did end on a HIGH
        if (highduration < (hightimesavg * 2) && highduration > (hightimesavg * 0.6)) { /// 0.6 filter out false dits
          strcat(CodeBuffer, ".");
          //Serial.print(".");
        }
        
        if (highduration > (hightimesavg * 2) && highduration < (hightimesavg * 6)) {
          strcat(CodeBuffer, "-");
          //Serial.print("-");
          wpm = (wpm + (1200 / ((highduration) / 3))) / 2; //// the most precise we can do ;o)
        }
      }

      if (filteredstate == HIGH) { // we did end a LOW
        float lacktime = 1;
        if (wpm > 25) lacktime = 1.0; ///  when high speeds we have to have a little more pause before new letter or new word
        if (wpm > 30) lacktime = 1.2;
        if (wpm > 35) lacktime = 1.5;

        if (lowduration > (hightimesavg * (2 * lacktime)) && lowduration < hightimesavg * (5 * lacktime)) { // letter space
            CodeToChar();
        }

        if (lowduration >= hightimesavg * (spaceDetector * lacktime)) { // word space
          CodeToChar();        
          AddCharacter(' ');
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
    } // filteredstate != filteredstatebefore

    if ((millis() - starttimelow) > (highduration * 6) && stop == LOW) {
      CodeToChar();
      stop = HIGH;
    }
    
    // Sécurité
    if (strlen(CodeBuffer) == bufSize - 1) {
      // On a reçu des . et -, mais pas de silence...
      clearCodeBuffer();
    }
  } // !bScan

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

  if (!bScan)
  {
    // Update display
    // Decoded CW  
    int posRow = 60 + (iRow * 20);
    tft.fillRect(394, posRow, 72, 20, TFT_BLACK); // Clear CodeBuffer
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tftDrawString(0, posRow, DisplayLine); // Affiche aussi CodeBuffer (qui suit DisplayLine en mémmoire et contient le \0)
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // WPM
    if (abs(sWpm - wpm) >= 5)
    {
      sWpm = wpm;
      tft.fillRect(302, 20, 24, 20, TFT_BLACK);
      tftDrawString(302, 20, String(wpm));
    }
  }

  // BarGraph Magnitude
  int barGraph = magnitude / 100;
  if (barGraph > 100)
    barGraph = 100;
  if (barGraph > silent) 
  {
    // Sound detected
    stopScan();
    bMoy = ( ( (bMoy * cptMoy) + barGraph ) / (cptMoy + 1) );
    dispMoy = bMoy;
    if (dispMoy > 97)
      dispMoy = 97; // Pour ne as dépasserr les 480 pixels
    moyChanged = true; //(abs(sBMoy - bMoy) > 1);
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
  
  if (trace)
  {
    // Affichage valeurs barGraph et bMoy
    tftDrawString(0, 260, "bMoy=" + String(bMoy) + "    barG=" + String(barGraph) + "   ");
  }

  if (moyChanged || bScan)
    tft.fillRect(387, 23, 93, 10, TFT_BLACK); // Clear BarGraph
  
  if (barGraph > 20)
  {
    // Sound detected
    if (bMoy > 75) 
    {
      // bMoy in [76..100]
      if (moyChanged)
        tft.fillRect(387, 23, dispMoy, 10, TFT_RED); // Draw BarGraph
      if (moyComputed)
        changeVolume(-4);
    }
    else if (bMoy > 50)
    {
      // bMoy in [51..75]
      if (moyChanged)
      {
        sBMoy = bMoy;
        tft.fillRect(387, 23, dispMoy, 10, TFT_ORANGE); // Draw BarGraph
      }
      if (moyComputed)
        changeVolume(-2);
    }
    else if (bMoy > 20)
    {
      // bMoy in [21..50]
      if (moyChanged)
      {
        startLowSound = 0;
        tft.fillRect(387, 23, dispMoy, 10, TFT_GREEN); // Draw BarGraph
      }
    }
  }
  else
  { 
    // Low sound detected
    // barGraph in [silent..20]
    if (moyChanged || bScan)
      tft.fillRect(387, 23, barGraph, 10, TFT_LIGHTGREY); // Draw BarGraph

    // Something heard, but low : Increase volume
    if (moyComputed)
    {
      if (startLowSound == 0)
        startLowSound = millis();
      else if ((millis() - startLowSound) > 2000) // 2s with low sound
      {
        // Increase volume
        if (barGraph > 20)
          // barGraph in [21..30]
          changeVolume(2);
        else
          // barGraph in [10..20]
          changeVolume(4);
      }
    }

    if (barGraph < 10)
    {
      // barGraph in [silent..9]
      // Very weak signal heard : Try to find better frequency tune
      if (autoTune)
      {
        if (startLowSignal == 0)
          startLowSignal = millis();
        else if ((millis() - startLowSignal) > 5000) // 5s without audible signal
        {
          bScan = true;
          setVolume(POTMIDVALUE); // Middle value
          cptMoy = 0;
          moyComputed = false;
          starttimehigh = 0;
          starttimelow = 0;
          lowduration = 0;         
          highduration = 0;         

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