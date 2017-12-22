/*

 -= R o v e r d u i n o =-
 
 Piccolo rover dotato di un modulo ad ultrasuoni per rilevamento ostacoli,
 un display 8x8 per interazione utente, un cicalino, una fotoresistenza, un
 microfono, un sensore di temperatura e encoder ottico incrementale sulle ruote.

 Hardware & Software by Flavio & Davide Giovannangeli
 Created on August 2015
 Updated on August 2016
 flavio.giovannangeli@gmail.com

*/

// Librerie display 8x8
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

// Mappa componenti Rover
#define microfono      A0     // Pin analogico di INPUT per il microfono.
#define fotoresistenza A1     // Pin analogico di INPUT per la fotoresistenza.
#define lm35           A2     // Pin analogico di INPUT per il sensore di temperatura LM35.
#define cicalino       A3     // Pin analogico di OUTPUT per il cicalino (buzzer).
#define radar          12     // Pin digitale di INPUT/OUTPUT per radar rilevamento ostacoli.
#define motdx1a        5      // Pin digitale di OUTPUT per canale 1A motore destro.
#define motdx2a        6      // Pin digitale di OUTPUT per canale 2A motore destro.
#define motsx3a        8      // Pin digitale di OUTPUT per canale 3A motore sinistro.
#define motsx4a        4      // Pin digitale di OUTPUT per canale 4A motore sinistro.
#define enmotdx        9      // Pin digitale di OUTPUT (PWM) per controllo motore destro (1,2EN).
#define enmotsx        10     // Pin digitale di OUTPUT (PWM) per controllo motore sinistro (3,4EN).
#define encoddx        1      // Pin digitale di INPUT per encoder ruota destra.
#define encodsx        7      // Pin digitale di INPUT per encoder ruota sinistra.
#define swretro        0      // Pin digitale di INPUT per microswitch rilevamento ostacolo in retromarcia.

// Variabili globali
unsigned long time;              // Time
unsigned long encsx_time;        //
unsigned long encsx_time_final;  //
unsigned long encsx_time_lap;    //
unsigned long encdx_time;        //
unsigned long encdx_time_final;  //
unsigned long encdx_time_lap;    //
int   DEBUG = 0;                 // Debug: 0=OFF, 1=ON
float vs = 343.80;               // Velocita' del suono in aria a 20C (default)
int   ostacolo = 7;           // Distanza minima dall'ostacolo (in centimetri)
int   cicafreq = 550;         // Frequenza cicalino
int   dirrover = 0;	          // Direzione rover: 0=Avanti, 1=indietro, 2=fermo
int   angolo = 0;             // Angolo rover
int   valspeed = 0;           // Velocita' ruote
int   intcntdx = 0;           // Contatore encoder ruota destra
int   intcntsx = 0;           // Contatore encoder ruora sinistra
int   fcrsx = 255;            // Fattore correzione ruota sinistra
int   fcrdx = 255;            // Fattore correzione ruota destra

// --------------------------------------------------------------------------------------------------
// --- S E T U P ------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
void setup() {
  // Inizializza millis
  time = millis();                                // Variabile millis       
  encsx_time = millis();                          // time encoder SX
  encdx_time = millis();                          // time encoder DX
  // Inizializza porte I/O
  pinMode(microfono, INPUT);                      // Pin analogico di INPUT per il microfono.
  pinMode(fotoresistenza, INPUT);                 // Pin analogico di INPUT per la fotoresistenza.
  pinMode(lm35, INPUT);                           // Pin analogico di INPUT per il sensore di temperatura LM35.
  pinMode(cicalino, OUTPUT);                      // Pin digitale di OUTPUT per il cicalino (buzzer).
  pinMode(motdx1a, OUTPUT);                       // Pin digitale di OUTPUT per canale 1A motore destro.
  pinMode(motdx2a, OUTPUT);                       // Pin digitale di OUTPUT per canale 2A motore destro.
  pinMode(motsx3a, OUTPUT);                       // Pin digitale di OUTPUT per canale 3A motore sinistro.
  pinMode(motsx4a, OUTPUT);                       // Pin digitale di OUTPUT per canale 4A motore sinistro.
  pinMode(encoddx, INPUT);                        // Pin digitale di INPUT per encoder ruota destra.
  pinMode(encodsx, INPUT);                        // Pin digitale di INPUT per encoder ruota sinistra.
  pinMode(swretro, INPUT);                        // Pin digitale di INPUT per microswitch rilevamento ostacolo in retromarcia.
  // Inizializza interrupt
  attachInterrupt(digitalPinToInterrupt(0), rover_fermo, RISING);        // Interrupt su pulsante rilevamento ostacolo in retromarcia (pin.0/RX).
  attachInterrupt(digitalPinToInterrupt(1), encoder_dx, CHANGE);         // Interrupt su encoder ruota DX (pin.1/TX).
  attachInterrupt(digitalPinToInterrupt(7), encoder_sx, CHANGE);         // Interrupt su encoder ruota SX (pin.7).
  // Inizializza comunicazione seriale
  if (DEBUG == 1)                                 // Se in DEBUG attiva comunicazione seriale.
  {                                               //
    Serial.begin(9600);                           // 
  }      
  // Inizializza display
  matrix.begin(0x70);                             // Indirizzo I2C display 8x8.
  matrix.setRotation(1);                          // Imposta orientamento corretto display.
  // Display WAIT
  //display_wait();                               // Mostra clessidra attesa per letture temperatura.
  // Inizializza velocita' del suono.
  //vs = vel_suono();                             // Calcolo velocita' del suono in base a temperatura ambiente (sborone!) 
  // Inizializza motori
  motori("ON");                                   // Attiva motori.
  rover_fermo();                                  // Il rover è fermo all'accensione.
  // Presentazione (no in debug!)
  if (DEBUG != 1)                                 // Se in DEBUG salta il saluto.
  {                                               //
    display_saluto();                             // Messaggio di saluto all'accensione.
  }
  else
  {                                               //  Se in debug mostra info sulla seriale.
    Serial.print("Velocita del suono:");          //
    Serial.println(vs);                           //
  }
  // READY! 
  rover_beep();                                   // BEEP di 'pronto'. 
}

// --------------------------------------------------------------------------------------------------
// --- S T A R T  L O O P ---------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
void loop() {
  // GO ROVER!     
  time = millis();                                // Variabile millis               
  int valdist = lettura_dist();                   // Rilevamento distanza eventuale ostacolo
  if (valdist > 0 && valdist < ostacolo)          // Se distanza ostacolo minore della distanza impostata (7 cm) ferma il rover
  {                                               // 
    display_faces(3);                             // Presenza ostacolo, rover triste! :(
    rover_fermo();                                // Rover fermo! 
    delay(200);                                   // Attesa 200ms.
//    if (valdist > 7)                              // Se distanza ostacolo < 7cm non andare indietro.
//    {                                             //
//      rover_indietro();                           // Rover indietro.
//    }
    rover_destra(90);                             // Rover a destra di 90 gradi!
    rover_fermo();                                // Rover fermo!
  }
  else
  {
    display_faces(1);                             // Rover sorridi! :)
    rover_avanti(255);                            // Rover avanti tutta!
  }
}

// --------------------------------------------------------------------------------------------------
// --- G E S T I O N E  M O T O R I -----------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
// Stato ON-OFF motori.
void motori(String stato) {
   if (stato == "ON") {
      digitalWrite(enmotdx, HIGH);
      digitalWrite(enmotsx, HIGH);
   }
   else {
      digitalWrite(enmotdx, LOW);
      digitalWrite(enmotsx, LOW);
   } 
}
// Rover avanti con velocita' determinata.
void rover_avanti(int valspeed) {
  dirrover = 0;
  // calcolo tempi encoder ruote
  if (encsx_time_lap > encdx_time_lap) {
    // Ruota sinistra piu' lenta, diminuisci velocita' ruota destra
    if (fcrdx > 200) {
       fcrdx--; 
    }
  }
  else {
    // Ruota destra piu' lenta, diminuisci velocita' ruota sinistra
    if (fcrsx > 200) {
       fcrsx--; 
    }
  } 
  // Imposta velocita' motori.
  analogWrite(enmotdx, fcrdx);
  analogWrite(enmotsx, fcrsx);
  // Imposta direzione rotazione motori
  digitalWrite(motdx1a, LOW);
  digitalWrite(motdx2a, HIGH);
  digitalWrite(motsx3a, HIGH);
  digitalWrite(motsx4a, LOW);
}
// Selezione velocita' con correzione differenza velocita ruote.
void vel_motori(int valspeed, int fattcorrdx, int fattcorrsx) {
   int fcdx = valspeed-fattcorrdx;
   int fcsx = valspeed-fattcorrsx;
   if (DEBUG == 1) 
   {
      Serial.println(fcdx);          
      Serial.println(fcsx);          
   }
   // Imposta velocita' motori.
   analogWrite(enmotdx, fcdx);
   analogWrite(enmotsx, fcsx);
}
// Rover fermo! Ferma entrambi i motori.
void rover_fermo() {
   dirrover = 1;
   digitalWrite(motdx1a, LOW);
   digitalWrite(motdx2a, LOW);
   digitalWrite(motsx3a, LOW);
   digitalWrite(motsx4a, LOW);
}
// Rover a destra di X gradi.
void rover_destra(int angolo) {
  digitalWrite(motdx1a, HIGH);
  digitalWrite(motdx2a, LOW);
  digitalWrite(motsx3a, HIGH);
  digitalWrite(motsx4a, LOW);
  delay(800);
  rover_fermo();
}
// Rover a sinistra di X gradi.
void rover_sinistra(int angolo) {
  digitalWrite(motdx1a, LOW);
  digitalWrite(motdx2a, HIGH);
  digitalWrite(motsx3a, LOW);
  digitalWrite(motsx4a, HIGH);
  delay(800);
  rover_fermo();
}
// Rover indietro.
void rover_indietro() {
  dirrover = 1;
  digitalWrite(motdx1a, HIGH);
  digitalWrite(motdx2a, LOW);
  digitalWrite(motsx3a, LOW);
  digitalWrite(motsx4a, HIGH);
  delay(1000);
}
// --------------------------------------------------------------------------------------------------
// --- G E S T I O N E  D I S P L A Y ---------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
static const uint8_t PROGMEM
  smile_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100 },
  neutral_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10111101,
    B10000001,
    B01000010,
    B00111100 },
  frown_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10011001,
    B10100101,
    B01000010,
    B00111100 },
  wait_bmp[] =
  { B00000000,
    B01111110,
    B00111100,
    B00011000,
    B00011000,
    B00100100,
    B01111110,
    B00000000 },
  park_bmp[] =
  { B00000000,
    B00111000,
    B00100100,
    B00100100,
    B00111000,
    B00100000,
    B00100000,
    B00000000 };
int display_lux(){
     // Adatta luminosita' del display alla luminosita' ambientale.
     int luxval = lettura_fres();         // Lettura valore luminosita ambientale.
     int displux = int(luxval/68);        // Luminosita display da 1-15; calcolato valore fotores. 0-1024.
     if (displux == 0) displux = 1;       // Se valoreluminosita'<1 imposta a 1.
     return displux;
}
void display_saluto(){
    // Mostra saluto sul display.
    matrix.setBrightness(display_lux());  // Adatta luminosita' del display alla luminosita' ambientale.
    matrix.setTextSize(1);
    matrix.setTextWrap(false);
    matrix.setTextColor(LED_ON);
    for (int8_t x=0; x>=-108; x--) {
      matrix.clear();
      matrix.setCursor(x,0);
      matrix.print("Hello! I'm ready!");
      matrix.writeDisplay();
      delay(80);
  }
}
void display_write(char mychar){
    // Mostra un carattere sul display.
    matrix.setBrightness(display_lux());  // Adatta luminosita' del display alla luminosita' ambientale.
    matrix.setTextSize(1);
    matrix.setTextWrap(false);
    matrix.setTextColor(LED_ON);
    matrix.clear();
    //matrix.setCursor(x,0);
    matrix.print(mychar);
    matrix.writeDisplay();
}
void display_clear() {
   // Pulisci display.
   matrix.clear();
   matrix.writeDisplay();
}
void display_faces(int face) {
   // Mostra faccine :)
   matrix.clear();
   matrix.setBrightness(display_lux());  // Adatta luminosita' del display alla luminosita' ambientale.
   if (face == 1) 
   {
      matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_ON);
   }
   else if (face == 2)
   {
      matrix.drawBitmap(0, 0, neutral_bmp, 8, 8, LED_ON);
   }
   else
   {
      matrix.drawBitmap(0, 0, frown_bmp, 8, 8, LED_ON);
   }
   matrix.writeDisplay();
}
void display_wait() {
   // Wait
   matrix.clear();
   matrix.setBrightness(display_lux());               // Adatta luminosita' del display alla luminosita' ambientale.
   matrix.drawBitmap(0, 0, wait_bmp, 8, 8, LED_ON);
   matrix.writeDisplay();
}
void display_park() {
   // Parking
   matrix.clear();
   matrix.setBrightness(display_lux());               // Adatta luminosita' del display alla luminosita' ambientale.
   matrix.drawBitmap(0, 0, park_bmp, 8, 8, LED_ON);
   matrix.writeDisplay();
}

// --------------------------------------------------------------------------------------------------
// --- G E S T I O N E  S E N S O R I ---------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
int lettura_fres(){
  // Lettura valore fotoresistenza.
  int luxval = analogRead(fotoresistenza);           // Lettura valore luminosita ambientale.
  return luxval;
}
int lettura_micp(){
  // Lettura valore microfono.
  int micval = analogRead(microfono);                // Lettura valore suoni ambientali.
  return micval;
}
float lettura_airtemp(){
  // Lettura temperatura da sonda LM35
  int lm35sens = analogRead(fotoresistenza);         // Lettura valore di temperatura dell'aria   
  int mv = (lm35sens/1023.0)*5000;                   // Calcolo millivolt in uscita dal sensore LM35.
  if (DEBUG == 1) 
  {
     Serial.println(mv);                             // Info millivolt in uscita LM35.
  }
  float airtemp = mv/10;                             // Valore temperatura ambiente rover.
  return airtemp;
}  
int lettura_dist(){
  // Lettura valore distanza ostacolo in cm.
  // In input riceve la temperatura ambiente.
  pinMode(radar, OUTPUT);                            // Pin digitale di OUTPUT per radar rilevamento ostacoli.
  digitalWrite(radar, LOW);                          // Digital LOW.
  delayMicroseconds(2);                              // Delay 2 microsec.
  digitalWrite(radar, HIGH);                         // Invio impulso di 10 microsec.
  delayMicroseconds(5);                              //  
  digitalWrite(radar, LOW);                          //
  pinMode(radar, INPUT);                             // Pin digitale di INPUT per radar rilevamento ostacoli.
  int echotime = pulseIn(radar, HIGH);               // Calcolo tempo A/R segnale.
  //int airvelsound = 331.45 + (0.62 * ta);          // Calcolo velocita' del suono nell'aria in base alla temperatura rilevata al momento.
  int dist = (vs/10000) * echotime;                  // Calcolo distanza ostacolo (v=s*t).
  dist = dist/2;
  //int dist = microsecondsToCentimeters(echotime);
  if (DEBUG == 1) 
  {
     Serial.print("Echo time:");                     // Info echotime.
     Serial.println(echotime);                       // Info echotime.
     Serial.print("Air.Vel.Sound:");
     Serial.println(vs);
     Serial.print("Dist:");                          // Info echotime.
     Serial.println(dist);                           // Info distanza ostacolo.
  }
  return dist;
}

// --------------------------------------------------------------------------------------------------
// --- G E S T I O N E  E N C O D E R S -------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
void encoder_dx(){
  // Encoder ruota destra
  intcntdx++;
  if (intcntdx == 3) {
    // Giro di ruota completo, calcola il tempo impiegato
    encdx_time_final = millis();
    encdx_time_lap = (encdx_time_final - encdx_time);
    intcntdx = 0;
    encdx_time = millis();
  }
} 

void encoder_sx(){
  // Encoder ruota sinistra
  intcntsx++;
  if (intcntsx == 3) {
    // Giro di ruota completo, calcola il tempo impiegato
    encsx_time_final = millis();
    encsx_time_lap = (encsx_time_final - encsx_time);
    intcntsx = 0;
    encsx_time = millis();
  }
}

// --------------------------------------------------------------------------------------------------
// --- R O U T I N E  V A R I E ---------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
void rover_beep(){
  // BEEP!
  tone(cicalino,cicafreq,500);
}
float vel_suono(){
  // Calcolo della velocita' del suono in base alla temperatura ambiente rover.
  // Esegue 3 letture della temperatura ogni 10s e poi fa la media.
  float somma = 0.00;
  float tam = 0.00;
  int i = 0;
  for (int i=0;i<3;i++){
      float ta = lettura_airtemp();
      somma = somma + ta;
      delay(10000);
  }
  tam = (somma/3.0);                  // Media delle temperature
  vs = (331.45)+(0.62*tam);             
  if (DEBUG == 1) 
  {
     Serial.print("LM35 uscita (millivolt):");
     Serial.println(tam);             // Info millivolt in uscita LM35.
     Serial.print("Vel. suono:");
     Serial.println(vs);
  }
  return vs;
}

