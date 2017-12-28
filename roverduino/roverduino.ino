/*

 -= R o v e r d u i n o =-
 
 Piccolo rover dotato di un modulo ad ultrasuoni per rilevamento ostacoli,
 un display 8x8 per interazione utente, un cicalino, una fotores, un
 microfono, un sensore di temperatura ed encoder ottico incrementale sulle ruote.

 Hardware & Software by Flavio & Davide Giovannangeli
 Created on August 2015
 Updated on December 2017 (aggiunta gestione LED RGB e ottimizzazione codice)
 flavio.giovannangeli@gmail.com

*/

// Librerie display 8x8
#include <Wire.h>
#include <gfxfont.h>
#include <Ultrasonic.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_SPITFT_Macros.h>

Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

// Mappatura dispositivi Rover
#define microfono      A0     // IN analogico microfono.
#define fotores        A1     // IN analogico fotores.
#define lm35           A2     // IN analogico sensore di temperatura LM35.
#define cicalino       A3     // OUT digitale cicalino.
#define red_led        A4     // OUT digitale LED RGB colore ROSSO.
#define blu_led        A5     // OUT digitale LED RGB colore BLU.
#define motdx1a        5      // OUT digitale motore DX - canale 1A.
#define motdx2a        6      // OUT digitale motore DX - canale 2A.
#define motsx3a        8      // OUT digitale motore SX - canale 3A.
#define motsx4a        4      // OUT digitale motore SX - canale 4A.
#define enmotdx        9      // OUT digitale PWM controllo velocita' motore DX (1,2EN).
#define enmotsx        10     // OUT digitale PWM controllo velocita' motore SX (3,4EN).
#define encoddx        1      // IN digitale encoder ruota DX.
#define encodsx        7      // IN digitale encoder ruota SX.
#define swretro        0      // IN digitale microswitch rilevamento ostacolo in retromarcia.

Ultrasonic usradar(12);       // IN-OUT sensore ultrasuoni per rilevamento ostacoli.

// Variabili globali
int MANUTENZIONE = 0;            // Modalita' manutenzione (debug sensori su seriale attivo): 0=OFF, 1=ON

unsigned long time;              // Time
unsigned long encsx_time;        //
unsigned long encsx_time_final;  //
unsigned long encsx_time_lap;    //
unsigned long encdx_time;        //
unsigned long encdx_time_final;  //
unsigned long encdx_time_lap;    //

int   dmin_obstacle = 15;        // Distanza minima dall'ostacolo (in centimetri)
int   dirrover = 0;	             // Direzione rover: 0=Avanti, 1=indietro, 2=fermo
int   angolo = 0;                // Angolo rover
int   valspeed = 0;              // Velocita' ruote
int   intcntdx = 0;              // Contatore encoder ruota DX
int   intcntsx = 0;              // Contatore encoder ruora SX
int   fcrsx = 255;               // Fattore correzione ruota SX
int   fcrdx = 255;               // Fattore correzione ruota DX

// --------------------------------------------------------------------------------------------------
// --- S E T U P ------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
void setup() {
  
  // Inizializza millis
  time = millis();       // Variabile millis       
  encsx_time = time;     // time encoder SX
  encdx_time = time;     // time encoder DX
  
  // Inizializza porte I/O come indicato nella mappatura
  pinMode(microfono, INPUT);
  pinMode(fotores, INPUT);
  pinMode(lm35, INPUT);
  pinMode(cicalino, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(blu_led, OUTPUT);
  pinMode(motdx1a, OUTPUT);
  pinMode(motdx2a, OUTPUT);
  pinMode(motsx3a, OUTPUT);
  pinMode(motsx4a, OUTPUT);
  pinMode(encoddx, INPUT); 
  pinMode(encodsx, INPUT);
  pinMode(swretro, INPUT);
  
  // Inizializza interrupt
  attachInterrupt(digitalPinToInterrupt(0), rover_fermo, RISING);        // Interrupt pulsante rilevamento ostacolo in retromarcia (pin 0/RX)
  attachInterrupt(digitalPinToInterrupt(1), encoder_dx, CHANGE);         // Interrupt encoder ruota DX (pin 1/TX)
  attachInterrupt(digitalPinToInterrupt(7), encoder_sx, CHANGE);         // Interrupt encoder ruota SX (pin 7)

  // Inizializza display
  matrix.begin(0x70);                             // Indirizzo I2C display 8x8
  matrix.setRotation(1);                          // Imposta orientamento display
  
  if (MANUTENZIONE == 1) {                        // Se in MANUTENZIONE...
    Serial.begin(9600);                           //   attiva comunicazione seriale
    motori("OFF");                                //   disattiva motori
    Serial.println("Test sensori...");            //   Avviso test sensori
  } 
  else {                                          // altrimenti...
    display_saluto();                             //   visualizza messaggio di benvenuto
    digitalWrite(blu_led, HIGH);                  //   Led blu acceso                             
    motori("ON");                                 //   Attiva motori
    rover_fermo();                                //   Rover è fermo all'accensione
    Beep();                                       //   BEEP!
  }
} // FINE SETUP.
    

// --------------------------------------------------------------------------------------------------
// --- S T A R T  L O O P ---------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
void loop() {
  
  // GO ROVER GO!    
  if (MANUTENZIONE == 1) {                        // Se in MANUTENZIONE...
    int luxval = ReadFres();                      //   test fotoresistenza
    String slux = "Fotoresistenza: ";
    Serial.println(slux + luxval);    
    int micval = ReadMic();                       //   test microfono
    String smic = "Microfono: ";
    Serial.println(smic + micval);    
    float airtemp = ReadAirTemp();                //   test LM35 (temperatura)
    String sairtemp = "Temperatura: ";
    Serial.println(sairtemp + airtemp);    
    int dobstacle = ReadDistObstacle();           //   test radar ultrasuoni
    String sdobstacle = "Dist. ostacolo: ";
    Serial.println(sdobstacle + dobstacle);
    delay(3000);                                  //   attesa 3s.
  }  
  else
  {
    int dobstacle = ReadDistObstacle();             // Rilevamento distanza eventuale ostacolo
    if (dobstacle > 0 && dobstacle < dmin_obstacle) // Se distanza ostacolo minore della distanza impostata (7cm) ferma il rover
    {                                               // 
      rover_fermo();                                //   Presenza ostacolo nel range impostato! Ferma il Rover! 
      display_faces(3);                             //   Visualizza faccina triste :(
      delay(200);                                   //   Attesa 200ms.
      rover_destra(90);                             //   Rover a destra di 90 gradi
      rover_fermo();                                //   Rover fermo!
    }
    else                                            // altrimenti...
    {
      rover_avanti(255);                            //   Rover avanti tutta!
      display_faces(1);                             //   Rover sorridente :)
    }
  }
}

// --------------------------------------------------------------------------------------------------
// --- G E S T I O N E  M O T O R I -----------------------------------------------------------------
// --------------------------------------------------------------------------------------------------

// ON-OFF motori.
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
   if (MANUTENZIONE == 1) 
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
     int luxval = ReadFres();         // Lettura valore luminosita ambientale.
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
      matrix.print("Roverduino");
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

int ReadFres(){
  // Lettura valore fotoresistenza (luminosita' ambiente)
  int luxval = analogRead(fotores);
  return luxval;
}

int ReadMic(){
  // Lettura valore microfono (suoni ambiente)
  int micval = analogRead(microfono);
  return micval;
}

float ReadAirTemp(){
  // Lettura temperatura ambiente
  int lm35val = analogRead(lm35);                    // Lettura valore di temperatura dell'aria   
  int lm35mv = (lm35val/1023.0)*5000;                // Calcolo millivolt in uscita dal sensore LM35.
  float airtemp = lm35mv/10;                         // Valore temperatura ambiente rover.
  return airtemp;
}  

int ReadDistObstacle(){
  // Lettura valore distanza ostacolo in cm.
  // In input riceve la temperatura ambiente.
  int dobstacle = usradar.MeasureInCentimeters();
  return dobstacle;
}

void Beep(){
  // BEEP!
  tone(cicalino,900,250);
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

