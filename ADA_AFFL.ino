//
// ADA control based on a AFFL-controller
//
// This code was developed for the release of a paper.
//
// All values with units are written with those.
//
// You shall not use this code without explicit permission by the author, Johann Lange
//
// In case of any questions feel free to contact me at johann.lange@tuhh.de
//
// Parts of this code are in german.

//ALLGEMEIN
//Includes
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <SPI.h>
#include <String.h>
#include <EEPROM.h>

//Deklaration
///Zeit
elapsedMillis looptime = 0; //Zeit die der Teensy fuer einen Durchlauf benoetigt in ms
int loopnumber = 0; //Anzahl der durchlaufenen Loops
elapsedMillis overalltime = 0; //Gesamte Durchlaufzeit in ms
const int iterationtime = 25; //Zeit die der Teensy fuer einen Durchlauf hat in ms
const int maxoveralltime = 64000; //Maximale Tauchzeit, bedingt durch den EEPROM in ms
///Koordinaten
double z = 0; //Tauchtiefe der Tauchzelle in m
double zpunkt = 0; //Geschwindigkeit der Tauchzelle in m/s
double zpp = 0; //Beschleunigung der Tauchzelle in m/s^2
const double z_ziel = 0.25; //Zieltiefe der Tauchzelle in m
double p0 = 0; //Nulldruck fuer z = 0 in mbar
double p_ziel = 0; //Zieldruck der Tauchzelle in mbar


//DRUCKSENSOR
// Declaration of Copyright
// Copyright (c) for pressure sensor 2009 MEAS Switzerland
// Edited 2015 Johann Lange
// @brief This C code is for starter reference only. It was based on the
// MEAS Switzerland MS56xx pressure sensor modules and Atmel Atmega644p
// microcontroller code and has been by translated Johann Lange
// to work with Teensy 3.1 microcontroller.
//Macros
#define TRUE 1
#define FALSE 0
#define CMD_RESET 0x1E //ADC reset command (ADC = Analgo-Digital-Converter)
#define CMD_ADC_READ 0x00 //ADC read command
#define CMD_ADC_CONV 0x40 //ADC conversion command
#define CMD_ADC_D1 0x00 //ADC D1 conversion
#define CMD_ADC_D2 0x10 //ADC D2 conversion
#define CMD_ADC_256 0x00 //ADC OSR=256 (OSR = Oversamplingrate)
#define CMD_ADC_512 0x02 //ADC OSR=512
#define CMD_ADC_1024 0x04 //ADC OSR=1024
#define CMD_ADC_2048 0x06 //ADC OSR=2056
#define CMD_ADC_4096 0x08 //ADC OSR=4096
#define CMD_PROM_RD 0xA0 //Prom read command

//Deklarationen
unsigned long D1; //ADC value of the pressure conversion
unsigned long D2; //ADC value of the temperature conversion
unsigned int C[8]; //calibration coefficients
double P; //compensated pressure value
double T; //compensated temperature value
double dT; //difference between actual and measured temperature
double OFF; //offset at actual temperature
double SENS; //sensitivity at actual temperature
double T2; //compensated pressure value, 2nd order
double OFF2; //compensated pressure value, 2nd order
double SENS2; //compensated pressure value, 2nd order
unsigned char n_crc;
// int i; //counting variable


//SPI
const int MISO_Pin = 12;
const int MOSI_Pin = 11;
const int SCK_Pin = 13;
const int CS_Pin_PS = 10; //Chip Select Pressure Sensor
SPISettings settings_PS(4000000, MSBFIRST, SPI_MODE0); //Grundeinstellungen fuer das SPI-Protokoll, Pressure Sensor


//SLIDING-MODE-OBSERVER (SMO)
//Deklaration
///Modelparameter
const double rho = 10.0; //Observerparameter
const double tau = 0.2; //Observerparameter
const double Phi = 0.5;
///Koordinaten
double xhat1 = 0.0; //Durch den SMO geschaetzte Tiefe der Tauchzelle in m
double xhat2 = 0.0; //Durch den SMO geschaetzte Geschwindigkeit der Tauchzelle in m/s
double xhat1_prev[2] = {0.0, 0.0}; //Durch den SMO vorangegangene geschaetzte Tiefe der Tauchzelle in m
double xhat2_prev[2] = {0.0, 0.0}; //Durch den SMO vorangegangene geschaetzte Geschwindigkeit der Tauchzelle in m/s

//REGLER
//Deklaration
///Drucke des Regelkreises
double pdiff = 0; //Differenzdruck zwischen P und p_ziel
double zdiff = 0; //Differenzdtiefe zwischen z und z_ziel
double pdiff_sum = 0; //aufsummierter Differenzdruck
double zdiff_sum = 0; //aufsummierte Differenztiefe
double palpha = 0; //Stellgroesse des PID-Reglers zur Berechnung des Winkel alpha
double palpha_max = 50; //Maximalwert der Stellgroesse fuer sinnvolle Auswertung
///Reglergroessen fuer PID-Regler
const double Kp = 1; //Proportionale Verstaerkung des PID-Reglers
const double Ki = 0; //Integrale Verstaerkung des PID-Reglers
const double Kd = 1000; //Differentiale Verstaerkung des PID-Reglers
///Reglergroessen fuer (AF)SMC
const double k = 0.0; //Dampening factor
const double lambda = 0.8;  //SMC performance bandwidth (1, 0.4)
const double eta = 0.1; //SMC reaching parameter (10, 0.1)
const double phi_s = 0.0; //AFSMC tuning parameter (10, 50)
const int nFuzzy = 7; //Number of Membership functions
const double m_K = 0.37577; //Masse in kg
const double rho_W = 1000; //Dichte von Wasser in kg/m^3
const double g = 9.81; //Erdbeschleunigung in m/s^2
const double V_0 = 0.0003663654; //Volumen der Tauchzelle in m^3
const double A_Membran = 0.000491; //Flaeche der Rollmembran in m^2
const double h_max = 0.01; //Maximaler einseitiger Hub in m
double Psi[7] = {0, 0, 0, 0, 0, 0, 0}; //Fuzzy-Membership-Values
const double Dc[11] = { -2.0 * Phi, -Phi, -Phi / 4.0, -Phi / 20.0, -Phi / 40.0, 0.0, Phi / 40.0, Phi / 20.0, Phi / 4.0, Phi, 2.0 * Phi}; //Fuzzy Center Points
double D[7] = {0, 0, 0, 0, 0, 0, 0};
double d_hat = 0; // Adaptive-Fuzzy-Variable
const double zppp_des = 0; //Desired jerk
const double a = 0.03; //Actuator time constant


//MOTOR
//Deklaration
///Pins
const int DIR_Pin = 21;
const int STEP_Pin = 22;
const int SLEEP_Pin = 23;
const int Endlagen_Pin = 17; //HIGH: nicht gedrueckt, LOW: gedrueckt
///Variablen
const int iplanet = 256; //Uerbersetzung des Planetengetriebes
const double schrittwinkel = 18; //Vollschrittwinkel des Motors in grd
double winkel = 0; //Tatsaechlicher Winkel des Zahnrades in grd
double alpha = 0; //Angestrebter Winkel des Zahnrades in grd
const double alpha_max = 57.5; //Maximaler Winkel des Zahnrades in grd
const double Schrittfrequenz = 2500; //Maximale Schrittfrequen bei welcher das maximal benoetigte Drehmoment noch aufgebracht werden kann in steps/s (max 3540)
const double Schrittzeit = 1000000.0 / Schrittfrequenz; //Die Zeit die der Motor fuer einen Schritt bekommt in µs
double Schrittzeit_halb = 0; //Die Zeit die der Motor fuer einen Puls bekommt in µs, Definition in setup
int Schrittzahl = 0; //Schrittzaehler ausgehend von 0 fuer Winkel = 0

//EEPROM
//Deklarations
int nsd = 0; //Zaehler fuer das Abspeichern der Daten in das Array
const int zeilenArrayMax = 128; //Maximale Anzahl der Zeilen des Arrays
int iArray = 0; //Anzahl der bereits belegten Zeilen des Arrays
long datenArray[128][4]; //Array um die Daten zu speichern
bool aufEEPROMgeschrieben = false;
double alleDaten[2560][2]; //Array zum speichern aller Tiefen und Winkeldaten


void setup() {
  //MOTOR
  Schrittzeit_halb = round(Schrittzeit / 2.0);

  //SPI
  pinMode(CS_Pin_PS, OUTPUT);
  pinMode(MISO_Pin, OUTPUT);
  pinMode(SCK_Pin, OUTPUT);
  pinMode(MOSI_Pin, INPUT);
  SPI.setMOSI(MOSI_Pin);
  SPI.setMISO(MISO_Pin);
  SPI.setSCK(SCK_Pin);
  SPI.begin();

  //Drucksensor
  SPI.beginTransaction(settings_PS);
  cmd_reset(); // reset the module after powerup
  for (int i = 0; i < 8; ++i) {
    C[i] = cmd_prom(i);
    //Serial.printf("C[%i] = %i\n", i, C[i]); // Wenn aktiviert, werden am Anfang die Kalibrierungskoeffs des Drucksensor aufgelistet
  } // read calibration coefficients
  n_crc = crc4(C);
  SPI.endTransaction();

  pinMode(DIR_Pin, OUTPUT);
  pinMode(STEP_Pin, OUTPUT);
  pinMode(SLEEP_Pin, OUTPUT);
  pinMode(Endlagen_Pin, INPUT_PULLUP);
  digitalWrite(DIR_Pin, LOW); // LOW = CCW = V down; HIGH = CW = V up
  digitalWrite(STEP_Pin, LOW);
  digitalWrite(SLEEP_Pin, LOW); // LOW = OFF; HIGH = ON


  // Auf Startsignal warten (Drucksensor)
  while (P < 1050) {
    P = berechneDruck();
  }


  digitalWrite(SLEEP_Pin, HIGH); // LOW = OFF; HIGH = ON
  Schrittzahl = resetPosition(); // Motor starten und zuruecksetzen
  delay(1000);

  //Koordinatentransformationen
  p0 = berechneDruck();
  p_ziel = p0 + 1000.0 * (z_ziel / 10.0);
}

void loop() {
  Serial.println("dummy");

  // Druck bestimmen
  P = berechneDruck();

  // Tauchtiefe berechnen
  z = 10.0 * ((P - p0) / 1000.0);

  // Geschw ueber SMO berechnen
  zpunkt = get_xhat2(z, 0);

  // Beschleunigung ueber SMO berechnen
  zpp = get_xhat2(zpunkt, 1);

  // Calculate the needed angle alpha based on the AFFL control laws
  alpha = berechneAFFL();

  // Winkel anfahren
  winkel = step(alpha);

  // Daten (z, zpunkt, alpha, winkel) in Array schreiben, wenn das Array voll ist, die Daten in den EEPROM schreiben
  ++nsd;
  if (nsd >= 20) {
    WritetoArray();
  }

  // Tiefe und Winkel in ein Array speichern
  save(loopnumber);

  // Loopnumber erhoehen
  loopnumber = (loopnumber + 1) % 2560;

  // Looptime zuruecksetzen
  looptime = 0;
}


//********************************************************
//
// SMO berechnet xhat2 ueber xhat1
//
//********************************************************
// Schaetzt die Geschwindigkeit der Tauchzelle
double get_xhat2(double x1, int n) {
  xhat1 = get_xhat1(x1, n);
  xhat2 = xhat2_prev[n] + (double(iterationtime) / (double(1000) * tau)) * (-xhat2_prev[n] - rho * sat(xhat1 - x1, 1));
  xhat2_prev[n] = xhat2;
  return xhat2;
}

// Schaetzt die Tiefe der Tauchzelle
double get_xhat1(double x1, int n) {
  xhat1 = xhat1_prev[n] - (double(iterationtime) / double(1000)) * rho * sat(xhat1_prev[n] - x1, 1);
  xhat1_prev[n] = xhat1;
  return xhat1;
}

// Saettigungfunktion
double sat(double x, double gamma) {
  double y = max(min(1.0, x / gamma), -1.0);
  return y;
}


//********************************************************
//
// Reglergroessen werden berechnet
//
// Input: Druck P und/oder Tauchtiefe z
// Output: Winkel alpha vom Regler
//
//********************************************************
//Adaptive-Fuzzy-Feedback-Linearization
//For a non adaptive control, set phi_s to zero in the delcaration
double berechneAFFL() {
  double e = z - z_ziel; // tracking error
  double s = zpp + 2.0 * lambda * zpunkt + pow(lambda, 2) * e; // sliding variable

  d_hat = 0;

  Psi[0] = max(min((s - Dc[0]) / (Dc[1] - Dc[0]), min(1, (Dc[3] - s) / (Dc[3] - Dc[2]))), 0); //Trapezoidal MF on left side (Psi[0])
  for (int n = 1; n <= 5; ++n) { // Calculate the Triangular MF for Psi[1] to Psi[5]
    Psi[n] = max(min((s - Dc[n + 1]) / (Dc[n + 2] - Dc[n + 1]), (Dc[n + 3] - s) / (Dc[n + 3] - Dc[n + 2])), 0); //Triangular MF
  }
  Psi[6] = max(min((s - Dc[7]) / (Dc[8] - Dc[7]), min(1, (Dc[10] - s) / (Dc[10] - Dc[9]))), 0); //Trapezoidal MF on right side (Psi[6])
  for (int n = 0; n <= 6; ++n) { // Calculating the dot-product
    d_hat += Psi[n] * D[n]; // add the product of each row
  }

  for (int n = 0; n <= 6; ++n) { // Integration based on rk4
    D[n] -= (double(iterationtime) / double(1000)) * phi_s * s * Psi[n];
  }

  double V_W_act = A_Membran * h_max * winkel / alpha_max; // Actual displaced Water-Volume

  double V_W = (1.0 / (a * rho_W * g)) * (a * rho_W * g * V_W_act - 2.0 * k * zpp * abs(zpunkt) - m_K * (zppp_des - 3.0 * lambda * zpp - 3.0 * pow(lambda, 2) * zpunkt - pow(lambda, 3) * e + d_hat));

  alpha = V_W / (A_Membran * h_max);

  alpha = alpha_max * sat(alpha / alpha_max, 1);

  return alpha;
}

//********************************************************
//
// Motor wird angesteuert
//
//********************************************************
// Ein Winkel wird explizit angefahren, solange die Zeit ausreicht
double step(double alpha) {
  digitalWrite(SLEEP_Pin, HIGH); // ON
  winkel = (double(Schrittzahl) * double(schrittwinkel)) / (double(iplanet) * double(6)) - alpha_max;
  double dalpha = alpha - winkel; // Winkeldifferenz zum Stellen
  int n = 6 * (int)abs((dalpha) * iplanet / schrittwinkel); // Anzahl der benoetigten Schritte des Motors
  int i = 0; // Lokaler Schrittzaehler
  // Wenn noch genug Zeit in der Loop ist, soll der Motor solange arbeiten, bis die Zeit abgelaufen ist
  while (looptime < iterationtime - 1) {
    // Fuer den Fall, dass alpha < -alpha_max soll zusaetzlich mit Hilfe der Endlagen getestet werden, ob die Position stimmt
    if (alpha > -alpha_max) {
      if (dalpha > 0) {
        digitalWrite(DIR_Pin, HIGH);
        while (i <= n && looptime < iterationtime - 1) {
          digitalWrite(STEP_Pin, HIGH);
          delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
          digitalWrite(STEP_Pin, LOW);
          delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
          ++Schrittzahl; // Schrittzahl um 1 erhoehen (ehemals verringern)
          ++i;
        }
      }
      else if (dalpha < 0) {
        digitalWrite(DIR_Pin, LOW);
        while (i <= n && looptime < iterationtime - 1) {
          digitalWrite(STEP_Pin, HIGH);
          delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
          digitalWrite(STEP_Pin, LOW);
          delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
          --Schrittzahl; // Schrittzahl um 1 verringern (ehemals erhoehen)
          ++i;
        }
      }
      else { // Fuer den Fall, dass dalpha = 0 soll keine Aktion ausgefuehrt werden
      }
    }
    else { //alpha = alpha_max, Endlage wird ueberprueft
      digitalWrite(DIR_Pin, LOW);
      while (digitalReadFast(Endlagen_Pin) == HIGH && looptime < iterationtime - 1) { // Endlage noch nicht erreicht
        digitalWrite(STEP_Pin, HIGH);
        delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
        digitalWrite(STEP_Pin, LOW);
        delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
        --Schrittzahl;
      }
      if (digitalRead(Endlagen_Pin) == LOW) {
        Schrittzahl = 0;
      }
    }
  }
  winkel = (double(Schrittzahl) * double(schrittwinkel)) / (double(iplanet) * double(6)) - alpha_max;
  return winkel;
}

// Die Zahnstange wird gegen den Endlagenschalter gefahren
// und die Schrittzahl zurueckgesetzt
int resetPosition() {
  digitalWrite(SLEEP_Pin, HIGH);
  digitalWrite(DIR_Pin, LOW);
  while (digitalReadFast(Endlagen_Pin) == HIGH) {
    digitalWrite(STEP_Pin, HIGH);
    delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
    digitalWrite(STEP_Pin, LOW);
    delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
    delay(1); // Nach jedem Schritt 1 ms Pause, um langsamer anzufahren und den Endlagenschalter nicht zu beschaedigen
  }
  digitalWrite(DIR_Pin, HIGH); // Nulllage anfahren
  for (int n = 0; n < 4900; ++n) {
    digitalWrite(STEP_Pin, HIGH);
    delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
    digitalWrite(STEP_Pin, LOW);
    delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
  }
  return 4900; // Wenn Nulllage erreicht, Schrittzahl auf 4900 setzen
}


//********************************************************
//
// Daten werden in ein Array geschrieben. Wenn das Array
// voll ist, taucht die Tauchzelle wieder auf.
//
// ACHTUNG! ALLE WERTE WERDEN MIT 10 000 SKALIERT
// UND ANSCHLIESSEND IN LONG INTEGER VERWANDELT!
//
// Speicherbedarf von long: 4 byte
// Speicherbedarf pro Datensatz: 4*4 byte = 16 byte
// EEPROM-Groesse: 2 kibyte
// Maximale Anzahl an Datensaetzen: 2048 byte / 16 byte = 128
// Wert-Quadrupel pro Sekunde: 2
// Maximale Tauchlaenge: 128 / 2 = 64 s
//
//********************************************************
void WritetoArray() {
  int skalierung = 10000;
  long zArray = (long)(skalierung * z);
  long zpunktArray = (long)(skalierung * zpunkt);
  long alphaArray = (long)(skalierung * alpha);
  long winkelArray = (long)(skalierung * winkel);
  if (iArray < zeilenArrayMax) {
    datenArray[iArray][0] = zArray;
    datenArray[iArray][1] = zpunktArray;
    datenArray[iArray][2] = alphaArray;
    datenArray[iArray][3] = winkelArray;
    ++iArray;
  }
  else {
    // Daten in den EEPROM schreiben
    if (aufEEPROMgeschrieben == false) {
      WritetoEEPROM();
      aufEEPROMgeschrieben = true;
    }
    // iArray = 0;
    // Auftauchen
    int breakcounter = 0;
    while (Schrittzahl < 9000) {
      ++breakcounter;
      ++Schrittzahl;
      digitalWrite(STEP_Pin, HIGH);
      delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
      digitalWrite(STEP_Pin, LOW);
      delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
      Serial.println(F("up, up, up you go"));
      if (breakcounter > 9000) {
        break;
      }
    }
    digitalWrite(SLEEP_Pin, LOW);
    while (true) {
      Serial.begin(9600);
      for (int n = 0; n < 2560; ++n) {
        Serial.printf("%f, %f\n", alleDaten[n][0], alleDaten[n][1]);
      }
      Serial.println("----------");
    }
  }
  nsd = 0;
}


//********************************************************
//
// Daten werden in den EEPROM geschrieben
//
//********************************************************
void WritetoEEPROM() {
  byte data;
  int adress = 0;
  int  i = 0;
  while (adress < 2048) {
    for (int n = 0; n <= 3; ++n) {
      for (int a = 0; a <= 3; ++a) {
        data = (datenArray[i][n] >> ((3 - a) * 8)) & 0x000000ff;
        EEPROM.write(adress + a, data);
      }
      adress = adress + 4;
    }
    ++i;
  }
}

//********************************************************
//
// Alle Tiefen- und Winkeldaten in ein Array im RAM
// speichern.
// Ausgabe erfolgt in WritetoArray(); nach erfolgreichem
// auftauchen.
//
//********************************************************
void save(int l) {
  if ((l <= 2560) && (0 == alleDaten[l][0]) && (0 == alleDaten[l][1])) {
    alleDaten[l][0] = z;
    alleDaten[l][1] = winkel;
  }
}


//********************************************************
//
// Druck wird berechnet
//
// Declaration of Copyright
// Copyright (c) 2009 MEAS Switzerland
// Edited 2015 Johann Lange
// This C code is for starter reference only. It was based on the
// MEAS Switzerland MS56xx pressure sensor modules and Atmel Atmega644p
// microcontroller code and has been by translated Johann Lange
// to work with Teensy 3.1 microcontroller.
//
//********************************************************
// Der Drucksensor wird ausgewertet und daraus der Druck bestimmt
double berechneDruck() {
  SPI.beginTransaction(settings_PS);
  // delay required in µs: OSR_4096: 9100, OSR_2048: 4600, OSR_1024: 2300, OSR_512: 1200, OSR_256: 700
  D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_256, 700); // read uncompensated pressure, Conversation Command + OSR, delay in µs
  D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_256, 700); // read uncompensated temperature, Conversation Command + OSR, delay in µs

  // calcualte 1st order temperature (MS5803_01b 1st order algorithm), base for 2nd order temperature and pressure
  dT = D2 - C[5] * pow(2, 8); //Serial.print("dT = "); Serial.println(dT);
  OFF = C[2] * pow(2, 16) + dT * C[4] / pow(2, 7); //Serial.print("OFF = "); Serial.println(OFF);
  SENS = C[1] * pow(2, 15) + dT * C[3] / pow(2, 8); //Serial.print("SENS = "); Serial.println(SENS);

  T = (2000 + (dT * C[6]) / pow(2, 23)) / 100;
  P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;

  // calcualte 2nd order pressure and temperature (MS5803_01b 2nd order algorithm)
  if (T > 20) {
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;

    if (T > 45) {
      SENS2 -= pow(T - 4500, 2) / pow(2, 3);
    }
  }
  else {
    T2 = pow(dT, 2) / pow(2, 31);
    OFF2 = 3 * pow(100 * T - 2000, 2);
    SENS2 = 7 * pow(100 * T - 2000, 2) / pow(2, 3);

    if (T < 15) {
      SENS2 += 2 * pow(100 * T + 1500, 2);
    }
  }

  // Recalculate T, OFF, SENS based on T2, OFF2, SENS2
  T -= T2;
  OFF -= OFF2;
  SENS -= SENS2;

  P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;
  return P;
  SPI.endTransaction();
}

void cmd_reset(void) {
  digitalWrite(CS_Pin_PS, LOW); // pull CSB low to start the command
  SPI.transfer(CMD_RESET); // send reset sequence
  delay(3); // wait for the reset sequence timing
  digitalWrite(CS_Pin_PS, HIGH); // pull CSB high to finish the command
}

//brief preform adc conversion
//return 24bit result
unsigned long cmd_adc(char cmd, int delaytime) {
  digitalWrite(CS_Pin_PS, LOW);
  unsigned long ret;
  unsigned long temp = 0;
  SPI.transfer(CMD_ADC_CONV + cmd); // send conversion command;
  cmd = SPI.transfer(0x00);
  delayMicroseconds(delaytime); // delay required in µs: OSR_4096: 9100, OSR_2048: 4600, OSR_1024: 2300, OSR_512: 1200, OSR_256: 700
  digitalWrite(CS_Pin_PS, HIGH);// pull CSB high to finish the conversion
  digitalWrite(CS_Pin_PS, LOW); // pull CSB low to start new command
  SPI.transfer(CMD_ADC_READ); // send ADC read command
  ret = SPI.transfer(0x00); // send 0 to read 1st byte (MSB)
  temp = 65536 * ret;
  ret = SPI.transfer(0x00); // send 0 to read 2nd byte
  temp = temp + 256 * ret;
  ret = SPI.transfer(0x00); // send 0 to read 3rd byte (LSB)
  temp = temp + ret;
  digitalWrite(CS_Pin_PS, HIGH); // pull CSB high to finish the read command
  return temp;
}

//brief Read calibration coefficients
//return coefficient
unsigned int cmd_prom(char coef_num) {
  unsigned int ret;
  unsigned int rC = 0;

  digitalWrite(CS_Pin_PS, LOW); // pull CSB low
  SPI.transfer(CMD_PROM_RD + coef_num * 2); // send PROM READ command
  ret = SPI.transfer(0x00); // send 0 to read the MSB
  rC = 256 * ret;
  ret = SPI.transfer(0x00); // send 0 to read the LSB
  rC = rC + ret;
  digitalWrite(CS_Pin_PS, HIGH);// pull CSB high
  return rC;
}

//brief calculate the CRC code for details look into CRC CODE NOTES
//return crc code
unsigned char crc4(unsigned int n_prom[]) {
  int cnt; // simple counter
  unsigned int n_rem; // crc reminder
  unsigned int crc_read; // original value of the crc
  unsigned char n_bit;
  n_rem = 0x00;
  crc_read = n_prom[7]; // save read CRC
  n_prom[7] = (0xFF00 & (n_prom[7])); // CRC byte is replaced by 0
  for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
  { // choose LSB or MSB
    if (cnt % 2 == 1) n_rem ^= (unsigned short) ((n_prom[cnt >> 1]) & 0x00FF);
    else n_rem ^= (unsigned short) (n_prom[cnt >> 1] >> 8);
    for (n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & (0x8000))
      {
        n_rem = (n_rem << 1) ^ 0x3000;
      }
      else
      {
        n_rem = (n_rem << 1);
      }
    }
  }
  n_rem = (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
  n_prom[7] = crc_read; // restore the crc_read to its original place
  return (n_rem ^ 0x00);
}
