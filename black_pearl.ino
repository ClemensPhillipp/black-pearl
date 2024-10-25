#include <Servo.h>
#include <math.h>
#include "SPI.h"

Servo mainServo;

int default_position1 = 110;
int pos1 = default_position1;
int pos2 = default_position1;
int pos = 90;                             // Ausgangsposition (90 Grad)
int timeStep = 10;                        // Zeitschritt in Millisekunden
float t = 0;                              // Initiale Zeit
float g = 9.81;                           // Erdbeschleunigung in m/s^2
float L = 2;                              // Länge des Pendels in Metern (anpassen)
float omega;                              // Winkelgeschwindigkeit
float maxAmplitude = 90;                  // Maximale Amplitude in Grad
float amplitude = 0;                      // Anfangsamplitude
bool increasing = true;                   // Flag für die Amplitudenerhöhung
bool holdMaxAmplitude = false;            // Flag, um anzuzeigen, ob die Amplitude 10 Sekunden lang konstant gehalten wird
unsigned long maxAmplitudeStartTime = 0;  // Zeitpunkt, zu dem die maximale Amplitude erreicht wurde
bool simulationComplete = true;           // Flag, das angibt, ob die Simulation abgeschlossen ist
int rounds = 0;
int x1 = 70;
int x2 = 170;
int y = 80;

void setup() {
  Serial.begin(9600);
  mainServo.attach(4);
  mainServo.write(pos);
  omega = sqrt(g / L);
}

void loop() {
  if (simulationComplete) {
    rounds = 0;
    amplitude = 0;
    t = 0;
    increasing = true;
    holdMaxAmplitude = false;
    simulationComplete = false;
    maxAmplitudeStartTime = 0;
    delay(10000);
  } else {
    pos = 80 + amplitude * cos(omega * t);
    mainServo.write(pos);

    delay(timeStep);

    t += (float)timeStep / 1000;

    if (increasing) {
      amplitude += 0.1;
      if (amplitude >= maxAmplitude) {
        amplitude = maxAmplitude;
        increasing = false;
        holdMaxAmplitude = true;
        maxAmplitudeStartTime = millis();
      }
    } else if (holdMaxAmplitude) {
      if (millis() - maxAmplitudeStartTime >= 30000) {
        holdMaxAmplitude = false;
      }
    } else {
      amplitude -= 0.1;
      if (amplitude <= 0) {
        amplitude = 0;
        simulationComplete = true;
      }
    }
    if (t >= (2 * M_PI / omega)) {
      t = 0;
    }
  }
  rounds++;
}

