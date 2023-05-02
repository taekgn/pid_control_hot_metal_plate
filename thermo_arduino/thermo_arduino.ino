#include "PID_v1.h"
#include "max6675.h"

int thermoDO = 16;  // MISO -Rx
int thermoCS = 17;
int thermoCLK = 18;

static int pwm_pin = 22;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Define the aggressive and conservative Tuning Parameters
double aggKp = 50, aggKi = 0.1, aggKd = 5;
double consKp = 3, consKi = 1.5, consKd = 0.25;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


void setup() {
  Serial.begin(9600);
  //  initialize the variables we're linked to
  Input = thermocouple.readCelsius();
  Setpoint = 30.00;  // Goal Temperature
  // turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  int i = 0;
  myPID.SetTunings(aggKp, aggKi, aggKd);
  while (1) {
    Input = thermocouple.readCelsius();
    double gap = abs(Setpoint - Input);  // distance away from setpoint
if (gap < 0.5) {                     // we're close to setpoint, use conservative tuning parameters
 myPID.SetTunings(consKp, consKi, consKd);
} else {
    // we're far from setpoint, use aggressive tuning parameters
    // myPID.SetTunings(aggKp, aggKi, aggKd);
 }
    myPID.Compute();
    analogWrite(pwm_pin, Output);

    Serial.print(millis() / 1000);
    Serial.print(",");
    Serial.print(Output);
    Serial.print(",");
    Serial.println(thermocouple.readCelsius());
    i++;
    // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
    delay(1000);
  }
}