#include <QTRSensors.h>

QTRSensorsAnalog qtra((unsigned char[]) {35}, 1);
unsigned int sensorValues[1];

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
}

void loop() {
  qtra.read(sensorValues);
  Serial.println(sensorValues[0]);
}
