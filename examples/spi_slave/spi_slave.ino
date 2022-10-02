#include "SPISlave_T4.h"

SPISlave_T4 mySPI(0, SPI_8_BITS);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");
  mySPI.begin(LSBFIRST, SPI_MODE3);
  mySPI.onReceive(myFunc);
}

void loop() {
  Serial.print("millis: "); Serial.println(millis());
  delay(1000);
}

void myFunc() {
  Serial.print("[");
  while ( mySPI.available() ) {
    Serial.print((char)mySPI.popr());
    //Serial.println(mySPI.popr(), HEX);
  }
  Serial.println("]");
}
