#include "SPISlave_T4.h"

SPISlave_T4 mySPI(0, SPI_8_BITS);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  mySPI.begin();
  //    mySPI.onReceive(myFunc);
}

void loop() {
  Serial.print("millis: "); Serial.println(millis());
  delay(1000);
}

void myFunc() {
  //Serial.println("START: ");
  while ( mySPI.available() ) {
    Serial.print("VALUE: ");
    Serial.println(mySPI.popr(), HEX);
  }
  //Serial.println("END");
}
