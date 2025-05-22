//Author: Sam Farkhooi

#include <project.hpp>
#include <Arduino.h>
#include <SPI.h>
 
//Make a reset at startup
void reset()
{

    // enable the motor and LEDs, and enable writes to the encoders
    byte mask = B01111111;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    digitalWrite(ss, HIGH);
    digitalWrite(ss, LOW);

    SPI.transfer(1);
    SPI.transfer(0);
    SPI.transfer(B01111111);  //Mask
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    digitalWrite(ss, HIGH);
    SPI.endTransaction();
}