/*
   Description: This program is used to control Turt: a three wheeled omni-directional robot

        Wiring: The required components are 3x 28BYJ-48 stepper motors, 2x ULN2003 transistors, and a NRF24L01 radio module
        The stepper motors are connected to the ULN2003 pins 1C-6C and to Vin
        The ULN2003 is connected to Vin, Gnd, and pins 1B-4B are connected according to the stepper declarations below
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D10, MOSI to D11, CE to D9, SCK to D13, MISO to D12

*/

#include "AccelStepper.h"
AccelStepper stepperA(4, 4, 6, 5, 7);
AccelStepper stepperB(4, 2, 19, 3, 18);
AccelStepper stepperC(4, 17, 15, 16, 14);

#include "RF24.h"
RF24 radio(9, 10);
const byte chan[6] = "00007";
byte data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int xDir = 0;
int yDir = 0;
int zDir = 0;
int spd = 0;

unsigned long millisPrev = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("Serial Communication Initialized");
  
  radio.begin();                                                                                    //Begin radio communication
  radio.openReadingPipe(1, chan);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Radio Communication Initialized");

  stepperA.setMaxSpeed(950.0);                                                                      //Configure stepper motors
  stepperB.setMaxSpeed(950.0);
  stepperC.setMaxSpeed(950.0);
  Serial.println("Stepper Motors Set");

  delay(1000);
}

void loop() {
  if (radio.available()) {                                                                          
    radio.read(&data, sizeof(data));                                                                //Read in data from remote control

    if (data[0] == 0) xDir = 0;                                                                     //Left and right motion controlled by left to right motion of left joystick
    else xDir = map(data[0], 1, 255, -10, 10);

    if (data[1] == 0) yDir = 0;                                                                     //Forward and reverse controlled by front to back motion of left joystick
    else yDir = map(data[1], 1, 255, -10, 10);

    if (data[4] == 0) zDir = 0;                                                                     //Turning controlled by left to right motion of right joystick
    else zDir = map(data[4], 1, 255, -10, 10);

    spd = map(data[9], 0, 255, 25, 75);                                                             //Driving speed controlled by right potentiometer
  }
  
  else if ((millis() - millisPrev) > 100){                                                          //Recalculate wheel speeds every 100 milliseconds
    if (zDir ==0){
      stepperA.setSpeed(spd * ( 1.0*xDir + 0.0*yDir));                                              //Calculate wheel speeds based on desired translation of robot
      stepperB.setSpeed(spd * (-0.5*xDir - 0.9*yDir));
      stepperC.setSpeed(spd * (-0.5*xDir + 0.9*yDir));
    }
    else{
      stepperA.setSpeed(spd * ( 1.0*xDir + 0.0*yDir + zDir) / 2);                                   //Calculate wheel speeds based on desired translation and rotation of robot
      stepperB.setSpeed(spd * (-0.5*xDir - 0.9*yDir + zDir) / 2);
      stepperC.setSpeed(spd * (-0.5*xDir + 0.9*yDir + zDir) / 2);
    }
    
    millisPrev = millis();
  }  

  stepperA.runSpeed();
  stepperB.runSpeed();
  stepperC.runSpeed();
}
