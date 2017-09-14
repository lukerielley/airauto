
//      SCK  - Pin 8
//      MOSI - Pin 9
//      DC   - Pin 10
//      RST  - Pin 11
//      CS   - Pin 12
//

#include <LCD5110_Graph.h>

LCD5110 myGLCD(8,9,10,11,12);

extern uint8_t SmallFont[];
extern uint8_t BigNumbers[];

const float psiPerVolt = 37.5;
const float psiPerBar = 14.5038;

int readings[50];

/*
  Indicates that the unit is inflating the tyre to reach the desired pressuure, and that the air inlet valve is open
*/
boolean isInflating = false;

/*
  Indicates that the unit is deflating the tyre to reach the desired pressure, and that the ventilation valve is open
*/
boolean isDeflating = false;


/*
  Indicates that the unit has been told by the user that they want to achieve the target 
  air pressure. It has been 'switched on'.
*/
boolean isSwitchedOn = false;

/*
  Indicates that the unit detcts that it is connected to a tyre. This is generally done by 
  detecting a pressure above a certain level.
*/
boolean hasTyreConnected = false;

/* 
  Indicates that inflating/deflating was attempted, but failed. (The pressure didn't change,
  indicating a problem with the compressor, a connection, or the tyre) 
*/
boolean inflationError = false;



void setup()
{
  Serial.begin(9600);
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
}

/*
  The sensor sends 0.5v for 0PSI, 4.5V for 150ps giving us a 4v range.
  This function will send back an equalised value of 0-4 volts for the sensors total range.
*/
float getSensorEqualisedVoltage() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  return voltage - 0.5;
}

void error() {

  myGLCD.clrScr();
  myGLCD.print("ERROR!", CENTER, 20);
  myGLCD.update();
  
  for (int i=0; i<5; i++)
  {
    myGLCD.invert(true);
    delay(500);
    myGLCD.invert(false);
    delay(500);
  }
  
}

// void pumpItUp()
// {
 
//   delay(1000);

//   // Reset
//   isInflating = false;
//   isDeflating = false;

//   switch(demoMode) {
//     case 1: {
//       // Taking Read
//       isPausedForRead = true;
//       break;
//     }
//     case 2: {
//       // Deflating
//       isDeflating = true;
//       break;
//     }
//     case 3: {
//       // Inflating
//       isInflating = true;
//       break;
//     }
//     case 4: {
      
//       break;
//     }
//   }
  
  
//   myGLCD.clrScr();
  
  

//   voltage = voltage + calibrate_value;

  
//   float psi = (voltage - 0.5) * 37.5;
//   float bar = (psi / psiPerBar);

//   Serial.println(F("Sensor Value:"));
//   Serial.println(sensorValue);
//   Serial.println(F("Voltage:"));
//   Serial.println(voltage);
  
//   Serial.println(F("PSI:"));
//   Serial.println(psi);

//   Serial.println(F("BAR:"));
//   Serial.println(bar);

// //  myGLCD.print("AUTO AIR", CENTER, 0);
// //  myGLCD.printNumF(voltage, 5, 1, 10);
// //  myGLCD.print("VOLTS", 52, 10);
// //  myGLCD.printNumF(psi, 0, 1, 20);
// //  myGLCD.print("PSI", 52, 20);

//   myGLCD.print("SET", 40, 10);
//   myGLCD.print("READ", 0, 10);

//   // myGLCD.print("32 PSI", 40, 0);
  
//   if (isInflating) {
//     myGLCD.drawLine(0,40,20,20);
//     myGLCD.drawLine(20,20,40,40);
//     myGLCD.drawLine(0,40,40,40);
//   }

//   if (isDeflating) {
//     myGLCD.drawLine(0,20,20,40); // left side
//     myGLCD.drawLine(20,40,40,20); // right side
//     myGLCD.drawLine(0,20,40,20); // top line
//   }

//   if (!isInflating && !isDeflating) {
//     // Show the pressure reading
//     // myGLCD.printNumF(psi, 0, 40, 10);
//       myGLCD.setFont(BigNumbers);
//       myGLCD.print("22", 0, 20);
//       myGLCD.setFont(SmallFont);
//   }

//   myGLCD.setFont(BigNumbers);
//   myGLCD.print("38", 42, 20);
//   myGLCD.setFont(SmallFont);
  
//   myGLCD.update();

//   demoMode++;
//   if (demoMode >= 5) {
//     demoMode = 1;
//   }

// }


void loop()
{

  // pumpItUp();

}



