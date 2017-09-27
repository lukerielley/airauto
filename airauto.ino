#include <LCD5110_Graph.h>

// The PINS the 5110 LCD is connected to
const int DIG_LCD_SCK = 8;
const int DIG_LCD_MOSI = 9;
const int DIG_LCD_DC = 10;
const int DIG_LCD_RST = 11;
const int DIG_LCD_CS = 12;

const int RELAY_INFLATE = 3;
const int RELAY_DEFLATE = 4;

// The analog pin of the pressure transducer
const uint8_t TRANSDUCER_PIN = A0;
// The amount of PSI that are represented by each volt of the transducer
const float psiPerVolt = 37.5;
// How many PSI per BAR (to allow unit conversion)
const float psiPerBar = 14.5038;
// The time duration (in MS) of the initial inflation
const int initialInflationTime = 1000;
// The time duration (in MS) of the initial deflation
const int initialDeflationTime = 1000;
// The minimum PSI that we require to determine that a tyre is connected
const float tyreConnectionMinPsi = 3.0;

LCD5110 myGLCD(DIG_LCD_SCK, DIG_LCD_MOSI, DIG_LCD_DC, DIG_LCD_RST, DIG_LCD_CS);

extern uint8_t SmallFont[];
extern uint8_t BigNumbers[];

// Indicates the tolerance between the target pressure and the actual pressure
const float accuracyTolerance = 0.5;
// Indicates that the unit is inflating the tyre to reach the desired pressuure, and that the air inlet valve is open
boolean isInflating = false;
// Indicates that the unit is deflating the tyre to reach the desired pressure, and that the ventilation valve is open
boolean isDeflating = false;
// Indicates that the unit has been told by the user that they want to achieve the target air pressure. It has been 'switched on'.
boolean isSwitchedOn = false;
// Indicates that an inflation/deflation session has started
boolean hasActiveSession = false;
// Indicates that the unit detcts that it is connected to a tyre. This is generally done by detecting a pressure above a certain level.
boolean hasTyreConnected = false;
// Indicates that inflating/deflating was attempted, but failed. (The pressure didn't change, indicating a problem with the compressor, a connection, or the tyre)
boolean inflationError = false;
// Indicates that the pressure reading is erratic. Sensor is not connected properly, or has a failure.
boolean isPressureReadingErratic = false;
// Indicates that the tyre is at the set pressure
boolean isAtSetPressure = false;
// The target set pressure of the tyre
float targetPressure = 0;
// An collection of the pressure readings of a session
float sessionReadings[] = {};

void setup()
{
  // Open Serial connection for debug output
  Serial.begin(9600);

  // Initialise the LCD screen
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);

  // Configure the I/O status of the pins we will use to control the solenoid valves
  pinMode(RELAY_INFLATE, OUTPUT);
  pinMode(RELAY_DEFLATE, OUTPUT);
}

/*
  The sensor sends 0.5v for 0PSI, 4.5V for 150ps giving us a 4v range.
  This function will send back an equalised value of 0-4 volts for the sensors total range.
*/
float getSensorEqualisedVoltage()
{
  int sensorValue = analogRead(TRANSDUCER_PIN);
  Serial.println(F("sensorValue"));
  float voltage = sensorValue * (5.0 / 1023.0);
  return voltage - 0.5;
}

// Gets the 
float getCurrentPsi()
{
  return getSensorEqualisedVoltage() * psiPerVolt;
}

void debugSensor()
{

  int sensorValue = analogRead(A0);

  Serial.println(F("sensorValue"));
  Serial.println(sensorValue);

  myGLCD.clrScr();
  myGLCD.setFont(BigNumbers);
  myGLCD.printNumI(sensorValue, CENTER, 10);

  myGLCD.update();

  delay(500);
}

void operationError()
{
  myGLCD.clrScr();
  myGLCD.print("ERROR!", CENTER, 10);
  myGLCD.update();

  for (int i = 0; i < 5; i++)
  {
    myGLCD.invert(true);
    delay(500);
    myGLCD.invert(false);
    delay(500);
  }
}

void inflate(int time)
{
  isInflating = true;
  Serial.println(F("Opening inflation solenoid"));
  digitalWrite(RELAY_INFLATE, LOW);
  Serial.println(F("Inflating"));
  delay(time);
  Serial.println(F("Closing inflation solenoid"));
  digitalWrite(RELAY_INFLATE, HIGH);
  delay(500);
  isInflating = false;
}

void deflate(int time)
{
  isDeflating = true;
  Serial.println(F("Opening deflation solenoid"));
  digitalWrite(RELAY_DEFLATE, LOW);
  Serial.println(F("Deflating"));
  delay(time);
  Serial.println(F("Closing deflation solenoid"));
  digitalWrite(RELAY_INFLATE, HIGH);
  delay(500);
  isDeflating = false;
}

void targetPressureAchieved() {
  // TODO: Play beeps?
  // TODO: Flash message on screen?
}

void sessionStart() {
  hasActiveSession = true;
}


void loop()
{

  if (!isSwitchedOn) {
    return;
  }

  // We are able to attempt to reach the target pressure with no issues
  if (inflationError || isPressureReadingErratic) {
    operationError();
  }

  // Read the current pressure
  float currentPsi = getCurrentPsi();

  // Determine if we have a tyre connected based on the pressure reading
  hasTyreConnected = (currentPsi > tyreConnectionMinPsi );
  
  if (!hasTyreConnected)
  {
    // If there is no tyre connected, but we have an active session, then the connection to the tyre has been lost before the target pressure was reached
    if (hasActiveSession) {
      // Cancel the active session
      hasActiveSession = false;
    }

    Serial.println(F("No tyre connection was detected"));
    return;
  }

  if (isAtSetPressure)
  {
    Serial.println(F("Tyre is at set pressure"));
    return;
  }

  // positive number means we need to inflate (eg target of 38 and current of 22 gives us 16)
  float adjustmentRequired = (targetPressure - currentPsi);
  
  // If there is no active session but we should start one, then let's do it
  if (!hasActiveSession && (adjustmentRequired > 0.5 || adjustmentRequired < -0.5)) {
    sessionStart();
  }

  if (adjustmentRequired > accuracyTolerance)
  {
    // TODO: Add logic to adjust the inflation time based on an estimation from previous inflation runs for this connection
    float inflationTime = initialInflationTime;
    hasActiveSession = true;
    inflate(inflationTime);
  }
  else if (adjustmentRequired < (-1 * accuracyTolerance))
  {
    // TODO: Add logic to adjust the inflation time based on an estimation from previous deflation runs for this connection
    float deflationTime = initialDeflationTime;
    hasActiveSession = true;
    deflate(deflationTime);
  }
  else
  {
    // The target pressure has been reached
    if (hasActiveSession) {
      targetPressureAchieved();
      hasActiveSession = false;
    }
    
    isAtSetPressure = true;
  }

  myGLCD.update();

  delay(100);

  // debugSensor();
}
