#include <LCD5110_Graph.h>

const bool IS_DEVELOPMENT_MODE = true;

class Valve {
  int valvePinId;
  public:
  Valve(int connectionPin) {
    valvePinId = connectionPin;
  }

  void Open() {
    if (IS_DEVELOPMENT_MODE) {
      digitalWrite(valvePinId, HIGH);
    } else {
      digitalWrite(valvePinId, LOW);
    }
  }

  void Close() {
  if (IS_DEVELOPMENT_MODE) {
    digitalWrite(valvePinId, LOW);
  } else {
    digitalWrite(valvePinId, HIGH);
  }
  }
};

// Mega2560 Interrupt Pins = 2, 3, 18, 19, 20, 21

// The PINS the 5110 LCD is connected to
const int DIG_LCD_SCK = 8;
const int DIG_LCD_MOSI = 9;
const int DIG_LCD_DC = 10;
const int DIG_LCD_RST = 11;
const int DIG_LCD_CS = 12;

const int RELAY_INFLATE = 6;
const int RELAY_DEFLATE = 7;

Valve inflationValve(RELAY_INFLATE);
Valve deflationValve(RELAY_DEFLATE);

const byte PIN_BUTTON_A = 2;
const byte PIN_BUTTON_B = 3;
const byte PIN_BUTTON_C = 4;

// The states for the buttons. These are volatile as they are modified by an interrupt
volatile int BTN_A_STATE = 0;
volatile int BTN_B_STATE = 0;
volatile int BTN_C_STATE = 0;

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
extern uint8_t MediumNumbers[];
extern uint8_t BigNumbers[];

// Indicates the tolerance between the target pressure and the actual pressure
const float accuracyTolerance = 0.5;
// Indicates that the unit is inflating the tyre to reach the desired pressuure, and that the air inlet valve is open
boolean isInflating = false;
// Indicates that the unit is deflating the tyre to reach the desired pressure, and that the ventilation valve is open
boolean isDeflating = false;
// Indicates that the unit has been told by the user that they want to achieve the target air pressure. It has been 'switched on'.
boolean isSwitchedOn = true;
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
float targetPressure = 38;
// An collection of the pressure readings of a session, maximum of 20
float sessionReadings[21] = {};
// A record of how many inflation/deflation cycles were made
int sessionCycles = 0;

void setup()
{
  // Open Serial connection for debug output
  Serial.begin(9600);

  if (IS_DEVELOPMENT_MODE) {
    Serial.println("DEVELOPMENT MODE");
  }

  // Initialise the LCD screen
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);

  // Configure the I/O status of the pins we will use
  pinMode(RELAY_INFLATE, OUTPUT);
  pinMode(RELAY_DEFLATE, OUTPUT);

  // Set default states (both air solenoids closed
  inflationValve.Close();
  deflationValve.Close();
  
  pinMode(PIN_BUTTON_A, INPUT_PULLUP);
  pinMode(PIN_BUTTON_B, INPUT_PULLUP);
  pinMode(PIN_BUTTON_C, INPUT_PULLUP);

  // Register interrupt event handlers
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_A), buttonAClicked, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_B), buttonBClicked, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_C), buttonCClicked, CHANGE);

  welcome();
}

void welcome() {
  myGLCD.clrScr();
  myGLCD.print("AIRAUTO", CENTER, 10);
  myGLCD.update();
  delay(1000);
}

/*
  Interrupt event handler for button A
*/
void buttonAClicked() {
  BTN_A_STATE = digitalRead(PIN_BUTTON_A);
  Serial.println(F("BUTTON A CHANGED"));
}

/*
  Interrupt event handler for button B
*/
void buttonBClicked() {
  BTN_B_STATE = digitalRead(PIN_BUTTON_B);
  Serial.println(F("BUTTON B CHANGED"));
}

void buttonCClicked() {
  BTN_C_STATE = digitalRead(PIN_BUTTON_C);
  Serial.println(F("BUTTON C CHANGED"));
}

/*
  The sensor sends 0.5v for 0PSI, 4.5V for 150ps giving us a 4v range.
  This function will send back an equalised value of 0-4 volts for the sensors total range.
*/
float getSensorEqualisedVoltage()
{
  if (IS_DEVELOPMENT_MODE) {
    /* In development mode we use a potentiometer to allow us to 
    simulate tyre pressure readings between 0.5-2.5 volts (0-75 PSI) */
    int sensorValue = analogRead(TRANSDUCER_PIN);
    float val = ((1023 - sensorValue) * (1.5 / 1023));
    return val;
  } else {
    int sensorValue = analogRead(TRANSDUCER_PIN);
    float voltage = sensorValue * (5.0 / 1023.0);
    return voltage - 0.5;
  }
}

/*
  Get the current PSI. This is the sensor voltage multiplied by the PSI per volt
*/
float getCurrentPsi()
{
  float psi = getSensorEqualisedVoltage() * psiPerVolt;
  Serial.print(F("ACTUAL PSI: "));
  Serial.print(psi);
  Serial.println();
  return psi;
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

void operationError(int errorCode)
{
  Serial.println(F("ERROR!!!!!"));
  Serial.print(errorCode);
  hasActiveSession = false;
  
  myGLCD.clrScr();
  myGLCD.print("ERROR!", CENTER, 10);
  myGLCD.print("CODE", CENTER, 20);
  myGLCD.printNumI(errorCode, CENTER, 30);
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
  inflationValve.Open();
  Serial.println(F("Inflating"));
  delay(time);
  Serial.println(F("Closing inflation solenoid"));
  inflationValve.Close();
  delay(1000);
  isInflating = false;
}

void deflate(int time)
{
  isDeflating = true;
  Serial.println(F("Opening deflation solenoid"));
  deflationValve.Open();
  Serial.println(F("Deflating"));
  delay(time);
  Serial.println(F("Closing deflation solenoid"));
  deflationValve.Close();
  delay(1000);
  isDeflating = false;
}

void targetPressureAchieved() {
    hasActiveSession = false;
    resetSessionReadings();
    Serial.println(F("SESSION Complete: Target pressure achieved."));
    myGLCD.clrScr();
    myGLCD.setFont(SmallFont);
    myGLCD.print("COMPLETED", CENTER, 5);
    myGLCD.setFont(BigNumbers);
    myGLCD.printNumF(targetPressure, 0, CENTER, 20);
    myGLCD.update();
    for (int i = 0; i < 10; i++)
    {
      myGLCD.invert(true);
      delay(250);
      myGLCD.invert(false);
      delay(250);
    }
    delay(2000);
  // TODO: Play beeps?
  // TODO: Flash message on screen?
}

void sessionStart() {
  Serial.println(F("Session starting!!"));
  resetSessionReadings();
  hasActiveSession = true;
}

void resetSessionReadings() {
  for (int count=0;count<20;count++) {
    sessionReadings[count] = -1;
  }
}

void addSessionReading(float value) {
  Serial.print(F("Adding session reading of:"));
  Serial.print(value);
  Serial.println();
  sessionReadings[sessionCycles] = value;

  String outputReadings = "";
  for (int count=0;count<20;count++) {
    if (sessionReadings[count] != -1) {
      outputReadings = outputReadings + sessionReadings[count] + ", ";
    }
  }

  Serial.println(outputReadings);
  
  if (sessionCycles < 20) {
    sessionCycles++;
  } else {
    operationError(1);
  }
}

void loop()
{
  myGLCD.clrScr();
  
  Serial.println(F("---------------------------------------------------------"));
  delay(500);

  // Read the current pressure
  float currentPsi = getCurrentPsi();

  if (!isSwitchedOn) {
    Serial.println(F("Unit is not switched on, let's do nothing for now"));
    // return;
  }

//  // We are able to attempt to reach the target pressure with no issues
//  if (inflationError || isPressureReadingErratic) {
//    Serial.println(F("Something is wrong, we can't attempt anything :("));
//    operationError();
//  }

  // Determine if we have a tyre connected based on the pressure reading
  boolean detectTyre = (currentPsi > tyreConnectionMinPsi );

  Serial.print(F("TYRE DETECTION: "));
  Serial.print(detectTyre);
  Serial.println();

  Serial.print(F("TARGET PSI: "));
  Serial.print(targetPressure);
  Serial.println();


  myGLCD.setFont(SmallFont);
  myGLCD.print("READ", 0, 13);
  myGLCD.print("SET", 40, 13);

//  myGLCD.drawLine(0, 18, 30, 18);
//  myGLCD.drawLine(40, 18, 84, 18);
  
  myGLCD.setFont(BigNumbers);
  if (detectTyre) {
    myGLCD.printNumF(currentPsi, 0, 0, 24);
  } else {
    // myGLCD.printNumF(currentPsi, 0, 0, 10);
  }
  myGLCD.printNumF(targetPressure, 0, 42, 24);
  myGLCD.setFont(SmallFont);
  myGLCD.update();

  if (!detectTyre) {
    Serial.println(F("No tyre detcted"));
    // No tyre detected
    if (hasActiveSession) {
      // Tyre has been disconnected! Let's end the session
      hasActiveSession = false;
      Serial.println(F("SESSION END!! There was an active session but the tyre was disconnected!"));
    }
    return;
  }

  // positive number means we need to inflate (eg target of 38 and current of 22 gives us 16)
  float adjustmentRequired = (targetPressure - currentPsi);

  Serial.print(F("ADJUSTMENT PSI: "));
  Serial.print(adjustmentRequired);
  Serial.println();
  
  // If there is no active session but we should start one, then let's do it
  if (!hasActiveSession && detectTyre && (adjustmentRequired > 0.5 || adjustmentRequired < -0.5)) {
    sessionStart();
  }

  if (adjustmentRequired > accuracyTolerance)
  {
    // TODO: Add logic to adjust the inflation time based on an estimation from previous inflation runs for this connection
    float inflationTime = initialInflationTime;
    hasActiveSession = true;
    addSessionReading(currentPsi);
    inflate(inflationTime);
  }
  else if (adjustmentRequired < (-1 * accuracyTolerance))
  {
    // TODO: Add logic to adjust the inflation time based on an estimation from previous deflation runs for this connection
    float deflationTime = initialDeflationTime;
    hasActiveSession = true;
    addSessionReading(currentPsi);
    deflate(deflationTime);
  }
  else
  {
    // The target pressure has been reached
    if (hasActiveSession) {
      targetPressureAchieved();
    }
    Serial.println(F("Tyre is at set pressure"));
    isAtSetPressure = true;
  }

  myGLCD.update();

  

  // debugSensor();
}
