#include <IRremote.hpp>

/* ================= IR ================= */
#define IR_PIN 2

#define Button_1      0xBA45FF00   // Pump
#define Button_2      0xB946FF00   // NEMA17 Down
#define Button_3      0xB847FF00   // NEMA17 Up

#define Button_UP     0x18
#define Button_DOWN   0x52
#define Button_RIGHT  0x5A
#define Button_LEFT   0x08

#define Button_star   0xE916FF00   // AUTO
#define Button_widow  0xF20DFF00   // MANUAL

/* ================= LDR ================= */
#define LDR_UR A0
#define LDR_UL A1
#define LDR_DR A2
#define LDR_DL A3

#define Voltage_sensor_PIN A4
#define Current_sensor_PIN A5

/* ================= Limits ================= */
#define LIMIT_UP    6
#define LIMIT_DOWN  7

/* ================= Motors ================= */
#define NEMA34_PUL 10
#define NEMA34_DIR 11

#define ACT_RPWM 13
#define ACT_LPWM 14
#define ACT_REN  15
#define ACT_LEN  16

#define NEMA17_STEP 17
#define NEMA17_DIR  18
#define NEMA17_ENA  19


#define PUMP_PIN 20

/* ================= Flags ================= */
bool manualMode = false;

bool mRight=false, mLeft=false, mUp=false, mDown=false;
unsigned long lastIR = 0;
#define IR_TIMEOUT 150

/* ================= Power Monitoring ================= */
unsigned long lastPowerCheck = 0;
#define POWER_CHECK_INTERVAL 10800000UL   // 3 ساعات

float referencePower = -1;   // تتسجل أول مرة
#define POWER_THRESHOLD 0.7  // 70%


bool nemaBusy = false;

/* ================= Setup ================= */
void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_PIN);

  pinMode(LIMIT_UP, INPUT_PULLUP);
  pinMode(LIMIT_DOWN, INPUT_PULLUP);

  pinMode(NEMA34_PUL, OUTPUT);
  pinMode(NEMA34_DIR, OUTPUT);

  pinMode(ACT_RPWM, OUTPUT);
  pinMode(ACT_LPWM, OUTPUT);
  pinMode(ACT_REN, OUTPUT);
  pinMode(ACT_LEN, OUTPUT);

  pinMode(NEMA17_STEP, OUTPUT);
  pinMode(NEMA17_DIR, OUTPUT);
  pinMode(NEMA17_ENA, OUTPUT);

  digitalWrite(NEMA17_ENA,HIGH);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
}

/* ================= Loop ================= */
void loop() 
{

  if (IrReceiver.decode()) 
  {
    handleIR();
    IrReceiver.resume();
  }

  if (manualMode) 
  {
    manualTracking();
  } 
  else 
  {
    autoTracking();
    powerCheckTask();   
  }
}

void powerCheckTask() 
{
  if (millis() - lastPowerCheck < POWER_CHECK_INTERVAL)
    return;

  lastPowerCheck = millis();

  float V = readVoltage();
  float I = readCurrent();
  float P = V * I;

  if (referencePower < 0) 
  {
    referencePower = P;   // أول مرة
    Serial.println("Reference power stored");
    return;
  }

  if (P <= referencePower * POWER_THRESHOLD) 
  {
    autoCleaningCycle();
  }
}

float readVoltage() 
{
  int v = analogRead(Voltage_sensor_PIN);
  return v * (5.0 / 1023.0);   
}

float readCurrent() 
{
  int i = analogRead(Current_sensor_PIN);
  return i * (5.0 / 1023.0);   
}

void autoCleaningCycle() 
{
  runPump();           
  moveNema17(false);   // Down
  delay(200);
  moveNema17(true);    // Up
}

/* ================= IR ================= */
void handleIR() 
{
  unsigned long raw = IrReceiver.decodedIRData.decodedRawData;
  uint8_t cmd = IrReceiver.decodedIRData.command;

  if (raw == Button_star) 
  {
    manualMode = false;
    stopActuator();
    resetMoves();
    Serial.println("AUTO MODE");
    return;
  }

  if (raw == Button_widow) 
  {
    manualMode = true;
    stopActuator();
    resetMoves();
    Serial.println("MANUAL MODE");
    return;
  }

  if (!manualMode) return;

  lastIR = millis();

  if (cmd == Button_RIGHT) 
  { 
    mRight=true;
    mLeft=false; 
  }
  if (cmd == Button_LEFT)  
  { 
    mLeft=true;  
    mRight=false; 
  }
  if (cmd == Button_UP)    
  { 
    mUp=true;    
    mDown=false; 
  }
  if (cmd == Button_DOWN)  
  { 
    mDown=true;  
    mUp=false; 
  }

  if (raw == Button_1)
  {
    runPump();
    delay(500);
    moveNema17(false);
    //delay(100);
    moveNema17(true);
  }
  if (raw == Button_2) 
  {
    moveNema17(false);
  }
  if (raw == Button_3) 
  {
    moveNema17(true);
  }
}

/* ================= Manual Tracking ================= */
void manualTracking() 
{

  if (millis() - lastIR > IR_TIMEOUT) 
  {
    resetMoves();
    stopActuator();
  }

  if (mRight) moveStepper(HIGH);
  else if (mLeft) moveStepper(LOW);
  else if (mUp) moveUp();
  else if (mDown) moveDown();
}

/* ================= Auto Tracking ================= */
void autoTracking() 
{
  int UR = analogRead(LDR_UR);
  int UL = analogRead(LDR_UL);
  int DR = analogRead(LDR_DR);
  int DL = analogRead(LDR_DL);

  int LR = (UR+DR)/2 - (UL+DL)/2;
  int UD = (UR+UL)/2 - (DR+DL)/2;

  if (LR > 70) 
  {
    while (LR > 30) 
    { 
      moveStepper(HIGH); 
      LR = analogRead(LDR_UR)-analogRead(LDR_UL); 
    }
  }
  if (LR < -70) 
  {
    while (LR < -30) 
    { 
      moveStepper(LOW);  
      LR = analogRead(LDR_UR)-analogRead(LDR_UL); 
    }
  }
  if (UD > 70) 
  {
    while (UD > 30) 
    { 
      moveUp();  
      UD = analogRead(LDR_UR)-analogRead(LDR_DR); 
    }
  }
  if (UD < -70) 
  {
    while (UD < -30) 
    { 
      moveDown(); 
      UD = analogRead(LDR_UR)-analogRead(LDR_DR); 
    }
  }
  stopActuator();
}

/* ================= Cleaning ================= */
void runPump() 
{
  
  digitalWrite(PUMP_PIN, HIGH);
  delay(1000);
  digitalWrite(PUMP_PIN, LOW);
  
}

void moveNema17(bool up) 
{

  if (nemaBusy) return;
  nemaBusy = true;

  IrReceiver.stop();              // وقف IR

  digitalWrite(NEMA17_ENA, LOW);  // فعّل الدرايفر
  delayMicroseconds(100);           // ⭐ استقرار ENA

  digitalWrite(NEMA17_DIR, up);   // حدّد الاتجاه
  delayMicroseconds(100);          // ⭐ استقرار DIR (مهم جدًا)

  while (true) 
  {
    if (up && digitalRead(LIMIT_UP) == LOW) break;
    if (!up && digitalRead(LIMIT_DOWN) == LOW) break;

    digitalWrite(NEMA17_STEP, HIGH);
    delayMicroseconds(800);
    digitalWrite(NEMA17_STEP, LOW);
    delayMicroseconds(800);
  }

  digitalWrite(NEMA17_ENA, HIGH); // عطّل الدرايفر

  IrReceiver.start();             // رجّع IR
  nemaBusy = false;
}


/* ================= Motors ================= */
void moveStepper(bool dir) 
{
  digitalWrite(NEMA34_DIR, dir);
  digitalWrite(NEMA34_PUL, HIGH);
  delayMicroseconds(800);
  digitalWrite(NEMA34_PUL, LOW);
  delayMicroseconds(800);
}

void moveUp() 
{
  digitalWrite(ACT_REN, HIGH);
  digitalWrite(ACT_LEN, HIGH);
  analogWrite(ACT_RPWM, 130);
  analogWrite(ACT_LPWM, 0);
}

void moveDown() 
{
  digitalWrite(ACT_REN, HIGH);
  digitalWrite(ACT_LEN, HIGH);
  analogWrite(ACT_RPWM, 0);
  analogWrite(ACT_LPWM, 130);
}

void stopActuator() 
{
  digitalWrite(ACT_REN, LOW);
  digitalWrite(ACT_LEN, LOW);
  analogWrite(ACT_RPWM, 0);
  analogWrite(ACT_LPWM, 0);
}

void resetMoves() 
{
  mRight = mLeft = mUp = mDown = false;
}
