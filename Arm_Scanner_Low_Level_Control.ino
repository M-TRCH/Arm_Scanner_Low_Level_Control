// Tested by Teensy 3.5 @ 24MHz
class SmartStepper
{
  private:
    /* i/o pin */
    uint8_t enablePin, resetPin, sleepPin, dirPin, stepPin, select1Pin, select2Pin, select3Pin, swPin;
    /* timer */
    signed long startTime, currentTime, intervalTime, intervalTimeAccel, intervalTimeDecel, intervalTimeMax, intervalTimeMin; 
    /* parameter */
    uint8_t mode;
    float velocity;
    uint16_t StepPerRev = 0;
    float D_pulley = 0;
    int8_t dirInvert;
    #define INVERT  1
    #define NON_INVERT -1
    #define CW  1
    #define CCW -1
    /* in procress */
    boolean firstRun = true;
    float currentPos = 0, goalPos;
    float realPos, originPos;
    long distance, currentDis = 0;
    boolean runFinished = false; 
    /* step per rev multiplier */
    #define FULL_STEP 1 
    #define HALF_STEP 2
    #define QUARTER_STEP 4
    #define EIGHTH_STEP 8
    #define SIXTEENTH_STEP 16
    void on()
    { /* Driver as enable */
      digitalWrite(enablePin, LOW);
      digitalWrite(resetPin, HIGH);      
      digitalWrite(sleepPin, HIGH);
    }
    void off()
    { /* Driver as disable */
      digitalWrite(enablePin, HIGH);
      digitalWrite(resetPin,   LOW);     
      digitalWrite(sleepPin,   LOW); 
    }
  public:
    SmartStepper(uint8_t enable_p,  uint8_t reset_p,   uint8_t sleep_p,   uint8_t dir_p, uint8_t step_p, 
                 uint8_t select1_p, uint8_t select2_p, uint8_t select3_p, uint8_t sw_p)
    { /* constuctor config */
      enablePin  = enable_p;
      resetPin   = reset_p;
      sleepPin   = sleep_p;
      dirPin     = dir_p;
      stepPin    = step_p;
      select1Pin = select1_p;
      select2Pin = select2_p;
      select3Pin = select3_p;
      swPin      = sw_p;
                    
      pinMode(enablePin,  OUTPUT);
      pinMode(resetPin,   OUTPUT);
      pinMode(sleepPin,   OUTPUT);
      pinMode(dirPin,     OUTPUT);
      pinMode(stepPin,    OUTPUT);
      pinMode(select1Pin, OUTPUT);
      pinMode(select2Pin, OUTPUT);
      pinMode(select3Pin, OUTPUT);
      pinMode(swPin,       INPUT);
    }
    void setParam(uint8_t m, float d, uint16_t step_rev, int8_t dir_invert)
    { /* parameter config */
      mode = m;
      switch (m)
      {  
        case FULL_STEP: 
          digitalWrite(select1Pin,  LOW); digitalWrite(select2Pin,  LOW); digitalWrite(select2Pin,  LOW); break;
        case HALF_STEP:
          digitalWrite(select1Pin, HIGH); digitalWrite(select2Pin,  LOW); digitalWrite(select2Pin,  LOW); break;
        case QUARTER_STEP:
          digitalWrite(select1Pin,  LOW); digitalWrite(select2Pin, HIGH); digitalWrite(select2Pin,  LOW); break;
        case EIGHTH_STEP:
          digitalWrite(select1Pin, HIGH); digitalWrite(select2Pin, HIGH); digitalWrite(select2Pin,  LOW); break;
        case SIXTEENTH_STEP:
          digitalWrite(select1Pin, HIGH); digitalWrite(select2Pin, HIGH); digitalWrite(select2Pin, HIGH); break;   
      }
      D_pulley = d;
      StepPerRev = step_rev;
      dirInvert = dir_invert;
    }
    void setSpeed(float  vel)
    { /* speed config (mm/s) */
      velocity = (vel*StepPerRev*mode)/(PI*D_pulley);
      intervalTime = 1000000/velocity;  
    }
    void setPosition(float goal)
    { /* position config (mm) */
      goalPos = goal;
      originPos = currentPos;
      distance = ((goalPos-currentPos)*StepPerRev*mode)/(PI*D_pulley);
           if(distance * dirInvert > 0) digitalWrite(dirPin, HIGH);
      else if(distance * dirInvert < 0) digitalWrite(dirPin,  LOW);
      runFinished = false;
      currentDis = 0;
    }
    void setZeroPos()
    { /* zero position adjust */
      currentDis = 0;
      currentPos = 0;
    }
    boolean runSpeed()
    { /* pulse generator */
      if(firstRun)
      {
        on();
        firstRun = false;
        runFinished = false;
        startTime = micros();
      }
      currentTime = micros();
           if(currentTime-startTime > intervalTime)  { firstRun = true;             return true;  }
      else if(currentTime-startTime > intervalTime/2){ digitalWrite(stepPin,  LOW); return false; }           
      else                                           { digitalWrite(stepPin, HIGH); return false; }          
    }
    void run()
    { /* run in loop */
      if(!runFinished && runSpeed())
      {
        currentDis++;
      }
      if(runFinished || currentDis >= fabs(distance))
      {
        off();
        runFinished = true;
        realPos = originPos + ((float)currentDis / StepPerRev / mode * PI * D_pulley * (distance >= 0 ? 1 : -1)); 
        currentPos = realPos;
        currentDis = 0;
      }
    }
    void runEndless(int8_t dir)
    { /* endless run */
           if(dir > 0) digitalWrite(dirPin, HIGH);
      else if(dir < 0) digitalWrite(dirPin,  LOW);
      runSpeed();
    }
    void goalPosition(float goal)
    { /* run until target position */
      setPosition(goal);
      while(!isRunFinished()) run(); 
    }
    void stop()
    { /* stop */
      off();
    }
    bool isLimited()
    { /* limited switch safety */
      if(digitalRead(swPin)){ runFinished = true; return true; }
      else return false;
    }
    bool isRunFinished()
    { /* update status */
      return runFinished;
    }
    float getCurrentPos()
    {
      return currentPos;
    }
    void forceFinished()
    {
      runFinished = true;
    }
};

// output pin config
#define enable_  36
#define reset_   38
#define sleep_   37
#define select1 35
#define select2 34
#define select3 33
#define green_pin 2
#define red_pin 3
#define sw_pin 26
// parameter config
#define pulley_diameter 12.75
#define stepRev 200
#define homeSpd 180
#define normalSpd 200
#define riskPos 20
int msg[8] = {0};

// pin config >> enable, reset, sleep, dir, step, select1, select2, select3, switch
SmartStepper motor02(enable_, reset_, sleep_, 12, 10, select1, select2, select3, 30); // tag 5
SmartStepper motor04(enable_, reset_, sleep_, 11,  9, select1, select2, select3, 31); // tag 3
SmartStepper motor06(enable_, reset_, sleep_, 14, 20, select1, select2, select3, 32); // tag 0 
SmartStepper motor08(enable_, reset_, sleep_, 17, 23, select1, select2, select3, 27); // tag 8 
SmartStepper motor10(enable_, reset_, sleep_, 16, 22, select1, select2, select3, 28); // tag 7 
SmartStepper motor12(enable_, reset_, sleep_, 15, 21, select1, select2, select3, 29); // tag 4

/* laser sensor */
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X laser;

volatile boolean serialUpdate = false;

void serialEvent1()
{
  if(Serial1.find('#'))
  { /* get commands from serial comunication. */ 
    // example packet -> "# 10 20 30 40 50 60 1 211"
    delay(10);
    int sum = 0;
    for(int i=0; i<7; i++)
    {
      msg[i] = Serial1.parseInt();
      sum += msg[i]; 
      delay(1);
    }
    msg[7] = Serial1.parseInt();
    
    if(sum == msg[7])
    {
      serialUpdate = true;
    }
  }
}

void setup() 
{    
  Serial.begin(9600);
  pinMode(green_pin, OUTPUT); // green pilot lamp 
  pinMode(red_pin, OUTPUT);   // red pilot lamp
  pinMode(sw_pin, INPUT);     // reset switch                                                                                                                                                                               
  
  // constant config >> mode, diameter, step/rev, invert direction?
  motor02.setParam(FULL_STEP, pulley_diameter, stepRev, NON_INVERT);  
  motor04.setParam(FULL_STEP, pulley_diameter, stepRev, INVERT);  
  motor06.setParam(FULL_STEP, pulley_diameter, stepRev, NON_INVERT);  
  motor08.setParam(FULL_STEP, pulley_diameter, stepRev, NON_INVERT);  
  motor10.setParam(FULL_STEP, pulley_diameter, stepRev, INVERT);  
  motor12.setParam(FULL_STEP, pulley_diameter, stepRev, INVERT);  
  
  Serial1.begin(9600);  Serial1.setTimeout(10);
  goHomeFrontAll(homeSpd);   

  /* laser sensor */
  Wire.begin(); Wire.setSCL(7); Wire.setSDA(8);
  laser.setTimeout(500);
  if(!laser.init())
    Serial.println("Failed to initialize!");
  laser.setMeasurementTimingBudget(200000);  // increase timimg budget
  Serial.println(laser.readRangeSingleMillimeters());
  
}
 

/* red pilot lamp control @ camera processing */
unsigned long blinkTimer = millis(); 
boolean l_state = true, blinkState = false;
     
void loop() 
{ 
  emergencyCheck();
  
  if(serialUpdate)
  { 
    serialUpdate = false;
    goPosAll(normalSpd, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5]);
    switch(msg[6])
    {
      case 0:
        blinkState = false; 
        break;
      case 1:
        blinkState = true;  
        break;
      case 2:
        Serial1.write('@');
        Serial1.println(laser.readRangeSingleMillimeters());
        break;
    }
  }
  if(blinkState)  cameraProcessing();
}
void cameraProcessing()
{
  digitalWrite(green_pin,LOW);
  digitalWrite(red_pin, l_state); 
  if(millis()-blinkTimer > 500)
  {
    blinkTimer = millis();
    l_state = !l_state;
  }
}
void goHomeFrontAll(float spd)
{ /* go to home all line */
  boolean m1 = true, m2 = true, m3 = true, m4 = true, m5 = true, m6 = true;
  motor02.setSpeed(spd);
  motor04.setSpeed(spd);
  motor06.setSpeed(spd);
  motor08.setSpeed(spd);
  motor10.setSpeed(spd);
  motor12.setSpeed(spd);

  while(m1 || m2 || m3 || m4 || m5 || m6) 
  {
    if(digitalRead(sw_pin)) return;
  
    if(!motor02.isLimited())
      motor02.runEndless(CW); 
    else m1 = false;   
    if(!motor04.isLimited())
      motor04.runEndless(CCW); 
    else m2 = false; 
    if(!motor06.isLimited())
      motor06.runEndless(CW); 
    else m3 = false; 
    if(!motor08.isLimited())
      motor08.runEndless(CW); 
    else m4 = false; 
    if(!motor10.isLimited())
      motor10.runEndless(CCW); 
    else m5 = false; 
    if(!motor12.isLimited())
      motor12.runEndless(CCW); 
    else m6 = false;  
  }
  #define delayTime 100 ///////////////////////////////////////////
  unsigned long delayTimer = 0;
  m1 = true, m2 = true, m3 = true, m4 = true, m5 = true, m6 = true;
  while(m1 || m2 || m3 || m4 || m5 || m6)  
  {
    if(digitalRead(sw_pin)) return;
    
    if(motor02.isLimited())
    {
      delayTimer = millis();
      while(motor02.isLimited() || millis()-delayTimer <= delayTime)
      {
        runEndlessBackAll(normalSpd); 
        if(digitalRead(sw_pin)) return;
      }
      break;
    }
    else m1 = false;   
    
    if(motor04.isLimited())
    {
      delayTimer = millis();
      while(motor04.isLimited() || millis()-delayTimer <= delayTime)
      {
        runEndlessBackAll(normalSpd); 
        if(digitalRead(sw_pin)) return;
      }
      break;
    }
    else m2 = false; 

    if(motor06.isLimited())
    {  
      delayTimer = millis();
      while(motor06.isLimited() || millis()-delayTimer <= delayTime)
      {
        runEndlessBackAll(normalSpd); 
        if(digitalRead(sw_pin)) return;
      }
      break;
    } 
    else m3 = false; 

    if(motor08.isLimited())
    { 
      delayTimer = millis();
      while(motor08.isLimited() || millis()-delayTimer <= delayTime)
      {
        runEndlessBackAll(normalSpd);
        if(digitalRead(sw_pin)) return; 
      }
      break;
    }
    else m4 = false; 
    
    if(motor10.isLimited())
    { 
      delayTimer = millis();
      while(motor10.isLimited() || millis()-delayTimer <= delayTime)
      {
        runEndlessBackAll(normalSpd); 
        if(digitalRead(sw_pin)) return;
      }
      break;
    }
    else m5 = false; 
    
    if(motor12.isLimited())
    {  
      delayTimer = millis();
      while(motor12.isLimited() || millis()-delayTimer <= delayTime)
      {
        runEndlessBackAll(normalSpd); 
        if(digitalRead(sw_pin)) return;
      }
      break;
    }
    else m6 = false;  
  }
  
  motor02.setZeroPos();
  motor04.setZeroPos();
  motor06.setZeroPos();
  motor08.setZeroPos();
  motor10.setZeroPos();
  motor12.setZeroPos();
  delay(1000);
  digitalWrite(red_pin, LOW);
  digitalWrite(green_pin, HIGH);
}
void goPosAll(float spd, float pos1, float pos2, float pos3, float pos4, float pos5, float pos6)
{ /* action for move to goal position */
  digitalWrite(red_pin, HIGH); 
  digitalWrite(green_pin,LOW);
  boolean m1 = true, m2 = true, m3 = true, m4 = true, m5 = true, m6 = true;
  motor02.setSpeed(spd);
  motor04.setSpeed(spd);
  motor06.setSpeed(spd);
  motor08.setSpeed(spd);
  motor10.setSpeed(spd);
  motor12.setSpeed(spd);
  motor02.setPosition(pos1);
  motor04.setPosition(pos2);
  motor06.setPosition(pos3);
  motor08.setPosition(pos4);
  motor10.setPosition(pos5);
  motor12.setPosition(pos6);

  #define delayTime 100 ///////////////////////////////////////////
  unsigned long delayTimer = 0;
  while(m1 || m2 || m3 || m4 || m5 || m6)
  {
    if(pos1 < riskPos || pos2 < riskPos || pos3 < riskPos || pos4 < riskPos || pos5 < riskPos || pos6 < riskPos){ goHomeFrontAll(homeSpd); break; }
    if(digitalRead(sw_pin)) return;
   
    if(!motor02.isRunFinished())
    {
      if(motor02.isLimited())
      {
        forceFinished();
        delayTimer = millis();
        while(motor02.isLimited() || millis()-delayTimer <= delayTime)
        {  
          runEndlessFrontAll(normalSpd);
          if(digitalRead(sw_pin)){ motor02.run(); return; } 
        }
        motor02.run();
        return; 
      } 
      motor02.run();
    }
    else  m1 = false;
       
    if(!motor04.isRunFinished())
    {
      if(motor04.isLimited())
      {
        forceFinished();
        delayTimer = millis();
        while(motor04.isLimited() || millis()-delayTimer <= delayTime)
        {
          runEndlessFrontAll(normalSpd); 
          if(digitalRead(sw_pin)){ motor04.run(); return; } 
        }
        motor04.run();
        return;
      }
      motor04.run();
    }
    else  m2 = false;
  
    if(!motor06.isRunFinished())
    {
      if(motor06.isLimited())
      {
        forceFinished();
        delayTimer = millis();
        while(motor06.isLimited() || millis()-delayTimer <= delayTime)
        {
          runEndlessFrontAll(normalSpd); 
          if(digitalRead(sw_pin)){ motor06.run(); return; } 
        }
        motor06.run();
        return;
      }
      motor06.run();
    }
    else  m3 = false;

    if(!motor08.isRunFinished())
    {
      if(motor08.isLimited())
      {       
        forceFinished();
        delayTimer = millis();
        while(motor08.isLimited() || millis()-delayTimer <= delayTime)
        {
          runEndlessFrontAll(normalSpd); 
          if(digitalRead(sw_pin)){ motor08.run(); return; } 
        }
        motor08.run();
        return;
      }
      motor08.run(); 
    }
    else  m4 = false;

    if(!motor10.isRunFinished())
    {
      if(motor10.isLimited())
      {
        forceFinished();
        delayTimer = millis();
        while(motor10.isLimited() || millis()-delayTimer <= delayTime)
        {
          runEndlessFrontAll(normalSpd); 
          if(digitalRead(sw_pin)){ motor10.run(); return; } 
        }
        motor10.run();
        return;
      }
      motor10.run();
    }
    else  m5 = false;

    if(!motor12.isRunFinished())
    {
      if(motor12.isLimited())
      {
        forceFinished();
        delayTimer = millis();
        while(motor12.isLimited() || millis()-delayTimer <= delayTime)
        {
          runEndlessFrontAll(normalSpd); 
          if(digitalRead(sw_pin)){ motor12.run(); return; } 
        }
        motor12.run();
        return;
      }
      motor12.run();
    }
    else  m6 = false;
  }
}
void stopAll()
{
  motor02.stop();
  motor04.stop();
  motor06.stop();
  motor08.stop();
  motor10.stop();
  motor12.stop();
}
void emergencyCheck()
{
  if(digitalRead(sw_pin))
  { /* emergency condition */
    stopAll();
    Serial1.flush();
    digitalWrite(green_pin,LOW);
    unsigned long blinkTimer = millis(); 
    boolean l_state = true;
    while(digitalRead(sw_pin))
    {
      digitalWrite(red_pin, l_state); 
      if(millis()-blinkTimer > 500)
      {
        blinkTimer = millis();
        l_state = !l_state;
      }
    }
    goHomeFrontAll(homeSpd);       
  }
}
void forceFinished()
{
  motor02.forceFinished();
  motor04.forceFinished();
  motor06.forceFinished();
  motor08.forceFinished();
  motor10.forceFinished();
  motor12.forceFinished();
  motor02.run();
  motor04.run();
  motor06.run();
  motor08.run();
  motor10.run();
  motor12.run();
}
void runEndlessFrontAll(float spd)
{
  motor02.setSpeed(spd); 
  motor04.setSpeed(spd); 
  motor06.setSpeed(spd); 
  motor08.setSpeed(spd); 
  motor10.setSpeed(spd); 
  motor12.setSpeed(spd); 
    
  motor02.runEndless(CW); 
  motor04.runEndless(CCW);
  motor06.runEndless(CW); 
  motor08.runEndless(CW);
  motor10.runEndless(CCW);
  motor12.runEndless(CCW);
}
void runEndlessBackAll(float spd)
{
  motor02.setSpeed(spd); 
  motor04.setSpeed(spd); 
  motor06.setSpeed(spd); 
  motor08.setSpeed(spd); 
  motor10.setSpeed(spd); 
  motor12.setSpeed(spd); 
    
  motor02.runEndless(CCW); 
  motor04.runEndless(CW);
  motor06.runEndless(CCW); 
  motor08.runEndless(CCW);
  motor10.runEndless(CW);
  motor12.runEndless(CW);
}
