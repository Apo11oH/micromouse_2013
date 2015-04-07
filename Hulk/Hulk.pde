/**********************************************
 *************** CONSTANTS ********************
 *********************************************/
#define ONE_CELL 31 // One cell
#define TURN_QTR 12 // A quarter turn
#define TURN_HALF 25 // A 180 turn
#define MAZE_MAX_X 16
#define MAZE_MAX_Y 16

// Constants for PID Control
//past KP 260, 240,230, 2000, 1650
//past KD 130, 50, 500
// Low speed:(2000, 500)
// Was working on: (1700, 100)
#define KP 1400
#define KD 500
#define DEFAULT_SPEED 15000
// margin of error will be within 1cm
#define CENTER_MARGIN .25
#define NO_WALL 7 //distance if no wall on left/right
#define WALL_FOLLOW 2.9 //distance when using wall follow

/**********************************************
 ************** PINS & VALUES *****************
 *********************************************/
// MotorPins
int motorSTBY = 32;
int motorAIN1 = 33; // Left
int motorAIN2 = 37;
int motorPWMA = 28;
int motorBIN1 = 25; // Right
int motorBIN2 = 29;
int motorPWMB = 24;

// SensorPins
int sensorFR = 15; 
int sensorFL = 18;
int frontLongSensor = 20;

// EncoderPins
int encoderLeft = 7;
int encoderRight = 5;

// PID values
double currentSpeed_R = DEFAULT_SPEED;
double currentSpeed_L = DEFAULT_SPEED;
double propValue = 0;
double oldPropValue = 0;
double newMotorSpeed = 0;
int oldTime = 0;
int currentTime = 0;
int derivative = 0;
int distanceFL = 0;
int distanceFR = 0;
int tmpFL = 0;
int tmpFR = 0;
int tmpFF = 0;

// Motor values
volatile int gc_right = 0;
volatile int gc_left = 0;

// Movement flags
boolean move = false;
boolean turn = false;
boolean flip = false;
int movingDirec = 0;

int mazeMap[MAZE_MAX_Y][MAZE_MAX_X];
int locx = 0;
int locy = MAZE_MAX_Y - 1;
int randomCount = 0;

/**********************************************
 ****************** SETUP *********************
 *********************************************/
void setup(){
  // Setup LED for...well..you know, flashy stuff X)
  pinMode(BOARD_LED_PIN, OUTPUT);
  // Setup motor pins
  pinMode(motorSTBY, OUTPUT);
  pinMode(motorAIN1, OUTPUT);
  pinMode(motorAIN2, OUTPUT);
  pinMode(motorPWMA, PWM);
  pinMode(motorBIN1, OUTPUT);
  pinMode(motorBIN2, OUTPUT);
  pinMode(motorPWMB, PWM);
  // Setup encoder pins
  pinMode(encoderRight, INPUT);
  pinMode(encoderLeft, INPUT);
  attachInterrupt(encoderRight, incCounterR, CHANGE);
  attachInterrupt(encoderLeft, incCounterL, CHANGE);
  // Setup sensor pins
  pinMode(sensorFR, INPUT_ANALOG);
  pinMode(sensorFL, INPUT_ANALOG);
  pinMode(frontLongSensor, INPUT_ANALOG);
  
  for(int i=0; i<MAZE_MAX_Y; i++){
    for(int j=0; j<MAZE_MAX_X; j++){
      mazeMap[i][j] = 0;
    }
  }
      
  //STBY==HIGH -> enables motors
  digitalWrite(motorSTBY, HIGH);
  for(int i=0; i<8; i++){
    toggleLED();
    delay(1000);
  }
}

/**********************************************
 *************** MAIN LOOP ********************
 *********************************************/
void loop(){
  
  //pin 8 test time
  //digitalWrite(timing, HIGH);
  toggleLED();
  pidHandler();
  
  if(move){
    switch(movingDirec){
      case 0: moveForward(); break;
      case 1: turnRight(); break;
      case 2: turnRight(); break;
      case 3: turnLeft(); break;
    }
  }else{
    randomCount++;
    moveStanby();
    movingDirec = chooseTurnSHand();
    delay(500);
    move = true;
  }
  
  //digitalWrite(timing, LOW);
}// end main loop *****************************

/**********************************************
 *************** MOVEMENT *********************
 *********************************************/ 
 
void moveStanby(){
  pwmWrite(motorPWMA, 0); 
  pwmWrite(motorPWMB, 0); 
}

void moveBackward(){
  digitalWrite(motorAIN1, LOW);
  digitalWrite(motorAIN2, HIGH);
  digitalWrite(motorBIN1, LOW);
  digitalWrite(motorBIN2, HIGH);
  pwmWrite(motorPWMA, currentSpeed_L);   
  pwmWrite(motorPWMB, currentSpeed_R); 
}

void moveForward(){
  digitalWrite(motorAIN1, HIGH);
  digitalWrite(motorAIN2, LOW);
  digitalWrite(motorBIN1, HIGH);
  digitalWrite(motorBIN2, LOW);
  pwmWrite(motorPWMA, currentSpeed_L);   
  pwmWrite(motorPWMB, currentSpeed_R); 
  if(analogRead(frontLongSensor) > 3500){
    move = false;
    gc_right = 0;
    gc_left = 0;
  }
}

void turnRight(){
  digitalWrite(motorAIN1, HIGH);
  digitalWrite(motorAIN2, LOW);
  digitalWrite(motorBIN1, LOW);
  digitalWrite(motorBIN2, HIGH);
  pwmWrite(motorPWMA, DEFAULT_SPEED);   
  pwmWrite(motorPWMB, DEFAULT_SPEED); 
}

void turnLeft(){
  digitalWrite(motorAIN1, LOW);
  digitalWrite(motorAIN2, HIGH);
  digitalWrite(motorBIN1, HIGH);
  digitalWrite(motorBIN2, LOW);
  pwmWrite(motorPWMA, DEFAULT_SPEED);   
  pwmWrite(motorPWMB, DEFAULT_SPEED); 
}

/***************chooseTurn**********************
 *  Choose if the bot should turn or not based on the 
 *  current direction and the direction the bot needs
 *  to turn next
 **********************************************/
int chooseTurnSHand(){
  tmpFF = analogRead(frontLongSensor);
  tmpFL = analogRead(sensorFL);
  tmpFR = analogRead(sensorFR);
  
  if(tmpFF < 3000){
    return 0; 
  }else if(tmpFR < 2000){
    turn = true;
    return 1;
  }else if(tmpFL < 2000){
    turn = true;
    return 3;
  }else{
    flip = true;
    return 2;
  }
}

int chooseTurnRHand(){
  tmpFF = analogRead(frontLongSensor);
  tmpFL = analogRead(sensorFL);
  tmpFR = analogRead(sensorFR);
  
  if(tmpFR < 2000){
    turn = true;
    return 1; 
  }else if(tmpFF < 3000){
    return 0;
  }else if(tmpFL < 2000){
    turn = true;
    return 3;
  }else{
    flip = true;
    return 2;
  }
}

/**********************************************
 *************** ENCODER COUNT ****************
 *********************************************/
void incCounterR(){
  gc_right++;
  if(turn){
    if((gc_right+gc_left)/2 > TURN_QTR){
      move = false;
      turn = false;
      gc_right = 0;
      gc_left = 0;
    }
  }else if(flip){
    if((gc_right+gc_left)/2 > TURN_HALF){
      move = false;
      flip = false;
      gc_right = 0;
      gc_left = 0;
    }
  }else{
    if((gc_right+gc_left)/2 > ONE_CELL){
      move = false;
      gc_right = 0;
      gc_left = 0;
    }
  }
}

void incCounterL(){
  gc_left++;
}

/**********************************************
 *************** PID CONTROL ******************
 *********************************************/
void pidHandler(){
  //digitalWrite(timing, HIGH);
  getDistance();
  // distance offset
  propValue = distanceFL - distanceFR;
  currentTime = millis();
  derivative = (propValue - oldPropValue)/(currentTime - oldTime);

  //reset back to default speed
  currentSpeed_R = DEFAULT_SPEED;
  currentSpeed_L = DEFAULT_SPEED;
  
  //check to see if there is an openning on the left/right
  // ref: WALL_FOLLOW=2.5, NO_WALL=7
  if( distanceFL > NO_WALL || distanceFR > NO_WALL ){ // when both sides are open
    propValue = 0;
  }else{ // else, leave propValue as is
  }
  
  // Near the left wall
  if(propValue > CENTER_MARGIN){
    newMotorSpeed = KP * propValue + KD * derivative;
    currentSpeed_R += newMotorSpeed;
    currentSpeed_L -= newMotorSpeed;
  }
  // Near the right wall
  else if (propValue < -CENTER_MARGIN){
    newMotorSpeed = KP * (-propValue) + KD * derivative;
    currentSpeed_R -= newMotorSpeed;
    currentSpeed_L += newMotorSpeed;
  }
  // within the center range
  else{
    currentSpeed_R = DEFAULT_SPEED;
    currentSpeed_L = DEFAULT_SPEED;
  }
  
  //SerialUSB.print("currentSpeed_R: ");
  //SerialUSB.println(currentSpeed_R);
  oldPropValue = propValue;
  oldTime = currentTime;
  //digitalWrite(timing, LOW);
  
}

//This function converts the voltage value from the sensors to a 
//distance value in cm.
void getDistance(){
  //Distance for the back left
  //calibrateSensors();
  tmpFL = analogRead(sensorFL);
  tmpFR = analogRead(sensorFR);

  // Get distance for back left
  if ( tmpFL < 2753 ){
    distanceFL =  -0.004 * tmpFL + 15.73;
  }
  else if( tmpFL < 3616 ){
    distanceFL =  -0.002 * tmpFL + 10.83;
  }
  else if( tmpFL < 4833 ){
    distanceFL =  -0.001 * tmpFL + 7.937;
  }
  else{
    distanceFL = 7.937; 
  }

  // Get distance for back right
  if ( tmpFR < 2847 ){
    distanceFR =  -0.004 * tmpFR + 15.73;
  }
  else if( tmpFR < 3899 ){
    distanceFR =  -0.002 * tmpFR + 10.83;
  }
  else if( tmpFR < 4841 ){
    distanceFR =  -0.001 * tmpFR + 7.937;
  }
  else{
    distanceFR = 7.937;
  }
}
