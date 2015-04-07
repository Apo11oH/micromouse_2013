/**********************************************
 *************** CONSTANTS ********************
 *********************************************/
#define ONE_CELL 31 // One cell
#define TURN_QTR 11 // A quarter turn
#define TURN_HALF 24 // A 180 turn
#define MAZE_MAX_X 16 // Max dimention of the maze for X
#define MAZE_MAX_Y 16 // Max dimention of the maze for Y

// Constants for PID Control
#define KP 1700
#define KD 500
#define DEFAULT_SPEED 15000
// <- -0.25 - middle - +0.25 ->
#define CENTER_MARGIN .25
#define NO_WALL 7 //distance if no wall on left/right

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

// Motor values
volatile int gc_right = 0;
volatile int gc_left = 0;

// Movement flags
boolean move = false;
boolean turn = false;
boolean flip = false;
boolean other = false;

/*
 * Mapping related
 *  Each cell byte:
 *    High _ _ _ _ | _ _ _ _ Low
 *         W S E N   W S E N
 *    Higher nibble: Saves the if explored or not
 *    Lower nibble: Saves the wall location
 *  North: 0, East: 1, South: 2, West: 3
 */
int traversalMap[MAZE_MAX_Y][MAZE_MAX_X]; 
int curLocX = 0;
int curLocY = MAZE_MAX_Y-1;
int curDirec = 0;
int nxtDirec = 0;
int turnCmd = 0;
int front = 0;
int right = 0; // starboard
int back = 0;
int left = 0; // port
int lowestPotential = 0;
int lowestIndex = 0;
int minNum = 0;
int minIndex = 0;

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

  // Initialize the traversal map
  initializeTraversal();
  
  //STBY==HIGH -> enables motors
  digitalWrite(motorSTBY, HIGH);
  for(int i=0; i<3; i++){
    toggleLED();
    delay(1000);
  }
}

/**********************************************
 *************** MAIN LOOP ********************
 *********************************************/
void loop(){ 
  toggleLED();
  pidHandler();

  if(move){
    switch(turnCmd){
      case 0: moveForward(); break;
      case 1: turnRight(); break;
      case 2: turnRight(); break;
      case 3: turnLeft(); break;
    }
  }else{
    moveStanby();
    SerialUSB.print("curDirec: ");
    SerialUSB.println(curDirec);
    nxtDirec = findNextDirection();
    SerialUSB.print("nxtDirec: ");
    SerialUSB.println(nxtDirec);
    turnCmd = chooseTurn();
    SerialUSB.print("turnCmd: ");
    SerialUSB.println(turnCmd);
    curDirec = (curDirec + turnCmd) % 4;
    SerialUSB.print("NewCurDirec: ");
    SerialUSB.println(curDirec);
    SerialUSB.println();
    delay(500);
    move = true;
  }
}

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
  // Stop if bot is too close to wall in front
  if(analogRead(frontLongSensor) > 3500){
    move = false;
    switch(curDirec){
      case 0: curLocY++; break;
      case 1: curLocX--; break;
      case 2: curLocY--; break;
      case 3: curLocX++; break;
    }
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
  
  //check to see if there is an opening on the left/right
  // ref: WALL_FOLLOW=2.5, NO_WALL=7
  if( distanceFL > NO_WALL || distanceFR > NO_WALL ){ // when both sides are open
    propValue = 0;
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
  
  oldPropValue = propValue;
  oldTime = currentTime;
  
}

//This function converts the voltage value from the sensors to a 
//distance value in cm.
void getDistance(){
  //Distance for the back left
  //calibrateSensors();
  tmpFL = analogRead(sensorFL);
  tmpFL += analogRead(sensorFL);
  tmpFL /= 2;
  
  tmpFR = analogRead(sensorFR);
  tmpFR += analogRead(sensorFR);
  tmpFR /= 2;

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

/**********************************************
 ********* MAPPING(CUSTOM. FLOOD FILL) *********
 *********************************************/
 
/*************** initializeTraversal ********************
*  Initializes the traversal array. Called in setup() 
*******************************************************/
void initializeTraversal(){
  for(int i=0; i<MAZE_MAX_Y; i++)
    for(int j=0; j<MAZE_MAX_X; j++)
      traversalMap[i][j] &= 0x00;
}

/***************** findNextDirection **********************
 *  
 *  
 *  
 *******************************************************/
int findNextDirection(){
  int around[4];
  
  for( int i=0; i<4; i++){
    around[i] = 10;
  }
  
  if( analogRead(frontLongSensor) < 1900 ){
    SerialUSB.println("Front Open");
    around[0] = 0;
  }
  
  if( analogRead(sensorFR) < 2200 ){
    SerialUSB.println("Right Open");
    around[1] = 1;
    //other = true;
  }
  
  if( turnCmd == 0 ){
    SerialUSB.println("Back Open");
    around[2] = 2;
  }
  
  if( analogRead(sensorFL) < 2200 ){
    SerialUSB.println("Left Open");
    around[3] = 3;
    //other = true;
  }
  
  if(other && (traversalMap[curLocY][curLocX] != 0) ){
    minNum = around[ traversalMap[curLocY][curLocX] ];
    minIndex = traversalMap[curLocY][curLocX];
    for(int i=traversalMap[curLocY][curLocX]; i<4; i++){
      if( around[i] < minNum ){
        minNum = around[ i ];
        minIndex = i;
      }
    }
  }else{
    minNum = around[0];
    minIndex = 0;
    for(int i=1; i<4; i++){
      if( around[i] < minNum ){
        minNum = around[i];
        minIndex = i;
      }
    }
  }
  
  traversalMap[curLocY][curLocX]++;
  return minIndex;
}


/***************chooseTurn**********************
 *  Choose if the bot should turn or not based on the 
 *  current direction and the direction the bot needs
 *  to turn next
 **********************************************/
int chooseTurn(){
  if(curDirec == nxtDirec){
    switch(curDirec){
      case 0: curLocY--; break;
      case 1: curLocX++; break;
      case 2: curLocY++; break;
      case 3: curLocX--; break;
    }
    // Keep current direction
    return 0;
  }else if((curDirec+1)%4 == nxtDirec){
    // Turn 90 right 
    turn = true;
    return 1;
  }else if((curDirec+2)%4 == nxtDirec){
    // Flip 180 
    flip = true;
    return 2;
  }else{
    // Turn 90 left 
    turn = true;
    return 3;
  }
}