/**********************************************
 *************** CONSTANTS ********************
 *********************************************/
#define ONE_CELL 31 // One cell
#define TURN_QTR 12 // A quarter turn
#define TURN_HALF 24 // A 180 turn
#define MAZE_MAX_X 3 // Max dimention of the maze for X
#define MAZE_MAX_Y 3 // Max dimention of the maze for Y
#define GOAL_X 0//MAZE_MAX_X/2-1 // Goal x-cord.
#define GOAL_Y 2//MAZE_MAX_Y/2-1 // Goal y-cord.

// Constants for PID Control
//past KP 260, 240,230, 2000, 1650
//past KD 130, 50, 500
// Low speed:(2000, 500)
// Was working on: (1700, 100)
#define KP 1700
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
int sensorFR = 15; // Corrected (5/11 22:30)
int sensorFL = 18; // Corrected (5/11 22:30)
int frontLongSensor = 20;

// EncoderPins
int encoderLeft = 7;
int encoderRight = 5;

// Timing Pins (DEBUGGING)
int timing = 8;

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
volatile int tickAdjust = 0;

/*
 * Mapping related
 *  Each cell byte:
 *    High _ _ _ _ | _ _ _ _ Low
 *         W S E N   W S E N
 *    Higher nibble: Saves the if explored or not
 *    Lower nibble: Saves the wall location
 *  North: 0, East: 1, South: 2, West: 3
 */
int potential[MAZE_MAX_Y][MAZE_MAX_X];
int traversalMap[MAZE_MAX_Y][MAZE_MAX_X]; 
int locx = 0;
int locy = MAZE_MAX_Y-1;
int curDirec = 0;
int nxtDirec = 0;
int turnCommand = 0;

// Movement flags
boolean move = false;
boolean turn = false;
boolean flip = false;
boolean prevTurn = false;
boolean prevFlip = false;

// DEBUG
int debugCount = 0;
int fiveAverage = 0;

/**********************************************
 ****************** SETUP *********************
 *********************************************/
void setup(){
  // Setup LED for...well..you know, flashy stuff X)
  pinMode(BOARD_LED_PIN, OUTPUT);
  // Setup motor pins
  pinMode(timing, OUTPUT);
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

  // Initialize the traversal array
  initializeTraversal();
  
  //STBY==HIGH -> enables motors
  digitalWrite(motorSTBY, HIGH);
  for(int i=0; i<3; i++){
    toggleLED();
    delay(1000);
  }
  
  // Initialize the potential map
  for(i=0; i<MAZE_MAX_Y; i++){
    for(j=0; j<MAZE_MAX_X; j++){
      potential[i][j] = MAZE_MAX_Y * MAZE_MAX_X +1;
    }
  }
 
}

/**********************************************
 *************** MAIN LOOP ********************
 *********************************************/
void loop(){ 
  //SerialUSB.print("setup move");
  //SerialUSB.println(move);
  
  //pin 8 test time
  //digitalWrite(timing, HIGH);
  toggleLED();
  pidHandler();

  
  //SerialUSB.print(move);
  //SerialUSB.println("first move");  
  if(move){
    switch(turnCommand){
      case 0: moveForward(); break;
      case 1: turnRight(); break;
      case 2: turnRight(); break; // may use turnLeft
      case 3: turnLeft(); break;
    }
  }else if(prevTurn){
    moveForward();
  }else{
    /*
    if(prevTurn || prevFlip){
      switch(curDirec){
        case 0: locy++; break;
        case 1: locx--; break;
        case 2: locy--; break;
        case 3: locx++; break;
      }
    }
    */
    moveStanby();
    SerialUSB.print("locx: ");
    SerialUSB.println(locx);
    SerialUSB.print("locy: ");
    SerialUSB.println(locy);
    readSetTraversalMap(locx, locy, curDirec, traversalMap);
    updatePotential(locx, locy, GOAL_X, GOAL_Y, potential, traversalMap);
    nxtDirec = decideNextDirec(locx, locy, potential, traversalMap);
    //nxtDirec = decideNextDirecDebug(locx, locy, traversalMap);
    SerialUSB.print("curDirec: ");
    SerialUSB.println(curDirec);
    SerialUSB.print("nxtDirec: ");
    SerialUSB.println(nxtDirec);
    turnCommand = chooseTurn(nxtDirec, curDirec);
    SerialUSB.print("turnCommand: ");
    SerialUSB.println(turnCommand);
    curDirec = nxtDirec;
    /*
    switch(turnCommand){
      case 0: break; // Don't turn
      case 1: curDirec+=3; curDirec%=4; break; // Turn 90 degrees right 
      case 2: curDirec+=1; curDirec%=4; break; // 180 degree turn 
      case 3: curDirec+=2; curDirec%=4; break; // Turn 90 degrees left
    }
    */
    switch(curDirec){
      case 0: locy--; break;
      case 1: locx++; break;
      case 2: locy++; break;
      case 3: locx--; break;
    }
    delay(1000);
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
  if(analogRead(frontLongSensor) > 3900){
    move = false;
    prevTurn = false;
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
int chooseTurn(int nxtDirec, int curDirec){
  if(curDirec == nxtDirec){
    // Keep current direction
    return 0;
  }else if((curDirec+1)%4 == nxtDirec){
    // Turn 90 right 
    turn = true;
    prevTurn = true;
    return 1;
  }else if((curDirec+2)%4 == nxtDirec){
    // Flip 180 
    flip = true;
    prevFlip = true;
    return 2;
  }else{
    // Turn 90 left 
    turn = true;
    prevTurn = true;
    return 3;
  }
}

/**********************************************
 ********* MAPPING(CUSTOM. FLOOD FILL) *********
 *********************************************/
 
/*********************** set ****************************
  Sets values from the traversal map 
     front
      0(1)
   3(8) 1(2)
      2(4)
      back
 *******************************************************/
void set(int locx, int locy, int direction, int traversal[][MAZE_MAX_X]){
  switch(direction){
    case 0: traversal[locy][locx] |= 0x11; // 0001 0001
            // if not out of range: set south wall of cell in front too
            //SerialUSB.println("Wall set north");
            if( locy > 0 ){
              traversal[locy-1][locx] |= 0x44;  
            }
            break;
    case 1: traversal[locy][locx] |= 0x22; // 0010 0010
            if( locx < MAZE_MAX_X-1 ){
              traversal[locy][locx+1] |= 0x88;
            }
            //SerialUSB.println("Wall set east");
            break;
    case 2: traversal[locy][locx] |= 0x44; // 0100 0100
            if( locy < MAZE_MAX_Y-1 ){
              traversal[locy+1][locx] |= 0x11;
            }
            //SerialUSB.println("Wall set south");
            break;
    case 3: traversal[locy][locx] |= 0x88; // 1000 1000
            if( locx > 0 ){
              traversal[locy][locx-1] |= 0x22;
            }
            //SerialUSB.println("Wall set west");
            break;
  }
}

/******************** unset *****************************
 *  Markes cell as traveled in the traversal map 
 *  ??Also changes the cell information in front
 *  ??as long as it's not at the end of the edge
 *******************************************************/
void unset(int locx, int locy, int direction, int traversal[][MAZE_MAX_X]){
  switch(direction){
    case 0: traversal[locy][locx] |= 0x10; // 0001 0000
            traversal[locy][locx] &= 0xfe; // 1111 1110
            if(locy>0){
              traversal[locy-1][locx] |= 0x40;
              traversal[locy-1][locx] &= 0xfb;
            }
            //SerialUSB.println("Wall not set north");
            break;
    case 1: traversal[locy][locx] |= 0x20; // 0010 0000
            traversal[locy][locx] &= 0xfd;  // 1111 1101
            // Next 4 line gives some sort of error in the IDE (IDE bug???)
            if( locx > MAZE_MAX_X-1 ){
              traversal[locy][locx+1] |= 0x80;
              traversal[locy][locx+1] &= 0xf7;
            }
            //SerialUSB.println("Wall not set east");
            break;
    case 2: traversal[locy][locx] |= 0x40; // 0100 0000
            traversal[locy][locx] &= 0xfb; // 1111 1011
            if(locy>MAZE_MAX_Y-1){
              traversal[locy+1][locx] |= 0x10;
              traversal[locy+1][locx] &= 0xfe;
            }
            //SerialUSB.println("Wall not set south");
            break;
    case 3: traversal[locy][locx] |= 0x80; //1000 0000
            traversal[locy][locx] &= 0xf7; // 1111 0111
            if(locx>0){
              traversal[locy][locx-1] |= 0x20;
              traversal[locy][locx-1] &= 0xfd;
            }
            //SerialUSB.println("Wall not set west");
            break;
  }
}
 
/*************** initializeTraversal ********************
 *  Initializes the traversal array. Called in setup() 
 *******************************************************/
void initializeTraversal(){
  for(int i=0; i<MAZE_MAX_Y; i++){
    for(int j=0; j<MAZE_MAX_X; j++){
      traversalMap[i][j] = 0x00;
    }
  }
}

/*************** readSetTraversalMap ********************
 *  Check the sensor values to see which way there is a 
 *  wall and whether we can travel through that path.
 *  Also, sets the traversal map with that information.
 *******************************************************/
void readSetTraversalMap(int locx, int locy, int direction, int traversalMap[][MAZE_MAX_X]){
  int front = 0;
  int right = 0;
  int left = 0;
  
  switch(direction){
    case 0: front=0; right=1; left=3; break; // looking north
    case 1: front=1; right=2; left=0; break; // looking east
    case 2: front=2; right=3; left=1; break; // looking south
    case 3: front=3; right=0; left=2; break; // looking west
  }
  
  // Check front sensor
  if( analogRead(frontLongSensor) > 900 ){
    set(locx,locy,front,traversalMap);
  }else {
    unset(locx,locy,front,traversalMap);
  }
  // Check right sensor
  if( analogRead(sensorFR) > 2000 ){
    set(locx,locy,right,traversalMap);
  }else {
    unset(locx,locy,right,traversalMap);
  }
  // Check left sensor
  if( analogRead(sensorFL) > 2000 ){
    set(locx,locy,left,traversalMap);
  }else {
    unset(locx,locy,left,traversalMap);
  }
}

/***************** updatePotential **********************
 *  Remaps the potential map every time the function is called
 *  by resetting the potential map and then checking the traversalMap
 *  for walls. 
 *******************************************************/
int updatePotential(int initX, int initY, int goalX, int goalY, int potential[][MAZE_MAX_X], int traversalMap[][MAZE_MAX_X]){
  int i, j, k;
  int foundFlag;
  
  
  //SerialUSB.println("Potential Initialized");
  
  // Set goal cell
  potential[goalY][goalX] = 0;
  //SerialUSB.println("Goal set");
  
  // Make the potential map
  for(i=0; i<MAZE_MAX_Y*MAZE_MAX_X; i++){
    foundFlag = 1;
    for(j=0; j<MAZE_MAX_Y; j++){
      for(k=0; k<MAZE_MAX_X; k++){
        if( potential[j][k] == i){
          foundFlag = 0;
          // look at north
          if((traversalMap[j][k]&0x01)==0 || (traversalMap[j][k]&0x10)==0)
            if(potential[j][k]+1 < potential[j-1][k])
              potential[j-1][k] = potential[j][k] + 1;
          // look at east
          if((traversalMap[j][k]&0x02)==0 || (traversalMap[j][k]&0x20)==0)
            if(potential[j][k]+1 < potential[j][k+1])
              potential[j+1][k] = potential[j][k] + 1;
          // look at south
          if((traversalMap[j][k]&0x04)==0 || (traversalMap[j][k]&0x40)==0)
            if(potential[j][k]+1 < potential[j+1][k])
              potential[j+1][k] = potential[j][k] + 1;
          // look at west
          if((traversalMap[j][k]&0x08)==0 || (traversalMap[j][k]&0x80)==0)
            if(potential[j][k]+1 < potential[j][k-1])
              potential[j-1][k] = potential[j][k] + 1;
          // look for start position
          if(j == initX && k == initY)
            return 1;
        }
      }
    }
    // Could not find start
    if(foundFlag) break;
  }
  return 0;
}

/***************** decideNextDirec **********************
 *  Check the neighboring cells for the potential value and
 *  determin which cell to go to next based on the smallest
 *  potential value. Returns that index/direction value
 *******************************************************/
int decideNextDirec(int currentX, int currentY, int potential[][MAZE_MAX_X], int traversalMap[][MAZE_MAX_X]){
  int minnum = 0;
  int minIndex = 0;
  int neighborhood[4];
  switch(curDirec){
    // Headed North
    case 1: front=1; right=2; back=4; left=8; break;
    // Headed East
    case 2: front=2; right=4; back=8; left=1; break;
    // Headed South
    case 4: front=4; right=8; back=1; left=2; break;
    // Headed West
    case 8: front=8; right=1; back=2; left=4; break;
  }
    
  for(i=0; i<4; i++){
      neighborhood[4] = MAZE_MAX_Y * MAZE_MAX_X + 1;
  }
  // check front 
  if( currentY > 0 ){
      if( (traversal[currentY][currentX]&0x01) != 0x01 ){
          neighborhood[front] = potential[currentY-1][currentX];
      }
  }
  // check right
  if( currentX < MAZE_MAX_X - 1 ){
      if( (traversal[currentY][currentX]&0x02) != 0x02 ){
          neighborhood[right] = potential[currentY][currentX+1];
      }
  }
  // check back
  if( currentY < MAZE_MAX_Y - 1 ){
      if( (traversal[currentY][currentX]&0x04) != 0x04 ){
          neighborhood[back] = potential[currentY+1][currentX];
      }
  }
  // check left
  if( currentX > 0 ){
      if( (traversal[currentY][currentX]&0x08) != 0x08 ){
          neighborhood[left] = potential[currentY][currentX-1];
      }
  }
  
  minnum = neighborhood[0];
  minIndex = 0;
  for(i=1; i<4; i++){
    if(neighborhood[i] < minnum)
      minnum = neighborhood[i];
      minIndex = i;
  }
  
  return minIndex;
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
      prevFlip = false;
      gc_right = 0;
      gc_left = 0;
    }
  }else{
    if((gc_right+gc_left)/2 > ONE_CELL-tickAdjust){
      move = false;
      prevTurn = false;
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
