/**********************************************
 *************** CONSTANTS ********************
 *********************************************/
#define ONE_CELL 31 // One cell
#define TURN_QTR 11 // A quarter turn
#define TURN_HALF 24 // A 180 turn
#define MAZE_MAX_X 16 // Max dimention of the maze for X
#define MAZE_MAX_Y 16 // Max dimention of the maze for Y
#define GOAL_X MAZE_MAX_X/2-1 //MAZE_MAX_X/2-1 // Goal x-cord.
#define GOAL_Y MAZE_MAX_X/2-1 //MAZE_MAX_Y/2-1 // Goal y-cord.

// Constants for PID Control
//was KP 1700
//was KD 500
#define KP 2000
#define KD 600
#define DEFAULT_SPEED 13000
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
int tmpFF = 0;
int tmpFL = 0;
int tmpFR = 0;

// Motor values
volatile int gc_right = 0;
volatile int gc_left = 0;

// Movement flags
boolean move = false;
boolean turn = false;
boolean flip = false;
boolean run = true;

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
int potential[MAZE_MAX_Y][MAZE_MAX_X];
int locx = 0;
int locy = MAZE_MAX_Y-1;
int curDirec = 0;
int nxtDirec = 0;
int turnCmd = 0;
int totCount = 0;
int potDif = 0;
int front = 0;
int right = 0; // starboard
int back = 0;
int left = 0; // port
int lowestPotential = 0;
int lowestIndex = 0;

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
  
  // initialize the potential map
  for(int i=0; i<MAZE_MAX_Y; i++){
    for(int j=0; j<MAZE_MAX_X; j++){
      potential[i][j] = MAZE_MAX_X * MAZE_MAX_Y + 1;
    }
  }
  
  // Reset all values, just in case
  resetAll();
  
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
    SerialUSB.print("X: ");
    SerialUSB.println(locx);
    SerialUSB.print("Y: ");
    SerialUSB.println(locy);
    SerialUSB.print("curDirec: ");
    SerialUSB.println(curDirec);
    delay(2000);
	
    setTraversalMap(locx, locy, curDirec, traversalMap);
    updatePotential(locx, locy, GOAL_X, GOAL_Y, potential, traversalMap);
    nxtDirec = findNextDirection(locx, locy);
	
    SerialUSB.print("nxtDirec: ");
    SerialUSB.println(nxtDirec);
	
    turnCmd = chooseTurn(nxtDirec, curDirec);
    SerialUSB.print("turnCmd: ");
    SerialUSB.println(turnCmd);
	
    curDirec = (curDirec + turnCmd) % 4;
	
    SerialUSB.println();
    delay(2000);
	
    move = true;
  }
}// end main

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
  /*
  if(analogRead(frontLongSensor) > 3500){
    move = false;
    switch(curDirec){
      case 0: locy++; break;
      case 1: locx--; break;
      case 2: locy--; break;
      case 3: locx++; break;
    }
    gc_right = 0;
    gc_left = 0;
  }
  */
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
  int i;
  int j;
  for(i=0; i<MAZE_MAX_Y; i++){
    for(j=0; j<MAZE_MAX_X; j++){
      traversalMap[i][j] = 0x00;
    }
  }
}

/*************** initializePotential ********************
*  Initializes the potential array. Called in setup() 
*******************************************************/
 void initializePotential(){
  int i;
  int j;
  for( i=0; i<MAZE_MAX_Y; i++)
    for( j=0; j<MAZE_MAX_X; j++)
      potential[i][j] = MAZE_MAX_X * MAZE_MAX_Y + 1;
}
 
 
void setWall(int curLocX, int curLocY, int setDirec, int traversal[][MAZE_MAX_X]){
  switch(setDirec){
    // Set North wall
    case 0: traversal[curLocY][curLocX] |= 0x11; // There's a wall!! also, we checked the cell
            if( curLocY > 0 ) traversal[curLocY-1][curLocX] |= 0x44; 
            break;
    // Set East wall
    case 1: traversal[curLocY][curLocX] |= 0x22;
            if( curLocX < MAZE_MAX_X - 1) traversal[curLocY][curLocX+1] |= 0x88;
            break;
    // Set South wall
    case 2: traversal[curLocY][curLocX] |= 0x44; 
            if( curLocY < MAZE_MAX_Y -1 ) traversal[curLocY+1][curLocX] |= 0x11; 
            break;
    // Set West wall
    case 3: traversal[curLocY][curLocX] |= 0x88; 
            if( curLocX > 0 ) traversal[curLocY][curLocX-1] |= 0x22;
            break;
  }
}

void setTraveled(int curLocX, int curLocY, int setDirec, int traversal[][MAZE_MAX_X]){
  switch(setDirec){
    // Set North 
    case 0: traversal[curLocY][curLocX] |= 0x10; 
            if( curLocY>0 ){
                traversal[curLocY-1][curLocX] |= 0x40;
            }
            break;
    // Set East 
    case 1: traversal[curLocY][curLocX] |= 0x20;
            if( curLocX<MAZE_MAX_X-1 ){
                traversal[curLocY][curLocX+1] |= 0x80;
            }
            break;
    // Set South 
    case 2: traversal[curLocY][curLocX] |= 0x40;
            if( curLocY<MAZE_MAX_Y-1 ){
                traversal[curLocY+1][curLocX] |= 0x10;
            }
            break;
    // Set West 
    case 3: traversal[curLocY][curLocX] |= 0x80;
            if( curLocX>0 ){
                traversal[curLocY][curLocX-1] |= 0x10;
            }
            break;
  }
}

/*************** readSetTraversalMap ********************
 *  Check the sensor values to see which way there is a 
 *  wall and whether we can travel through that path.
 *  Also, sets the traversal map with that information.
 *******************************************************/
void setTraversalMap(int curLocX, int curLocY, int curDirec, int traversal[][MAZE_MAX_X]){
   int tmpFL;

  // assign directions to sensors based on the current direction
  switch(curDirec){
    // Headed North
    case 0: front=0; right=1; left=3; break;
    // Headed East
    case 1: front=1; right=2; left=0; break;
    // Headed South
    case 2: front=2; right=3; left=1; break;
    // Headed West
    case 3: front=3; right=0; left=2; break;
  }
  
  tmpFF = analogRead(frontLongSensor);
  tmpFF += analogRead(frontLongSensor);
  tmpFF /= 2;
  
  tmpFL = analogRead(sensorFL);
  tmpFL += analogRead(sensorFL);
  tmpFL /= 2;
  
  tmpFR = analogRead(sensorFR);
  tmpFR += analogRead(sensorFR);
  tmpFR /= 2;
  
  // check front sensor
  if( analogRead(frontLongSensor) > 1900 ){
    setWall( curLocX, curLocY, front, traversal);
  }
  // Check right sensor
  if( tmpFR > 2200 ){
    setWall( curLocX, curLocY, right, traversal);
  }
  // Check left sensor
  if( tmpFL > 2200 ){
    setWall( curLocX, curLocY, left, traversal);
  }
}

/***************** updatePotential **********************
 *  Remaps the potential map every time the function is called
 *  by resetting the potential map and then checking the traversalMap
 *  for walls. 
 *******************************************************/
int updatePotential(int currentX, int currentY, int goalX, int goalY, int potential[][MAZE_MAX_X], int traversalMap[][MAZE_MAX_X]){
  int i, j, k;
  int foundFlag;
  
  initializePotential();
  
  // Set goal cell
  potential[goalY][goalX] = 0;
  
  // Make the potential map
  for(i=0; i<MAZE_MAX_Y*MAZE_MAX_X; i++){
    foundFlag = 1;
    // look for where the potential is i
    for(j=0; j<MAZE_MAX_Y; j++){
      for(k=0; k<MAZE_MAX_X; k++){
        if( potential[j][k] == i ){
          foundFlag = 0;
          // look at north
          // cond. true if there is no wall or we haven't travelled there yet
          if((traversalMap[j][k]&0x01)==0 || (traversalMap[j][k]&0x10)==0)
            // if ( potential of current cell + 1 is smaller than potential of cell in front
            if(potential[j][k]+1 < potential[j-1][k])
              // set potential of cell in front to potential of current cell + 1
              potential[j-1][k] = potential[j][k] + 1;
          // look at east
          if((traversalMap[j][k]&0x02)==0 || (traversalMap[j][k]&0x20)==0)
            if(potential[j][k]+1 < potential[j][k+1])
              potential[j][k+1] = potential[j][k] + 1;
          // look at south
          if((traversalMap[j][k]&0x04)==0 || (traversalMap[j][k]&0x40)==0)
            if(potential[j][k]+1 < potential[j+1][k])
              potential[j+1][k] = potential[j][k] + 1;
          // look at west
          if((traversalMap[j][k]&0x08)==0 || (traversalMap[j][k]&0x80)==0)
            if(potential[j][k]+1 < potential[j][k-1])
              potential[j][k-1] = potential[j][k] + 1;
          // look for start position
          if(j == currentX && k == currentY)
            return 1;
        }
      }
    }
    // Could not find start
    if(foundFlag) break;
  }
  return 0;
}

/***************** findNextDirection **********************
 *  Finds the Next Direction based off of the traversal
 *  map and sets the possible values to the neighborhood
 *  array.
 *******************************************************/
int findNextDirection(int currentX, int currentY){
  int i;
  int neighbourhood[4];
    
  for(i=0; i<4; i++){
      neighbourhood[i] = MAZE_MAX_Y * MAZE_MAX_X + 1;
  }
  // check front 
  if( currentY > 0 ){
      // if there is no wall and we haven't travelled there
      if(((traversalMap[currentY][currentX]&0x01)) != 0x01 && (traversalMap[currentY][currentX]&0x10) != 0x10 ){
          neighbourhood[0] = potential[currentY-1][currentX];
      }
  }
  // check right
  if( currentX < MAZE_MAX_X - 1 ){
      if( (traversalMap[currentY][currentX]&0x02) != 0x02 && (traversalMap[currentY][currentX]&0x20) != 0x20 ){
          neighbourhood[1] = potential[currentY][currentX+1];
      }
  }
  // check back
  if( currentY < MAZE_MAX_Y - 1 ){
      if( (traversalMap[currentY][currentX]&0x04) != 0x04 && (traversalMap[currentY][currentX]&0x40) != 0x40 ){
          neighbourhood[2] = potential[currentY+1][currentX];
      }
  }
  // check left
  if( currentX > 0 ){
      if( (traversalMap[currentY][currentX]&0x08) != 0x08 && (traversalMap[currentY][currentX]&0x80) != 0x80 ){
          neighbourhood[3] = potential[currentY][currentX-1];
      }
  }
  
  
  SerialUSB.print("N: ");
  SerialUSB.println(neighbourhood[0]);
  SerialUSB.print("E: ");
  SerialUSB.println(neighbourhood[1]);
  SerialUSB.print("S: ");
  SerialUSB.println(neighbourhood[2]);
  SerialUSB.print("W: ");
  SerialUSB.println(neighbourhood[3]);
  
  lowestPotential = neighbourhood[0];
  lowestIndex = 0;
  for(i=1; i<4; i++){
    if(neighbourhood[i] < lowestPotential){
      lowestPotential = neighbourhood[i];
      lowestIndex = i;
    }
  }

  //if( (traversalMap[currentY][currentX]&0xBB) == 0xBB ){
  //  lowestIndex = 2;
  //}
  
  //delay(1000);
  
  return lowestIndex;
}

/***************chooseTurn**********************
 *  Choose if the bot should turn or not based on the 
 *  current direction and the direction the bot needs
 *  to turn next
 **********************************************/
int chooseTurn(int nxtDirec, int curDirec){
  if(curDirec == nxtDirec){
    setTraveled( locx, locy, curDirec, traversalMap );
    switch(curDirec){
      case 0: locy--; break;
      case 1: locx++; break;
      case 2: locy++; break;
      case 3: locx--; break;
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

/***************buttonHandler**********************
 *  Flips the run flag to skip any movement
 **********************************************/
void buttonHandler(){
  if(run){
    run = false;
  }else{
    run = true;
  }
}

/***************resetAll**********************
 *  Reset all values for a clean excecution
 **********************************************/
void resetAll(){
    move = false;
    gc_right = 0;
    gc_left = 0;
    turnCmd = 0;
    nxtDirec = 0;
    curDirec = 0;
    locx = 0;
    locy = MAZE_MAX_Y-1;
    turn = false;
    flip = false;
}
