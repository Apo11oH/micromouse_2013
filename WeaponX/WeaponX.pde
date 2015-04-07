/**********************************************
 *************** CONSTANTS ********************
 *********************************************/
#define ONE_CELL 31 // One cell
#define TURN_QTR 12 // A quarter turn
#define TURN_HALF 24 // A 180 turn
#define MAZE_MAX_X 16 // Max dimention of the maze for X
#define MAZE_MAX_Y 16 // Max dimention of the maze for Y
#define GOAL_X MAZE_MAX_X/2-1 //MAZE_MAX_X/2-1 // Goal x-cord.
#define GOAL_Y MAZE_MAX_X/2-1 //MAZE_MAX_Y/2-1 // Goal y-cord.

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
volatile int motorSTBY = 32;
volatile int motorAIN1 = 33; // Left
volatile int motorAIN2 = 37;
volatile int motorPWMA = 28;
volatile int motorBIN1 = 25; // Right
volatile int motorBIN2 = 29;
volatile int motorPWMB = 24;

// SensorPins
volatile int sensorFR = 15; // Corrected (5/11 22:30)
volatile int sensorFL = 18; // Corrected (5/11 22:30)
volatile int frontLongSensor = 20;

// EncoderPins
volatile int encoderLeft = 7;
volatile int encoderRight = 5;

// PID values
volatile double currentSpeed_R = DEFAULT_SPEED;
volatile double currentSpeed_L = DEFAULT_SPEED;
volatile double propValue = 0;
volatile double oldPropValue = 0;
volatile double newMotorSpeed = 0;
volatile int oldTime = 0;
volatile int currentTime = 0;
volatile int derivative = 0;
volatile int distanceFL = 0;
volatile int distanceFR = 0;
volatile int tmpFL = 0;
volatile int tmpFR = 0;
volatile int tmpFF = 0;

// Motor values
volatile int gc_right = 0;
volatile int gc_left = 0;

// Movement flags
volatile boolean move = false;
volatile boolean turn = false;
volatile boolean flip = false;
volatile boolean run = true;

/*
 * Mapping related
 *  Each cell byte:
 *    High _ _ _ _ | _ _ _ _ Low
 *         W S E N   W S E N
 *    Higher nibble: Saves the if explored or not
 *    Lower nibble: Saves the wall location
 *  North: 0, East: 1, South: 2, West: 3
 */
volatile int traversalMap[MAZE_MAX_Y][MAZE_MAX_X]; 
volatile int potential[MAZE_MAX_Y][MAZE_MAX_X];
volatile int curLocX = 0;
volatile int curLocY = MAZE_MAX_Y-1;
volatile int curDirec = 0;
volatile int nxtDirec = 0;
volatile int turnCmd = 0;
volatile int front = 0;
volatile int right = 0;
volatile int back = 0;
volatile int left = 0;
volatile int lowestPotent = 0;
volatile int lowestIndex = 0;
volatile int neighbourhood[4];

// Counter vals
volatile int i = 0;
volatile int j = 0;
volatile int k = 0;

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

  // Initialize the maps
  initializeTraversal();
  initializePotential();
  
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
    setTraversalMap();
    updatePotential();
    nxtDirec = findNextDirection();
    turnCmd = chooseTurn();
    switch(turnCmd){
      case 0: break; // Don't turn
      case 1: curDirec+=1; curDirec%=4; break; // Turn 90 degrees right 
      case 2: curDirec+=2; curDirec%=4; break; // 180 degree turn 
      case 3: curDirec+=3; curDirec%=4; break; // Turn 90 degrees left
    }
    delay(1000);
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
  // Stop if droid is too close to wall in front
  if(analogRead(frontLongSensor) > 3900){
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

/**********************************************
 ********* MAPPING(CUSTOM. FLOOD FILL) *********
 *********************************************/
 
/*************** initializeTraversal ********************
*  Initializes the traversal array. Called in setup() 
*******************************************************/
void initializeTraversal(){
  for( i=0; i<MAZE_MAX_Y; i++)
    for( j=0; j<MAZE_MAX_X; j++)
      traversalMap[i][j] = 0x00;
}
 
/*************** initializePotential ********************
*  Initializes the potential array. Called in setup() 
*******************************************************/
 void initializePotential(){
  for( i=0; i<MAZE_MAX_Y; i++)
    for( j=0; j<MAZE_MAX_X; j++)
      potential[i][j] = MAZE_MAX_X * MAZE_MAX_Y + 1;
}
 
void setWall(int setDirec){
  switch(setDirec){
    // Set North wall
    case 0: traversalMap[curLocY][curLocX] |= 0x11; // There's a wall!! also, we checked the cell
            if( curLocY > 0 ) traversalMap[curLocY-1][curLocX] |= 0x44; 
            break;
    // Set East wall
    case 1: traversalMap[curLocY][curLocX] |= 0x22;
            if( curLocX < MAZE_MAX_X - 1) traversalMap[curLocY][curLocX+1] |= 0x88;
            break;
    // Set South wall
    case 2: traversalMap[curLocY][curLocX] |= 0x44; 
            if( curLocY < MAZE_MAX_Y -1 ) traversalMap[curLocY+1][curLocX] |= 0x11; 
            break;
    // Set West wall
    case 3: traversalMap[curLocY][curLocX] |= 0x88; 
            if( curLocX > 0 ) traversalMap[curLocY][curLocX-1] |= 0x22;
            break;
  }
}

void setSeen(int setDirec){
  switch(setDirec){
    // Set North wall
    case 0: traversalMap[curLocY][curLocX] |= 0x10; 
            traversalMap[curLocY][curLocX] &= 0xfe; 
            if( curLocY>0 ){
                traversalMap[curLocY+1][curLocX] |= 0x40;
                traversalMap[curLocY+1][curLocX] &= 0xfb;
            }
            break;
    // Set East wall
    case 1: traversalMap[curLocY][curLocX] |= 0x20;
            traversalMap[curLocY][curLocX] &= 0xfd; 
            if( curLocX<MAZE_MAX_X-1 ){
                traversalMap[curLocY][curLocX+1] |= 0x80;
                traversalMap[curLocY][curLocX+1] &= 0xf7;
            }
            break;
    // Set South wall
    case 2: traversalMap[curLocY][curLocX] |= 0x40;
            traversalMap[curLocY][curLocX] &= 0xfb; 
            if( curLocY<MAZE_MAX_Y-1 ){
                traversalMap[curLocY+1][curLocX] |= 0x10;
                traversalMap[curLocY+1][curLocX] &= 0xfe;
            }
            break;
    // Set West wall
    case 3: traversalMap[curLocY][curLocX] |= 0x80;
            traversalMap[curLocY][curLocX] &= 0xf7; 
            if( curLocX>0 ){
                traversalMap[curLocY][curLocX-1] |= 0x10;
                traversalMap[curLocY][curLocX-1] &= 0xfe;
            }
            break;
  }
}

/*************** readSetTraversalMap ********************
 *  Check the sensor values to see which way there is a 
 *  wall and whether we can travel through that path.
 *  Also, sets the traversal map with that information.
 *******************************************************/
void setTraversalMap(){
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
  
  tmpFF =0;
  tmpFL = 0;
  tmpFR = 0;
  for( i=0; i<3; i++ ){
    tmpFF += analogRead(frontLongSensor);
    tmpFL += analogRead(sensorFR);
    tmpFR += analogRead(sensorFL) ;
  }
  tmpFF /= 3;
  tmpFL /= 3;
  tmpFR /= 3;
  
  // check front sensor
  if( tmpFF > 1900 ){
    setWall( front );
  }
  // Check right sensor
  if( tmpFR > 2200 ){
    setWall( right );
  }
  // Check left sensor
  if( tmpFR > 2200 ){
    setWall( left );
  }
}

/***************** updatePotential **********************
 *  Remaps the potential map every time the function is called
 *  by resetting the potential map and then checking the traversalMap
 *  for walls. 
 *******************************************************/
int updatePotential(){
  int foundFlag;
  
  // Set goal cell
  potential[GOAL_Y][GOAL_X] = 0;
  
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
          if(j == curLocX && k == curLocY)
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
 *  
 *  
 *  
 *******************************************************/
int findNextDirection(){
  switch(curDirec){
    // Headed North
    case 0: front=0; right=1; back=2; left=3; break;
    // Headed East
    case 1: front=1; right=2; back=3; left=0; break;
    // Headed South
    case 2: front=2; right=3; back=0; left=1; break;
    // Headed West
    case 3: front=3; right=0; back=1; left=2; break;
  }
    
  for(i=0; i<4; i++)
      neighborhood[i] = MAZE_MAX_Y * MAZE_MAX_X + 1;
      
  // check front 
  if( curLocY > 0 )
      if( (traversalMap[curLocY][curLocX]&0x11) != 0x11 )
          neighborhood[front] = potential[curLocY-1][curLocX];
          
  // check right
  if( curLocX < MAZE_MAX_X - 1 )
      if( (traversalMap[curLocY][curLocX]&0x22) != 0x22 )
          neighborhood[right] = potential[curLocY][curLocX+1];
  // check back
  if( curLocY < MAZE_MAX_Y - 1 )
      if( (traversalMap[curLocY][curLocX]&0x44) != 0x44 )
          neighborhood[back] = potential[curLocY+1][curLocX];
          
  // check left
  if( curLocX > 0 )
      if( (traversalMap[curLocY][curLocX]&0x88) != 0x88 )
          neighborhood[left] = potential[curLocY][curLocX-1];
  
  lowestPotent = neighborhood[0];
  lowestIndex = 0;
  for(i=1; i<4; i++){
    if(neighborhood[i] < lowestPotent){
      lowestPotent = neighborhood[i];
      lowestIndex = i;
    }
  }
  
  if( (traversalMap[curLocY][curLocX]&0xBB) == 0xBB ){
    lowestIndex = 2;
  }
  
  return lowestIndex;
}// end findNextDirection

/***************chooseTurn**********************
 *  Choose if the bot should turn or not based on the 
 *  current direction and the direction the bot needs
 *  to turn next
 **********************************************/
int chooseTurn(){
  if(curDirec == nxtDirec){
    setSeen( curDirec );
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