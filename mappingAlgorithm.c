#define MAZE_MAX_X 16
#define MAZE_MAX_Y 16

/*
 *  Each cell byte:
 *    High _ _ _ _ | _ _ _ _ Low
 *         W S E N   W S E N
 *    Higher nibble: Saves the 
 *    Lower nibble: Saves the wall location
 */

// Map with the potential values
int potential[MAZE_MAX_Y][MAZE_MAX_X];
// traversal map
int traversalMap[MAZE_MAX_Y][MAZE_MAX_X]; 
int locx = 0;
int locy = MAXE_MAX_Y-1;
/*
 *  North: 0, East: 1, South: 2, West: 3
 */
int curDirec = 0;
int nxtDirec = 0;
int turnCommand = 0;

/***************initializeTraversal**********************
 *  Initializes the traversal array. Called in setup() 
 */
void initializeTraversal(){
  for(int i=0; i<MAZE_MAX_Y; i++){
    for(int j=0; j<MAZE_MAX_X; j++){
      traversalMap[i][j] = 0x00;
    }
  }
}

/***************set**********************
 *  Sets values from the traversal map 
     front
      0(1)
   3(8) 1(2)
      2(4)
      back
 */
void set(int locx, int locy, int direction, int traversalMap[MAZE_MAX_X][MAZE_MAX_Y]){
  switch(direction){
    case 0: traversalMap[locy][locx] |= 0x11;
            // if not out of range: set south wall of cell in front too
            if(locy>0) traversalMap[locy-1][locx] |= 0x44; break;
    case 1: traversalMap[locy][locx]|=0x22;
           if(locy<MAZE_MAX_X-1)traversalMap[locy][locx+1] |= 0x88; break;
		case 2: traversalMap[locy][locx]|=0x44;
           if(locy<MAZE_MAX_Y-1)traversalMap[locy+1][locx] |= 0x11; break;
		case 3: traversalMap[locy][locx]|=0x88;
           if(locx>0)traversalMap[locy][locx-1] |= 0x22;break;
  }
}

/***************unset**********************
 *  Unsets values from the traversal map 
 *  Also changes the cell information in front
 *  as long as it's not at the end of the edge
 */
void unset(int locx, int locy, int direction, int traversalMap[MAZE_MAX_X][MAZE_MAX_Y]){
  switch(direction){
    case 0: traversalMap[locy][locx] |= 0x10; // bit location 4 get's set
            traversalMap[locy][locx] &= 0xfe; // bit location 0 get's cleared
            if(locy>0){
              traversalMap[locy-1][locx] |= 0x40;
              traversalMap[locy-1][locx] &= 0xfb;
            }break;
    case 1: traversalMap[locy][locx] |= 0x20;
            traversalMap[locy][locx] &= 0xfd;
            if(locx>MAZE_MAX_X-1){
              traversalMap[locy][locx+1] |= 0x80;
              traversalMap[locy][locx+1] &= 0xfb;
            }break;
		case 2: traversalMap[locy][locx] |= 0x40;
            traversalMap[locy][locx] &= 0xfb;
            if(locy>MAZE_MAX_Y-1){
              traversalMap[locy+1][locx] |= 0x10;
              traversalMap[locy+1][locx] &= 0xfb;
            }break;
		case 3: traversalMap[locy][locx]|=0x80;
            traversalMap[locy][locx] &= 0xf7;
            if(locx>0){
              traversalMap[locy][locx-1] |= 0x20;
              traversalMap[locy][locx-1] &= 0xfb;
            }break;
  }
}

/***************readSetTraversalMap**********************
 *  Check the sensor values to see which way there is a 
 *  wall and whether we can travel through that path.
 *  Also, sets the traversal map with that information.
 */
void readSetTraversalMap(int locx, int locy, int direcion, int traversalMap[MAZE_MAX_X][MAZE_MAX_Y]){
  int front = 0;
  int right = 0;
  int left = 0;
  
  /*
   * front
   *   0
   * 3   1
   *   2
   *  back
  */
  switch(direction){
    case 0: front=0; right=1; left=3; break; 
    case 1: front=1; right=2; left=0; break; 
    case 2: front=2; right=3; left=1; break; 
    case 3: front=3; right=0; left=2; break; 
  }
  
  // Check front sensor
  if( analogRead(frontLongSensor) < 2500){
    set(locx,locy,front,traversalMap);
  }else {
    unset(locx,locy,front,traversalMap);
  }
  // Check right sensor
	if( analogRead(sensorFR) < 2847 ){
    unset(locx,locy,right,traversalMap);
  }else {
    set(locx,locy,right,traversalMap);
  }
  // Check left sensor
	if( analogRead(sensorBR) < 2753 ){
    unset(locx,locy,left,traversalMap);
  }else {
    set(locx,locy,left,traversalMap);
  }
}

/***************decideNextDirec**********************
 *  Check the neighboring cells for the potential value and
 *  determin which cell to go to next based on the smallest
 *  potential value.
 */
void decideNextDirec(int locx, int locy, int traversalMap[MAZE_MAX_X][MAZE_MAX_Y], int potential[MAZE_MAX_Y][MAZE_MAX_X]){
  int neighborhood[4]; // Cells around the current cell
  int minnum = 0;
  int minIndex = 0;
  
  // Fill the neighborhood array with damn big numbers
  for(int i=0; i<4; i++)
    neighborhood[i] = MAZE_MAX_X * MAZE_MAX_Y + 1;
    
  // Check neighboring cells for walls
  // Check north wall
  if( locy > 0 )
    if( ( traversalMap[locy][locx]&0x01 ) != 0x01 )
      neighborhood[0] = potential[locy-1][locx];
      
  // Check east wall
  if( locx < MAZE_MAX_X - 1 )
    if( ( traversalMap[locy][locx]&0x02) != 0x02 )
      neighborhood[1] = potential[locy][locx+1];
      
  // Check south wall
  if( locy < MAZE_MAX_Y-1 )
    if( ( traversalMap[locy][locx]&0x04 ) != 0x04 )
      neighborhood[2] = potetntial[locy+1][locx];
      
  // Check west wall
  if( locx > 0 )
    if( ( traversalMap[locy][locx]&0x08 ) != 0x08 )
      neighborhood[3] = potntial[locy][locx-1];
      
  minnum = neighboorhood[0];
  minindex = 0;
  // Check the neighboorhood cell with the minimum number
  for(int i=1; i<4; i++){
    if(neighboorhood[i]<minnum){
      minnum = neighboorhood[i];
      minindex = i;
    }
  }
  // Returns the index containing the smallest number
  // 0: north, 1: east, 2: south, 3: west
  return minindex;
}

/***************updatePotential**********************
 *  Remaps the potential map every time the function is called
 *  by resetting the potential map and then checking the traversalMap
 *  for walls. 
 */
void updatePotential(int initX, int initY, int goalX, int goalY, int potential[MAZE_MAX_Y][MAZE_MAX_X], int traversalMap[MAZE_MAX_X][MAZE_MAX_Y]){
  int i, j, k;
  int foundFlag;
  
  // Initialize the potential map
  for(i=0; i<MAZE_MAX_Y; i++)
    for(j=0; i<MAZE_MAX_Y; j++)
      potential[i][j] = MAZE_MAX_Y * MAZE_MAX_X +1;
      
  // Set goal cell
  potential[goalY][goalX] = 0;
  // Make the potential map
  for(i=0; i<MAZE_MAX_Y*MAZE_MAX_X; i++){
    foundFlag = 1;
    for(j=0; i<MAZE_MAX_Y; i++){
      for(k=0; k<MAZE_MAX_X; k++){
        foundFlag = 0;
        // look at north
        if( j>0 )
          if((traversalMap[j][k]&0x01)==0 || (traversalMap[j][k]&0x10)==0)
            if(potential[j][t]+1 < potential[j-1][t])
              potential[j-1][t] = potential[j][t] + 1;
        // look at east
        if( k < MAZE_MAX_X-1 )
          if((traversalMap[j][k]&0x02)==0 || (traversalMap[j][k]&0x20)==0)
            if(potential[j][t]+1 < potential[j][t+1])
              potential[j+1][t] = potential[j][t] + 1;
        // look at south
        if( k < MAZE_MAX_Y-1 )
          if((traversalMap[j][k]&0x04)==0 || (traversalMap[j][k]&0x40)==0)
            if(potential[j][t]+1 < potential[j+1][t])
              potential[j+1][t] = potential[j][t] + 1;
        // look at west
        if( k > 0 )
          if((traversalMap[j][k]&0x08)==0 || (traversalMap[j][k]&0x80)==0)
            if(potential[j][t]+1 < potential[j][t-1])
              potential[j-1][t] = potential[j][t] + 1;
        // look for start position
        if(j == initX && k == initY)
          return 1;
      }
    }
    // Could not find start
    if(foundFlag) break;
  }
  return 0;
}

/***************chooseTurn**********************
 *  Choose if the bot should turn or not based on the 
 *  current direction and the direction the bot needs
 *  to turn next
 */
int chooseTurn(int nxtDirec, int curDirec){
  if(curDirec == nxtDirec){
    return 0;
  }else if((curDirec+3)%4 == nxtDirec){
    return 1;
  }else if((curDirec+1)%4 == nxtDirec){
    return 2;
  }else{
    return 3;
  }
}

void setup(){
  initializeTraversal();
}

void loop(){
  readSetTraversalMap(locx, loxy, curDirec, traversalMap);
  updatePotential(locx, locy, MAZE_MAX_X/2 - 1, MAZE_MAX_Y/2 - 1, curDirec, traversalMap);
  nxtDirec = decideNextDirec(locx, locy, potential, traversalMap);
  turnCommand = chooseTurn(nxtDirec, curDirec);
  switch(turnCommand){
    case 0: break; // Don't turn
    case 1: curDirec+=3; curDirec%=4; break; // Turn 90 degrees left
    case 2: curDirec+=1; curDirec%=4; break; // Turn 90 degrees right
    case 3: curDirec+=2; curDirec%=4; break; // 180 degree turn
  }
  switch(curDirec){
    case 0: locy--; break;
    case 1: locx++; break;
    case 2: locy++; break;
    case 3: locx--; break;
  }
}
