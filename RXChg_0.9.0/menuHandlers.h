extern uint8_t makeChar(uint8_t value);
extern void setSpeed(uint8_t rspeed, uint8_t hspeed);
extern float Dobot_GetPose(uint8_t temp);
extern void robotProcess(uint8_t);
  uint8_t modeFlag;

uint8_t checkCase(uint8_t mode)
{
  uint8_t cass;
  while(cass!=(mode&0x07))
  {
    cass = station[0].caseSet()+(station[1].caseSet()*2)+(station[2].caseSet()*4);
    if(((millis()/100)%10)>5)
      digitalWrite(outPin[2], HIGH);
    else
      digitalWrite(outPin[2], LOW);
  }
  digitalWrite(outPin[2], LOW);
}

/*********************************************************************************************************
** Function name:       msc
** Descriptions:        This displays the operator screen and updates according to run status
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void msc(){

  static uint8_t modeInvert;

  if(BTM.check())
    modeInvert = !modeInvert;
  
  dt = clock.getDateTime();
  humidity1 = dht.readHumidity();
  temperature1 = dht.readTemperature();
  if(humidity1>1)
    humidity = humidity1;
  if(temperature1>1)
    temperature = temperature1;
  // static char buf[30];
  strcpy(menu.sbuf, " ");
  if(dt.hour<10) strcat(menu.sbuf, "0");
  strcat(menu.sbuf,itoa((int)dt.hour,buf,10));
  strcat(menu.sbuf,":");
  if(dt.minute<10) strcat(menu.sbuf, "0");
  strcat(menu.sbuf,itoa((int)dt.minute,buf,10));
  strcat(menu.sbuf,"  ");
  if(dt.month<10) strcat(menu.sbuf, "0");
  strcat(menu.sbuf,itoa((int)dt.month,buf,10));strcat(menu.sbuf,"/");
  if(dt.day<10) strcat(menu.sbuf, "0");
  strcat(menu.sbuf,itoa((int)dt.day,buf,10));
  strcat(menu.sbuf,"/");strcat(menu.sbuf,itoa((int)dt.year,buf,10));
  strcat(menu.sbuf,"\n");
  switch(processFlag+(completed*5)){
    case 1:     // Main operation menu
      {
        modeFlag = station[0].caseSet()+(station[1].caseSet()*2)+(station[2].caseSet()*4) + (modeInvert * 8);
        strcpy_P(buf, (char *)pgm_read_word(&(states[modeFlag])));
        strcat(menu.sbuf,buf);
        if(((millis()/1000)%60)>45)
        {
          strcat(menu.sbuf,"\n  ");strcat(menu.sbuf,thermometer);strcat(menu.sbuf,": ");strcat(menu.sbuf,itoa((int)temperature,buf,10));strcat(menu.sbuf,".");strcat(menu.sbuf,itoa((int)((temperature-(int)temperature)*10),buf,10));
          strcat(menu.sbuf,"  ");strcat(menu.sbuf,drip);strcat(menu.sbuf,": ");strcat(menu.sbuf,itoa((int)humidity,buf,10));strcat(menu.sbuf,".");strcat(menu.sbuf,itoa((int)((humidity-(int)humidity)*10),buf,10));
        }
        else
        {
          strcat(menu.sbuf,"\n Uptime (s): ");strcat(menu.sbuf,itoa((int)(millis()/1000),buf,10));//2nd lcd line
        }

          
        strcat(menu.sbuf,"\n  Free mem  : ");strcat(menu.sbuf,itoa((int)menu.freeRam(),buf,10));//3rd lcd line
        if(BTG.check())
        {
          robotProcess(modeFlag);
        }
        break;
      }
    case 2:     // Please Wait Message
      {
        strcpy_P(buf, (char *)pgm_read_word(&(states[17])));
        strcat(menu.sbuf, buf);
        break;
      }
    case 3:     // Change Case Routine
      {
        strcpy_P(buf, (char *)pgm_read_word(&(states[18])));
        strcat(menu.sbuf, buf);
        break;
      }
    case 4:
      break;
    case 5:
      break;
    case 6:     // Operation Complete
      {
        strcpy_P(buf, (char *)pgm_read_word(&(states[19])));
        strcat(menu.sbuf, buf);
        break;
      }
    case 7:
      break;
    case 8:
      break;
      
  }
  menu.drawUsrScreen(menu.sbuf);
}

/*********************************************************************************************************
** Function name:       waitForRobot
** Descriptions:        Checks pose status of robot and waits for it to get to position called for
** Input parameters:    Position to wait for and offset
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void waitForRobot(coordinates station, float offset){
  float error[4];
  error[4] = 100;
  Pose pose;
  while(error[4]>=.8){
    GetPose(&pose);
    error[0] = station.x - pose.x;
    error[1] = station.y - pose.y;
    error[2] = station.z - (pose.z - offset);
    error[3] = station.r - pose.rHead;
    error[4] = abs(error[0]) + abs(error[1]) + abs(error[2]) + abs(error[3]);
  }
  delay(1000);
}

/*********************************************************************************************************
** Function name:       moveRobot
** Descriptions:        Moves robot to station defined in call
** Input parameters:    toStation
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void moveRobot(uint8_t toStation){
  checkCase(modeFlag);
  coordinates curStation;
  if(toStation==5){
    curStation.x = 200;
    curStation.y = 0;
    curStation.z = 0;
    curStation.r = 0;
  }
  else
  {
    curStation = station[toStation].getCoords();
  }
  setSpeed(2, 10);
  gPTPCmd.ptpMode = JUMP_XYZ;
  gPTPCmd.x = curStation.x;
  gPTPCmd.y = curStation.y;
  gPTPCmd.z = curStation.z;
  gPTPCmd.rHead = curStation.r;
  SetPTPCmd(&gPTPCmd);
  // robotReply = ProtocolProcess();
  /*
  while(robotReply)
  {
    Serial.print("Current Index is ");
    Serial.println(GetQueuedCmdCurrentIndex());
  }
  Serial.print("End of move, QuedCommandIndex = ");
  Serial.println(GetQueuedCmdCurrentIndex());

  Serial.print("Robot reply is: ");
  Serial.println(robotReply);
  */
  waitForRobot(curStation, 0);

  delay(500);
}

/*********************************************************************************************************
** Function name:       resetRobot
** Descriptions:        Reset the robot, turn off vacuum and clear alarms
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void resetRobot(){
  digitalWrite(outPin[1], HIGH);
  Dobot_SetEndEffectorSuctionCup(0);
  moveRobot(5);
  ClearAllAlarmsState();
  delay(1000);
  
  SetHomeCmd();
  ProtocolProcess();
  delay(30000);
  moveRobot(4);
  delay(1000);
  digitalWrite(outPin[1], LOW);
}

/*********************************************************************************************************
** Function name:       robotProcess
** Descriptions:        Based on mode selected, run robot through defined path
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void robotProcess(uint8_t modeFlag){
  coordinates curStation;
  digitalWrite(outPin[1],HIGH);
  /*
  setSpeed(2,10);
  ProtocolProcess();
  delay(1000);
  */
  switch(modeFlag+1)
  {
    case 1:
      break;
    case 2:
      processFlag = 2;
      msc();
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 3:
      processFlag = 2;
      msc();
      moveRobot(1);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(1);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 4:
      processFlag = 2;
      msc();
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(1);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 5:
      processFlag = 2;
      msc();
      moveRobot(2);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(2);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 6:
      processFlag = 2;
      msc();
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(2);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 7:
      processFlag = 2;
      msc();
      moveRobot(1);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(2);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 8:
      break;
    case 9:
      break;
    case 10:
      processFlag = 2;
      msc();
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      processFlag = 3;
      msc();
      while(!(BTG.check())){}
      processFlag = 2;
      msc();
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 11:
      processFlag = 2;
      msc();
      moveRobot(1);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      processFlag = 3;
      msc();
      while(!(BTG.check())){}
      processFlag = 2;
      msc();
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 12:
      processFlag = 2;
      msc();
      moveRobot(1);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 13:
      processFlag = 2;
      msc();
      moveRobot(2);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      processFlag = 3;
      msc();
      while(!(BTG.check())){}
      processFlag = 2;
      msc();
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(2);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 14:
      processFlag = 2;
      msc();
      moveRobot(2);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(0);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    case 15:
      processFlag = 2;
      msc();
      moveRobot(2);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      delay(10000);
      moveRobot(3);
      Dobot_SetEndEffectorSuctionCup(1);
      delay(1000);
      moveRobot(1);
      Dobot_SetEndEffectorSuctionCup(0);
      delay(1000);
      moveRobot(4);
      completed = 1;
      break;
    default:
      break;
  }
  processFlag = 1;
  digitalWrite(outPin[1],LOW);
}

/*********************************************************************************************************
** Function name:       diScreen
** Descriptions:        Handles the display of the digital input diagnostic screen
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void diScreen(){
  uint8_t lastRow = numDI/4;
  uint8_t lastCol = numDI%4;
  lastRow += (lastCol)>0;
  if (lastCol==0)
    lastCol = 4;   

  uint8_t curItem = 0;
  uint8_t drawStart = 0;
  uint8_t curRow = 0;
  uint8_t curCol = 0;
  uint8_t selected = 0;  

  do {
    strcpy_P(menu.sbuf, (char *)pgm_read_word(&(messages[1])));                          // Title row of Digital Input Screen
    for (uint8_t rowLoop = 0; rowLoop <= 2; rowLoop++) {
      for (uint8_t colLoop = 0; colLoop <= 3; colLoop++){
        curItem = ((rowLoop*4)+drawStart+colLoop);
        if(curItem >= numDI)
          break;
        if(selected == curItem)
          strcat(menu.sbuf, nCursor);
        else
          strcat(menu.sbuf, blank);
        strcat(menu.sbuf, "I");
        if(inPin[curItem]<10)
          strcat(menu.sbuf, "0");
        strcat(menu.sbuf,itoa((int)inPin[curItem],buf,10));
        if(digitalRead(inPin[curItem]))
          strcat(menu.sbuf, bHigh);
        else
          strcat(menu.sbuf, bLow);
      }
      strcat(menu.sbuf,"\n");
    }
    menu.drawUsrScreen(menu.sbuf);    
    if (BTU.check()&&(curRow>=1))
      curRow = curRow -1;
    if (BTD.check()&&(curRow<lastRow-1))
      curRow = curRow + 1;
    if (curRow > 2)
      drawStart = (curRow - 2)*4;
    else if (curRow < 2)
      drawStart = 0;
    if (BTR.check()&&(curCol<3))
      curCol = curCol+1;
    if (BTL.check()&&(curCol>0))
      curCol = curCol - 1;
    if (((curRow*4)+curCol)>=numDI)
      curCol = lastCol - 1;
    selected = (curRow*4) + curCol;
  } while ((BTC.check())<4);
}

/*********************************************************************************************************
** Function name:       doScreen
** Descriptions:        Handles the display of the digital input diagnostic screen
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void doScreen(){
  uint8_t lastRow = numDO/4;
  uint8_t lastCol = numDO%4;
  lastRow += (lastCol)>0;
  if (lastCol==0)
    lastCol = 4;   

  uint8_t curItem = 0;
  uint8_t drawStart = 0;
  uint8_t curRow = 0;
  uint8_t curCol = 0;
  uint8_t selected = 0;  

  do {
    strcpy_P(menu.sbuf, (char *)pgm_read_word(&(messages[2])));               // Title row of Digital Output Screen
    for (uint8_t rowLoop = 0; rowLoop <= 2; rowLoop++) {
      for (uint8_t colLoop = 0; colLoop <= 3; colLoop++) {
        curItem = ((rowLoop*4)+drawStart+colLoop);
        if(curItem >= numDO)
        break;
        if(selected == curItem)
          strcat(menu.sbuf, nCursor);
        else
          strcat(menu.sbuf, blank);
        strcat(menu.sbuf, "O");
        if(outPin[curItem]<10)
          strcat(menu.sbuf, "0");
        strcat(menu.sbuf,itoa((int)outPin[curItem],buf,10));
        if(digitalRead(outPin[curItem]))
          strcat(menu.sbuf, bLow);
        else
          strcat(menu.sbuf, bHigh);
      }
      strcat(menu.sbuf,"\n");
    }
    menu.drawUsrScreen(menu.sbuf);
    if (BTU.check()&&(curRow>=1))
      curRow = curRow -1;
    if (BTD.check()&&(curRow<lastRow-1))
      curRow = curRow + 1;
    if (curRow > 2)
      drawStart = (curRow - 2)*4;
    else if (curRow < 2)
    drawStart = 0;
    if (BTR.check()&&(curCol<3))
      curCol = curCol+1;
    if (BTL.check()&&(curCol>0))
      curCol = curCol - 1;
    if (((curRow*4)+curCol)>=numDO)
      curCol = lastCol - 1;
    selected = (curRow*4) + curCol;
    if (BTE.check()) {
      uint8_t currentState;
      currentState = digitalRead(outPin[selected]);
      digitalWrite(outPin[selected], (!currentState));
    }
  } while ((BTC.check())<4);
}

/*********************************************************************************************************
** Function name:       storeJoy
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void storeJoy(uint8_t selected){
  Serial.println("Store joystick data");
  selected = selected + 1;
  uint16_t volatile analog;
  joy_params saveJoy;
  BTG.check();
  char message[] = "                    ";
  if(selected == 1){
    strcpy(message, " Joy X to MIN -- GO ");
  }
  if(selected == 2){
    strcpy(message, " Joy Y to MIN -- GO ");
  }
  for(int i = 0; i<sizeof(message); i++)    // removed volatile
  {
    menu.sbuf[(41)+i] = message[i];
  }
  menu.drawUsrScreen(menu.sbuf);
  
  do{
    analog = analogRead(aiPin[selected-1]);
  } while(!(BTG.check()));
  saveJoy.jmin = analog;

  if(selected == 1){
    strcpy(message, " Joy X to MID -- GO ");
  }
  if(selected == 2){
    strcpy(message, " Joy Y to MID -- GO ");
  }
  for(int volatile i = 0; i<sizeof(message); i++)
  {
    menu.sbuf[(41)+i] = message[i];
  }
  menu.drawUsrScreen(menu.sbuf);
  
  do{
    analog = analogRead(aiPin[selected-1]);
  } while(!(BTG.check()));
  saveJoy.jmid = analog;

  if(selected == 1){
    strcpy(message, " Joy X to MAX -- GO ");
  }
  if(selected == 2){
    strcpy(message, " Joy Y to MAX -- GO ");
  }
  for(int volatile i = 0; i<sizeof(message); i++)
  {
    menu.sbuf[(41)+i] = message[i];
  }
  menu.drawUsrScreen(menu.sbuf);
  
  do{
    analog = analogRead(aiPin[selected-1]);
  } while(!(BTG.check()));
  saveJoy.jmax = analog;

  EEPROM.updateBlock(joy_stg[selected], saveJoy);

  #ifdef TRACE
  strcpy_P(buf, (char *)pgm_read_word(&(traces[2])));
  Serial.print(buf);
  Serial.print(joy[0].jmin);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[3])));
  Serial.print(buf);
  Serial.print(joy[0].jmid);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[4])));
  Serial.print(buf);
  Serial.print(joy[0].jmax);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[5])));
  Serial.print(buf);
  Serial.print(joy[1].jmin);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[6])));
  Serial.print(buf);
  Serial.print(joy[1].jmid);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[7])));
  Serial.print(buf);
  Serial.println(joy[1].jmax);
  Serial.println("");
  #endif
  
  strcpy(message, "                    ");
  for(int volatile i = 0; i<sizeof(message); i++)
  {
    menu.sbuf[(41)+i] = message[i];
  }
  menu.drawUsrScreen(menu.sbuf);
  
}

/*********************************************************************************************************
** Function name:       aiScreen (analog Input)
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void aiScreen(){
  BTE.setMode(Timer);
  BTE.setLogic(Invert);
  BTE.setTimer(3000);
  static const char subScreen[] PROGMEM = "***ANALOG INPUTS***\n                    \n                    \n                    ";
  strcpy_P(menu.sbuf,(char *)subScreen);

  uint8_t lastRow = numAI/2;
  uint8_t lastCol = numAI%2;
  lastRow += (lastCol)>0;
  if (lastCol==0)
    lastCol = 2;   

  uint8_t colStep = 10;
  uint8_t rowStep = 21;
  uint8_t curItem = 0;
  uint8_t drawStart = 0;
 

  uint8_t curRow = 0;
  uint8_t curCol = 0;
  uint8_t navCursor = 0;
  static uint8_t prvCursor = 1;
  static uint8_t prvCol = 0;
  uint8_t selected = 0;

  uint16_t analog = 0;
  uint16_t analogD = 0;

do {
  for (uint8_t rowLoop = 1; rowLoop <= 3; rowLoop++) {
    for (uint8_t colLoop = 0; colLoop <= 1; colLoop++) {
      curItem = (((rowLoop-1) * 4) + colLoop)+(drawStart * 4);
      analog = analogRead(aiPin[curItem]);
      analogD = analog;
      // analogD = map(analog, 0, 1023, 0, 9);
      if (curItem<numAI) {
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 0))] = 65;
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 1))] = makeChar(aiPin[curItem]);
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 2))] = makeChar((aiPin[curItem]%10));
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 4))] = makeChar(analogD/1000);
        if (analogD>1000)
          analogD = analogD - 1000;
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 5))] = makeChar(analogD/100);
        if (analogD>100)
          analogD = analogD - ((analogD/100)*100);
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 6))] = makeChar(analogD/10);
        if (analogD>10)
          analogD = analogD - ((analogD/10)*10);
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 7))] = makeChar(analogD);

        // menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 4))] = (digitalRead(inPin[curItem]) + 6);
      } else {
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 0))] = 32;
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 1))] = 32;
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 2))] = 32;
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 4))] = 32;
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 5))] = 32;
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 6))] = 32;
        menu.sbuf[((rowLoop * rowStep) + ((colLoop * colStep) + 7))] = 32;
      }
    }
    
    if (BTU.check()&&(curRow>=1)) {
      curRow = curRow - 1;
    }
    if (BTD.check()&&(curRow<lastRow-1)) {
      curRow = curRow + 1;
    }

    if (curRow > 2)
      drawStart = curRow - 2;
    else if (curRow < 2)
      drawStart = 0;

    if (BTR.check()&&(curCol<1)) {
      curCol = curCol+1;
    }
    if (BTL.check()&&(curCol>0)) {
      curCol = curCol-1;
    }

    if (((curRow*4)+curCol)>=numDI) {
      curCol = (lastCol-1);
    }
    selected = (curRow*4) + curCol;

    navCursor = (curRow);
    if (navCursor > 2)
      navCursor = 2;
    menu.sbuf[(((prvCursor + 1) * rowStep + (prvCol * colStep))-1)] = 32;
    menu.sbuf[(((navCursor + 1) * rowStep + (curCol * colStep))-1)] = 4;
    prvCursor = navCursor;
    prvCol = curCol;
  };
  if (BTE.check()>3) {
    storeJoy(selected);
    menu.sbuf[1] = 252;
    menu.drawUsrScreen(menu.sbuf);
    delay(2000);
    menu.sbuf[1] = 42;
    Serial.print("Currently selected item is ");
    Serial.println(selected);
  }

    menu.drawUsrScreen(menu.sbuf);
  } while ((BTC.check())<4);

  BTE.setMode(OneShot);
  BTE.setLogic(Invert);
  
  Serial.println(menu.sbuf);  
}



/*********************************************************************************************************
** Function name:       robotScreen
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void robotScreen(uint8_t choice){
  joy_stg[0]        = 0;
  joy_stg[1]        = 6;

  EEPROM.readBlock(joy_stg[0], joy[0]);
  EEPROM.readBlock(joy_stg[1], joy[1]);
  
  strcpy_P(buf, (char *)pgm_read_word(&(traces[0])));
  Serial.print(buf);
  Serial.print(joy_stg[0]);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[1])));
  Serial.print(buf);
  Serial.println(joy_stg[1]);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[2])));
  Serial.print(buf);
  Serial.print(joy[0].jmin);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[3])));
  Serial.print(buf);
  Serial.print(joy[0].jmid);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[4])));
  Serial.print(buf);
  Serial.print(joy[0].jmax);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[5])));
  Serial.print(buf);
  Serial.print(joy[1].jmin);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[6])));
  Serial.print(buf);
  Serial.print(joy[1].jmid);
  strcpy_P(buf, (char *)pgm_read_word(&(traces[7])));
  Serial.print(buf);
  Serial.print(joy[1].jmax);
  Serial.println("");

  // static const char subScreen[] PROGMEM = "**TEACH STATION X***\n L/R TO SELECT AXIS \nU/D TO SELECT SPEED \n     GO TO SAVE     ";
  // strcpy_P(menu.sbuf,(char *)subScreen);
  strcpy_P(menu.sbuf, (char *)pgm_read_word(&(messages[4])));
  menu.sbuf[16] = 49+choice;
  menu.drawUsrScreen(menu.sbuf);
  delay(3000);
  uint8_t joyX, joyY, lastX, lastY;
  uint16_t readX, readY, centerX, centerY;

  uint16_t joyX_limit[] = {0, (joy[0].jmid - 384), (joy[0].jmid - 128), (joy[0].jmid + 128), (joy[0].jmid + 384)};
  uint16_t joyY_limit[] = {0, (joy[1].jmid - 384), (joy[1].jmid - 128), (joy[1].jmid + 128), (joy[1].jmid + 384)};

  joyX = joyY = lastX = lastY = 2;
  uint8_t q_loop;
  uint8_t rspeed = 1;
  uint8_t hspeed = 1;
  uint8_t modeX = 1;
  BTG.check();

  for(; ;)
    {
      readX = map(analogRead(A0),joy[0].jmin,joy[0].jmax,0,1023);
      #ifdef HW_VERSION
      readY = map(analogRead(A1),joy[1].jmin,joy[1].jmax,0,1023);
      #else
      readY = 1023 - (map(analogRead(A1),joy[1].jmin, joy[1].jmax, 0, 1023));
      #endif
      /*
      readX = map(analogRead(A0),40,1023,0,1023);
      #ifdef HW_VERSION
      readY = map(analogRead(A1),0,952,0,1023);
      #else
      readY = 1023 - (map(analogRead(A1),0, 1023, 0, 1023);)
      #endif
      */    
      for(q_loop = 0; q_loop<5; q_loop++){
        if(readX>(joyX_limit[q_loop]-40))
          joyX = q_loop;
        if(readY>(joyY_limit[q_loop]-40))
          joyY = q_loop;
      }

      // bjoyX = constrain(readX, 0, 4);
      // joyY = constrain(readY, 0, 4);
               
      if(joyX!=2)
        joyY = 2;

      joyX++;
      joyY++;

      if(BTU.check()){
        rspeed = rspeed + 1;
        if(rspeed > 5)
          rspeed = 5;
      }
      if(BTD.check()){
        rspeed = rspeed - 1;
        if(rspeed < 1)
          rspeed = 1;
      }
      if(BTL.check()){
        modeX = 1;
      }
      if(BTR.check()){
        modeX = 5;
      }
      if(BTM.check()){
        resetRobot();
        // ProtocolProcess();
        Serial.println("The robot should be going to HOME Now!");
      }
      if(BTG.check()){
        coordinates curStation;

        curStation.x = Dobot_GetPose(1);
        Serial.print("X = ");
        Serial.print(curStation.x,4);
        curStation.y = Dobot_GetPose(2);
        Serial.print(", Y = ");
        Serial.print(curStation.y,4);
        curStation.z = Dobot_GetPose(3);
        Serial.print(", Z = ");
        Serial.print(curStation.z,4);
        curStation.r = Dobot_GetPose(4);
        Serial.print(", R = ");
        Serial.println(curStation.r,4);
        Serial.print("Joint 1 Angle = ");
        Serial.print(Dobot_GetPose(5),4);
        Serial.print(", Joint 2 Angle = ");
        Serial.print(Dobot_GetPose(6),4);
        Serial.print(", Joint 3 Angle = ");
        Serial.print(Dobot_GetPose(7),4);
        Serial.print(", Joint 4 Angle = ");
        Serial.println(Dobot_GetPose(8),4);

        Serial.print("Choice is ");
        Serial.print(choice);
        station[choice].setCoords(curStation);
        break;
      }

      if(joyX!=lastX){
        switch(joyX){
          case 1:
            hspeed = 10;
            setSpeed(rspeed, hspeed);
            gJOGCmd.cmd = modeX;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
          case 2:
            hspeed = 1;
            setSpeed(rspeed, hspeed);
            gJOGCmd.cmd = modeX;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
          case 3:
            gJOGCmd.cmd = IDLE;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
          case 4:
            hspeed = 1;
            setSpeed(rspeed, hspeed);
            gJOGCmd.cmd = modeX + 1;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
          default:
            hspeed = 10;
            setSpeed(rspeed, hspeed);
            gJOGCmd.cmd = modeX + 1;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
        }
      }

      if(joyY!=lastY){
        switch(joyY){
          case 1:
            hspeed = 10;
            setSpeed(rspeed, hspeed);
            gJOGCmd.cmd = modeX + 3;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
          case 2:
            hspeed = 1;
            setSpeed(rspeed, hspeed);
            gJOGCmd.cmd = modeX + 3;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
          case 3:
            gJOGCmd.cmd = IDLE;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
          case 4:
            hspeed = 1;
            setSpeed(rspeed, hspeed);
            gJOGCmd.cmd = modeX + 2;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
            break;
          default:
            hspeed = 10;
            setSpeed(rspeed, hspeed);
            gJOGCmd.cmd = modeX + 2;
            gJOGCmd.isJoint = COORDINATE_MODEL;
            SetJOGCmd(&gJOGCmd);
            ProtocolProcess();
          break;
        }
      }
      lastX = joyX;
      lastY = joyY;
      joyX = 0;
      joyY = 0;
    }

  // strcpy(menu.sbuf, "\n   POSITION SAVED   \n\nHOLD CANCEL TO EXIT.");
  strcpy_P(menu.sbuf, (char *)pgm_read_word(&(messages[5])));
  menu.drawUsrScreen(menu.sbuf);
  while ((BTC.check())<4);

  resetRobot();
  Serial.println(menu.sbuf);  
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void teach1(){
  robotScreen(0);
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void teach2(){
  robotScreen(1);
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void teach3(){
  robotScreen(2);
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void teach4(){
  robotScreen(3);
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void teach5(){
  robotScreen(4);
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void printCoords(){
  Pose pose;
  GetPose(&pose);
  Serial.print("Pose -- X: ");
  Serial.print(pose.x, 4);
  Serial.print(", Y: ");
  Serial.print(pose.y, 4);
  Serial.print(", Z: ");
  Serial.print(pose.z, 4);
  Serial.print(", R: ");
  Serial.println(pose.rHead, 4);
  Serial.print("Angles -- Joint 1: ");
  Serial.print(pose.jointAngle[0], 4);
  Serial.print(", Joint 2 : ");
  Serial.print(pose.jointAngle[1], 4);
  Serial.print(", Joint 3 : ");
  Serial.print(pose.jointAngle[2], 4);
  Serial.print(", Joint 4 : ");
  Serial.println(pose.jointAngle[3], 4);
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
uint8_t initRobot(){
  static char buf[30];
  BTG.check();
  digitalWrite(outPin[2], HIGH);
  while(!(BTG.check()))
  {
    strcpy_P(buf, (char *)pgm_read_word(&(states[16])));
    strcpy(menu.sbuf, buf);
    menu.drawUsrScreen(menu.sbuf);
  }
  digitalWrite(outPin[2], LOW);
  return 1;
}
