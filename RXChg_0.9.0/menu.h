  _menu *r,*s1,*s2;
  _var *v; 
  int  mem;

  mem=menu.freeRam();
  
  // inizialize the menu object (20 colums x 4 rows)
  menu.begin(&lcd,20,4);
  //menu.setBehaviour(MW_MENU_INDEX,false);
  menu.setBehaviour(MW_MENU_INDEX,false);    

  //create the menu tree
  r=menu.addMenu(MW_ROOT,NULL,F("**MAINTENANCE MENU**"));
  
    s1=menu.addMenu(MW_VAR,r,F("1) DIGITAL INPUTS"));
      s1->addVar(MW_ACTION,diScreen);
      s1->setBehaviour(MW_ACTION_CONFIRM, false);
 
    s1=menu.addMenu(MW_VAR,r,F("2) DIGITAL OUTPUTS"));
      s1->addVar(MW_ACTION,doScreen);
      s1->setBehaviour(MW_ACTION_CONFIRM,false);

    s1=menu.addMenu(MW_VAR,r,F("3) ANALOG INPUTS"));
      s1->addVar(MW_ACTION,aiScreen);
      s1->setBehaviour(MW_ACTION_CONFIRM,false);

    s1=menu.addMenu(MW_VAR,r,F("4) ANALOG OUTPUTS"));
      s1->addVar(MW_ACTION,act);

    s1=menu.addMenu(MW_VAR,r,F("5) SET DATE/TIME"));
      s1->addVar(MW_ACTION,act);
      
    s1=menu.addMenu(MW_VAR,r,F("6) ENVIR SENSORS"));
      s1->addVar(MW_ACTION,act);
   
    s1=menu.addMenu(MW_SUBMENU,r,F("7) TEACH ROBOT"));
      s2=menu.addMenu(MW_VAR,s1,F("STATION 1"));
        s2->addVar(MW_ACTION,teach1);
      s2=menu.addMenu(MW_VAR,s1,F("STATION 2"));
        s2->addVar(MW_ACTION,teach2);
      s2=menu.addMenu(MW_VAR,s1,F("STATION 3"));
        s2->addVar(MW_ACTION,teach3);
      s2=menu.addMenu(MW_VAR,s1,F("STATION 4"));
        s2->addVar(MW_ACTION,teach4);
      s2=menu.addMenu(MW_VAR,s1,F("STATION 5"));
        s2->addVar(MW_ACTION,teach5);
        
    s1=menu.addMenu(MW_VAR,r,F("8) RESET ROBOT"));
      s1->addVar(MW_ACTION,resetRobot);
      
    s1=menu.addMenu(MW_VAR,r,F("9) EXIT"));
      s1->addVar(MW_ACTION,menuQuit);
      s1->setBehaviour(MW_ACTION_CONFIRM, false);


  //declare navigation buttons (required)
  // menu.navButtons(UP_BUTTON_PIN,DOWN_BUTTON_PIN,ESCAPE_BUTTON_PIN,CONFIRM_BUTTON_PIN);
  menu.navButtons(UP_BUTTON_PIN,DOWN_BUTTON_PIN,RIGHT_BUTTON_PIN,LEFT_BUTTON_PIN,ESCAPE_BUTTON_PIN,CONFIRM_BUTTON_PIN);

  //User screen with 5mSec timeout to prevent access to menus
  menu.addUsrScreen(msc,5);

  //(optional) create a splash screen (duration 5.000 millis)with some usefull infos the character \n marks end of LCD line 
  //(tip): use preallocated internal menu.sbuf buffer to save memory space!
  // Splash Screen
  // strcpy(menu.sbuf," X-FAB TEXAS, INC.\n\n  RETICLE HANDLER\n  firmware: v");
  strcpy_P(menu.sbuf, (char *)pgm_read_word(&(messages[0])));
  strcat(menu.sbuf,FIRMWARE_VERSION);
  strcat(menu.sbuf, signature);
  menu.addSplash((char *) menu.sbuf, 5000);
