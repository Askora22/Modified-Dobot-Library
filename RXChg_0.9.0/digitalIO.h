
    BTU.setMode(OneShot);
    BTU.assign(UP_BUTTON_PIN);
    BTU.turnOnPullUp();
    BTD.setMode(OneShot);
    BTD.assign(DOWN_BUTTON_PIN);
    BTD.turnOnPullUp();
    BTL.setMode(OneShot);
    BTL.assign(RIGHT_BUTTON_PIN);
    BTL.turnOnPullUp();
    BTR.setMode(OneShot);
    BTR.assign(LEFT_BUTTON_PIN);
    BTR.turnOnPullUp();
    BTE.setMode(OneShot);
    BTE.assign(CONFIRM_BUTTON_PIN);
    BTE.turnOnPullUp();
    BTE.setTimer(3000);
    BTE.setLogic(Invert);
    BTC.setMode(Timer);
    BTC.assign(ESCAPE_BUTTON_PIN);
    BTC.turnOnPullUp();
    BTC.setTimer(5000);
    BTC.setLogic(Invert);

    BTM.setMode(OneShot);
    BTM.assign(MODE_BUTTON_PIN);
    BTM.turnOnPullUp();
    BTG.setMode(OneShot);
    BTG.assign(GO_BUTTON_PIN);
    BTG.turnOnPullUp();
    VAC.setMode(OneShot);
    VAC.assign(VAC_SWITCH);

    pinMode(outPin[0], OUTPUT);
    pinMode(outPin[1], OUTPUT);
    pinMode(outPin[2], OUTPUT);
    pinMode(outPin[3], OUTPUT);
    pinMode(outPin[4], OUTPUT);
    pinMode(outPin[5], OUTPUT);
    pinMode(outPin[6], OUTPUT);
    pinMode(outPin[7], OUTPUT);
    
