//Stepper Utils
struct REF_AXIS { 
    //IN
    bool IN_xLifeBit;
    bool IN_xPowerOnReq;
    bool IN_xStartReq;
    bool IN_xStopReq;     
    int IN_iModeReq;   
    int IN_iTargetPosition;
    int IN_iTargetSpeed;
    int IN_iTargetAcc;
    int IN_iTargetDec;

    //OUT
    bool OUT_xLifeBitFB;    
    bool OUT_xPowered;
    bool OUT_xRunning;   
    int OUT_iActualPosition;
    int OUT_iActualSpeed;   
    int OUT_iCheckSum;   
} ; 
