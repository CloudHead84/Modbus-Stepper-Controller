//Stepper Utils
struct REF_AXIS { 
    //IN
    bool IN_xLifeBit;
    bool IN_xPowerOnReq;
    bool IN_xStartReq;
    bool IN_xStopReq;     
    int IN_iModeReq;   
    long IN_diTargetPosition;   
    float IN_rTargetSpeed;
    float IN_rTargetAcc;
    float IN_rTargetDec;

    //OUT
    bool OUT_xLifeBitFB;    
    bool OUT_xPowered;
    bool OUT_xRunning;   
    long OUT_diActualPosition;  
    float OUT_rActualSpeed;   
    int OUT_iCheckSum;   
} ; 

//-----------------------------------------------------------------------------
// Concatenate two words into an long
//-----------------------------------------------------------------------------
long create_long(word low, word high)
{
    long result;
    result = (long)(high << 16) | (long)low;
    return result;
}

//-----------------------------------------------------------------------------
// deConcatenate long into two words
//-----------------------------------------------------------------------------
long create_words(long in, bool selection)
{
    word result;
    long tmpLong;

    if (selection == false)
    {
      tmpLong = (in << 16);
      tmpLong = (tmpLong >> 16);
      result = (word)tmpLong;
    }
    else
    {
      tmpLong = (in >> 16);
      result = (word)tmpLong;
    }
    return result;
}
