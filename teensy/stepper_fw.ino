// -------------------------------------------------------------------------------------------
// ------------------ INCLUDE ----------------------------------------------------------------
// -------------------------------------------------------------------------------------------
#include <NativeEthernet.h>
#include <Arduino.h>
#include "TeensyThreads.h"
#include "modbus.h"
#include "stepper_utils.h"
#include <AsyncStepper.hpp>

// -------------------------------------------------------------------------------------------
// ------------------ GLOBAL VARS ------------------------------------------------------------
// -------------------------------------------------------------------------------------------

// Ethernet
IPAddress ip(192, 168, 0, 118);
EthernetServer ethServer(502);
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// Config Stepper 0
#define StepPin_Stepper0      0
#define DirPin_Stepper0       1
#define EnablePin_Stepper0    2
#define SwitchPin_Stepper0    3
#define FullSteps_Stepper0    400

// Config Stepper 1
#define StepPin_Stepper1      6
#define DirPin_Stepper1       7
#define EnablePin_Stepper1    8
#define SwitchPin_Stepper1    9
#define FullSteps_Stepper1    400

// Config Stepper 2
#define StepPin_Stepper2      24
#define DirPin_Stepper2       25
#define EnablePin_Stepper2    26
#define SwitchPin_Stepper2    27
#define FullSteps_Stepper2    400

// Config Stepper 3
#define StepPin_Stepper3      28
#define DirPin_Stepper3       29
#define EnablePin_Stepper3    30
#define SwitchPin_Stepper3    31
#define FullSteps_Stepper3    400

//stepper config
STEPPER_CONFIG StepperConfig[4] = { {FullSteps_Stepper0, DirPin_Stepper0, StepPin_Stepper0, EnablePin_Stepper0, SwitchPin_Stepper0},
                                    {FullSteps_Stepper1, DirPin_Stepper1, StepPin_Stepper1, EnablePin_Stepper1, SwitchPin_Stepper1},
                                    {FullSteps_Stepper2, DirPin_Stepper2, StepPin_Stepper2, EnablePin_Stepper2, SwitchPin_Stepper2},
                                    {FullSteps_Stepper3, DirPin_Stepper3, StepPin_Stepper3, EnablePin_Stepper3, SwitchPin_Stepper3} };

//steppers
AsyncStepper stepper[4] = { {FullSteps_Stepper0, DirPin_Stepper0, StepPin_Stepper0},
                            {FullSteps_Stepper1, DirPin_Stepper1, StepPin_Stepper1},
                            {FullSteps_Stepper2, DirPin_Stepper2, StepPin_Stepper2},
                            {FullSteps_Stepper3, DirPin_Stepper3, StepPin_Stepper3} };

//axis data
REF_AXIS axis[MAX_AXIS];

// Modbus things
unsigned char modbus_buffer[100];
int processModbusMessage(unsigned char *buffer, int bufferSize);
bool modbus_connected;

extern bool mb_discrete_input[MAX_DISCRETE_INPUT];
extern bool mb_coils[MAX_COILS];
extern uint16_t mb_input_regs[MAX_INP_REGS];
extern uint16_t mb_holding_regs[MAX_HOLD_REGS];

//helper variables
long AbsPos[MAX_AXIS];

// threads
int threadid;

// -------------------------------------------------------------------------------------------
// ------------------ MODBUS DATA ------------------------------------------------------------
// -------------------------------------------------------------------------------------------
void updateIO()
{
 for (int i = 0; i < MAX_AXIS ; i++) {

    //INs
    axis[i].IN_xLifeBit     = mb_coils[0+i*COIL_COUNT];
    axis[i].IN_xPowerOnReq  = mb_coils[1+i*COIL_COUNT];
    axis[i].IN_xStartReq    = mb_coils[2+i*COIL_COUNT];
    axis[i].IN_xStopReq     = mb_coils[3+i*COIL_COUNT];
          
    axis[i].IN_iModeReq           = (int)mb_holding_regs[0+i*HOLD_REGS_COUNT];   
    axis[i].IN_diTargetPosition   = create_long(mb_holding_regs[1+i*HOLD_REGS_COUNT],mb_holding_regs[2+i*HOLD_REGS_COUNT]);   
    axis[i].IN_rTargetSpeed       = word_to_float(mb_holding_regs[3+i*HOLD_REGS_COUNT]);   
    axis[i].IN_rTargetAcc         = word_to_float(mb_holding_regs[4+i*HOLD_REGS_COUNT]);
    axis[i].IN_rTargetDec         = word_to_float(mb_holding_regs[5+i*HOLD_REGS_COUNT]);  

    //OUTs        
    mb_discrete_input[0+i*DISCRETE_INPUT_COUNT] = axis[i].OUT_xLifeBitFB;
    mb_discrete_input[1+i*DISCRETE_INPUT_COUNT] = axis[i].OUT_xPowered;    
    mb_discrete_input[2+i*DISCRETE_INPUT_COUNT] = axis[i].OUT_xRunning;    
    mb_discrete_input[3+i*DISCRETE_INPUT_COUNT] = axis[i].OUT_xAck;   
    mb_discrete_input[4+i*DISCRETE_INPUT_COUNT] = axis[i].OUT_xSwitch;       
    mb_input_regs[0+i*INP_REGS_COUNT] = create_words(axis[i].OUT_diActualPosition,false);
    mb_input_regs[1+i*INP_REGS_COUNT] = create_words(axis[i].OUT_diActualPosition,true);
    mb_input_regs[2+i*INP_REGS_COUNT] = (int)axis[i].OUT_rActualSpeed;
  
    axis[i].OUT_wCheckSum =   (word)mb_holding_regs[0+i*HOLD_REGS_COUNT] 
                            + (word)mb_holding_regs[1+i*HOLD_REGS_COUNT]
                            + (word)mb_holding_regs[2+i*HOLD_REGS_COUNT]
                            + (word)mb_holding_regs[3+i*HOLD_REGS_COUNT]
                            + (word)mb_holding_regs[4+i*HOLD_REGS_COUNT]
                            + (word)mb_holding_regs[5+i*HOLD_REGS_COUNT];

    mb_input_regs[3+i*INP_REGS_COUNT] = axis[i].OUT_wCheckSum;   
    
    axis[i].OUT_xLifeBitFB = axis[i].IN_xLifeBit;   
  }   
}


void aborting()
{
     stepper[0].Stop();
     stepper[1].Stop();
     stepper[2].Stop();
     stepper[3].Stop();     
     stepper[0].Update();
     stepper[1].Update(); 
     stepper[2].Update(); 
     stepper[3].Update();  
}


// -------------------------------------------------------------------------------------------
// ------------------ SETUP ------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
  void setup() {

//Initialize pins
for (int i = 0; i < MAX_AXIS ; i++) {
   
    //enable
    pinMode(StepperConfig[i].EnablePin, OUTPUT);
    digitalWrite(StepperConfig[i].EnablePin, HIGH); 
    
    //reference switch
    pinMode(StepperConfig[i].SwitchPin, INPUT_PULLUP);
}

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial.println("Ethernet Modbus TCP Example");

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  Serial.println("Started Ethernet!");
  
  // start the server
  ethServer.begin();
  Serial.println("Started Ethernet Server!");
   
  // start the Modbus TCP server

  // stepper      

  threads.setMicroTimer(10);
  threads.addThread(thread_stepper);

}

// -------------------------------------------------------------------------------------------
// ------------------ STEPPER THREAD ---------------------------------------------------------
// -------------------------------------------------------------------------------------------
void thread_stepper() {
  while (1) {


for (int i = 0; i < MAX_AXIS ; i++) {

 
  if (not axis[i].IN_xStartReq and axis[i].OUT_xAck) 
  {
        axis[i].OUT_xAck = false;                  
  }

    
switch (axis[i].IN_iModeReq) {
  
    // homing-------------------------------------------------------------------------------
    case 20:

     if (not digitalRead(StepperConfig[i].SwitchPin))
      {
        stepper[i].Stop();
      }   
    
    else if (axis[i].IN_xStartReq and not axis[i].OUT_xAck) 
        {
        axis[i].OUT_xAck = true;

        if (axis[i].IN_rTargetSpeed > 0 )
        {
        stepper[i].SetSpeed(axis[i].IN_rTargetSpeed);  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 
        stepper[i].RotateContinuous(AsyncStepper::CW);  
        } 
        else    
        {
        stepper[i].SetSpeed(axis[i].IN_rTargetSpeed * -1);  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 
        stepper[i].RotateContinuous(AsyncStepper::CCW); 
        }
                    
    }
    break;    
    
       case 21:

     if (digitalRead(StepperConfig[i].SwitchPin))
      {
        stepper[i].Stop();
        stepper[i].SetAbsoluteStep(axis[i].IN_diTargetPosition); 
      }   
    
    else if (axis[i].IN_xStartReq and not axis[i].OUT_xAck) 
        {
        axis[i].OUT_xAck = true;

        if (axis[i].IN_rTargetSpeed > 0 )
        {
        stepper[i].SetSpeed(axis[i].IN_rTargetSpeed);  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 
        stepper[i].RotateContinuous(AsyncStepper::CW);  
        } 
        else    
        {
        stepper[i].SetSpeed(axis[i].IN_rTargetSpeed * -1);  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 
        stepper[i].RotateContinuous(AsyncStepper::CCW); 
        }
                    
    }
    break;  
    
    
    // velocity -------------------------------------------------------------------------------
    case 30:
    
    if (axis[i].IN_xStartReq and not axis[i].OUT_xAck) 
        {
        axis[i].OUT_xAck = true;

        if (axis[i].IN_rTargetSpeed > 0 )
        {
        stepper[i].SetSpeed(axis[i].IN_rTargetSpeed);  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 
        stepper[i].RotateContinuous(AsyncStepper::CW);  
        } 
        else    
        {
        stepper[i].SetSpeed(axis[i].IN_rTargetSpeed * -1);  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 
        stepper[i].RotateContinuous(AsyncStepper::CCW); 
        }
                    
    }
    break;

    // absolute -------------------------------------------------------------------------------
    case 40:
    
    if (axis[i].IN_xStartReq and not axis[i].OUT_xAck) 
        {
          
        axis[i].OUT_xAck = true;
        stepper[i].SetSpeed(axis[i].IN_rTargetSpeed);  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 


        AbsPos[0] = axis[i].IN_diTargetPosition - stepper[i].GetAbsoluteStep();

        
        if (AbsPos[0] > 0 )
        {
        stepper[i].Rotate(AbsPos[0],AsyncStepper::CW);
        }
        else
        { 
         stepper[i].Rotate(AbsPos[0],AsyncStepper::CCW);
        }
        
    }
    break;
    
    // relative -------------------------------------------------------------------------------
    case 50:
    
    if (axis[i].IN_xStartReq and not axis[i].OUT_xAck) 
        {
          
        axis[i].OUT_xAck = true;
        stepper[i].SetSpeed(axis[i].IN_rTargetSpeed);  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 
        if (axis[i].IN_diTargetPosition > 0 )
        {
        stepper[i].Rotate(axis[i].IN_diTargetPosition,AsyncStepper::CW);
        }
        else
        { 
         stepper[i].Rotate(axis[i].IN_diTargetPosition,AsyncStepper::CCW);
        }
       }
    
    break;

    
    default:
    // Statement(s)
    break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
}




    // stopping -------------------------------------------------------------------------------   
    if (axis[i].IN_xStopReq and stepper[i].GetCurrentSpeed() != 0 and not axis[i].OUT_xAck) 
    {
        axis[i].OUT_xAck = true;  
        stepper[i].SetAcceleration(axis[i].IN_rTargetAcc, axis[i].IN_rTargetDec); 
        stepper[i].Break();
    }

    //power on ------------------------------------------------------------------------------- 
    if (axis[i].IN_xPowerOnReq) 
    {
       digitalWrite(StepperConfig[i].EnablePin, LOW);
       axis[i].OUT_xPowered = true;
    }
    else
    {
       digitalWrite(StepperConfig[i].EnablePin, HIGH);
       axis[i].OUT_xPowered = false;
    }

     //status data
     axis[i].OUT_xSwitch = not digitalRead(StepperConfig[i].SwitchPin);
     axis[i].OUT_diActualPosition = stepper[i].GetAbsoluteStep(); 
     axis[i].OUT_rActualSpeed = stepper[i].GetCurrentSpeed();
     axis[i].OUT_xRunning = axis[i].OUT_rActualSpeed != 0;  
 

    }
     //update stepper object
     stepper[0].Update();
     stepper[1].Update(); 
     stepper[2].Update(); 
     stepper[3].Update();                     
     threads.yield();

  }
}

// -------------------------------------------------------------------------------------------
// ------------------ LOOP -------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
void loop() {
      // listen for incoming clients
  EthernetClient client = ethServer.available();
  
   if (!client){
        aborting();
        return;
   }
       
    
    Serial.println("new client!");

    while(client.connected())
    {
        // Wait until the client sends some data
        while(!client.available())
        {
            delay(1);
            if (!client.connected())
                  return;
        
        }
        
        int i = 0;
        while(client.available())
        {
            modbus_connected = true;
            modbus_buffer[i] = client.read();
            i++;
            if (i == 100)
                break;
        }
        
        updateIO();
        unsigned int return_length = processModbusMessage(modbus_buffer, i);
        client.write((const uint8_t *)modbus_buffer, return_length);
        updateIO();
        delay(1);
    }
    
    Serial.println("Client disonnected");
    modbus_connected = false;
    aborting();
    delay(1);
}
