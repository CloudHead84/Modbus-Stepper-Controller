FUNCTION_BLOCK MC_VirtualAxis
VAR_INPUT
	I_xEnable	: BOOL;
END_VAR
VAR_OUTPUT
END_VAR
VAR
	ton_PowerOn	: TON;
	instVirtualSpeed	: FB_RampGenerator;
END_VAR
VAR_IN_OUT
	Axis : Axis_Ref;
END_VAR
VAR CONSTANT 
	tDeadTime : TIME := T#100MS;
END_VAR

//Power
ton_PowerOn(IN:= Axis.OUT_xPowerOn,
			PT:= tDeadTime);
			
Axis.IN_xPowered := ton_PowerOn.Q AND I_xEnable;

	instVirtualSpeed(	I_xEnable	:= I_xEnable,
						I_rValue 	:= TO_REAL(Axis.OUT_iTargetSpeed));
						
	Axis.IN_iActualSpeed := TO_INT(instVirtualSpeed.Q_rActualValue);
