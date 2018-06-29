;===============================================================================================
;					Controller Slave - Slave motor controller (Right)
;===============================================================================================

; GIT: Master

; Author:	Tim Stek
; Organization:	TU/ecomotive
VCL_App_Ver = 011
; Date:		09-03-2018


; Description
;-------------------
;  This program controls the right wheel, right motor controller, right fans and the right
;	Smesh gear. It calculates when to apply the Smesh gear and when to turn on fans.


; I/O Requirements
;-------------------
;  Inputs:
;	- Position feedback from motor
;	- CAN

;  Outputs:
;	- Smesh gear Digital
;	- Torque for right motor

;  CAN input:
;	- DNR
;	- Throttle and Brake (intensity)
;	- Battery errors
;	- RM: Steering angle
;	- RM: Speed right wheel
;	- RM: Temperature right motor and controller

;  CAN output:
;	- Speed of the wheels
;	- Temperature of controller and battery
;	- RM: Torque, with steering compensation
;	- RM: Smesh enable

; TO DO:






;****************************************************************
;							CONSTANTS
;****************************************************************

; Input Current Limits
Battery_Current_Limit_Ramp_Rate = 1
Battery_Current_Limiter_enable = 1

RESET_PASSWORD                  constant 141        ; password for "resetControllerRemote" to reset controller

DEBOUNCE_INPUTS					constant    5		; this sets the debounce time for switch inputs to 20ms, 4mS/unit

; CAN
CAN_EMERGENCY_DELAY_ACK         constant    50      ; Amount of time system waits until sending next emergency message
CAN_CYCLIC_RATE					constant    25	    ; this sets the cyclic cycle to every 100 ms, 4mS/unit
CAN_DELAY_FOR_ACK               constant    100

MULTIPLIER_CAN_NOTHING_RECEIVE  constant    2       ; Cyclic Rate times 2. If nothing is received from slave for this amount of time, create error
CAN_NOTHING_RECEIVE_INIT        constant    1000

; Timers
MAX_TIME_GEARCHANGE			    constant    1500    ; Maximum time gear change may take [ms]
STARTUP_DELAY                   constant    2000    ; Delay before system starts up [ms]
SMESH_FININSHED_DELAY           constant    100      ;

; DNR, Throttle and Brake
FULL_BRAKE                      constant    32767   ; On a scale of 0-32767, how hard controller will brake

; STATES IN STATEMACHINE
NEUTRAL                         constant    0
DRIVE16                         constant    2
DRIVE118                        constant    3
REVERSE                         constant    4





;****************************************************************
;							VARIABLES
;****************************************************************

; Accesible from programmer handheld (max. 100 user, 10 bit)

;PAR_Total_Maint_interval		alias P_User1		;Can be saved to non-volatile memory

;Auto user variables (max. 300 user, 16 bit)

create resetControllerRemote variable

;-------------- CAN ---------------

create rcvFaultSystemACK variable
create rcvSystemActionACK variable

create requestParamStatus variable
create requestSlaveParam variable

create Slave_Param_Var1 variable
create Slave_Param_Var2 variable
create Slave_Param_Var3 variable
create Slave_Param_Var4 variable

;-------------- Temporaries ---------------
create  mapOutputTemp   variable
create  calculationTemp    variable
create  VCLThrottleTemp   variable

create test variable



;Standard user variables (max. 120 user, 16 bit)

; States
rcvStateGearChange                      alias       user10          ; Smesh gear change state received from master
stateGearChange                         alias       user11          ; Own state Smesh gear change

; Temperature and current protection
motorTemperatureDisplay                 alias       user30          ; Temperature of Motor 0-255 C
contrTemperatureDisplay                 alias       user31          ; Temperature of Controller 0-255 C
rcvBattDrivePowerLim                    alias       user32          ; Current limits received from master
rcvBattRegenPowerLim                    alias       user33          ; Current limits received from master
NEUTRAL_BRAKING_INIT                    alias       user35
CURRENT_THRESHOLD_POWER_LIMITING        alias       user36

; DNR, Throttle and Brake
rcvDNRCommand                           alias       user40          ; Received DNR state from master
rcvThrottleCompensated                  alias       user41          ; Received throttle command from master
rcvInterlock                            alias       user42          ; Key Switch state, received from master
rcvBrake                                alias       user43          ; Received brake signal from master





;------------- CAN MAILBOXES --------------
MAILBOX_SM_MISO1						alias CAN1
MAILBOX_SM_MISO1_addr                   constant 0x111

MAILBOX_SM_MOSI1						alias CAN2
MAILBOX_SM_MOSI1_received               alias CAN2_received
MAILBOX_SM_MOSI1_addr                   constant 0x101

MAILBOX_SM_MOSI2						alias CAN3
MAILBOX_SM_MOSI2_received               alias CAN3_received
MAILBOX_SM_MOSI2_addr                   constant 0x102

MAILBOX_SM_MOSI3						alias CAN4
MAILBOX_SM_MOSI3_received               alias CAN4_received
MAILBOX_SM_MOSI3_addr                   constant 0x100

MAILBOX_SM_MISO2						alias CAN5
MAILBOX_SM_MISO2_addr                   constant 0x110


MAILBOX_ERROR_MESSAGES                  alias CAN19
MAILBOX_ERROR_MESSAGES_addr             constant 0x001

MAILBOX_ERROR_MESSAGES_RCV_ACK          alias CAN20
MAILBOX_ERROR_MESSAGES_RCV_ACK_received alias CAN20_output
MAILBOX_ERROR_MESSAGES_RCV_ACK_addr     constant 0x002

MAILBOX_MISO_REQUEST_PARAM              alias CAN21
MAILBOX_MISO_REQUEST_PARAM_addr         constant 0x112

MAILBOX_MOSI_INIT_PARAM                alias CAN22
MAILBOX_MOSI_INIT_PARAM_received       alias CAN22_received
MAILBOX_MOSI_INIT_PARAM_addr           constant 0x103




;----------- User Defined Faults ------------

faultSystem                 alias      UserFault1
    regenCritError                  bit        faultSystem.1             ; (1, Code 51) Critical Error occured with Regen
    driveCritError                  bit        faultSystem.2             ; (2, Code 52) Critical Error occured with Driving
    temperatureCritError                   bit        faultSystem.4             ; (3, Code 53) Critical Error occured related to temperature
    regenFault                       bit        faultSystem.8             ; (4, Code 54) Some fault occured with Regen
    driveFault                       bit        faultSystem.16            ; (5, Code 55) Some fault occured with Driving
    temperatureFault                        bit        faultSystem.32            ; (6, Code 56) Some fault occured related to temperature
    generalFault                     bit        faultSystem.64            ; (7, Code 57) Some fault occured generally
    generalCritError                bit        faultSystem.128           ; (8, Code 58) Critical Error occured generally
    
user_fault_action_01 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
user_fault_action_02 = 1101100000100000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off, Full brake
user_fault_action_03 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
user_fault_action_04 = 0000000000000000b
user_fault_action_05 = 0000000000000000b
user_fault_action_06 = 0000000000000000b
user_fault_action_07 = 0000000000000000b
user_fault_action_08 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off


;--------------- INPUTS ----------------
;Free				alias Sw_1					;Pin J1-24
;Interlock_sw		alias Sw_3					;Pin J1-09
;Free				alias Sw_4					;Pin J1-10
;Free				alias Sw_5					;Pin J1-11
;Free				alias Sw_6					;Pin J1-12
;Forward_sw			alias Sw_7					;Pin J1-22
;Reverse_Sw			alias Sw_8					;Pin J1-33
;Free				alias Sw_14					;Pin J1-19
;Free				alias Sw_15					;Pin J1-20

;--------------- OUTPUTS ----------------
;Main				alias PWM1					;Pin J1-06
;FREE				alias PWM2					;Pin J1-05
;FREE				alias PWM3					;Pin J1-04
;FREE				alias PWM4					;Pin J1-03
;FREE				alias PWM5					;Pin J1-02

;FREE			    alias DigOut6				;Pin J1-19
;FREE_output        alias DigOut6_output
;Free   			alias DigOut7				;Pin J1-20
;Free_output        alias DigOut7_output

;--------- Declaration Section ----------

; none

;--------------- DELAYS -----------------
startupDLY                      alias		DLY1             
startupDLY_output               alias		DLY1_output
smeshFinishedDLY                alias     DLY3
smeshFinishedDLY_output         alias     DLY3_output
emergencyACKDLY                 alias     DLY4
emergencyACKDLY_output          alias     DLY4_output
CANACKDLY                       alias     DLY6
CANACKDLY_output                alias     DLY6_output
communicaitonMasterErrorDLY             alias     DLY7
communicaitonMasterErrorDLY_ouput       alias     DLY7_output






;****************************************************************
;					ONE TIME INITIALISATION
;****************************************************************

; RAM variables should be initialized to a known value before starting
; execution of VCL logic.  All other tasks that need to be performed at
; startup but not during main loop execution should be placed here.
; Signal chains should be set up here as well.


; Use the delay to make sure that all hardware resourses are ready to run
setup_delay(startupDLY, STARTUP_DELAY)
while (startupDLY_output <> 0) {}			; Wait 500ms before start

clear_interlock()

; setup inputs
setup_switches(DEBOUNCE_INPUTS)

;Initiate sytems
call startup_CAN_System 		;setup and start the CAN communications system

setup_delay(communicaitonMasterErrorDLY, CAN_NOTHING_RECEIVE_INIT)





;****************************************************************
;						MAIN PROGRAM LOOP
;****************************************************************

; The continuously running portion of the program should be placed here
; It is important to structure the main loop such that there is no
; possibility for the program to get stuck in a loop that will prevent
; important vehicle functions from occuring regularly.  Be particularly
; careful with while loops.  Use of signal chains and automated functions
; as described in the VCL documentation can greatly reduce the complexity
; of the main loop.


Mainloop:
    
    if (resetControllerRemote = RESET_PASSWORD) {
        Reset_Controller()
    }
    
    call checkCANMailboxes
    
    call DNR_statemachine
    
    call faultHandling
    
    
    goto Mainloop 

    





    
;****************************************************************
;							SUBROUTINES
;****************************************************************

; As with any programming language, the use of subroutines can allow
; easier re-use of code across multiple parts of a program, and across
; programs.  Function specific subroutines can also improve the
; Readability of the code in the main loop.


startup_CAN_System:

   			; CAN mailboxes 1-5 are available to VCL CAN. Mailboxes 6-14 are available for CANopen
   			; C_SYNC is buddy check, C_CYCLIC is cyclic CAN messages, C_EVENT are called with send_mailbox()
   			; Set Message Type and Master ID to 0, and put Slave ID to pre-defined 11bit identifier.
   			;

    suppress_CANopen_init = 0			;first undo suppress, then startup CAN, then disable CANopen
    disable_CANopen_pdo()				; disables OS PDO mapping and frees 

    setup_CAN(CAN_500KBAUD, 0, 0, -1, 0)		;(Baud, Sync, Reserved[0], Slave ID, Restart)
												;Baudrate = 500KB/s setting, no Sync, Not Used, Not Used, Auto Restart



   			; MAILBOX 1
   			; Purpose:		Send information: Torque, speed, Temperature motor/controller, Smesh enabled
   			; Type:			PDO1 MISO1
   			; Partner:		Master Motorcontroller

    setup_mailbox(MAILBOX_SM_MISO1, 0, 0, MAILBOX_SM_MISO1_addr, C_CYCLIC, C_XMT, 0, 0)

    setup_mailbox_data(MAILBOX_SM_MISO1, 6,			
        @system_action,				        ; DC battery current , calculated not measured
        @system_action + USEHB, 
        @motorTemperatureDisplay,			    ; Motor temperature 0-255°C
        @contrTemperatureDisplay,        ; Controller temperature  0-255°C
        @stateGearChange,                      ; Gear change state
        @faultSystem,
        0,
        0)                          ; Fault system
		


   			; MAILBOX 2
   			; Purpose:		Receive information: Torque, Speed, Smesh enabled and fault codes
   			; Type:			PDO2 MOSI1
   			; Partner:		Slave Motorcontroller
        
    setup_mailbox(MAILBOX_SM_MOSI1, 0, 0, MAILBOX_SM_MOSI1_addr, C_EVENT, C_RCV, 0, 0)
    
    setup_mailbox_data(MAILBOX_SM_MOSI1, 6,
        @rcvThrottleCompensated,			    ; Torque for right motorcontroller
        @rcvThrottleCompensated + USEHB, 
        @rcvStateGearChange,			        ; Command to change gear
        @rcvBattDrivePowerLim,			    ; Set max speed
        @rcvBattDrivePowerLim + USEHB, 
        @rcvDNRCommand,
        0,
        0)


   			; MAILBOX 3
   			; Purpose:		Receive information: Torque, Smesh change gear, max speed, regen, commands
   			; Type:			PDO1 MOSI2
   			; Partner:		Slave Motorcontroller

    setup_mailbox(MAILBOX_SM_MOSI2, 0, 0, MAILBOX_SM_MOSI2_addr, C_EVENT, C_RCV, 0, 0)

    setup_mailbox_data(MAILBOX_SM_MOSI2, 2,
        @rcvBrake,
        @rcvInterlock,
        0,
        0,
        0,
        0,
        0,
        0)


            ; MAILBOX 4
   			; Purpose:		Receive information: While gear change, valuable information: 
   			; Type:			PDO MOSI3
   			; Partner:		Slave Motorcontroller
            
    setup_mailbox(MAILBOX_SM_MOSI3, 0, 0, MAILBOX_SM_MOSI3_addr, C_EVENT, C_RCV, 0, 0)

    setup_mailbox_data(MAILBOX_SM_MOSI3, 6, 		
        @rcvThrottleCompensated,			
        @rcvThrottleCompensated + USEHB,
        @rcvStateGearChange,							
        @resetControllerRemote,
		@rcvBattDrivePowerLim,
        @rcvBattDrivePowerLim + USEHB,
        0,
        0)
        
            
            ; MAILBOX 5
   			; Purpose:		Send information:
   			; Type:			PDO MISO3
   			; Partner:		Slave Motorcontroller
         
    setup_mailbox(MAILBOX_SM_MISO2, 0, 0, MAILBOX_SM_MISO2_addr, C_EVENT, C_XMT, 0, 0)

    setup_mailbox_data(MAILBOX_SM_MISO2, 1, 		
        @stateGearChange,			; Motor torque
        0, 
        0,				
        0, 
        0,				
        0,
		0,
		0)
        
        
            ; MAILBOX 19
   			; Purpose:		send information: Error messages to Master controller
   			; Type:			PDO6
   			; Partner:		Master controller

    setup_mailbox(MAILBOX_ERROR_MESSAGES, 0, 0, MAILBOX_ERROR_MESSAGES_addr, C_EVENT, C_XMT, 0, 0)
    setup_mailbox_data(MAILBOX_ERROR_MESSAGES, 3, 		
        @faultSystem,
        @system_action,				; DC battery current , calculated not measured
        @system_action + USEHB, 
        0,
        0,
        0,
        0,
        0)
        
        
            ; MAILBOX 20
   			; Purpose:		receive information: ACK on Error messages to Master controller
   			; Type:			PDO6
   			; Partner:		Master controller

    setup_mailbox(MAILBOX_ERROR_MESSAGES_RCV_ACK, 0, 0, MAILBOX_ERROR_MESSAGES_RCV_ACK_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_ERROR_MESSAGES_RCV_ACK, 3, 		
        @rcvFaultSystemACK,
        @rcvSystemActionACK,				; DC battery current , calculated not measured
        @rcvSystemActionACK + USEHB, 
        0,
        0,
        0,
        0,
        0)
        
        
            ; MAILBOX 21
   			; Purpose:		send information: request for init parameters
   			; Type:			MISO
   			; Partner:		Master controller

    setup_mailbox(MAILBOX_MISO_REQUEST_PARAM, 0, 0, MAILBOX_MISO_REQUEST_PARAM_addr, C_EVENT, C_XMT, 0, 0)
    setup_mailbox_data(MAILBOX_MISO_REQUEST_PARAM, 1, 		
        @requestSlaveParam,
        0,
        0,
        0,
        0,
        0,
        0,
        0)
        
            ; MAILBOX 22
   			; Purpose:		receive information: Parameters at Init
   			; Type:			MOSI
   			; Partner:		Master controller

    setup_mailbox(MAILBOX_MOSI_INIT_PARAM, 0, 0, MAILBOX_MOSI_INIT_PARAM_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_MOSI_INIT_PARAM, 8, 		
        @Slave_Param_Var1,
        @Slave_Param_Var1 + USEHB,				
        @Slave_Param_Var2, 
        @Slave_Param_Var2 + USEHB,
        @Slave_Param_Var3,
        @Slave_Param_Var3 + USEHB,
        @Slave_Param_Var4,
        @Slave_Param_Var4 + USEHB)
        




    CAN_set_cyclic_rate(CAN_CYCLIC_RATE)			; this sets the cyclic cycle to
    startup_CAN()					; Start the event driven mailbox;
    startup_CAN_cyclic()			; Start the cyclic mailboxes

    return


checkCANMailboxes:

    call calculateTemperature
    
    ; Receive the right parameters from master
    
    if ( (MAILBOX_MOSI_INIT_PARAM_received = ON) & (requestParamStatus = 0)) {
        MAILBOX_MOSI_INIT_PARAM_received = OFF
        
        Max_Speed_TrqM          = Slave_Param_Var1
        Accel_Rate_TrqM         = Slave_Param_Var2
        Accel_Release_Rate_TrqM = Slave_Param_Var3
        Brake_Rate_TrqM         = Slave_Param_Var4
        
        requestParamStatus = 1
        
    } else if ( (MAILBOX_MOSI_INIT_PARAM_received = ON) & (requestParamStatus = 1)) {
        MAILBOX_MOSI_INIT_PARAM_received = OFF
        
        Brake_Release_Rate_TrqM = Slave_Param_Var1
        NEUTRAL_BRAKING_INIT    = Slave_Param_Var2
        ;FREE                   = Slave_Param_Var3
        ;FREE                   = Slave_Param_Var4
        
        Neutral_Braking_TrqM = NEUTRAL_BRAKING_INIT
        
        requestParamStatus = 2
        
    }
    
    ; Request parameters from master
    
    if ( (requestParamStatus = 0) & (CANACKDLY_output = 0) ) {
        
        requestSlaveParam = 1
        
        send_mailbox(MAILBOX_MISO_REQUEST_PARAM)
        setup_delay(CANACKDLY, CAN_DELAY_FOR_ACK)
        
    } else if ( (requestParamStatus = 1) & (CANACKDLY_output = 0) ) {
        requestSlaveParam = 2
        
        send_mailbox(MAILBOX_MISO_REQUEST_PARAM)
        setup_delay(CANACKDLY, CAN_DELAY_FOR_ACK)
    }
    
    
    ; When nothing received from Master, create error
    
    if (MAILBOX_SM_MOSI1_Received = ON) {
        MAILBOX_SM_MOSI1_Received = OFF
        
        calculationTemp = CAN_CYCLIC_RATE * 4 * MULTIPLIER_CAN_NOTHING_RECEIVE
        setup_delay(communicaitonMasterErrorDLY, calculationTemp)
        
    }

    return
    
    

    
    
faultHandling:
    
    if (regenFault = ON) {
        
        ; Limits are controlled by master controller
        
    }
    if (driveFault = ON) {

        ; Limits are controlled by master controller
        
    }
    if (temperatureFault = ON) {
        
        ; Limits are controlled by master controller
        
    }
    if (generalFault = ON) {
        
        ; Limits are controlled by master controller
        
    }
    
    if ( (regenFault = OFF) & (driveFault = OFF) & (temperatureFault = OFF) & (generalFault = OFF)) {
        
        ; Limits are controlled by master controller
        
    }
    
    
    ; Silence from Master over CAN, so create error
    if ( (communicaitonMasterErrorDLY_ouput <> 0)) {
        generalCritError = OFF
    } else {
        generalCritError = ON
    }
    
    
    if ( (system_action <> 0) | (faultSystem <> 0) ) {
        ; There is some fault, so send to Master controller
        
        if ( (emergencyACKDLY_output = 0) & (rcvFaultSystemACK <> faultSystem) & (rcvSystemActionACK <> system_action) ) {
            send_mailbox(MAILBOX_ERROR_MESSAGES)
        
            setup_delay(emergencyACKDLY, CAN_EMERGENCY_DELAY_ACK)
        }

    } else {
        rcvFaultSystemACK = 0
        rcvSystemActionACK = 0
    }
    
    ; Control Power Limiting
    Battery_Power_Limit = rcvBattDrivePowerLim
    
    return

    
    
    
    
    
setup_2D_MAP:
    
    
    
    return


    
    
    
    
    
DNR_statemachine:

    ; STATE MACHINE
    
    if ((rcvInterlock = 1) & (Interlock_State = OFF)) {
        ; Turn car 'on'
        set_interlock()
    } else if ((rcvInterlock = 0) & (Interlock_State = ON)) {
        clear_interlock()
        if (system_action <> 0) {
            ; There is some fault, so reset controller at turning off car
            reset_controller()
        }
    }
    
    
    if (rcvBrake = 1) {
        vcl_brake = FULL_BRAKE
        ;rcvThrottleCompensated = 0
    } else {
        vcl_brake = 0
    }
    
    
    
    if ( (rcvStateGearChange >= 0x60) & (rcvStateGearChange < 0x6D) ) {
        ; Changing gear to 1:6
        
        call setSmeshTo16Simple
        
        
    } else if ( (rcvStateGearChange >= 0x80) & (rcvStateGearChange < 0x8D) ) {
        ; Changing gear to 1:18
        
        call setSmeshTo118Simple
        
    } else if ( (rcvDNRCommand = DRIVE118) | (rcvDNRCommand = DRIVE16) ) {
        
        if (rcvStateGearChange < 0x60) {
            stateGearChange = 0x01
        }
        
        if (Neutral_Braking_TrqM <> NEUTRAL_BRAKING_INIT) {
            Neutral_Braking_TrqM = NEUTRAL_BRAKING_INIT
        }
        
        VCLThrottleTemp = rcvThrottleCompensated         ; Set throttle to position of pedal
        
        
    } else if (rcvDNRCommand = REVERSE) {
        
        if (rcvStateGearChange < 0x60) {
            stateGearChange = 0x01
        }
        
        if (Neutral_Braking_TrqM <> NEUTRAL_BRAKING_INIT) {
            Neutral_Braking_TrqM = NEUTRAL_BRAKING_INIT
        }
        
        VCLThrottleTemp = rcvThrottleCompensated    ; Set throttle to position of pedal
        
        
    } else {
        ; When in neutral or undefined state, set throttle to zero
        
        if (Neutral_Braking_TrqM <> NEUTRAL_BRAKING_INIT) {
            Neutral_Braking_TrqM = NEUTRAL_BRAKING_INIT
        }
        
        VCLThrottleTemp = 0
        
        stateGearChange = 0x01
    }
    
    vcl_throttle = VCLThrottleTemp

    
    return
    
    
    
    
    
calculateTemperature:
    
    motorTemperatureDisplay = map_two_points(motor_temperature, -500, 2050, 0, 255)
    contrTemperatureDisplay = map_two_points(controller_temperature, -500, 2050, 0, 255)
    
    return
    
    
    
    
setSmeshTo16Simple:
    
    
    ;;;;; 6. Send ACK: Throttle has been reduced
    
    if ( (rcvStateGearChange = 0x65) & (stateGearChange <> 0x66) ) {
        
        Neutral_Braking_TrqM = 0
        
        stateGearChange = 0x66
        
        send_mailbox(MAILBOX_SM_MISO2)
        
    }
    
    
    ;;;;; 12. Send ACK: procedure is finished
    
    if ( (rcvStateGearChange = 0x6B) & (stateGearChange <> 0x6D) ) {
        
        stateGearChange = 0x6C
        
        send_mailbox(MAILBOX_SM_MISO2)
        
        speed_to_rpm = 601          ; (G/d)*5305 ... 6/530*5305 ... One decimal
        Neutral_Braking_TrqM = NEUTRAL_BRAKING_INIT
        
        ; Gear state to complete
        
        stateGearChange = 0x6D
        
        
    }
    
    VCLThrottleTemp = rcvThrottleCompensated    ; Set throttle to position of pedal
    
    return





setSmeshTo118Simple:

    
    ;;;;; 6. Send ACK: Throttle has been reduced

    if ( (rcvStateGearChange = 0x85) & (stateGearChange <> 0x86) ) {
        
        Neutral_Braking_TrqM = 0
        
        stateGearChange = 0x86
        
        send_mailbox(MAILBOX_SM_MISO2)
        
    }
    
    
    ;;;;; 12. Send ACK: procedure is finished
    
    if ( (rcvStateGearChange = 0x8B) & (stateGearChange <> 0x8D) ) {
        
        stateGearChange = 0x8C
        
        send_mailbox(MAILBOX_SM_MISO2)
        
        speed_to_rpm = 1802          ; (G/d)*5305 ... 18/530*5305 ... One decimal
        Neutral_Braking_TrqM = NEUTRAL_BRAKING_INIT
        
        ; Gear state to complete
        
        stateGearChange = 0x8D
        
        
    }
    
    VCLThrottleTemp = rcvThrottleCompensated    ; Set throttle to position of pedal
    
    
    return
    
    
    





;****************************************************************
;				1311/1314 VARIABLES DECLARATIONS
;****************************************************************

;           1311/1314 Parameter, Monitor, and Fault Declarations
; These are generally placed at the end of the program, because they can
; be large, and hinder the general readability of the code when placed
; elsewhere.  Please note that Aliases and other declared variables
; cannot be used as addresses in parameter declarations, Only native
; OS variable names may be used.


;PARAMETERS:
;	PARAMETER_ENTRY	"CUSTOMER NAME"
;		TYPE		PROGRAM
;		LEVEL		1
;		END
;

;	PARAMETER_ENTRY	"Engage 2 Node ID"
;		TYPE		PROGRAM
;		ADDRESS		P_User1
;		WIDTH		16BIT
;		MAXRAW		127
;		MAXDSP		127
;		MINDSP		1
;		MINRAW		1
;		DEFAULT		123
;		LAL_READ	5
;		LAL_WRITE	5
;		DECIMALPOS	0
;	END
