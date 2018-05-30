;===============================================================================================
;					Controller Slave - Slave motor controller (Right)
;===============================================================================================

; GIT: MASTER

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
Battery_Power_Limit = 20                ; per 10W
;; Regen_Battery_Current_Limit

RESET_PASSWORD                  constant 141        ; password for "reset_controller_remote" to reset controller

DEBOUNCE_INPUTS					constant    5		; this sets the debounce time for switch inputs to 20ms, 4mS/unit

; CAN
CAN_EMERGENCY_DELAY_ACK         constant    50      ; Amount of time system waits until sending next emergency message
CAN_CYCLIC_RATE					constant    25	    ; this sets the cyclic cycle to every 100 ms, 4mS/unit

; Timers
MAX_TIME_GEARCHANGE			    constant    2000    ; Maximum time gear change may take [ms] (normally 70ms + delays)
STARTUP_DELAY                   constant    3000    ; Delay before system starts up [ms]
CAN_NOTHING_RECEIVE_SHUTDOWN_TIME constant  500     ; If slave does receive nothing for this amount of time, interlock turns off [ms]
SMESH_FININSHED_DELAY           constant    10      ;

; DNR, Throttle and Brake
FULL_BRAKE                      constant    32767   ; On a scale of 0-32767, how hard controller will brake

; STATES IN STATEMACHINE
NEUTRAL                         constant    0
DRIVE16                         constant    2
DRIVE118                        constant    3
REVERSE                         constant    4

; Current settings
INITIAL_OUTPUT_DRIVE_CURRENT_LIMIT constant 32767       ; at 0 Battery current, output current limit, 350A /350A*32767
INITIAL_OUTPUT_REGEN_CURRENT_LIMIT constant 2000        ; at 0 Battery regen current, output current limit,  10A /350A*32767 = 937, minimal 1638
MIN_DRIVE_OUTPUT_CURRENT_PER_CTRLR constant 1640
MIN_REGEN_OUTPUT_CURRENT_PER_CTRLR constant 1640

MARGIN_DRIVE_INPUT_CURRENT_LIMIT   constant 700       ; 10A /350A*32767, higher than this input current, the output current will be cutback
MARGIN_REGEN_INPUT_CURRENT_LIMIT   constant 400        ; 35A / 350A*32767, higher than this input current, the ouput current will be cutback
MAX_DRIVE_INPUT_CURRENT_PER_CTRLR  constant 940       ; 150A /350A*32767
MAX_REGEN_INPUT_CURRENT_PER_CTRLR  constant 500        ; 30A /350A*32767





;****************************************************************
;							VARIABLES
;****************************************************************

; Accesible from programmer handheld (max. 100 user, 10 bit)

Key_Switch_Hard_Reset_Complete		alias P_User1		;Can be saved to non-volatile memory

;Auto user variables (max. 300 user, 16 bit)

create reset_controller_remote variable

;-------------- CAN ---------------

create RCV_ACK_Fault_System variable
create RCV_ACK_System_Action variable

create RM_System_Init_Complete variable

;-------------- Temporaries ---------------
create  temp_Map_Output_1   variable
create  temp_Calculation    variable
create  temp_VCL_Throttle   variable
create  temp_Drive_Current_Limit variable
create  temp_Regen_Current_Limit variable

create test variable


;Standard user variables (max. 120 user, 16 bit)

; States
RCV_State_GearChange                    alias       user10          ; Smesh gear change state received from master
State_GearChange                        alias       user11          ; Own state Smesh gear change

; Efficiency
RM_Efficiency                           alias       user20          ; Slave Efficiency
Power_In                                alias       user21          ; Input power (P=U*I)
Power_Out                               alias       user22          ; Output power (P=w*T)
Motor_Rads                              alias       user23          ; Speed of axes [rad/s]

; Temperature and current protection
Motor_Temperature_Display               alias       user30          ; Temperature of Motor 0-255 C
Controller_Temperature_Display          alias       user31          ; Temperature of Controller 0-255 C
RCV_Drive_Current_Limit                 alias       user32          ; Current limits received from master
RCV_Regen_Current_Limit                 alias       user33          ; Current limits received from master

; DNR, Throttle and Brake
RCV_DNR_Command                         alias       user40          ; Received DNR state from master
RCV_Throttle_Compensated                alias       user41          ; Received throttle command from master
Interlock_RCV                           alias       user42          ; Key Switch state, received from master
Brake_RCV                               alias       user43          ; Received brake signal from master





;------------- CAN MAILBOXES --------------
MAILBOX_SM_MISO1						alias CAN1
MAILBOX_SM_MOSI1						alias CAN2
MAILBOX_SM_MOSI1_received               alias CAN2_received
MAILBOX_SM_MOSI2						alias CAN3
MAILBOX_SM_MOSI2_received               alias CAN3_received
MAILBOX_SM_MOSI3						alias CAN4
MAILBOX_SM_MOSI3_received               alias CAN4_received
MAILBOX_SM_MISO2						alias CAN5

MAILBOX_ERROR_MESSAGES                  alias CAN19
MAILBOX_ERROR_MESSAGES_RCV_ACK          alias CAN20
MAILBOX_ERROR_MESSAGES_RCV_ACK_received alias CAN20_received

MAILBOX_SM_MISO3_Init                   alias CAN21
MAILBOX_SM_MOSI4_Init                   alias CAN22
MAILBOX_SM_MOSI4_Init_received          alias CAN22_received
MAILBOX_RESET_CONTROLLER                alias CAN23
MAILBOX_RESET_CONTROLLER_received       alias CAN23_received


;----------- User Defined Faults ------------

Fault_System                 alias      UserFault1
    Regen_Crit_Error                  bit        Fault_System.1             ; (1, Code 51) Critical Error occured with Regen
    Drive_Crit_Error                  bit        Fault_System.2             ; (2, Code 52) Critical Error occured with Driving
    Temp_Crit_Error                   bit        Fault_System.4             ; (3, Code 53) Critical Error occured related to temperature
    Regen_Fault                       bit        Fault_System.8             ; (4, Code 54) Some fault occured with Regen
    Drive_Fault                       bit        Fault_System.16            ; (5, Code 55) Some fault occured with Driving
    Temp_Fault                        bit        Fault_System.32            ; (6, Code 56) Some fault occured related to temperature
    General_Fault                     bit        Fault_System.64            ; (7, Code 57) Some fault occured generally
    General_Crit_Error                bit        Fault_System.128           ; (8, Code 58) Critical Error occured generally
    
User_Fault_Action_01 = 0000000000011011b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
User_Fault_Action_02 = 0000010000011011b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off, Full brake
User_Fault_Action_03 = 0000000000011011b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
User_Fault_Action_04 = 0000000000000000b
User_Fault_Action_05 = 0000000000000000b
User_Fault_Action_06 = 0000000000000000b
User_Fault_Action_07 = 0000000000000000b
User_Fault_Action_08 = 0000000000011011b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off


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
Fan1IN				alias PWM2					;Pin J1-05
Fan2OUT				alias PWM3					;Pin J1-04
;FREE				alias PWM4					;Pin J1-03
;FREE				alias PWM5					;Pin J1-02

;SmeshPin			alias DigOut6				;Pin J1-19
;SmeshPin_output	alias DigOut6_output
;Free   			alias DigOut7				;Pin J1-20
;Free_output        alias DigOut7_output

;--------- Declaration Section ----------

; none

;--------------- DELAYS -----------------
Startup_DLY                       alias     DLY1             
Startup_DLY_output                alias     DLY1_output
Smesh_DLY                         alias     DLY2
Smesh_DLY_output                  alias     DLY2_output
EMERGENCY_ACK_DLY                 alias     DLY3
EMERGENCY_ACK_DLY_output          alias     DLY3_output
CAN_INIT_ACK_DLY                  alias     DLY4
CAN_INIT_ACK_DLY_output           alias     DLY4_output
CAN_RCV_MASTER_DLY                alias     DLY5
CAN_RCV_MASTER_DLY_output         alias     DLY5_output
General_DLY                       alias     DLY4
General_DLY_output                alias     DLY4_output





;****************************************************************
;					ONE TIME INITIALISATION
;****************************************************************

; RAM variables should be initialized to a known value before starting
; execution of VCL logic.  All other tasks that need to be performed at
; startup but not during main loop execution should be placed here.
; Signal chains should be set up here as well.


; Use the delay to make sure that all hardware resourses are ready to run
Setup_Delay(Startup_DLY, STARTUP_DELAY)
while (Startup_DLY_output <> 0) {}			; Wait 500ms before start

; setup inputs
setup_switches(DEBOUNCE_INPUTS)

;Initiate sytems
call startup_CAN_System 		;setup and start the CAN communications system





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
    
    Setup_Delay(DLY11, 10)
    
    if (reset_controller_remote = RESET_PASSWORD) {
        Reset_Controller()
    }
    
    call CheckCANMailboxes
    
    call DNR_statemachine
    
    call faultHandling
    
    
    while (DLY11_output <> 0) {}			; Wait 100ms before start
    
    
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

    Suppress_CANopen_Init = 0			;first undo suppress, then startup CAN, then disable CANopen
    Disable_CANOpen_PDO()				; disables OS PDO mapping and frees 

    setup_CAN(CAN_500KBAUD, 0, 0, -1, 0)		;(Baud, Sync, Reserved[0], Slave ID, Restart)
												;Baudrate = 500KB/s setting, no Sync, Not Used, Not Used, Auto Restart



   			; MAILBOX 1
   			; Purpose:		Send information: Torque, speed, Temperature motor/controller, Smesh enabled
   			; Type:			PDO1 MISO1
   			; Partner:		Master Motorcontroller

    Setup_Mailbox(MAILBOX_SM_MISO1, 0, 0, 0x111, C_CYCLIC, C_XMT, 0, 0)

    Setup_Mailbox_Data(MAILBOX_SM_MISO1, 6,			
        @System_Action,				        ; DC battery current , calculated not measured
        @System_Action + USEHB,
        @Motor_Temperature_Display,			    ; Motor temperature 0-255°C
        @Controller_Temperature_Display,        ; Controller temperature  0-255°C
        @State_GearChange,                      ; Gear change state
        @Fault_System,
        0,
        0)                          ; Fault system
		


   			; MAILBOX 2
   			; Purpose:		Send information: Torque, Speed, Smesh enabled and fault codes
   			; Type:			PDO2 MOSI1
   			; Partner:		Slave Motorcontroller
        
    Setup_Mailbox(MAILBOX_SM_MOSI1, 0, 0, 0x101, C_EVENT, C_RCV, 0, 0)
    
    Setup_Mailbox_Data(MAILBOX_SM_MOSI1, 8,
        @RCV_Throttle_Compensated,			    ; Torque for right motorcontroller
        @RCV_Throttle_Compensated + USEHB, 
        @RCV_State_GearChange,			        ; Command to change gear
        @RCV_Drive_Current_Limit,			    ; Set max speed
        @RCV_Drive_Current_Limit + USEHB,
        @RCV_Regen_Current_Limit,				; Set Regen limit
        @RCV_Regen_Current_Limit + USEHB, 
        @RCV_DNR_Command)


   			; MAILBOX 3
   			; Purpose:		Receive information: Torque, Smesh change gear, max speed, regen, commands
   			; Type:			PDO1 MOSI2
   			; Partner:		Slave Motorcontroller

    Setup_Mailbox(MAILBOX_SM_MOSI2, 0, 0, 0x102, C_EVENT, C_RCV, 0, 0)

    Setup_Mailbox_Data(MAILBOX_SM_MOSI2, 2,
        @Brake_RCV,
        @Interlock_RCV,
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
            
    Setup_Mailbox(MAILBOX_SM_MOSI3, 0, 0, 0x100, C_EVENT, C_RCV, 0, 0)

    Setup_Mailbox_Data(MAILBOX_SM_MOSI3, 5, 		
        @RCV_Throttle_Compensated,			
        @RCV_Throttle_Compensated + USEHB,
        @Drive_Current_Limit,
        @Drive_Current_Limit + USEHB,
        @RCV_State_GearChange,							
        0,
		0,
		0)
        
            
            ; MAILBOX 5
   			; Purpose:		Send information:
   			; Type:			PDO MISO3
   			; Partner:		Slave Motorcontroller
         
    Setup_Mailbox(MAILBOX_SM_MISO2, 0, 0, 0x110, C_EVENT, C_XMT, 0, 0)

    Setup_Mailbox_Data(MAILBOX_SM_MISO2, 1, 		
        @State_GearChange,			; Motor torque
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

    Setup_Mailbox(MAILBOX_ERROR_MESSAGES, 0, 0, 0x001, C_EVENT, C_XMT, 0, 0)
    Setup_Mailbox_Data(MAILBOX_ERROR_MESSAGES, 3, 		
        @Fault_System,
        @System_Action,				; DC battery current , calculated not measured
        @System_Action + USEHB, 
        0,
        0,
        0,
        0,
        0)
        
        
            ; MAILBOX 20
   			; Purpose:		receive information: ACK on Error messages to Master controller
   			; Type:			PDO6
   			; Partner:		Master controller

    Setup_Mailbox(MAILBOX_ERROR_MESSAGES_RCV_ACK, 0, 0, 0x002, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_ERROR_MESSAGES_RCV_ACK, 3, 		
        @RCV_ACK_Fault_System,
        @RCV_ACK_System_Action,				; DC battery current , calculated not measured
        @RCV_ACK_System_Action + USEHB, 
        0,
        0,
        0,
        0,
        0)
        
        
        
            ; MAILBOX 21
   			; Purpose:		Send information: Request for Init
   			; Type:			MISO3
   			; Partner:		Master controller
        
    Setup_Mailbox(MAILBOX_SM_MISO3_Init, 0, 0, 0x112, C_EVENT, C_XMT, 0, 0)
    Setup_Mailbox_Data(MAILBOX_SM_MISO3_Init, 1, 		
        @RM_System_Init_Complete,
        0,
        0, 
        0,
        0,
        0,
        0,
        0)
        
        
        
            ; MAILBOX 22
   			; Purpose:		Receive information: Init Slave controller
   			; Type:			MOSI4
   			; Partner:		Master controller

    Setup_Mailbox(MAILBOX_SM_MOSI4_Init, 0, 0, 0x103, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_SM_MOSI4_Init, 8, 		
        @Max_Speed_TrqM,
        @Max_Speed_TrqM + USEHB,				
        @Accel_Rate_TrqM, 
        @Accel_Rate_TrqM + USEHB,
        @Brake_Rate_TrqM,
        @Brake_Rate_TrqM + USEHB,
        @Neutral_Braking_TrqM,
        @Neutral_Braking_TrqM + USEHB)
        
            
            
            ; MAILBOX 23
   			; Purpose:		Receive information: reset command
   			; Type:			PDO6
   			; Partner:		Master controller

    Setup_Mailbox(MAILBOX_RESET_CONTROLLER, 0, 0, 0x003, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_RESET_CONTROLLER, 1, 		
        @reset_controller_remote,
        0,
        0,
        0,
        0,
        0,
        0,
        0)




    CAN_Set_Cyclic_Rate(CAN_CYCLIC_RATE)			; this sets the cyclic cycle to
    Startup_CAN()					; Start the event driven mailbox;
    Startup_CAN_Cyclic()			; Start the cyclic mailboxes

    return


CheckCANMailboxes:
    
    if (MAILBOX_SM_MOSI4_Init_received = ON) {
        
        RM_System_Init_Complete = 1
    }
    
    if ( (RM_System_Init_Complete = 0) & (CAN_INIT_ACK_DLY_output = 0) ) {
        send_mailbox(MAILBOX_SM_MISO3_Init)
        
        Setup_Delay(CAN_INIT_ACK_DLY, CAN_EMERGENCY_DELAY_ACK)
    }
    
    if ( (MAILBOX_SM_MOSI1_received = ON) | (MAILBOX_SM_MOSI2_received = ON) | (MAILBOX_SM_MOSI3_received = ON) | (MAILBOX_ERROR_MESSAGES_RCV_ACK_received = ON) | (MAILBOX_SM_MOSI4_Init_received = ON) ) {
        MAILBOX_SM_MOSI1_received = OFF
        MAILBOX_SM_MOSI2_received = OFF
        MAILBOX_SM_MOSI3_received = OFF
        MAILBOX_ERROR_MESSAGES_RCV_ACK_received = OFF
        MAILBOX_SM_MOSI4_Init_received = OFF
        
        setup_delay(CAN_RCV_MASTER_DLY, CAN_NOTHING_RECEIVE_SHUTDOWN_TIME)
    }
    
    if (CAN_RCV_MASTER_DLY_output = 0) {
        ; Too much time is elapsed from last CAN Message
        clear_Interlock()
    }
    
    
    

    return
    
    

    
    
faultHandling:

    call calculateTemperature


    ;0-12800
    ; Transform Battery_current to percentage of rated current
    ;if (Battery_Current >= 0) {
    ;    temp_Calculation = Map_Two_Points(Battery_Current, 0, 12800, 0, 32767)
    ;    
    ;    ; Reduce Current at higher battery current
    ;    temp_Drive_Current_Limit = Map_Two_Points(temp_Calculation, MARGIN_DRIVE_INPUT_CURRENT_LIMIT, MAX_DRIVE_INPUT_CURRENT_PER_CTRLR, INITIAL_OUTPUT_DRIVE_CURRENT_LIMIT, MIN_DRIVE_OUTPUT_CURRENT_PER_CTRLR)
    ;    temp_Regen_Current_Limit = INITIAL_OUTPUT_REGEN_CURRENT_LIMIT
    ;} else {
    ;    temp_Calculation = Map_Two_Points(-Battery_Current, 0, 12800, 0, 32767)
    ;    
    ;    ; Reduce Current at higher battery current
    ;    temp_Drive_Current_Limit = INITIAL_OUTPUT_DRIVE_CURRENT_LIMIT
    ;    temp_Regen_Current_Limit = Map_Two_Points(temp_Calculation, MARGIN_DRIVE_INPUT_CURRENT_LIMIT, MAX_REGEN_INPUT_CURRENT_PER_CTRLR, INITIAL_OUTPUT_REGEN_CURRENT_LIMIT, MIN_REGEN_OUTPUT_CURRENT_PER_CTRLR)
    ;}
    
    if (RCV_Drive_Current_Limit <> 0) {
        ; Limit is determined by master
        temp_Drive_Current_Limit = RCV_Drive_Current_Limit
    } else {
        temp_Drive_Current_Limit = INITIAL_OUTPUT_DRIVE_CURRENT_LIMIT
    }
    
    if (RCV_Regen_Current_Limit <> 0) {
        ; Limit is determined by master
        temp_Regen_Current_Limit = RCV_Regen_Current_Limit
    } else {
        temp_Regen_Current_Limit = INITIAL_OUTPUT_REGEN_CURRENT_LIMIT
    }
    
    ;if (Battery_Current >= 0) {
    ;    temp_Calculation = Map_Two_Points(Battery_Current, 0, 12800, 0, 32767)
    ;    
    ;    ; Reduce Current at higher battery current
    ;    if (RCV_Drive_Current_Limit = 0) {
    ;        ; When limit is not determined by master, calculate own limit
    ;        
    ;        ; Reduce Current at higher battery current
    ;        temp_Drive_Current_Limit = Map_Two_Points(temp_Calculation, MARGIN_DRIVE_INPUT_CURRENT_LIMIT, MAX_DRIVE_INPUT_CURRENT_PER_CTRLR, INITIAL_OUTPUT_DRIVE_CURRENT_LIMIT, MIN_DRIVE_OUTPUT_CURRENT_PER_CTRLR)
    ;        
    ;    } else {
    ;        
    ;    }
    ;    
    ;    if (RCV_Regen_Current_Limit = 0) {
    ;        ; When limit is not determined by master, calculate own limit
    ;        temp_Regen_Current_Limit = INITIAL_OUTPUT_REGEN_CURRENT_LIMIT
    ;    } else {
    ;        ; Limit is determined by master
    ;        temp_Regen_Current_Limit = RCV_Regen_Current_Limit
    ;    }
    ;    
    ;} else {
    ;    temp_Calculation = Map_Two_Points(-Battery_Current, 0, 12800, 0, 32767)
    ;    
    ;    ; Reduce Current at higher battery current
    ;    
    ;    if (RCV_Drive_Current_Limit = 0) {
    ;        ; When limit is not determined by master, calculate own limit
    ;        temp_Calculation = Map_Two_Points(-Battery_Current, 0, 12800, 0, 32767)
    ;        
    ;        ; Reduce Current at higher battery current
    ;        temp_Drive_Current_Limit = INITIAL_OUTPUT_DRIVE_CURRENT_LIMIT
    ;        
    ;    } else {
    ;        ; Limit is determined by master
    ;        temp_Drive_Current_Limit = RCV_Drive_Current_Limit
    ;    }
    ;    
    ;    if (RCV_Regen_Current_Limit = 0) {
    ;        ; When limit is not determined by master, calculate own limit
    ;        temp_Regen_Current_Limit = Map_Two_Points(temp_Calculation, MARGIN_DRIVE_INPUT_CURRENT_LIMIT, MAX_REGEN_INPUT_CURRENT_PER_CTRLR, INITIAL_OUTPUT_REGEN_CURRENT_LIMIT, MIN_REGEN_OUTPUT_CURRENT_PER_CTRLR)
    ;    } else {
    ;        ; Limit is determined by master
    ;        temp_Regen_Current_Limit = RCV_Regen_Current_Limit
    ;    }
    ;    
    ;}
    
    
    
    if (Regen_Fault = ON) {
        
        ; Limits are controlled by master controller
        
    }
    if (Drive_Fault = ON) {

        ; Limits are controlled by master controller
        
    }
    if (Temp_Fault = ON) {
        
        ; Limits are controlled by master controller
        
    }
    if (General_Fault = ON) {
        
        ; Limits are controlled by master controller
        
    }
    
    if ( (Regen_Fault = OFF) & (Drive_Fault = OFF) & (Temp_Fault = OFF) & (General_Fault = OFF)) {
        
        ; Limits are controlled by master controller
        
    }
    
    if ( (System_Action <> 0) | (Fault_System <> 0) ) {
        ; There is some fault, so send to Master controller
        
        if ( (EMERGENCY_ACK_DLY_output = 0) & (RCV_ACK_Fault_System <> Fault_System) & (RCV_ACK_System_Action <> System_Action) ) {
            send_mailbox(MAILBOX_ERROR_MESSAGES)
        
            Setup_Delay(EMERGENCY_ACK_DLY, CAN_EMERGENCY_DELAY_ACK)
        }

    } else {
        RCV_ACK_Fault_System = 0
        RCV_ACK_System_Action = 0
    }
    
    ; Set current limits to the correct values
    Drive_Current_Limit = temp_Drive_Current_Limit
    Regen_Current_Limit = temp_Regen_Current_Limit
    
    ; Set Regen limits correct
    Brake_Current_Limit = Regen_Current_Limit
    Interlock_Brake_Current_Limit = Regen_Current_Limit
    
    return

    
    
    
    
    
setup_2D_MAP:
    
    
    
    return


    
    
    
    
    
DNR_statemachine:

    ; STATE MACHINE
    
    if ((Interlock_RCV = 1)) {
        ; Turn car 'on'
        if ((Interlock_State = OFF)) {
            set_Interlock()
        }
        if (Key_Switch_Hard_Reset_Complete <> 0) {
            Key_Switch_Hard_Reset_Complete = 0
        }
        
    } else if ((Interlock_RCV = 0)) {
        ; turn car 'off'
        if ((Interlock_State = ON)) {
            clear_Interlock()
        }
        
        
        if ((System_Action <> 0) & (Key_Switch_Hard_Reset_Complete = 0)) {
            ; There is some fault, so reset controller at turning off car
            Key_Switch_Hard_Reset_Complete = 1
            Reset_Controller()
        }
    }
    
    if (Brake_RCV = 1) {
        VCL_Brake = FULL_BRAKE
        RCV_Throttle_Compensated = 0
    } else {
        VCL_Brake = 0
    }
    
    
    if ( (RCV_State_GearChange >= 0x60) & (RCV_State_GearChange <= 0x6D) ) {
        ;; Changing gear to 1:6
        call setSmeshTo16
        
        exit
        
    } else if ( (RCV_State_GearChange >= 0x80) & (RCV_State_GearChange <= 0x8D) ) {
        ;; Changing gear to 1:18
        call setSmeshTo118
        
        exit

    } else if ( (State_GearChange = 0xFF) | (RCV_State_GearChange = 0xFF) ) {
        State_GearChange = 0xFF
        temp_VCL_Throttle = 0
        send_mailbox(MAILBOX_SM_MISO2)

    } else if ( (RCV_DNR_Command = DRIVE118) | (RCV_DNR_Command = DRIVE16) ) {
        
        State_GearChange = 0x01
        
        temp_VCL_Throttle = RCV_Throttle_Compensated         ; Set throttle to position of pedal
        
        
    } else if (RCV_DNR_Command = REVERSE) {
        
        State_GearChange = 0x01
        
        temp_VCL_Throttle = RCV_Throttle_Compensated    ; Set throttle to position of pedal
        
        
    } else {
        ; When in neutral or undefined state, set throttle to zero
        temp_VCL_Throttle = 0
        
        State_GearChange = 0x01
    }
    
    VCL_Throttle = temp_VCL_Throttle

    
    return
    
    
    
    
    
calculateTemperature:
    
    Motor_Temperature_Display = Map_Two_Points(Motor_Temperature, 0, 2550, 0, 255)
    Controller_Temperature_Display = Map_Two_Points(Controller_Temperature, 0, 2550, 0, 255)
    
    return
    
    
    
    
    
setSmeshTo16:
    
    ;;;;; 1. Reduce Left throttle
    
    if ( (RCV_State_GearChange = 0x61) & (State_GearChange <> 0x62) ) {
        setup_delay(Smesh_DLY, MAX_TIME_GEARCHANGE)
        State_GearChange = 0x62
        
        send_mailbox(MAILBOX_SM_MISO2)
    }
    
    
    ;;;;; 2. Receive ACK: Procedure has started
    ;;;;; 3. multiply throttle of Right controller with 2
    
    if ( (RCV_State_GearChange = 0x63) & (State_GearChange <> 0x64) ) {
        VCL_Throttle = RCV_Throttle_Compensated
        State_GearChange = 0x64
        
        send_mailbox(MAILBOX_SM_MISO2)
    }

    
    ;;;;; 4. receive ACK from controller: throttle is increased
    ;;;;; 5. Switch left gear to 1:18
	
    
    ;;;;; 6. Reduce throttle right controller
    
    if ( (RCV_State_GearChange = 0x66) & (State_GearChange <> 0x67) ) {
        VCL_Throttle = RCV_Throttle_Compensated
        State_GearChange = 0x67
        
        send_mailbox(MAILBOX_SM_MISO2)
    }
    
    
    ;;;;; 7. receive ACK from controller: throttle is reduced
    ;;;;; 8. Increase left throttle
    ;;;;; 10. receive ACK from controller: procedure has started
    ;;;;; 11. reduce left throttle to normal

    
    ;;;;; 12. increase throttle right controller to normal
    
    if ( (RCV_State_GearChange = 0x6C) & (State_GearChange <> 0x6D) ) {
        VCL_Throttle = RCV_Throttle_Compensated
        
        
    ;;;;; 13. Change is successful, thus speed_to_RPM can be changed
        
        Speed_to_RPM = 601          ; (G/d)*5305 ... 18/530*5305 ... One decimal
        
        ; Gear state to complete
        State_GearChange = 0x6D
        
        send_mailbox(MAILBOX_SM_MISO2)
        
        Setup_Delay(General_DLY, SMESH_FININSHED_DELAY)
        while (General_DLY_output <> 0) {}
    }
    
    
    ;;;;; Change is not successful
    
    if ( (Smesh_DLY_output = 0) | (RCV_State_GearChange = 0xFF) ) {
        ; Send signal that mission is failed
        RCV_State_GearChange = 0;
        State_GearChange = 0xFF
        
        send_mailbox(MAILBOX_SM_MISO2)
        
        Setup_Delay(General_DLY, SMESH_FININSHED_DELAY)
        while (General_DLY_output <> 0) {}
    }
    
    
	return

    
setSmeshTo118:

    ;;;;; 1. Reduce Left throttle
    
    if ( (RCV_State_GearChange = 0x81) & (State_GearChange <> 0x82) ) {
        setup_delay(Smesh_DLY, MAX_TIME_GEARCHANGE)
        State_GearChange = 0x82
        
        send_mailbox(MAILBOX_SM_MISO2)
    }
    
    
    ;;;;; 2. Receive ACK: Procedure has started
    ;;;;; 3. multiply throttle of Right controller with 2
    
    if ( (RCV_State_GearChange = 0x83) & (State_GearChange <> 0x84) ) {
        VCL_Throttle = RCV_Throttle_Compensated
        State_GearChange = 0x84
        
        send_mailbox(MAILBOX_SM_MISO2)
    }

    
    ;;;;; 4. receive ACK from controller: throttle is increased
    ;;;;; 5. Switch left gear to 1:18
	
    
    ;;;;; 6. Reduce throttle right controller
    
    if ( (RCV_State_GearChange = 0x86) & (State_GearChange <> 0x87) ) {
        VCL_Throttle = RCV_Throttle_Compensated
        State_GearChange = 0x87
        
        send_mailbox(MAILBOX_SM_MISO2)
    }
    
    
    ;;;;; 7. receive ACK from controller: throttle is reduced
    ;;;;; 8. Increase left throttle
    ;;;;; 10. receive ACK from controller: procedure has started
    ;;;;; 11. reduce left throttle to normal

    
    ;;;;; 12. increase throttle right controller to normal
    
    if ( (RCV_State_GearChange = 0x8C) & (State_GearChange <> 0x8D) ) {
        VCL_Throttle = RCV_Throttle_Compensated
        
        
    ;;;;; 13. Change is successful, thus speed_to_RPM can be changed
        
        Speed_to_RPM = 1802          ; (G/d)*5305 ... 18/530*5305 ... One decimal
        
        ; Gear state to complete
        State_GearChange = 0x8D
        
        send_mailbox(MAILBOX_SM_MISO2)
        
        Setup_Delay(General_DLY, SMESH_FININSHED_DELAY)
        while (General_DLY_output <> 0) {}
    }
    
    
    ;;;;; Change is not successful
    
    if ( (Smesh_DLY_output = 0) | (RCV_State_GearChange = 0xFF) ) {
        ; Send signal that mission is failed
        RCV_State_GearChange = 0;
        State_GearChange = 0xFF
        
        send_mailbox(MAILBOX_SM_MISO2)
        
        Setup_Delay(General_DLY, SMESH_FININSHED_DELAY)
        while (General_DLY_output <> 0) {}
    }
	
	
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