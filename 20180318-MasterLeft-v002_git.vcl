;===============================================================================================
;					Controller Master - Master motor controller (Left)
;===============================================================================================

; GIT: Smesh

; Author:	Tim Stek
; Organization:	TU/ecomotive
VCL_App_Ver = 012
; Date:		09-03-2018


; Description
;-------------------
;  This program controls the left wheel, right motor controller, left fans and the left
;	Smesh gear. It calculates when to apply the Smesh gear and when to turn on fans.


; I/O Requirements
;-------------------
;  Inputs:
;	- Throttle (0-255) and brake pot (6 / 78) (or via CAN)
;	- Left motor temp sensor
;	- DNR buttons (or via CAN)
;	- Key lock
;	- Position feedback from motor
;	- CAN

;  Outputs:
;	- Left fans PWM
;	- Smesh gear Digital (16 OFF(clear); 118 ON(set))
;	- Torque for left motor

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



;  QUESTIONS CURTIS:
;;;;   Battery_Current, how many decimals (0-12800)? 
;;;;   Brake command does not override throttle command ; When error is encountered again, send tact file
;;;;   Full throttle current limit


; TO DO:
; Get Batteries working with controller. Then get more info from 1 battery
; transmit parameters to slave

; TEST:
; Errors cleared by key switch cycle 
; current cutbacks
; Smesh gear change cycle
; ACK from master when slave sends error
; High pedal drive
; Fans with checking output duty cycle
; Check efficiency calculation


; REMINDER:
; With 'Neutral Braking' regen with 0 throttle can be reduced

; disable regen:
; set neutral braking to 0%
; don't brake with VCL, or disable brake pedal and VCL brake



;****************************************************************
;							CONSTANTS
;****************************************************************

; Input Current Limits
Battery_Current_Limit_Ramp_Rate = 1
Battery_Current_Limiter_enable = 1

;; Regen_Battery_Current_Limit
;; Motor_Power

RESET_PASSWORD                  constant    141     ; password for "reset_controller_remote" to reset controller

STARTUP_DELAY					constant    3000    ; time before startup system [ms]

DEBOUNCE_INPUTS					constant    5		; this sets the debounce time for switch inputs to 20ms, 4mS/unit

; CAN Settings
CAN_DELAY_FOR_ACK               constant    500     ; in ms
CAN_EMERGENCY_DELAY_ACK         constant    100
CAN_LOWPRIORITY_RATE            constant    800     ; sets the cyclic rate of low priority mailboxes
CAN_CYCLIC_RATE					constant    25		; this sets the cyclic cycle to every 100 ms, 4mS/unit

; Fan Settings
FAN_TEMPERATURE_HYSTER          constant    5       ; Temperature should first drop this amount under threshold to turn off fans
MOTOR_COOLDOWN_THOLD            constant    60      ; from this temperature in C Fans will turn on
CONTROLLER_COOLDOWN_THOLD       constant    50
MOTOR_TEMP_FAN_MAX              constant    85      ; At this temperature of motor, fans are spinning at maximum
CONTR_TEMP_FAN_MAX              constant    75
FANSPEED_IDLE                   constant    10      ; In idle mode, fans will always run at 10%

; Current settings
Battery_Power_Limit_Init        constant    20      ; per 10W

INIT_DRIVE_OUTPUT_CURLIM         constant   28086        ; 300A  /350A*32767
INIT_REGEN_OUTPUT_CURLIM         constant   5617         ; 60A  /350A*32767

REDUCED_DRIVE_INPUT_CURLIM_PERC constant    60      ; Drive current limit is 60%, when fault is detected
REDUCED_REGEN_INPUT_CURLIM_PERC constant    15      ; Regen current limit is 15%, when fault is detected
REDUCED_THROTTLE_MULTIPLIER     constant    75      ; 60% of 128 = 75
REVERSE_THROTTLE_MULTIPLIER     constant    128     ; 90% of 128 = 115
TIME_TO_FULL_LOS_FAULT          constant    6000    ; Time it takes to go to full effect of the reduced current limits
                                                    ; Due to a fault, max 6000 ms

; DNR, Gear change and steering settings
PROTECTION_DELAY_GRCHANGE       constant    1000    ; Allowed to only change gear once per second, in ms
THROTTLE_MULTIP_REDUCE          constant    26      ; 26/128 = 0.2 multiplier for reducing throttle during gear change
THROTTLE_MULTIP_INCREASE        constant    230     ; 230/128 = 1.8 multiplier to compensate for reduced throttle
MAX_SPEED_CHANGE_DNR            constant    80      ; in km/h, with 1 decimal
IDLE_RPM_THRESHOLD              constant    5       ; If RPM is lower than this value, interlock is turned on after some time
IDLE_THROTTLE_THRESHOLD         constant    2       ; If throttle signal is below this value, interlock is turned on after some time
TIME_TO_START_STOP              constant    10000   ; If car is idle for 10s, turn off interlock

FULL_BRAKE                      constant    32767   ; On a scale of 0-32767, how hard controller will brake
INIT_STEER_COMPENSATION         constant    125     ; 255 no effect, 0 full effect. At max steer angle, multiplier has this value
STEER_ANGLE_NO_EFFECT           constant    2       ; first 2 degrees has no effect on the compensation
HIGH_PEDAL_DISABLE_THRESHOLD    constant    5       ; 0-255, When going into drive mode, throttle has to be reduced below this value

; Received command for DNR
DNR_DRIVE                       constant    1       ; CAN code for Drive
DNR_NEUTRAL                     constant    2       ; CAN code for Neutral
DNR_REVERSE                     constant    3       ; CAN code for Reverse

; STATES IN STATEMACHINE
NEUTRAL                         constant    0
DRIVE16                         constant    2
DRIVE118                        constant    3
REVERSE                         constant    4
PRE_DRIVE                       constant    5
PRE_REVERSE                     constant    6
PRE_NEUTRAL                     constant    7
BRAKE                           constant    8







;****************************************************************
;							VARIABLES
;****************************************************************

; Accesible from programmer handheld (max. 100 user, 10 bit)

;PAR_Total_Maint_interval		alias P_User1		;Can be saved to non-volatile memory

;Auto user variables (max. 300 user, 16 bit)

create reset_controller_remote variable

create  RCV_State_GearChange        variable            ; Smesh enabled, received from Right Motorcontroller

create RCV_System_Action variable

create HPO variable

;-------------- CAN ---------------

create ACK_RM_Fault_System variable
create ACK_RCV_System_Action variable
create RCV_ACK_Fault_System variable
create RCV_ACK_Fault_System_Battery variable

create Interlock_XMT variable

;-------------- Temporaries ---------------

create  temp_Map_Output_1   variable
create  temp_Map_Output_2   variable
create  temp_Calculation    variable
create  temp_Calculation_2    variable
create  temp_VCL_Throttle   variable

create temp_Drive_Current_Limit variable
create temp_Regen_Current_Limit variable

;------------- Fan variables --------------

create MOTOR_FAN_PERCENTAGE variable
create CONTROLLER_FAN_PERCENTAGE variable
create HIGHEST_FAN_PERCENTAGE variable

create RM_Motor_Temperature_Display variable
create Highest_Motor_Temperature variable

create RM_Controller_Temperature_Display variable
create Highest_Controller_Temperature variable

;-------------- Batteries ---------------

create Batt_Serial1 variable                        ; Serial number of Battery
create Batt_Serial2 variable                        
create State_Of_Charge variable                     ; Percentage of the capacity (0-100)
create BMS_State variable                           ; State of the BMS, charging, discharging, fault, etc.
create Pack_capacity variable                       ; Pack capacity in mAh
create Batt_ErrorMSG1 variable                      ; Battery fault bits #1
    Batt_Temp_Alarm                   bit        Batt_ErrorMSG1.1           ; Power stage temperature alarm1
    ;EMPTY
    Batt_CHG_Cur_Alarm                bit        Batt_ErrorMSG1.4           ; 1=Charge current too high
    Batt_DISCHG_Cur_Alarm             bit        Batt_ErrorMSG1.8           ; 1=Discharge current too high
    Batt_OverVoltage_Alarm            bit        Batt_ErrorMSG1.16          ; 1=Pack voltage too high
    ;EMPTY
    ;EMPTY
    Batt_EEPROM_ERROR                 bit        Batt_ErrorMSG1.128         ; EEPROM/Flash error

create Batt_ErrorMSG2 variable                      ; Battery fault bits #2
	Batt_CM_CRC                       bit        Batt_ErrorMSG2.1           ; 1=communication problem between MCU and AFE
    ;EMPTY
    Batt_CM_ALERT                     bit        Batt_ErrorMSG2.4           ; 1=Alert in cell monitoring system
    Batt_CM_FAULT                     bit        Batt_ErrorMSG2.8           ; 1=Fault in cell monitoring system
    Batt_PowerStage_ERROR             bit        Batt_ErrorMSG2.16          ; 1=Problem in power stage
    ;EMPTY...

create Batt_ErrorMSG3 variable                      ; Battery fault bits #3
	Batt_DISCHG_UnderVoltage          bit        Batt_ErrorMSG3.1           ; 1=Pack voltage too low
    Batt_CM_CELL_UnderVoltage         bit        Batt_ErrorMSG3.2           ; 1=Voltage in one or more cells too low.
    Batt_CM_CELL_OverVoltage          bit        Batt_ErrorMSG3.4           ; 1=Voltage in one or more cells too high
    ;EMPTY
    Batt_CHG_TEMP_High                bit        Batt_ErrorMSG3.16          ; 1=Charge temperature too high
    Batt_DISCHG_TEMP_High             bit        Batt_ErrorMSG3.32          ; 1=Disharge temperature too high
    Batt_CHG_TEMP_Low                 bit        Batt_ErrorMSG3.64          ; 1=Charge temperature too low
    Batt_DISCHG_TEMP_Low              bit        Batt_ErrorMSG3.128          ; 1=Disharge temperature too low

create Batt_ErrorMSG4 variable                      ; Battery fault bits #4
	Batt_LeakCurrent                  bit        Batt_ErrorMSG4.1           ; 1=Leakage current detected
    Batt_CAN_TimeOut                  bit        Batt_ErrorMSG4.2           ; 1=CAN message timeout (CAN bus congested)
    ;EMPTY...					



;Standard user variables (max. 120 user, 16 bit)


; Efficiency calculation
LM_Efficiency                     alias      user11     ; Left controller Efficiency
Power_In                          alias      user13     ; Input Power
Power_Out                         alias      user14     ; Ouput Power
Motor_Rads                        alias      user15     ; Rotor speed [rad/s]


; Steering angle
R_Steering_Multiplier             alias      user20     ; Mulitplier for throttle signal to right controller
L_Steering_Multiplier             alias      user21     ; Mulitplier for throttle signal to Left controller
MAX_STEER_COMPENSATION            alias      user22     ; Multiplier at maximum steering angle
Steerangle_VCL                        alias      user23     ; Steerangle_VCL


; Current limits
RM_Regen_Current_Limit            alias      user30     ; Regen current limit for Right controller
RM_Drive_Current_Limit            alias      user31     ; Drive current limit for left controller


; DNR and Throttle
RM_Throttle_Compensated           alias      user40           			; Torque for right motorcontroller, command to Right controller
DNR_Command                       alias      user41           			; DNR command to Right controller
RCV_DNR_Command                   alias      user42           			; DNR command Received from switch
RCV_Throttle                      alias      user43           			; Throttle pedal state
Brake_RCV                         alias      user44           			; Brake pedal pushed
fault_CNT_GearChange              alias      user45                     ; Counts number of times gear change has fault    


; Temperature
Temp_Motor_Display                alias      user50           			; Temperature of Motors
Temp_Contr_Display                alias      user51           			; Temperature of Controllers
Temp_Index_Display                alias      user52           			; Index which motor, controller and battery is the hottest (M-C-BBB)
	Temp_Index_M                      bit        Temp_Index_Display.1   ; 1 = This motor, 0 = Right motor
	Temp_Index_C                      bit        Temp_Index_Display.4   ; 1 = This controller, 0 = Right controller

airFlowIN_PWM					  alias      user53						; (0-32767) PWM output for air inlet fans
airFlowOUT_PWM					  alias      user54						; (0-32767) PWM output for air outlet fans


; States
State_GearChange                  alias      user60     ; State of changing gear
state_DNR                         alias      user61     ; DNR active State
RCV_Key_Switch                    alias      user62     ; position of the Key (On-Off)
Key_Switch                        alias      user63     ; Internal state of the switch
Start_Stop_Init_Complete          alias      user64     ; Init of Start Stop system is complete



test                              alias      user70
    test_bit0                         bit        test.1






;------------- CAN MAILBOXES --------------

; Messages from and to slave controller
MAILBOX_SM_MISO1						alias CAN1
MAILBOX_SM_MISO1_Received				alias CAN1_received
MAILBOX_SM_MOSI1						alias CAN2
MAILBOX_SM_MOSI2						alias CAN3
MAILBOX_SM_MOSI3						alias CAN4
MAILBOX_SM_MISO2						alias CAN5
MAILBOX_SM_MISO2_Received				alias CAN5_received

MAILBOX_RDW_RCV							alias CAN11
MAILBOX_RDW_RCV_Received				alias CAN11_received
MAILBOX_RDW_XMT							alias CAN12

MAILBOX_DRVSEN_RCV						alias CAN13
MAILBOX_DRVSEN_RCV_Received				alias CAN13_received
MAILBOX_BRK_RCV							alias CAN14
MAILBOX_BRK_RCV_Received				alias CAN14_received

; Battery Messages
MAILBOX_BATT_RCV_InfoA					alias CAN15
MAILBOX_BATT_RCV_InfoA_received			alias CAN15_received
MAILBOX_BATT_RCV_InfoC					alias CAN16
MAILBOX_BATT_RCV_InfoC_received			alias CAN16_received

; Error CAN Messages
MAILBOX_ERROR_MESSAGES                  alias CAN17
MAILBOX_ERROR_MESSAGES_RCV_ACK              alias CAN18
MAILBOX_ERROR_MESSAGES_RCV_ACK_received     alias CAN18_received
MAILBOX_ERROR_MESSAGES_RCV_RM           alias CAN19
MAILBOX_ERROR_MESSAGES_RCV_RM_received  alias CAN19_received
MAILBOX_ERROR_MESSAGES_XMT_ACK_RM       alias CAN20

;------------------- Maps --------------------
MAP_GEAR16                              alias MAP1
MAP_GEAR118                             alias MAP2
Angle2Multiplier_MAP                    alias MAP4

;----------- User Defined Faults ------------
Fault_System_Battery                    alias   user119
    Battery_Fault                           bit Fault_System_Battery.1      ; Fault with own measured battery input

Fault_System                 alias      UserFault1
    Regen_Crit_Error                  bit        Fault_System.1             ; (1, Code 51) Critical Error occured with Regen
    Drive_Crit_Error                  bit        Fault_System.2             ; (2, Code 52) Critical Error occured with Driving
    Temp_Crit_Error                   bit        Fault_System.4             ; (3, Code 53) Critical Error occured related to temperature
    Regen_Fault                       bit        Fault_System.8             ; (4, Code 54) Some fault occured with Regen
    Drive_Fault                       bit        Fault_System.16            ; (5, Code 55) Some fault occured with Driving
    Temp_Fault                        bit        Fault_System.32            ; (6, Code 56) Some fault occured related to temperature
    General_Fault                     bit        Fault_System.64            ; (7, Code 57) Some fault occured generally
    General_Crit_Error                bit        Fault_System.128           ; (8, Code 58) Critical Error occured generally
    
User_Fault_Action_01 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
User_Fault_Action_02 = 1101100000100000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off, Full brake
User_Fault_Action_03 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
User_Fault_Action_04 = 0000000000000000b
User_Fault_Action_05 = 0000000000000000b
User_Fault_Action_06 = 0000000000000000b
User_Fault_Action_07 = 0000000000000000b
User_Fault_Action_08 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off

RM_Fault_System                 alias      UserFault2
    RM_Regen_Crit_Error                  bit        RM_Fault_System.1             ; (9, Code 59) Critical Error occured with Regen
    RM_Drive_Crit_Error                  bit        RM_Fault_System.2             ; (10, Code 60) Critical Error occured with Driving
    RM_Temp_Crit_Error                   bit        RM_Fault_System.4             ; (11, Code 61) Critical Error occured related to temperature
    RM_Regen_Fault                       bit        RM_Fault_System.8             ; (12, Code 62) Some fault occured with Regen
    RM_Drive_Fault                       bit        RM_Fault_System.16            ; (13, Code 63) Some fault occured with Driving
    RM_Temp_Fault                        bit        RM_Fault_System.32            ; (14, Code 64) Some fault occured related to temperature
    RM_General_Fault                     bit        RM_Fault_System.64            ; (15, Code 65) Some fault occured generally
    RM_General_Crit_Error                bit        RM_Fault_System.128           ; (16, Code 66) Critical Error occured generally
    
User_Fault_Action_09 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
User_Fault_Action_10 = 1101100000100000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off, Full brake
User_Fault_Action_11 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
User_Fault_Action_12 = 0000000000000000b
User_Fault_Action_13 = 0000000000000000b
User_Fault_Action_14 = 0000000000000000b
User_Fault_Action_15 = 0000000000000000b
User_Fault_Action_16 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off

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
Fan1IN				alias PWM1					;Pin J1-06
Fan2OUT				alias PWM2					;Pin J1-05
;FREE				alias PWM3					;Pin J1-04
;FREE				alias PWM4					;Pin J1-03
;Free				alias PWM5					;Pin J1-02

SmeshLeftPin			alias DigOut6				;Pin J1-19 (16 OFF(clear); 118 ON(set))
SmeshLeftPin_output	    alias Dig6_output
SmeshRightPin   		alias DigOut7				;Pin J1-20
SmeshRightPin_output    alias Dig7_output

;--------- Declaration Section ----------

; none

;--------------- DELAYS -----------------
Startup_DLY                       alias		DLY1             
Startup_DLY_output                alias		DLY1_output
Smesh_DLY						  alias		DLY2
Smesh_DLY_output				  alias		DLY2_output
GEAR_CHANGE_PROT_DLY			  alias		DLY3
GEAR_CHANGE_PROT_DLY_output		  alias		DLY3_output
EMERGENCY_ACK_DLY                 alias     DLY4
EMERGENCY_ACK_DLY_output          alias     DLY4_output
Low_Priority_Mailbox_DLY          alias     DLY5
Low_Priority_Mailbox_DLY_output   alias     DLY5_output
Start_Stop_DLY                    alias     DLY6
Start_Stop_DLY_output             alias     DLY6_output

FAULT_TIMER                       alias      TMR1
FAULT_TIMER_ms_output             alias      TMR1_ms_output
FAULT_TIMER_sec_output            alias      TMR1_sec_output
FAULT_TIMER_enable                alias      TMR1_enable

TEST_TIMER                       alias      TMR2
TEST_TIMER_ms_output             alias      TMR2_ms_output
TEST_TIMER_sec_output            alias      TMR2_sec_output
TEST_TIMER_enable                alias      TMR2_enable

FAULT_RAMP                       alias      RMP1
FAULT_RAMP_output                alias      RMP1_output
FAULT_RAMP_hold                  alias      RMP1_hold




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
setup_Delay(DLY15, 5000)

; Ramp
setup_ramp(FAULT_RAMP, 0, 0, 1)         ; RMP#, Initial value = 0, Move, rate 1/ms
set_ramp_target(FAULT_RAMP, 0)          ; Target init is 0ms

;Initiate sytems
call startupCANSystem 		;setup and start the CAN communications system
call setup_2D_MAP

MAX_STEER_COMPENSATION = INIT_STEER_COMPENSATION            ; 120/255 = 47% reduction on inner wheel with max steering

Battery_Power_Limit = Battery_Power_Limit_Init

; For testing purposes
Steerangle_VCL = 125
RCV_Key_Switch = 1
RCV_Throttle = 0








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
    
    call faultHandling
    
    call checkCANMailboxes
    
    call DNRStatemachine
    
    call controlFans
    
    call calculateEfficiency
    
    if (reset_controller_remote = RESET_PASSWORD) {
        Reset_Controller()
    }
    
    
    goto mainLoop 



    





    
;****************************************************************
;							SUBROUTINES
;****************************************************************

; As with any programming language, the use of subroutines can allow
; easier re-use of code across multiple parts of a program, and across
; programs.  Function specific subroutines can also improve the
; Readability of the code in the main loop.


startupCANSystem:

   			; CAN mailboxes 1-5 are available to VCL CAN. Mailboxes 6-14 are available for CANopen
   			; C_SYNC is buddy check, C_CYCLIC is cyclic CAN messages, C_EVENT are called with send_mailbox()
   			; Set Message T ype and Master ID to 0, and put Slave ID to pre-defined 11bit identifier.
   			;

    Suppress_CANopen_Init = 0			;first undo suppress, then startup CAN, then disable CANopen
    Disable_CANOpen_PDO()				; disables OS PDO mapping and frees 

    setup_CAN(CAN_BAUD_RATE, 0, 0, -1, 0)		;(Baud, Sync, Reserved[0], Slave ID, Restart)
												;Baudrate = 500KB/s setting, no Sync, Not Used, Not Used, Auto Restart



   			; MAILBOX 1
   			; Purpose:		Receive information: Torque, speed, Temperature motor/controller, Smesh enabled
   			; Type:			PDO MISO1
   			; Partner:		Slave Motorcontroller

    Setup_Mailbox(MAILBOX_SM_MISO1, 0, 0, 0x111, C_EVENT, C_RCV, 0, 0)

    Setup_Mailbox_Data(MAILBOX_SM_MISO1, 7,			
        @RCV_System_Action,				
        @RCV_System_Action + USEHB,
        @RM_Motor_Temperature_Display,			; Motor temperature 0-255°C
        @RM_Controller_Temperature_Display,     ; Controller temperature  0-255°C
        @RCV_State_GearChange,				    ; Smesh enabled
        @RM_Fault_System,
        0,
        0)
        
        
        
        
            ; MAILBOX 2
   			; Purpose:		Send information: Torque, Smesh change gear, max speed, regen, commands
   			; Type:			PDO MOSI1
   			; Partner:		Slave Motorcontroller

    Setup_Mailbox(MAILBOX_SM_MOSI1, 0, 0, 0x101, C_CYCLIC, C_XMT, 0, 0)

    Setup_Mailbox_Data(MAILBOX_SM_MOSI1, 8,
        @RM_Throttle_Compensated,			; Torque for right motorcontroller
        @RM_Throttle_Compensated + USEHB, 
        @State_GearChange,			        ; Command to change gear
        @RM_Drive_Current_Limit,						; Set max speed
        @RM_Drive_Current_Limit + USEHB,
        @RM_Regen_Current_Limit,					; Set Regen limit
        @RM_Regen_Current_Limit + USEHB, 
        @DNR_Command)
		
		


   			; MAILBOX 3
   			; Purpose:		Send information: 
   			; Type:			PDO MOSI2
   			; Partner:		Slave Motorcontroller
            
    Setup_Mailbox(MAILBOX_SM_MOSI2, 0, 0, 0x102, C_CYCLIC, C_XMT, 0, 0)
    Setup_Mailbox_Data(MAILBOX_SM_MOSI2, 2,
        @Brake_RCV,
        @Interlock_XMT,
        0,
        0,
        0,
        0,
        0,
        0)


   			; MAILBOX 4
   			; Purpose:		Send information: While gear change, valuable information: 
   			; Type:			PDO MOSI3
   			; Partner:		Slave Motorcontroller
            
    Setup_Mailbox(MAILBOX_SM_MOSI3, 0, 0, 0x100, C_EVENT, C_XMT, 0, 0)

    Setup_Mailbox_Data(MAILBOX_SM_MOSI3, 5, 		
        @RM_Throttle_Compensated,			
        @RM_Throttle_Compensated + USEHB,
        @RM_Drive_Current_Limit,
        @RM_Drive_Current_Limit + USEHB,
        @State_GearChange,				
        0, 
        0,
		0)
            
            ; MAILBOX 5
   			; Purpose:		Receive information:
   			; Type:			PDO MISO2
   			; Partner:		Slave Motorcontroller
         
    Setup_Mailbox(MAILBOX_SM_MISO2, 0, 0, 0x110, C_EVENT, C_RCV, 0, 0)

    Setup_Mailbox_Data(MAILBOX_SM_MISO2, 1, 		
        @RCV_State_GearChange,			; Motor torque
        0, 
        0,				
        0, 
        0,				
        0,
		0,
		0)



   			; MAILBOX 11
   			; Purpose:		Receive information: DNR
   			; Type:			PDO1
   			; Partner:		RDW Scherm

    Setup_Mailbox(MAILBOX_RDW_RCV, 0, 0, 0x012, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_RDW_RCV, 3, 			; DNR switch state
        @RCV_DNR_Command,
		@RCV_Key_Switch,
		@Steerangle_VCL,					; Steerangle_VCL
        0, 
		0,
		0,
		0,
		0)


   			; MAILBOX 12
   			; Purpose:		Send information: Speed, temperature motor & controller, efficiency
   			; Type:			PDO2
   			; Partner:		RDW Scherm

    Setup_Mailbox(MAILBOX_RDW_XMT, 0, 0, 0x200, C_CYCLIC, C_XMT, 0, 0)
    Setup_Mailbox_Data(MAILBOX_RDW_XMT, 8,
        @Vehicle_Speed,				; Speed of the wheels
        @Temp_Motor_Display,		; Temperature of Motors
        @Temp_Contr_Display,		; Temperature of Controllers
        @state_DNR,			    ; Temperature errors from Batteries
        @Temp_Index_Display,		; Index which motor, controller and battery is the hottest (M-C)
        @LM_Efficiency,            ; Regen, Eco or power region
		@State_Of_Charge,
        0)


   			; MAILBOX 13
   			; Purpose:		receive information: Steerangle_VCL, throttle, brake
   			; Type:			PDO3
   			; Partner:		Interieur Verlichting?

    Setup_Mailbox(MAILBOX_DRVSEN_RCV, 0, 0, 0x011, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_DRVSEN_RCV, 1, 		; Efficiency
        @RCV_Throttle,					; Throttle pedal
        0,
		0,
		0,
		0,
		0,
        0,
        0)


   			; MAILBOX 14
   			; Purpose:		receive information: Brake
   			; Type:			PDO4
   			; Partner:		Achterlichten

    Setup_Mailbox(MAILBOX_BRK_RCV, 0, 0, 0x010, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_BRK_RCV, 1, 		; Brake pedal state input
        @Brake_RCV,
		0,
		0,
		0,
		0,
		0,
		0,
		0)


   			; MAILBOX 15
   			; Purpose:		receive information: State and state of charge
   			; Type:			PDO5
   			; Partner:		Batteries

    Setup_Mailbox(MAILBOX_BATT_RCV_InfoA, 0, 0, 0x17A, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_BATT_RCV_InfoA, 8, 		
        @Batt_Serial1,					; Serial number Battery
        @Batt_Serial1 + USEHB,
        @Batt_Serial2, 
        @Batt_Serial2 + USEHB,
        @State_Of_Charge,				; Percentage of the capacity 
        @BMS_State,						; State of BMS: 1-4 init, 5 idle, 6 discharging, 7 charging, 10 fault, 11 critical error, 99 prepare deepsleep, 100 deepsleep
        @Pack_capacity, 				; Capacity remaining in pack, 0 - 65535 mAh
        @Pack_capacity + USEHB)
		
		   	
			; MAILBOX 16
   			; Purpose:		receive information: Error messages of temperature, current flow, voltage
   			; Type:			PDO6
   			; Partner:		Batteries

    Setup_Mailbox(MAILBOX_BATT_RCV_InfoC, 0, 0, 0x17C, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_BATT_RCV_InfoC, 8, 		
        @Batt_Serial1,					; Serial number Battery
        @Batt_Serial1 + USEHB,
        @Batt_Serial2, 
        @Batt_Serial2 + USEHB,
        @Batt_ErrorMSG1, 				; Fault code bits
        @Batt_ErrorMSG2,
        @Batt_ErrorMSG3, 
        @Batt_ErrorMSG4)
        
        
            ; MAILBOX 17
   			; Purpose:		Send information: Error messages of temperature, current flow, voltage
   			; Type:			PDO6
   			; Partner:		RDW Scherm and Slave controller

    Setup_Mailbox(MAILBOX_ERROR_MESSAGES, 0, 0, 0x000, C_EVENT, C_XMT, 0, 0)
    Setup_Mailbox_Data(MAILBOX_ERROR_MESSAGES, 3, 		
        @Fault_System,
        @Fault_System_Battery,
        @RM_Fault_System,
        0,
        0,
        0,
        0,
        0)
        
            ; MAILBOX 18
   			; Purpose:		receive information: Error messages of temperature, current flow, voltage
   			; Type:			PDO6
   			; Partner:		Batteries

    Setup_Mailbox(MAILBOX_ERROR_MESSAGES_RCV_ACK, 0, 0, 0x008, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_ERROR_MESSAGES_RCV_ACK, 2, 		
        @RCV_ACK_Fault_System,
        @RCV_ACK_Fault_System_Battery,
        0,
        0,
        0,
        0,
        0,
        0)
        
        
            ; MAILBOX 19
   			; Purpose:		receive information: Error messages from slave controller
   			; Type:			PDO6
   			; Partner:		Slave controller

    Setup_Mailbox(MAILBOX_ERROR_MESSAGES_RCV_RM, 0, 0, 0x001, C_EVENT, C_RCV, 0, 0)
    Setup_Mailbox_Data(MAILBOX_ERROR_MESSAGES_RCV_RM, 3, 		
        @RM_Fault_System,
        @RCV_System_Action,				
        @RCV_System_Action + USEHB, 
        0,
        0,
        0,
        0,
        0)
        
        
            ; MAILBOX 20
   			; Purpose:		send information: ACK to Error messages from slave controller
   			; Type:			PDO6
   			; Partner:		Slave controller

    Setup_Mailbox(MAILBOX_ERROR_MESSAGES_XMT_ACK_RM, 0, 0, 0x002, C_EVENT, C_XMT, 0, 0)
    Setup_Mailbox_Data(MAILBOX_ERROR_MESSAGES_XMT_ACK_RM, 3, 		
        @ACK_RM_Fault_System,
        @ACK_RCV_System_Action,				
        @ACK_RCV_System_Action + USEHB, 
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

    ;send low priority mailboxes
    
    if (Low_Priority_Mailbox_DLY = 0) {
        Setup_Delay(Low_Priority_Mailbox_DLY, CAN_LOWPRIORITY_RATE)
        
        send_mailbox(MAILBOX_ERROR_MESSAGES)
    }
    
    if (MAILBOX_ERROR_MESSAGES_RCV_RM_received = ON) {
        MAILBOX_ERROR_MESSAGES_RCV_RM_received = OFF
        
        ACK_RM_Fault_System = RM_Fault_System
        ACK_RCV_System_Action = RCV_System_Action
        
        send_mailbox(MAILBOX_ERROR_MESSAGES_XMT_ACK_RM)
        
    }
    
    Interlock_XMT = Interlock_State
    

    return
    
    


    
clear_Obsolete_Errors:

    if ( (Batt_Temp_Alarm = OFF) ) {
        Temp_Crit_Error = OFF
        
    }
	if ( (Batt_CHG_TEMP_High = OFF) & (Batt_DISCHG_TEMP_High = OFF) ) {
        Temp_Fault = OFF
        
        
        
    }
	if ( (Batt_CHG_Cur_Alarm = OFF) & (Batt_OverVoltage_Alarm = OFF) ) {
        Regen_Crit_Error = OFF
        
    }
	if ( (Batt_CM_CELL_OverVoltage = OFF) & (Batt_CHG_TEMP_Low = OFF) ) {
        Regen_Fault = OFF
        
        
        
    }
	if ( (Batt_DISCHG_Cur_Alarm = OFF) & (Batt_DISCHG_UnderVoltage = OFF) ) {
        Drive_Crit_Error = OFF
        
    }
	if ( (Batt_CM_CELL_UnderVoltage = OFF) & (Batt_DISCHG_TEMP_Low = OFF) ) {
        Drive_Fault = OFF
        
        
        
    }
	if ( (Batt_CM_ALERT = OFF) & (Batt_PowerStage_ERROR = OFF) ) {
        General_Crit_Error = OFF
        
    }
	if ( (Batt_EEPROM_ERROR = OFF) & (Batt_CM_CRC = OFF) & (Batt_CM_FAULT = OFF) & (Batt_LeakCurrent = OFF) ) {
        General_Fault = OFF
    }


    return
    
    
    
    
    
    
retrieve_Errors:
    
    if ( (Batt_Temp_Alarm = ON) ) {
        Temp_Crit_Error = ON
        
    }
    if ( (Batt_CHG_TEMP_High = ON) | (Batt_DISCHG_TEMP_High = ON) ) {
        Temp_Fault = ON
        
        
        
    }
    if ( (Batt_CHG_Cur_Alarm = ON) | (Batt_OverVoltage_Alarm = ON) ) {
        Regen_Crit_Error = ON
        
    }
    if ( (Batt_CM_CELL_OverVoltage = ON) | (Batt_CHG_TEMP_Low = ON) ) {
        Regen_Fault = ON
        
        
        
    }
    if ( (Batt_DISCHG_Cur_Alarm = ON) | (Batt_DISCHG_UnderVoltage = ON) ) {
        Drive_Crit_Error = ON
        
    }
    if ( (Batt_CM_CELL_UnderVoltage = ON) | (Batt_DISCHG_TEMP_Low = ON) ) {
        Drive_Fault = ON
        
        
        
    }
    if ( (Batt_CM_ALERT = ON) | (Batt_PowerStage_ERROR = ON) ) {
        General_Crit_Error = ON
        
    }
    if ( (Batt_EEPROM_ERROR = ON) | (Batt_CM_CRC = ON) | (Batt_CM_FAULT = ON) | (Batt_LeakCurrent = ON) ) {
        General_Fault = ON
    }
    
    
    
    if ( (BMS_State = 10) ) {
        General_Fault = ON
    } else if ( (BMS_State = 11) ) {
        General_Crit_Error = ON
    }
    
    if (RCV_System_Action <> 0) {
        System_Action = RCV_System_Action
    }
    
    
    return

    
    
    
faultHandling:
    
    ; When the errors are dissappeared, clear the errors
    call clear_Obsolete_Errors
    
    call retrieve_Errors
    
    if ( (Regen_Fault = ON) | (RM_Regen_Fault = ON) ) {
        
        set_ramp_target(FAULT_RAMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        temp_Map_Output_1 = Map_Two_Points(FAULT_RAMP_output, 0, TIME_TO_FULL_LOS_FAULT, 100, REDUCED_REGEN_INPUT_CURLIM_PERC)
        
        
        Regen_Current_Limit = Map_Two_Points(temp_Map_Output_1, 0, 100, 1638, INIT_REGEN_OUTPUT_CURLIM)
        Brake_Current_Limit = Regen_Current_Limit
        Interlock_Brake_Current_Limit = Regen_Current_Limit
        
        RM_Regen_Current_Limit = Regen_Current_Limit
        
    }
    if ( (Drive_Fault = ON) | (RM_Drive_Fault = ON) ) {
        
        set_ramp_target(FAULT_RAMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        Throttle_Multiplier = REDUCED_THROTTLE_MULTIPLIER        ; Reduce throttle to 80% = 102/128

        temp_Map_Output_1 = Map_Two_Points(FAULT_RAMP_output, 0, TIME_TO_FULL_LOS_FAULT, 100, REDUCED_DRIVE_INPUT_CURLIM_PERC)

        Drive_Current_Limit = Map_Two_Points(temp_Map_Output_1, 0, 100, 1638, INIT_DRIVE_OUTPUT_CURLIM)
        RM_Drive_Current_Limit = Drive_Current_Limit
        
    }
    if ( (Temp_Fault = ON) | (RM_Temp_Fault = ON) ) {
        
        set_ramp_target(FAULT_RAMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        Throttle_Multiplier = REDUCED_THROTTLE_MULTIPLIER        ; Reduce throttle to 80% = 102/128
        
        temp_Map_Output_1 = Map_Two_Points(FAULT_RAMP_output, 0, TIME_TO_FULL_LOS_FAULT, 100, REDUCED_REGEN_INPUT_CURLIM_PERC)
        temp_Map_Output_2 = Map_Two_Points(FAULT_RAMP_output, 0, TIME_TO_FULL_LOS_FAULT, 100, REDUCED_DRIVE_INPUT_CURLIM_PERC)
        
        Drive_Current_Limit = Map_Two_Points(temp_Map_Output_1, 0, 100, 1638, INIT_DRIVE_OUTPUT_CURLIM)
        
        Regen_Current_Limit = Map_Two_Points(temp_Map_Output_1, 0, 100, 1638, INIT_REGEN_OUTPUT_CURLIM)
        Brake_Current_Limit = Regen_Current_Limit
        Interlock_Brake_Current_Limit = Regen_Current_Limit
        
        RM_Drive_Current_Limit = Drive_Current_Limit
        RM_Regen_Current_Limit = Regen_Current_Limit
        
    }
    if ( (General_Fault = ON) | (RM_General_Fault = ON) ) {
        
        set_ramp_target(FAULT_RAMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        Throttle_Multiplier = REDUCED_THROTTLE_MULTIPLIER        ; Reduce throttle to 80% = 102/128
        
        temp_Map_Output_1 = Map_Two_Points(FAULT_RAMP_output, 0, TIME_TO_FULL_LOS_FAULT, 100, REDUCED_REGEN_INPUT_CURLIM_PERC)
        temp_Map_Output_2 = Map_Two_Points(FAULT_RAMP_output, 0, TIME_TO_FULL_LOS_FAULT, 100, REDUCED_DRIVE_INPUT_CURLIM_PERC)
        
        Drive_Current_Limit = Map_Two_Points(temp_Map_Output_1, 0, 100, 1638, INIT_DRIVE_OUTPUT_CURLIM)
        
        Regen_Current_Limit = Map_Two_Points(temp_Map_Output_1, 0, 100, 1638, INIT_REGEN_OUTPUT_CURLIM)
        Brake_Current_Limit = Regen_Current_Limit
        Interlock_Brake_Current_Limit = Regen_Current_Limit
        
        RM_Drive_Current_Limit = Drive_Current_Limit
        RM_Regen_Current_Limit = Regen_Current_Limit
        
    }
    
    if ( (Regen_Fault = OFF) & (Drive_Fault = OFF) & (Temp_Fault = OFF) & (General_Fault = OFF) & (RM_Regen_Fault = OFF) & (RM_Drive_Fault = OFF) & (RM_Temp_Fault = OFF) & (RM_General_Fault = OFF) & (State_GearChange < 0x60) | (State_GearChange > 0x8D) ) {
        ; When there are no faults, set current limits normally
        ; When gear change is busy, don't set limits normally
        
        set_ramp_target(FAULT_RAMP, 0)          ; Ramp goes to 0 again
        
        Throttle_Multiplier = 128
        
        Drive_Current_Limit = INIT_DRIVE_OUTPUT_CURLIM
        
        Regen_Current_Limit = INIT_REGEN_OUTPUT_CURLIM
        Brake_Current_Limit = INIT_REGEN_OUTPUT_CURLIM
        Interlock_Brake_Current_Limit = INIT_REGEN_OUTPUT_CURLIM
        
        ; When there are no faults, let the controller determine it's own current limits
        RM_Drive_Current_Limit = INIT_DRIVE_OUTPUT_CURLIM
        RM_Regen_Current_Limit = INIT_REGEN_OUTPUT_CURLIM
    }
    
    if ( (System_Action <> 0) | (Fault_System <> 0) | (RM_Fault_System <> 0) | (Fault_System_Battery <> 0) ) {
        ; There is some fault, so send to RDW scherm
        
        if ( (EMERGENCY_ACK_DLY_output = 0) & (RCV_ACK_Fault_System <> Fault_System) & (RCV_ACK_Fault_System_Battery <> Fault_System_Battery) ) {
            send_mailbox(MAILBOX_ERROR_MESSAGES)
        
            Setup_Delay(EMERGENCY_ACK_DLY, CAN_EMERGENCY_DELAY_ACK)
        }

    } else {
        RCV_ACK_Fault_System = 0
        RCV_ACK_Fault_System_Battery = 0
    }
    
    return

    
    
 
setup_2D_MAP:
    
    ; When more torque is required at low speeds, change gear to 1:18
    setup_map(MAP_GEAR16, 6,      ; Number of points
        0, 18,            ; At     0 rpm, 18Nm /50
     1500, 18,            ; At  1500 rpm, 18Nm /50
     1800, 21,            ; At  1800 rpm, 20.5Nm /50
     1950, 27,            ; At  1950 rpm, 26.5Nm /50
     2000, 38,            ; At  2000 rpm, 38Nm /50
     2050, 500,            ; From 2000 rpm gear should always stay in 1:6
        0, 0)
    
    ; When torque is not needed or speed is too high, change gear to 1:6
    setup_map(MAP_GEAR118, 5,      ; Number of points
        0, 4,             ; At     0 rpm, 4Nm /50
     4500, 4,             ; At  4500 rpm, 4Nm /50
     5400, 5,             ; At  5400 rpm, 4.8Nm /50
     6400, 9,             ; At  6400 rpm, 8.8Nm /50
     6450, 500,            ; From 6450 rpm gear should always go to 1:6
        0, 0,
        0, 0)
        
    ; Steering angle compensation
    setup_map(Angle2Multiplier_MAP, 3,      ; Number of points
        0, MAX_STEER_COMPENSATION,      ; Steering to the left
      125, 255,                         ; 
      250, MAX_STEER_COMPENSATION,      ; Steering to the right
        0, 0,
        0, 0,
        0, 0,
        0, 0)
    
    return

    
    
    
controlFans:
    
    
    if (Motor_Temperature > RM_Motor_Temperature_Display) {
        Temp_Motor_Display = Map_Two_Points(Motor_Temperature, 0, 2550, 0, 255)
        Temp_Index_M = ON
    } else {
        Temp_Motor_Display = RM_Motor_Temperature_Display
        Temp_Index_M = OFF
    }
    
    ; Fans for motor should run at 100% at 85 C, 95 is HOT, 115 MAX
    if (Temp_Motor_Display > MOTOR_COOLDOWN_THOLD) {
        ; When motor is too hot (above threshold), turn on fans, make them spin faster when temperature rises
        
        MOTOR_FAN_PERCENTAGE = Map_Two_Points(Temp_Motor_Display, MOTOR_COOLDOWN_THOLD, 10, MOTOR_TEMP_FAN_MAX, 100)
    } else if (Temp_Motor_Display < MOTOR_COOLDOWN_THOLD - FAN_TEMPERATURE_HYSTER) {
        ; When motor temperature is below hysteresis, set fan to 0%
        
        MOTOR_FAN_PERCENTAGE = 0
    } ; When temperature is inside hystereses, value will stay the same as last set
    
    
    
    
    if (Controller_Temperature > RM_Controller_Temperature_Display) {
        Temp_Contr_Display = Map_Two_Points(Controller_Temperature, 0, 2550, 0, 255)
        Temp_Index_C = ON
    } else {
        Temp_Contr_Display = RM_Controller_Temperature_Display
        Temp_Index_C = OFF
    }
    
    ; Fans for controller should run at 100% at 75 C, 95 C is MAX
    if (Temp_Contr_Display > CONTROLLER_COOLDOWN_THOLD) {
        ; When controller is too hot (above threshold), turn on fans, make them spin faster when temperature rises
        
        CONTROLLER_FAN_PERCENTAGE = Map_Two_Points(Temp_Contr_Display, CONTROLLER_COOLDOWN_THOLD, 10, CONTR_TEMP_FAN_MAX, 100)
    } else if (Temp_Contr_Display < CONTROLLER_COOLDOWN_THOLD - FAN_TEMPERATURE_HYSTER) {
        ; When controller temperature is below hysteresis, set fan to 0%
        
        CONTROLLER_FAN_PERCENTAGE = 0
    } ; When temperature is inside hystereses, value will stay the same as last set
    
    
    if (MOTOR_FAN_PERCENTAGE > CONTROLLER_FAN_PERCENTAGE) {
        HIGHEST_FAN_PERCENTAGE = MOTOR_FAN_PERCENTAGE
    } else {
        HIGHEST_FAN_PERCENTAGE = CONTROLLER_FAN_PERCENTAGE
    }
    
    if (HIGHEST_FAN_PERCENTAGE = 0) {
        airFlowIN_PWM = 0
        airFlowOUT_PWM = Map_Two_Points(FANSPEED_IDLE, 0, 100, 0, 32767)
    } else {
        airFlowIN_PWM = Map_Two_Points(HIGHEST_FAN_PERCENTAGE, 0, 100, 0, 32767)
        airFlowOUT_PWM = Map_Two_Points(HIGHEST_FAN_PERCENTAGE, 0, 100, 0, 32767)
    }
    
    Put_PWM(Fan1IN, airFlowIN_PWM)
	Put_PWM(Fan2OUT, airFlowOUT_PWM)
    
    
    return
    
    
    
    
    
calculateEfficiency:
    Power_In = get_muldiv(MTD1, Battery_Current, Capacitor_Voltage, 640)        ; Battery_Current  ; Capacitor_Voltage 0-200V (0-12800) [W]
    
    if (Motor_RPM >= 0) {
        Motor_Rads = Map_Two_Points(Motor_RPM, 0, 12000, 0, 1257)
    } else {
        Motor_Rads = Map_Two_Points(-Motor_RPM, 0, 12000, 0, -1257)
    }
    
    Power_Out = Motor_Torque * Motor_Rads               ; Motor_Torque ; Motor_RPM -12000-12000rpm (-12000-12000) [W]
    
    return
    

    
DNRStatemachine:

    ; STATE MACHINE
    ; 52Hz
    
    if ((Key_Switch = 1)) {
        ; Turn car 'on'
        if ((Interlock_State = OFF)) {
            set_Interlock()
        }
        
        
        Key_Switch = RCV_Key_Switch
        
    } else {
        ; turn car 'off'
        if ((Interlock_State = OFF)) {
            clear_Interlock()
        }
        
        Key_Switch = RCV_Key_Switch
    }
    
    if (Brake_RCV = 1) {
        VCL_Brake = FULL_BRAKE   ; Map_Two_Points(100, 0, 100, 0, 32767)
        ;RCV_Throttle = 0
    } else {
        VCL_Brake = 0
    }
    
    
    
    
    ;Put_Spy_Message("LOOP",1,"AG",PSM_TEXT_ONLY)
    
    ;Setup_Delay(DLY16, 500)
    ;while (DLY16_output <> 0) {}			; Wait 500ms before start
    
    if ( (State_GearChange >= 0x60) & (State_GearChange < 0x6D) ) {
        
        ;; Changing gear to 1:6
        call setSmeshTo16
        
        
        if (State_GearChange = 0x6D) {
            ; Gear change is finished
            fault_CNT_GearChange = 0
            State_GearChange = 0x01
            
            if (state_DNR = DRIVE118) {
                state_DNR= DRIVE16
            }
            
        } else if (State_GearChange = 0xFF) {
            fault_CNT_GearChange = fault_CNT_GearChange + 1
            State_GearChange = 0x60
            
            exit
        } else {
            exit
        }
        
        
    } else if ( (State_GearChange >= 0x80) & (State_GearChange < 0x8D) ) {
        ;; Changing gear to 1:18
        call setSmeshTo118
        
        
        if (State_GearChange = 0x8D) {
            ; Gear change is finished
            fault_CNT_GearChange = 0
            State_GearChange = 0x01
            
            if (state_DNR = DRIVE16) {
                state_DNR = DRIVE118
            } else if (state_DNR = PRE_DRIVE) {
                ; Just go back to the main state
            } else if (state_DNR = PRE_REVERSE) {
                ; Just go back to the main state
            }
            
        } else if (State_GearChange = 0xFF) {
            fault_CNT_GearChange = fault_CNT_GearChange + 1
            State_GearChange = 0x80
            
            exit
        } else {
            exit
        }
        

    } else if (state_DNR = NEUTRAL) {
        temp_VCL_Throttle = 0
        
        if (RCV_DNR_Command = DNR_DRIVE) {          ; if received DNR command is Drive
            
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
            
            state_DNR= PRE_DRIVE
        } else if (RCV_DNR_Command = DNR_REVERSE) {       ; if received DNR command is Reverse
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
            
            state_DNR= PRE_REVERSE
        } else if (RCV_DNR_Command = DNR_NEUTRAL) {          ; if received DNR command is Neutral
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
        }
        
    
    } else if (state_DNR = DRIVE118) {      ; In 1:18 gear
        
        ; Check efficiency
        ; check with map
        ; if efficency is wrong switch gear to 16
        
        if ( HPO = 0 ) {
            temp_VCL_Throttle = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)     ; Set throttle to position of pedal
        } else if ( RCV_Throttle >= HIGH_PEDAL_DISABLE_THRESHOLD ) {
            temp_VCL_Throttle = 0
        } else {
            ; HPO is still 1 and throttle is below threshold
            HPO = 0
            temp_VCL_Throttle = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)     ; Set throttle to position of pedal
        }
        
        
        ; Check Efficiency
        temp_Map_Output_1 = get_map_output(MAP_GEAR118, Motor_RPM)
        
        if ( (Motor_Torque < temp_Map_Output_1) & (GEAR_CHANGE_PROT_DLY = 0) ) {
            ; 1:6 is more efficient, so switch to 1:6
            
            State_GearChange = 0x60
            
        }
        
        if (RCV_DNR_Command = DNR_NEUTRAL) {          ; if received DNR command is Neutral
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
            
            state_DNR= PRE_NEUTRAL
        } else if (RCV_DNR_Command = DNR_REVERSE) {       ; if received DNR command is Reverse
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
            
            state_DNR= PRE_REVERSE
        } else if (RCV_DNR_Command = DNR_DRIVE) {          ; if received DNR command is Drive
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
        }
        
        
        
        
    } else if (state_DNR = DRIVE16) {
        
        ; Check efficiency
        ; check with map
        ; if efficency is wrong switch gear to 118
        
        if ( HPO = 0 ) {
            temp_VCL_Throttle = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)     ; Set throttle to position of pedal
        } else if ( RCV_Throttle >= HIGH_PEDAL_DISABLE_THRESHOLD ) {
            temp_VCL_Throttle = 0
        } else {
            ; HPO is still 1 and throttle is below threshold
            HPO = 0
            temp_VCL_Throttle = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)     ; Set throttle to position of pedal
        }
        
        ; Check Efficiency
        temp_Map_Output_1 = get_map_output(MAP_GEAR16, Motor_RPM)
        
        if ( (Motor_Torque > temp_Map_Output_1) & (GEAR_CHANGE_PROT_DLY = 0) ) {
            ; 1:18 is more efficient, so switch to 1:18
            
            State_GearChange = 0x80
            
        }
        
        if (RCV_DNR_Command = DNR_NEUTRAL) {          ; if received DNR command is Neutral
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
            
            state_DNR= PRE_NEUTRAL
        } else if (RCV_DNR_Command = DNR_REVERSE) {       ; if received DNR command is Reverse
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
            
            state_DNR = PRE_REVERSE
        } else if (RCV_DNR_Command = DNR_DRIVE) {          ; if received DNR command is Drive
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
        }
        
        
        
        
    } else if (state_DNR = REVERSE) {
        
        if ( HPO = 0 ) {
            temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, -32767)
            temp_VCL_Throttle = get_muldiv(MTD1, temp_Map_Output_1, REVERSE_THROTTLE_MULTIPLIER, 128)     ; Set throttle to position of pedal
        } else if ( RCV_Throttle >= HIGH_PEDAL_DISABLE_THRESHOLD ) {
            temp_VCL_Throttle = 0
        } else {
            ; HPO is still 1 and throttle is below threshold
            HPO = 0
            temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, -32767)
            temp_VCL_Throttle = get_muldiv(MTD1, temp_Map_Output_1, REVERSE_THROTTLE_MULTIPLIER, 128)     ; Set throttle to position of pedal
        }
        
        if (RCV_DNR_Command = DNR_NEUTRAL) {          ; if received DNR command is Neutral
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
            
            state_DNR= PRE_NEUTRAL
        } else if (RCV_DNR_Command = DNR_DRIVE) {       ; if received DNR command is Drive
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
            
            state_DNR = PRE_DRIVE
        } else if (RCV_DNR_Command = DNR_REVERSE) {          ; if received DNR command is Reverse
            RCV_DNR_Command = 0     ; Clear Command from RDW scherm
        }
        
        
    } else if (state_DNR = PRE_DRIVE) {
        ;Put_Spy_Message("PRE_DR",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        ; Check whether car can be put in Drive
        if (Vehicle_Speed < -MAX_SPEED_CHANGE_DNR) {
            ; Car drives too fast BACKWARDS, so put back to neutral
            
            state_DNR = NEUTRAL
        } else {
            ; set gear to 1:18 and go to drive state
            State_GearChange = 0x80
            call setSmeshTo118
            
            if (State_GearChange = 0x8D) {
                ; When gear change is not aborted, change state
                fault_CNT_GearChange = 0
                HPO = 1                     ; Initialize High Pedal, cleared when throttle is below threshold
                
                state_DNR= DRIVE118
                State_GearChange = 0x01
            }

        }
        
        
    } else if (state_DNR = PRE_REVERSE) {
        
        
        ; Check whether car can be put in Reverse
        if (Vehicle_Speed > MAX_SPEED_CHANGE_DNR) {
            ; Car drives too fast, so put back to neutral
            
            state_DNR= NEUTRAL
            
        } else {
            ; Set gear to 1:18 and go to reverse state
            State_GearChange = 0x80
            call setSmeshTo118
            
            if (State_GearChange = 0x8D) {
                ; When gear change is not aborted, change state
                fault_CNT_GearChange = 0
                HPO = 1                     ; Initialize High Pedal, cleared when throttle is below threshold
                
                state_DNR= REVERSE
                State_GearChange = 0x01
            }
        }
        
        
    } else if (state_DNR = PRE_NEUTRAL) {
        
        state_DNR = NEUTRAL
        
    }
    
    if (fault_CNT_GearChange > 3) {
        state_DNR = PRE_NEUTRAL
        State_GearChange = 0x01
    }
    
    
    ; Calculate Steeringangle factor
    ; reduce throttle on inner wheel, independent of speed
    
    if (Steerangle_VCL > (125+STEER_ANGLE_NO_EFFECT)) {
        ; Steering to right
        R_Steering_Multiplier = get_map_output(Angle2Multiplier_MAP, Steerangle_VCL)
        L_Steering_Multiplier = 255
        
    } else if (Steerangle_VCL < (125-STEER_ANGLE_NO_EFFECT)) {
        ; Steering to left
        L_Steering_Multiplier = get_map_output(Angle2Multiplier_MAP, Steerangle_VCL)
        R_Steering_Multiplier = 255
        
    } else {
        L_Steering_Multiplier = 255
        R_Steering_Multiplier = 255
    }
    
    VCL_Throttle = get_muldiv(MTD1, temp_VCL_Throttle, L_Steering_Multiplier, 255)
    temp_Calculation = get_muldiv(MTD1, temp_VCL_Throttle, R_Steering_Multiplier, 255)
    RM_Throttle_Compensated = get_muldiv(MTD1, temp_Calculation, Throttle_Multiplier, 128)
    
    
    ; Set right DNR equal to left DNR
    if ( (state_DNR = NEUTRAL) | (state_DNR = REVERSE) | (state_DNR = DRIVE118) | (state_DNR = DRIVE16) ) {
        ; If state is defined, send to right controller
        DNR_Command = state_DNR
        
    } ; else keep previous state
    

    
    return
    
    
    
    
    
setSmeshTo16:

    if ( (State_GearChange = 0x60) & (SmeshLeftPin_output = OFF) & (SmeshRightPin_output = OFF) ) {
        ; Smesh is already in 1:6 mode (16 OFF(clear); 118 ON(set))
        ; So directly go to the end of the functions
        
        ;Put_Spy_Message("16G",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        State_GearChange = 0x6C
    }

    
    ;;;;; 1. Reduce left throttle
    
    if (State_GearChange = 0x60) {
        temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        VCL_Throttle = get_muldiv(MTD1, THROTTLE_MULTIP_REDUCE, temp_Map_Output_1, 128)
        State_GearChange = 0x61
        
        ;Put_Spy_Message("16B",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        send_mailbox(MAILBOX_SM_MOSI3)
        Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK)
    }
    
    
    ;;;;; 2. Receive ACK: Procedure has started
    
    if (State_GearChange = 0x61) {
        
        if (RCV_State_GearChange = 0x62) {
            ; Right controller has changed throttle
            State_GearChange = 0x62
        } else if (Smesh_DLY_output = 0) {
            ; It takes too much time to change gear
            State_GearChange = 0xFF
            
        }
        
    }
        
        
    ;;;;; 3. multiply throttle of right controller with 2
    if (State_GearChange = 0x62) {
        temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        RM_Throttle_Compensated = get_muldiv(MTD1, THROTTLE_MULTIP_INCREASE, temp_Map_Output_1, 128)
        RM_Drive_Current_Limit = get_muldiv(MTD1, THROTTLE_MULTIP_INCREASE, temp_Drive_Current_Limit, 128)
        State_GearChange = 0x63
        
        send_mailbox(MAILBOX_SM_MOSI3)
        Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK)
    }
    
    
    ;;;;; 4. receive ACK from controller: throttle is increased
    
    if (State_GearChange = 0x63) {
        
        if (RCV_State_GearChange = 0x64) {
            ; Right controller has changed throttle
            State_GearChange = 0x64
        } else if (Smesh_DLY_output = 0) {
            ; It takes too much time to change gear
            State_GearChange = 0xFF
            
        }
        
    }
    
    
    ;;;;; 5. Switch gear to 1:6
    
    if (State_GearChange = 0x64) {
        ; 16 OFF(clear); 118 ON(set)
        clear_DigOut(SmeshLeftPin)                          ; Change gear
        State_GearChange = 0x65
        
        
    ;;;;; 6. Reduce throttle right controller
        temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        RM_Throttle_Compensated = get_muldiv(MTD1, THROTTLE_MULTIP_REDUCE, temp_Map_Output_1, 128)
        RM_Drive_Current_Limit = temp_Drive_Current_Limit
        State_GearChange = 0x66
        
        send_mailbox(MAILBOX_SM_MOSI3)
        Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK)
    }
    
    
    ;;;;; 7. receive ACK from controller: throttle is reduced
    
    if (State_GearChange = 0x66) {
        
        if (RCV_State_GearChange = 0x67) {
            ; Right controller has changed throttle
            State_GearChange = 0x67
        } else if (Smesh_DLY_output = 0) {
            ; It takes too much time to change gear
            State_GearChange = 0xFF
            
        }
    }
    
    ;;;;; 8. Increase left throttle
    if (State_GearChange = 0x67) {
        temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        VCL_Throttle = get_muldiv(MTD1, THROTTLE_MULTIP_INCREASE, temp_Map_Output_1, 128)
        Drive_Current_Limit = get_muldiv(MTD1, THROTTLE_MULTIP_INCREASE, temp_Drive_Current_Limit, 128)
        State_GearChange = 0x68
        
        
    ;;;;; 10. receive ACK from controller: procedure has started
        ; 16 OFF(clear); 118 ON(set)
        clear_DigOut(SmeshRightPin)                          ; Change gear
        
        State_GearChange = 0x6A
        send_mailbox(MAILBOX_SM_MOSI3)
    
    
    ;;;;; 11. reduce left throttle to normal

        VCL_Throttle = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        Drive_Current_Limit = temp_Drive_Current_Limit
        State_GearChange = 0x6B
        
        
    ;;;;; 12. increase throttle right controller to normal
        
        RM_Throttle_Compensated = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        State_GearChange = 0x6C
        send_mailbox(MAILBOX_SM_MOSI3)
        
        
    ;;;;; 13. Change is successful, thus speed_to_RPM can be changed
    
        ;Put_Spy_Message("16S",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        Speed_to_RPM = 601          ; (G/d)*5305 ... 6/530*5305 ... One decimal
        
        ; Gear state to complete
        State_GearChange = 0x6D
        send_mailbox(MAILBOX_SM_MOSI3)
        
        Setup_Delay(GEAR_CHANGE_PROT_DLY, PROTECTION_DELAY_GRCHANGE)
    }
    
    
    
    if ( (State_GearChange = 0xFF) | (RCV_State_GearChange = 0xFF) ) {
        
        ;Put_Spy_Message("FL_CGR",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        clear_DigOut(SmeshLeftPin)                          ; Change gear
        clear_DigOut(SmeshRightPin)                          ; Change gear
        State_GearChange = 0x6D

        ; Abort mission
        ;RCV_State_GearChange = 0
        ;State_GearChange = 0xFF
        ;
        ;Drive_Current_Limit = temp_Drive_Current_Limit
        ;RM_Drive_Current_Limit = temp_Drive_Current_Limit
        ;VCL_Throttle = get_muldiv(MTD1, THROTTLE_MULTIP_REDUCE, RCV_Throttle, 128)
        ;RM_Throttle_Compensated = RCV_Throttle
        ;
        ;send_mailbox(MAILBOX_SM_MOSI3)
        ;
        ;Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK*2)
        ;while ((Smesh_DLY_output <> 0) & (RCV_State_GearChange <> 0xFF)) {}		; Wait until right controller has changed throttle
       ; 
        ;if (RCV_State_GearChange = 0xFF) {
        ;    ; Slave accepts mission abort, reset left throttle
        ;    VCL_Throttle = RCV_Throttle
        ;} else {
        ;    ; Serious CAN problems, turn off throttle
        ;    RM_Throttle_Compensated = 0
        ;    VCL_Throttle = 0
         ;   
        ;}
        ;send_mailbox(MAILBOX_SM_MOSI3)
        
    }
    
    
	return

    
setSmeshTo118:
    
    
	if ( (State_GearChange = 0x80) & (SmeshLeftPin_output = ON) & (SmeshRightPin_output = ON) ) {
        ; Smesh is already in 1:18 mode (16 OFF(clear); 118 ON(set))
        ; So directly go to the end of the functions
        
        ;Put_Spy_Message("118G",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        State_GearChange = 0x8C
    }

    
    ;;;;; 1. Reduce left throttle
    
    if (State_GearChange = 0x80) {
        temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        VCL_Throttle = get_muldiv(MTD1, THROTTLE_MULTIP_REDUCE, temp_Map_Output_1, 128)
        State_GearChange = 0x81
        
        ;Put_Spy_Message("118B",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        send_mailbox(MAILBOX_SM_MOSI3)
        Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK)
    }
    
    
    ;;;;; 2. Receive ACK: Procedure has started
    
    if (State_GearChange = 0x81) {
        
        if (RCV_State_GearChange = 0x82) {
            ; Right controller has changed throttle
            State_GearChange = 0x82
        } else if (Smesh_DLY_output = 0) {
            ; It takes too much time to change gear
            State_GearChange = 0xFF
            
        }
        
    }
        
        
    ;;;;; 3. multiply throttle of right controller with 2
    if (State_GearChange = 0x82) {
        temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        RM_Throttle_Compensated = get_muldiv(MTD1, THROTTLE_MULTIP_INCREASE, temp_Map_Output_1, 128)
        RM_Drive_Current_Limit = get_muldiv(MTD1, THROTTLE_MULTIP_INCREASE, temp_Drive_Current_Limit, 128)
        State_GearChange = 0x83
        
        send_mailbox(MAILBOX_SM_MOSI3)
        Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK)
    }
    
    
    ;;;;; 4. receive ACK from controller: throttle is increased
    
    if (State_GearChange = 0x83) {
        
        if (RCV_State_GearChange = 0x84) {
            ; Right controller has changed throttle
            State_GearChange = 0x84
        } else if (Smesh_DLY_output = 0) {
            ; It takes too much time to change gear
            State_GearChange = 0xFF
            
        }
        
    }
    
    
    ;;;;; 5. Switch gear to 1:18
    
    if (State_GearChange = 0x84) {
        ; 16 OFF(clear); 118 ON(set)
        set_DigOut(SmeshLeftPin)                          ; Change gear to 1:18
        State_GearChange = 0x85
        
        
    ;;;;; 6. Reduce throttle right controller
        temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        RM_Throttle_Compensated = get_muldiv(MTD1, THROTTLE_MULTIP_REDUCE, temp_Map_Output_1, 128)
        RM_Drive_Current_Limit = temp_Drive_Current_Limit
        State_GearChange = 0x86
        
        send_mailbox(MAILBOX_SM_MOSI3)
        Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK)
    }
    
    
    ;;;;; 7. receive ACK from controller: throttle is reduced
    
    if (State_GearChange = 0x86) {
        
        if (RCV_State_GearChange = 0x87) {
            ; Right controller has changed throttle
            State_GearChange = 0x87
        } else if (Smesh_DLY_output = 0) {
            ; It takes too much time to change gear
            State_GearChange = 0xFF
            
        }
    }
    
    ;;;;; 8. Increase left throttle
    if (State_GearChange = 0x87) {
        temp_Map_Output_1 = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        VCL_Throttle = get_muldiv(MTD1, THROTTLE_MULTIP_INCREASE, temp_Map_Output_1, 128)
        Drive_Current_Limit = get_muldiv(MTD1, THROTTLE_MULTIP_INCREASE, temp_Drive_Current_Limit, 128)
        State_GearChange = 0x88
        
    
    ;;;;; 10. receive ACK from controller: procedure has started
        ; 16 OFF(clear); 118 ON(set)
        set_DigOut(SmeshRightPin)                          ; Change gear to 1:18
        State_GearChange = 0x8A
        

    ;;;;; 11. reduce left throttle to normal
        VCL_Throttle = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        Drive_Current_Limit = temp_Drive_Current_Limit
        State_GearChange = 0x8B
        
        
    ;;;;; 12. increase throttle right controller to normal
        
        RM_Throttle_Compensated = Map_Two_Points(RCV_Throttle, 0, 255, 0, 32767)
        State_GearChange = 0x8C
        
        send_mailbox(MAILBOX_SM_MOSI3)
        Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK)
    }
    
    ;;;;; 13. Receive ACK, procedure has finished
    
    if (State_GearChange = 0x8C) {
        if (RCV_State_GearChange = 0x8D) {
            ; Right controller has changed throttle
            State_GearChange = 0x8D
        } else if (Smesh_DLY_output = 0) {
            ; It takes too much time to change gear
            State_GearChange = 0xFF
            
        }
    }
        
        
    ;;;;; 13. Change is successful, thus speed_to_RPM can be changed
    
    if (State_GearChange = 0x8D) {
        Speed_to_RPM = 1802          ; (G/d)*5305 ... 18/530*5305 ... One decimal
        
        ; Gear state to complete
        
        ;Put_Spy_Message("118S",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        Setup_Delay(GEAR_CHANGE_PROT_DLY, PROTECTION_DELAY_GRCHANGE)
    }
    
    
    
    if ( (State_GearChange = 0xFF) | (RCV_State_GearChange = 0xFF) ) {
        
        ;Put_Spy_Message("FL_CGR",0,"",PSM_TEXT_ONLY)
        
        ;Setup_Delay(DLY16, 500)
        ;while (DLY16_output <> 0) {}			; Wait 500ms before start
        
        set_DigOut(SmeshLeftPin)                          ; Change gear
        set_DigOut(SmeshRightPin)                          ; Change gear
        State_GearChange = 0x8D

        ; Abort mission
        ;RCV_State_GearChange = 0
        ;State_GearChange = 0xFF
        ;
        ;Drive_Current_Limit = temp_Drive_Current_Limit
        ;RM_Drive_Current_Limit = temp_Drive_Current_Limit
        ;VCL_Throttle = get_muldiv(MTD1, THROTTLE_MULTIP_REDUCE, RCV_Throttle, 128)
        ;RM_Throttle_Compensated = RCV_Throttle
        ;
        ;send_mailbox(MAILBOX_SM_MOSI3)
        ;
        ;Setup_Delay(Smesh_DLY, CAN_DELAY_FOR_ACK*2)
        ;while ((Smesh_DLY_output <> 0) & (RCV_State_GearChange <> 0xFF)) {}		; Wait until right controller has changed throttle
        ;
        ;if (RCV_State_GearChange = 0xFF) {
        ;    ; Slave accepts mission abort, reset left throttle
        ;    VCL_Throttle = RCV_Throttle
        ;} else {
        ;    ; Serious CAN problems, turn off throttle
        ;    RM_Throttle_Compensated = 0
        ;    VCL_Throttle = 0
        ;    
        ;}
        ;send_mailbox(MAILBOX_SM_MOSI3)
        
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