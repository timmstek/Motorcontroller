;===============================================================================================
;					Controller Master - Master motor controller (Left)
;===============================================================================================

; GIT: Master

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
;;;;   Current_Request, how many decimals (0-12800)? 
;;;;   Brake command does not override throttle command ; When error is encountered again, send tact file


; TO DO:
; Get Batteries working with controller. Then get more info from 1 battery

; TEST:
; current cutbacks
; ACK from master when slave sends error

; In Car:
; - Torque limiting


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

RESET_PASSWORD                  constant    141     ; password for "resetControllerRemote" to reset controller

STARTUP_DELAY					constant    3000    ; time before startup system [ms]

DEBOUNCE_INPUTS					constant    5		; this sets the debounce time for switch inputs to 20ms, 4mS/unit

LOW_PRIO_LOOP_RATE              constant    100     ; 

; CAN Settings
CAN_DELAY_FOR_ACK               constant    500     ; in ms
CAN_EMERGENCY_DELAY_ACK         constant    100
CAN_LOWPRIORITY_RATE            constant    800     ; sets the cyclic rate of low priority mailboxes
CAN_CYCLIC_RATE					constant    25		; this sets the cyclic cycle to every 100 ms, 4mS/unit

MULTIPLIER_CAN_NOTHING_RECEIVE  constant    2       ; Cyclic Rate times 2. If nothing is received from slave for this amount of time, create error
CAN_NOTHING_RECEIVE_INIT        constant    1000

; Fan Settings
FAN_TEMPERATURE_HYSTER          constant    5       ; Temperature should first drop this amount under threshold to turn off fans
MOTOR_COOLDOWN_THOLD            constant    60      ; from this temperature in C Fans will turn on
CONTROLLER_COOLDOWN_THOLD       constant    50
MOTOR_TEMP_FAN_MAX              constant    85      ; At this temperature of motor, fans are spinning at maximum
CONTR_TEMP_FAN_MAX              constant    75
FANSPEED_IDLE_IN                constant    10      ; In idle mode, fans will always run at 10%
FANSPEED_IDLE_OUT               constant    0


; Current settings
BATT_DRIVE_PWR_LIM_INIT         constant    20          ; per 10W
BATT_REGEN_PWR_LIM_INIT         constant    20           ; per 10W

BATTERY_DRIVE_POWER_LIMIT_MIN   constant    0
BATTERY_REGEN_POWER_LIMIT_MIN   constant    0

REDUCED_DRIVE_INPUT_CURLIM_PERC constant    60      ; Drive current limit is 60%, when fault is detected
REDUCED_REGEN_INPUT_CURLIM_PERC constant    15      ; Regen current limit is 15%, when fault is detected
REVERSE_THROTTLE_MULTIPLIER     constant    128     ; 90% of 128 = 115
TIME_TO_FULL_LOS_FAULT          constant    10000    ; Time it takes to go to full effect of the reduced current limits
                                                    ; Due to a fault, max 6000 ms

; DNR, Gear change and steering settings
GEAR_CHANGE_BEFORE_DELAY        constant    2000
GEAR_CHANGE_AFTER_DELAY         constant    500
PROTECTION_DELAY_GRCHANGE       constant    5000    ; Allowed to only change gear once per second, in ms
THROTTLE_MULTIP_REDUCE          constant    26      ; 26/128 = 0.2 multiplier for reducing throttle during gear change
MAX_SPEED_CHANGE_DNR            constant    50      ; in km/h, with 1 decimal
MINIMUM_SPEED_GEAR16            constant    50       ; minimum vehicle speed for gear 1:6, with 1 decimal

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

keySwitchHardResetComplete		alias P_User1		;Can be saved to non-volatile memory

;Auto user variables (max. 300 user, 16 bit)

create resetControllerRemote variable

create rcvStateGearChange variable            ; Smesh enabled, received from Right Motorcontroller

create rcvSystemAction variable

create HPO variable

;-------------- CAN ---------------

create riMcFaultSystemACK variable
create riMcSystemActionACK variable
create rcvFaultSystemACK variable
create rcvRiMcFaultSystemACK variable
create FaultSystemDisplay variable

create interlockXMT variable

create rcvRequestSlaveParam variable

create XMT_BATT_DRIVE_PWR_LIM_INIT variable

create Slave_Param_Var1 variable
create Slave_Param_Var2 variable
create Slave_Param_Var3 variable
create Slave_Param_Var4 variable

;-------------- Temporaries ---------------

create  mapOutputTemp1   variable
create  calculationTemp1  variable
create  leMcThrottleTemp   variable
create  riMcThrottleTemp    variable

;------------- Fan variables --------------

create motorFanPerc variable
create contrFanPerc variable
create fanPercHighest variable

create riMcMotorTempDisplay variable
create riMcContrTempDisplay variable

create contrTempDisplay variable
create contrTempDisplayHighest variable
create motorTempDisplay variable
create motorTempDisplayHighest variable

;-------------- Start Stop ---------------

create startStopInit variable
create startStopActive variable

;-------------- Batteries ---------------

create battSerial1 variable                        ; Serial number of Battery
create battSerial2 variable                        
create stateOfCharge variable                     ; Percentage of the capacity (0-100)
create BMSState variable                           ; State of the BMS, charging, discharging, fault, etc.
create packCapacity variable                       ; Pack capacity in mAh
create battErrorMsg1 variable                      ; Battery fault bits #1
    battTempAlarm                       bit        battErrorMsg1.1           ; Power stage temperature alarm1
    ;EMPTY
    battChargeCurAlarm                  bit        battErrorMsg1.4           ; 1=Charge current too high
    battDischargeCurAlarm               bit        battErrorMsg1.8           ; 1=Discharge current too high
    battOvervoltageAlarm                bit        battErrorMsg1.16          ; 1=Pack voltage too high
    ;EMPTY
    ;EMPTY
    battEEPROMError                     bit        battErrorMsg1.128         ; EEPROM/Flash error

create battErrorMsg2 variable                      ; Battery fault bits #2
	battCMCRC                           bit        battErrorMsg2.1           ; 1=communication problem between MCU and AFE
    ;EMPTY
    battCMAlert                         bit        battErrorMsg2.4           ; 1=Alert in cell monitoring system
    battCMFault                         bit        battErrorMsg2.8           ; 1=Fault in cell monitoring system
    battPowerStageError                 bit        battErrorMsg2.16          ; 1=Problem in power stage
    ;EMPTY...

create battErrorMsg3 variable                      ; Battery fault bits #3
	battDischargeUndervoltage           bit        battErrorMsg3.1           ; 1=Pack voltage too low
    battCMCellUndervoltage              bit        battErrorMsg3.2           ; 1=Voltage in one or more cells too low.
    battCMCellOvervoltage               bit        battErrorMsg3.4           ; 1=Voltage in one or more cells too high
    ;EMPTY
    battChargeTempHigh                  bit        battErrorMsg3.16          ; 1=Charge temperature too high
    battDischargeTempHigh               bit        battErrorMsg3.32          ; 1=Disharge temperature too high
    battChargeTempLow                   bit        battErrorMsg3.64          ; 1=Charge temperature too low
    battDischargeTempLow                bit        battErrorMsg3.128          ; 1=Disharge temperature too low

create battErrorMsg4 variable                      ; Battery fault bits #4
	battLeakCurrent                     bit        battErrorMsg4.1           ; 1=Leakage current detected
    battCANTimeout                      bit        battErrorMsg4.2           ; 1=CAN message timeout (CAN bus congested)
    ;EMPTY...					



;Standard user variables (max. 120 user, 16 bit)


; Efficiency calculation
leMcEfficiencyPerc                  alias      user11     ; Left controller Efficiency
powerDissipContr                    alias      user12
powerIn                             alias      user13     ; Input Power
powerOut                            alias      user14     ; Ouput Power
powerMech                           alias      user15
motorRads                           alias      user16     ; Rotor speed [rad/s]
inputVoltage                        alias      user17
outputVoltage                       alias      user18
powerInDisplay                      alias      user19


; Steering angle
riSteeringMultiplier                alias      user20     ; Mulitplier for throttle signal to right controller
leSteeringMultiplier                alias      user21     ; Mulitplier for throttle signal to Left controller
MAX_STEER_COMPENSATION              alias      user22     ; Multiplier at maximum steering angle
rcvSteerangle                       alias      user23     ; rcvSteerangle


; Current limits
riMcBattRegenPowerLim               alias      user30     ; Regen current limit for Right controller
riMcBattDrivePowerLim               alias      user31     ; Drive current limit for Right controller
battRegenPowerLim                   alias      user32     ; Regen current limit for Left controller
battDrivePowerLim                   alias      user33     ; Drive current limit for Left controller

LOSActive                           alias      user34

; DNR and Throttle
riMcThrottleComp                    alias      user40           			; Torque for right motorcontroller, command to Right controller
DNRCommand                          alias      user41           			; DNR command to Right controller
rcvDNRCommand                       alias      user42           			; DNR command Received from switch
rcvThrottle                         alias      user43           			; Throttle pedal state
rcvBrake                            alias      user44           			; Brake pedal pushed
faultCNTGearChange                  alias      user45                     ; Counts number of times gear change has fault    


; Temperature
tempDisplayTemp                     alias      user52           			; Index which motor, controller and battery is the hottest (M-C-BBB)
	Temp_Index_M                        bit        tempDisplayTemp.1   ; 1 = This motor, 0 = Right motor
	Temp_Index_C                        bit        tempDisplayTemp.4   ; 1 = This controller, 0 = Right controller

airFlowInPWM					    alias      user53						; (0-32767) PWM output for air inlet fans
airFlowOutPWM					    alias      user54						; (0-32767) PWM output for air outlet fans


; States
stateGearChange                     alias      user60     ; State of changing gear
stateDNR                            alias      user61     ; DNR active State
rcvKeySwitch                        alias      user62     ; position of the Key (On-Off)
keySwitch                           alias      user63     ; Internal state of the switch



test                                alias      user70
    test_bit0                           bit        test.1
    

    
rcvRiMcFaultSystem                  alias     user80






;------------- CAN MAILBOXES --------------

; Messages from and to slave controller
MAILBOX_SM_MISO1						alias CAN1
MAILBOX_SM_MISO1_Received				alias CAN1_received
MAILBOX_SM_MISO1_addr                   constant 0x111

MAILBOX_SM_MOSI1						alias CAN2
MAILBOX_SM_MOSI1_addr                   constant 0x101

MAILBOX_SM_MOSI2						alias CAN3
MAILBOX_SM_MOSI2_addr                   constant 0x102

MAILBOX_SM_MOSI3						alias CAN4
MAILBOX_SM_MOSI3_addr                   constant 0x100

MAILBOX_SM_MISO2						alias CAN5
MAILBOX_SM_MISO2_Received				alias CAN5_received
MAILBOX_SM_MISO2_addr                   constant 0x110


MAILBOX_RDW_RCV							alias CAN11
MAILBOX_RDW_RCV_Received				alias CAN11_received
MAILBOX_RDW_RCV_addr                    constant 0x012

MAILBOX_RDW_XMT							alias CAN12
MAILBOX_RDW_XMT_addr                    constant 0x200

MAILBOX_DRVSEN_RCV						alias CAN13
MAILBOX_DRVSEN_RCV_Received				alias CAN13_received
MAILBOX_DRVSEN_RCV_addr                 constant 0x011

MAILBOX_BRK_RCV							alias CAN14
MAILBOX_BRK_RCV_Received				alias CAN14_received
MAILBOX_BRK_RCV_addr                    constant 0x010


; Battery Messages
MAILBOX_BATT_RCV_InfoA					alias CAN15
MAILBOX_BATT_RCV_InfoA_received			alias CAN15_received
MAILBOX_BATT_RCV_InfoA_addr             constant 0x17A

MAILBOX_BATT_RCV_InfoC					alias CAN16
MAILBOX_BATT_RCV_InfoC_received			alias CAN16_received
MAILBOX_BATT_RCV_InfoC_addr             constant 0x17C


; Error CAN Messages
MAILBOX_ERROR_MESSAGES                  alias CAN17
MAILBOX_ERROR_MESSAGES_addr             constant 0x000

MAILBOX_ERROR_MESSAGES_RCV_ACK              alias CAN18
MAILBOX_ERROR_MESSAGES_RCV_ACK_received     alias CAN18_received
MAILBOX_ERROR_MESSAGES_RCV_ACK_addr         constant 0x008

MAILBOX_ERROR_MESSAGES_RCV_RM           alias CAN19
MAILBOX_ERROR_MESSAGES_RCV_RM_received  alias CAN19_received
MAILBOX_ERROR_MESSAGES_RCV_RM_addr      constant 0x001

MAILBOX_ERROR_MESSAGES_XMT_ACK_RM       alias CAN20
MAILBOX_ERROR_MESSAGES_XMT_ACK_RM_addr  constant 0x002

MAILBOX_MISO_REQUEST_PARAM              alias CAN21
MAILBOX_MISO_REQUEST_PARAM_received     alias CAN21_received
MAILBOX_MISO_REQUEST_PARAM_addr         constant 0x112

MAILBOX_MOSI_INIT_PARAM                alias CAN22
MAILBOX_MOSI_INIT_PARAM_addr           constant 0x103


;------------------- Maps --------------------
gear16Map                              alias MAP1
gear118Map                             alias MAP2
angle2MultiplierMap                    alias MAP3
powerDissipContrMap                    alias MAP4

;----------- User Defined Faults ------------

FaultSystem                 alias      UserFault1
    regenCritError                      bit        FaultSystem.1             ; (1, Code 51) Critical Error occured with Regen
    driveCritError                      bit        FaultSystem.2             ; (2, Code 52) Critical Error occured with Driving
    tempCritError                       bit        FaultSystem.4             ; (3, Code 53) Critical Error occured related to temperature
    regenFault                          bit        FaultSystem.8             ; (4, Code 54) Some fault occured with Regen
    driveFault                          bit        FaultSystem.16            ; (5, Code 55) Some fault occured with Driving
    tempFault                           bit        FaultSystem.32            ; (6, Code 56) Some fault occured related to temperature
    generalFault                        bit        FaultSystem.64            ; (7, Code 57) Some fault occured generally
    generalCritError                    bit        FaultSystem.128           ; (8, Code 58) Critical Error occured generally
    
User_Fault_Action_01 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
User_Fault_Action_02 = 1101100000100000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off, Full brake
User_Fault_Action_03 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off
User_Fault_Action_04 = 0000000000000000b
User_Fault_Action_05 = 0000000000000000b
User_Fault_Action_06 = 0000000000000000b
User_Fault_Action_07 = 0000000000000000b
User_Fault_Action_08 = 1101100000000000b            ; Shutdown motor, shut down main contactor, Set Throttle command to 0, Set interlock off

riMcFaultSystem                 alias      UserFault2
    riMcRegenCritError                  bit        riMcFaultSystem.1             ; (9, Code 59) Critical Error occured with Regen
    riMcDriveCritError                  bit        riMcFaultSystem.2             ; (10, Code 60) Critical Error occured with Driving
    riMcTempCritError                   bit        riMcFaultSystem.4             ; (11, Code 61) Critical Error occured related to temperature
    riMcRegenFault                      bit        riMcFaultSystem.8             ; (12, Code 62) Some fault occured with Regen
    riMcDriveFault                      bit        riMcFaultSystem.16            ; (13, Code 63) Some fault occured with Driving
    riMcTempFault                       bit        riMcFaultSystem.32            ; (14, Code 64) Some fault occured related to temperature
    riMcGeneralFault                    bit        riMcFaultSystem.64            ; (15, Code 65) Some fault occured generally
    riMcGeneralCritError                bit        riMcFaultSystem.128           ; (16, Code 66) Critical Error occured generally
    
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
;Main contactor     alias PWM1					;Pin J1-06
; EM Brake			alias PWM2					;Pin J1-05
fanIn1				alias PWM3					;Pin J1-04
fanOut2				alias PWM4					;Pin J1-03
;Free				alias PWM5					;Pin J1-02

SMESH_LEFT_PIN			    alias DigOut6				;Pin J1-19 (16 OFF(clear); 118 ON(set))
SMESH_LEFT_PIN_OUTPUT	    alias Dig6_output
SMESH_RIGHT_PIN   		    alias DigOut7				;Pin J1-20
SMESH_RIGHT_PIN_OUTPUT      alias Dig7_output

;--------- Declaration Section ----------

; none

;--------------- DELAYS -----------------
startupDLY                      alias DLY1             
startupDLY_output               alias DLY1_output
smeshDLY                        alias DLY2
smeshDLY_output				    alias DLY2_output
gearChangeProtectDLY			alias DLY3
gearChangeProtectDLY_output     alias DLY3_output
emergencyACKDLY                 alias DLY4
emergencyACKDLY_output          alias DLY4_output
lowPrioMailboxDLY               alias DLY5
lowPrioMailboxDLY_output        alias DLY5_output
startStopDLY                    alias DLY6
startStopDLY_output             alias DLY6_output
lowPrioLoopDLY                  alias DLY7
lowPrioLoopDLY_output           alias DLY7_output
gearChangeDLY                   alias DLY8
gearChangeDLY_output            alias DLY8_output
communicationSlaveErrorDLY          alias DLY9
communicationSlaveErrorDLY_ouput    alias DLY9_output

regenFaultRMP                   alias RMP1
regenFaultRMP_output            alias RMP1_output
regenFaultRMP_hold              alias RMP1_hold

driveFaultRMP                   alias RMP2
driveFaultRMP_output            alias RMP2_output
driveFaultRMP_hold              alias RMP2_hold




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

; setup inputs
setup_switches(DEBOUNCE_INPUTS)

; Ramp
setup_ramp(regenFaultRMP, 0, 0, 1)         ; RMP#, Initial value = 0, Move, rate 1/ms
set_ramp_target(regenFaultRMP, 0)          ; Target init is 0ms

setup_ramp(driveFaultRMP, 0, 0, 1)         ; RMP#, Initial value = 0, Move, rate 1/ms
set_ramp_target(driveFaultRMP, 0)          ; Target init is 0ms

;Initiate sytems
call startupCANSystem 		;setup and start the CAN communications system
call setup2DMap

setup_delay(communicationSlaveErrorDLY, CAN_NOTHING_RECEIVE_INIT)

MAX_STEER_COMPENSATION = INIT_STEER_COMPENSATION            ; 120/255 = 47% reduction on inner wheel with max steering

XMT_BATT_DRIVE_PWR_LIM_INIT = BATT_DRIVE_PWR_LIM_INIT

battDrivePowerLim = BATT_DRIVE_PWR_LIM_INIT
battRegenPowerLim = BATT_REGEN_PWR_LIM_INIT
Battery_Power_Limit = BATT_DRIVE_PWR_LIM_INIT

; For testing purposes
rcvSteerangle = 125
;rcvKeySwitch = 1
;rcvThrottle = 0
;rcvDNRCommand = 1







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


mainLoop:
    
    if (resetControllerRemote = RESET_PASSWORD) {
        reset_controller()
    }
    
    ;test = 1
    
    call faultHandling
    
    ;test = 2
    
    call checkCANMailboxes
    
    ;test = 3
    
    call DNRStateMachine
    
    if (lowPrioLoopDLY_output = 0) {
        
        ;test = 4
        
        call controlFans
        
        ;test = 5
    
        call calculateEfficiency
        
        setup_delay(lowPrioLoopDLY, LOW_PRIO_LOOP_RATE)
    }
    
    test = test +1
    
    
    goto mainLoop 



    





    
;****************************************************************
;							SUBROUTINES
;****************************************************************

; As with any programming language, the use of subroutines can allow
; easier re-use of code across multiple parts of a program, and across
; programs.  Function specific subroutines can also improve the
; Readability of the code in the main loop.


startupCANSystem:
    
    ;TODO: 0x410 send to toradex: temperature motor (-50 - 205), controller, Battery, Batt volt, batt current (2 byte)
    ; 0x411: Range (0-255 km), Torque per axis (0-255 Nm)

   			; CAN mailboxes 1-5 are available to VCL CAN. Mailboxes 6-14 are available for CANopen
   			; C_SYNC is buddy check, C_CYCLIC is cyclic CAN messages, C_EVENT are called with send_mailbox()
   			; Set Message T ype and Master ID to 0, and put Slave ID to pre-defined 11bit identifier.
   			;

    suppress_CANopen_init = 0			;first undo suppress, then startup CAN, then disable CANopen
    disable_CANopen_pdo()				; disables OS PDO mapping and frees 

    setup_CAN(CAN_BAUD_RATE, 0, 0, -1, 0)		;(Baud, Sync, Reserved[0], Slave ID, Restart)
												;Baudrate = 500KB/s setting, no Sync, Not Used, Not Used, Auto Restart



   			; MAILBOX 1
   			; Purpose:		Receive information: Torque, speed, Temperature motor/controller, Smesh enabled
   			; Type:			PDO MISO1
   			; Partner:		Slave Motorcontroller

    setup_mailbox(MAILBOX_SM_MISO1, 0, 0, MAILBOX_SM_MISO1_addr, C_EVENT, C_RCV, 0, 0)

    setup_mailbox_data(MAILBOX_SM_MISO1, 6,			
        @rcvSystemAction,				
        @rcvSystemAction + USEHB,
        @riMcMotorTempDisplay,			; Motor temperature 0-255°C
        @riMcContrTempDisplay,     ; Controller temperature  0-255°C
        @rcvStateGearChange,				    ; Smesh enabled
        @rcvRiMcFaultSystem,
        0,
        0)
        
        
        
        
            ; MAILBOX 2
   			; Purpose:		Send information: Torque, Smesh change gear, max speed, regen, commands
   			; Type:			PDO MOSI1
   			; Partner:		Slave Motorcontroller

    setup_mailbox(MAILBOX_SM_MOSI1, 0, 0, MAILBOX_SM_MOSI1_addr, C_CYCLIC, C_XMT, 0, 0)

    setup_mailbox_data(MAILBOX_SM_MOSI1, 8,
        @riMcThrottleComp,			; Torque for right motorcontroller
        @riMcThrottleComp + USEHB, 
        @stateGearChange,			        ; Command to change gear
        @riMcBattDrivePowerLim,						; Set max speed
        @riMcBattDrivePowerLim + USEHB,
        @riMcBattRegenPowerLim,					; Set Regen limit
        @riMcBattRegenPowerLim + USEHB, 
        @DNRCommand)
		
		


   			; MAILBOX 3
   			; Purpose:		Send information: 
   			; Type:			PDO MOSI2
   			; Partner:		Slave Motorcontroller
            
    setup_mailbox(MAILBOX_SM_MOSI2, 0, 0, MAILBOX_SM_MOSI2_addr, C_CYCLIC, C_XMT, 0, 0)
    setup_mailbox_data(MAILBOX_SM_MOSI2, 2,
        @rcvBrake,
        @interlockXMT,
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
            
    setup_mailbox(MAILBOX_SM_MOSI3, 0, 0, MAILBOX_SM_MOSI3_addr, C_EVENT, C_XMT, 0, 0)

    setup_mailbox_data(MAILBOX_SM_MOSI3, 4, 		
        @riMcThrottleComp,			
        @riMcThrottleComp + USEHB,
        @stateGearChange,				
        @resetControllerRemote, 
        0,
		0,
        0,
        0)
            
            ; MAILBOX 5
   			; Purpose:		Receive information:
   			; Type:			PDO MISO2
   			; Partner:		Slave Motorcontroller
         
    setup_mailbox(MAILBOX_SM_MISO2, 0, 0, MAILBOX_SM_MISO2_addr, C_EVENT, C_RCV, 0, 0)

    setup_mailbox_data(MAILBOX_SM_MISO2, 1, 		
        @rcvStateGearChange,			; Motor torque
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

    setup_mailbox(MAILBOX_RDW_RCV, 0, 0, MAILBOX_RDW_RCV_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_RDW_RCV, 3, 			; DNR switch state
        @rcvDNRCommand,
		@rcvKeySwitch,
		@rcvSteerangle,					; rcvSteerangle
        0, 
		0,
		0,
		0,
		0)


   			; MAILBOX 12
   			; Purpose:		Send information: Speed, temperature motor & controller, efficiency
   			; Type:			PDO2
   			; Partner:		RDW Scherm

    setup_mailbox(MAILBOX_RDW_XMT, 0, 0, MAILBOX_RDW_XMT_addr, C_CYCLIC, C_XMT, 0, 0)
    setup_mailbox_data(MAILBOX_RDW_XMT, 5,
        @Vehicle_Speed,				; Speed of the wheels
        @leMcEfficiencyPerc,
        @powerInDisplay,            ; TODO: send verbruik 0-150 1 decimal kW
        @stateOfCharge,
        @stateDNR,			        ; Temperature errors from Batteries
        0,		                    ; Index which motor, controller and battery is the hottest (M-C)
        0,                          ; Regen, Eco or power region
		0)


   			; MAILBOX 13
   			; Purpose:		receive information: rcvSteerangle, throttle, brake
   			; Type:			PDO3
   			; Partner:		Interieur Verlichting?

    setup_mailbox(MAILBOX_DRVSEN_RCV, 0, 0, MAILBOX_DRVSEN_RCV_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_DRVSEN_RCV, 1, 		; Efficiency
        @rcvThrottle,					; Throttle pedal
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

    setup_mailbox(MAILBOX_BRK_RCV, 0, 0, MAILBOX_BRK_RCV_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_BRK_RCV, 1, 		; Brake pedal state input
        @rcvBrake,
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

    setup_mailbox(MAILBOX_BATT_RCV_InfoA, 0, 0, MAILBOX_BATT_RCV_InfoA_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_BATT_RCV_InfoA, 8, 		
        @battSerial1,					; Serial number Battery
        @battSerial1 + USEHB,
        @battSerial2, 
        @battSerial2 + USEHB,
        @stateOfCharge,				; Percentage of the capacity 
        @BMSState,						; State of BMS: 1-4 init, 5 idle, 6 discharging, 7 charging, 10 fault, 11 critical error, 99 prepare deepsleep, 100 deepsleep
        @packCapacity, 				; Capacity remaining in pack, 0 - 65535 mAh
        @packCapacity + USEHB)
		
		   	
			; MAILBOX 16
   			; Purpose:		receive information: Error messages of temperature, current flow, voltage
   			; Type:			PDO6
   			; Partner:		Batteries

    setup_mailbox(MAILBOX_BATT_RCV_InfoC, 0, 0, MAILBOX_BATT_RCV_InfoC_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_BATT_RCV_InfoC, 8, 		
        @battSerial1,					; Serial number Battery
        @battSerial1 + USEHB,
        @battSerial2, 
        @battSerial2 + USEHB,
        @battErrorMsg1, 				; Fault code bits
        @battErrorMsg2,
        @battErrorMsg3, 
        @battErrorMsg4)
        
        
            ; MAILBOX 17
   			; Purpose:		Send information: Error messages of temperature, current flow, voltage
   			; Type:			PDO6
   			; Partner:		RDW Scherm

    setup_mailbox(MAILBOX_ERROR_MESSAGES, 0, 0, MAILBOX_ERROR_MESSAGES_addr, C_EVENT, C_XMT, 0, 0)
    setup_mailbox_data(MAILBOX_ERROR_MESSAGES, 3, 		
        @FaultSystemDisplay,               ; TODO: 0,1,2 (good, worse bad)
        @FaultSystem,
        @riMcFaultSystem,
        0,
        0,
        0,
        0,
        0)
        
            ; MAILBOX 18
   			; Purpose:		receive information: Error messages of temperature, current flow, voltage
   			; Type:			PDO6
   			; Partner:		RDW Scherm

    setup_mailbox(MAILBOX_ERROR_MESSAGES_RCV_ACK, 0, 0, MAILBOX_ERROR_MESSAGES_RCV_ACK_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_ERROR_MESSAGES_RCV_ACK, 2, 		
        @rcvFaultSystemACK,
        @rcvRiMcFaultSystemACK,
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

    setup_mailbox(MAILBOX_ERROR_MESSAGES_RCV_RM, 0, 0, MAILBOX_ERROR_MESSAGES_RCV_RM_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_ERROR_MESSAGES_RCV_RM, 3, 		
        @rcvRiMcFaultSystem,
        @rcvSystemAction,				
        @rcvSystemAction + USEHB, 
        0,
        0,
        0,
        0,
        0)
        
        
            ; MAILBOX 20
   			; Purpose:		send information: ACK to Error messages from slave controller
   			; Type:			PDO6
   			; Partner:		Slave controller

    setup_mailbox(MAILBOX_ERROR_MESSAGES_XMT_ACK_RM, 0, 0, MAILBOX_ERROR_MESSAGES_XMT_ACK_RM_addr, C_EVENT, C_XMT, 0, 0)
    setup_mailbox_data(MAILBOX_ERROR_MESSAGES_XMT_ACK_RM, 3, 		
        @riMcFaultSystemACK,
        @riMcSystemActionACK,				
        @riMcSystemActionACK + USEHB, 
        0,
        0,
        0,
        0,
        0)
        
        
            ; MAILBOX 21
   			; Purpose:		receive information: request for init parameters
   			; Type:			MISO
   			; Partner:		Slave controller

    setup_mailbox(MAILBOX_MISO_REQUEST_PARAM, 0, 0, MAILBOX_MISO_REQUEST_PARAM_addr, C_EVENT, C_RCV, 0, 0)
    setup_mailbox_data(MAILBOX_MISO_REQUEST_PARAM, 1, 		
        @rcvRequestSlaveParam,
        0,
        0,
        0,
        0,
        0,
        0,
        0)
        
            ; MAILBOX 22
   			; Purpose:		send information: Parameters at Init
   			; Type:			MOSI
   			; Partner:		Slave controller

    setup_mailbox(MAILBOX_MOSI_INIT_PARAM, 0, 0, MAILBOX_MOSI_INIT_PARAM_addr, C_EVENT, C_XMT, 0, 0)
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


CheckCANMailboxes:

    ;send low priority mailboxes
    
    if (lowPrioMailboxDLY_output = 0) {
        send_mailbox(MAILBOX_ERROR_MESSAGES)
        
        setup_delay(lowPrioMailboxDLY, CAN_LOWPRIORITY_RATE)
    }
    
    if (MAILBOX_ERROR_MESSAGES_RCV_RM_received = ON) {
        MAILBOX_ERROR_MESSAGES_RCV_RM_received = OFF
        
        riMcFaultSystemACK = rcvRiMcFaultSystem
        riMcSystemActionACK = rcvSystemAction
        
        send_mailbox(MAILBOX_ERROR_MESSAGES_XMT_ACK_RM)
        
    }
    
    interlockXMT = Interlock_State
    
    
    ; Request from slave to send parameters
    if (MAILBOX_MISO_REQUEST_PARAM_received = ON) {
        MAILBOX_MISO_REQUEST_PARAM_received = OFF
        
        if (rcvRequestSlaveParam = 1) {
            
            Slave_Param_Var1 = Max_Speed_TrqM
            Slave_Param_Var2 = Accel_Rate_TrqM
            Slave_Param_Var3 = Accel_Release_Rate_TrqM
            Slave_Param_Var4 = Brake_Rate_TrqM
        
            send_mailbox(MAILBOX_MOSI_INIT_PARAM)
            
        } else if (rcvRequestSlaveParam = 2) {
            
            Slave_Param_Var1 = Brake_Release_Rate_TrqM
            Slave_Param_Var2 = Neutral_Braking_TrqM
            Slave_Param_Var3 = XMT_BATT_DRIVE_PWR_LIM_INIT
            Slave_Param_Var4 = 0
        
            send_mailbox(MAILBOX_MOSI_INIT_PARAM)
        }
    }
    
    
    ; When nothing received from Slave, create error
    
    if (MAILBOX_SM_MISO1_Received = ON) {
        MAILBOX_SM_MISO1_Received = OFF
        
        calculationTemp1 = CAN_CYCLIC_RATE * 4 * MULTIPLIER_CAN_NOTHING_RECEIVE
        setup_delay(communicationSlaveErrorDLY, calculationTemp1)
        
    }
    
    ; TODO: vehicle_speed int 1 byte
    

    return
    
    


    
retrieveErrors:

    if ( (battTempAlarm = OFF) ) {
        tempCritError = OFF 
    } else {
        tempCritError = ON
    }
    
	if ( (battChargeTempHigh = OFF) & (battDischargeTempHigh = OFF) ) {
        tempFault = OFF  
    } else {
        tempFault = ON
    }
    
	if ( (battChargeCurAlarm = OFF) & (battOvervoltageAlarm = OFF) ) {
        regenCritError = OFF
    } else {
        regenCritError = ON
    }
    
	if ( (battCMCellOvervoltage = OFF) & (battChargeTempLow = OFF) ) {
        regenFault = OFF 
    } else {
        regenFault = ON
    }
    
	if ( (battDischargeCurAlarm = OFF) & (battDischargeUndervoltage = OFF) ) {
        driveCritError = OFF
    } else {
        driveCritError = ON
    }
    
	if ( (battCMCellUndervoltage = OFF) & (battDischargeTempLow = OFF) ) {
        driveFault = OFF 
    } else {
        driveFault = ON
    }
    
	if ( (battCMAlert = OFF) & (battPowerStageError = OFF) & (communicationSlaveErrorDLY_ouput <> 0) & (BMSState <> 11) ) {
        generalCritError = OFF
    } else {
        generalCritError = ON
    }
    
	if ( (battEEPROMError = OFF) & (battCMCRC = OFF) & (battCMFault = OFF) & (battLeakCurrent = OFF) & (BMSState <> 10) ) {
        generalFault = OFF
    } else {
        generalFault = ON
    } 
    
    
    if (rcvSystemAction <> 0) {
        FaultSystemDisplay = 3
        
        System_Action = rcvSystemAction
    } else if (System_Action <> 0) {
        FaultSystemDisplay = 3
    }
    
    riMcFaultSystem = rcvRiMcFaultSystem

    return
    
    

    
    
    
faultHandling:
    
    ; When the errors are dissappeared, clear the errors

    call retrieveErrors
    
    
    if ( (regenFault = ON) | (riMcRegenFault = ON) ) {
        
        FaultSystemDisplay = 1
        
        set_ramp_target(regenFaultRMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        LOSActive = 1
        
    }
    if ( (driveFault = ON) | (riMcDriveFault = ON) ) {
        
        FaultSystemDisplay = 1
        
        set_ramp_target(driveFaultRMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        LOSActive = 1
        
    }
    if ( (tempFault = ON) | (riMcTempFault = ON) ) {
        
        FaultSystemDisplay = 1
        
        set_ramp_target(driveFaultRMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        set_ramp_target(regenFaultRMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        LOSActive = 1
        
    }
    if ( (generalFault = ON) | (riMcGeneralFault = ON) ) {
        
        FaultSystemDisplay = 1
        
        set_ramp_target(driveFaultRMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        set_ramp_target(regenFaultRMP, TIME_TO_FULL_LOS_FAULT)          ; Target is 6000ms
        
        LOSActive = 1
        
    }
    
    if ( (FaultSystem = 0) & (riMcFaultSystem = 0) & (LOSActive = 1) ) {
        ; When there are no faults, set current limits normally
        ; When gear change is busy, don't set limits normally
        
        FaultSystemDisplay = 0
        
        set_ramp_target(driveFaultRMP, 0)          ; Target is 0ms
        
        set_ramp_target(regenFaultRMP, 0)          ; Target is 0ms
        
        LOSActive = 0
    }
    
    
    ; Drive limiting
    if (driveFaultRMP_output = 0) {
        ; When there are no limitations, just use initial power limit
        battDrivePowerLim = BATT_DRIVE_PWR_LIM_INIT
    } else {
        mapOutputTemp1 = map_two_points(driveFaultRMP_output, 0, TIME_TO_FULL_LOS_FAULT, 100, REDUCED_DRIVE_INPUT_CURLIM_PERC)

        battDrivePowerLim = map_two_points(mapOutputTemp1, 0, 100, BATTERY_DRIVE_POWER_LIMIT_MIN, BATT_DRIVE_PWR_LIM_INIT)
    }
    riMcBattDrivePowerLim = battDrivePowerLim
    
    
    ; Regen limiting
    if (regenFaultRMP_output = 0) {
        battRegenPowerLim = BATT_REGEN_PWR_LIM_INIT
    } else {
        mapOutputTemp1 = map_two_points(regenFaultRMP_output, 0, TIME_TO_FULL_LOS_FAULT, 100, REDUCED_REGEN_INPUT_CURLIM_PERC)
        
        battRegenPowerLim = map_two_points(mapOutputTemp1, 0, 100, BATTERY_REGEN_POWER_LIMIT_MIN, BATT_REGEN_PWR_LIM_INIT)
    }
    riMcBattRegenPowerLim = battRegenPowerLim
    
    
    ; Control Power Limiting
    if (Regen_State = OFF) {
        ; Motors are consuming current
        Battery_Power_Limit = battDrivePowerLim
        
    } else {
        ; Motors are producing current
        Battery_Power_Limit = battRegenPowerLim
        
    }
    
    
    if ( (System_Action <> 0) | (FaultSystem <> 0) | (rcvRiMcFaultSystem <> 0) ) {
        ; There is some fault, so send to RDW scherm
        
        if ( (emergencyACKDLY_output = 0) & (rcvFaultSystemACK <> FaultSystem) & (rcvRiMcFaultSystemACK <> riMcFaultSystem) ) {
            send_mailbox(MAILBOX_ERROR_MESSAGES)
        
            setup_delay(emergencyACKDLY, CAN_EMERGENCY_DELAY_ACK)
        }

    } else {
        rcvFaultSystemACK = 0
        rcvRiMcFaultSystemACK = 0
    }
    
    return

    
    
 
setup2DMap:
    
    ; When more torque is required at low speeds, change gear to 1:18
    setup_map(gear16Map, 6,      ; Number of points
        0, 18,            ; At     0 rpm, 18Nm /50
     1500, 18,            ; At  1500 rpm, 18Nm /50
     1800, 21,            ; At  1800 rpm, 20.5Nm /50
     1950, 27,            ; At  1950 rpm, 26.5Nm /50
     2000, 38,            ; At  2000 rpm, 38Nm /50
     2050, 500,            ; From 2000 rpm gear should always stay in 1:6
        0, 0)
    
    ; When torque is not needed or speed is too high, change gear to 1:6
    setup_map(gear118Map, 5,      ; Number of points
        0, 4,             ; At     0 rpm, 4Nm /50
     4500, 4,             ; At  4500 rpm, 4Nm /50
     5400, 5,             ; At  5400 rpm, 4.8Nm /50
     6400, 9,             ; At  6400 rpm, 8.8Nm /50
     6450, 500,            ; From 6450 rpm gear should always go to 1:6
        0, 0,
        0, 0)
        
    ; Steering angle compensation
    setup_map(angle2MultiplierMap, 3,      ; Number of points
        0, MAX_STEER_COMPENSATION,      ; Steering to the left
      125, 255,                         ; 
      250, MAX_STEER_COMPENSATION,      ; Steering to the right
        0, 0,
        0, 0,
        0, 0,
        0, 0)
        
    ; Power dissipation of the controller [W]
    setup_map(powerDissipContrMap, 7,      ; Number of points
        0, 0,      ; Steering to the left
       75, 41,                         ; 
      100, 71,      ; Steering to the right
      125, 109,
      150, 155,
      175, 210,
      250, 423)
    
    return

    
    
    
controlFans:
    
    motorTempDisplay = map_two_points(motor_temperature, 0, 2550, 0, 255)
    
    if (motorTempDisplay > riMcMotorTempDisplay) {
        motorTempDisplayHighest = motorTempDisplay
        Temp_Index_M = ON
    } else {
        motorTempDisplayHighest = riMcMotorTempDisplay
        Temp_Index_M = OFF
    }
    
    ; Fans for motor should run at 100% at 85 C, 95 is HOT, 115 MAX
    if (motorTempDisplayHighest > MOTOR_COOLDOWN_THOLD) {
        ; When motor is too hot (above threshold), turn on fans, make them spin faster when temperature rises
        
        motorFanPerc = map_two_points(motorTempDisplayHighest, MOTOR_COOLDOWN_THOLD, MOTOR_TEMP_FAN_MAX, 10, 100)
    } else if (motorTempDisplayHighest < MOTOR_COOLDOWN_THOLD - FAN_TEMPERATURE_HYSTER) {
        ; When motor temperature is below hysteresis, set fan to 0%
        
        motorFanPerc = 0
    } ; When temperature is inside hystereses, value will stay the same as last set
    
    
    
    contrTempDisplay = map_two_points(controller_temperature, 0, 2550, 0, 255)
    
    if (contrTempDisplay > riMcContrTempDisplay) {
        contrTempDisplayHighest = contrTempDisplay
        Temp_Index_C = ON
    } else {
        contrTempDisplayHighest = riMcContrTempDisplay
        Temp_Index_C = OFF
    }
    
    ; Fans for controller should run at 100% at 75 C, 95 C is MAX
    if (contrTempDisplayHighest > CONTROLLER_COOLDOWN_THOLD) {
        ; When controller is too hot (above threshold), turn on fans, make them spin faster when temperature rises
        
        contrFanPerc = map_two_points(contrTempDisplayHighest, CONTROLLER_COOLDOWN_THOLD, CONTR_TEMP_FAN_MAX, 10, 100)
    } else if (contrTempDisplayHighest < CONTROLLER_COOLDOWN_THOLD - FAN_TEMPERATURE_HYSTER) {
        ; When controller temperature is below hysteresis, set fan to 0%
        
        contrFanPerc = 0
    } ; When temperature is inside hystereses, value will stay the same as last set
    
    
    if (motorFanPerc > contrFanPerc) {
        fanPercHighest = motorFanPerc
    } else {
        fanPercHighest = contrFanPerc
    }
    
    if (fanPercHighest = 0) {
        airFlowInPWM = map_two_points(FANSPEED_IDLE_IN, 0, 100, 0, 32767)
        airFlowOutPWM = map_two_points(FANSPEED_IDLE_OUT, 0, 100, 0, 32767)
    } else {
        airFlowInPWM = map_two_points(fanPercHighest, 0, 100, 0, 32767)
        airFlowOutPWM = map_two_points(fanPercHighest, 0, 100, 0, 32767)
    }
    
    put_PWM(fanIn1, airFlowInPWM)
	put_PWM(fanOut2, airFlowOutPWM)
    
    
    return
    
    
    
    
    
calculateEfficiency:

    inputVoltage = map_two_points(Capacitor_Voltage, 0, 12800, 0, 2000)          ; 1 decimal
    outputVoltage = map_two_points(Modulation_Depth, 0, 1182, 0, inputVoltage)        ; 1 decimal
    powerOut = get_muldiv(MTD1, outputVoltage, Current_RMS, 100)                ; [W]
    
    powerDissipContr = get_map_output(powerDissipContrMap, Current_RMS)     ; [W]
    
    powerIn = powerOut + powerDissipContr
    
    motorRads = map_two_points(ABS_Motor_RPM, 0, 12000, 0, 1257)
    powerMech = get_muldiv(MTD2, Motor_Torque, motorRads, 10)               ; Motor_Torque 1 decimal [Nm] ; Motor_RPM -12000-12000rpm (-12000-12000) ; [W]
    
    leMcEfficiencyPerc = get_muldiv(MTD1, powerMech, 100, powerIn)
    
    powerInDisplay = map_two_points(powerIn, 0, 25500, 0, 255)

    return
    

    
DNRStateMachine:

    ; STATE MACHINE
    ; 52Hz
    
    if ((keySwitch = 1)) {
        ; Turn car 'on'
        
        if ((rcvKeySwitch = 0)) {
            ; Key switch state has changed, so turn off car
            clear_interlock()
            
            keySwitch = rcvKeySwitch
            startStopInit = 0
            startStopActive = 0
        } else if ((Motor_RPM < IDLE_RPM_THRESHOLD) & (rcvDNRCommand = 0) & (stateGearChange < 0x60) & (rcvThrottle < IDLE_THROTTLE_THRESHOLD) ) {
            ; Car is standing still and there is no change in DNR state
            
            if ((startStopInit = 0)) {
                setup_delay(startStopDLY, TIME_TO_START_STOP)
                startStopInit = 1
            } else if ((startStopDLY_output = 0) & (startStopActive <> 1) ) {
                ; timer is over, so engage Start Stop
                clear_interlock()
                startStopActive = 1
            }
        } else {
            startStopInit = 0
            startStopActive = 0
            if ((Interlock_State = OFF)) {
                set_interlock()
            }
            
        }
        
        if ((keySwitchHardResetComplete <> 0)) {
            keySwitchHardResetComplete = 0
        }
        
        
        
    } else {
        
        if ((rcvKeySwitch = 1)) {
            ; Key switch state has changed, so turn on car
            set_interlock()
            stateDNR = PRE_NEUTRAL
            
            keySwitch = rcvKeySwitch
        } else if ((Interlock_State = ON)) {
            ; When interlock is unexpected turned ON, turn car 'off'
            clear_interlock()
        }
        
        
        if ((System_Action <> 0) & (keySwitchHardResetComplete = 0)) {
            ; There is some fault, so reset controller at turning off car
            keySwitchHardResetComplete = 1
            resetControllerRemote = RESET_PASSWORD
            
            send_mailbox(MAILBOX_SM_MOSI3)
        }
        
        if (rcvDNRCommand <> 0) {
            rcvDNRCommand = 0
        }
        stateDNR = NEUTRAL
    }
    
    
    if ( (stateGearChange >= 0x60) & (stateGearChange < 0x6D) ) {
        
        call setSmeshTo16Simple
        
        if (stateGearChange = 0x6D) {
            ; Gear change is finished
            faultCNTGearChange = 0
            stateGearChange = 0x01
            
            if (stateDNR = DRIVE118) {
                stateDNR= DRIVE16
            }
        } else if (stateGearChange = 0xFF) {
            faultCNTGearChange = faultCNTGearChange + 1
            stateGearChange = 0x60
            
        }
        
        
    } else if ( (stateGearChange >= 0x80) & (stateGearChange < 0x8D) ) {
        ;; Changing gear to 1:18
        
        call setSmeshTo118Simple
        
        if (stateGearChange = 0x8D) {
            ; Gear change is finished
            faultCNTGearChange = 0
            
            
            if (stateDNR = DRIVE16) {
                stateGearChange = 0x01
                stateDNR = DRIVE118
            } else if (stateDNR = PRE_DRIVE) {
                ; Just go back to the main state
            } else if (stateDNR = PRE_REVERSE) {
                ; Just go back to the main state
            }
        } else if (stateGearChange = 0xFF) {
            faultCNTGearChange = faultCNTGearChange + 1
            stateGearChange = 0x80
            
        }
        
        if (stateDNR = PRE_DRIVE) {
            ; Not yet in Drive mode, so don't apply Throttle
            leMcThrottleTemp = 0
            riMcThrottleTemp = leMcThrottleTemp
        } else if (stateDNR = PRE_REVERSE) {
            ; Not yet in Reverse mode, so don't apply Throttle
            leMcThrottleTemp = 0
            riMcThrottleTemp = leMcThrottleTemp
        }
        
    } else if (stateDNR = NEUTRAL) {
        stateGearChange = 0x01
        
        leMcThrottleTemp = 0
        riMcThrottleTemp = leMcThrottleTemp
        
        if ( (rcvDNRCommand = DNR_DRIVE) & (keySwitch = 1) ) {          ; if received DNR command is Drive
            
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
            
            stateDNR= PRE_DRIVE
        } else if ( (rcvDNRCommand = DNR_REVERSE) & (keySwitch = 1) ) {       ; if received DNR command is Reverse
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
            
            stateDNR= PRE_REVERSE
        } else if ( (rcvDNRCommand = DNR_NEUTRAL) & (keySwitch = 1) ) {          ; if received DNR command is Neutral
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
        }
        
    
    } else if (stateDNR = DRIVE118) {      ; In 1:18 gear
    
        stateGearChange = 0x01
        
        ; Check efficiency
        ; check with map
        ; if efficency is wrong switch gear to 16
        
        if ( HPO = 0 ) {
            leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)     ; Set throttle to position of pedal
            riMcThrottleTemp = leMcThrottleTemp
        } else if ( rcvThrottle < HIGH_PEDAL_DISABLE_THRESHOLD ) {
            ; HPO is still 1 and throttle is below threshold
            HPO = 0
            leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)     ; Set throttle to position of pedal
            riMcThrottleTemp = leMcThrottleTemp
        } else {
            ; HPO is active
            leMcThrottleTemp = 0
            riMcThrottleTemp = leMcThrottleTemp
        }
        
        
        ; Check Efficiency
        mapOutputTemp1 = get_map_output(gear118Map, Motor_RPM)
        
        if ( (Motor_Torque < mapOutputTemp1) & (Vehicle_Speed > MINIMUM_SPEED_GEAR16) & (gearChangeProtectDLY_output = 0) ) {
        ;if ( (Motor_RPM > 1500) & (gearChangeProtectDLY_output = 0) ) {
            ; 1:6 is more efficient, so switch to 1:6
            
            stateGearChange = 0x60
            
        }
        
        if (rcvDNRCommand = DNR_NEUTRAL) {          ; if received DNR command is Neutral
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
            
            stateDNR= PRE_NEUTRAL
        } else if (rcvDNRCommand = DNR_REVERSE) {       ; if received DNR command is Reverse
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
            
            stateDNR= PRE_REVERSE
        } else if (rcvDNRCommand = DNR_DRIVE) {          ; if received DNR command is Drive
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
        }
        
        
        
        
    } else if (stateDNR = DRIVE16) {
        
        stateGearChange = 0x01
        
        ; Check efficiency
        ; check with map
        ; if efficency is wrong switch gear to 118
        
        if ( HPO = 0 ) {
            leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)     ; Set throttle to position of pedal
            riMcThrottleTemp = leMcThrottleTemp
        } else if ( rcvThrottle < HIGH_PEDAL_DISABLE_THRESHOLD ) {
            ; HPO is still 1 and throttle is below threshold
            HPO = 0
            leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)     ; Set throttle to position of pedal
            riMcThrottleTemp = leMcThrottleTemp
        } else {
            ; HPO is active
            leMcThrottleTemp = 0
            riMcThrottleTemp = leMcThrottleTemp
        }
        
        ; Check Efficiency
        mapOutputTemp1 = get_map_output(gear16Map, Motor_RPM)
        
        ;if ( (Motor_Torque > mapOutputTemp1) & (gearChangeProtectDLY_output = 0) ) {
        if ( (Motor_RPM < 2100) & (gearChangeProtectDLY_output = 0) ) {
            ; 1:18 is more efficient, so switch to 1:18
            
            stateGearChange = 0x80
            
        }
        
        if (rcvDNRCommand = DNR_NEUTRAL) {          ; if received DNR command is Neutral
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
            
            stateDNR= PRE_NEUTRAL
        } else if (rcvDNRCommand = DNR_REVERSE) {       ; if received DNR command is Reverse
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
            
            stateDNR = PRE_REVERSE
        } else if (rcvDNRCommand = DNR_DRIVE) {          ; if received DNR command is Drive
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
        }
        
        
        
        
    } else if (stateDNR = REVERSE) {
        
        stateGearChange = 0x01
        
        if ( HPO = 0 ) {
            mapOutputTemp1 = map_two_points(rcvThrottle, 0, 255, 0, -32767)
            leMcThrottleTemp = get_muldiv(MTD1, mapOutputTemp1, REVERSE_THROTTLE_MULTIPLIER, 128)     ; Set throttle to position of pedal
            riMcThrottleTemp = leMcThrottleTemp
        } else if ( rcvThrottle < HIGH_PEDAL_DISABLE_THRESHOLD ) {
            
            ; HPO is still 1 and throttle is below threshold
            HPO = 0
            mapOutputTemp1 = map_two_points(rcvThrottle, 0, 255, 0, -32767)
            leMcThrottleTemp = get_muldiv(MTD1, mapOutputTemp1, REVERSE_THROTTLE_MULTIPLIER, 128)     ; Set throttle to position of pedal
            riMcThrottleTemp = leMcThrottleTemp
        }
        
        if (rcvDNRCommand = DNR_NEUTRAL) {          ; if received DNR command is Neutral
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
            
            stateDNR= PRE_NEUTRAL
        } else if (rcvDNRCommand = DNR_DRIVE) {       ; if received DNR command is Drive
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
            
            stateDNR = PRE_DRIVE
        } else if (rcvDNRCommand = DNR_REVERSE) {          ; if received DNR command is Reverse
            rcvDNRCommand = 0     ; Clear Command from RDW scherm
        }
        
        
    } else if (stateDNR = PRE_DRIVE) {
        
        ; Check whether car can be put in Drive
        if (Vehicle_Speed < -MAX_SPEED_CHANGE_DNR) {
            ; Car drives too fast BACKWARDS, so put back to neutral
            
            stateDNR = PRE_NEUTRAL
        } else {
            ; set gear to 1:18 and go to drive state
            
            if (stateGearChange = 0x8D) {
                ; When gear change is not aborted, change state
                faultCNTGearChange = 0
                HPO = 1                     ; Initialize High Pedal, cleared when throttle is below threshold
                
                stateDNR= DRIVE118
                stateGearChange = 0x01
            } else {
                stateGearChange = 0x80
                call setSmeshTo118Simple
            }

        }
        
        
    } else if (stateDNR = PRE_REVERSE) {
        
        
        ; Check whether car can be put in Reverse
        if (Vehicle_Speed > MAX_SPEED_CHANGE_DNR) {
            ; Car drives too fast, so put back to neutral
            
            stateDNR = PRE_NEUTRAL
            
        } else {
            ; Set gear to 1:18 and go to reverse state
            
            if (stateGearChange = 0x8D) {
                ; When gear change is not aborted, change state
                faultCNTGearChange = 0
                HPO = 1                     ; Initialize High Pedal, cleared when throttle is below threshold
                
                stateDNR = REVERSE
                stateGearChange = 0x01
            } else {
                stateGearChange = 0x80
                call setSmeshTo118Simple
            }
        }
        
        
    } else if (stateDNR = PRE_NEUTRAL) {
        
        stateDNR = NEUTRAL
        
    }
    
    if (faultCNTGearChange > 3) {
        stateDNR = PRE_NEUTRAL
        stateGearChange = 0x01
    }
    
    
    ; Calculate Steeringangle factor
    ; reduce throttle on inner wheel, independent of speed
    
    if (rcvSteerangle > (125+STEER_ANGLE_NO_EFFECT)) {
        ; Steering to right
        riSteeringMultiplier = get_map_output(angle2MultiplierMap, rcvSteerangle)
        leSteeringMultiplier = 255
        
        riMcThrottleTemp = get_muldiv(MTD1, riMcThrottleTemp, riSteeringMultiplier, 255)
        
    } else if (rcvSteerangle < (125-STEER_ANGLE_NO_EFFECT)) {
        ; Steering to left
        leSteeringMultiplier = get_map_output(angle2MultiplierMap, rcvSteerangle)
        riSteeringMultiplier = 255
        
        leMcThrottleTemp = get_muldiv(MTD1, leMcThrottleTemp, leSteeringMultiplier, 255)
        
    }
    
    
    if (rcvBrake = 1) {
        VCL_Brake = FULL_BRAKE
        leMcThrottleTemp = 0
        riMcThrottleTemp = leMcThrottleTemp
    } else {
        VCL_Brake = 0
    }
    
    vcl_throttle = leMcThrottleTemp
    riMcThrottleComp = riMcThrottleTemp
    
    
    ; Set right DNR equal to left DNR
    if ( (stateDNR = NEUTRAL) | (stateDNR = REVERSE) | (stateDNR = DRIVE118) | (stateDNR = DRIVE16) ) {
        ; If state is defined, send to right controller
        DNRCommand = stateDNR
        
    } ; else keep previous state
    

    
    return
    
    
    
    
    
setSmeshTo16Simple:

    if ( (stateGearChange = 0x60) & (SMESH_LEFT_PIN_OUTPUT = OFF) & (SMESH_RIGHT_PIN_OUTPUT = OFF) ) {
        ; Smesh is already in 1:18 mode (16 OFF(clear); 118 ON(set))
        ; So directly go to the end of the functions
        stateGearChange = 0x6B
        
        send_mailbox(MAILBOX_SM_MOSI3)
        setup_delay(smeshDLY, CAN_DELAY_FOR_ACK)
    }
    
    if ((stateGearChange = 0x60)) {
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        leMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        stateGearChange = 0x61
        
        setup_delay(gearChangeDLY, GEAR_CHANGE_BEFORE_DELAY)
    }
    
    if (stateGearChange = 0x61) {
        
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        leMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if (gearChangeDLY_output = 0) {
            stateGearChange = 0x62
        }
        
    }
    
    if (stateGearChange = 0x62) {
        ; 16 OFF(clear); 118 ON(set) ; & (gearChangeDLY_output = 0) 
        clear_DigOut(SMESH_LEFT_PIN)                          ; Change gear
        
        setup_delay(gearChangeDLY, GEAR_CHANGE_AFTER_DELAY)
        
        stateGearChange = 0x63
    }
    
    if (stateGearChange = 0x63) {
        
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        leMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if (gearChangeDLY_output = 0) {
            stateGearChange = 0x64
        }
        
    }
    
    if ((stateGearChange = 0x64)) {
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        riMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        stateGearChange = 0x65
        
        setup_delay(gearChangeDLY, GEAR_CHANGE_BEFORE_DELAY)
    }
    
    if (stateGearChange = 0x65) {
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        riMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if (gearChangeDLY_output = 0) {
            stateGearChange = 0x66
        }
        
    }
    
    if (stateGearChange = 0x66) {
        ; 16 OFF(clear); 118 ON(set) ; & (gearChangeDLY_output = 0) 
        clear_DigOut(SMESH_RIGHT_PIN)                          ; Change gear
        
        setup_delay(gearChangeDLY, GEAR_CHANGE_AFTER_DELAY)
        
        stateGearChange = 0x67
    }
    
    if (stateGearChange = 0x67) {
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        riMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if (gearChangeDLY_output = 0) {
            stateGearChange = 0x6B
            
            send_mailbox(MAILBOX_SM_MOSI3)
            setup_delay(smeshDLY, CAN_DELAY_FOR_ACK)
        }
        
    }
    
    if (stateGearChange = 0x6B) {
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if ( (rcvStateGearChange = 0x6C) | (rcvStateGearChange = 0x6D) ) {
            ; Right controller has changed throttle
            stateGearChange = 0x6C
        } else if (smeshDLY_output = 0) {
            ; It takes too much time to change gear
            stateGearChange = 0xFF
            
        }
    }
        
    if (stateGearChange = 0x6C) {
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        Speed_to_RPM = 601          ; (G/d)*5305 ... 6/530*5305 ... One decimal
        
        ; Gear state to complete
        stateGearChange = 0x6D
        
        send_mailbox(MAILBOX_SM_MOSI3)
        
        setup_delay(gearChangeProtectDLY, PROTECTION_DELAY_GRCHANGE)
    }
    
    return

setSmeshTo118Simple:
    
    if ( (stateGearChange = 0x80) & (SMESH_LEFT_PIN_OUTPUT = ON) & (SMESH_RIGHT_PIN_OUTPUT = ON) ) {
        ; Smesh is already in 1:18 mode (16 OFF(clear); 118 ON(set))
        ; So directly go to the end of the functions
        stateGearChange = 0x8B
        
        send_mailbox(MAILBOX_SM_MOSI3)
        setup_delay(smeshDLY, CAN_DELAY_FOR_ACK)
    }
    
    if ((stateGearChange = 0x80)) {
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        leMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        stateGearChange = 0x81
        
        setup_delay(gearChangeDLY, GEAR_CHANGE_BEFORE_DELAY)
    }
    
    if (stateGearChange = 0x81) {
        
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        leMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if (gearChangeDLY_output = 0) {
            stateGearChange = 0x82
        }
        
    }
    
    if (stateGearChange = 0x82) {
        ; 16 OFF(clear); 118 ON(set) ; & (gearChangeDLY_output = 0) 
        set_DigOut(SMESH_LEFT_PIN)                          ; Change gear
        
        setup_delay(gearChangeDLY, GEAR_CHANGE_AFTER_DELAY)
        
        stateGearChange = 0x83
    }
    
    if (stateGearChange = 0x83) {
        
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        leMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if (gearChangeDLY_output = 0) {
            stateGearChange = 0x84
        }
        
    }
    
    if ((stateGearChange = 0x84)) {
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        riMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        stateGearChange = 0x85
        
        setup_delay(gearChangeDLY, GEAR_CHANGE_BEFORE_DELAY)
    }
    
    if (stateGearChange = 0x85) {
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        riMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if (gearChangeDLY_output = 0) {
            stateGearChange = 0x86
        }
        
    }
    
    if (stateGearChange = 0x86) {
        ; 16 OFF(clear); 118 ON(set) ; & (gearChangeDLY_output = 0) 
        set_DigOut(SMESH_RIGHT_PIN)                          ; Change gear
        
        setup_delay(gearChangeDLY, GEAR_CHANGE_AFTER_DELAY)
        
        stateGearChange = 0x87
    }
    
    if (stateGearChange = 0x87) {
        mapOutputTemp1 = get_muldiv(MTD1, rcvThrottle, THROTTLE_MULTIP_REDUCE, 128)
        riMcThrottleTemp = map_two_points(mapOutputTemp1, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if (gearChangeDLY_output = 0) {
            stateGearChange = 0x8B
            
            send_mailbox(MAILBOX_SM_MOSI3)
            setup_delay(smeshDLY, CAN_DELAY_FOR_ACK)
        }
        
    }
    
    if (stateGearChange = 0x8B) {
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        if ( (rcvStateGearChange = 0x8C) | (rcvStateGearChange = 0x8D) ) {
            ; Right controller has changed throttle
            stateGearChange = 0x8C
        } else if (smeshDLY_output = 0) {
            ; It takes too much time to change gear
            stateGearChange = 0xFF
            
        }
    }
        
    if (stateGearChange = 0x8C) {
        riMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        leMcThrottleTemp = map_two_points(rcvThrottle, 0, 255, 0, 32767)
        
        Speed_to_RPM = 1802          ; (G/d)*5305 ... 18/530*5305 ... One decimal
        
        ; Gear state to complete
        stateGearChange = 0x8D
        
        send_mailbox(MAILBOX_SM_MOSI3)
        
        setup_delay(gearChangeProtectDLY, PROTECTION_DELAY_GRCHANGE)
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
