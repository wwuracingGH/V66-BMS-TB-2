#pragma once
#include <stdint.h>

#define BMS_CANSTRUCT typedef struct __attribute__((packed, scalar_storage_order("little-endian")))
#define CHARGER_CANSTRUCT typedef struct __attribute__((packed, scalar_storage_order("big-endian")))
/*Elcon Charger- CAN docs:
https://www.evwest.com/support/Elcon%20CAN%20Specification%202019.pdf

Note: Elcon Charger's CAN protocol is big endian*/

typedef uint16_t    BMS_Temp;
typedef uint16_t    BMS_Voltage;
typedef uint8_t     BMS_Byte;
typedef uint16_t    Charger_Voltage;
typedef uint16_t    Charger_Current;
typedef uint8_t     Charger_Byte;
typedef uint32_t    Charger_Reserved;


#define BMS_CANID_DATA_0                  0x020
#define BMS_CANID_DATA_1                  0x021
#define BMS_CANID_DATA_2                  0x022
#define BMS_CANID_DATA_3                  0x023
#define BMS_CANID_DATA_4                  0x024
#define BMS_CANID_DATA_5                  0x025
#define BMS_CANID_DATA_6                  0x026
#define BMS_CANID_DATA_7                  0x027
#define BMS_CANID_DATA_8                  0x028
#define BMS_CANID_DATA_9                  0x029

#define BMS_CANID_CHARGER_CTRL            0x0F4
#define BMS_CANID_CHARGER_STATUS          0X0E5


#define EightByteEndianSwap(data)   \
( (((data) >> 56) & 0x00000000000000FF) | (((data) >> 40) & 0x000000000000FF00) | \
  (((data) >> 24) & 0x0000000000FF0000) | (((data) >>  8) & 0x00000000FF000000) | \
  (((data) <<  8) & 0x000000FF00000000) | (((data) << 24) & 0x0000FF0000000000) | \
  (((data) << 40) & 0x00FF000000000000) | (((data) << 56) & 0xFF00000000000000) ) 


BMS_CANSTRUCT {
	BMS_Voltage maxVolt : 11;
	BMS_Voltage minVolt : 11;
	BMS_Voltage avgVolt : 11;
	BMS_Temp    maxTemp : 10;
	BMS_Temp    minTemp : 10;
	BMS_Temp    avgTemp : 10;
	BMS_Byte    Fault   : 1;
} BMS_Status;


CHARGER_CANSTRUCT  { 
	Charger_Voltage     maxChargingVoltage;
	Charger_Current     maxChargingCurrent;
	Charger_Byte        chargerEnable :  1;
	Charger_Reserved    _RESERVED     : 31;
} Charger_Control;   /*Send at intervals of 1s*/


CHARGER_CANSTRUCT  {
	Charger_Voltage   outputVoltage;
	Charger_Current   outputCurrent;
	Charger_Byte      hardwareFailureFlag : 1;
	Charger_Byte      overtempFlag        : 1;
	Charger_Byte      inputVoltageFlag    : 1;
	Charger_Byte      reversePolarityFlag : 1;
	Charger_Byte      communicationState  : 1;
	Charger_Reserved  _RESERVED           : 27;
} Charger_Status;

