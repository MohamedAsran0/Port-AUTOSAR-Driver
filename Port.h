 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Asran
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Asran's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* PORT Module Id */
#define PORT_MODULE_ID    (124U)

/* PORT Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for PORT Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and PORT Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* PORT Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for PORT Init */
#define PORT_INIT_SID                      (uint8)0x00
   
/* Service ID for PORT SetPinDirection */
#define PORT_SET_PIN_DIRECTION_SID         (uint8)0x01

/* Service ID for PORT Refresh Direction */
#define PORT_REFRESH_PORT_DIRECTION_SID    (uint8)0x02
   
/* Service ID for PORT GetVersionInfo */
#define PORT_GET_VERSION_INFO_SID          (uint8)0x03

/* Service ID for PORT SetPinMode */
#define PORT_SET_PIN_MODE_SID              (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN               (uint8)0x0A

/* DET code to report that Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE  (uint8)0x0B

/* Port_Init API service called with NULL pointer parameter */
#define PORT_E_PARAM_CONFIG            (uint8)0x0C   

/* Port_SetPinMode API service called when mode is unchangeable */
#define PORT_E_PARAM_INVALID_MODE      (uint8)0x0D   

/* Port_SetPinMode API service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE       (uint8)0x0E      
      
/* API service used without module initialization */ 
#define PORT_E_UNINIT                  (uint8)0x0F
  
/* APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER           (uint8)0x10   
   
/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Description: Tiva-c Ports */
#define PORT_PortA	(0U)
#define PORT_PortB	(1U)
#define PORT_PortC	(2U)
#define PORT_PortD	(3U)
#define PORT_PortE	(4U)
#define PORT_PortF	(5U)
   
/* Description: Tiva-c Pins */
#define PORT_Pin0       (0U)
#define PORT_Pin1       (1U)
#define PORT_Pin2       (2U)
#define PORT_Pin3       (3U)
#define PORT_Pin4       (4U)
#define PORT_Pin5       (5U)
#define PORT_Pin6       (6U)
#define PORT_Pin7       (7U)
   
/* Description: uint8 to hold PIN Number */
typedef uint8 Port_PinType;

/* Description: uint8 to hold PORT Number */
typedef uint8 Port_PortType;

/* Description: uint8 to hold PIN Mode */
typedef uint8 Port_PinModeType;

/* Description: Enum to hold PIN direction */
typedef enum
{
  PORT_PIN_IN,
  PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
  OFF,
  PULL_UP,
  PULL_DOWN
}Port_InternalResistor;

/* Description: Enum to hold the initial mode for PIN */
typedef enum
{
  PORT_PIN_MODE_DIO,
  PORT_PIN_MODE_ALTERNATIVE_1,
  PORT_PIN_MODE_ALTERNATIVE_2,
  PORT_PIN_MODE_ALTERNATIVE_3,
  PORT_PIN_MODE_ALTERNATIVE_4,
  PORT_PIN_MODE_ALTERNATIVE_5,
  PORT_PIN_MODE_ALTERNATIVE_6,
  PORT_PIN_MODE_ALTERNATIVE_7,
  PORT_PIN_MODE_ALTERNATIVE_8,
  PORT_PIN_MODE_ALTERNATIVE_9,
  PORT_PIN_MODE_ALTERNATIVE_10,
  PORT_PIN_MODE_ALTERNATIVE_11,
  PORT_PIN_MODE_ALTERNATIVE_12,
  PORT_PIN_MODE_ALTERNATIVE_13,
  PORT_PIN_MODE_ALTERNATIVE_14,
  PORT_PIN_MODE_ANALOG
}Port_PinInitialMode;

/* Description: Structure to configure each individual PIN:
*	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
*	2. the number of the pin in the PORT.
*       3. Pin mode (e.g. DIO, ADC, SPI …)
*       4. the direction of pin --> INPUT or OUTPUT
*       5. the internal resistor --> Disable, Pull up or Pull down
*       6. Pin level init value
*       7. Pin direction changeable during runtime (STD_ON/STD_OFF)
*       8. Pin mode changeable during runtime (STD_ON/STD_OFF)
*/
typedef struct 
{
  Port_PortType port_num; 
  Port_PinType pin_num;
  Port_PinModeType pin_mode;
  Port_PinDirectionType direction;
  Port_InternalResistor resistor;
  uint8 initial_value;
  uint8 pin_direction_changeable;
  uint8 pin_mode_changeable;
}Port_ConfigPin;

/* Data Structure required for initializing the Port Driver */
typedef struct 
{
  Port_ConfigPin Pins[PORT_NUMBER_OF_PINS];
}Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module.   
************************************************************************************/
void Port_Init(const Port_ConfigType *ConfigPtr );

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in):  Pin -> Port Pin ID number 
                    Direction -> Port Pin Direction 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction.
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin, 
                          Port_PinDirectionType Direction);
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in):  None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction.
************************************************************************************/
void Port_RefreshPortDirection(void);

/************************************************************************************
* Service Name:Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in):  None
* Parameters (inout): None
* Parameters (out): versioninfo -> Pointer to where to store
*                                  the version information of this module. 
* Return value: None
* Description: Returns the version information of this module. 
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in):  Pin -> Port Pin ID number 
                    Mode -> New Port Pin mode to be set on port pin. 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode( Port_PinType Pin, 
                      Port_PinModeType Mode);
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */
