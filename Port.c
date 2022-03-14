 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Asran
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
STATIC const Port_ConfigPin *Port_ConfigPtr = NULL_PTR;

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
void Port_Init(const Port_ConfigType *ConfigPtr)
{
 #if (PORT_DEV_ERROR_DETECT == STD_ON)
      /* check if the input configuration pointer is not a NULL_PTR */
      if (NULL_PTR == ConfigPtr)
      {
              Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_CONFIG);
      }
      else
#endif
      {  
        /* point to the required Port Registers base address */
        volatile uint32 * PortGpio_Ptr = NULL_PTR;
        volatile uint32 delay = 0;
	
        /* The port is initialized */
        Port_Status = PORT_INITIALIZED;
        
        /* Points to the first structure address in the array of structures */
        Port_ConfigPtr = ConfigPtr->Pins;
        
        
        for(uint8 index=0; index < PORT_NUMBER_OF_PINS; index++)
        {   
            switch(Port_ConfigPtr[index].port_num)
            {
                case PORT_PortA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                 break;
                case PORT_PortB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                                 break;
                case PORT_PortC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                                 break;
                case PORT_PortD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                                 break;
                case PORT_PortE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                                 break;
                case PORT_PortF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                                 break;
            }
            
            /* Enable clock for PORT and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<Port_ConfigPtr[index].port_num);
            delay = SYSCTL_REGCGC2_REG;
            
            
            /* PD7 or PF0 */
            if( ((Port_ConfigPtr[index].port_num == PORT_PortD) && (Port_ConfigPtr[index].pin_num == PORT_Pin7)) 
              || ((Port_ConfigPtr[index].port_num == PORT_PortF) && (Port_ConfigPtr[index].pin_num == PORT_Pin0)) ) 
            {
                /* Unlock the GPIOCR register */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
                
                /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_ConfigPtr[index].pin_num);  
            }
            
            
            /* PC0 to PC3 */
            else if( (Port_ConfigPtr[index].port_num == PORT_PortC) && (Port_ConfigPtr[index].pin_num <= PORT_Pin3) ) 
            {
                /* Do Nothing ...  this is the JTAG pins */
                continue;
            }
            
            else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }
            
            /***************************************** Direction & Initial Values *****************************************/
            /* If it's an OUTPUT PIN */
            if(Port_ConfigPtr[index].direction == PORT_PIN_OUT)
            {
                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[index].pin_num);                
                
                /* If the initial value is high */
                if(Port_ConfigPtr[index].initial_value == STD_HIGH)
                {
                    /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_ConfigPtr[index].pin_num);          
                }
                
                /* If the initial value is low */
                else
                {
                    /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_ConfigPtr[index].pin_num);        
                }
            }
            
            /* If it's an INPUT PIN */
            else if(Port_ConfigPtr[index].direction == PORT_PIN_IN)
            {
                /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[index].pin_num);            
                
                /* If it's internal PULL_UP */
                if(Port_ConfigPtr[index].resistor == PULL_UP)
                {
                    /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_ConfigPtr[index].pin_num);       
                }
                
                /* If it's internal PULL_DOWN */
                else if(Port_ConfigPtr[index].resistor == PULL_DOWN)
                {
                    /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_ConfigPtr[index].pin_num);     
                }
                
                /* Neither PULL_UP nor PULL_DOWN */ 
                else
                {
                    /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_ConfigPtr[index].pin_num);
                    
                    /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_ConfigPtr[index].pin_num);   
                }
            }
            else
            {
                /* Do Nothing */
            }
            
            /***************************************** Modes *****************************************/
            /* DIO MODE*/
            if(Port_ConfigPtr[index].pin_mode == PORT_PIN_MODE_DIO)
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[index].pin_num);
                
                /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[index].pin_num);
                
                /* Clear the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_ConfigPtr[index].pin_num * 4));
                
                /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[index].pin_num);
                
            }
            
            /* ANALOG MODE */ 
            else if(Port_ConfigPtr[index].pin_mode == PORT_PIN_MODE_ANALOG)
            {
                /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[index].pin_num);

                /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[index].pin_num);
                
                /* Clear the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_ConfigPtr[index].pin_num * 4));
                
                /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[index].pin_num);

            }
  
            /* Any Other MODE */
            else if( (Port_ConfigPtr[index].pin_mode > PORT_PIN_MODE_DIO) && (Port_ConfigPtr[index].pin_mode < PORT_PIN_MODE_ANALOG) )
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[index].pin_num);
                
                /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[index].pin_num);
                
                /* Clear the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_ConfigPtr[index].pin_num * 4));
                
                /* Set the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (Port_ConfigPtr[index].pin_mode << (Port_ConfigPtr[index].pin_num * 4));
                
                /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[index].pin_num);
            }
            else
            {
                /* Do Nothing */
            }
        }
      }
}

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
                          Port_PinDirectionType Direction)
{
    boolean error = FALSE;
    
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
    
    /* Check if the Port Pin ID passed is correct */
    if (Pin > PORT_NUMBER_OF_PINS) 
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
    
     /* Check if the Port Pin is not configured as changeable */
    if (STD_OFF == Port_ConfigPtr[Pin].pin_direction_changeable)
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
#endif

    /* In-case there are no errors */
    if(FALSE == error)
    {
        volatile uint32 * PortGpio_Ptr = NULL_PTR;
        
        switch(Port_ConfigPtr[Pin].port_num)
        {
            case PORT_PortA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                             break;
            case PORT_PortB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                             break;
            case PORT_PortC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                             break;
            case PORT_PortD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                             break;
            case PORT_PortE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                             break;
            case PORT_PortF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                             break;
        }
      
        /* If it's an OUTPUT PIN */
        if(Direction == PORT_PIN_OUT)
        {
            /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);                
        }
        
        /* If it's an INPUT PIN */
        else if(Direction == PORT_PIN_IN)
        {
            /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);            
        }
    }
    else
    {
            /* No Action Required */
    }
}
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
void Port_RefreshPortDirection(void)
{
    boolean error = FALSE;
    
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
#endif
    
    /* In-case there are no errors */
    if(FALSE == error)
    {
        for(uint8 index=0; index<PORT_NUMBER_OF_PINS; index++)
        {
            volatile uint32 * PortGpio_Ptr = NULL_PTR;
        
            switch(Port_ConfigPtr[index].port_num)
            {
                case PORT_PortA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                 break;
                case PORT_PortB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                                 break;
                case PORT_PortC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                                 break;
                case PORT_PortD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                                 break;
                case PORT_PortE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                                 break;
                case PORT_PortF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                                 break;
            }
            
            if(Port_ConfigPtr[index].pin_direction_changeable == STD_OFF )
            {
                /* If it's an OUTPUT PIN */
                if(Port_ConfigPtr[index].direction == PORT_PIN_OUT)
                {
                    /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[index].pin_num);                
                }
                
                /* If it's an INPUT PIN */
                else if(Port_ConfigPtr[index].direction == PORT_PIN_IN)
                {
                    /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[index].pin_num);            
                }
                else
                {
                    /* Do Nothing */
                }
            }
            else
            {
                /* Do Nothing */
            }
        }
    }
    else
    {
            /* No Action Required */
    }
}

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
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{
    boolean error = FALSE;
    
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if(NULL_PTR == versioninfo)
    {
            /* Report to DET  */
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
            error = TRUE;                
    }
    else
    {
            /* No Action Required */
    }
    
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_UNINIT);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
#endif

    /* In-case there are no errors */
    if(FALSE == error)
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
    else
    {
            /* No Action Required */
    }
}
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
                      Port_PinModeType Mode)
{
    boolean error = FALSE;
    
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
    
    /* Check if the Port Pin ID passed is correct */
    if (Pin > PORT_NUMBER_OF_PINS)
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
    
    /* Check if the Port Pin MODE passed is valid */
    if (Mode > PORT_PIN_MODE_ANALOG) 
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
    
    /* Check if the Port Pin MODE is changeable */
    if (STD_OFF == Port_ConfigPtr[Pin].pin_mode_changeable)
    {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
            error = TRUE;
    }
    else
    {
            /* No Action Required */
    }
#endif

    /* In-case there are no errors */
    if(FALSE == error)
    {
        volatile uint32 * PortGpio_Ptr = NULL_PTR;
        
        switch(Port_ConfigPtr[Pin].port_num)
        {
            case PORT_PortA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                             break;
            case PORT_PortB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                             break;
            case PORT_PortC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                             break;
            case PORT_PortD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                             break;
            case PORT_PortE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                             break;
            case PORT_PortF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                             break;
        }
      
        /* DIO MODE*/
        if(Mode == PORT_PIN_MODE_DIO)
        {
            /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);
            
            /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);
            
            /* Clear the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_ConfigPtr[Pin].pin_num * 4));
            
            /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);
        }
        
        /* ANALOG MODE */ 
        else if(Mode == PORT_PIN_MODE_ANALOG)
        {
            /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);

            /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);
            
            /* Clear the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_ConfigPtr[Pin].pin_num * 4));
            
            /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);                              
        }
        
        /* Any Other MODE */
        else if( (Mode > PORT_PIN_MODE_DIO) && (Mode < PORT_PIN_MODE_ANALOG) )
        {
            /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);
            
            /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);
            
            /* Clear the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_ConfigPtr[Pin].pin_num * 4));
            
            /* Set the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (Port_ConfigPtr[Pin].pin_mode << (Port_ConfigPtr[Pin].pin_num * 4));
            
            /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);
        }
        else
        {
            /* Do Nothing */
        }
    }
    else
    {
            /* No Action Required */
    }
}
#endif
