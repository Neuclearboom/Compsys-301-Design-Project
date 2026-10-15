/*******************************************************************************
* File Name: ENCD1.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_ENCD1_H) /* Pins ENCD1_H */
#define CY_PINS_ENCD1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "ENCD1_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 ENCD1__PORT == 15 && ((ENCD1__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    ENCD1_Write(uint8 value);
void    ENCD1_SetDriveMode(uint8 mode);
uint8   ENCD1_ReadDataReg(void);
uint8   ENCD1_Read(void);
void    ENCD1_SetInterruptMode(uint16 position, uint16 mode);
uint8   ENCD1_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the ENCD1_SetDriveMode() function.
     *  @{
     */
        #define ENCD1_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define ENCD1_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define ENCD1_DM_RES_UP          PIN_DM_RES_UP
        #define ENCD1_DM_RES_DWN         PIN_DM_RES_DWN
        #define ENCD1_DM_OD_LO           PIN_DM_OD_LO
        #define ENCD1_DM_OD_HI           PIN_DM_OD_HI
        #define ENCD1_DM_STRONG          PIN_DM_STRONG
        #define ENCD1_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define ENCD1_MASK               ENCD1__MASK
#define ENCD1_SHIFT              ENCD1__SHIFT
#define ENCD1_WIDTH              1u

/* Interrupt constants */
#if defined(ENCD1__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in ENCD1_SetInterruptMode() function.
     *  @{
     */
        #define ENCD1_INTR_NONE      (uint16)(0x0000u)
        #define ENCD1_INTR_RISING    (uint16)(0x0001u)
        #define ENCD1_INTR_FALLING   (uint16)(0x0002u)
        #define ENCD1_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define ENCD1_INTR_MASK      (0x01u) 
#endif /* (ENCD1__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define ENCD1_PS                     (* (reg8 *) ENCD1__PS)
/* Data Register */
#define ENCD1_DR                     (* (reg8 *) ENCD1__DR)
/* Port Number */
#define ENCD1_PRT_NUM                (* (reg8 *) ENCD1__PRT) 
/* Connect to Analog Globals */                                                  
#define ENCD1_AG                     (* (reg8 *) ENCD1__AG)                       
/* Analog MUX bux enable */
#define ENCD1_AMUX                   (* (reg8 *) ENCD1__AMUX) 
/* Bidirectional Enable */                                                        
#define ENCD1_BIE                    (* (reg8 *) ENCD1__BIE)
/* Bit-mask for Aliased Register Access */
#define ENCD1_BIT_MASK               (* (reg8 *) ENCD1__BIT_MASK)
/* Bypass Enable */
#define ENCD1_BYP                    (* (reg8 *) ENCD1__BYP)
/* Port wide control signals */                                                   
#define ENCD1_CTL                    (* (reg8 *) ENCD1__CTL)
/* Drive Modes */
#define ENCD1_DM0                    (* (reg8 *) ENCD1__DM0) 
#define ENCD1_DM1                    (* (reg8 *) ENCD1__DM1)
#define ENCD1_DM2                    (* (reg8 *) ENCD1__DM2) 
/* Input Buffer Disable Override */
#define ENCD1_INP_DIS                (* (reg8 *) ENCD1__INP_DIS)
/* LCD Common or Segment Drive */
#define ENCD1_LCD_COM_SEG            (* (reg8 *) ENCD1__LCD_COM_SEG)
/* Enable Segment LCD */
#define ENCD1_LCD_EN                 (* (reg8 *) ENCD1__LCD_EN)
/* Slew Rate Control */
#define ENCD1_SLW                    (* (reg8 *) ENCD1__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define ENCD1_PRTDSI__CAPS_SEL       (* (reg8 *) ENCD1__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define ENCD1_PRTDSI__DBL_SYNC_IN    (* (reg8 *) ENCD1__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define ENCD1_PRTDSI__OE_SEL0        (* (reg8 *) ENCD1__PRTDSI__OE_SEL0) 
#define ENCD1_PRTDSI__OE_SEL1        (* (reg8 *) ENCD1__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define ENCD1_PRTDSI__OUT_SEL0       (* (reg8 *) ENCD1__PRTDSI__OUT_SEL0) 
#define ENCD1_PRTDSI__OUT_SEL1       (* (reg8 *) ENCD1__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define ENCD1_PRTDSI__SYNC_OUT       (* (reg8 *) ENCD1__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(ENCD1__SIO_CFG)
    #define ENCD1_SIO_HYST_EN        (* (reg8 *) ENCD1__SIO_HYST_EN)
    #define ENCD1_SIO_REG_HIFREQ     (* (reg8 *) ENCD1__SIO_REG_HIFREQ)
    #define ENCD1_SIO_CFG            (* (reg8 *) ENCD1__SIO_CFG)
    #define ENCD1_SIO_DIFF           (* (reg8 *) ENCD1__SIO_DIFF)
#endif /* (ENCD1__SIO_CFG) */

/* Interrupt Registers */
#if defined(ENCD1__INTSTAT)
    #define ENCD1_INTSTAT            (* (reg8 *) ENCD1__INTSTAT)
    #define ENCD1_SNAP               (* (reg8 *) ENCD1__SNAP)
    
	#define ENCD1_0_INTTYPE_REG 		(* (reg8 *) ENCD1__0__INTTYPE)
#endif /* (ENCD1__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_ENCD1_H */


/* [] END OF FILE */
