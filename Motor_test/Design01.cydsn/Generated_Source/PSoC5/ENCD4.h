/*******************************************************************************
* File Name: ENCD4.h  
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

#if !defined(CY_PINS_ENCD4_H) /* Pins ENCD4_H */
#define CY_PINS_ENCD4_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "ENCD4_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 ENCD4__PORT == 15 && ((ENCD4__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    ENCD4_Write(uint8 value);
void    ENCD4_SetDriveMode(uint8 mode);
uint8   ENCD4_ReadDataReg(void);
uint8   ENCD4_Read(void);
void    ENCD4_SetInterruptMode(uint16 position, uint16 mode);
uint8   ENCD4_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the ENCD4_SetDriveMode() function.
     *  @{
     */
        #define ENCD4_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define ENCD4_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define ENCD4_DM_RES_UP          PIN_DM_RES_UP
        #define ENCD4_DM_RES_DWN         PIN_DM_RES_DWN
        #define ENCD4_DM_OD_LO           PIN_DM_OD_LO
        #define ENCD4_DM_OD_HI           PIN_DM_OD_HI
        #define ENCD4_DM_STRONG          PIN_DM_STRONG
        #define ENCD4_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define ENCD4_MASK               ENCD4__MASK
#define ENCD4_SHIFT              ENCD4__SHIFT
#define ENCD4_WIDTH              1u

/* Interrupt constants */
#if defined(ENCD4__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in ENCD4_SetInterruptMode() function.
     *  @{
     */
        #define ENCD4_INTR_NONE      (uint16)(0x0000u)
        #define ENCD4_INTR_RISING    (uint16)(0x0001u)
        #define ENCD4_INTR_FALLING   (uint16)(0x0002u)
        #define ENCD4_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define ENCD4_INTR_MASK      (0x01u) 
#endif /* (ENCD4__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define ENCD4_PS                     (* (reg8 *) ENCD4__PS)
/* Data Register */
#define ENCD4_DR                     (* (reg8 *) ENCD4__DR)
/* Port Number */
#define ENCD4_PRT_NUM                (* (reg8 *) ENCD4__PRT) 
/* Connect to Analog Globals */                                                  
#define ENCD4_AG                     (* (reg8 *) ENCD4__AG)                       
/* Analog MUX bux enable */
#define ENCD4_AMUX                   (* (reg8 *) ENCD4__AMUX) 
/* Bidirectional Enable */                                                        
#define ENCD4_BIE                    (* (reg8 *) ENCD4__BIE)
/* Bit-mask for Aliased Register Access */
#define ENCD4_BIT_MASK               (* (reg8 *) ENCD4__BIT_MASK)
/* Bypass Enable */
#define ENCD4_BYP                    (* (reg8 *) ENCD4__BYP)
/* Port wide control signals */                                                   
#define ENCD4_CTL                    (* (reg8 *) ENCD4__CTL)
/* Drive Modes */
#define ENCD4_DM0                    (* (reg8 *) ENCD4__DM0) 
#define ENCD4_DM1                    (* (reg8 *) ENCD4__DM1)
#define ENCD4_DM2                    (* (reg8 *) ENCD4__DM2) 
/* Input Buffer Disable Override */
#define ENCD4_INP_DIS                (* (reg8 *) ENCD4__INP_DIS)
/* LCD Common or Segment Drive */
#define ENCD4_LCD_COM_SEG            (* (reg8 *) ENCD4__LCD_COM_SEG)
/* Enable Segment LCD */
#define ENCD4_LCD_EN                 (* (reg8 *) ENCD4__LCD_EN)
/* Slew Rate Control */
#define ENCD4_SLW                    (* (reg8 *) ENCD4__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define ENCD4_PRTDSI__CAPS_SEL       (* (reg8 *) ENCD4__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define ENCD4_PRTDSI__DBL_SYNC_IN    (* (reg8 *) ENCD4__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define ENCD4_PRTDSI__OE_SEL0        (* (reg8 *) ENCD4__PRTDSI__OE_SEL0) 
#define ENCD4_PRTDSI__OE_SEL1        (* (reg8 *) ENCD4__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define ENCD4_PRTDSI__OUT_SEL0       (* (reg8 *) ENCD4__PRTDSI__OUT_SEL0) 
#define ENCD4_PRTDSI__OUT_SEL1       (* (reg8 *) ENCD4__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define ENCD4_PRTDSI__SYNC_OUT       (* (reg8 *) ENCD4__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(ENCD4__SIO_CFG)
    #define ENCD4_SIO_HYST_EN        (* (reg8 *) ENCD4__SIO_HYST_EN)
    #define ENCD4_SIO_REG_HIFREQ     (* (reg8 *) ENCD4__SIO_REG_HIFREQ)
    #define ENCD4_SIO_CFG            (* (reg8 *) ENCD4__SIO_CFG)
    #define ENCD4_SIO_DIFF           (* (reg8 *) ENCD4__SIO_DIFF)
#endif /* (ENCD4__SIO_CFG) */

/* Interrupt Registers */
#if defined(ENCD4__INTSTAT)
    #define ENCD4_INTSTAT            (* (reg8 *) ENCD4__INTSTAT)
    #define ENCD4_SNAP               (* (reg8 *) ENCD4__SNAP)
    
	#define ENCD4_0_INTTYPE_REG 		(* (reg8 *) ENCD4__0__INTTYPE)
#endif /* (ENCD4__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_ENCD4_H */


/* [] END OF FILE */
