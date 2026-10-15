/*******************************************************************************
* File Name: ENCD3.h  
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

#if !defined(CY_PINS_ENCD3_H) /* Pins ENCD3_H */
#define CY_PINS_ENCD3_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "ENCD3_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 ENCD3__PORT == 15 && ((ENCD3__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    ENCD3_Write(uint8 value);
void    ENCD3_SetDriveMode(uint8 mode);
uint8   ENCD3_ReadDataReg(void);
uint8   ENCD3_Read(void);
void    ENCD3_SetInterruptMode(uint16 position, uint16 mode);
uint8   ENCD3_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the ENCD3_SetDriveMode() function.
     *  @{
     */
        #define ENCD3_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define ENCD3_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define ENCD3_DM_RES_UP          PIN_DM_RES_UP
        #define ENCD3_DM_RES_DWN         PIN_DM_RES_DWN
        #define ENCD3_DM_OD_LO           PIN_DM_OD_LO
        #define ENCD3_DM_OD_HI           PIN_DM_OD_HI
        #define ENCD3_DM_STRONG          PIN_DM_STRONG
        #define ENCD3_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define ENCD3_MASK               ENCD3__MASK
#define ENCD3_SHIFT              ENCD3__SHIFT
#define ENCD3_WIDTH              1u

/* Interrupt constants */
#if defined(ENCD3__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in ENCD3_SetInterruptMode() function.
     *  @{
     */
        #define ENCD3_INTR_NONE      (uint16)(0x0000u)
        #define ENCD3_INTR_RISING    (uint16)(0x0001u)
        #define ENCD3_INTR_FALLING   (uint16)(0x0002u)
        #define ENCD3_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define ENCD3_INTR_MASK      (0x01u) 
#endif /* (ENCD3__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define ENCD3_PS                     (* (reg8 *) ENCD3__PS)
/* Data Register */
#define ENCD3_DR                     (* (reg8 *) ENCD3__DR)
/* Port Number */
#define ENCD3_PRT_NUM                (* (reg8 *) ENCD3__PRT) 
/* Connect to Analog Globals */                                                  
#define ENCD3_AG                     (* (reg8 *) ENCD3__AG)                       
/* Analog MUX bux enable */
#define ENCD3_AMUX                   (* (reg8 *) ENCD3__AMUX) 
/* Bidirectional Enable */                                                        
#define ENCD3_BIE                    (* (reg8 *) ENCD3__BIE)
/* Bit-mask for Aliased Register Access */
#define ENCD3_BIT_MASK               (* (reg8 *) ENCD3__BIT_MASK)
/* Bypass Enable */
#define ENCD3_BYP                    (* (reg8 *) ENCD3__BYP)
/* Port wide control signals */                                                   
#define ENCD3_CTL                    (* (reg8 *) ENCD3__CTL)
/* Drive Modes */
#define ENCD3_DM0                    (* (reg8 *) ENCD3__DM0) 
#define ENCD3_DM1                    (* (reg8 *) ENCD3__DM1)
#define ENCD3_DM2                    (* (reg8 *) ENCD3__DM2) 
/* Input Buffer Disable Override */
#define ENCD3_INP_DIS                (* (reg8 *) ENCD3__INP_DIS)
/* LCD Common or Segment Drive */
#define ENCD3_LCD_COM_SEG            (* (reg8 *) ENCD3__LCD_COM_SEG)
/* Enable Segment LCD */
#define ENCD3_LCD_EN                 (* (reg8 *) ENCD3__LCD_EN)
/* Slew Rate Control */
#define ENCD3_SLW                    (* (reg8 *) ENCD3__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define ENCD3_PRTDSI__CAPS_SEL       (* (reg8 *) ENCD3__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define ENCD3_PRTDSI__DBL_SYNC_IN    (* (reg8 *) ENCD3__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define ENCD3_PRTDSI__OE_SEL0        (* (reg8 *) ENCD3__PRTDSI__OE_SEL0) 
#define ENCD3_PRTDSI__OE_SEL1        (* (reg8 *) ENCD3__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define ENCD3_PRTDSI__OUT_SEL0       (* (reg8 *) ENCD3__PRTDSI__OUT_SEL0) 
#define ENCD3_PRTDSI__OUT_SEL1       (* (reg8 *) ENCD3__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define ENCD3_PRTDSI__SYNC_OUT       (* (reg8 *) ENCD3__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(ENCD3__SIO_CFG)
    #define ENCD3_SIO_HYST_EN        (* (reg8 *) ENCD3__SIO_HYST_EN)
    #define ENCD3_SIO_REG_HIFREQ     (* (reg8 *) ENCD3__SIO_REG_HIFREQ)
    #define ENCD3_SIO_CFG            (* (reg8 *) ENCD3__SIO_CFG)
    #define ENCD3_SIO_DIFF           (* (reg8 *) ENCD3__SIO_DIFF)
#endif /* (ENCD3__SIO_CFG) */

/* Interrupt Registers */
#if defined(ENCD3__INTSTAT)
    #define ENCD3_INTSTAT            (* (reg8 *) ENCD3__INTSTAT)
    #define ENCD3_SNAP               (* (reg8 *) ENCD3__SNAP)
    
	#define ENCD3_0_INTTYPE_REG 		(* (reg8 *) ENCD3__0__INTTYPE)
#endif /* (ENCD3__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_ENCD3_H */


/* [] END OF FILE */
