/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <string.h>
#include <inttypes.h>
#include <math.h>

#ifndef __DWM1000_H
#define __DWM1000_H


/* ------------------------------------- */
/* REGISTERS LENGTHS                     */
/* ------------------------------------- */
#define DEV_ID_LEN					4
#define TX_FCTRL_LEN				5
#define SYS_MASK_LEN				4
#define LEN_SYS_CTRL				4
#define SYS_CFG_LEN					4
#define CHAN_CTRL_LEN				4
#define PMSC_CTRL0_LEN				4

/* ------------------------------------- */
/* REGISTERS MAP		                     */
/* ------------------------------------- */
#define DEV_ID      0x00
#define EUI         0x01
#define PANADR      0x03
#define SYS_CFG     0x04
#define SYS_TIME    0x06
#define TX_FCTRL    0x08
#define TX_BUFFER   0x09
#define DX_TIME     0x0A
#define RX_FWTO     0X0C
#define SYS_CTRL    0x0D
#define SYS_MASK    0x0E
#define SYS_STATUS  0x0F
#define RX_FINFO    0x10
#define RX_BUFFER   0x11
#define RX_FQUAL    0x12
#define RX_TTCKI    0x13
#define RX_TTCKO    0x14
#define RX_TIME     0x15
#define TX_TIME     0x17
#define TX_ANTD     0x18
#define SYS_STATE   0x19
#define ACK_RESP_T  0x1A
#define RX_SNIFF    0x1D
#define TX_POWER    0x1E
#define CHAN_CTRL   0x1F
#define USR_SFD     0x21
#define AGC_CTRL    0x23
#define EXT_SYNC    0x24
#define ACC_MEM     0x25
#define GPIO_CTRL   0x26
#define DRX_CONF    0x27
#define RF_CONF     0x28
#define TX_CAL      0x2A
#define FS_CTRL     0x2B
#define AON         0x2C
#define OTP_IF      0x2D
#define LDE_CTRL    0x2E
#define DIG_DIAG    0x2F
#define PMSC        0x36

#define PMSC_CTRL0	0x00

/* ------------------------------------- */
/* REGISTERS BIT MAP                     */
/* ------------------------------------- */
#define TRXOFF_BIT						6
#define TXSTRT_BIT						1
#define TX_OK_BIT						7
#define RX_FINISHED_BIT					13
#define RX_NO_ERROR_BIT					14
#define WAIT4RESP_BIT					7

/* ------------------------------------- */
/* READ-WRITE DEFINES                    */
/* ------------------------------------- */
#define NO_SUB		0xFFFFU
#define WRITE		0x80U
#define WRITE_SUB	0xC0U
#define READ		0x00U
#define READ_SUB	0x40U
#define RW_SUB_EXT	0x80U


enum CHANNEL { _1 = 1U, _2 = 2U, _3 = 3U, _4 = 4U, _5 = 5U, _7 = 7U };

enum PRF { _16MHZ = 0b01U, _64MHZ = 0b10U };

enum BITRATE { _110KBPS = 0b00U, _850KBPS = 0b01U, _6800KBPS = 0b10U };

enum PE { _64 = 0b0001U, _128 = 0b0101U, _256 = 0b1001U, _512 = 0b1101U, _1024 = 0b0010U, _1536 = 0b0110U, _2048 = 0b1010U, _4096 = 0b0011U };


/* ------------------------------------- */
/* UTILITIES DEFINES                     */
/* ------------------------------------- */
#define Bitset(var, bitno) ((var) |= 1UL<<(bitno))
#define Bitclear(var, bitno) ((var) &= ~(1UL<<(bitno)))




/* -------------------- Variables Defnitions ----------------------------- */

/* ------------------------------------- */
/* GLOBAL VARIABLES                      */
/* ------------------------------------- */
extern uint8_t _sysctrl[LEN_SYS_CTRL];




/* -------------------- Functions Definitions ---------------------------- */

/* ------------------------------------- */
/* UTILITIES FUNCTION                    */
/* ------------------------------------- */

/**
* @brief Set bit in a array of data (LSB)
* @param *data the array
* @param len length of the array
* @param bit nummer of the bit (LSB)
* @param val 1 or 0
*/
void setBit(uint8_t *data, uint16_t len, uint8_t bit, uint8_t val);


/**
* @brief Convert uint32_t to uint8_t[4] in little-endian
* @param dataInt uint32_t to convert
* @param data[4] array to store result
*/
void int2Bytes(const uint32_t dataInt, uint8_t data[4]);


/**
* @brief Convert uint16_t to uint8_t[2] in little-endian
* @param dataInt uint16_t to convert
* @param data[2] array to store result
*/
void short2Bytes(const uint16_t dataShort, uint8_t data[2]);


/* ------------------------------------- */
/* STATE MANAGEMENT FUNCTION             */
/* ------------------------------------- */

/**
* @brief Initialise the DW and setup it
*/
void DWM_Init(void);

/**
* @brief Set the DW in IDLE mode and update _deviceMode
*/
void idle(void);

/**
* @brief soft reset the DW and load the LDE (manual pp23-24)
*/
//void DWM_Reset(void);

/**
* @brief soft reset the RX
*/
void DWM_Reset_Rx(void);

/**
* @brief enable the RX state of the decawave
*/
void DWM_Enable_Rx(void);


/**
* @brief disable the RX state of the decawave
*/
//void DWM_Disable_Rx(void);

/* ------------------------------------- */
/*  RAW READ-WRITE FUNCTIONS             */
/* ------------------------------------- */

/**
* @brief Read data on the DW via SPI
* @param *spiHandle the STM32 spi
* @param address the register address to read
* @param offset the offset address to read
* @param *data the Rx buffer to receive data
* @param len The length of the data to read
*/
void DWM_ReadSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len);

/**
* @brief Write data on the DW via SPI
* @param *spiHandle the STM32 spi
* @param address the register address to write
* @param offset the offset address to write
* @param *data the Tx buffer to transfer data
* @param len The length of the data to write
*/
void DWM_WriteSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len);

/* ------------------------------------- */
/*  DWM1000 COMMUNICATIONS	             */
/* ------------------------------------- */

/**
* @brief Send data with the DW
* @param *data the data to send
* @param len the length of these data
* @param waitResponse activate Rx after data sent
*/
void DWM_SendData(uint8_t* data, uint8_t len, uint8_t waitResponse);

/**
* @brief Receive data from the DW communication
* @param *buffer where to store the data
*/
void DWM_ReceiveData(uint8_t* buffer);

#endif /* __DWM1000_H */

/* [] END OF FILE */
