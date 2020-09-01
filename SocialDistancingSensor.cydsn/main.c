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
#include "project.h"
#include "dwm1000.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define LIGHT_OFF                       (0u)
#define LIGHT_ON                        (1u)
#define DEFAULT_PRIORITY                    (3u)

static volatile uint8_t interruptFlag = 0;
CY_ISR_PROTO(ISR);
CY_ISR(ISR){
    IRQ_ClearPending();    /* Clear pending Interrupt */
    pin_1_ClearInterrupt();    /* Clear pin Interrupt */
	interruptFlag = 1;
}

enum STATE_e {
    STATE_INIT,
    
    /* Social */
    STATE_INITIATE_CONTACT,
    STATE_INITIATE_CONTACT_PENDING,
    STATE_PREPARE_TO_LISTEN,
    STATE_LISTENING,
    STATE_SEND_FINAL,
    STATE_SEND_FINAL_PENDING,
    STATE_WAIT_FOR_RESULTS, // DEL
    STATE_PROCESS_RESULTS,
    STATE_SEND_DISTANCE,
    STATE_SEND_DISTANCE_PENDING,

    STATE_WAIT_RANDOM,
    STATE_ANSWER,
    STATE_ANSWER_PENDING,
    STATE_WAIT_FOR_FINAL,
    STATE_SEND_RESULTS,
    STATE_SEND_RESULTS_PENDING,
    STATE_WAIT_FOR_DISTANCE,
};

enum Message_ID_e {
    INITIAL_MSG = 0x10,
    ANSWER_MSG = 0x11,
    FINAL_MSG = 0x12,
    RESULTS_MSG = 0x13,
    DISTANCE_MSG = 0x14,
};

struct Message_s{
    uint8_t id;
    uint8_t from;
    uint8_t to;
    int data;
};
struct Message_s msg2;
uint8_t deviceID = 0;
uint8_t remoteDeviceID;


enum STATE_e state = STATE_INIT;

#define DATA_LEN (15u)
static uint8_t RxData[DATA_LEN];
static uint8_t TxData[128];

uint8_t RxOk = 0u;
uint8_t TxOk = 0u;
uint8_t RxError = 0u;
uint8_t cycleTimeoutFlag = 0u;
uint16_t cycleTimeout = 0u;
uint16_t messageTimeout = 0u;
uint16_t responseTime = 0u;

uint8_t alertOn = 0u;
uint16 alertTimer = 0u;
#define ALERT_PERIOD (1000u)

#define CYCLE_PERIOD (200u)
#define MESSAGE_TIMEOUT (5u)

#define SAFE_DISTANCE_CM (100u)
#define SAFE_DISTANCE_STEP_CM (15u)

#define TICK2S ((1.0f / (128 * 499.2f * 10e6)) * 10.0f) // Unknown factor 10 
#define LIGHT_SPEED (299792458.0f) // m/s

void interrupt_routine(void);

void socialDistancindLoop(void);
void setRandomSeed(void);

struct Message_s readRxData(void);
void sendTxData(struct Message_s, uint16_t);

CY_ISR( isr_timeout_Handler ){
    /* Clear the inteerrupt */
    Timer_ReadStatusRegister();
    
    if (cycleTimeout == 0u){
        cycleTimeoutFlag = 1u;
        cycleTimeout = CYCLE_PERIOD;
    }else{
        cycleTimeout--;
    }
    
    if (alertOn){
        LED1_Write(alertOn >= 1u);
        LED2_Write(alertOn >= 2u);
        LED3_Write(alertOn >= 3u);
        LED4_Write(alertOn >= 4u);
        alertTimer = ALERT_PERIOD;
        alertOn = 0;
    }
    
    if (alertTimer == 1u){
        LED1_Write(LIGHT_OFF);
        LED2_Write(LIGHT_OFF);
        LED3_Write(LIGHT_OFF);
        LED4_Write(LIGHT_OFF);
    }
    
    messageTimeout = messageTimeout > 1u ? messageTimeout-1u : 0u;
    responseTime = responseTime > 1u ? responseTime-1u : 0u;
    alertTimer = alertTimer > 1u ? alertTimer-1u : 0u;
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

	LED1_Write(LIGHT_OFF);
    LED2_Write(LIGHT_OFF);
    LED3_Write(LIGHT_OFF);
    LED4_Write(LIGHT_OFF);
    
    SPIM_Start();
    LCD_Start();    

    /* Sets up the GPIO interrupt and enables it */
    IRQ_StartEx(ISR);
    IRQ_SetPriority(DEFAULT_PRIORITY);
    
    LCD_ClearDisplay();
    LCD_Position(0u,0u);
    LCD_PrintString("Init...");
    
	/* initialisation of the DecaWave */
	CyDelay(10u); //time for the DW to go from Wakeup to init and then IDLE	
	DWM_Init();
    
    LCD_ClearDisplay();
    LCD_Position(0u,0u);
    LCD_PrintString("Init..Ok");
  
    Timer_Start();
    isr_timeout_StartEx(isr_timeout_Handler);
    
    setRandomSeed();
    socialDistancindLoop();
}

void setRandomSeed(){
    int16 temp;
    DieTemp_GetTemp(&temp);
    srand( temp );
}

void interrupt_routine(void){

    uint8_t RxBuffer[4];
	uint32_t StatusRegister;
	uint8_t ack[4] = {0};
	// Getting status Register
	DWM_ReadSPI_ext(SYS_STATUS, NO_SUB,  RxBuffer, 4);
	StatusRegister  = (RxBuffer[3] << 24) | (RxBuffer[2] << 16) | (RxBuffer[1] << 8) | RxBuffer[0];
	if (StatusRegister == 0xFFFFFFFF) {
		// Read again
		DWM_ReadSPI_ext(SYS_STATUS, NO_SUB,  RxBuffer, 4);
		StatusRegister  = (RxBuffer[3] << 24) | (RxBuffer[2] << 16) | (RxBuffer[1] << 8) | RxBuffer[0];
	}
	if (StatusRegister & TX_OK_MASK){
		TxOk = 1;
		setBit(ack, 4, TX_OK_BIT, 1);
	}
	// check if RX finished
	if ((StatusRegister & RX_FINISHED_MASK) == RX_FINISHED_MASK) {
		ack[0] |= RX_FINISHED_MASK;
		ack[1] |= RX_FINISHED_MASK >> 8;
		ack[2] |= RX_FINISHED_MASK >> 16;
		ack[3] |= RX_FINISHED_MASK >> 24;
		RxOk = 1;
	}
	if (StatusRegister & RX_ERROR_MASK) {
		ack[0] |= RX_ERROR_MASK;
		ack[1] |= RX_ERROR_MASK >> 8;
		ack[2] |= RX_ERROR_MASK >> 16;
		ack[3] |= RX_ERROR_MASK >> 24;
		RxError = 1;
	}
	// clear IRQ flags on DW
	DWM_WriteSPI_ext(SYS_STATUS, NO_SUB, ack, 4);
    interruptFlag = 0;
}

struct Message_s readRxData(){
    struct Message_s msg;
    
    /* RxData is shifted by 1 byte and byte 0 is duplicated so message starts
       at byte 1. */
    DWM_ReceiveData(RxData);
    memcpy(&msg, &RxData[1], sizeof(msg2));
    
    return msg;
}

void sendTxData(struct Message_s msg, uint16_t timeout){
    memcpy(&TxData, &msg, sizeof(msg));
    DWM_SendData(TxData, sizeof(msg), 1);
    messageTimeout = timeout;
}

void socialDistancindLoop(){
    
    /* Asymmetric Double-Sided Two-Way Ranging */
    uint64_t t1 = 0u, t2 = 0u, t3 = 0u, t4 = 0u, t5 = 0u, t6 = 0u;
    uint8_t t1_8[5], t2_8[5], t3_8[5], t4_8[5], t5_8[5], t6_8[5];

    double TroundMaster;
    double TreplySlave;
    double TroundSlave;
    double TreplyMaster;
    double tof_num;
    double tof_denum;
    double tof_tick;
    double distance;
    
    state = STATE_INITIATE_CONTACT;
    deviceID = rand() & 0xFF;

    for (;;){
        switch(state){
            case STATE_INITIATE_CONTACT :
                idle();                
                sendTxData((struct Message_s) {
                    .id = INITIAL_MSG,
                    .from = deviceID,
                    .to = deviceID,
                }, 105); // @TODO timeout
                
                state = STATE_INITIATE_CONTACT_PENDING;
                break;
            
            case STATE_INITIATE_CONTACT_PENDING:
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }  
                if (interruptFlag){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    DWM_ReadSPI_ext(TX_TIME, NO_SUB, t1_8, 5);
                    state = STATE_PREPARE_TO_LISTEN;
                }
                break;
                
            case STATE_PREPARE_TO_LISTEN:
                remoteDeviceID = 0;
                idle();
                DWM_Enable_Rx();
                state = STATE_LISTENING;
                break;

            case STATE_LISTENING:    

                if (cycleTimeoutFlag){
                    cycleTimeoutFlag = 0u;
                    state = STATE_INITIATE_CONTACT;
                }
                
                if (interruptFlag){ interrupt_routine(); }

                if (RxError){
                    RxError = 0;
                    DWM_Reset_Rx();
                    state = STATE_PREPARE_TO_LISTEN;
                }
                if (RxOk){
                    RxOk = 0;
                    struct Message_s msg = readRxData();
                    if (msg.id == INITIAL_MSG){                        
                        DWM_ReadSPI_ext(RX_TIME,NO_SUB, t2_8,5);
                        remoteDeviceID = msg.from;
                        responseTime = (rand() % (50u - 1u)) + 1u;/// @TODO
                        //responseTime = 1u;
                        state = STATE_WAIT_RANDOM;
                    }
                    else if(msg.id == ANSWER_MSG && messageTimeout > 0u && msg.to == deviceID){
                        DWM_ReadSPI_ext(RX_TIME,NO_SUB, t4_8,5);
                        remoteDeviceID = msg.from;
                        state = STATE_SEND_FINAL;
                    }else{
                        state = STATE_LISTENING; // @TODO Verif
                    }
                }

                break;
            
            case STATE_SEND_FINAL:
                    sendTxData((struct Message_s) {
                        .id = FINAL_MSG,
                        .from = deviceID,
                        .to = remoteDeviceID,
                    }, MESSAGE_TIMEOUT);
                    state = STATE_SEND_FINAL_PENDING;
                break;
                
            case STATE_SEND_FINAL_PENDING:
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }  
                if (interruptFlag){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    DWM_ReadSPI_ext(TX_TIME, NO_SUB, t5_8, 5);
                    idle();
                    DWM_Enable_Rx();
                    state = STATE_WAIT_FOR_RESULTS;
                    messageTimeout = MESSAGE_TIMEOUT;
                }
                break;

            case STATE_WAIT_FOR_RESULTS:
                if (interruptFlag){ interrupt_routine(); }
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }  
                if (RxError){
                    RxError = 0;
                    state = STATE_PREPARE_TO_LISTEN;  
                    DWM_Reset_Rx();
                }
                if (RxOk){
                    DWM_ReceiveData(RxData);
                    /* Results messages are identified with their length instead,
                       of their ID because there isn't enough space left is the
                       data field for an ID */
                    uint8_t len;
                	DWM_ReadSPI_ext(RX_FINFO, NO_SUB, &len, 1);
                    if (len == DATA_LEN + 2){
                        // @TODO make a struct for that 1
                        for (int i=0;i<5;i++){
                            t2_8[i] = RxData[i];
                            t3_8[i] = RxData[i+5];
                            t6_8[i] = RxData[i+10];
                        }

                        t1 = t2 = t3 = t4 = t5 = t6 = 0;
                        for (int i=0;i<5;i++){
                            t1 = (t1 << 8) | t1_8[4-i];
                            t2 = (t2 << 8) | t2_8[4-i];
                            t3 = (t3 << 8) | t3_8[4-i];
                            t4 = (t4 << 8) | t4_8[4-i];
                            t5 = (t5 << 8) | t5_8[4-i];
                            t6 = (t6 << 8) | t6_8[4-i];
                        }
                        if (t6 < t2 || t5 < t1){
                            state = STATE_PREPARE_TO_LISTEN;  
                        }
                        else{
                            state = STATE_PROCESS_RESULTS;
                            RxOk = 0; // @TODO should it be higher ?
                        }   
                    }else{
                        state = STATE_PREPARE_TO_LISTEN;
                    }
                }
                break;

            case STATE_PROCESS_RESULTS:

                TroundMaster = (t4-t1);
                TreplySlave = (t3-t2);
                TroundSlave = (t6-t3);
                TreplyMaster = (t5-t4);
                
                tof_num = TroundMaster * TroundSlave - TreplyMaster * TreplySlave;
                tof_denum = TroundMaster + TroundSlave + TreplySlave + TroundMaster;
                tof_tick = tof_num /tof_denum; 
                
                distance = LIGHT_SPEED * tof_tick * TICK2S; 

                if (distance > 0.0 && distance < 100.0){
                        
                        if ((uint) (distance * 100.0) < SAFE_DISTANCE_CM + SAFE_DISTANCE_STEP_CM * 3){
                            alertOn = 1;
                        }
                        if ((uint) (distance * 100.0) < SAFE_DISTANCE_CM + SAFE_DISTANCE_STEP_CM * 2){
                            alertOn = 2;
                        }
                        if ((uint) (distance * 100.0) < SAFE_DISTANCE_CM + SAFE_DISTANCE_STEP_CM){
                            alertOn = 3;
                        }
                        if ((uint) (distance * 100.0) < SAFE_DISTANCE_CM){
                            alertOn = 4;
                        }
                        
                        char str[8];
                        sprintf(str, "%d cm", (uint) (distance * 100.0));
                        LCD_ClearDisplay();
                        LCD_Position(0,0);
                        LCD_PrintString(str);
                    //}                    
                }
                state = STATE_SEND_DISTANCE;                         
                break;
            
            case STATE_SEND_DISTANCE:
                sendTxData((struct Message_s) {
                    .id = DISTANCE_MSG,
                    .from = deviceID,
                    .to = remoteDeviceID,
                    .data = (int) (distance * 100),
                }, MESSAGE_TIMEOUT);
                
                state = STATE_SEND_DISTANCE_PENDING;
                break;
                
            case STATE_SEND_DISTANCE_PENDING:
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }  
                if (interruptFlag){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    state = STATE_PREPARE_TO_LISTEN;
                }
                break;
            
            case STATE_WAIT_RANDOM:

                if (interruptFlag){ interrupt_routine(); }

                if (RxError){
                    RxError = 0;
                    DWM_Reset_Rx();
                    state = STATE_PREPARE_TO_LISTEN;
                }
                if (RxOk){
                    RxOk = 0;
                    struct Message_s msg = readRxData();
                    if(msg.id == ANSWER_MSG && msg.to == remoteDeviceID){
                        state = STATE_PREPARE_TO_LISTEN;
                    }else if(msg.id == ANSWER_MSG && msg.to == deviceID){
                        DWM_ReadSPI_ext(RX_TIME,NO_SUB, t4_8,5);
                        remoteDeviceID = msg.from;
                        state = STATE_SEND_FINAL;
                    }
                }
                if (responseTime == 0u && remoteDeviceID != 0u){
                     state = STATE_ANSWER;
                }

                break;
                
            case STATE_ANSWER:
                idle();
                sendTxData((struct Message_s) {
                    .id = ANSWER_MSG,
                    .from = deviceID,
                    .to = remoteDeviceID,
                }, MESSAGE_TIMEOUT);
                state = STATE_ANSWER_PENDING;
                break;
            
            case STATE_ANSWER_PENDING:
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; } 
                if (interruptFlag){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    DWM_ReadSPI_ext(TX_TIME, NO_SUB, t3_8, 5);
                    idle();
                    DWM_Enable_Rx();
                    state = STATE_WAIT_FOR_FINAL;
                    messageTimeout = MESSAGE_TIMEOUT;
                }
                break;

            case STATE_WAIT_FOR_FINAL:
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; } 
                if (interruptFlag){ interrupt_routine(); }
                if (RxError){
                    RxError = 0;
                    DWM_Reset_Rx();
                    state = STATE_PREPARE_TO_LISTEN; 
                }
                else if (RxOk){
                    RxOk = 0;
                    struct Message_s msg = readRxData();
                    if (msg.id == FINAL_MSG && msg.to == deviceID && msg.from == remoteDeviceID){
                        DWM_ReadSPI_ext(RX_TIME,NO_SUB, t6_8,5);
                        state = STATE_SEND_RESULTS;
                    }
                    else{
                        state = STATE_PREPARE_TO_LISTEN; 
                    }
                }
                break;

                case STATE_SEND_RESULTS:
                    idle();
                    // @TODO make a struct for that 2
                    for (int i=0;i<5;i++){
                        TxData[i] = t2_8[i];
                        TxData[i+5] = t3_8[i];
                        TxData[i+10] = t6_8[i];
                    }
                    DWM_SendData(TxData, DATA_LEN, 1);
                    state = STATE_SEND_RESULTS_PENDING;
                    messageTimeout = MESSAGE_TIMEOUT;
                    break;
                
                case STATE_SEND_RESULTS_PENDING:
                    if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }
                    if (interruptFlag){ interrupt_routine(); }
                    if (TxOk){
                        TxOk = 0;
                        state = STATE_WAIT_FOR_DISTANCE;
                        messageTimeout = MESSAGE_TIMEOUT * 3; // @TODO Fix me
                    }
                    break;
                
                case STATE_WAIT_FOR_DISTANCE:
                    if (messageTimeout == 0u){
                        state = STATE_PREPARE_TO_LISTEN;
                    } 
                    if (interruptFlag){ interrupt_routine(); }
                    if (RxError){
                        RxError = 0;
                        DWM_Reset_Rx();
                        state = STATE_PREPARE_TO_LISTEN; 
                    }
                    else if (RxOk){
                        RxOk = 0;

                        struct Message_s msg = readRxData();
                        if (msg.id == DISTANCE_MSG && msg.to == deviceID && msg.from == remoteDeviceID){
                            
                            if ((uint) msg.data < SAFE_DISTANCE_CM + SAFE_DISTANCE_STEP_CM * 3){
                                alertOn = 1;
                            }
                            if ((uint) msg.data < SAFE_DISTANCE_CM + SAFE_DISTANCE_STEP_CM * 2){
                                alertOn = 2;
                            }
                            if ((uint) msg.data < SAFE_DISTANCE_CM + SAFE_DISTANCE_STEP_CM){
                                alertOn = 3;
                            }
                            if ((uint) msg.data < SAFE_DISTANCE_CM){
                                alertOn = 4;
                            }
                            
                            char str[8];
                            sprintf(str, "%d cm", msg.data);
                            LCD_ClearDisplay();
                            LCD_Position(0,0);
                            LCD_PrintString(str);
                            
                            state = STATE_PREPARE_TO_LISTEN;
                        }
                        else{
                            state = STATE_PREPARE_TO_LISTEN; 
                        }
                    }
                    break;
                
            default:
                break;
        }
    }

}

/* [] END OF FILE */
