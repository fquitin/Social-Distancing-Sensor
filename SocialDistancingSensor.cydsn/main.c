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

#define LIGHT_OFF                       (0u)
#define LIGHT_ON                        (1u)
#define DEFAULT_PRIORITY                    (3u)

static volatile uint8_t inter = 0;
CY_ISR_PROTO(ISR);
CY_ISR(ISR){
    IRQ_ClearPending();    /* Clear pending Interrupt */
    pin_1_ClearInterrupt();    /* Clear pin Interrupt */
	inter = 1;
}
enum ROLE_e {
    RECEIVER,
    TRANSMITTER,
    ANCHOR,
    TAG,
    NOT_DEFINED
};

enum STATE_e {
    STATE_INIT,
    /* Tag */
    STATE_TAG_WAIT_FIRST_SEND,
    STATE_TAG_WAIT_RESPONSE,	
    STATE_TAG_SECOND_SEND,
    STATE_TAG_WAIT_SECOND_SEND,
    STATE_TAG_GET_TIMES,
    STATE_TAG_PROCESS_DISTANCE,
    STATE_TAG_END,
    
    /* Anchor */
    STATE_ANCHOR_LISTENING,
    STATE_ANCHOR_SEND_ANSWER,
    STATE_ANCHOR_WAIT_ANSWER_COMPLETION,
    STATE_ANCHOR_WAIT_FINAL,
    STATE_ANCHOR_SEND_TIMES,
    STATE_ANCHOR_END,
    
    /* Social */
    STATE_INITIATE_CONTACT,
    STATE_INITIATE_CONTACT_PENDING,
    STATE_PREPARE_TO_LISTEN,
    STATE_LISTENING,
    STATE_SEND_FINAL,
    STATE_SEND_FINAL_PENDING,
    STATE_WAIT_FOR_RESULTS,
    STATE_PROCESS_RESULTS,
    STATE_ANSWER,
    STATE_ANSWER_PENDING,
    STATE_WAIT_FOR_FINAL,
    STATE_SEND_RESULTS,
    STATE_SEND_RESULTS_PENDING,
};

enum Message_ID_e {
    INITIAL_MSG = 0x10,
    //ANSWER_MSG = 0x11,
    ANSWER_MSG = 0xFA,
    FINAL_MSG = 0x12,
    RESULTS_MSG = 0x13,
};

enum STATE_e state = STATE_INIT;
enum ROLE_e role = NOT_DEFINED;

static uint8_t RxData[15];
static uint8_t TxData[128];

uint8_t SW1_cur = 0;
uint8_t SW2_cur = 0;
uint8_t SW3_cur = 0;
uint8_t SW4_cur = 0;
uint8_t SW1_prev = 0;
uint8_t SW2_prev = 0;
uint8_t SW3_prev = 0;
uint8_t SW4_prev = 0;

uint8_t RxOk = 0u;
uint8_t TxOk = 0u;
uint8_t RxError = 0u;
uint32_t cycleTimeoutFlag = 0u;
uint16_t cycleTimeout = 0u;
uint16_t messageTimeout = 0u;
#define CYCLE_TIMEOUT (500u)
#define MESSAGE_TIMEOUT (5u)

static uint64_t t1, t2, t3, t4, t5, t6;
static uint8_t t1_8[5];
static uint8_t t2_8[5];
static uint8_t t3_8[5];
static uint8_t t4_8[5];
static uint8_t t5_8[5];
static uint8_t t6_8[5];

/*
static volatile uint64_t TroundMaster;
static volatile uint64_t TreplySlave;
static volatile uint64_t TroundSlave;
static volatile uint64_t TreplyMaster;
*/

double TroundMaster;
double TreplySlave;
double TroundSlave;
double TreplyMaster;

double timeOfFly_tick;
//double tick2s;
double distance;
double tof_num;
double tof_denum;

#define MEAN_LEN (1u)
double distLog[MEAN_LEN];
uint8_t distIdx = 0;
double mean = 0.0;

uint8_t succesRate = 0;
uint8_t successCntr = 0;
#define SUCCESS_LEN (20u)
float successPercent = 0.0f;

#define MAIN_TIMEOUT (500u)

#define TICK2S (1.0f / (128 * 499.2f * 10e6)) * 10 // Unknown factor 10 
//#define TICK2S (1.0f / (72499865600.0f))
//#define TICK2S (1.0f / (125e6))
#define LIGHT_SPEED (299792458.0f) // m/s

void interrupt_routine(void);
void rxLoop(void);
void txLoop(void);
void anchorLoop(void);
void tagLoop(void);
void socialDistancindLoop(void);

CY_ISR( isr_timeout_Handler ){
    /* Clear the inteerrupt */
    Timeout_timer_ReadStatusRegister();
    
    //timeoutFlag = timeoutFlag > 1u ? timeoutFlag-1u : 0u;
    
    if (cycleTimeout == 0u){
        cycleTimeoutFlag = 1u;
        cycleTimeout = CYCLE_TIMEOUT;
    }else{
        cycleTimeout--;
    }
    
    messageTimeout = messageTimeout > 1u ? messageTimeout-1u : 0u;
}

CY_ISR( isr_timer_Handler ){
    
    Timer_ReadStatusRegister();
    state = STATE_INIT;
    LED1_Write(~LED1_Read());
    successCntr++;
    if (successCntr >= SUCCESS_LEN)
    {
        successCntr = 0;
        successPercent = (SUCCESS_LEN - succesRate) * 100.0f / SUCCESS_LEN;
        succesRate = 0;
    }
    succesRate++;
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

    //isr_timeout_StartEx(isr_timer_Handler);
    //Timer_Start();
    
    Timeout_timer_Start();
    isr_timeout_StartEx(isr_timeout_Handler);
    
    socialDistancindLoop();
    
    if (Pot_Read())
    {
        role = ANCHOR;       
    }else
    {
        role = TAG;
    }
    
    if (role == TRANSMITTER){ txLoop(); }
    else if(role == RECEIVER){ rxLoop(); }
    else if(role == ANCHOR){ anchorLoop(); }
    else if(role == TAG){ tagLoop(); }
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
    inter=0;
}

void rxLoop(){
    LCD_Position(0u, 0u);
    LCD_PrintString(" Rx     ");

    idle();
    DWM_Enable_Rx();
    
    for(;;){
        if (inter){
            LED1_Write(~LED1_Read());
            interrupt_routine();
        }
        if (RxOk){
            RxOk = 0;
            LED2_Write(~LED2_Read());
			DWM_ReceiveData(RxData); 	// Read Rx buffer
            LCD_Position(0u,3u);
            LCD_PrintHexUint8(RxData[0]);
			
            DWM_ReadSPI_ext(RX_TIME,NO_SUB, t1_8, 5);// get T4
			for (int i=0;i<5;i++){
				t1 = (t1 << 8) | t1_8[4-i];
			}
            
            LCD_Position(1u,0u);
            LCD_PrintInt32(t1);

            DWM_Enable_Rx();

        }
        if (RxError)
        {
            RxError = 0u;
            LED3_Write(~LED3_Read());
            DWM_Reset_Rx();
            DWM_Enable_Rx();
        }
    }
}

void txLoop(){

    LCD_Position(0u, 0u);
    LCD_PrintString(" Tx     ");
    uint8_t cntr = 0u;
    
    for(;;){
        SW3_prev = SW3_cur;
        SW3_cur = SW3_Read();

        if (SW3_cur && !(SW3_prev))
        {
            cntr ++;
            CyDelay(50u);
            LCD_Position(1u, 0u);
            LCD_PrintString("sent    ");
            LCD_Position(1u, 6u);
            LCD_PrintNumber(cntr);
            LED1_Write(~LED1_Read());

            idle();
            
            TxData[0] = 0xFA;
            DWM_SendData(TxData,1,1);
            CyDelay(100u);
        }
        if (inter)
        {
            interrupt_routine();
        }
        if (TxOk)
        {
            TxOk = 0u;
            LED4_Write(~LED4_Read());
        }
    }
}

void anchorLoop(){
    LCD_Position(0u, 0u);
    LCD_PrintString("anchor  ");
    
    

    
    for(;;){
        switch(state){
            case STATE_INIT:
                idle();
                DWM_Enable_Rx();
                state = STATE_ANCHOR_LISTENING;
                LCD_Position(1,6);
                LCD_PrintNumber(successPercent);

            break;

            case STATE_ANCHOR_LISTENING :
                if(inter){ interrupt_routine(); }
                if (RxError){
                    RxError = 0;
                    DWM_Reset_Rx();
                    state = STATE_INIT;
                }
                else if (RxOk){
                    RxOk = 0;
                    DWM_ReceiveData(RxData); 	// Read Rx buffer
                    DWM_ReadSPI_ext(RX_TIME,NO_SUB, t2_8,5);
                    state = STATE_ANCHOR_SEND_ANSWER;

                }
            break;

            case STATE_ANCHOR_SEND_ANSWER :
                idle();
                TxData[0] = 0xFA;
                DWM_SendData(TxData, 1, 1);
                state = STATE_ANCHOR_WAIT_ANSWER_COMPLETION;

                
            break;
                
            case STATE_ANCHOR_WAIT_ANSWER_COMPLETION :
                if(inter){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    DWM_ReadSPI_ext(TX_TIME, NO_SUB, t3_8, 5);
                    idle();
                    DWM_Enable_Rx();
                    state = STATE_ANCHOR_WAIT_FINAL;

                }
            break;
                
            case STATE_ANCHOR_WAIT_FINAL :
                if(inter){ interrupt_routine(); }
                if (RxError){
                    RxError = 0;
                    DWM_Reset_Rx();
                    state = STATE_INIT;
                }
                else if (RxOk){
                    RxOk = 0;
                    DWM_ReceiveData(RxData);
                    DWM_ReadSPI_ext(RX_TIME,NO_SUB, t6_8,5);
                    state = STATE_ANCHOR_SEND_TIMES;

                }
            break;
                
            case STATE_ANCHOR_SEND_TIMES :
                idle();
				for (int i=0;i<5;i++){
					TxData[i] = t2_8[i];
					TxData[i+5] = t3_8[i];
					TxData[i+10] = t6_8[i];
				}
                DWM_SendData(TxData, 15, 1);
                state = STATE_ANCHOR_END;

                
            break;
                
            case STATE_ANCHOR_END :
                if(inter){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    /*LED1_Write(LIGHT_ON);
                    LED2_Write(LIGHT_ON);
                    LED3_Write(LIGHT_ON);
                    LED4_Write(LIGHT_ON);*/
                    LED4_Write(~LED4_Read());
                    idle();
                    DWM_Enable_Rx();
                    state = STATE_INIT;
                    succesRate--;

                    /*
                    for(;;){
                        for (uint8 i = 0u; i < 5u; i++){
                            LED1_Write(i == 0u ? 0u : 1u);
                            LED2_Write(i == 1u ? 0u : 1u);
                            LED3_Write(i == 2u ? 0u : 1u);
                            LED4_Write(i == 3u ? 0u : 1u);
                            CyDelay(75u);
                        }
                        CyDelay(500u);   
                    }
                    */
                }
            break;
                
            default:
            break;
        }
    }
}

void tagLoop(){
    LCD_Position(0u, 0u);
    LCD_PrintString("tag     ");

    for(;;){
        switch(state){
            case STATE_INIT :

                LCD_Position(1,6);
                LCD_PrintNumber(successPercent);

                if (1)

                {

                    idle();
                    TxData[0] = 0xFA;
                    DWM_SendData(TxData,1,1);
                    state = STATE_TAG_WAIT_FIRST_SEND;
                }
            break;
                
    		case STATE_TAG_WAIT_FIRST_SEND :
                if(inter){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    DWM_ReadSPI_ext(TX_TIME, NO_SUB, t1_8, 5);//get tx time (T1)
                    idle();
                    DWM_Enable_Rx();
                    state = STATE_TAG_WAIT_RESPONSE;

                    
                }
            break;
                
    		case STATE_TAG_WAIT_RESPONSE :
                if(inter){ interrupt_routine(); }
                if (RxError){
                    RxError = 0;
                    DWM_Reset_Rx();
                    state = STATE_INIT;

                }
                else if (RxOk){
                    RxOk = 0;
                    DWM_ReceiveData(RxData); 	// Read Rx buffer
                    DWM_ReadSPI_ext(RX_TIME,NO_SUB, t4_8,5);// get T4
                    state = STATE_TAG_SECOND_SEND;

                }
            break;
                
    		case STATE_TAG_SECOND_SEND :
                //idle();
                DWM_SendData(TxData, 1, 1);
                state = STATE_TAG_WAIT_SECOND_SEND;

            break;
                
    		case STATE_TAG_WAIT_SECOND_SEND :
                if(inter){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    DWM_ReadSPI_ext(TX_TIME, NO_SUB, t5_8, 5);	//get tx time (T5)
                    idle();
                    DWM_Enable_Rx();
                    state = STATE_TAG_GET_TIMES;

                    
                }
            break;
                
    		case STATE_TAG_GET_TIMES :
                if(inter){ interrupt_routine(); }
                if (RxError){
                    RxError = 0;
                    state = STATE_INIT;

                    DWM_Reset_Rx();
                }
                else if (RxOk){
                    DWM_ReceiveData(RxData);	//Read Rx Buffer
                    for (int i=0;i<5;i++){
                        t2_8[i] = RxData[i];
                        t3_8[i] = RxData[i+5];
                        t6_8[i] = RxData[i+10];
                    }

                    // Cast all times to uint64
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
                        state = STATE_INIT;

                    }
                    else{
                        state = STATE_TAG_PROCESS_DISTANCE;

                        RxOk = 0;
                    }
                    //LED4_Write(1);
                }
            break;
                
    		case STATE_TAG_PROCESS_DISTANCE :
                succesRate--;
                LED3_Write(~LED3_Read());

                TroundMaster = (t4-t1);
                TreplySlave = (t3-t2);
                TroundSlave = (t6-t3);
                TreplyMaster = (t5-t4);
                
                
                tof_num = TroundMaster * TroundSlave - TreplyMaster * TreplySlave;
                tof_denum = TroundMaster + TroundSlave + TreplySlave + TroundMaster;
                
                timeOfFly_tick = tof_num /tof_denum; 
                distance = LIGHT_SPEED * timeOfFly_tick * TICK2S; 

                if (distance > 0.0 && distance < 100.0)
                {

                    distLog[distIdx] = distance;
                    distIdx += 1;
                    if(distIdx >= MEAN_LEN){
                        distIdx = 0;
                        LED4_Write(~LED4_Read());
                        
                        mean = 0.0;
                        for (uint8_t i = 0; i < MEAN_LEN; i++){
                            mean += distLog[i];
                            //distLog[i] = 0;
                        }
                        mean = mean / MEAN_LEN;
                        
                        char str[8];
                        sprintf(str, "%d cm", (int) (mean * 100));
                        LCD_ClearDisplay();
                        LCD_Position(0,0);
                        LCD_PrintString(str);
                        
                    }                    
                }
                
                state = STATE_TAG_END;

                
            break;
            
            case STATE_TAG_END :
                CyDelay(1u);
            break;

            default:
            break;
        }
    }
}

void socialDistancindLoop(){
    //Timeout_timer_Start();
    state = STATE_INITIATE_CONTACT;
    for (;;){
        switch(state){
            case STATE_INITIATE_CONTACT :
                LED1_Write(~LED1_Read());
                idle();
                TxData[0] = INITIAL_MSG;
                DWM_SendData(TxData, 1, 1);
                state = STATE_INITIATE_CONTACT_PENDING;
                messageTimeout = MESSAGE_TIMEOUT;
                break;
            
            case STATE_INITIATE_CONTACT_PENDING:
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }  
                if(inter){ interrupt_routine(); }
                if (TxOk){
                    TxOk = 0;
                    DWM_ReadSPI_ext(TX_TIME, NO_SUB, t1_8, 5);
                    state = STATE_PREPARE_TO_LISTEN;
                }
                break;
                
            case STATE_PREPARE_TO_LISTEN:
                idle();
                DWM_Enable_Rx();
                state = STATE_LISTENING;
                break;

            case STATE_LISTENING:    

                if (cycleTimeoutFlag){
                    cycleTimeoutFlag = 0u;
                    state = STATE_INITIATE_CONTACT;
                }
                
                if(inter){ interrupt_routine(); }

                if (RxError){
                    RxError = 0;
                    DWM_Reset_Rx();
                    state = STATE_PREPARE_TO_LISTEN;
                }
                if (RxOk){
                    RxOk = 0;
                    DWM_ReceiveData(RxData);
                    if (RxData[0] == INITIAL_MSG){
                        DWM_ReadSPI_ext(RX_TIME,NO_SUB, t2_8,5);
                        state = STATE_ANSWER;
                    }
                    else if(RxData[0] == ANSWER_MSG){
                        DWM_ReadSPI_ext(RX_TIME,NO_SUB, t4_8,5);
                        state = STATE_SEND_FINAL;
                    }else{
                        state = STATE_LISTENING; // @TODO Verif
                    }
                }

                break;
            
            case STATE_SEND_FINAL:
                    TxData[0] = FINAL_MSG;
                    DWM_SendData(TxData, 1, 1);
                    state = STATE_SEND_FINAL_PENDING;
                    messageTimeout = MESSAGE_TIMEOUT;
                break;
                
            case STATE_SEND_FINAL_PENDING:
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }  
                if(inter){ interrupt_routine(); }
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
                if(inter){ interrupt_routine(); }
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }  
                if (RxError){
                    RxError = 0;
                    state = STATE_PREPARE_TO_LISTEN;  
                    DWM_Reset_Rx();
                }
                if (RxOk){
                    // @TODO handle first byte of RxData
                    DWM_ReceiveData(RxData);
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
                        RxOk = 0;
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
                timeOfFly_tick = tof_num /tof_denum; 
                
                distance = LIGHT_SPEED * timeOfFly_tick * TICK2S; 

                if (distance > 0.0 && distance < 100.0)
                {
                    distLog[distIdx] = distance;
                    distIdx += 1;
                    if(distIdx >= MEAN_LEN){
                        distIdx = 0;
                        LED4_Write(~LED4_Read());
                        
                        mean = 0.0;
                        for (uint8_t i = 0; i < MEAN_LEN; i++){
                            mean += distLog[i];
                            //distLog[i] = 0;
                        }
                        mean = mean / MEAN_LEN;
                        
                        char str[8];
                        sprintf(str, "%d cm", (int) (mean * 100));
                        LCD_ClearDisplay();
                        LCD_Position(0,0);
                        LCD_PrintString(str);
                        
                    }                    
                }

                
                //CyDelay(500u);
                //while(timeoutFlag > 0u){};
                
                state = STATE_PREPARE_TO_LISTEN;
                
                
                break;

                
            case STATE_ANSWER:
                idle();
                TxData[0] = ANSWER_MSG;
                DWM_SendData(TxData, 1, 1);
                state = STATE_ANSWER_PENDING;
                messageTimeout = MESSAGE_TIMEOUT;
                break;
            
            case STATE_ANSWER_PENDING:
                if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; } 
                if(inter){ interrupt_routine(); }
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
                if(inter){ interrupt_routine(); }
                if (RxError){
                    RxError = 0;
                    DWM_Reset_Rx();
                    state = STATE_PREPARE_TO_LISTEN; 
                }
                else if (RxOk){
                    RxOk = 0;
                    DWM_ReceiveData(RxData);
                    DWM_ReceiveData(RxData);
                    if (RxData[0] == FINAL_MSG){
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
                    for (int i=0;i<5;i++){
                        TxData[i] = t2_8[i];
                        TxData[i+5] = t3_8[i];
                        TxData[i+10] = t6_8[i];
                    }
                    DWM_SendData(TxData, 15, 1);
                    state = STATE_SEND_RESULTS_PENDING;
                    messageTimeout = MESSAGE_TIMEOUT;
                    break;
                
                case STATE_SEND_RESULTS_PENDING:
                    if (messageTimeout == 0u){ state = STATE_PREPARE_TO_LISTEN; }
                    if(inter){ interrupt_routine(); }
                    if (TxOk){
                        TxOk = 0;
                        LED3_Write(~LED3_Read());
                        state = STATE_PREPARE_TO_LISTEN;
                    }
                    break;
                
            default:
                break;
        }
    }

}



/* [] END OF FILE */
