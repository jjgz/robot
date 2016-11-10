//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _PROCESSING_H
#define _PROCESSING_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "network/recv.h"
#define MY_NAME "Joe Gdaniec"
#define PROCESSING_QUEUE_LEN 10
// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END
typedef enum {
    PR_ADC,
    PR_NR,
    PR_PWM,
    PR_TMR,
    //PR_DEBUG,
} PRType;

typedef enum {
    LEADER_INIT,
            LEADER_MOVE,
            LEADER_WAIT,
            LEADER_TURN,
            LEADER_STOP,
            LEADER_TURN_STOP,
            LEADER_BORDER,
            LEADER_BACK,
}LStates;

typedef enum {
    RIGHT_FIRST,
    LEFT_FIRST,
    BACK_UP,
    FORWARD,
    FINISHED,
    BORDER_WAIT,
}BStates;
typedef union {
<<<<<<< HEAD
    NRMessage nr_message;    
    TimerJGDebug timer;   
=======
    NRMessage nr_message;
    unsigned adc_sample;
    TimerDebug timer;
    bool left_mvmnt;
    bool right_mvmnt;
>>>>>>> parent of ac911b9... changed where my case block for rover movement was and made adjustments to speed of rover.  As of now rover moves with decent accuracy on straight movement and turns as well.
    MSGDebugJoeTread debug_joe_tread;
} PRUnion;

typedef struct{
    uint32_t prev_left;
    uint32_t prev_right;
    uint32_t t_right;
    uint32_t t_left;
}pwm_ticks;

typedef struct {
    PRType type;
    PRUnion data;
} PRMessage;

typedef struct{
    double target_left_spd;
    double target_right_spd;
}pwm_to_isr;

typedef struct{
    double dist_thresh;
    double border_thresh;
}thresh;
typedef struct {
    pwm_ticks ticks;
    LStates lead_state;
    BStates border;
    bool stop_left;
    bool stop_right;
    bool slow_left;
    bool slow_right;
<<<<<<< HEAD
    thresh thresholds;
    bool got_cmnd;
    ldr_move ldr_m;
    SensorReading sensors;
    
=======
    //uint32_t time_r;
    //uint32_t time_l;
    //uint32_t last_rmotor;
    //uint32_t last_lmotor;
>>>>>>> parent of ac911b9... changed where my case block for rover movement was and made adjustments to speed of rover.  As of now rover moves with decent accuracy on straight movement and turns as well.
}leader;
void processing_add_recvmsg(NRMessage *message);
void interrupt_add_pwm(pwm_to_isr *pwm);
void processing_add_pwm_reading(uint32_t left_pwm, uint32_t right_pwm, uint32_t tmr3, uint32_t tmr4);
<<<<<<< HEAD
void processing_change_rover_state(uint32_t timer_state);
void leader_state_change(double ultrasonic, double left_photo, double right_photo, double prev_left, double prev_right, double dist_thresh, double border_thresh);
=======
>>>>>>> parent of ac911b9... changed where my case block for rover movement was and made adjustments to speed of rover.  As of now rover moves with decent accuracy on straight movement and turns as well.
void enable_start();
void leader_move(unsigned right, unsigned left);
void PROCESSING_Initialize();
void PROCESSING_Tasks();

#endif /* _PROCESSING_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */
