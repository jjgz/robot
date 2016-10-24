// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

#include "processing.h"
#include "network/send.h"
#include "debug.h"
#include "pid/pid.h"

#define TEN_CM(a) (750*a)
#define ROTATE(a) (650*a)
#define X_SCALE(a) (a*10)

#define MAX_GRID 20
#define X 4
#define Y 5
#define MAX_WEIGHT 500

extern Pid controller_right;
extern Pid controller_left;
QueueHandle_t processing_queue;
extern QueueHandle_t interrupt_queue;
extern rover my_rover;
extern double wanted_speed_left;
extern double wanted_speed_right;
unsigned output_right_avg;
unsigned output_left_avg;
uint16_t processing_counter;

//**********************************************************
//**Everything below to the stars (*) is for the pathfinding
unsigned target = 34;
unsigned short int edges_size;
//unsigned short int next_edges_size;
edges EDGES[MAX_GRID];
edges NEXT_EDGES[MAX_GRID];
edges path_to_go[MAX_GRID];
tiles my_world[X][Y];

//path_index is the size of the path array that contains all the points to the target
unsigned path_index;
unsigned target;

//this is essentially a boolean to check if there is indeed a path..if there isnt a path dont move
uint8_t no_path;

//map_size is how your indexer for the path_to_go array..compare it to path_index (i.e. if(map_size < path_index))
//unsigned x = path_to_go[map_size].xy/10 - my_rover.position.x;
//unsigned y = path_to_go[map_size].xy%10 - my_rover.position.y;
unsigned map_size;
//***********************************************************
//**********************************************************

void enable_init()
{
    pid_start(&controller_left);
    pid_start(&controller_right);
   // PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_5);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    DRV_TMR3_Start();
    DRV_OC0_Enable();
    DRV_OC1_Enable();
    DRV_OC0_Start();
    DRV_OC1_Start();
    
    //THIS HAS THEM STOP/////
    wanted_speed_right = 2e-5;
    wanted_speed_left = 2e-5;
    //////////////////////////
    output_left_avg = 0;
    output_right_avg = 0;
    processing_counter = 0;
    map_size = 0;
    init_rover();
    map_init();
}
void init_rover()
{
    my_rover.rover_state = ROVER_INIT;
    //your location
    my_rover.xy_points.x = 0;
    my_rover.xy_points.y = 0;
    my_rover.ticks.tick_left = 0;
    my_rover.ticks.tick_right = 0;
    my_rover.bools.stop_left = false;
    my_rover.bools.stop_right = false;
    my_rover.bools.slow_left = false;
    my_rover.bools.slow_right = false;
    my_rover.bools.got_name = false;
    my_rover.bools.test_move = false;
    my_rover.bools.test_rotate = false;
    my_rover.bools.rotate = true;
}
void map_init()
{
 	my_world[0][0].weight = 10;
	my_world[0][1].weight = 20;
	my_world[0][2].weight = 20;
	my_world[0][3].weight = 7;
	my_world[0][4].weight = 5;

	my_world[1][0].weight = 20;
	my_world[1][1].weight = MAX_WEIGHT;
	my_world[1][2].weight = 50;
	my_world[1][3].weight = 10;
	my_world[1][4].weight = 7;

	my_world[2][0].weight = 30;
	my_world[2][1].weight = 40;
	my_world[2][2].weight = MAX_WEIGHT;
	my_world[2][3].weight = 30;
	my_world[2][4].weight = 10;

	my_world[3][0].weight = MAX_WEIGHT;
	my_world[3][1].weight = 60;
	my_world[3][2].weight = MAX_WEIGHT;
	my_world[3][3].weight = 30;
	my_world[3][4].weight = 0;
}

void init_world_diff()
{
    	//go through x
    int i,j;
	for (i = 0; i < 4; i++)
	{
		//go through y
		for (j = 0; j < 5; j++)
		{
			my_world[i][j].difficulty = 1000;
		}
	}
}
void path_init()
{
    no_path = 1;
    path_index = 0;
	edges_size = 0;
    //WHERE YOU ASSIGN TARGET
	target = 34; //its at position <3,4>
    int i;
	for (i = 0; i < MAX_GRID; i++)
		path_to_go[i].xy = -1;
}

void my_path()
{
    unsigned find = 1;
	unsigned total = MAX_GRID;
	//my_world[my_pos / 10][my_pos%10].
	int x = my_rover.xy_points.x;
	int y = my_rover.xy_points.y;
	//just in case
	if (x == target / 10 && y == target % 10){
		find = 0;
		path_to_go[0].xy = (x*10) + y;
        no_path = 0;
	}

	int index = (x*10) + y;
    //GET RID OF WHILE LOOP AND MAKE IT SUCH THAT YOU JSUT FIND the NEXT PATH
	//AND JUST RETURN INDEX
    while (find && total)
	{
			int min;
			if (x + 1 < 4)
			{
				index = (x+1)*10 + y;
				min = my_world[x + 1][y].difficulty;
			}
			else
			{
				index = (x+1)*10 + y;
				min = my_world[x - 1][y].difficulty;
			}

			if (my_world[x - 1][y].difficulty < min && x - 1 >= 0 && my_world[x - 1][y].difficulty < 100)
			{
				min = my_world[x - 1][y].difficulty;
				index = (x-1)*10 + y;
			}
			if (my_world[x + 1][y].difficulty < min && x + 1 < 4 && my_world[x + 1][y].difficulty < 100)
			{
				
				min = my_world[x + 1][y].difficulty;
				index = (x+1)*10 + y;
			}
			if (my_world[x][y + 1].difficulty < min && y + 1 < 5 && my_world[x][y+1].difficulty < 100)
			{
				min = my_world[x][y+1].difficulty;
				index = (X_SCALE(x)) + y + 1;
			}
			if (my_world[x][y - 1].difficulty < min && y - 1 >= 0 && my_world[x][y-1].difficulty < 100)
			{
				min = my_world[x - 1][y].difficulty;
				index = (X_SCALE(x)) + y - 1;
			}

			if (index / 10 == target / 10 && index % 10 == target % 10){
				find = 0;
                no_path = 0;
            }
			
			if (min < 100)
				path_to_go[path_index++].xy = index;
            debug_loc(index);
			x = index / 10;
			y = index % 10;
			total--;
	}
}
void find_path()
{
    init_world_diff();
	my_world[target / 10][target % 10].difficulty = 0;
	my_world[target / 10][target % 10].weight = 0;
	EDGES[0].xy = target;//<3,4>
	edges_size = 1;
	while (edges_size)
	{
		int j = 0;
		int i;
		for (i = 0; i < edges_size; i++)
		{
			int x = EDGES[i].xy / 10; //the current tile
			int y = EDGES[i].xy % 10; //the current tile
			
			unsigned short int world_diff_n = my_world[x][y].difficulty + my_world[x][y+1].weight;
			unsigned short int world_diff_s = my_world[x][y].difficulty + my_world[x][y-1].weight;
			unsigned short int world_diff_e = my_world[x][y].difficulty + my_world[x+1][y].weight;
			unsigned short int world_diff_w = my_world[x][y].difficulty + my_world[x-1][y].weight;

			if (world_diff_n < my_world[x][y + 1].difficulty && y+1 < 5)
			{
				NEXT_EDGES[j].xy = EDGES[i].xy + 1; //for north
				my_world[x][y + 1].difficulty = world_diff_n;
				j++;
			}
			if (world_diff_s < my_world[x][y - 1].difficulty && y - 1 >= 0)
			{
				NEXT_EDGES[j].xy = EDGES[i].xy - 1; //for south
				my_world[x][y - 1].difficulty = world_diff_s;
				j++;
			}
			if (world_diff_e < my_world[x+1][y].difficulty && x+1 < 4)
			{
				NEXT_EDGES[j].xy = EDGES[i].xy + 10; //for east
				my_world[x+1][y].difficulty = world_diff_e;
				j++;
			}
			if (world_diff_w < my_world[x-1][y].difficulty && x-1 >= 0)
			{
				NEXT_EDGES[j].xy = EDGES[i].xy - 10; //for west
				my_world[x-1][y].difficulty = world_diff_w;
				j++;
			}
		}
		int k;
		for (k = 0; k < j; k++)
			EDGES[k].xy = NEXT_EDGES[k].xy;
		edges_size = j;
		//print_world();
		//print_edges();
	}
}

void change_direction(unsigned turn_left, uint16_t degree, orientation dir)
{
    move_wheels(1,0);
    if(turn_left)
        move_wheels(0,1);
    my_rover.bools.rotate = true;
    my_rover.value.rotate_val = ROTATE(degree);
    my_rover.ori = dir;
}


void processing_add_recvmsg(NRMessage *message) {
    PRMessage pr_message;
    pr_message.type = PR_NR;
    pr_message.data.nr_message = *message;
    xQueueSendToBack(processing_queue, &pr_message, portMAX_DELAY);
}
//tmr3 = right
//tmr4 = left
void processing_add_pwm_reading(uint32_t left_pwm, uint32_t right_pwm,uint32_t tmr3, uint32_t tmr4, double left_error, double right_error){
    
    if(my_rover.bools.got_name)
    {
       processing_counter++;
       output_left_avg += left_pwm;
       output_right_avg += right_pwm;

       my_rover.ticks.tick_left += tmr4;
       my_rover.ticks.tick_right += tmr3;
       if(processing_counter == 80)
       {
           processing_counter = 0;
           output_left_avg = output_left_avg/80;
           output_right_avg = output_right_avg/80;
           PRMessage pr_adc_message;
           pr_adc_message.type = PR_PWM;
           pr_adc_message.data.timer.speed_left = output_left_avg;
           pr_adc_message.data.timer.speed_right = output_right_avg;
           pr_adc_message.data.timer.tmr3 = tmr3;
           pr_adc_message.data.timer.tmr4 =  tmr4;
           pr_adc_message.data.timer.left_error = left_error;
           pr_adc_message.data.timer.right_error = right_error;
           output_left_avg = 0;
           output_right_avg = 0;
           BaseType_t higher_priority_task_woken = pdFALSE;
           // Attempt add the buffer from the isr to the queue.
           if (xQueueSendToBackFromISR(processing_queue, &pr_adc_message, &higher_priority_task_woken)) {
               // If a higher priority task was waiting for something on the queue, switch to it.
               portEND_SWITCHING_ISR(higher_priority_task_woken);
           // We didn't receive a buffer.
           } else {
               // Indicate on LD4 that we lost a packet.
               // NOTE: LD4 conflicts with SDA2 (I2C).
               SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
           }    
       }

    }
   
}

void interrupt_add_pwm(pwm_to_isr *pwm)
{
    pwm_to_isr pwm_val = *pwm;
    xQueueSendToBack(interrupt_queue, &pwm_val, portMAX_DELAY);
}
void PROCESSING_Initialize() {
    enable_init();
    processing_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(NRMessage));
    interrupt_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(pwm_to_isr));
}

void PROCESSING_Tasks() {
    PRMessage recv_message;
    NSMessage send_message;
    pwm_to_isr pwm;
    unsigned blocks = 0;
    // Create netstats message.
    NSMessage netstats_message;
    netstats_message.type = NS_NETSTATS;
    MSGNetstats *netstats = &netstats_message.data.netstats;
    netstats->numGoodMessagesRecved = 0;
    netstats->numCommErrors = 0;
    netstats->numJSONRequestsRecved = 0;
    netstats->numJSONResponsesRecved = 0;
    netstats->numJSONRequestsSent = 0;
    netstats->numJSONResponsesSent = 0;
   move_wheels(0,0);
    
    network_send_add_message(&netstats_message);

    while (1) {
        // We responded to a request, so we increase the responses sent.
        xQueueReceive(processing_queue, &recv_message, portMAX_DELAY);
        
        switch (recv_message.type) {
            case PR_NR:{
                NRMessage *nr_message = &recv_message.data.nr_message;
                 switch (nr_message->type) {
                    case NR_QUERY_STATS: {
                        netstats->numGoodMessagesRecved++;
                        netstats->numJSONRequestsRecved++;
                        network_send_add_message(&netstats_message);

                        // We responded to a request, so we increase the responses sent.
                        netstats->numJSONResponsesSent++;
                    } break;
                    case NR_INVALID_ERROR:
                    {
                        netstats->numCommErrors++;
                    } break;
                    case NR_REQ_NAME:
                    {
                        //my_rover.got_name = true;
                        send_message.type = NS_SEND_NAME_JOSH;
                        network_send_add_message(&send_message);
                    } break;
                     case NR_REQ_JOSH_POINTS:
                     {
                         send_message.type = NS_JOSH_REQ_POINTS;
                         send_message.data.point.x = 5;
                         send_message.data.point.y = 6;
                         network_send_add_message(&send_message);
                     }break;
                     case NR_REQ_IN_POS:
                     {
                         send_message.type = NS_IN_POS;
                         send_message.data.answer = true;
                         network_send_add_message(&send_message);
                     }break;
                     case NR_TEST_ROTATE:
                     {
                         my_rover.bools.test_rotate = true;
                         my_rover.debug_test.test_rotate_val = recv_message.data.nr_message.data.rotate_val;
                     }break;
                     case NR_TEST_RESET:
                     {
                        init_rover();
                        my_rover.bools.got_name = true;
                        pwm.wanted_speed_right = 0;
                        pwm.wanted_speed_left = 0;
                        interrupt_add_pwm(&pwm);
                     }break;
                     case NR_TEST_MOVE:
                     {
                         my_rover.bools.test_move = true;
                         my_rover.debug_test.test_move_val = recv_message.data.nr_message.data.move_val;
                     }break;
                     //TODO: Add a NR_REQ_TEST_RESET case such that you can reset the values and everything 
                     //you could also send back a send message to get a confirmation that the command was sent..or do it down in the case statement
                     //and just push it in the queue. If you choose to do this part then add it into send.c
                     //TODO2: Eventually you need to add request (NR_REQ_TEST_MOVE or NR_REQ_TEST_ROT) to indicate that you want to run a move test or a reotation test
                     //just set the my_rover state to init and create boolean that you set in this condition and use in the bottom switch() statement.)
                    default:
                        break;
                }       
            } break;
            case PR_PWM:
            {
                if(my_rover.bools.got_name)
                {
                    send_message.type = NS_PWM;

                    send_message.data.tmr.speed_left= recv_message.data.timer.speed_left;
                    send_message.data.tmr.speed_right = recv_message.data.timer.speed_right;
                    send_message.data.tmr.tmr3= my_rover.ticks.tick_right;
                    send_message.data.tmr.tmr4 = my_rover.ticks.tick_left;
                    send_message.data.tmr.left_error = recv_message.data.timer.left_error;
                    send_message.data.tmr.right_error = recv_message.data.timer.right_error;
                    network_send_add_message(&send_message);   
                }
            }break;
            default:
                break;
        }
        switch(my_rover.rover_state)
        {
            case ROVER_INIT:
            {
                my_rover.ori = SOUTH;
                my_rover.ticks.tick_right = 0;
                my_rover.ticks.tick_left = 0;
                //if you wanna test rotation via debug assign next_ori and current_ori and go to state ROVER_MOVE
                if(my_rover.bools.got_name && (my_rover.bools.test_rotate || my_rover.bools.test_move)){
                    blocks = 0;
                    my_rover.rover_state = ROVER_FIND_PATH;
                }
            }break;
            case ROVER_FIND_PATH:
            {
                pwm.wanted_speed_right =0;
                pwm.wanted_speed_left = 0;
                interrupt_add_pwm(&pwm);
                path_index = 0;
                find_path();
                my_path();
                if(!no_path)
                    my_rover.rover_state = ROVER_FIND_DIR;
            }break;
            case ROVER_FIND_DIR:
            {
                
               // SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
//                pwm.wanted_speed_right =0;
//                pwm.wanted_speed_left = 0;
//                interrupt_add_pwm(&pwm);
                unsigned x = path_to_go[map_size].xy/10 - my_rover.xy_points.x;
                unsigned y = path_to_go[map_size].xy%10 - my_rover.xy_points.y;
                 
                //****************************************************
                if(x)
                {
                    //debug_loc(20);
                    my_rover.next_ori = WEST;
                    if(x > 0) //move east
                    {
                        //debug_loc(22);
                        my_rover.next_ori = EAST;
                    }
                }
                else if(y)
                {
                   // debug_loc(30);
                    my_rover.next_ori = SOUTH;
                    if(y > 0)
                    {
                        //debug_loc(33);
                        my_rover.next_ori = NORTH;
                    }
                }
                my_rover.rover_state = MY_ROVER_CHANGE_ORI;
            }break;
            case MY_ROVER_CHANGE_ORI:
            {
//                pwm.wanted_speed_right =0;
//                pwm.wanted_speed_left = 0;
//                interrupt_add_pwm(&pwm);
                switch(my_rover.ori)
                {
                    case NORTH:
                    {
                        //debug_loc(40);
                        if(my_rover.next_ori == SOUTH){
                            change_direction(0,2, my_rover.next_ori);
                          //  debug_loc(99);
                        }
                        else if(my_rover.next_ori == EAST){
                            change_direction(0,1, my_rover.next_ori);
                            //debug_loc(44);
                        }
                        else if(my_rover.next_ori == WEST)
                            change_direction(1,1, my_rover.next_ori);
                        else
                            my_rover.bools.rotate = false;
                        break;
                    }
                    case SOUTH:
                    {
                        if(my_rover.next_ori == NORTH)
                            change_direction(0,2, my_rover.next_ori);
                        else if(my_rover.next_ori == EAST)
                            change_direction(0,1, my_rover.next_ori);
                        else if(my_rover.next_ori == WEST)
                            change_direction(1,1, my_rover.next_ori);
                        else
                        {
                            my_rover.bools.rotate = false;
                        }
                        break;
                    }
                    case EAST:
                    {
                        //debug_loc(50);
                        if(my_rover.next_ori == NORTH){
                            change_direction(1,1, my_rover.next_ori);
                            //debug_loc(55);
                        }
                        else if(my_rover.next_ori == SOUTH)
                            change_direction(0,1, my_rover.next_ori);
                        else if(my_rover.next_ori == WEST)
                            change_direction(0,2, my_rover.next_ori);
                        else
                        {
                            my_rover.bools.rotate = false;
                        }
                        break;
                    }
                    case WEST:
                    {
                        if(my_rover.next_ori == NORTH)
                            change_direction(0,1, my_rover.next_ori);
                        else if(my_rover.next_ori == EAST)
                            change_direction(0,2, my_rover.next_ori);
                        else if(my_rover.next_ori == SOUTH)
                            change_direction(1,1, my_rover.next_ori);
                        else
                        {
                            my_rover.bools.rotate = false;
                        }
                        break;
                    }
                
                default:
                    break;
            }
                my_rover.rover_state = ROVER_MOVE;
            }break;
            //
            case ROVER_MOVE:
            {   
                my_rover.ticks.tick_right = 0;
                my_rover.ticks.tick_left = 0;
                pwm.wanted_speed_right = 2e-1;
                pwm.wanted_speed_left = 2e-1;
                
                //move_wheels(0,1);//left rotate
                
                interrupt_add_pwm(&pwm);
                my_rover.rover_state = ROVER_BLOCK;
                if(my_rover.bools.rotate){
                    my_rover.rover_state = ROVER_ROTATE;
                }
                
            }break;
            
            case ROVER_ROTATE:
            {
                //unsigned rotate = 60;
                if(my_rover.ticks.tick_right >= my_rover.value.rotate_val)
                {                 
                    my_rover.ticks.tick_right = 0;
                    my_rover.bools.stop_right = true;
                }
                if(my_rover.ticks.tick_left >= my_rover.value.rotate_val)
                {         
                    my_rover.ticks.tick_left= 0;
                    my_rover.bools.stop_left = true;
                }
                
                if(my_rover.bools.stop_left && my_rover.bools.stop_right){
                    my_rover.rover_state = ROVER_BLOCK;
                    my_rover.ticks.tick_left= 0;
                    my_rover.ticks.tick_right= 0;
                    my_rover.bools.stop_left = false;
                    my_rover.bools.stop_right = false;
                    move_wheels(0,0); 
                }
                else
                {
                    pwm.wanted_speed_right = 2e-1;
                    pwm.wanted_speed_left = 2e-1;
                }
                
                interrupt_add_pwm(&pwm);
            }break;
            
            //***********************************
            //TODO: Move this all into a function + create the move block functions
            //***********************************
            case ROVER_BLOCK:
            {
                if(my_rover.ticks.tick_right >= TEN_CM(1))
                {                 
                    my_rover.ticks.tick_right = 0;
                    my_rover.bools.stop_right = true;
                }
                else if(my_rover.ticks.tick_right == 650)
                {
                    my_rover.bools.slow_right = true;
                }
                
                if(my_rover.ticks.tick_left >= TEN_CM(1))
                {         
                    my_rover.ticks.tick_left= 0;
                    my_rover.bools.stop_left = true;
                }
                else if(my_rover.ticks.tick_left == 650)
                {
                    my_rover.bools.slow_left = true;
                }
                
                
//                if(my_rover.bools.slow_right && my_rover.bools.slow_left)
//                {
//                    pwm.wanted_speed_left = 1.5e-1;
//                    pwm.wanted_speed_right = 1.5e-1;
//                    my_rover.bools.slow_left = false;
//                    my_rover.bools.slow_right = false;
//                }
                

                if(my_rover.bools.stop_left && my_rover.bools.stop_right){
                    blocks++;
                    my_rover.rover_state = ROVER_STOP;
                    pwm.wanted_speed_right = 0;
                    pwm.wanted_speed_left = 0;
                    
                    my_rover.xy_points.x = path_to_go[map_size].xy/10;
                    my_rover.xy_points.y = path_to_go[map_size].xy%10;
                    
                    map_size++;
                    if(map_size < path_index){
                        my_rover.rover_state = ROVER_FIND_DIR;
                        my_rover.bools.stop_left = false;
                        my_rover.bools.stop_right = false;
                        send_message.type = NS_ROVER_DATA;
                        send_message.data.rd.point.x = my_rover.xy_points.x;
                        send_message.data.rd.point.y =  my_rover.xy_points.y;
                        send_message.data.rd.ori = my_rover.ori;
                        send_message.data.rd.target = target;

                        network_send_add_message(&send_message); 
                    }
                }
                interrupt_add_pwm(&pwm);
            }break;
            
            //***********************************
            //TODO: Move this all into a function + create the rotate functions
            //TODO2: Make the network signals for ROVER_ROTATE_R and ROVER_ROTATE_L -- mainly for debugging
            //***********************************
          
            case ROVER_STOP:
            {
                SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 0);
                pwm.wanted_speed_right = 0;
                pwm.wanted_speed_left = 0;
                interrupt_add_pwm(&pwm);
            }break;
            
            default:
                break;
        }
        
    }
}

void move_wheels(unsigned right, unsigned left)
{
    SYS_PORTS_PinWrite(0, PORT_CHANNEL_C, PORTS_BIT_POS_14, right);
    SYS_PORTS_PinWrite(0, PORT_CHANNEL_G, PORTS_BIT_POS_1, left);
}
/*******************************************************************************
 End of File
 */
