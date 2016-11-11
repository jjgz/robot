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

#define MAX_GRID 500
#define X 128
#define Y 128
#define MAX_WEIGHT 99

extern Pid controller_right;
extern Pid controller_left;
QueueHandle_t processing_queue;
extern QueueHandle_t interrupt_queue;
rover my_rover;
extern double wanted_speed_left;
extern double wanted_speed_right;
unsigned output_right_avg;
unsigned output_left_avg;
uint16_t processing_counter;

//**********************************************************
//**Everything below to the stars (*) is for the pathfinding
unsigned short int edges_size;
//unsigned short int next_edges_size;
edges EDGES[MAX_GRID];
edges NEXT_EDGES[MAX_GRID];
edges path_to_go[MAX_GRID];
tiles my_world[X][Y];
uint16_t OFFSET;
bool find;
bool go_home;
bool home;
int path_point;

//path_index is the size of the path array that contains all the points to the target
unsigned path_index;

//this is essentially a boolean to check if there is indeed a path..if there isnt a path dont move
uint8_t no_path;

//map_size is how your indexer for the path_to_go array..compare it to path_index (i.e. if(map_size < path_index))
//unsigned x = path_to_go[map_size].xy/10 - my_rover.position.x;
//unsigned y = path_to_go[map_size].xy%10 - my_rover.position.y;
unsigned map_size;
unsigned int target_list[20];
uint8_t target_size;
uint8_t target_index;
uint8_t stopped_counter;
//***********************************************************
//**********************************************************

void enable_init()
{
    stopped_counter = 0;
    path_point = 0;
    find = false;
    go_home = false;
    home = false;
    OFFSET = 1000;
    target_size = 0;
    target_index = 0;
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
    path_init();
    map_init();
}
void init_rover()
{
    my_rover.rover_state = ROVER_INIT;
    //your location
    my_rover.xy_points.x = 35;
    my_rover.xy_points.y = 35;
    my_rover.home.x = 35;
    my_rover.home.y = 35;
    my_rover.ticks.tick_left = 0;
    my_rover.ticks.tick_right = 0;
    my_rover.ticks.prev_tick_left = my_rover.ticks.tick_left;
    my_rover.ticks.prev_tick_right = my_rover.ticks.tick_right;
    my_rover.ticks.tick_edge_detect_right = 0;
    my_rover.ticks.tick_distance_right = 0;
    my_rover.ticks.tick_edge_detect_left = 0;
    my_rover.ticks.tick_distance_left = 0;
    my_rover.ticks.rotate_tick_left = 0;
    my_rover.ticks.rotate_tick_right = 0;
    my_rover.edge_vals.left = 0;
    my_rover.edge_vals.right = 0;
    my_rover.edge_vals.mid = 0;
    my_rover.distance = 127;
    my_rover.bools.stop_left = false;
    my_rover.bools.stop_right = false;
    my_rover.bools.slow_left = false;
    my_rover.bools.slow_right = false;
    my_rover.bools.got_name = false;
    my_rover.bools.test_move = false;
    my_rover.bools.test_rotate = false;
    my_rover.bools.rotate = true;
    my_rover.bools.grabbed = false;
    my_rover.bools.dropped = false;
    my_rover.bools.moved_right = false;
    my_rover.bools.straight = false;
    my_rover.bools.rotate_correct = false;
}

void path_init()
{
	path_index = 0;
	edges_size = 0;
	target_list[0] = 80080; //its at position <80,80>
    target_size += 1;
    target_list[1] = 30085; //its at position <30,85>
    target_size += 1;
}
void map_init()
{
    int i;
 	int j;
    for (i = 0; i < X; i++)
	{
		//go through y
		for (j = 0; j < Y; j++)
		{
			my_world[i][j].difficulty = 65535;
            my_world[i][j].weight = MAX_WEIGHT;
		}
	}
//	set_block(55, 40);
//	set_block(75, 40);
//	set_block(60, 58);
	//set_block(40, 35);
   //set_target();
}
void path_find_index_min(int *index, unsigned int *min, unsigned short int x, unsigned short int y)
{
	*index = (x)*OFFSET + y;
	*min = my_world[x][y].difficulty;
}
void world_diff(unsigned short int *world_diff, int diff_plus_weight, unsigned short int weight)
{
	if (diff_plus_weight > 65535 ||
		weight == 99)
		*world_diff = 65535;
	else
		*world_diff = diff_plus_weight;

}
void init_world_diff()
{
    	//go through x
    int i,j;
	for (i = 0; i < X; i++)
	{
		//go through y
		for (j = 0; j < Y; j++)
		{
			my_world[i][j].difficulty = 65535;
		}
	}
}

int my_path(uint8_t t_index)
{
    NSMessage s_message;
	int x = my_rover.xy_points.x;
	int y = my_rover.xy_points.y;
    int index = x*OFFSET+y;
	//just in case
	if (x == target_list[t_index] / OFFSET && y == target_list[t_index] % OFFSET){
		find = true;
	}
    int target = target_list[t_index];
    unsigned int min = 0;
    uint8_t path_offset = 6;
    //go east first
    switch(my_rover.ori)
    {
        case NORTH:
        {
            if (y + 1 < Y && (find || (!go_home)))
            {
                path_find_index_min(&index, &min, x, y+1);
            }
            else
                path_find_index_min(&index, &min,x, y-1);
                
        }break;
        case SOUTH:
        {
            if (y - 1 >= 0 && (find || (!go_home)))
                path_find_index_min(&index, &min,x, y-1);
            else
                path_find_index_min(&index, &min, x, y+1);
        }break;
        case EAST:
        {
            if (x + 1 < X && (find ||(!go_home)))
                path_find_index_min(&index, &min, x+1, y);
            else
                 path_find_index_min(&index, &min, x - 1, y);
        }break;
        case WEST:
        {
            if (x - 1 >= 0 && (find ||(!go_home)))
                path_find_index_min(&index, &min, x - 1, y);
            else
                path_find_index_min(&index, &min, x+1, y);
        }break;
        default:
        {
            index = (x)*OFFSET + y+1;
            min = my_world[x][y+1].difficulty;
        }break;       
    }

    //********NORTH***************
    if (y + 1 < Y && (find || (!go_home))&& my_world[x][y + 1].difficulty < min)
    {
        path_find_index_min(&index, &min, x, y+1);
    }
    //to check if we wanna go west
    //******************WEST******************
   if (x - 1 >= 0 && (find || (!go_home)) && my_world[x - 1][y].difficulty < min)
    {
        path_find_index_min(&index, &min, x -1, y);
    }
    /***********EAST***********/
   if (x + 1 < X && (find || (!go_home)) && my_world[x + 1][y].difficulty < min)
    {
        path_find_index_min(&index, &min, x + 1, y);
    }
    //**********SOUTH**********
    if (y - 1 >= 0 && (find || (!go_home)) && my_world[x][y - 1].difficulty < min)
    {
        path_find_index_min(&index, &min,x, y-1);
    }


    unsigned short int offset_bias = 7;
    if ((index / OFFSET < target_list[target_index] / OFFSET + offset_bias &&
        index / OFFSET > target_list[target_index] / OFFSET - offset_bias &&
        index % OFFSET < target_list[target_index] % OFFSET + offset_bias &&
        index % OFFSET > target_list[target_index] % OFFSET - offset_bias) &&
        !go_home){
        go_home = true;
        find = true;
    }
                s_message.type = NS_ROVER_DATA;
                    s_message.data.rd.point.x = index/OFFSET;
                    s_message.data.rd.point.y = index%OFFSET;
                    s_message.data.rd.ori = 0;
                    s_message.data.rd.target = min;
                    network_send_add_message(&s_message);
    if (index / OFFSET ==  my_rover.home.x && index % OFFSET == my_rover.home.y && go_home) 
        home = true;
	return index;
}
void find_path(bool home)
{
    NSMessage s_message;
    if(home)
    {
        my_world[my_rover.home.x][my_rover.home.y].difficulty = 0;
        my_world[my_rover.home.x][my_rover.home.y].weight = 0;
        EDGES[0].x = my_rover.home.x;//<80,80>
        EDGES[0].y = my_rover.home.y;
    }
    else
    {
        uint16_t temp_x = target_list[target_index] / OFFSET;
        uint16_t temp_y = target_list[target_index] % OFFSET;
        my_world[target_list[target_index] / OFFSET][target_list[target_index] % OFFSET].difficulty = 0;
        my_world[target_list[target_index] / OFFSET][target_list[target_index] % OFFSET].weight = 0;
        EDGES[0].x = target_list[target_index] / OFFSET;//<80,80>
        EDGES[0].y = target_list[target_index] % OFFSET;
        
        s_message.type = NS_ROVER_DATA;
        s_message.data.rd.point.x = target_list[target_index] / OFFSET;
        s_message.data.rd.point.y = my_world[80][81].weight;
        s_message.data.rd.ori = 0;
        s_message.data.rd.target = my_world[81][80].weight;
        network_send_add_message(&s_message);
    }
	edges_size = 1;
	unsigned short int world_diff_n;
	unsigned short int world_diff_s;
	unsigned short int world_diff_e;
	unsigned short int world_diff_w;
    
	while (edges_size)
	{
		int j = 0;
		int i;
		for (i = 0; i < edges_size; i++)
		{
			int x = EDGES[i].x; //the current tile
			int y = EDGES[i].y; //the current tile
			world_diff(&world_diff_n, (int)(my_world[x][y].difficulty + my_world[x][y + 1].weight), my_world[x][y + 1].weight);
			world_diff(&world_diff_s, (int)(my_world[x][y].difficulty + my_world[x][y - 1].weight), my_world[x][y - 1].weight);
			world_diff(&world_diff_e, (int)(my_world[x][y].difficulty + my_world[x + 1][y].weight), my_world[x + 1][y].weight);
			world_diff(&world_diff_w, (int)(my_world[x][y].difficulty + my_world[x - 1][y].weight), my_world[x - 1][y].weight);
            //NORTH
            if (y+1 < Y && world_diff_n < my_world[x][y + 1].difficulty)
			{
				NEXT_EDGES[j].y = EDGES[i].y + 1; //for north
				NEXT_EDGES[j].x = EDGES[i].x;
				my_world[x][y + 1].difficulty = world_diff_n;
				j++;
			}
            //SOUTH
			if (y - 1 >= 0 && world_diff_s < my_world[x][y - 1].difficulty)
			{
				NEXT_EDGES[j].y = EDGES[i].y - 1; //for south
				NEXT_EDGES[j].x = EDGES[i].x;
				my_world[x][y - 1].difficulty = world_diff_s;
				j++;
			}
            //EAST
			if (x+1 < X  && world_diff_e < my_world[x+1][y].difficulty)
			{
				NEXT_EDGES[j].x = EDGES[i].x + 1; //for east
				NEXT_EDGES[j].y = EDGES[i].y;
				my_world[x+1][y].difficulty = world_diff_e;
				j++;
			}
            //WEST
			if (x-1 >= 0 && world_diff_w < my_world[x-1][y].difficulty)
			{
				NEXT_EDGES[j].x = EDGES[i].x - 1; //for west
				NEXT_EDGES[j].y = EDGES[i].y;
				my_world[x-1][y].difficulty = world_diff_w;
				j++;
			}
		}
		int k;
		for (k = 0; k < j; k++)
        {
			EDGES[k].x = NEXT_EDGES[k].x;
			EDGES[k].y = NEXT_EDGES[k].y;
        }
		edges_size = j;
		//print_world();
		//print_edges();
	}
}

void change_direction(unsigned turn_left, uint16_t degree, orientation dir)
{
    pwm_to_isr pwm_cd;
    pwm_cd.wanted_speed_right = 1.1e-1;
    pwm_cd.wanted_speed_left = 1.1e-1;
    interrupt_add_pwm(&pwm_cd);
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
void processing_add_pwm_reading(uint16_t left_pwm, uint16_t right_pwm,uint8_t tmr3, uint8_t tmr4, double left_error, double right_error){
    
    if(my_rover.bools.test_move)
    {
        uint32_t temp_left = my_rover.ticks.tick_left;
        uint32_t temp_right = my_rover.ticks.tick_right;
        my_rover.ticks.tick_left += tmr4;
        my_rover.ticks.tick_right += tmr3;
        my_rover.ticks.prev_tick_left = temp_left;
        my_rover.ticks.prev_tick_right = temp_right;
        if(my_rover.rover_state == ROVER_EDGE_DETECT)
        {
            my_rover.ticks.tick_edge_detect_right += tmr3;
            my_rover.ticks.tick_edge_detect_left += tmr4;
        }
        else if(my_rover.rover_state == ROVER_EDGE_DETECT_BACK)
        {
            if(my_rover.ticks.tick_edge_detect_right)
                my_rover.ticks.tick_edge_detect_right -= tmr3;
            if(my_rover.ticks.tick_edge_detect_left)
                my_rover.ticks.tick_edge_detect_left -= tmr4;
        }
        if(my_rover.rover_state == ROVER_DISTANCE)
        {
            my_rover.ticks.tick_distance_right += tmr3;
            my_rover.ticks.tick_distance_left += tmr4;
        }
        else if(my_rover.rover_state == ROVER_DISTANCE_BACK)
        {
            if(my_rover.ticks.tick_distance_right)
                my_rover.ticks.tick_distance_right -= tmr3;
            if(my_rover.ticks.tick_distance_left)
                my_rover.ticks.tick_distance_left -= tmr4;
        }
        if(my_rover.rover_state == ROVER_ROTATE_CORRECTION)
        {
            if(my_rover.ticks.rotate_tick_right)
                my_rover.ticks.tick_distance_right -= tmr3;
            if(my_rover.ticks.rotate_tick_left)
                my_rover.ticks.tick_distance_left -= tmr4;
        }
        output_left_avg += left_pwm;
        output_right_avg += right_pwm;
        processing_counter++;
        if(processing_counter == 100)
        {
            processing_counter = 0;
            output_left_avg = output_left_avg/100;
            output_right_avg = output_right_avg/100;
            PRMessage pr_adc_message;
            pr_adc_message.type = PR_PWM;
            pr_adc_message.data.timer.speed_left = output_left_avg;
            pr_adc_message.data.timer.speed_right = output_right_avg;
            pr_adc_message.data.timer.tmr3 = tmr3;
            pr_adc_message.data.timer.tmr4 =  tmr4;
            pr_adc_message.data.timer.left_error =left_error;
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
    processing_queue = xQueueCreate(PROCESSING_QUEUE_LEN, sizeof(PRMessage));
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
   int wait_grabbed = 0;
   int wait_ramp_rotate = 0;
   int wait_to_send_req_row = 0;
   uint16_t rows = 50;
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
                     case NR_TEST_ROW:
                     {
                         int i;
                         send_message.type = NS_TEST_ROW;
                         network_send_add_message(&send_message);
                     }break;
                     case NR_GRABBED:
                     {
                         my_rover.bools.grabbed = recv_message.data.nr_message.data.answer;
                     }break;
                     case NR_DROPPED:
                     {
                         my_rover.bools.dropped = recv_message.data.nr_message.data.answer;
                     }break;
                     case NR_HALF_ROW:
                     {
                         int i;
                        for(i = 0; i < 64; i++)
                        {
                            my_world[((rows%2)*64)+i][rows >> 1].weight = recv_message.data.nr_message.data.w_array[i];
//                            if(((rows%2)*64)+i == 80)
//                            {
//                                send_message.type = NS_ROVER_DATA;
//                                send_message.data.rd.point.x = target_list[target_index] / OFFSET;
//                                send_message.data.rd.point.y = my_world[80][81].weight;
//                                send_message.data.rd.ori = 1;
//                                send_message.data.rd.target = 400;
//                                network_send_add_message(&send_message);
//                            }
                        }
                         
                        send_message.type = NS_JC_REQ_HALF_ROW;
                        send_message.data.row_req = ++rows;
                        if(rows < 206)
                            network_send_add_message(&send_message);
                        wait_to_send_req_row = 0;                            
                     }break;
                     case NR_EDGE_DETECT:
                     {
                         my_rover.edge_vals.left = recv_message.data.nr_message.data.edge.left;
                         my_rover.edge_vals.mid = recv_message.data.nr_message.data.edge.mid;
                         my_rover.edge_vals.right = recv_message.data.nr_message.data.edge.right;
                     }break;
                     case NR_DISTANCE:
                     {
                         my_rover.distance = recv_message.data.nr_message.data.distance;
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
                if(my_rover.bools.test_move)
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
        //START OF SWITCH
        switch(my_rover.rover_state)
        {
            case ROVER_INIT:
            {
                my_rover.ori = NORTH;
                my_rover.ticks.tick_right = 0;
                my_rover.ticks.tick_left = 0;
                wait_to_send_req_row++;
                if(wait_to_send_req_row == 6)
                {
                    send_message.type = NS_JC_REQ_HALF_ROW;
                    send_message.data.row_req = rows;
                    network_send_add_message(&send_message);
                    
                    wait_to_send_req_row = 0;
                }               
                
//**********************************************************************************
//************************BELOW IS FOR DEBUG****************************************
//                    if(my_rover.bools.test_move)                          //******
//                        my_rover.rover_state = ROVER_BLOCK;                //******
//                    if(my_rover.bools.test_rotate)                        //******
//                    {                                                     //******
//                        my_rover.rover_state = ROVER_MOVE;                //******
//                        change_direction(0,1, my_rover.next_ori);         //******
//                    }                                                     //******
//**********************************************************************************
//**********************************************************************************
                
                //if you wanna test rotation via debug assign next_ori and current_ori and go to state ROVER_MOVE
                //if((my_rover.bools.test_rotate || my_rover.bools.test_move))
                if(rows == 206)// && (my_rover.bools.test_rotate || my_rover.bools.test_move))
                {
                    blocks = 0;
                    my_rover.rover_state = ROVER_FIND_PATH;
                }
            }break;
            case ROVER_FIND_PATH:
            {
                network_send_add_message(&send_message);
                pwm.wanted_speed_right =0;
                pwm.wanted_speed_left = 0;
                interrupt_add_pwm(&pwm);
                path_index = 0;
                find_path(go_home);
                my_rover.rover_state = ROVER_FIND_DIR;
            }break;
            case ROVER_FIND_DIR:
            {
                path_point = my_path(target_index);
                int16_t x = path_point/OFFSET - my_rover.xy_points.x;
                int16_t y = path_point%OFFSET - my_rover.xy_points.y;
                
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
                my_rover.rover_state = ROVER_MOVE;
            }break;
             case ROVER_MOVE:
            {   
                my_rover.ticks.tick_right = 0;
                my_rover.ticks.tick_left = 0;
                pwm.wanted_speed_right = 2e-1;
                pwm.wanted_speed_left = 2e-1;
                
                //move_wheels(0,1);//left rotate
                
                my_rover.rover_state = ROVER_BLOCK;
                if(my_rover.next_ori != my_rover.ori){
                    my_rover.rover_state = ROVER_RAMP_ROTATE;
                    pwm.wanted_speed_right = 1.00e-1;
                    pwm.wanted_speed_left = 1.00e-1;
                    if(go_home)
                    {
                        pwm.wanted_speed_right = 0.00e-1;
                        pwm.wanted_speed_left = 0.00e-1;
                        
                    }
                }
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_RAMP_ROTATE:
            {
                pwm.wanted_speed_right = (0.5e-1) - (double)wait_ramp_rotate/100.0;
                pwm.wanted_speed_left = (0.5e-1) - (double)wait_ramp_rotate/100.0;
                interrupt_add_pwm(&pwm);
                wait_ramp_rotate++;
                if(wait_ramp_rotate == 5)
                {
                    my_rover.rover_state = ROVER_ENSURE_STOP;
                    wait_ramp_rotate = 0;
                }
            }break;
            case ROVER_ENSURE_STOP:
            {
                if(my_rover.ticks.tick_right == my_rover.ticks.prev_tick_right && 
                        my_rover.ticks.tick_left == my_rover.ticks.prev_tick_left)
                {
                    stopped_counter++;
                }
                if(stopped_counter == 10)
                {
                    stopped_counter = 0;
                    my_rover.rover_state = MY_ROVER_CHANGE_ORI;
                    my_rover.ticks.tick_right = 0;
                    my_rover.ticks.tick_left = 0;
                }
            }break;
            case MY_ROVER_CHANGE_ORI:
            {
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
                            change_direction(1,1, my_rover.next_ori);
                        else if(my_rover.next_ori == WEST)
                            change_direction(0,1, my_rover.next_ori);
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
                my_rover.rover_state = ROVER_ROTATE;
            }break;
            //
            case ROVER_ROTATE:
            {
                //unsigned rotate = 60;
                if((my_rover.ticks.tick_right) >= my_rover.value.rotate_val-10)
                {                 
                    my_rover.ticks.tick_right = 0;
                    my_rover.bools.stop_right = true;
                    my_rover.ticks.rotate_tick_right = my_rover.ticks.tick_right;
                }
                if((my_rover.ticks.tick_left) >= my_rover.value.rotate_val-10)
                {         
                    my_rover.ticks.tick_left= 0;
                    my_rover.ticks.rotate_tick_left = my_rover.ticks.tick_left;
                    my_rover.bools.stop_left = true;
                }
                
                if(my_rover.bools.stop_left && my_rover.bools.stop_right){
//**********************************************************************************                    
//*******************************DEBUG**********************************************
//                    my_rover.rover_state = ROVER_STOP;                    //******
//**********************************************************************************
//************************UNCOMMENT BELOW FOR ORIGINAL******************************
                    my_rover.rover_state = ROVER_RAMP;                     //******
//**********************************************************************************
                    my_rover.ticks.tick_left= 0;
                    my_rover.ticks.tick_right= 0;
                    my_rover.bools.stop_left = false;
                    my_rover.bools.stop_right = false;
                    pwm.wanted_speed_right = 1.0e-1;
                    pwm.wanted_speed_left = 1.0e-1;
                    move_wheels(0,0); 
                }
                else
                {
                    pwm.wanted_speed_left = 1.1e-1;
                    pwm.wanted_speed_right = 1.1e-1;
                    if(my_rover.ticks.tick_left >= my_rover.value.rotate_val - 200)
                        pwm.wanted_speed_left = 1.3e-1;
                    if(my_rover.ticks.tick_right >= my_rover.value.rotate_val - 200)
                        pwm.wanted_speed_right = 1.3e-1;
                }
                
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_RAMP:
            {
                pwm.wanted_speed_right = (0.5e-1) - (double)wait_ramp_rotate/100.0;
                pwm.wanted_speed_left = (0.5e-1) - (double)wait_ramp_rotate/100.0;
                interrupt_add_pwm(&pwm);
                wait_ramp_rotate++;
                if(wait_ramp_rotate == 10)
                {
                    my_rover.rover_state = ROVER_ENSURE_STOP2;
                    wait_ramp_rotate = 0;
                }
            }break;
            case ROVER_ENSURE_STOP2:
            {
                if(my_rover.ticks.tick_right == my_rover.ticks.prev_tick_right && 
                        my_rover.ticks.tick_left == my_rover.ticks.prev_tick_left)
                {
                    stopped_counter++;
                }
                if(stopped_counter == 10)
                {
                    stopped_counter = 0;
//                    my_rover.rover_state = ROVER_ROTATE_CORRECTION;
//                    if(my_rover.bools.rotate_correct)
//                    {
                        pwm.wanted_speed_right = 1.3e-1;
                        pwm.wanted_speed_left = 1.3e-1;
                        interrupt_add_pwm(&pwm);
                        my_rover.rover_state = ROVER_BLOCK;
                        my_rover.bools.rotate_correct = false;
//                    }
                        
                    my_rover.ticks.tick_right = 0;
                    my_rover.ticks.tick_left = 0;
                }   
            }break;
            //***********************************
            //TODO: Move this all into a function + create the move block functions
            //***********************************
            case ROVER_BLOCK:
            {
                pwm.wanted_speed_right = 2e-1;
                pwm.wanted_speed_left = 2e-1;
                if((my_rover.ticks.tick_right) >= 32)
                {                 
                    my_rover.ticks.tick_right = 0;
                    my_rover.bools.stop_right = true;
                }
                //original was TEN_CM(1)
                if((my_rover.ticks.tick_left)>= 32)
                {
                    my_rover.ticks.tick_left= 0;
                    my_rover.bools.stop_left = true;
                }
                
//**********************************************************************************
//***********************THIS IS FOR DEBUG******************************************
//                 if(my_rover.bools.stop_left && my_rover.bools.stop_right) //*****
//                {                                                          //*****
//                    my_rover.ticks.tick_left= 0;                           //*****
//                    my_rover.ticks.tick_right= 0;                          //*****
//                     my_rover.rover_state = ROVER_STOP;                    //*****
//                     if(my_rover.debug_test.test_move_val--)               //*****
//                         my_rover.rover_state = ROVER_BLOCK;               //*****
//                }                                                          //*****
//**********************************************************************************
//***********************UNCOMMENT BELOW FOR ACTUAL*********************************
                if(my_rover.bools.stop_left && my_rover.bools.stop_right)
                {
                    my_rover.xy_points.x = path_point/OFFSET;
                    my_rover.xy_points.y = path_point%OFFSET;
                    
                    //replace path_index to 1
                    //This needs to change to g
                   
                    if(go_home && home)
                    {
                        my_rover.rover_state = ROVER_STOP;
                        pwm.wanted_speed_right = 0;
                        pwm.wanted_speed_left = 0;
                        find = false;
                        home = false;
                        go_home = false;
                        target_index++;
                        init_world_diff();
                        if(target_index < target_size)
                        {
                            my_rover.rover_state = ROVER_WAIT_DROPPED;
                        }
                        my_rover.xy_points.x = path_point/OFFSET;
                        my_rover.xy_points.y = path_point%OFFSET;
                    }
                    else if(!find)
                    {
                        my_rover.rover_state = ROVER_FIND_DIR;
                    }
                    else
                    {
                        my_rover.rover_state = ROVER_GO_HOME;
                        if(!my_rover.bools.grabbed)
                        {
                            my_rover.ticks.tick_edge_detect_right= 0;
                            my_rover.ticks.tick_edge_detect_left = 0;
                            my_rover.rover_state = ROVER_EDGE_DETECT;
                            pwm.wanted_speed_right = 1.0e-1;
                            pwm.wanted_speed_left = 1.0e-1;
                        }
                        home = false;
                    }

                    my_rover.ticks.tick_left= 0;
                    my_rover.ticks.tick_right= 0;
                    my_rover.bools.stop_left = false;
                    my_rover.bools.stop_right = false;
                }
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_EDGE_DETECT:
            {
                //this is turn EAST
                if(my_rover.ori == EAST || my_rover.ori == WEST)
                {
                    if(my_rover.xy_points.y == target_list[target_index]%OFFSET)
                    {
                        my_rover.bools.moved_right = false;
                        pwm.wanted_speed_right = 0;
                        pwm.wanted_speed_left = 0;
                        my_rover.ticks.tick_edge_detect_right= 0;
                        my_rover.ticks.tick_edge_detect_left = 0;
                        my_rover.bools.straight = true;
                        move_wheels(0,0);
                        my_rover.rover_state = ROVER_DISTANCE;
                    }
                    else if(my_rover.xy_points.y < target_list[target_index]%OFFSET)
                    {
                        //turn left
                        my_rover.bools.moved_right = false;
                        move_wheels(0,1);
                    }
                    else
                    {
                        my_rover.bools.moved_right = true;
                        move_wheels(1,0);
                    }
                }
                else //orientation is up or down
                {
                    if(my_rover.xy_points.x == target_list[target_index]/OFFSET)
                    {
                        my_rover.bools.moved_right = false;
                        my_rover.bools.straight = true;
                        move_wheels(0,0);
                        pwm.wanted_speed_right = 0;
                        pwm.wanted_speed_left = 0;
                        my_rover.ticks.tick_edge_detect_right= 0;
                        my_rover.ticks.tick_edge_detect_left = 0;
                        my_rover.rover_state = ROVER_DISTANCE;
                    }
                     else if(my_rover.xy_points.x < target_list[target_index]/OFFSET)
                    {
                        //turn right
                        move_wheels(1,0);
                        my_rover.bools.moved_right = true;
                    }
                    else
                    {
                        my_rover.bools.moved_right = false;
                        move_wheels(0,1);
                    }
                }
                 uint8_t wanted_val = 30;
                 if(!my_rover.bools.straight)
                 {
                    if(my_rover.edge_vals.left == wanted_val && my_rover.edge_vals.mid == wanted_val && my_rover.edge_vals.right == wanted_val)
                    {
                        pwm.wanted_speed_right = 0;
                        pwm.wanted_speed_left = 0;
                        my_rover.rover_state = ROVER_ENSURE_STOP3;
                        move_wheels(0,0);
                    }
                    else if(my_rover.edge_vals.left <= wanted_val+20 ||
                            my_rover.edge_vals.mid <= wanted_val+20 ||
                            my_rover.edge_vals.right <= wanted_val+20)
                    {
                        pwm.wanted_speed_right = 0.5e-1;
                        pwm.wanted_speed_left = 0.5e-1;
                    }
                    else if(my_rover.edge_vals.left <= wanted_val+30 ||
                            my_rover.edge_vals.mid <= wanted_val+30 ||
                            my_rover.edge_vals.right <= wanted_val+30)
                    {
                        pwm.wanted_speed_right = 1.0e-1;
                        pwm.wanted_speed_left = 1.0e-1;
                    }
                    else if(my_rover.edge_vals.left >= wanted_val+50 &&
                            my_rover.edge_vals.mid >= wanted_val+50 &&
                            my_rover.edge_vals.right >= wanted_val+50)
                    {
                        my_rover.ticks.tick_edge_detect_right= 0;
                        my_rover.ticks.tick_edge_detect_left = 0;
                        pwm.wanted_speed_right = 0;
                        pwm.wanted_speed_left = 0;
                    }
                 }
                 
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_ENSURE_STOP3:
            {
                if(my_rover.ticks.tick_right == my_rover.ticks.prev_tick_right && 
                       my_rover.ticks.tick_left == my_rover.ticks.prev_tick_left)
               {
                   stopped_counter++;
               }
               if(stopped_counter == 10)
               {
                   stopped_counter = 0;
                   my_rover.rover_state = ROVER_DISTANCE;
                   my_rover.ticks.tick_right = 0;
                   my_rover.ticks.tick_left = 0;
               }   
            }break;
            case ROVER_DISTANCE:
            {
                if (my_rover.distance < 30)
                {
                    pwm.wanted_speed_right = 0.0e-1;
                    pwm.wanted_speed_left = 0.0e-1;
                    my_rover.rover_state = ROVER_WAIT_GRAB;
                }
                else if (my_rover.distance < 50)
                {
                    pwm.wanted_speed_right = 0.5e-1;
                    pwm.wanted_speed_left = 0.5e-1;
                }
                else if(my_rover.distance < 100 && my_rover.distance > 80)
                {
                     pwm.wanted_speed_right = 1.0e-1;
                     pwm.wanted_speed_left = 1.0e-1;
                }
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_WAIT_DROPPED:
            {
                pwm.wanted_speed_right = 0;
                pwm.wanted_speed_left = 0;
                wait_grabbed++;
                if(my_rover.bools.dropped){
                    my_rover.rover_state = ROVER_FIND_PATH;
                    wait_grabbed = 0;
                    my_rover.bools.grabbed = false;
                    my_rover.bools.dropped = false;
                }
                else if(wait_grabbed == 300)
                {
                    wait_grabbed = 0;
                    send_message.type = NS_TEST_REQ_DROPPED;
                    network_send_add_message(&send_message);
                }
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_WAIT_GRAB:
            {
                pwm.wanted_speed_right = 0;
                pwm.wanted_speed_left = 0;
                wait_grabbed++;
                if(my_rover.bools.grabbed){
                    my_rover.rover_state = ROVER_DISTANCE_BACK;
                    move_wheels(1,1);
                    wait_grabbed = 0;
                }
                else if(wait_grabbed == 300)
                {
                    wait_grabbed = 0;
                    send_message.type = NS_TEST_REQ_GRABBED;
                    network_send_add_message(&send_message);
                }
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_DISTANCE_BACK:
            {
                pwm.wanted_speed_right = 1.0e-1;
                pwm.wanted_speed_left = 1.0e-1;
                if(my_rover.ticks.tick_distance_right == 0)
                    my_rover.bools.stop_right = true;
                
                if(my_rover.ticks.tick_distance_left == 0)
                    my_rover.bools.stop_left = true;
                if(my_rover.bools.stop_left && my_rover.bools.stop_right)
                {
                    pwm.wanted_speed_right = 0;
                    pwm.wanted_speed_left = 0;
                    my_rover.bools.stop_right = false;
                    my_rover.bools.stop_left = false;
                    my_rover.bools.straight = false;
                     my_rover.ticks.tick_distance_right= 0;
                     my_rover.ticks.tick_distance_left = 0;
                    my_rover.rover_state = ROVER_EDGE_DETECT_BACK;
                }
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_EDGE_DETECT_BACK:
            {
                pwm.wanted_speed_right = 1.2e-1;
                pwm.wanted_speed_left = 1.2e-1;
                
                move_wheels(1,0);
                if(my_rover.bools.moved_right)
                    move_wheels(0,1);

                if(my_rover.ticks.tick_edge_detect_right == 0)
                    my_rover.bools.stop_right = true;
                
                if(my_rover.ticks.tick_edge_detect_left == 0)
                    my_rover.bools.stop_left = true;
                if(my_rover.bools.stop_left && my_rover.bools.stop_right)
                {
                    pwm.wanted_speed_right = 0;
                    pwm.wanted_speed_left = 0;
                    my_rover.bools.stop_right = false;
                    my_rover.bools.stop_left = false;
                     my_rover.ticks.tick_edge_detect_right= 0;
                     my_rover.ticks.tick_edge_detect_left = 0;
                    my_rover.rover_state = ROVER_ENSURE_STOP4;
                    move_wheels(0,0); 
                }
                interrupt_add_pwm(&pwm);
            }break;
            case ROVER_ENSURE_STOP4:
            {
                if(my_rover.ticks.tick_right == my_rover.ticks.prev_tick_right && 
                        my_rover.ticks.tick_left == my_rover.ticks.prev_tick_left)
                {
                    stopped_counter++;
                }
                if(stopped_counter == 10)
                {
                    stopped_counter = 0;
                    my_rover.rover_state = ROVER_GO_HOME;
                    my_rover.ticks.tick_right = 0;
                    my_rover.ticks.tick_left = 0;
                }   
            }break;
            
            //***********************************
            //TODO: Move this all into a function + create the rotate functions
            //TODO2: Make the network signals for ROVER_ROTATE_R and ROVER_ROTATE_L -- mainly for debugging
            //***********************************
            case ROVER_GO_HOME:
            {
                my_rover.ticks.tick_left= 0;
                my_rover.ticks.tick_right= 0;
                init_world_diff();
                find_path(go_home);
                path_point = my_path(target_index);
                int16_t x = path_point/OFFSET - my_rover.xy_points.x;
                int16_t y = path_point%OFFSET - my_rover.xy_points.y;
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
                my_rover.rover_state = ROVER_MOVE;
              //  my_rover.rover_state = ROVER_STOP;
            }break;
            case ROVER_STOP:
            {
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
