#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <rmw_uros/options.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <semphr.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

int i;
uint16_t cs = 0;

void kobuki_set_speed_command(float translation, float rotation);
void kobuki_loop();
geometry_msgs__msg__Twist cmd_vel;


SemaphoreHandle_t xSemaphore = NULL;

#define KOBUKI_WHEELBASE 0.230
#define KOBUKI_RATE_MS 20

int16_t control_frame[]={0xAA,0x55,0x05,0x01,0x04,0x00,0x00,0x00,0x00,0x00};

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}


#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel;

void subscription_callback(const void *msgin)
{
    kobuki_set_speed_command(0.0, 0.0);
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    int int_value = msg->linear.x;
	//printf("Received: %d ", int_value);
	kobuki_set_speed_command(msg->linear.x, msg->angular.z);
	
}
    


void appMain(void * arg)
{
	
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node=rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "MicroROS_kobuki", "", &support));

	// create subscriber
	RCCHECK(rclc_subscription_init_best_effort(
		&cmd_vel_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel, &subscription_callback, ON_NEW_DATA));

    	TaskHandle_t kobuki_handle = NULL;
        xTaskCreate(kobuki_loop, "kobuki_thread", 4000, NULL,  23, &kobuki_handle); 
	while(1){
	                
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			usleep(100000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&cmd_vel_sub, &node));
	RCCHECK(rcl_node_fini(&node));
	
	vTaskDelete(NULL);
}

void kobuki_loop()
{
    
       
    while(1)
     {
         
     	 for(int i=0;i<10;i++)
      	 {
      	 printf("%c",control_frame[i]);
      	 }
      	 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      usleep(1000);
      //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      //usleep(5000);	 
     }
          
     vTaskDelete(NULL);    
}


void kobuki_set_speed_command(float translation, float rotation)
{
    
    
    int16_t * speed = (int16_t *) &control_frame[5];
    int16_t * radius = (int16_t *) &control_frame[7];
        
    // convert to mm;
    translation *= 1000;
    float b2 = KOBUKI_WHEELBASE * 500.0;
         
    if (fabs(translation) < 1)
    {
        //Pure rotation
        *radius = 1;
        *speed =  (int16_t) (rotation * b2);
    } 
    else if (fabs(rotation) < 1e-3 )
     {
        //Pure translation
        *speed = (int16_t) translation;
        *radius = 0;
    }
    else 
    {
        //Translation and rotation
        float r = translation/rotation;
        *radius = (int16_t) r;
        if (r > 1) 
        {
            *speed = (int16_t) (translation * (r + b2)/ r);
        } else if (r < -1) 
        {
            *speed = (int16_t) (translation * (r - b2)/ r);
        }
    }
    for(int i=2;i<9;i++)
   {
   cs^=control_frame[i];  
   
   }
   cs += 0x03;
   control_frame[9]=cs; 
   
      
}

   	 


