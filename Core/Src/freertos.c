/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "usart.h"

#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
std_msgs__msg__Float32MultiArray encoderValues_msg;
std_msgs__msg__Float32MultiArray referenceValues_msg;

rcl_publisher_t axis_publisher;
//
rcl_subscription_t axis_subscriber;

//
PID_TypeDef Motor_1_PID;
PID_TypeDef Motor_2_PID;
PID_TypeDef Motor_3_PID;
PID_TypeDef Motor_4_PID;
PID_TypeDef Motor_5_PID;
//PID_TypeDef Motor_6_PID;

//float axis_referenceValues[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float PWM_Motor_1 = 0.0;
float PWM_Motor_2 = 0.0;
float PWM_Motor_3 = 0.0;
float PWM_Motor_4 = 0.0;
float PWM_Motor_5 = 0.0;
//float PWM_Motor_6 = 0.0;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;

rcl_timer_t timer_pub;
rcl_timer_t timer_pid;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 4500 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void pwm_1_setvalue(uint16_t value);
void pwm_2_setvalue(uint16_t value);
void pub_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
	encoderValues_msg.data.data[0]++;
	RCSOFTCHECK(rcl_publish(&axis_publisher, &encoderValues_msg, NULL));
  }
}

void pid_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
	    if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	    {
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	    	encoderValues_msg.data.data[0] = 0.0;
	    	encoderValues_msg.data.data[1] = 0.0;
	    	encoderValues_msg.data.data[2] = 0.0;
	    	encoderValues_msg.data.data[3] = 0.0;
	    	encoderValues_msg.data.data[4] = 0.0;
	    	encoderValues_msg.data.data[5] = 0.0;
	    }
	    else
	    {
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	    	// Save encoder value.  Temp cast to signed value, then save as float
	    	uint32_t counter = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
	    	encoderValues_msg.data.data[0] = (int32_t)counter;
	    	counter = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
	    	encoderValues_msg.data.data[1] = (int32_t)counter;
	    	counter = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
	    	encoderValues_msg.data.data[2] = (int32_t)counter;
	    	counter = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
	    	encoderValues_msg.data.data[3] = (int32_t)counter;
	    	counter = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);
	    	encoderValues_msg.data.data[4] = (int32_t)counter;
	    	//encoderValues_msg.data.data[5] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		  PID_Compute(&Motor_1_PID);
		  PID_Compute(&Motor_2_PID);
		  PID_Compute(&Motor_3_PID);
		  PID_Compute(&Motor_4_PID);
		  PID_Compute(&Motor_5_PID);
		  //PID_Compute(&Motor_6_PID);
		  // Set PWMs
		  // for each motor, check direction based on sign of float,
		  // set direction of H-Bridge by setting DIR pin.
		  // Then check End-Switch for chosen direction
		  // If End-Switch active, set PWM to 0
		  // Otherwise set PWM to H-Bridge

		  // Motor 1
		  if (PWM_Motor_1 < 0)
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET)
		  		PWM_Motor_1 = 0;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET)
		  		PWM_Motor_1 = 0;
		  }
		  __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,abs(PWM_Motor_1));

		  // Motor 2
		  if (PWM_Motor_2 < 0)
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 1);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET)
		  		PWM_Motor_2 = 0;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 0);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET)
		  		PWM_Motor_2 = 0;
		  }
		  __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,PWM_Motor_2);

		  // Motor 3
		  if (PWM_Motor_3 < 0)
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == GPIO_PIN_RESET)
		  		PWM_Motor_3 = 0;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == GPIO_PIN_RESET)
		  		PWM_Motor_3 = 0;
		  }
		  __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,PWM_Motor_3);

		  // Motor 4
		  if (PWM_Motor_4 < 0)
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == GPIO_PIN_RESET)
		  		PWM_Motor_4 = 0;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_RESET)
		  		PWM_Motor_4 = 0;
		  }
		  __HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,PWM_Motor_4);

		  // Motor 5
		  if (PWM_Motor_5 < 0)
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, 1);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == GPIO_PIN_RESET)
		  		PWM_Motor_1 = 0;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, 0);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_RESET)
		  		PWM_Motor_1 = 0;
		  }
		  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,PWM_Motor_5);


		  // Motor 5
		  // Note Motor 5 does not use a PID for positioning.  The Gripper here
		  // uses the references values directly as the voltage to be applied
		  // as a (crude) method of force control for gripping.
		  if (referenceValues_msg.data.data[5] < 0)
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, 1);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET)
		  		PWM_Motor_1 = 0;
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, 0);
		  	  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_RESET)
		  		PWM_Motor_1 = 0;
		  }
		  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,referenceValues_msg.data.data[5]);
	    }
  }
}


void axis_subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Float32MultiArray * msgIn = (const std_msgs__msg__Float32MultiArray *)msgin;

  referenceValues_msg.data.data[0] = msgIn->data.data[0];
  referenceValues_msg.data.data[1] = msgIn->data.data[1];
  referenceValues_msg.data.data[2] = msgIn->data.data[2];
  referenceValues_msg.data.data[3] = msgIn->data.data[3];
  referenceValues_msg.data.data[4] = msgIn->data.data[4];
  referenceValues_msg.data.data[5] = msgIn->data.data[5];
}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	encoderValues_msg.data.data[0] = __HAL_TIM_GET_COUNTER(htim);
//}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	  // micro-ROS configuration

	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart3,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app



	  const rosidl_message_type_support_t * type_support =
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);


	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, "robot_HIL_node", "", &support);

	   //create publishers
	    rclc_publisher_init_default(
	      &axis_publisher,
	      &node,
	  	type_support,
	      "axis_positions");

	    encoderValues_msg.data.capacity = 5;
	    encoderValues_msg.data.size = 5;
	    encoderValues_msg.data.data = (float*)malloc(encoderValues_msg.data.capacity *sizeof(float));

	    encoderValues_msg.layout.dim.capacity = 1;
	    encoderValues_msg.layout.dim.size = 0;
	    encoderValues_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(encoderValues_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

	    for(size_t i = 0; i < encoderValues_msg.layout.dim.capacity; i++){
	    	encoderValues_msg.layout.dim.data[i].label.capacity = 1;
	    	encoderValues_msg.layout.dim.data[i].label.size = 1;
	    	encoderValues_msg.layout.dim.data[i].label.data = (char*) malloc(encoderValues_msg.layout.dim.data[i].label.capacity * sizeof(char));
	    	encoderValues_msg.layout.dim.data[i].label.data[0] = 0;
	    }
	    snprintf(encoderValues_msg.layout.dim.data[0].label.data, 11, "Robot Axes");

	    encoderValues_msg.data.data[0] = 0.0;
	    encoderValues_msg.data.data[1] = 0.0;
	    encoderValues_msg.data.data[2] = 0.0;
	    encoderValues_msg.data.data[3] = 0.0;
	    encoderValues_msg.data.data[4] = 0.0;
	    //encoderValues_msg.data.data[5] = 0.0;

	    referenceValues_msg.data.capacity = 6;
	    referenceValues_msg.data.size = 6;
	    referenceValues_msg.data.data = (float*)malloc(referenceValues_msg.data.capacity *sizeof(float));

	    referenceValues_msg.layout.dim.capacity = 1;
	    referenceValues_msg.layout.dim.size = 0;
	    referenceValues_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(referenceValues_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

	    for(size_t i = 0; i < referenceValues_msg.layout.dim.capacity; i++){
	    	referenceValues_msg.layout.dim.data[i].label.capacity = 1;
	    	referenceValues_msg.layout.dim.data[i].label.size = 1;
	    	referenceValues_msg.layout.dim.data[i].label.data = (char*) malloc(referenceValues_msg.layout.dim.data[i].label.capacity * sizeof(char));
	    	referenceValues_msg.layout.dim.data[i].label.data[0] = 0;
	    }
	    snprintf(referenceValues_msg.layout.dim.data[0].label.data, 11, "Robot Axes");

	    referenceValues_msg.data.data[0] = 0.0;
	    referenceValues_msg.data.data[1] = 0.0;
	    referenceValues_msg.data.data[2] = 0.0;
	    referenceValues_msg.data.data[3] = 0.0;
	    referenceValues_msg.data.data[4] = 0.0;
	    referenceValues_msg.data.data[5] = 0.0;


	  // Initialize a reliable subscriber
	  rclc_subscription_init_default(
		&axis_subscriber,
		&node,
		type_support,
		"axis_references");


	  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

	  rclc_executor_init(&executor, &support.context, 3, &allocator);


	  // Add subscription to the executor
	  rclc_executor_add_subscription(
	    &executor, &axis_subscriber, &referenceValues_msg,
	    &axis_subscription_callback, ON_NEW_DATA);


	  const unsigned int pub_timer_timeout = 500;
	  rclc_timer_init_default(
	      &timer_pub,
	      &support,
	      RCL_MS_TO_NS(pub_timer_timeout),
	      pub_callback);
	  rclc_executor_add_timer(&executor, &timer_pub);

	  const unsigned int pid_timer_timeout = 20;
	  rclc_timer_init_default(
	      &timer_pid,
	      &support,
	      RCL_MS_TO_NS(pid_timer_timeout),
	      pid_callback);
	  rclc_executor_add_timer(&executor, &timer_pid);

	  PID(&Motor_1_PID, &(encoderValues_msg.data.data[0]), &PWM_Motor_1, &(referenceValues_msg.data.data[0]), 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	  PID_SetMode(&Motor_1_PID, _PID_MODE_AUTOMATIC);
	  PID_SetSampleTime(&Motor_1_PID, 500);
	  PID_SetOutputLimits(&Motor_1_PID, 1, 100);
	  PID(&Motor_2_PID, &(encoderValues_msg.data.data[1]), &PWM_Motor_2, &(referenceValues_msg.data.data[1]), 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	  PID_SetMode(&Motor_2_PID, _PID_MODE_AUTOMATIC);
	  PID_SetSampleTime(&Motor_2_PID, 500);
	  PID_SetOutputLimits(&Motor_2_PID, 1, 100);
	  PID(&Motor_3_PID, &(encoderValues_msg.data.data[2]), &PWM_Motor_3, &(referenceValues_msg.data.data[2]), 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	  PID_SetMode(&Motor_3_PID, _PID_MODE_AUTOMATIC);
	  PID_SetSampleTime(&Motor_3_PID, 500);
	  PID_SetOutputLimits(&Motor_3_PID, 1, 100);
	  PID(&Motor_4_PID, &(encoderValues_msg.data.data[3]), &PWM_Motor_4, &(referenceValues_msg.data.data[3]), 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	  PID_SetMode(&Motor_4_PID, _PID_MODE_AUTOMATIC);
	  PID_SetSampleTime(&Motor_4_PID, 500);
	  PID_SetOutputLimits(&Motor_4_PID, 1, 100);
	  PID(&Motor_5_PID, &(encoderValues_msg.data.data[4]), &PWM_Motor_5, &(referenceValues_msg.data.data[4]), 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	  PID_SetMode(&Motor_5_PID, _PID_MODE_AUTOMATIC);
	  PID_SetSampleTime(&Motor_5_PID, 500);
	  PID_SetOutputLimits(&Motor_5_PID, 1, 100);


	  for(;;)
	  {

	    // Output to PWMs

	    rclc_executor_spin(&executor);
	  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

