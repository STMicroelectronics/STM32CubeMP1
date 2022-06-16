#ifdef __cplusplus
 extern "C" {
#endif
/**
  ******************************************************************************
  * @file           : app_x-cube-ai.c
  * @brief          : AI program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "network.h"
#include "network_data.h"
#include "app_x-cube-ai.h"

static ai_buffer net_in[AI_NETWORK_IN_NUM]  = {{AI_NETWORK_IN_1_SIZE}};
static ai_buffer net_out[AI_NETWORK_OUT_NUM] = {{AI_NETWORK_OUT_1_SIZE}};
static ai_handle network = AI_HANDLE_NULL;
static uint8_t g_net_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
static float g_ai_output[AI_NETWORK_OUT_1_SIZE];

/*************************************************************************
  *
  */
void MX_X_CUBE_AI_Init(void)
{
  /* USER CODE BEGIN 0 */
  ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  ai_network_params net_params = AI_NETWORK_PARAMS_INIT(AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
		  	  	  	  	  	  	  	  	  	  	  	    AI_NETWORK_DATA_ACTIVATIONS(g_net_activations));
  ai_network_init(network, &net_params);
  /* USER CODE END 0 */
}

/*
 * Equalizer background task
 */
char MX_X_CUBE_AI_Process(float *image_28x28, uint8_t *accuracy, uint8_t input_type)
{ 
  /* USER CODE BEGIN 1 */
  char prediction;
  float max = 0.0F;
  int32_t imax = -1;
  int ii;
  net_in[0].data = AI_HANDLE_PTR(image_28x28);
  net_in[0].n_batches = 1;
  net_out[0].n_batches = 1;
  net_out[0].data = AI_HANDLE_PTR(g_ai_output);

  ai_network_run(network, &net_in[0], &net_out[0]);

  for(ii=0;ii<AI_NETWORK_OUT_1_SIZE;ii++) {
	if( g_ai_output[ii] > max ) { max = g_ai_output[ii]; imax = ii; }
  }

  if(input_type == INPUT_IS_DIGIT) {
	log_info("DIGIT\n");
    if(imax == 24) imax = 0;      // O -> 0
    else if(imax == 18) imax = 1; // I -> 1
    else if(imax == 16) imax = 6; // G -> 6
    else if(imax == 28) imax = 5; // S -> 5
    else if(imax == 35) imax = 2; // Z -> 2
    else if(imax > 9) max = 0.0F;
  } else if (input_type == INPUT_IS_LETTER) {
	log_info("LETTER\n");
    if(imax == 0) imax = 24;      // 0 -> O
    else if(imax == 1) imax = 18; // 1 -> I
    else if(imax == 6) imax = 16; // 6 -> G
    else if(imax == 5) imax = 28; // 5 -> S
    else if(imax == 2) imax = 35; // 2 -> Z
    else if(imax < 10) max = 0.0F;
  } else {
	log_info("BOTH\n");
  }

  if(imax >= 0 && max > 0.5F)
	prediction = (imax<10)?imax+48:imax+55;
  else
	prediction = '?';

  *accuracy = (uint8_t)(max * 100);
log_info("accuracy = %d\n", (uint8_t)(max * 100));

  return prediction;
  /* USER CODE END 1 */
}

#ifdef __cplusplus
}
#endif
