/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_AI_H
#define __APP_AI_H
#ifdef __cplusplus
 extern "C" {
#endif
/**
  ******************************************************************************
  * @file           : app_x-cube-ai.h
  * @brief          : AI entry function definitions
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

#define INPUT_IS_LETTER_AND_DIGIT  0
#define INPUT_IS_LETTER            1
#define INPUT_IS_DIGIT             2

void MX_X_CUBE_AI_Init(void);
char MX_X_CUBE_AI_Process(float *image_28x28, uint8_t *accuracy, uint8_t input_type);

#ifdef __cplusplus
}
#endif

#endif /*__STMicroelectronics_X-CUBE-AI_3_2_0_H */
