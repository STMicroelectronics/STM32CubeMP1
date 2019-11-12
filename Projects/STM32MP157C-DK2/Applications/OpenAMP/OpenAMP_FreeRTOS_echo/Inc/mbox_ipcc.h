/**
  ******************************************************************************
  * @file           : mbox_ipcc.h
  * @brief          : Header for mailbox_ipcc_if.c module
  ******************************************************************************
    * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef MBOX_IPCC_H_
#define MBOX_IPCC_H_

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int MAILBOX_Notify(void *priv, uint32_t id);
int MAILBOX_Init(void);
int MAILBOX_Poll(struct virtio_device *vdev);

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new definitions */
/* USER CODE END lastSection */

#endif /* MBOX_IPCC_H_ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
