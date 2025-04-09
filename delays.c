/*
 * delays.c
 *
 *  Created on: Mar 31, 2025
 *      Author: Admin
 */

#include "delays.h"

extern TIM_HandleTypeDef htim1;

void DelayUs(uint32_t us) {
  __HAL_TIM_SET_COUNTER(&htim1, 0); // Đặt lại bộ đếm
  while (__HAL_TIM_GET_COUNTER(&htim1) < us); // Chờ đến khi đạt số micro giây
}

void DelayMs(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    DelayUs(1000); // 1000 us = 1 ms
  }
}
