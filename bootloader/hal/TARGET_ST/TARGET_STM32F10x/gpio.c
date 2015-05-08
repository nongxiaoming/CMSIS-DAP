/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "stm32f10x.h"
#include "gpio.h"

#define LED0 0
#define LED1 1
#define LED0_PIN 10
#define LED1_PIN 11

#define led_hw_on(led)  GPIOB->BRR|=0x01<<(LED0_PIN+led)
#define led_hw_off(led) GPIOB->BSRR|=0x01<<(LED0_PIN+led)

void gpio_init(void) {
    /* enable the clock of GPIOA and GPIOB. */
    RCC->APB2RSTR |= RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN;

    /* clear the PB10,PB11 GPIO configure */ 
	  GPIOB->CRH&=~(0xff<<3);
	  //set the PB10,PB11 as output
    GPIOB->CRH |= GPIO_CRH_MODE10| GPIO_CRH_MODE11;
    //turn off the led0,led1
	  GPIOB->ODR |= GPIO_ODR_ODR10|GPIO_ODR_ODR11;
	  /* clear the PA6 GPIO configure */ 
	  GPIOA->CRL&=~(0x0f<<6);
	  //set the PA6 as input
    GPIOA->CRH |= (0x08 << 6);
	  /* PA6 pull-up*/
	  GPIOA->ODR |= (0x01 << 6);                             
}

void gpio_set_dap_led(uint8_t state) {
    if (state) {
        led_hw_on(LED0); // LED on
    } else {
       led_hw_off(LED0); // LED off
    }
}

void gpio_set_msd_led(uint8_t state) {
     if (state) {
        led_hw_on(LED1); // LED on
    } else {
       led_hw_off(LED1); // LED off
    }
}

void gpio_set_cdc_led(uint8_t state) {
     if (state) {
        led_hw_on(LED1); // LED on
    } else {
       led_hw_off(LED1); // LED off
    }
}
uint8_t gpio_get_pin_loader_state(void) {
    return ( GPIOA->IDR & (1UL << 6));
}


