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
#include "uart.h"
#include "string.h"

extern uint32_t SystemCoreClock;

static void clear_buffers(void);

// Size must be 2^n for using quick wrap around
#define  BUFFER_SIZE          (512)

struct {
    uint8_t  data[BUFFER_SIZE];
    volatile uint32_t idx_in;
    volatile uint32_t idx_out;
    volatile uint32_t cnt_in;
    volatile uint32_t cnt_out;
} write_buffer, read_buffer;

uint32_t tx_in_progress = 0;

void clear_buffers(void)
{
    memset((void*)&read_buffer, 0xBB, sizeof(read_buffer.data));
    read_buffer.idx_in = 0;
    read_buffer.idx_out = 0;
    read_buffer.cnt_in = 0;
    read_buffer.cnt_out = 0;
    memset((void*)&write_buffer, 0xBB, sizeof(read_buffer.data));
    write_buffer.idx_in = 0;
    write_buffer.idx_out = 0;
    write_buffer.cnt_in = 0;
    write_buffer.cnt_out = 0;
}

int32_t uart_initialize (void) {

  RCC->APB2ENR |=  (   1UL <<  0);         /* enable clock Alternate Function */
  AFIO->MAPR   &= ~(   1UL <<  2);         /* clear USART1 remap              */

  RCC->APB2ENR |=  (   1UL <<  2);         /* enable GPIOA clock              */
  GPIOA->CRH   &= ~(0xFFUL <<  4);         /* clear PA9, PA10                 */
  GPIOA->CRH   |=  (0x0BUL <<  4);         /* USART1 Tx (PA9) output push-pull*/
  GPIOA->CRH   |=  (0x04UL <<  8);         /* USART1 Rx (PA10) input floating */

  RCC->APB2ENR |=  (   1UL << 14);         /* enable USART1 clock             */

  uart_reset ();
  NVIC_EnableIRQ(USART1_IRQn);             /* Enable USART interrupt          */

  return 1;
}

int32_t uart_uninitialize (void) {

  RCC->APB2ENR &= ~(1 << 14);
  
  GPIOA->CRH   &= ~(0xFF << 4);
  GPIOA->CRH   |=  (0x44 << 4);

	NVIC_DisableIRQ(USART1_IRQn);             /* Disable USART interrupt          */
    
	clear_buffers();

    return 1;
}

int32_t uart_reset (void) {
   

  NVIC_DisableIRQ(USART1_IRQn);         /* Enable USART interrupt             */

    clear_buffers();

    tx_in_progress = 0;

  NVIC_EnableIRQ (USART1_IRQn);         /* Enable USART interrupt             */
	

    return 1;
}

int32_t uart_set_configuration (UART_Configuration *config) {

  uint32_t i, cr1, cr2;

  /* USART supports:
      8bit data; even, odd and none parity; 1, 1.5 and 2 stop bits */
  
  /* Data bits */
  if (config->DataBits != UART_DATA_BITS_8) return(0);

  /* Parity */
  switch (config->Parity) {
    case UART_PARITY_NONE: cr1 = 0;                                 break;
    case UART_PARITY_EVEN: cr1 = (1 << 12) | (1 << 10);             break;
    case UART_PARITY_ODD:  cr1 = (1 << 12) | (1 << 10) | (1 <<  9); break;
    default: return (0);
  }

  /* Stop bits */
  switch (config->StopBits) {
    case UART_STOP_BITS_1:    cr2 = 0;         break;
    case UART_STOP_BITS_2:    cr2 = (2 << 12); break;
    case UART_STOP_BITS_1_5:  cr2 = (3 << 12); break;
    default: return (0);
  }
  
  /* Baudrate */
  Baudrate = config->Baudrate;
  USART1->BRR = __USART_BRR(UART_CLK, Baudrate);
  
  /* Flow control */
  FlowControl = config->FlowControl;

  switch (config->FlowControl) {        /* Prepare flow control value for MR  */
    case UART_FLOW_CONTROL_NONE:
      FlowControl = UART_FLOW_CONTROL_NONE;
      break;
    case UART_FLOW_CONTROL_RTS_CTS:
    default:
      return (0);
  }

  USART1->CR2 = cr2;                    /* stop bits settings                 */
  for (i = 0; i < 0x1000; i++) __NOP(); /* avoid unwanted output              */
  USART1->CR1 = (1 << 13) |             /* USART enable                       */
               (1 <<  3) |              /* transmiter enable                  */
               (1 <<  2) |              /* receiver enable                    */
               (1 <<  5) |              /* enable RXNE interrupt              */
                cr1;                    /* parity and data bit settings       */

    return 1;
}

int32_t uart_get_configuration (UART_Configuration *config) {

    return 1;
}

int32_t uart_write_free(void) {

    return BUFFER_SIZE - (write_buffer.cnt_in - write_buffer.cnt_out);
}

int32_t uart_write_data (uint8_t *data, uint16_t size) {
    uint32_t cnt;
    int16_t  len_in_buf;

    if (size == 0) {
        return 0;
    }

    cnt = 0;

    while (size--) {
        len_in_buf = write_buffer.cnt_in - write_buffer.cnt_out;
        if (len_in_buf < BUFFER_SIZE) {
            write_buffer.data[write_buffer.idx_in++] = *data++;
            write_buffer.idx_in &= (BUFFER_SIZE - 1);
            write_buffer.cnt_in++;
            cnt++;
        }
    }

    if (!tx_in_progress)
    {
        // Wait for D register to be free
        while(!(UART1->S1 & UART_S1_TDRE_MASK)) { }

        tx_in_progress = 1;

        // Write the first byte into D
        UART1->D = write_buffer.data[write_buffer.idx_out++];
        write_buffer.idx_out &= (BUFFER_SIZE - 1);
        write_buffer.cnt_out++;

        // enable TX interrupt
        UART1->C2 |= UART_C2_TIE_MASK;
    }

    return cnt;
}

int32_t uart_read_data (uint8_t *data, uint16_t size) {
    uint32_t cnt;

    if (size == 0) {
        return 0;
    }

    cnt = 0;

    while (size--) {
        if (read_buffer.cnt_in != read_buffer.cnt_out) {
            *data++ = read_buffer.data[read_buffer.idx_out++];
            read_buffer.idx_out &= (BUFFER_SIZE - 1);
            read_buffer.cnt_out++;
            cnt++;
        }
        else
        {
            break;
        }
    }

    return cnt;
}

void UART1_RX_TX_IRQHandler (void) {
    uint32_t s1;
    volatile uint8_t errorData;

    // read interrupt status
    s1 = UART1->S1;

    // handle character to transmit
    if (write_buffer.cnt_in != write_buffer.cnt_out) {
        // if TDRE is empty
        if (s1 & UART_S1_TDRE_MASK) {
            UART1->D = write_buffer.data[write_buffer.idx_out++];
            write_buffer.idx_out &= (BUFFER_SIZE - 1);
            write_buffer.cnt_out++;
            tx_in_progress = 1;
        }
    }
    else {
        // disable TIE interrupt
        UART1->C2 &= ~(UART_C2_TIE_MASK);
        tx_in_progress = 0;
    }

    // handle received character
    if (s1 & UART_S1_RDRF_MASK) {
        if ((s1 & UART_S1_NF_MASK) || (s1 & UART_S1_FE_MASK))
        {
            errorData = UART1->D;
        }
        else
        {
            read_buffer.data[read_buffer.idx_in++] = UART1->D;
            read_buffer.idx_in &= (BUFFER_SIZE - 1);
            read_buffer.cnt_in++;
        }
    }
}

/*------------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/

