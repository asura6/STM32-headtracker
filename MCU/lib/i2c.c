#include <stm32f10x.h>
#include <stdint.h>
#include "USART.h"
#include "i2c.h"
#include "timer.h"

/*******************************
 ***** PREPROCESSOR MACROS *****
 *******************************/

#define I2C_Stop() I2C1->CR1 |= I2C_CR1_STOP
#define I2C_Start() I2C1->CR1 |= I2C_CR1_START
#define I2C_TX_Data(data) (I2C1->DR = (0x00FFU & data))
#define I2C_RX_Data() I2C1->DR
#define I2C_TX_Data_Empty() (I2C1->SR1 & I2C_SR1_TXE)
#define I2C_RX_Data_Not_Empty() (I2C1->SR1 & I2C_SR1_RXNE)
#define I2C_RX_Stop() (I2C1->SR1 & I2C_SR1_STOPF)
#define I2C_Start_Was_Sent() (I2C1->SR1 & I2C_SR1_SB)
#define I2C_Byte_Transferred() (I2C1->SR1 & I2C_SR1_BTF)
#define I2C_Address_Was_Sent() (I2C1->SR1 & I2C_SR1_ADDR)
#define I2C_Is_Transmitter() (I2C1->SR2 & I2C_SR2_TRA)
#define I2C_Start_Condition() (I2C1->SR1 & I2C_SR1_SB)
#define I2C_Is_Busy() ((I2C1->SR2 & I2C_SR2_BUSY) ? 1U : 0U) 
#define I2C_ACK_Failed() (I2C1->SR1 & I2C_SR1_AF)

#define I2C_Enable_ACK() (I2C1->CR1 |= I2C_CR1_ACK)
#define I2C_Disable_ACK() (I2C1->CR1 &= ~I2C_CR1_ACK)
#define I2C_Set_POS()   (I2C1->CR1 |= I2C_CR1_POS)
#define I2C_Clear_POS()   (I2C1->CR1 &= ~I2C_CR1_POS)

#define DMA_CH6_Is_Enabled() (DMA1_Channel6->CCR & DMA_CCR6_EN)
#define DMA_CH7_Is_Enabled() (DMA1_Channel7->CCR & DMA_CCR7_EN)
#define DMA_CH7_Is_Last_Byte() (DMA1_Channel7->CNDTR == 0x01U)
#define I2C_Set_DMA_Last_Transfer() (I2C1->CR2 |= I2C_CR2_LAST)

/************************************************
 ***** FORWARD DECLARATIONS *****
 ************************************************/

static void I2C_Update_DMA_CH6(uint32_t *DMA_buffer, uint32_t nr_bytes);

/*********************
 ***** CONSTANTS *****
 *********************/

#define TX_LSB 0U; //LSB of address for a transmission
#define RX_LSB 1U; //LSB of address for receiving data 
#define I2C_DEFAULT             0x00U
#define I2C_READ_NOT_FINISHED   0x01U
#define I2C_WRITE_NOT_FINISHED  0x02U
#define I2C_READING_SINGLE_BYTE 0x03U


/****************************
 ***** GLOBAL VARIABLES *****
 ****************************/

/* We store the address in this global variable so that it can be accessed in
 * the interrupt service routines */
volatile uint8_t I2C_address; 
volatile I2C_word_t I2C_word; //Data is saved to this global variable 
volatile uint8_t I2C_status = I2C_DEFAULT; //State variable for returning received data 

/*************************
 ***** I2C FUNCTIONS *****
 *************************/

/* Initialize the I2C module
 * I2C1 Pins: SCL on PB6, SDA on PB7 */
void Init_I2C(void) {
    /* Enable the system clock for the I2C1 module */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //Clock enable I2C1
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; //Clock enable port B
    /* Enable the alternate function on the pins (I2C function) and set them to
     * open drain-output configuration */
    GPIOB->CRL |= GPIO_CRL_MODE6_1; //2 MHz output mode
    GPIOB->CRL |= GPIO_CRL_CNF6;  //Alternate function open-drain
    GPIOB->CRL |= GPIO_CRL_MODE7_1; //2 MHz output mode
    GPIOB->CRL |= GPIO_CRL_CNF7;  //Alternate function open-drain 

    /* Enable event interrupt */
    I2C1->CR2 |= I2C_CR2_ITEVTEN;
    /* Set the peripheral clock frequency */
    I2C1->CR2 |= (I2C_CR2_FREQ & 0x24U); //36 MHz 

    /* Here we set the frequency to 400 kHz  */
    /* We want 400 kHz at 36 MHz peripheral clock so we use CRR = 30,
     * using DUTY bit = 0. Because
     * Thigh = CRR * Tpclk -> 1/(CRR*Tpclk + CRR*2*Tpclk) = 400e3 Hz */
    I2C1->CCR |= I2C_CCR_FS; //Fast mode
    I2C1->CCR |= (I2C_CCR_CCR & 30U);

    /* Set the maximum rise/fall time */
    /* This is calculated from the maximum allowed 1000 ns divided by the
     * peripheral time period + 1. In this case
     * TRISE = 1000e-9/(1/36e6) + 1 = 37 = 0x0025 */
    I2C1->TRISE = 0x0025U; 

    /* End of configuration */
    /* Enable the peripheral */
    I2C1->CR1 |= I2C_CR1_PE;
    /* Enable the IRQ */
    NVIC_EnableIRQ(I2C1_EV_IRQn); 
}

/* Initialize the DMA module */
void I2C_Init_DMA(void) {
    /* Enable clock to the DMA 1 module */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    /* Set the peripheral address register to the I2C1 DR register */
    DMA1_Channel6->CPAR = (uint32_t)(&(I2C1->DR)); //CH6 is for transmision
    DMA1_Channel7->CPAR = (uint32_t)(&(I2C1->DR)); //CH7 is for reception
    /* Configure the channel priority to very high */
    DMA1_Channel6->CCR |= DMA_CCR6_PL;
    DMA1_Channel7->CCR |= DMA_CCR7_PL;
    /* Set data direction */
    DMA1_Channel6->CCR |= DMA_CCR6_DIR;     //From memory
    //DMA1_Channel7->CCR &= ~DMA_CCR7_DIR;    //From peripheral
    /* Enable memory increment mode */
    DMA1_Channel6->CCR |= DMA_CCR6_MINC; 
    DMA1_Channel7->CCR |= DMA_CCR7_MINC; 
 //   /* Set peripheral size to half-word size */
 //   DMA1_Channel6->CCR |= DMA_CCR6_PSIZE_0;
 //   DMA1_Channel7->CCR |= DMA_CCR7_PSIZE_0;
 //   /* Set memory-size to half-word size */
 //   DMA1_Channel6->CCR |= DMA_CCR6_MSIZE_0;
 //   DMA1_Channel7->CCR |= DMA_CCR7_MSIZE_0;
    /* Enable Transfer error and transfer complete interrupts */
    DMA1_Channel6->CCR |= DMA_CCR6_TEIE | DMA_CCR6_TCIE;
    DMA1_Channel7->CCR |= DMA_CCR7_TEIE | DMA_CCR7_TCIE;
    /* Enable DMA1 Channel interrupts */
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/* Update the DMA module with new memory buffer information and then enable the
 * DMA channel */
void I2C_Update_DMA_CH6(uint32_t *DMA_buffer, uint32_t nr_bytes) {
    /* Set the memory address that the DMA uses to read data from */
    DMA1_Channel6->CMAR = (uint32_t)(DMA_buffer);
    /* Set the number of data transfers */
    DMA1_Channel6->CNDTR = nr_bytes;
    /* Enable the DMA channel */
    DMA1_Channel6->CCR |= DMA_CCR6_EN;
}

void I2C_Update_DMA_CH7(uint32_t *DMA_buffer, uint32_t nr_bytes) {
    /* Set the memory address that the DMA uses to read data from */
    DMA1_Channel7->CMAR = (uint32_t)(DMA_buffer);
    /* Set the number of data transfers */
    DMA1_Channel7->CNDTR = nr_bytes; 
    /* Enable the DMA channel */
    DMA1_Channel7->CCR |= DMA_CCR7_EN; 
}

/* Send a single byte through I2C */
void I2C_Send_Byte(uint8_t address, uint8_t byte) {
    /* Wait until BUS is ready */
    I2C_Wait_Until_Ready();
    /* Update global variables as the IRQ functions use them */
    I2C_address = (address << 1U) | TX_LSB;
    I2C_word.byte0 = byte;
    /* Set the state variable */
    I2C_status = I2C_WRITE_NOT_FINISHED; 
    /* Start the bus and transititon the I2C module to master mode */
    I2C_Start(); 
    /* From here on the IRQ will transmit the data */
}

/* Read a single byte through I2C */
void I2C_Read_Byte(uint8_t address) { 
    /* Wait until BUS is ready */
    I2C_Wait_Until_Ready();
    /* Update global variables as the IRQ functions use them */
    I2C_address = (address << 1U) | RX_LSB; 
    /* Set the state variable */
    I2C_status = I2C_READING_SINGLE_BYTE;
    /* Start the bus and transititon the I2C module to master mode */
    I2C_Start(); 
    /* From here on the IRQ will transmit the data */ 
}

/* Send multiple bytes through I2C */
void I2C_Send_Bytes(uint8_t address, uint32_t nr_bytes, uint8_t *DMA_buffer) { 
    /* Wait until BUS is ready */
    I2C_Wait_Until_Ready();
    /* Update global variables as the IRQ functions use them */
    I2C_address = (address << 1U) | TX_LSB;
    /* Update the DMA module with a pointer to the data-buffer and the number of
     * bytes to transmit */
    I2C_Update_DMA_CH6((uint32_t *)DMA_buffer, nr_bytes);
    /* Set the state variable */
    I2C_status = I2C_WRITE_NOT_FINISHED;
    /* Start the bus and transition the I2C module to master mode */
    I2C_Start();
    /* From here on the IRQs will transmit the data */
}

void I2C_Read_Bytes(uint8_t address, uint32_t nr_bytes, uint8_t *DMA_buffer) {
    /* Wait until BUS is ready */
    I2C_Wait_Until_Ready();
    /* Update global variables as the IRQ functions use them */
    I2C_address = (address << 1U) | RX_LSB;
    /* Update the DMA module with a pointer to the data-buffer and the number of
     * bytes to transmit */
    I2C_Update_DMA_CH7((uint32_t *)DMA_buffer, nr_bytes);
    /* Enable ACK until last byte */
    I2C_Enable_ACK();
    I2C_Set_DMA_Last_Transfer(); 
    /* Set the state variable */
    I2C_status = I2C_READ_NOT_FINISHED; 
    /* Start the bus and transition the I2C module to master mode */
    I2C_Start();
    /* From here on the IRQs will transmit the data */ 
}

///* Read word with high byte first */
//uint16_t I2C_Read_Word(uint8_t address) {
//    /* Wait until BUS is ready */
//    I2C_Wait_Until_Ready();
//    /* Update global variables as the IRQ functions use them */
//    I2C_address = (address << 1U) | RX_LSB; 
//    /* Set the state variable */
//    I2C_status = I2C_READ_NOT_FINISHED; 
//    I2C_Enable_ACK();
//    I2C_Set_POS(); 
//    I2C_Start();
//    I2C_Wait_Until_Done();
//    return (uint16_t)(I2C_word.byte1 | (I2C_word.byte0 << 8U));
//}

/* Wait until the I2C bus is ready to accept new transmissions */
void I2C_Wait_Until_Ready(void) {
    while (I2C_Is_Busy()) {
        /* Do nothing */
    }
} 

void I2C_Wait_Until_Done(void) {
    while (I2C_status) {
        __ASM("WFI");
    }
    for (uint8_t i = 0xff; i > 0; i--) {
        /* Minimal delay in addition */
    }
}

uint8_t I2C_Get_Byte(void) { 
    return I2C_word.byte0;
} 

void I2C_Restart(void) { 
    I2C_Start();
}

void I2C_Request_Stop(void) {
    I2C_Stop();
}

void I2C_Errata_Workaround(void) {
    I2C_Disable_ACK(); 
    GPIOB->CRL &= ~GPIO_CRL_CNF6_1;  //general purpose output open-drain 
    GPIOB->BRR |= GPIO_BRR_BR6; //Output low
    (void)(I2C_Is_Transmitter()); //Clear ADDR 
    I2C1->CR2 |= I2C_CR2_ITBUFEN; //Need to read data
    I2C_Stop();

    for (uint8_t i = 0x1; i > 0; i--) {
        /* This delay should align the transfer with the clock. When the
         * GPIO is changed to the alternate function it can cause a very
         * low duty-cycle clock pulse which can be problematic. I
         * characterized this delay using a logic analyzer and found
         * that it aligned the duty cycle closer to the expected 2.5 us
         * which was configured earlier */ 
    } 

    GPIOB->CRL |= GPIO_CRL_CNF6_1;  //alternative function open-drain 
    I2C_status = I2C_DEFAULT; 
}

/* I2C interrupt service routine */
void I2C1_EV_IRQ(void) { 

    /* Check if start condition has been set */
    //EV5
    if (I2C_Start_Was_Sent()) { 
        /* Transmit the address */ 
        I2C_TX_Data(I2C_address); //Clears SB interrupt
        return; 
    } else if (I2C_Address_Was_Sent()) {
        /* Check if the address has been sent */
        //EV6 
        if (I2C_status == I2C_READING_SINGLE_BYTE) {
            /* Receiving a single byte using ERRATA WORKAROUND 3 */
            I2C_Errata_Workaround();
            return; 

        } else if (I2C_Is_Transmitter()) {
            /* Deremine if transmission or reception should be done */
            if (DMA_CH6_Is_Enabled()) {
                /* Enable DMA requests */
                I2C1->CR2 |= I2C_CR2_DMAEN;
            } else if (I2C_TX_Data_Empty()) { 
                I2C_TX_Data(I2C_word.byte0); 
            } 
            return; 

        } else { 
            if (DMA_CH7_Is_Enabled()) { 
                /* Enable DMA requests */ 
                I2C1->CR2 |= I2C_CR2_DMAEN;
            } else {
                I2C_Disable_ACK(); 
            }
            return;
        } 
    } else if (I2C_RX_Data_Not_Empty()) { 
        I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
        I2C_word.byte0 = I2C_RX_Data(); 
        I2C_Stop(); 

    } else if (I2C_Byte_Transferred()) { 
        /* Check if the transfer is finished */
        I2C_Stop(); 

        if (~I2C_Is_Transmitter()) {
            I2C_word.byte0 = I2C_RX_Data();
            I2C_word.byte1 = I2C_RX_Data();
            I2C_status = I2C_DEFAULT;

        } else if (I2C_ACK_Failed()) { 
            I2C_Stop();
            I2C1->SR1 &= ~I2C_SR1_AF;
        }
    }
}

    /* DMA interrupt service routine used for DMA transmissions */
    void DMA1_Channel6_IRQ(void) { 
        /* Check for transfer error */
        if (DMA1->ISR & DMA_ISR_TEIF6) {
            /* Stop the transfer */
            I2C_Stop(); 
            /* Clear the interrupt flag */
            DMA1->IFCR |= DMA_IFCR_CTEIF6;
            return;
        }
        /* Check if the transfer is completed */
        if (DMA1->ISR & DMA_ISR_TCIF6) {
            /* Stop the transfer */
            I2C_Stop();
            /* Set the state variable that the transfer is complete */
            I2C_status = I2C_DEFAULT;
            /* Clear the interrupt flag */
            DMA1->IFCR |= DMA_IFCR_CTCIF6;
            /* Disable I2C DMA requests */
            I2C1->CR2 &= ~I2C_CR2_DMAEN;
            /* Disable the DMA channel */
            DMA1_Channel6->CCR &= ~DMA_CCR6_EN;
            return;
        }
        /* Check for any other interrupt */
        if (DMA1->ISR & DMA_ISR_TCIF6) {
            /* Clear the interrupt flag */
            DMA1->IFCR |= DMA_IFCR_CGIF6;
        }
    }

    /* DMA interrupt service routine used for DMA reception */
    void DMA1_Channel7_IRQ(void) { 
        /* Check for transfer error */
        if (DMA1->ISR & DMA_ISR_TEIF7) {
            /* Clear the interrupt flag */
            DMA1->IFCR |= DMA_IFCR_CTEIF7; 
            return;
        } else if (DMA1->ISR & DMA_ISR_TCIF7) { 
            /* Check if the transfer is completed */ 
            /* Stop the transfer */
            I2C_Stop();
            /* Set the state variable that the transfer is complete */
            I2C_status = I2C_DEFAULT;
            /* Clear the interrupt flag */
            DMA1->IFCR |= DMA_IFCR_CTCIF7;
            /* Disable I2C DMA requests */
            I2C1->CR2 &= ~I2C_CR2_DMAEN;
            /* Disable the DMA channel */
            DMA1_Channel7->CCR &= ~DMA_CCR7_EN; 
            return; 
        } else if (DMA1->ISR & DMA_ISR_TCIF6) {
            /* Check for any other interrupt */
            /* Clear the interrupt flag */
            DMA1->IFCR |= DMA_IFCR_CGIF7; 
        }
    }
