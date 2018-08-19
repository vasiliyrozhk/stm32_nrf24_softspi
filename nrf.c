#define delay_mul 8;

//подключение модуля:
#define mosi_lo GPIOA->BRR |= GPIO_PIN_7;
#define mosi_hi GPIOA->BSRR |= GPIO_PIN_7;
#define clk_lo GPIOA->BRR |= GPIO_PIN_5;
#define clk_hi GPIOA->BSRR |= GPIO_PIN_5;
#define csn_lo GPIOB->BRR |= GPIO_PIN_1;
#define csn_hi GPIOB->BSRR |= GPIO_PIN_1;
#define ce_lo GPIOA->BRR |= GPIO_PIN_1;
#define ce_hi GPIOA->BSRR |= GPIO_PIN_1;

uint8_t nrf_array[32]; //буффер для передачи и приема данных из модуля
uint8_t nrf_tmp[5];    //в основном для отладки


#define R_REGISTER                        0x00 //читаем регистр
#define W_REGISTER                        0x20 //пишем в регистр
#define R_RX_PAYLOAD                        0x61 //считывание из буфера принятых данных из космоса
#define W_TX_PAYLOAD                        0xA0 //запись данных в буфер для отправки в космос
#define FLUSH_TX                        0xE1 //очистка буфера отправки
#define FLUSH_RX                        0xE2 //очистка буфера приема
#define REUSE_TX_PL                        0xE3
#define ACTIVATE                        0x50
#define R_RX_PL_WID                        0x60
#define W_ACK_PAYLOAD                        0xA8
#define W_TX_PAYLOAD_NOACK                0x58
#define NOP                                0xFF //команда заглушка, ничего не делает.

//Описание регистров nRF24L01
/*регистр CONFIG*/    //Конфигурационный регистр
#define CONFIG                0x00
#define MASK_RX_DR  6 //вкл/откл прерывание от бита RX_DR в рег. STATUS. 0-вкл, 1-выкл.
#define MASK_TX_DS  5 //вкл/откл прерывание от бита TX_DS в рег. STATUS. 0-вкл, 1-выкл.
#define MASK_MAX_RT 4 //вкл/откл прерывание от бита MAX_RT в рег. STATUS. 0-вкл, 1-выкл.
#define EN_CRC      3 //включение CRC. По умолчанию вкл. если один из битов регистра EN_AA включен.
#define CRCO        2 //режим CRC. 0-1 байт, 1-2 байта.
#define PWR_UP      1 //1-POWER UP, 0-POWER DOWN, по умолчанию 0.
#define PRIM_RX     0 //0-режим передачи, 1-режим приема.

/*регистр EN_AA*/
#define EN_AA                0x01
/*регистр EN_RXADDR*/
#define EN_RXADDR        0x02
/*регистр SETUP_AW*/
#define SETUP_AW        0x03
/*регистр SETUP_RETR*/
#define SETUP_RETR        0x04
/*регистр RF_CH*/
#define RF_CH                0x05
/*регистр RF_SETUP*/
#define RF_SETUP        0x06
/*регистр STATUS*/
#define STATUS                0x07
#define RX_DR                6 /*прерывание: данные получены. Для сброса записать 1.*/
#define TX_DS                5 /*прерывание: данные переданы. Для сброса записать 1.*/
#define MAX_RT                4 /*прерывание: данные не переданы. Для сброса записать 1.*/
#define RX_P_NO2        3
#define RX_P_NO1        2
#define RX_P_NO0        1
#define TX_FULL0        0 /*флаг переполнения TX FIFO буфера передачи. 1-переполнен, 0-есть еще место.*/

/*регистр RX_ADDR_P0*/
#define RX_ADDR_P0  0x0A
/*регистр RX_ADDR_P1*/
#define RX_ADDR_P1  0x0B
/*регистр RX_ADDR_P2*/
#define RX_ADDR_P2  0x0C
/*регистр RX_ADDR_P3*/
#define RX_ADDR_P3  0x0D
/*регистр RX_ADDR_P4*/
#define RX_ADDR_P4  0x0E
/*регистр RX_ADDR_P5*/
#define RX_ADDR_P5  0x0F
/*регистр TX_ADDR*/
#define TX_ADDR     0x10
/*регистр RX_PW_P0*/
#define RX_PW_P0        0x11
/*регистр RX_PW_P1*/
#define RX_PW_P1        0x12
/*регистр RX_PW_P2*/
#define RX_PW_P2        0x13
/*регистр RX_PW_P3*/
#define RX_PW_P3        0x14
/*регистр RX_PW_P4*/
#define RX_PW_P4        0x15
/*регистр RX_PW_P5*/
#define RX_PW_P5        0x16
/*регистр FIFO_STATUS*/
#define FIFO_STATUS 0x17
/*регистр DYNPD*/
#define DYNPD                0x1C


inline uint8_t irq_get() {
       return (GPIOA->IDR & GPIO_IDR_0);
                           }

inline uint8_t miso_get() {
       return (GPIOA->IDR & GPIO_IDR_6);
                           }

static void delay_us(uint16_t dz) {
dz = dz * delay_mul;
do {
    dz--;  
    } while (dz != 0);
  
}

static uint8_t SPI1_WR (uint8_t dta) {
  uint8_t xx, dt, dts;              // счетчик для цикла
  dt = 0;
  dts = dta;
  for (xx=0; xx<8; xx++) {   // цикл на 8 бит
    delay_us(2);             // формируем половину паузы
    mosi_lo;
    if (dts & 0x80) mosi_hi;
    dts<<=1;
    delay_us(2);             // завершаем паузу
    clk_hi;                  // тактовый фронт
    delay_us(2);             // формируем импульс
    dt<<=1;                  // двигаем байт 
//    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) dt++; 
    if (miso_get()) dt++;
    delay_us(2);             // формируем импульс
    clk_lo;                  // тактовый спад
  
                          }
return dt;  
}

static void w_register(uint8_t a, uint8_t b)//а-адрес регистра, b-что пишем в регистр.
{
        a=a | W_REGISTER;
        csn_lo;
        SPI1_WR(a);
        if (b==0 && a!=W_TX_PAYLOAD)
        {
                csn_hi;
                return;
        }
        SPI1_WR(b);
        csn_hi;
}//W_REGISTER (CONFIG,0b00000110);

static uint8_t r_register(uint8_t a)//чтение байта из озу. a-адрес байта
{       uint8_t dr;
        csn_lo;//Прижимаем вывод CSN(SS) МК к земле, тем самым сообщаем о начале обмена данных.
        dr = SPI1_WR(a);
        if (a==STATUS)
        {
                csn_hi;//Вывод CSN(SS) МК к питанию, обмен данных завершен.
                return dr;
        }

        dr = SPI1_WR(NOP);
        csn_hi;//Вывод CSN(SS) МК к питанию, обмен данных завершен.
        return dr;
}//uint8_t a=r_register(EN_AA);

void prx(void)//Настроим nRF на прием.
{       
        //(1<<PWR_UP)|(1<<EN_CRC)|(1<<PRIM_RX);
        w_register(CONFIG, 0x3B);
        j = r_register(CONFIG);
        ce_hi;
        delay_us(135);
        //режим према RX
}

void ptx(void)//Настроим nRF на передачу.
{       
        ce_lo;
        //(1<<PWR_UP)|(1<<EN_CRC)|(0<<PRIM_RX);
        w_register(CONFIG, 0x3A);
        ce_hi;
        delay_us(15);
        ce_lo;
        delay_us(135);
}

static void nrf_write_array(uint8_t reg, uint8_t array[], uint8_t len)
{       uint8_t tcnt;
        reg=reg | W_REGISTER;
        csn_lo;
        SPI1_WR(reg);
        for (tcnt = 0; tcnt < len; tcnt++)
        {
        SPI1_WR(array[tcnt]);
        }
        csn_hi;
}

static void nrf_read_array(uint8_t reg, uint8_t *array, uint8_t len)
{       uint8_t tcnt;
        reg=reg | R_REGISTER;
        csn_lo;
        SPI1_WR(reg);

        for (tcnt = 0; tcnt < len; tcnt++)
        {
                array[tcnt] = SPI1_WR(0x00);
        }
        csn_hi;
}

static void nrf_setup() {
    w_register(CONFIG, 0x1A);
    j = r_register(CONFIG);
    w_register(EN_AA, 0x0F);
    j = r_register(EN_AA);
    w_register(EN_RXADDR, 0x0F);
    j = r_register(EN_RXADDR);
    w_register(SETUP_AW, 0x03); //adress - 5 byte
    j = r_register(SETUP_AW);
    w_register(SETUP_RETR, 0x0F);
    j = r_register(SETUP_RETR);
    w_register(RF_CH, 0x46);
    j = r_register(RF_CH);
    w_register(RF_SETUP, 0x00);  //1MBPS 0x06
    j = r_register(RF_SETUP);

    nrf_write_array(RX_ADDR_P0, "KVART", 5);//запись адреса для приемника
    nrf_read_array(RX_ADDR_P0, nrf_tmp, 5);
    
    nrf_write_array(TX_ADDR, "KVART", 5);//запись адреса для приемника
    nrf_read_array(TX_ADDR, nrf_tmp, 5);
    w_register(RX_PW_P0, 32);//размер поля данных 32 байт.
    j = r_register(RX_PW_P0);
    j = r_register(STATUS);

    prx();
}
