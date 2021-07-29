#ifndef __HI_GPIO_H__
#define __HI_GPIO_H__ 
#define GPIO_PL061 
#define HI_K3_GPIO 
#define GPIODIR 0x400
#define GPIOIS 0x404
#define GPIOIBE 0x408
#define GPIOIEV 0x40C
#define GPIOIE 0x410
#define GPIOIE2 0x500
#define GPIOIE3 0x504
#define GPIOIE4 0x508
#define GPIORIS 0x414
#define GPIOMIS 0x418
#define GPIOMIS2 0x530
#define GPIOMIS3 0x534
#define GPIOMIS4 0x538
#define GPIOIC 0x41C
#define GPIO_MIS_CCORE GPIOMIS3
#define GPIO_IE_CCORE GPIOIE3
#define PL061_GPIO_NR 8
#define GPIO_MAX_BANK_NUM 37
#define GPIO_MAX_PINS PL061_GPIO_NR
#define GPIO_MAX_GROUP GPIO_MAX_BANK_NUM
#define GPIO_MAX_NUMBER PL061_GPIO_NR
#define GPIO_TEST_NUM (72)
#endif
