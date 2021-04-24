---
title: Freescale Kinetis K60
tags: 
 - MCU
 - Note
 - VCDC
---

飞思卡尔K60学习笔记，使用超核电子渡鸦开发板，芯片型号MK60D10

<!--more-->

## 配置文件

```c
MK60D10.h //芯片头文件
system_MK60D10.h //时钟配置头文件
startup_MK60D10.s //启动文件
system_MK60D10.c //时钟配置文件
```

`.c`文件在

`.s`文件在

## 头文件配置

`startup_MK60D10.s` 

1. `Stack_Size` `EQU` `0x00001000`
2. `Heap_Size` `EQU` `0x000010000`

## Port寄存器

### PDOR(Port Data Output Register)

| Value |               |
| ----- | ------------- |
| 0     | logic level 0 |
| 1     | logic level 1 |

### PSOR(Port Set Output Register)

| Value |                |
| ----- | -------------- |
| 0     | no change      |
| 1     | set to logic 1 |

### PCOR(Port Clear Output Register)

| Value |                |
| ----- | -------------- |
| 0     | no change      |
| 1     | set to logic 0 |

### PTOR(Port Toggle Output Register)

| Value |                     |
| ----- | ------------------- |
| 0     | no change           |
| 1     | inverse logic state |

### PDIR(Port Data Input Register)

only read

### PDDR(Port Data Direction Register)

| Value |        |
| ----- | ------ |
| 0     | input  |
| 1     | output |

## 注意

* 开启代码补全功能,需要引入头文件

* `__IO` 、`__I` 、`__O` voiatile 防止编译器优化

## 输出电平

```c
//output volt
PTE->PDOR &= ~(1 << 6);//low volt
PTA->PDOR |= (1 << 6);//high volt
```

## 步骤

1. 开启时钟

   ```c
   //open PORTA clock
   SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;//open PORTA clock
   ```

2. 配置复用

   ```c
   //config PE6 alt1
   PORTE->PCR[6] &= ~PORT_PCR_MUX_MASK;//led1
   PORTE->PCR[6] |= PORT_PCR_MUX(1);//led1
   ```

3. 设置输入输出

   ```c
   PTE->PDDR |= (1 << 6);//set PE6 to output
   PTE->PDDR &= ~(1 << 26);//set PE26 to input
   ```

   1. 若为输入，设置上拉/下拉

       ```c
       //set PE26 to pull-up
       PORTE->PCR[26] |= PORT_PCR_PE_MASK;
       PORTE->PCR[26] |= PORT_PCR_PS_MASK;
       //set PE26 to pull-down
       PORTE->PCR[26] &= ~PORT_PCR_PE_MASK;
       PORTE->PCR[26] &= ~PORT_PCR_PS_MASK;
       ```

   2. 若为输出，设置高电平/低电平

       ```c
       PTE->PDOR |= (1 << 6);//set PE6 to high level
       PTE->PDOR &= ~(1 << 6);//set PE6 to low level
       ```

       

## 步骤（函数）

### 头函数



### 宏定义



### 初始化函数



### 快速初始化函数

#### GPIO_QuickInit

```c
 /**
 * @brief  快速初始化一个GPIO引脚 实际上是GPIO_Init的最简单配置
 * @code
 *      //初始化配置PORTB端口的10引脚为推挽输出引脚
 *      GPIO_QuickInit(HW_GPIOB, 10, kGPIO_Mode_OPP);
 * @endcode
 * @param  instance: GPIO模块号
 *         @arg HW_GPIOA :芯片的PORTA端口
 *         @arg HW_GPIOB :芯片的PORTB端口
 *         @arg HW_GPIOC :芯片的PORTC端口
 *         @arg HW_GPIOD :芯片的PORTD端口
 *         @arg HW_GPIOE :芯片的PORTE端口
 * @param  pin  :端口上的引脚号 0~31
 * @param  mode  :引脚工作模式
 *         @arg kGPIO_Mode_IFT :悬空输入
 *         @arg kGPIO_Mode_IPD :下拉输入
 *         @arg kGPIO_Mode_IPU :上拉输入
 *         @arg kGPIO_Mode_OOD :开漏输出
 *         @arg kGPIO_Mode_OPP :推挽输出
 * @retval None
 */
uint8_t GPIO_QuickInit(uint32_t instance, uint32_t pinx, GPIO_Mode_Type mode)
{
    GPIO_InitTypeDef GPIO_InitStruct1;
    GPIO_InitStruct1.instance = instance;
    GPIO_InitStruct1.mode = mode;
    GPIO_InitStruct1.pinx = pinx;
    GPIO_Init(&GPIO_InitStruct1);
    return  instance;
}
```



#### UART_QuickInit

```c
 /**
 * @brief  串口快速化配置函数
 * @code
 *      // 初始化 UART4 属性: 115200-N-8-N-1, Tx:PC15 Rx:PC14
 *      UART_QuickInit(UART4_RX_PC14_TX_PC15, 115200);
 * @endcode
 * @param  MAP  : 串口引脚配置缩略图
 *         例如 UART1_RX_PE01_TX_PE00 ：使用串口1的PTE1/PTE0引脚
 * @param  baudrate: 波特率 9600 115200...
 * @retval UART模块号
 */
uint8_t UART_QuickInit(uint32_t MAP, uint32_t baudrate)
{
    uint8_t i;
    uint32_t clock;
    UART_InitTypeDef UART_InitStruct1;
    QuickInit_Type * pq = (QuickInit_Type*)&(MAP);
    UART_InitStruct1.baudrate = baudrate;
    UART_InitStruct1.instance = pq->ip_instance;
    UART_InitStruct1.parityMode = kUART_ParityDisabled;
    UART_InitStruct1.bitPerChar = kUART_8BitsPerChar;
    
    /* src clock */
    CLOCK_GetClockFrequency(kBusClock, &clock);
    if((pq->ip_instance == HW_UART0) || (pq->ip_instance == HW_UART1))
    {
        CLOCK_GetClockFrequency(kCoreClock, &clock); /* UART0 UART1 are use core clock */
    }
    UART_InitStruct1.srcClock = clock;
    
    /* init pinmux */
    for(i = 0; i < pq->io_offset; i++)
    {
        PORT_PinMuxConfig(pq->io_instance, pq->io_base + i, (PORT_PinMux_Type) pq->mux); 
    }
    
    /* init UART */
    UART_Init(&UART_InitStruct1);
    
    /* default: disable hardware buffer */
    UART_EnableTxFIFO(pq->ip_instance, false);
    UART_EnableRxFIFO(pq->ip_instance, false);
    
    return pq->ip_instance;
}
```



### 复用函数

#### PORT_PinMuxConfig

```c
 /**
 * @brief 设置引脚复用功能 这个函数会被很多其他外设模块驱动程序调用
 * @note  复用功能可参考 Reference Manual 的 Signal Multiplexing and Signal Descriptions 章节 
 * @code
 *      // 将一PORTA端口的3引脚复用成1模式.
 *      PORT_PinMuxConfig(HW_GPIOA, 3, kPinAlt1);
 * @endcode
 * @param  instance: GPIO模块号 
 *         @arg HW_GPIOA :芯片的PORTA端口
 *         @arg HW_GPIOB :芯片的PORTB端口
 *         @arg HW_GPIOC :芯片的PORTC端口
 *         @arg HW_GPIOD :芯片的PORTD端口
 *         @arg HW_GPIOE :芯片的PORTE端口
 * @param  pin  :端口上的引脚号 0~31
 * @param  pinMux    :复用功能选项，不同的复用值代表不同的功能
 *         @arg kPinAlt0 :引脚复用成0模式
 *         @arg        . : .
 *         @arg        . : .
 *         @arg        . : .
 *         @arg kPinAlt7 :引脚复用成7模式
 * @retval None
 */
void PORT_PinMuxConfig(uint32_t instance, uint8_t pin, PORT_PinMux_Type pinMux)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    PORT_InstanceTable[instance]->PCR[pin] &= ~(PORT_PCR_MUX_MASK);
    PORT_InstanceTable[instance]->PCR[pin] |=  PORT_PCR_MUX(pinMux);
```



### 其他函数

#### GPIO_ToggleBit

```c
 /**
 * @brief  翻转一个引脚的电平状态
 * @code
 *      //翻转PORTB端口的10引脚的电平状态
 *      GPIO_ToggleBit(HW_GPIOB, 10); 
 * @endcode
 * @param  instance: GPIO模块号
 *         @arg HW_GPIOA :芯片的PORTA端口
 *         @arg HW_GPIOB :芯片的PORTB端口
 *         @arg HW_GPIOC :芯片的PORTC端口
 *         @arg HW_GPIOD :芯片的PORTD端口
 *         @arg HW_GPIOE :芯片的PORTE端口
 * @param  pin  :端口上的引脚号 0~31
 * @retval None
 */
void GPIO_ToggleBit(uint32_t instance, uint8_t pin)
{
    GPIO_InstanceTable[instance]->PTOR |= (1 << pin);
}
```



#### UART_WriteByte

```c
/**
 * @brief  串口发送一个字节
 * @note   阻塞式发送 只有发送完后才会返回
 * @code
 *      //使用UART0模块 发送数据0x5A
 *    UART_WriteByte(HW_UART0, 0x5A);
 * @endcode
 * @param  instance      :芯片串口端口
 *         @arg HW_UART0 :芯片的UART0端口
 *         @arg HW_UART1 :芯片的UART1端口
 *         @arg HW_UART2 :芯片的UART2端口
 *         @arg HW_UART3 :芯片的UART3端口
 *         @arg HW_UART4 :芯片的UART4端口
 *         @arg HW_UART5 :芯片的UART5端口
 * @param  ch: 需要发送的一字节数据
 * @retval None
 */
void UART_WriteByte(uint32_t instance, uint16_t ch)
{
	/* param check */
    assert_param(IS_UART_ALL_INSTANCE(instance));

    if(UART_InstanceTable[instance]->PFIFO & UART_PFIFO_TXFE_MASK)
    {
        /* buffer is used */
        while(UART_InstanceTable[instance]->TCFIFO >= UART_GetTxFIFOSize(instance));
    }
    else
    {
        /* no buffer is used */
        while(!(UART_InstanceTable[instance]->S1 & UART_S1_TDRE_MASK));
    }
    
    UART_InstanceTable[instance]->D = (uint8_t)(ch & 0xFF);
    
    /* config ninth bit */
    uint8_t ninth_bit = (ch >> 8) & 0x01U;
    (ninth_bit)?(UART_InstanceTable[instance]->C3 |= UART_C3_T8_MASK):(UART_InstanceTable[instance]->C3 &= ~UART_C3_T8_MASK);
}
```



#### UART_ReadByte

```c
/**
 * @brief  UART接受一个字节
 * @note   非阻塞式接收 立即返回
 * @code
 *      //接收UART0模块的数据
 *      uint8_t data; //申请变量，存储接收的数据
 *      UART_ReadByte(HW_UART0, &data);
 * @endcode
 * @param  instance      :芯片串口端口
 *         @arg HW_UART0 :芯片的UART0端口
 *         @arg HW_UART1 :芯片的UART1端口
 *         @arg HW_UART2 :芯片的UART2端口
 *         @arg HW_UART3 :芯片的UART3端口
 *         @arg HW_UART4 :芯片的UART4端口
 *         @arg HW_UART5 :芯片的UART5端口
 * @param  ch: 接收到的数据指针
 * @retval 0:成功接收到数据  非0:没有接收到数据
 */
uint8_t UART_ReadByte(uint32_t instance, uint16_t *ch)
{
	/* param check */
    assert_param(IS_UART_ALL_INSTANCE(instance));
    uint8_t temp = 0;
    if((UART_InstanceTable[instance]->S1 & UART_S1_RDRF_MASK) != 0)
    {
        /* get ninth bit */
        temp = (UART_InstanceTable[instance]->C3 & UART_C3_R8_MASK) >> UART_C3_R8_SHIFT;
        *ch = temp << 8;
        *ch |= (uint8_t)(UART_InstanceTable[instance]->D);	
        return 0; 		  
    }
    return 1;
}
```



#### UART_CallbackRxInstall

```c
/**
 * @brief  注册接收中断回调函数
 * @param  instance      :芯片串口端口
 *         @arg HW_UART0 :芯片的UART0端口
 *         @arg HW_UART1 :芯片的UART1端口
 *         @arg HW_UART2 :芯片的UART2端口
 *         @arg HW_UART3 :芯片的UART3端口
 *         @arg HW_UART4 :芯片的UART4端口
 *         @arg HW_UART5 :芯片的UART5端口
 * @param AppCBFun: 回调函数指针入口
 * @retval None
 * @note 对于此函数的具体应用请查阅应用实例
 */
void UART_CallbackRxInstall(uint32_t instance, UART_CallBackRxType AppCBFun)
{
	/* param check */
    assert_param(IS_UART_ALL_INSTANCE(instance));
    
    /* enable clock gate */
    *((uint32_t*) SIM_UARTClockGateTable[instance].addr) |= SIM_UARTClockGateTable[instance].mask;
    if(AppCBFun != NULL)
    {
        UART_CallBackRxTable[instance] = AppCBFun;
    }
}
```



#### UART_ITDMAConfig

```c
/**
 * @brief  配置UART模块的中断或DMA属性
 * @code
 *      //配置UART0模块开启接收中断功能
 *      UART_ITDMAConfig(HW_UART0, kUART_IT_Rx, true);
 * @endcode
 * @param  instance      :芯片串口端口
 *         @arg HW_UART0 :芯片的UART0端口
 *         @arg HW_UART1 :芯片的UART1端口
 *         @arg HW_UART2 :芯片的UART2端口
 *         @arg HW_UART3 :芯片的UART3端口
 *         @arg HW_UART4 :芯片的UART4端口
 *         @arg HW_UART5 :芯片的UART5端口
 * @param  status      :开关
 * @param  config: 工作模式选择
 *         @arg kUART_IT_Tx:
 *         @arg kUART_DMA_Tx:
 *         @arg kUART_IT_Rx:
 *         @arg kUART_DMA_Rx:
 * @retval None
 */
void UART_ITDMAConfig(uint32_t instance, UART_ITDMAConfig_Type config, bool status)
{
    /* enable clock gate */
    *((uint32_t*) SIM_UARTClockGateTable[instance].addr) |= SIM_UARTClockGateTable[instance].mask;
    switch(config)
    {
        case kUART_IT_Tx:
            (status)?
            (UART_InstanceTable[instance]->C2 |= UART_C2_TIE_MASK):
            (UART_InstanceTable[instance]->C2 &= ~UART_C2_TIE_MASK);
            NVIC_EnableIRQ(UART_IRQnTable[instance]);
            break; 
        case kUART_IT_Rx:
            (status)?
            (UART_InstanceTable[instance]->C2 |= UART_C2_RIE_MASK):
            (UART_InstanceTable[instance]->C2 &= ~UART_C2_RIE_MASK);
            NVIC_EnableIRQ(UART_IRQnTable[instance]);
            break;
        case kUART_DMA_Tx:
            (status)?
            (UART_InstanceTable[instance]->C2 |= UART_C2_TIE_MASK):
            (UART_InstanceTable[instance]->C2 &= ~UART_C2_TIE_MASK);
            (status)?
            (UART_InstanceTable[instance]->C5 |= UART_C5_TDMAS_MASK):
            (UART_InstanceTable[instance]->C5 &= ~UART_C5_TDMAS_MASK);
            break;
        case kUART_DMA_Rx:
            (status)?
            (UART_InstanceTable[instance]->C2 |= UART_C2_RIE_MASK):
            (UART_InstanceTable[instance]->C2 &= ~UART_C2_RIE_MASK);
            (status)?
            (UART_InstanceTable[instance]->C5 |= UART_C5_RDMAS_MASK):
            (UART_InstanceTable[instance]->C5 &= ~UART_C5_RDMAS_MASK);
            break;
        default:
            break;
    }
}
```



#### UART_CallbackTxInstall

```c
/**
 * @brief  注册发送中断回调函数
 * @param  instance      :芯片串口端口
 *         @arg HW_UART0 :芯片的UART0端口
 *         @arg HW_UART1 :芯片的UART1端口
 *         @arg HW_UART2 :芯片的UART2端口
 *         @arg HW_UART3 :芯片的UART3端口
 *         @arg HW_UART4 :芯片的UART4端口
 *         @arg HW_UART5 :芯片的UART5端口
 * @param AppCBFun: 回调函数指针入口
 * @retval None
 * @note 对于此函数的具体应用请查阅应用实例
 */
void UART_CallbackTxInstall(uint32_t instance, UART_CallBackTxType AppCBFun)
{
	/* param check */
    assert_param(IS_UART_ALL_INSTANCE(instance));
    
    /* enable clock gate */
    *((uint32_t*) SIM_UARTClockGateTable[instance].addr) |= SIM_UARTClockGateTable[instance].mask;
    if(AppCBFun != NULL)
    {
        UART_CallBackTxTable[instance] = AppCBFun;
    }
}
```




## 使用KEIL建立工程

1. 安装芯片**pack**
2. **Project** -> **New uVision Project**，保存工程，并选择芯片型号
3. 右键工程文件夹，选择**Manage Project Items**
   1. 建立**setup**和**user**文件组
   2. 在**setup**中添加启动文件`startup_MK60D10.s`和时钟配置文件`system_MK60D10.c`
   3. 在**user**中添加主程序`main.c`
4. 右键工程文件夹，选择**Options for Target**
   1. 选择**Debug**，设置**JLink**在线调试，**ort**设置为`SW`，**Max**为`5MHz`
   2. 选择**Utilities** -> **Settings** -> **Flash Download**，选择合适的编程文件（默认），勾选`Reset and Run`可在下载完成后自动运行

5. **Edit** -> **Configuration** -> **Editor**，在**C/C++ Files**中勾选**Insert spaces for tabs**，设置**Tab size** 为`4`
6. 

## 使用例程包驱动文件建立工程

2. 新建工程模板文件夹
   1. 在文件夹内新建**MDK**、**IAR**和**src**文件夹
   2. 在**src**中新建主程序`main.c`
3. **Project** -> **New uVision Project**，保存工程，并选择芯片型号
4. 右键工程文件夹，选择**Manage Project Items**
   1. 建立**setup**、**user**和**driver**文件组
   2. 在**setup**中添加启动文件`startup_MK60D10.s`和时钟配置文件`system_MK60D10.c`
   3. 在**user**中添加主程序`main.c`
   4. 在**driver**中添加`gpio.c`和`common.c`

5. 右键工程文件夹，选择**Options for Target**
   1. 选择**C/C++，**设置**Include Paths**
   
       1. 新建路径，手动选择到`\Libraries\startup\DeviceSupport`
   
       2. 新建路径，手动选择到`\Libraries\startup\CoreSupport`
       3. 新建路径，手动选择到`\Libraries\drivers\K\inc`
   2. 选择**C/C++**，设置**Preprocessor Symbols** -> **Define**，参考`common.h`
   3. 选择**C/C++**，勾选`C99 Mode`



## 位带操作(Bit-Band)

[Example](#按键分控灯)

## 时钟



## 系统节拍(SysTick)



## 串口(UART)

### 波特率

### 校验

| Name       | UART_ParityMode_Type   |
| ---------- | ---------------------- |
| 禁止校验位 | `kUART_ParityDisabled` |
| 偶校验     | `kUART_ParityEven`     |
| 奇校验     | `kUART_ParityOdd`      |

#### 偶校验

对于偶校验

* `1`的个数为偶数时，校验位是`0`；
* `1`的个数为奇数时，校验位是`1`。

> With even parity, an even number of 1s clears the parity bit and an odd number of 1s sets the parity bit. 

#### 奇校验

对于奇校验

* `1`的个数为奇数时，校验位是`0`；
* `1`的个数为偶数时，校验位是`1`。

> With odd parity, an odd number of 1s clears the parity bit and an even number of 1s sets the parity bit. 

### 串口发送与接收

串口发送常用轮询方式，串口接收常用中断方式

|       |          |                                             |                 |                                                              |
| ----- | -------- | ------------------------------------------- | --------------- | ------------------------------------------------------------ |
| ***** | 轮询发送 | `UART_WriteByte`                            | 阻塞式发送      | [Function](#UART_WriteByte),[Example](#UART轮询发送)         |
|       | 轮询接收 | `UART_ReadByte`                             |                 | [Function](#UART_ReadByte),[Example](#UART轮询接收)          |
| ***** | 中断接收 | `UART_CallbackRxInstall`,`UART_ITDMAConfig` | 对CPU资源消耗少 | [Function1](#UART_CallbackRxInstall),[Function2](#UART_ITDMAConfig),[Example](#UART中断接收) |
|       | 中断发送 | `UART_CallbackTxInstall`,`UART_ITDMAConfig` |                 | [Function1](#UART_CallbackTxInstall),[Function2](#UART_ITDMAConfig),[Example](#UART中断发送) |



## 案例

### 亮灯

```c
#include "MK60D10.h"

int main(void)
{
    //open PORTE clock
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    //config PE6 alt1
    PORTE->PCR[6] &= ~PORT_PCR_MUX_MASK;//led1
    PORTE->PCR[6] |= PORT_PCR_MUX(1);//led1
    //set registers
    PTE->PDDR |= (1 << 6);//set PE6 to output
    PTE->PDOR &= ~(1 << 6);//set PE6 low
}

```

### 蜂鸣器

```c
#include "MK60D10.h"

int main(void)
{
    //open PORTA clock
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    //config PA6 alt1
    PORTA->PCR[6] &= ~PORT_PCR_MUX_MASK;//buzzer
    PORTA->PCR[6] |= PORT_PCR_MUX(1);//buzzer
    //set registers
    PTA->PDDR |= (1 << 6);//set PA6 to output
    PTA->PDOR &= ~(1 << 6);//set PA6 high
}

```

### 按键灯亮

```c
#include "MK60D10.h"

int main(void)
{
    //open PORT clock
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    //config PE6 alt1
    PORTE->PCR[6] &= ~PORT_PCR_MUX_MASK;//led
    PORTE->PCR[6] |= PORT_PCR_MUX(1);//led
    //config PE26 alt1
    PORTE->PCR[26] &= ~PORT_PCR_MUX_MASK;//key
    PORTE->PCR[26] |= PORT_PCR_MUX(1);//key
    //set PE6 to output
    PTE->PDDR |= (1 << 6);
    //set PE26 to input
    PTE->PDDR &= ~(1 << 26);
    //set PE26 to pull-up
    PORTE->PCR[26] |= PORT_PCR_PE_MASK;
    PORTE->PCR[26] |= PORT_PCR_PS_MASK;
    
    while(1)
    {
        //press key
        if((PTE->PDIR >> 26) == 0)
        {
             PTE->PDOR &= ~(1 << 6);
        }
        else
        {
            PTE->PDOR |= (1 << 6);
        }
    }
}

```

## 案例（函数）

### 按键灯亮

```c
#include "gpio.h"
#include "common.h"

#define LED1 PEout(6)//bit-band
#define KEY1 PEin(26)
 
int main(void)
{  
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);//set output
    GPIO_QuickInit(HW_GPIOE, 26, kGPIO_Mode_IPU);//set pull-up
    while(1)
    {
        if(KEY1 == 0) LED1 = 0; else LED1 = 1;//judge KEY1
    }
}

```

### 按键分控灯

```c
#include "gpio.h"
#include "common.h"

#define LED1 PEout(6)//bit-band
#define LED2 PEout(7)
#define KEY1 PEin(26)
#define KEY2 PEin(27)

int main(void)
{   
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);//set output
    GPIO_QuickInit(HW_GPIOE, 7, kGPIO_Mode_OPP);
    GPIO_QuickInit(HW_GPIOE, 26, kGPIO_Mode_IPU);//set pull-up
    GPIO_QuickInit(HW_GPIOE, 27, kGPIO_Mode_IPU);
    while(1)
    {
        if(KEY1 == 0) LED1 = 0; else LED1 = 1;//judge KEY1
        if(KEY2 == 0) LED2 = 0; else LED2 = 1;//judge KEY2
    }
}

```

### 流水灯

```c
#include "gpio.h"
#include "common.h"

int main(void)
{   
    DelayInit();
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);//set output
    GPIO_QuickInit(HW_GPIOE, 7, kGPIO_Mode_OPP);
    GPIO_QuickInit(HW_GPIOE, 11, kGPIO_Mode_OPP);
    GPIO_QuickInit(HW_GPIOE, 12, kGPIO_Mode_OPP);
    //GPIO_QuickInit(HW_GPIOA, 6, kGPIO_Mode_OPP);
    while(1)
    {
        GPIO_ToggleBit(HW_GPIOE, 6); DelayMs(100);//led1
        GPIO_ToggleBit(HW_GPIOE, 7); DelayMs(100);//led2
        GPIO_ToggleBit(HW_GPIOE, 11); DelayMs(100);//led3
        GPIO_ToggleBit(HW_GPIOE, 12); DelayMs(100);//led4
        //GPIO_ToggleBit(HW_GPIOA, 6); DelayMs(100);//buzzer
    }
}

```

### *按键扫描

```c
#include "gpio.h"
#include "common.h"

#define KEY1  PEin(26)
#define LED1  PEout(6)

#define NO_KEY          (0x00)
#define KEY_SINGLE      (0x01)

static uint8_t gRetValue;

typedef enum
{
    kKEY_Idle,
    kKEY_Debounce,
    kKEY_Confirm,
}KEY_Status;

static void KEY_Scan(void)
{
    static KEY_Status status = kKEY_Idle;
    switch(status)
    {
        case kKEY_Idle:
            gRetValue = NO_KEY;
            if(KEY1 == 0)
            {
                status = kKEY_Debounce;
            }
            break;
        case kKEY_Debounce:
            if(KEY1 == 0)
            {
                status = kKEY_Confirm;
            }
            else
            {
                status = kKEY_Idle;
                gRetValue = NO_KEY;
            }
            break;
        case kKEY_Confirm: 
            if(KEY1 == 1) 
            {
                gRetValue = KEY_SINGLE;
                status = kKEY_Idle;
            }
            break;
        default:
            break;
    }
}

int main(void)
{
    DelayInit();
    GPIO_QuickInit(HW_GPIOE, 26, kGPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOE,  6, kGPIO_Mode_OPP);
    while(1)
    {
        KEY_Scan(); 
        DelayMs(10);
        if(gRetValue == KEY_SINGLE) 
        {
            LED1 = !LED1;
        }
    }
}

```

### UART轮询发送

```c
#include "gpio.h"
#include "common.h"
#include "uart.h"

int main(void)
{
    uint32_t instance;
    DelayInit();
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);
    instance = UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);//quickinit GPIO return UART hardware num
    printf("\rUART%d OK! Hello Li Ning\r\n", instance);
    while(1)
    {
        UART_WriteByte(instance, 'h');
        UART_WriteByte(instance, 'e');
        UART_WriteByte(instance, 'l');
        UART_WriteByte(instance, 'l');
        UART_WriteByte(instance, 'o');
        UART_WriteByte(instance, '\r');
        UART_WriteByte(instance, '\n');
        GPIO_ToggleBit(HW_GPIOE, 6); DelayMs(500);
    }
}

```

### UART轮询接收

```c
#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "cpuidy.h"
 
int main(void)
{
    uint16_t ch;
    DelayInit();
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);
    
    UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
    
    /* 打印信息 */
    printf("\rtype any character whitch will echo your input...\r\n");
    
    while(1)
    {
        /* 不停的查询 串口接收的状态 一旦接收成功 返回0 发送回接收到的数据 实现回环测试*/
        if(UART_ReadByte(HW_UART0, &ch) == 0)
        {
            /****************打印提示信息**************************/
            UART_WriteByte(HW_UART0,'R');
            UART_WriteByte(HW_UART0,'e');
            UART_WriteByte(HW_UART0,'c');
            UART_WriteByte(HW_UART0,'v');
            UART_WriteByte(HW_UART0,':');
            UART_WriteByte(HW_UART0,' ');
            /****************将接收到的字节打印出来****************/
            UART_WriteByte(HW_UART0, ch);
            /***********每次接受的数据打印完之后换行***************/
            UART_WriteByte(HW_UART0,'\r');
            UART_WriteByte(HW_UART0,'\n');				
        }
    }
}



```



### UART打印信息

```c
#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "cpuidy.h"

int main(void)
{   
    uint32_t clock;
    DelayInit();
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);//set output
    UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
    printf("\r%s - %dP\r\n", CPUIDY_GetFamID(), CPUIDY_GetPinCount());
    CLOCK_GetClockFrequency(kCoreClock, &clock);//get core clock
    printf("core clock: %dHz\r\n", clock);//print core clock
    CLOCK_GetClockFrequency(kSystemClock, &clock);
    printf("system clock:%dHz\r\n", clock);
    CLOCK_GetClockFrequency(kBusClock, &clock);
    printf("bus clock: %dHz\r\n", clock);
    while(1)
    {
        GPIO_ToggleBit(HW_GPIOE, 6); DelayMs(150);//led1
    }
}

```

### UART中断接收

```c
#include "gpio.h"
#include "common.h"
#include "uart.h"

static void UART_RX_ISR(uint16_t byteReceived)
{
    /* 将接收到的数据发送回去 */
    UART_WriteByte(HW_UART0, byteReceived);
}

 
int main(void)
{
    DelayInit();
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);
    UART_QuickInit(UART0_RX_PD06_TX_PD07 , 115200);
    printf("\rtype any character which will echo...\r\n");
    
    /*  配置UART 中断配置 打开接收中断 安装中断回调函数 */
    UART_CallbackRxInstall(HW_UART0, UART_RX_ISR);
    /* 打开串口接收中断功能 IT 就是中断的意思*/
    UART_ITDMAConfig(HW_UART0, kUART_IT_Rx, true);
    
    while(1)
    {
        GPIO_ToggleBit(HW_GPIOE, 6);
        DelayMs(500);
    }
}



```

### UART中断发送

```c
#include "gpio.h"
#include "common.h"
#include "uart.h"

static const char UART_String1[] = "HelloWorld\r\n";

static void UART_TX_ISR(uint16_t * byteToSend)
{
    static const char *p = UART_String1;
    *byteToSend = *p++;
    if((p - UART_String1) == sizeof(UART_String1))
    {
        p = UART_String1;
        UART_ITDMAConfig(HW_UART0, kUART_IT_Tx, false);  
    }
}

int main(void)
{
    DelayInit();
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);
    
    UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
    
    /** print message before mode change*/
    printf("\ruart will be send on interrupt mode...\r\n");
    
    /** register callback function*/
    UART_CallbackTxInstall(HW_UART0, UART_TX_ISR);
    
    /** open TX interrupt */
    UART_ITDMAConfig(HW_UART0, kUART_IT_Tx, true);
    
    /**main loop*/
    while(1)
    {
        /** indicate program is running */
        GPIO_ToggleBit(HW_GPIOE, 6);
        DelayMs(500);
    }
}


```

