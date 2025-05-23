/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "DBUS_remote_control.h"
#include "main.h"
#include "Error_detect.h"
#include "Stm32_time.h"
// 此处移植官方

#define Ka 0.03f // 滤波系数
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RC_ctrl_t RC_data;
float DBUStime1;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
  // 使能DMA串口接收
  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

  // 使能空闲中断
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart3_rx);
  while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

  hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);

  // 内存缓冲区1
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);

  // 内存缓冲区2
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);

  // 数据长度
  hdma_usart3_rx.Instance->NDTR = dma_buf_num;

  // 使能双缓冲区
  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

  // 使能DMA
  __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          遥控器协议解析
 * @param[in]      sbus_buf: 原生数据指针
 * @param[out]     rc_ctrl: 遥控器数据指
 * @retval         none
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// receive data, 18 bytes one frame, but set 36 bytes
// 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief          remote control init
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          遥控器初始化
 * @param[in]      none
 * @retval         none
 */
void remote_control_init(void)
{
  RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
 * @brief          get remote control data point
 * @param[in]      none
 * @retval         remote control data point
 */
/**
 * @brief          获取遥控器数据指针
 * @param[in]      none
 * @retval         遥控器数据指针
 */
const RC_ctrl_t *get_remote_control_point(void)
{
  return &RC_data;
}

// 串口中断
void Dbus_USARTx_IRQHandler(void) // 过于耦合，请求更改 by：sethome
{
  DBUStime1=Get_sys_time_ms();
  if (huart3.Instance->SR & UART_FLAG_RXNE) // 接收到数据
  {
    __HAL_UART_CLEAR_PEFLAG(&huart3);
  }
  else if (USART3->SR & UART_FLAG_IDLE)
  {
    static uint16_t this_time_rx_len = 0;

    __HAL_UART_CLEAR_PEFLAG(&huart3);

    Error_detect_remote();

    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
      /* Current memory buffer used is Memory 0 */

      // 失效DMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);

      // 获取接收数据长度,长度 = 设定长度 - 剩余长度
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

      // 重新设定数据长度
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

      // 设定缓冲区1
      hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

      // 使能DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == RC_FRAME_LENGTH)
      {
        sbus_to_rc(sbus_rx_buf[0], &RC_data);
        //				romote_filter();
      }
    }
    else
    {
      /* Current memory buffer used is Memory 1 */
      // 失效DMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);

      // 获取接收数据长度,长度 = 设定长度 - 剩余长度
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

      // 重新设定数据长度
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

      // 设定缓冲区0
      DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

      // 使能DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == RC_FRAME_LENGTH)
      {
        // 处理遥控器数据
        sbus_to_rc(sbus_rx_buf[1], &RC_data);
        //				romote_filter();
      }
    }
  }
}

/**
 * @brief          遥控器协议解析
 * @param[in]      sbus_buf: 原生数据指针
 * @param[out]     rc_ctrl: 遥控器数据指
 * @retval         none
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{ 

  if (sbus_buf == NULL || rc_ctrl == NULL)
  {
    return;
  }

  rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
  rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
  rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                       (sbus_buf[4] << 10)) &
                      0x07ff;
  rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
  rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
  rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
  rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
  rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
  rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
  rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
  rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
  rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
  rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // NULL

  rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

  
}


void image_to_RC(RC_ctrl_t *rc_ctrl){

  //
//  rc_ctrl->mouse.x = remote_control.mouse_x;                    //!< Mouse X axis
//  rc_ctrl->mouse.y = remote_control.mouse_y;                    //!< Mouse Y axis
//  rc_ctrl->mouse.z = remote_control.mouse_z;                  //!< Mouse Z axis
//  rc_ctrl->mouse.press_l = remote_control.left_button_down;                                  //!< Mouse Left Is Press ?
//  rc_ctrl->mouse.press_r = remote_control.right_button_down;                                  //!< Mouse Right Is Press ?
//  rc_ctrl->key.v = remote_control.keyboard_value;             //!< KeyBoard value
  rc_ctrl->rc.s[0] = 1;                  //直接使用键盘鼠标模式
  rc_ctrl->rc.s[1] = 1;                  //直接使用键盘鼠标模式

  rc_ctrl->rc.ch[0] = 0;
  rc_ctrl->rc.ch[1] = 0;
  rc_ctrl->rc.ch[2] = 0;
  rc_ctrl->rc.ch[3] = 0;
  rc_ctrl->rc.ch[4] = 0;

}
int filtering_algorithm(int data, uint8_t channel) // 遥控器数据滤波算法
{

  static int filtered_data[5] = {0};

  if (filtered_data[channel] < 50 && filtered_data[channel] > -50)
    filtered_data[channel] = data;
  else
    filtered_data[channel] = Ka * (data - filtered_data[channel]) + filtered_data[channel];

  return filtered_data[channel];
}

void romote_filter(void) // 滤波调用
{
  RC_data.rc.ch[0] = filtering_algorithm(RC_data.rc.ch[0], 0);
  RC_data.rc.ch[1] = filtering_algorithm(RC_data.rc.ch[1], 1);
  RC_data.rc.ch[2] = filtering_algorithm(RC_data.rc.ch[2], 2);
  RC_data.rc.ch[3] = filtering_algorithm(RC_data.rc.ch[3], 3);
  RC_data.rc.ch[4] = filtering_algorithm(RC_data.rc.ch[4], 4);
}
