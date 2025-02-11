#include "dr16.h"
//#include "stream_buffer.h"
//#include "chassis_task.h"
//#include "online_ctrl_task.h"

RC_Ctrl rc_Ctrl = {.isUnpackaging = 0,
                   .is_online      = 0};

/**
 * @brief  创建dr16的接收机缓存数组
 */
uint8_t DR16_recData[DR16_rxBufferLengh] __attribute__((at(0x24006010)));
uint8_t DR16_RxBuffer0[36];
uint8_t DR16_RxBuffer1[36];

/**
 * @brief  创建dr16串口缓存区数据结构
 */
UART_RxBuffer_t uart1_buffer = {
    .Data = DR16_recData,
    .Size = DR16_rxBufferLengh};

/**
 * @brief 遥控器串口中断回调函数
 *
 * @param recBuffer
 * @param len
 * @return uint8_t
 */
uint8_t DR16_Callback(uint8_t *recBuffer, uint16_t len)
{
    
    DR16_DataUnpack(&rc_Ctrl, recBuffer);
	if(rc_Ctrl.FPS > 30)
	rc_Ctrl.online_cnt = 0;
	
    return 0;
}

/**
 * @brief  初始化接收机数据类型的数据，将杆量和按键信息归零
 */
void DR16Init(RC_Ctrl *rc_ctrl)
{
    rc_ctrl->rc.ch0          = 1024;
    rc_ctrl->rc.ch1          = 1024;
    rc_ctrl->rc.ch2          = 1024;
    rc_ctrl->rc.ch3          = 1024;
    rc_ctrl->rc.s1           = 3;
    rc_ctrl->rc.s2           = 3;
    rc_ctrl->rc.sw           = 1024;
    rc_ctrl->mouse.x         = 0;
    rc_ctrl->mouse.y         = 0;
    rc_ctrl->mouse.z         = 0;
    rc_ctrl->key_Q_flag      = 0;
    rc_ctrl->key_E_flag      = 0;
    rc_ctrl->key_R_flag      = 0;
    rc_ctrl->key_F_flag      = 0;
    rc_ctrl->key_G_flag      = 0;
    rc_ctrl->key_Z_flag      = 0;
    rc_ctrl->key_X_flag      = 0;
    rc_ctrl->key_C_flag      = 0;
    rc_ctrl->key_V_flag      = 0;
    rc_ctrl->key_B_flag      = 0;
    rc_ctrl->key_ctrl_flag   = 0;
    rc_ctrl->chassis_y_integ = 0; // 斜坡积分变量
    rc_ctrl->chassis_x_integ = 0;
	rc_ctrl->online_cnt      = 50;
}

/**
 * @brief  创建dr16的接收机缓存数组, 并对全局变量rc_Ctrl赋值，以供其他函数调用
 */
void DR16_DataUnpack(RC_Ctrl *rc_ctrl, uint8_t *recBuffer)
{
	rc_ctrl->FPS++;
	if (rc_ctrl->FPS>1000)   rc_ctrl->FPS=1000;
    // 解算期间不允许读取数据
    rc_ctrl->isUnpackaging = 1;
    uint8_t correct_num    = 0;
    correct_num            = 0;
    if (((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff) <= 1684 && ((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff) >= 364)
        correct_num++;
    if ((((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff) <= 1684 && (((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff) >= 364)
        correct_num++;
    if ((((recBuffer[2] >> 6) | (recBuffer[3] << 2) | (recBuffer[4] << 10)) & 0x07ff) <= 1684 && (((recBuffer[2] >> 6) | (recBuffer[3] << 2) | (recBuffer[4] << 10)) & 0x07ff) >= 364)
        correct_num++;
    if ((((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff) <= 1684 && (((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff) >= 364)
        correct_num++;
    if ((((recBuffer[5] >> 4) & 0x000C) >> 2) == 1 || (((recBuffer[5] >> 4) & 0x000C) >> 2) == 2 || (((recBuffer[5] >> 4) & 0x000C) >> 2) == 3)
        correct_num++;
    if (((recBuffer[5] >> 4) & 0x0003) == 1 || ((recBuffer[5] >> 4) & 0x0003) == 2 || ((recBuffer[5] >> 4) & 0x0003) == 3)
        correct_num++;
    if (correct_num == 6) {
        rc_ctrl->rc.ch0 = (recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff;
        rc_ctrl->rc.ch1 = ((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff;
        rc_ctrl->rc.ch2 = ((recBuffer[2] >> 6) | (recBuffer[3] << 2) | (recBuffer[4] << 10)) & 0x07ff;
        rc_ctrl->rc.ch3 = ((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff;
        rc_ctrl->rc.s1  = ((recBuffer[5] >> 4) & 0x000C) >> 2;
        rc_ctrl->rc.s2  = ((recBuffer[5] >> 4) & 0x0003);
        rc_ctrl->rc.sw  = (uint16_t)(recBuffer[16] | (recBuffer[17] << 8)) & 0x7ff;

        if ((rc_ctrl->rc.ch0 > 1020) && (rc_ctrl->rc.ch0 < 1028)) // 遥控器零飘
            rc_ctrl->rc.ch0 = 1024;
        if ((rc_ctrl->rc.ch1 > 1020) && (rc_ctrl->rc.ch1 < 1028))
            rc_ctrl->rc.ch1 = 1024;
        if ((rc_ctrl->rc.ch2 > 1020) && (rc_ctrl->rc.ch2 < 1028))
            rc_ctrl->rc.ch2 = 1024;
        if ((rc_ctrl->rc.ch3 > 1020) && (rc_ctrl->rc.ch3 < 1028))
            rc_ctrl->rc.ch3 = 1024;
        /***********按键映射*************/
        rc_ctrl->mouse.x       = recBuffer[6] | (recBuffer[7] << 8);   //< Mouse X axis
        rc_ctrl->mouse.y       = recBuffer[8] | (recBuffer[9] << 8);   //< Mouse Y axis
        rc_ctrl->mouse.z       = recBuffer[10] | (recBuffer[11] << 8); //< Mouse Z axis
        rc_ctrl->mouse.press_l = recBuffer[12];                        //< Mouse Left Is Press ?
        rc_ctrl->mouse.press_r = recBuffer[13];                        //< Mouse Right Is Press ?

        if (rc_ctrl->mouse.x > 25000)
            rc_ctrl->mouse.x = 25000; // 限幅
        if (rc_ctrl->mouse.x < -25000)
            rc_ctrl->mouse.x = -25000;
        if (rc_ctrl->mouse.y > 25000)
            rc_ctrl->mouse.y = 25000;
        if (rc_ctrl->mouse.y < -25000)
            rc_ctrl->mouse.y = -25000;

        rc_ctrl->key_W     = recBuffer[14] & 0x01;
        rc_ctrl->key_S     = (recBuffer[14] >> 1) & 0x01;
        rc_ctrl->key_A     = (recBuffer[14] >> 2) & 0x01;
        rc_ctrl->key_D     = (recBuffer[14] >> 3) & 0x01;
        rc_ctrl->key_B     = (recBuffer[15] >> 7) & 0x01;
        rc_ctrl->key_V     = (recBuffer[15] >> 6) & 0x01;
        rc_ctrl->key_C     = (recBuffer[15] >> 5) & 0x01;
        rc_ctrl->key_X     = (recBuffer[15] >> 4) & 0x01;
        rc_ctrl->key_Z     = (recBuffer[15] >> 3) & 0x01;
        rc_ctrl->key_G     = (recBuffer[15] >> 2) & 0x01;
        rc_ctrl->key_F     = (recBuffer[15] >> 1) & 0x01;
        rc_ctrl->key_R     = (recBuffer[15]) & 0x01;
        rc_ctrl->key_E     = (recBuffer[14] >> 7) & 0x01;
        rc_ctrl->key_Q     = (recBuffer[14] >> 6) & 0x01;
        rc_ctrl->key_ctrl  = (recBuffer[14] >> 5) & 0x01;
        rc_ctrl->key_shift = (recBuffer[14] >> 4) & 0x01;
        PC_keybroad_filter(rc_ctrl); // 防抖
    }

    rc_ctrl->isUnpackaging = 0; // 解算完成标志位，允许读取

}

/**
 * @brief  	按键消抖,检测是否为有效按下,支持连续按 W A S D
 * @note	不同按键与flag的对应逻辑不同，应分别查找
 */
void PC_keybroad_filter(RC_Ctrl *rc_ctrl)
{
    static uint16_t key_W_cnt, key_A_cnt, key_S_cnt, key_D_cnt, key_ctrl_cnt,
        key_shift_cnt, mouse_press_l_cnt, mouse_press_r_cnt,
        key_C_cnt, key_F_cnt, key_G_cnt, key_Q_cnt, key_E_cnt,
        key_Z_cnt, key_V_cnt, key_X_cnt, key_B_cnt, key_R_cnt;

    if (rc_ctrl->key_W == 1) {
        key_W_cnt++;
        if (key_W_cnt == Key_Filter_Num) {
            rc_ctrl->key_W_flag = 1;
        }
    } else {
        rc_ctrl->key_W_flag = 0;
        key_W_cnt           = 0;
    }

    if (rc_ctrl->key_A == 1) {
        key_A_cnt++;
        if (key_A_cnt == Key_Filter_Num) {
            rc_ctrl->key_A_flag = 1;
        }
    } else {
        key_A_cnt           = 0;
        rc_ctrl->key_A_flag = 0;
    }

    if (rc_ctrl->key_S == 1) {
        key_S_cnt++;
        if (key_S_cnt == Key_Filter_Num) {
            rc_ctrl->key_S_flag = 1;
        }
    } else {
        key_S_cnt           = 0;
        rc_ctrl->key_S_flag = 0;
    }

    if (rc_ctrl->key_D == 1) {
        key_D_cnt++;
        if (key_D_cnt == Key_Filter_Num) {
            rc_ctrl->key_D_flag = 1;
        }
    } else {
        key_D_cnt           = 0;
        rc_ctrl->key_D_flag = 0;
    }

    if (rc_ctrl->key_B == 1) {
        key_B_cnt++;
        if (key_B_cnt == Key_Filter_Num) {
            rc_ctrl->key_B_flag = 1;
        }
    } else {
        key_B_cnt           = 0;
        rc_ctrl->key_B_flag = 0;
    }

    if (rc_ctrl->key_C == 1) {
        key_C_cnt++;
        if (key_C_cnt == Key_Filter_Num) {
            rc_ctrl->key_C_flag = 1;
        }
    } else {
        key_C_cnt           = 0;
        rc_ctrl->key_C_flag = 0;
    }

    if (rc_ctrl->key_R == 1) {
        key_R_cnt++;
        if (key_R_cnt == Key_Filter_Num) {
            rc_ctrl->key_R_flag++;
        }
    } else {
        key_R_cnt = 0;
        // rc_ctrl.key_R_flag=0;
    }

    if (rc_ctrl->key_F == 1) {
        key_F_cnt++;
        if (key_F_cnt == Key_Filter_Num) {
            rc_ctrl->key_F_flag = !rc_ctrl->key_F_flag;
        }
    } else {
        key_F_cnt = 0;
    }

    if (rc_ctrl->key_X == 1) {
        key_X_cnt++;
        if (key_X_cnt == Key_Filter_Num) {
            rc_ctrl->key_X_flag = 1;
        }
    } else {
        key_X_cnt           = 0;
        rc_ctrl->key_X_flag = 0;
    }

    if (rc_ctrl->key_G == 1) {
        key_G_cnt++;
        if (key_G_cnt == Key_Filter_Num) {
            rc_ctrl->key_G_flag++;
        }
    } else {
        key_G_cnt = 0;
        // rc_ctrl.key_G_flag=0;
    }

    if (rc_ctrl->key_Q == 1) {
        key_Q_cnt++;
        if (key_Q_cnt == Key_Filter_Num) {
            rc_ctrl->key_Q_flag = 1;
        }
    } else {
        key_Q_cnt           = 0;
        rc_ctrl->key_Q_flag = 0;
    }

    if (rc_ctrl->key_E == 1) {
        key_E_cnt++;
        if (key_E_cnt == Key_Filter_Num) {
            rc_ctrl->key_E_flag++;
            if (rc_ctrl->key_E_flag > 1) {
                rc_ctrl->key_E_flag = 0;
            }
        }
    } else {
        key_E_cnt = 0;
        // rc_ctrl.key_E_flag=0;
    }

    if (rc_ctrl->key_Z == 1) {
        key_Z_cnt++;
        if (key_Z_cnt == Key_Filter_Num) {
            rc_ctrl->key_Z_flag = 1;
        }
    } else {
        key_Z_cnt           = 0;
        rc_ctrl->key_Z_flag = 0;
    }

    if (rc_ctrl->key_V == 1) {
        key_V_cnt++;
        if (key_V_cnt == Key_Filter_Num) {
            rc_ctrl->key_V_flag = 1;
        }
    } else {
        key_V_cnt           = 0;
        rc_ctrl->key_V_flag = 0;
    }

    if (rc_ctrl->key_ctrl == 1) {
        key_ctrl_cnt++;
        if (key_ctrl_cnt == Key_Filter_Num) {
            rc_ctrl->key_ctrl_flag = 1;
            key_ctrl_cnt           = 0;
        }
    } else {
        rc_ctrl->key_ctrl_flag = 0;
    }

    if (rc_ctrl->key_shift == 1) {
        key_shift_cnt++;
        if (key_shift_cnt == Key_Filter_Num) {
            rc_ctrl->key_shift_flag = 1;
        }
    } else {
        rc_ctrl->key_shift_flag = 0;
    }

    if (rc_ctrl->mouse.press_l == 1) {
        mouse_press_l_cnt++;
        if (mouse_press_l_cnt == Key_Filter_Num) {
            rc_ctrl->mouse.press_l_flag = 1;
            mouse_press_l_cnt           = 0;
        }
    } else {
        rc_ctrl->mouse.press_l_flag = 0;
    }

    if (rc_ctrl->mouse.press_r == 1) {
        mouse_press_r_cnt++;
        if (mouse_press_r_cnt == Key_Filter_Num) {
            rc_ctrl->mouse.press_r_flag = 1;
            mouse_press_r_cnt           = 0;
        }
    } else {
        rc_ctrl->mouse.press_r_flag = 0;
    }
}
