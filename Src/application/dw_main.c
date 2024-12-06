#include "instance.h"

uint8_t switch8 = 0b00000001;                           //拨码开关键值
uint8_t instance_mode = ANCHOR;                         //设备运行角色
uint8_t dev_id;                                         //设备ID
uint8_t group_id;                                       //组ID
uint8_t anc_id;                                         //如当前角色是基站，则表示当前基站ID
uint8_t tag_id;                                         //如当前角色是标签，则表示当前标签ID
uint8_t state = STA_IDLE;                               //状态机状态控制
int32_t distance_report[8];                             //基站测距值数组，用于打包输出
int32_t group_report[8];                                //基站组ID数组，用于打包输出
uint32_t range_time;                                    //测距产生时间，串口打包发送
uint8_t frame_seq_nb = 0;                               //每帧数据增加1
uint8_t range_nb = 0;                                   //每次range增加1(poll resp1~4 fianl维护一套range_nb)
uint8_t recv_tag_id;                                    //如当前角色是基站，则表示当前基站收到标签发送过来数据的标签ID
uint8_t recv_anc_id;                                    //如当前角色是标签，则表示当前标签收到基站发送过来数据的基站ID
uint8_t range_status = RANGE_NULL;                      //测距成功标志位，用于打包输出
float rx_power;                                         //接收RSSI
#if defined (ULM1) || defined (ULM3) || defined (ULM3_PA)
static char lcd_data[10];                               //OLED显示数据
#endif
uint16_t inst_slot_number;                              //系统内最大标签容量
uint8_t inst_dataRate;                                  //通信速率，用于根据当前110K还是6.8M确定数据超时等通信过程相关参数
uint8_t inst_ch;                                        //信道号Channel number
uint8_t inst_prf;                                       //PRF
uint8_t inst_one_slot_time;                             //一个slot的时间，根据通信速率不同而不同，单位ms
uint32_t inst_final_rx_timeout;                         //基站final接收超时时间，根据通信速率不同而不同，单位us
uint32_t inst_resp_rx_timeout;                          //标签发送poll后接收resp超时时间，根据通信速率不同而不同，单位us
uint64_t inst_poll2final_time;                          //单TWR周期poll起始到final结束的总时间
uint32_t inst_data_interval;                            //相邻两条数据的间隔，如poll和第一个resp的间隔，resp1和resp2的间隔，根据通信速率不同而不同，单位us
uint16 ant_dly = ANT_DLY;                               //天线延时
uint32 tx_power;                                        //发射增益代码
uint8_t UART_RX_BUF[200];                               //串口接收BUF
uint32_t uart_rx_len;                                   //串口接收数据长度
vec3d anchorArray[8];                                   //基站坐标，用于标签解算自身位置
double distance_now_m;                                  //基站计算本周期测距结果，单位米
int32 distance_offset_cm;                               //距离校准，单位cm
uint8_t sos = 0;
uint8_t alarm = 0;
uint8_t battery = 0;
int user_data[10];
uint8_t USE_IMU;

#if defined(ANCRANGE)
uint8_t temp_dev_id;
uint8_t ancrange_flag = 0;
uint8_t ancrange_count = 0;
uint8_t target_ancid = 0;
#endif

/*******************函数声明***************************/
void parse_uart(uint8_t* data);
void read_anc_coord(void);
void print_config(void);


void set_instance(void)
{
	led_off(LED_ALL);

    if(instance_mode == ANCHOR)
    {
        //设置基站的中断回调函数

        dwt_setcallbacks(&anc_tx_conf_cb, &anc_rx_ok_cb, &anc_rx_to_cb, &anc_rx_err_cb, NULL, NULL);

    }
    else
    {   //设置标签的中断回调函数
        dwt_setcallbacks(&tag_tx_conf_cb, &tag_rx_ok_cb, &tag_rx_to_cb, &tag_rx_err_cb, NULL, NULL);
    }

    //按角色初始化设备短地址，设置状态机初始状态
    if(instance_mode == ANCHOR)
    {
        /* 设备短地址为2个字节，为了区分A0和T0短地址，基站的最高位为1
        * 如A1短地址=0x8001，T1短地址=0x0001
        */
        uint16_t anc_short_add = 0x8000 | dev_id;
        dwt_setaddress16(anc_short_add);
        anc_id = dev_id;

#if defined (ULM1) || defined (ULM3) || defined (ULM3_PA)
        if(USE_OLED == 1)
        {
            sprintf(lcd_data,"Anc:%d",dev_id);
            lcd_display(1, 4, lcd_data);
        }
#endif

        if(anc_id == 0)  //A0的group ID最高bit设置为1，为时序校准基站
        {
            group_id = group_id | 0x80;
        }
        dwt_forcetrxoff();
        state = STA_INIT_POLL_SYNC;

    }
    else //tag
    {
        dwt_setaddress16(dev_id);
        tag_id = dev_id;
#if defined (ULM1) || defined (ULM3) || defined (ULM3_PA)
        if(USE_OLED == 1)
        {
            sprintf(lcd_data,"Tag:%d",dev_id);
            lcd_display(1, 4, lcd_data);
        }
#endif
        dwt_forcetrxoff();
        state = STA_IDLE;
    }
}


#if defined(ULM3)|| defined (ULM3_PA)
void DW3000_init(void)
{
	uint8_t e2prom_data_read[EEP_UNIT_SIZE] = {0};
    uint8_t e2prom_data_write[EEP_UNIT_SIZE] = {0};

    static dwt_config_t config1 = {
        .chan = 5,                           /* Channel number. */
        .txPreambLength = DWT_PLEN_256,      /* Preamble length. Used in TX only. */
        .rxPAC = DWT_PAC16,                  /* Preamble acquisition chunk size. Used in RX only. */
        .txCode = 9,                         /* TX preamble code. Used in TX only. */
        .rxCode = 9,                         /* RX preamble code. Used in RX only. */
        .sfdType = 1,                        /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
        .dataRate = DWT_BR_850K,             /* Data rate. */
        .phrMode = DWT_PHRMODE_STD,          /* PHY header mode. */
        .phrRate = DWT_PHRRATE_STD,          /* PHY header rate. */
        .sfdTO = (257 + 16 - 16),            /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        .stsMode = DWT_STS_MODE_1 | DWT_STS_MODE_SDC, /* STS mode 1 with SDC*/
        .stsLength = DWT_STS_LEN_256,        /* STS length see allowed values in Enum dwt_sts_lengths_e */
 
        .pdoaMode = DWT_PDOA_M0              /* PDOA mode 3 */

    };
    
    static dwt_config_t config2 = {
        .chan = 5,                           /* Channel number. */
        .txPreambLength = DWT_PLEN_256,      /* Preamble length. Used in TX only. */
        .rxPAC = DWT_PAC16,                  /* Preamble acquisition chunk size. Used in RX only. */
        .txCode = 9,                         /* TX preamble code. Used in TX only. */
        .rxCode = 9,                         /* RX preamble code. Used in RX only. */
        .sfdType = 1,                        /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
        .dataRate = DWT_BR_850K,             /* Data rate. */
        .phrMode = DWT_PHRMODE_STD,          /* PHY header mode. */
        .phrRate = DWT_PHRRATE_STD,          /* PHY header rate. */
        .sfdTO = (257 + 16 - 16),            /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        .stsMode = DWT_STS_MODE_1 | DWT_STS_MODE_SDC, /* STS mode 1 with SDC*/
        .stsLength = DWT_STS_LEN_256,        /* STS length see allowed values in Enum dwt_sts_lengths_e */
 
        .pdoaMode = DWT_PDOA_M0              /* PDOA mode 3 */

    };

    static dwt_txconfig_t txconfig_options ={
        .PGdly = 0x34,            /* PG delay */
        .power = TX_POWER,        /* TX power */
        .PGcount = 0x0            /* PG count */
    };    

    port_set_dw_ic_spi_fastrate();
    reset_DWIC(); 
    Sleep(2);     //延时等待复位

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    if(dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        led_on(LED_ALL);//DW3000初始化错误
#if defined (ULM1) || defined (ULM3) || defined (ULM3_PA)
        if(USE_OLED == 1)
        {
            lcd_display(1, 2, "  ERROR  ");
            lcd_display(1, 3, "INIT FAIL");
        }
#endif
        while (1);
    }

    /* 1号：预留
     * 2号：预留
     * 3号：外部耗电开关，用于加大设备功耗，避免充电宝自动关闭，on=开启，off=关闭（默认on）
     * 4号：角色控制，on=基站，off=标签
     * 5，6，7号：设备ID，000=0，001=1，010=2 ....
     * 8号：硬件测距卡尔曼滤波开关，on=开启，off=关闭（默认on）
     */
#if defined(L051_DEV)
    read_l151_eeprom(L151_SW8_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
    if(e2prom_data_read[0] == 0xAA)
    {
        switch8 = e2prom_data_read[1];         
    }
#else
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        memset(e2prom_data_read, 0, EEP_UNIT_SIZE);
        E2prom_Read(SW8_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA)
        {
            switch8 = e2prom_data_read[1];         
        }
        else
        {
            switch8 = switch_is_on(SW_8)   //读取拨码开关键值
                    | switch_is_on(SW_7) << 1    
                    | switch_is_on(SW_6) << 2
                    | switch_is_on(SW_5) << 3
                    | switch_is_on(SW_4) << 4
                    | switch_is_on(SW_3) << 5
                    | switch_is_on(SW_2) << 6
                    | switch_is_on(SW_1) << 7;
        }
    }
    else
    {
        switch8 = switch_is_on(SW_8)   //读取拨码开关键值
                | switch_is_on(SW_7) << 1    
                | switch_is_on(SW_6) << 2
                | switch_is_on(SW_5) << 3
                | switch_is_on(SW_4) << 4
                | switch_is_on(SW_3) << 5
                | switch_is_on(SW_2) << 6
                | switch_is_on(SW_1) << 7;
    }
#endif

    if(switch8 & SWS1_SHF_MODE)//第2拨码开关=on
    {
        memset(e2prom_data_read, 0, EEP_UNIT_SIZE);
#if defined(L051_DEV)
        read_l151_eeprom(L151_SLOT_2ON_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
#else
        E2prom_Read(SLOT_2ON_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
#endif
        if(e2prom_data_read[0] == 0xAA)
        {
            inst_slot_number = e2prom_data_read[1];         
        }
        else
        {
            inst_slot_number = MAX_TAG_NUMBER_SW2_ON;
        }
        dwt_configure(&config1);//设置射频通信参数
        inst_dataRate = config1.dataRate;
        inst_ch = config1.chan;
        inst_prf = DWT_PRF_64M;
    }
    else //第2拨码开关=off,正常出厂默认最大10标签模式，100ms更新一次数据
    {
        memset(e2prom_data_read, 0, EEP_UNIT_SIZE);
#if defined(L051_DEV)
        read_l151_eeprom(L151_SLOT_2OFF_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
#else
        E2prom_Read(SLOT_2OFF_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
#endif
        if(e2prom_data_read[0] == 0xAA)
        {
            inst_slot_number = e2prom_data_read[1];         
        }
        else
        {
            inst_slot_number = MAX_TAG_NUMBER_SW2_OFF;
        }
        dwt_configure(&config2);//设置射频通信参数
        inst_dataRate = config2.dataRate;
        inst_ch = config2.chan;
        inst_prf = DWT_PRF_64M;
    }

    if(inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_6P8M;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_6P8M;
        inst_resp_rx_timeout = RESP_RX_TIMEOUT_6P8M;
        inst_data_interval = DATA_INTERVAL_TIME_6P8M;

        inst_poll2final_time = ((FIRST_RESP_SEND_6P8M + 1 * inst_data_interval) * UUS_TO_DWT_TIME);

    }

    else if(inst_dataRate == DWT_BR_850K)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_850K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_850K;
        inst_resp_rx_timeout = RESP_RX_TIMEOUT_850K;
        inst_data_interval = DATA_INTERVAL_TIME_850K;

        inst_poll2final_time = ((FIRST_RESP_SEND_850K + MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);

    }

    memset(e2prom_data_read, 0, EEP_UNIT_SIZE);
#if defined(L051_DEV)
    read_l151_eeprom(L151_TX_PWR_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
    if(e2prom_data_read[0] == 0xAA) //固定AA
    {
        txconfig_options.power  = (uint32)e2prom_data_read[1] << 24;
        txconfig_options.power += (uint32)e2prom_data_read[2] << 16;
        txconfig_options.power += (uint32)e2prom_data_read[3] << 8;
        txconfig_options.power += (uint32)e2prom_data_read[4];
    }
    else  //未配置，写入默认值
    {
        txconfig_options.power = TX_POWER;
        memset(e2prom_data_write, 0, EEP_UNIT_SIZE);
        e2prom_data_write[0] = 0xAA;
        e2prom_data_write[1] = txconfig_options.power >> 24;
        e2prom_data_write[2] = txconfig_options.power >> 16;
        e2prom_data_write[3] = txconfig_options.power >> 8;
        e2prom_data_write[4] = txconfig_options.power;
        write_l151_eeprom(L151_TX_PWR_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
    }             
#else
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        E2prom_Read(TX_PWR_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA) //固定AA
        {
            txconfig_options.power  = (uint32)e2prom_data_read[1] << 24;
            txconfig_options.power += (uint32)e2prom_data_read[2] << 16;
            txconfig_options.power += (uint32)e2prom_data_read[3] << 8;
            txconfig_options.power += (uint32)e2prom_data_read[4];
        }
        else    //未配置，写入默认值
        {
            txconfig_options.power = TX_POWER;
            memset(e2prom_data_write, 0, EEP_UNIT_SIZE);
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = txconfig_options.power >> 24;
            e2prom_data_write[2] = txconfig_options.power >> 16;
            e2prom_data_write[3] = txconfig_options.power >> 8;
            e2prom_data_write[4] = txconfig_options.power;
            E2prom_Write(TX_PWR_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
        }
    }
    else    //未板载EEPROM，按define值设置
    {
        txconfig_options.power = TX_POWER;
    }
#endif
    tx_power = txconfig_options.power;
    dwt_configuretxrf(&txconfig_options);//设置发射功率和pg值

    memset(e2prom_data_read, 0, EEP_UNIT_SIZE);
#if defined(L051_DEV)
    read_l151_eeprom(L151_ANT_DLY_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
    if(e2prom_data_read[0] == 0xAA) //固定AA
    {
        ant_dly = (uint16)e2prom_data_read[1] << 8 | (uint16)e2prom_data_read[2];
    }
    else    //未配置，使用默认值
    {
        ant_dly = ANT_DLY;
        // memset(e2prom_data_write, 0, EEP_UNIT_SIZE);
        // e2prom_data_write[0] = 0xAA;
        // e2prom_data_write[1] = ant_dly >> 8;
        // e2prom_data_write[2] = (uint8)ant_dly;
        // write_l151_eeprom(L151_ANT_DLY_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
    }           
#else    
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        E2prom_Read(ANT_DLY_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA) //固定AA
        {
            ant_dly = (uint16)e2prom_data_read[1] << 8 | (uint16)e2prom_data_read[2];
        }
        else    //未配置，使用默认值
        {
            ant_dly = ANT_DLY;
            // memset(e2prom_data_write, 0, EEP_UNIT_SIZE);
            // e2prom_data_write[0] = 0xAA;
            // e2prom_data_write[1] = ant_dly >> 8;
            // e2prom_data_write[2] = (uint8)ant_dly;
            // E2prom_Write(ANT_DLY_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
        }
    }
    else    //未板载EEPROM，按define值设置
    {
        ant_dly = ANT_DLY;
    }
#endif

    dwt_setrxantennadelay(ant_dly);//设置天线延时
    dwt_settxantennadelay(ant_dly);

    dwt_setpanid(PAN_ID);//设置PAN ID 组号
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN); //设置帧过滤模式开启

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);//设置外置PA和LNA控制开启(DWM3000模块无实际作用)
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);//设置DW3000控制的收发指示灯开启，低功耗时可注释掉

    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);//设置射频诊断调试开启，用于读取接收功率

#if defined(L051_DEV)
           
#else   
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        memset(e2prom_data_read, 0, EEP_UNIT_SIZE);
        E2prom_Read(DIS_OFFSET_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA) //固定AA
        {
            distance_offset_cm  = (uint32)e2prom_data_read[1] << 24;
            distance_offset_cm += (uint32)e2prom_data_read[2] << 16;
            distance_offset_cm += (uint32)e2prom_data_read[3] << 8;
            distance_offset_cm += (uint32)e2prom_data_read[4];
        }
        else    //未配置，写入默认值
        {
            distance_offset_cm = 0;
            memset(e2prom_data_write, 0, EEP_UNIT_SIZE);
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = distance_offset_cm >> 24;
            e2prom_data_write[2] = distance_offset_cm >> 16;
            e2prom_data_write[3] = distance_offset_cm >> 8;
            e2prom_data_write[4] = distance_offset_cm;
            E2prom_Write(DIS_OFFSET_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
        }
    }
    else    //未板载EEPROM，按默认值设置
    {
        distance_offset_cm = 0;
    }
#endif

    if(switch8 & SWS1_ROLE_MODE)//第4拨码开关on=anchor
    {
        instance_mode = ANCHOR; //当前角色控制为基站
    }
    else
    {
        instance_mode = TAG; //当前角色控制为标签
    }
#if defined (ULM1) || defined (ULM3) || defined (ULM3_PA)
    if(USE_OLED == 1)//显示版本号，系统最大基站个数，最大标签容量，如“4A10T”表示最大4基站10标签
    {
        if(inst_slot_number < 10)
        {
            sprintf(lcd_data, "%s  %dA%dT", SOFTWARE_VER, MAX_AHCHOR_NUMBER, inst_slot_number);
        }
        else
        {         
            sprintf(lcd_data, "%s %dA%dT", SOFTWARE_VER, MAX_AHCHOR_NUMBER, inst_slot_number);
        }
        lcd_display(1, 1, lcd_data);
        
        if(inst_dataRate == DWT_BR_110K)
            lcd_display(1, 3, "110K");
        else if(inst_dataRate == DWT_BR_6M8)
            lcd_display(1, 3, "6.8M");
        else if(inst_dataRate == DWT_BR_850K)
            lcd_display(1, 3, "850K");

        sprintf(lcd_data,"CH%d", inst_ch);
        lcd_display(7, 3, lcd_data);
        if(instance_mode == ANCHOR)
        {
            sprintf(lcd_data, "%dms", inst_slot_number * inst_one_slot_time);
            lcd_display(1, 2, lcd_data);
        }

        if(switch8 & SWS1_KAM_MODE)//第8拨码开关=on,开启卡尔曼滤波
        {
            lcd_display(9, 4, "K");//屏幕右下脚显示K，表示开启
        }

        if(switch8 & SWS1_HPR_MODE)//第3拨码开关on=打开外部耗电开关
        {
            expr_on();
            lcd_display(8, 4, "P");//屏幕右下脚显示K，表示开启
        }

    }
#endif
    
    //设置设备ID
#if defined(L051_DEV)
    read_l151_eeprom(L151_DEV_ID_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
    if(e2prom_data_read[0] == 0xAA)
    {
        dev_id = e2prom_data_read[1];
    }
    else
    {
        dev_id = 0;
    }
#else
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        memset(e2prom_data_read, 0, EEP_UNIT_SIZE);
        E2prom_Read(DEV_ID_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA) //固定AA
        {
            dev_id = e2prom_data_read[1];
        }
        else
        {
            //读取拨码开关设备ID
            dev_id = ((switch8 & SWS1_A1A_MODE) + (switch8 & SWS1_A2A_MODE) + (switch8 & SWS1_A3A_MODE)) >> 1;
        }
    }
    else
    {
        //读取拨码开关设备ID
        dev_id = ((switch8 & SWS1_A1A_MODE) + (switch8 & SWS1_A2A_MODE) + (switch8 & SWS1_A3A_MODE)) >> 1;
    }
#endif

    //设置组ID
#if defined(L051_DEV)

#else
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        memset(e2prom_data_read, 0, EEP_UNIT_SIZE);
        E2prom_Read(GROP_ID_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA) //固定AA
        {
            group_id = e2prom_data_read[1];
        }
        else
        {
            group_id = 0;
        }
    }
    else
    {
        group_id = 0;
    }
#endif



    //设置中断标志
    dwt_setinterrupt( SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK  | SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | 
                      SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | 
                      SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT);

    //开启中断
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);
    //使能STM32外部中断
    port_set_dwic_isr(dwt_isr);

    set_instance();
}
#endif



void dw_main(void)
{
    uint8_t UART_TX_DATA[512], UART_TX_DEBUG[128];//串口数据输出
    int len,len2=0;
    static uint16_t fn = 0;         //串口数据流水号，每帧数据+1
    led_off(LED_ALL);

#if defined(ULM3) || defined (ULM3_PA)
    DW3000_init();
#else
    DW1000_init();
#endif


    if(instance_mode == TAG)
    {
        read_anc_coord();//读取配置的基站坐标，用于标签解算输出自身坐标
    }
    //打印系统参数信息
    print_config();

    MX_IWDG_Init();

    while(1)//测距功能实现，按角色执行基站状态机或标签状态机
    {
        HAL_IWDG_Refresh(&hiwdg); //喂狗
        if(instance_mode == ANCHOR)
        {
            anchor_app();
        }
        else
        {
            tag_app();
        }
        
        if(range_status == RANGE_TWR_OK) //TWR测距有效，进行数据滤波和打包输出、屏显
        {
            range_status = RANGE_NULL;//清空标志位
/*********************************************************卡尔曼滤波计算******************************************************************************/          
#if defined (ANCRANGE) 					
            if((switch8 & SWS1_KAM_MODE) && (ancrange_flag == 0))//第8拨码开关=on,开启卡尔曼滤波,自标定时候不开启卡尔曼滤波
#else
            if(switch8 & SWS1_KAM_MODE)//第8拨码开关=on,开启卡尔曼滤波
#endif
            {

                for(int i=0; i<MAX_AHCHOR_NUMBER; i++)
                {
                    if(distance_report[i] > 0) //数据有效，进行卡尔曼滤波计算
                    {
                        distance_report[i] = kalman_filter(distance_report[i], i, (instance_mode == TAG)?tag_id:recv_tag_id);
                    }
                }
            }
/*********************************************************串口数据打包输出******************************************************************************/            
            len = 0;
            len2 = 0;

            /************标签十进制打包*************************************************************/
            char distance_report_char[8][20];
            //if((instance_mode == TAG) && ((inst_slot_number > 1) || (inst_dataRate != DWT_BR_6M8)))
            {
                for(int i = 0; i < 8; i++)
                {
                    if(distance_report[i] >= 0)
                    {
                        sprintf(distance_report_char[i], "%.2f", (float)distance_report[i]/1000.0);
                    }
                }
            }

            {
#if defined (ANCRANGE)
                if((ancrange_flag > 0) && (instance_mode == ANCHOR))   //基站测距模式
                {
                    if((recv_tag_id == 0) && (range_nb > 1))  //只输出T0数据（基站切换为T0），其他标签数据暂不输出,range_nb=0为第一个数据不准确
                    {
                        len = sprintf((char*)&UART_TX_DATA[0], "ma %02x %08x %08x %08x %08x %08x %08x %08x %08x %04x %02x %08x %c%d:%d %04x\r\n",
                                    sos, 
                                    distance_report[0], 
                                    distance_report[1], 
                                    distance_report[2], 
                                    distance_report[3], 
                                    distance_report[4], 
                                    distance_report[5], 
                                    distance_report[6], 
                                    distance_report[7], 
                                    fn++, range_nb, range_time, 'a', 0, target_ancid, 0);
                    }
                }
                else if(battery != 0xff)
#endif
                {
#if (MAX_AHCHOR_NUMBER == 4)
                    {
                        len = sprintf((char*)&UART_TX_DATA[0], "mc %02x %08x %08x %08x %08x %04x %02x %08x %c%d:%d %04x\r\n",
                                    sos, 
                                    distance_report[0], distance_report[1], distance_report[2], distance_report[3], 
                                    fn++, range_nb, range_time, (instance_mode == TAG)?'t':'a', (instance_mode == TAG)?tag_id:recv_tag_id, (instance_mode == TAG)?0:anc_id, (int)(-(rx_power*100)));
                    }
#elif (MAX_AHCHOR_NUMBER == 8)

                    {
                        len = sprintf((char*)&UART_TX_DATA[0], "mc %02x %08x %08x %08x %08x %08x %08x %08x %08x %04x %02x %08x %c%d:%d %04x\r\n",
                                    sos, 
                                    distance_report[0], distance_report[1], distance_report[2], distance_report[3], 
                                    distance_report[4], distance_report[5], distance_report[6], distance_report[7], 
                                    fn++, range_nb, range_time, (instance_mode == TAG)?'t':'a', (instance_mode == TAG)?tag_id:recv_tag_id, (instance_mode == TAG)?0:anc_id, (int)(-(rx_power*100)));
                    }
#endif
                }
            }

#if defined(ANCRANGE)
            if(ancrange_flag == 0)   //非基站测距模式
#endif
            {
                /************标签自己算坐标*************************************************************/            
                char Location_char[30]="LO=[not calculated]";
                if((instance_mode == TAG) && ((inst_slot_number > 5) || (inst_dataRate != DWT_BR_6M8)))
                {
                    int result = 0;
                    vec3d report;
                    result = GetLocation(&report, &anchorArray[0], &distance_report[0]);
                    
                    if(result > 0)
                    {
                        sprintf(Location_char,"LO=[%.2f,%.2f,%.2f]",report.x,report.y,report.z);
                    }
                    else
                    {
                        sprintf(Location_char,"LO=[no solution]");
                    }
                }
                if((instance_mode == TAG) && !(switch8 & SWS1_IMU_MODE) && !(switch8 & SWS1_HPR_MODE) && ((inst_slot_number > 1) || (inst_dataRate != DWT_BR_6M8)))
                {
                    len2 = sprintf((char*)&UART_TX_DEBUG[0], "$%sT%d,%s,%s,%s,%s,%s\r\n", 
                                        (switch8 & SWS1_KAM_MODE) ? "K":"NK", tag_id, 
                                        (distance_report[0] >= 0)?distance_report_char[0]:"null", 
                                        (distance_report[1] >= 0)?distance_report_char[1]:"null", 
                                        (distance_report[2] >= 0)?distance_report_char[2]:"null", 
                                        (distance_report[3] >= 0)?distance_report_char[3]:"null", 
                                        Location_char);     
                    
                    strcat((char*)&UART_TX_DATA[0], (char*)&UART_TX_DEBUG[0]);
                    len = len + len2;           
                }
            }

/************标签屏显测距距离数据***********************************************************/            
#if defined (ULM1) || defined (ULM3) || defined (ULM3_PA)
            if((USE_OLED == 1) && (instance_mode == TAG) && (((inst_slot_number > 5) && (inst_dataRate == DWT_BR_6M8)) || ((inst_slot_number > 1) && (inst_dataRate == DWT_BR_110K))))
            {
                static uint64_t printLCDTWRReports = 0;
                static int toggle = 0;
                if(printLCDTWRReports + 2000 <= portGetTickCnt()) //每2S更新一次数据
                {
                    printLCDTWRReports = portGetTickCnt();
                    sprintf((char*)&lcd_data[0], "A%d:%3.2fm ", toggle, (float)distance_report[toggle]/1000.0);
                    lcd_display(1,2, lcd_data);                    
                    toggle++;
                    if(toggle >= MAX_AHCHOR_NUMBER)
                        toggle = 0;
                }
            }
#endif
            for(uint8_t i = 0; i < 8; i++)  //清空distance_report数组，设置无效值
            {
               distance_report[i] = -1;  
               group_report[i] = -1;
            }
        }
        else if(range_status == RANGE_ERROR) 
        {
            range_status = RANGE_NULL;//清空标志位
            len = sprintf((char*)&UART_TX_DATA[0], "$RANGE_ERROR,ID=%d,rb=%d,rangetime=%d\r\n", dev_id, range_nb, range_time);
            for(uint8_t i = 0; i < 8; i++)  //清空distance_report数组，设置无效值
            {
               distance_report[i] = -1; 
               group_report[i] = -1; 
            }
        }
        
        if(len > 0)
        {
            int i;
            for(i = 0; i < 10; i++)  //校验是否接收用户字节
            {
                if(user_data[i] > 0)
                    break;
            }
            if(i < 10)  //有非0的用户字节
            {
                printf("T%d = ", recv_tag_id);
                for(i = 0; i < 9; i++)  //接收用户字节
                {
                    printf("%02X,", user_data[i]);
                }
                printf("%02X\r\n", user_data[9]);
            }
            //HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);//串口发送接收到的数据
            HAL_UART_Transmit_DMA(&UART_PORT, &UART_TX_DATA[0], len);//串口发送接收到的数据
            len = 0;
        }

        if(uart_rx_len > 0)//收到串口数据
        {
            if((UART_RX_BUF[0] == '$') && (UART_RX_BUF[uart_rx_len - 1] == '\n'))
            {
                parse_uart(&UART_RX_BUF[0]);//数据解析和指令执行
            }
            uart_rx_len = 0;
            memset(UART_RX_BUF, 0, sizeof(UART_RX_BUF));
        }
    }
}

void print_config(void)
{
    int len;
    uint8_t UART_TX_DATA[512];
    len = sprintf((char*)&UART_TX_DATA[0], "\r\n**********************DEVICE CONFIG**********************\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);


#if defined (ULM3)
    len = sprintf((char*)&UART_TX_DATA[0], "* model = ULM3\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
#elif defined (ULM3_PA)
    len = sprintf((char*)&UART_TX_DATA[0], "* model = U3-PA\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
#endif

    len = sprintf((char*)&UART_TX_DATA[0], "* firmware = %s\r\n* role = %s\r\n* addr = %d\r\n* blink = 0\r\n", SOFTWARE_VER, (instance_mode == TAG)?"TAG":"ANCHOR", dev_id);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* max_anc_num = %d\r\n* max_tag_num = %d\r\n", MAX_AHCHOR_NUMBER, inst_slot_number);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

    if(inst_dataRate == DWT_BR_110K)
        len = sprintf((char*)&UART_TX_DATA[0], "* uwb_data_rate = 110K\r\n");
    else if(inst_dataRate == DWT_BR_6M8)
        len = sprintf((char*)&UART_TX_DATA[0], "* uwb_data_rate = 6.8M\r\n");    
    else if(inst_dataRate == DWT_BR_850K)
        len = sprintf((char*)&UART_TX_DATA[0], "* uwb_data_rate = 850K\r\n");    
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* channel = CH%d\r\n* update_frequency = %dHz\r\n* update_Period = %dms\r\n* kalmanfilter = %d\r\n", inst_ch, 1000 / (inst_slot_number * inst_one_slot_time), inst_slot_number * inst_one_slot_time, (switch8 & SWS1_KAM_MODE)? 1:0);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* ant_dly = %d\r\n* tx_power = %08lx\r\n", ant_dly, tx_power);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
//#if defined (ULM1) || defined (ULM3)
    len = sprintf((char*)&UART_TX_DATA[0], "* use_ext_eeprom = %d\r\n* use_imu = %d\r\n* group_id = %d\r\n", USE_EXT_EEPROM, USE_IMU, group_id&0x7f);
//#else
    //len = sprintf((char*)&UART_TX_DATA[0], "* use_ext_eeprom = %d\r\n* use_imu = %d\r\n", USE_EXT_EEPROM, USE_IMU);
//#endif
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
    if(instance_mode == TAG)
    {
        len = sprintf((char*)&UART_TX_DATA[0], "* A0(%.2f, %.2f, %.2f) A1(%.2f, %.2f, %.2f)\r\n* A2(%.2f, %.2f, %.2f) A3(%.2f, %.2f, %.2f)\r\n", 
                                                    anchorArray[0].x, anchorArray[0].y, anchorArray[0].z, 
                                                    anchorArray[1].x, anchorArray[1].y, anchorArray[1].z,
                                                    anchorArray[2].x, anchorArray[2].y, anchorArray[2].z,
                                                    anchorArray[3].x, anchorArray[3].y, anchorArray[3].z);
        HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
        if(MAX_AHCHOR_NUMBER == 8)
        {
            len = sprintf((char*)&UART_TX_DATA[0], "* A4(%.2f, %.2f, %.2f) A5(%.2f, %.2f, %.2f)\r\n* A6(%.2f, %.2f, %.2f) A7(%.2f, %.2f, %.2f)\r\n", 
                                                        anchorArray[4].x, anchorArray[4].y, anchorArray[4].z, 
                                                        anchorArray[5].x, anchorArray[5].y, anchorArray[5].z,
                                                        anchorArray[6].x, anchorArray[6].y, anchorArray[6].z,
                                                        anchorArray[7].x, anchorArray[7].y, anchorArray[7].z);
            HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
        }
    }

    len = sprintf((char*)&UART_TX_DATA[0], "***************************END***************************\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

}


void read_anc_coord(void)
{
    char anc_coord_read[COORD_LENGTH];
    char coord_cut_data[MAX_AHCHOR_NUMBER * 3 + 1][10];
#if defined(L051_DEV)
           
#else
    E2prom_Read(ANC_COORD_ADDR, (uint8_t*)anc_coord_read, COORD_LENGTH);
#endif
    if(anc_coord_read[0] == '$')
    {
        char *ptr, *retptr;
        ptr = anc_coord_read;
        uint8_t i = 0;

        while((retptr=strtok(ptr,",")) != NULL)
        {
            strcpy(coord_cut_data[i], retptr);
            ptr = NULL;
            i++;
        }

        anchorArray[0].x = atof(coord_cut_data[1]);
        anchorArray[0].y = atof(coord_cut_data[2]);
        anchorArray[0].z = atof(coord_cut_data[3]);

        anchorArray[1].x = atof(coord_cut_data[4]);
        anchorArray[1].y = atof(coord_cut_data[5]);
        anchorArray[1].z = atof(coord_cut_data[6]);

        anchorArray[2].x = atof(coord_cut_data[7]);
        anchorArray[2].y = atof(coord_cut_data[8]);
        anchorArray[2].z = atof(coord_cut_data[9]);

        anchorArray[3].x = atof(coord_cut_data[10]);
        anchorArray[3].y = atof(coord_cut_data[11]);
        anchorArray[3].z = atof(coord_cut_data[12]);
        
        if(MAX_AHCHOR_NUMBER == 8)
        {
            anchorArray[4].x = atof(coord_cut_data[13]);
            anchorArray[4].y = atof(coord_cut_data[14]);
            anchorArray[4].z = atof(coord_cut_data[15]);

            anchorArray[5].x = atof(coord_cut_data[16]);
            anchorArray[5].y = atof(coord_cut_data[17]);
            anchorArray[5].z = atof(coord_cut_data[18]);

            anchorArray[6].x = atof(coord_cut_data[19]);
            anchorArray[6].y = atof(coord_cut_data[20]);
            anchorArray[6].z = atof(coord_cut_data[21]);

            anchorArray[7].x = atof(coord_cut_data[22]);
            anchorArray[7].y = atof(coord_cut_data[23]);
            anchorArray[7].z = atof(coord_cut_data[24]);
        }
    }
}


/*
    串口指令集，注意发送指令以$开头，以\r\n结尾
    $rboot              重启
    $rantdly            查询天线延时参数
    $reset              恢复默认参数
    $santdly,16375      设置天线延时参数（10进制）
    $stxpwr,1f1f1f1f    设置发射增益参数（16进制）
    $sanccd,0,0,2,0,3.1,2,3.1,0,2,3.1,3.1,2  设置基站坐标A0.X,A0.Y,A0.Z,A1.X,A1,Y,A1,Z,A2.X,A2,Y,A2,Z,A3.X,A3,Y,A3,Z
    $cali,-30,-169      校准距离和PDOA，第一个参数为距离，单位cm，第二个参数为PDOA单位角度
    $saddr,1            设置标签ID
    $sgrop,1            设置组ID
    $ssw,00010001       设置拨码开关值，设置后实体拨码开关将不起作用，16进制 0001 0001
    $s2onslot,1         设置2号拨码on时的最大标签容量
    $s2offslot,1        设置2号拨码off时的最大标签容量
    $synctime,8640000   设置系统同步时间
    $ancrangestart      基站间测距开始
    $ancrangestop       基站间测距停止
    $sdata,12,34,56,78,90,AA,BB,00,00,00   标签发送自定义数据，长度固定，不足补0
*/

void parse_uart(uint8_t* data)
{
    uint8_t UART_COMMAND_BUF[30];
    uint8_t len = 0;
    uint8_t e2prom_data_write[EEP_UNIT_SIZE] = {0};


    if((strcmp((char*)data, "$sdata\r\n") == 0) || (strlen((char*)data) == 38))  //标签发送自定义数据
    {
        char *ptr, *retptr;
        ptr = (char*)data;
        retptr = strtok(ptr, ",");//解析数据头
        if((strcmp(retptr, "$sdata") == 0) && (instance_mode == TAG))
        {
            for(uint8_t i = 0; i < 10; i++)
            {
                ptr = NULL;
                retptr = strtok(ptr, ",");
                sscanf(retptr, "%x", &user_data[i]);
                //printf("user_data[%d] = %x\r\n", i, user_data[i]);
            }
        }
    }


    else if(strchr((char*)data, ',') > 0)//带参数指令 如 $santdly,11223
    {
        char *ptr, *retptr;
        ptr = (char*)data;
        retptr = strtok(ptr, ",");//解析数据头

        if(strcmp(retptr, "$synctime") == 0)    //$synctime,8640000 设置系统同步时间
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            sys_time_diff = HAL_GetTick() - atoi(retptr);
        }



        else if(strcmp(retptr, "$santdly") == 0)//设置天线延时参数  $santdly,16375
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            ant_dly = atoi(retptr);
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = ant_dly >> 8;
            e2prom_data_write[2] = (uint8)ant_dly;
#if defined(L051_DEV)
            write_l151_eeprom(L151_ANT_DLY_ADDR, e2prom_data_write, EEP_UNIT_SIZE); 
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(ANT_DLY_ADDR, e2prom_data_write, EEP_UNIT_SIZE); 
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$stxpwr") == 0)//设置发射增益参数  $stxpwr,1f1f1f1f 
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            sscanf(retptr, "%08lx", &tx_power);
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = tx_power >> 24;
            e2prom_data_write[2] = tx_power >> 16;
            e2prom_data_write[3] = tx_power >> 8;
            e2prom_data_write[4] = tx_power;
#if defined(L051_DEV)
            write_l151_eeprom(L151_TX_PWR_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {

                E2prom_Write(TX_PWR_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            HAL_NVIC_SystemReset();//重启 
        }
        else if(strcmp(retptr, "$sanccd") == 0) //给标签设置基站坐标，用于标签自己三边定位输出定位结果
        {
            retptr[7] = ',';
            uint8_t zero[COORD_LENGTH] = {0};
#if defined(L051_DEV)
           
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(ANC_COORD_ADDR, (uint8_t*)zero, COORD_LENGTH);
                HAL_Delay(10);
                E2prom_Write(ANC_COORD_ADDR, (uint8_t*)retptr, strlen((char*)retptr));
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$saddr") == 0) //设置标签ID
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = atoi(retptr);
#if defined(L051_DEV)
            write_l151_eeprom(L151_DEV_ID_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(DEV_ID_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$sgrop") == 0) //设置组ID
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = atoi(retptr);
#if defined(L051_DEV)

#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(GROP_ID_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$ssw") == 0) //设置拨码开关值
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            int sw8 = strtol(retptr, NULL, 2);
            e2prom_data_write[1] = sw8;
#if defined(L051_DEV)
            write_l151_eeprom(L151_SW8_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(SW8_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            //len = sprintf((char*)UART_COMMAND_BUF, "e2prom_data_write=0x%x\r\n", e2prom_data_write[1]);
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$s2onslot") == 0) //设置2号拨码on最大标签容量
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = atoi(retptr);
#if defined(L051_DEV)
            write_l151_eeprom(L151_SLOT_2ON_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(SLOT_2ON_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            //len = sprintf((char*)UART_COMMAND_BUF, "e2prom_data_write=0x%x\r\n", e2prom_data_write[1]);
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$s2offslot") == 0) //设置2号拨码off最大标签容量
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = atoi(retptr);
#if defined(L051_DEV)
            write_l151_eeprom(L151_SLOT_2OFF_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(SLOT_2OFF_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            //len = sprintf((char*)UART_COMMAND_BUF, "e2prom_data_write=0x%x\r\n", e2prom_data_write[1]);
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }

    }
    else //无参数指令 如$rboot
    {
        if(strcmp((char*)data, "$rboot\r\n") == 0)//重启
        {
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            HAL_NVIC_SystemReset();//重启 
        }
        else if(strcmp((char*)data, "$rantdly\r\n") == 0)//查询天线延时参数
        {
            len = sprintf((char*)UART_COMMAND_BUF, "ant_dly = %d\r\n", ant_dly);
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
        }
        else if(strcmp((char*)data, "$reset\r\n") == 0)//恢复默认参数
        {
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            uint8_t write_zero[256]={0};
#if defined(L051_DEV)
            write_l151_eeprom(L151_START_ADDR, write_zero, 256);
#else
            E2prom_Write(0, write_zero, 256);
#endif
            HAL_NVIC_SystemReset();//重启 
        }
#if defined(ANCRANGE)
        else if(strcmp((char*)data, "$ancrangestart\r\n") == 0)//基站测距自标定流程
        {
            if((instance_mode == ANCHOR) && (dev_id == 0) && (ancrange_flag == 0))
            {
                ancrange_flag = 1; //设置基站间测距模式开启 1=开始 2=运行中 0=停止
                target_ancid = 1;//从A1基站开始通知
                printf("$setok\r\n");
            }
        }
        else if(strcmp((char*)data, "$ancrangestop\r\n") == 0)//基站测距自标定流程
        {
            if((instance_mode == ANCHOR) && (dev_id == 0) && (ancrange_flag > 0))
            {
                ancrange_flag = 0;          //设置基站间测距模式开启 1=开始 2=运行中 0=停止
                printf("$setok\r\n");
            }      
        }
#endif
    }
}

void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
    //HAL_UART_Transmit(&UART_PORT, &UART_RX_BUF[0], strlen((char*)UART_RX_BUF), 1000);
    uart_rx_len = strlen((char*)UART_RX_BUF);
}

