#include "instance.h"

#ifndef L051_DEV

#define FB       (499.2e6f)                    /* Basis frequency */
#define L_M_5    (SPEED_OF_LIGHT /FB /13.0f)   /* Lambda, m, CH5 */
#define D_M_5    (0.0204f)       /* Distance between centers of antennas, ~(L_M/2), m, CH5 */

extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);

/* RESP数据帧格式 */
static uint8_t tx_resp_msg[RESP_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x80, FUNC_CODE_RESP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#if defined (ANCRANGE)
static uint8_t tx_sync_msg[SYNC_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x80, 0x00, 0x80, FUNC_CODE_SYNC};
#endif
/* 接收数据buffer */
static uint8_t rx_buffer[FRAME_LEN_MAX_EX];

/* TWR时间戳，用于计算飞行时间 */
static uint64_t poll_rx_ts;    
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* 发送和接收数据中断标志 */
static volatile uint8_t rx_status = RX_WAIT;
static volatile uint8_t tx_status = TX_WAIT;

/* 保存当前ID标签的测距值，下次发送resp时发给标签 */
typedef struct {
	uint8_t range_nb;
	int32_t distance;
} prev_range_t;
static prev_range_t prev_range[MAX_TAG_LIST_SIZE];
static uint8_t resp_valid = 0x00;                    //基站数据有效标志 

static uint8_t sr, rr; //用于控制当前基站处于resp时是发送还是接收
#if defined(ULM3) || defined (ULM3_PA)
static int16_t stsqual;//STS数据接收质量
#endif


#define MAX_AHCHOR_NUMBER_ 80

// 全局变量记录接收次数
uint32_t receive_count = 0;
// 全局变量记录异常数据个数
uint32_t abnormal_data_count = 0;
// 全局变量记录正确帧率和异常帧率相关计数
uint32_t correct_frame_count = 0;
uint32_t abnormal_frame_count = 0;

// 检查接收数据一致性的函数
uint32_t checkDataConsistency(uint8_t* rx_buffer) {
    uint32_t inconsistent_count = 0;
    uint8_t seq_nb = rx_buffer[SEQ_NB_IDX];
    uint8_t range_nb = rx_buffer[RANGE_NB_IDX];

    // 检查seq_nb和range_nb与接收次数的一致性

    // if (seq_nb!= (receive_count % 256) || range_nb!= ((receive_count / 256) % 256)) {
    //     inconsistent_count++;        
    //     printf("Inconsistent data:[%d][%d][%d]\n",receive_count,seq_nb, range_nb);
    // }

    uint32_t user_data[MAX_AHCHOR_NUMBER_];
    for (int i = 0; i < MAX_AHCHOR_NUMBER_-1; i++) {
        user_data[i] = *((uint32_t*)(rx_buffer + FINAL_MSG_RESP1_RX_TS_IDX + 4 * i));
        // 检查user_data每个元素是否符合预期格式（预期是0x56abcdef + i）
        uint32_t expected_data = 0x56abcdef + i;
        if (user_data[i]!= expected_data) {
            inconsistent_count++;
            abnormal_data_count++;
            // 输出异常数据（可根据实际需求调整输出格式等）
            printf("Abnormal data: 0x%X (Expected: 0x%X)\n", user_data[i], expected_data);
        }
    }

    if (inconsistent_count == 0) {
        correct_frame_count++;
    } else {
        abnormal_frame_count++;
    }

    receive_count++;
    return inconsistent_count;
}
#define MAX_TIMEOUT 263095    // 最大超时次数
// 模拟间隔1秒打印信息的函数（通过简单循环计数模拟时间，实际应用可结合定时器等精确控制时间）
void printInfoPeriodically() {
    static uint32_t loop_count = 0;
    if (loop_count++ >= MAX_TIMEOUT) {
            
            double correct_frame_rate = (double)correct_frame_count / (correct_frame_count + abnormal_frame_count);
            double abnormal_frame_rate = (double)abnormal_frame_count / (correct_frame_count + abnormal_frame_count);
            printf("Correct frame rate: %.2f", correct_frame_rate);
            printf("Abnormal frame rate: %.2f", abnormal_frame_rate);
            printf("Number of abnormal data: %u", abnormal_data_count);
            printf("receive_count : %ld\n", receive_count);

            // 重置计数，准备下一轮统计
            correct_frame_count = 0;
            abnormal_frame_count = 0;
            abnormal_data_count = 0;
            loop_count = 0;
        
    }
}

void anchor_app(void)
{
    printInfoPeriodically();
    switch (state)
    {
        case STA_INIT_POLL_SYNC: //初始化接收机，接收poll消息
		{
#if defined(ULM3) || defined (ULM3_PA)
            dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN); //使能帧过滤
#else
            dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  //设置帧过滤模式开启
#endif
            dwt_setpreambledetecttimeout(0);                        //清除前导码超时，一直接收
            dwt_setrxtimeout(0);                                    //清除接收数据超时，一直接收
            int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);         //打开接收机，等待接收数据    
            if(ret == DWT_ERROR)
            {
                state = STA_INIT_POLL_SYNC;
                break;
            }           

            rx_status = RX_WAIT;                                    //清rx标志位，中断服务函数更改其状态
            state = STA_WAIT_POLL_SYNC; 
            break;
		}
        
        
        case STA_WAIT_POLL_SYNC:                                   //等待poll消息，中断回调函数状态变更
        {
            if(rx_status == RX_OK)                                  //接收到数据
            {
                rx_status = RX_WAIT;
                state = STA_RECV_TEST_RESP;
            }
            else if((rx_status == RX_TIMEOUT) || (rx_status == RX_ERROR))  //接收数据错误，重新开启接收
            {
                state = STA_INIT_POLL_SYNC;
                printf("STA_WAIT_POLL_SYNC RX_TIMEOUT or RX_ERROR\r\n");
            }
            break;
        }
        case STA_RECV_TEST_RESP://接收处理测试消息帧
            if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_FINAL_)      //判断收到的数据是POLL
            {
                // uint8_t seq_nb=rx_buffer[SEQ_NB_IDX];  
                // range_nb = rx_buffer[RANGE_NB_IDX];             //取range_nb，resp发送时发送相同的range_nb
                // recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];  //取发送标签的ID
                
                // for(int i = 0; i < MAX_AHCHOR_NUMBER_; i++)  //接收用户字节
                // {
                //     user_data[i] = *((int*)(rx_buffer+FINAL_MSG_RESP1_RX_TS_IDX + 4*i));
                // }
                checkDataConsistency(rx_buffer);

                dwt_rxenable(DWT_START_RX_IMMEDIATE);         //打开接收机，等待接收数据    
                memset(rx_buffer,0,FRAME_LEN_MAX_EX);	      
                state = STA_WAIT_POLL_SYNC;     //等待下一个Test消息
                //state = STA_INIT_POLL_SYNC;     //重新初始化
            }
            break;
#if defined (ANCRANGE) 
        case STA_SEND_SYNC:  //A0负责发送时间戳并通知其他基站启动测距  todo 指定对方地址模式，非广播
        {
            /* sync数据打包 */
            tx_sync_msg[SEQ_NB_IDX] = frame_seq_nb++;  
            tx_sync_msg[PANID_IDX] = (uint8_t)PAN_ID; 
            tx_sync_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
            tx_sync_msg[SENDER_SHORT_ADD_IDX] = anc_id;
            tx_sync_msg[RECEIVER_SHORT_ADD_IDX] = target_ancid;
            tx_sync_msg[FUNC_CODE_IDX] = FUNC_CODE_SYNC;
            uint32_t now_time = portGetTickCnt();

            tx_sync_msg[SYNC_MSG_TIME_IDX + 0] = now_time;
            tx_sync_msg[SYNC_MSG_TIME_IDX + 1] = now_time >> 8;
            tx_sync_msg[SYNC_MSG_TIME_IDX + 2] = now_time >> 16;
            tx_sync_msg[SYNC_MSG_TIME_IDX + 3] = now_time >> 24;

            dwt_writetxdata(SYNC_MSG_LEN + FCS_LEN, tx_sync_msg, 0);    //数据写入DW3000数据缓冲区
            dwt_writetxfctrl(SYNC_MSG_LEN + FCS_LEN, 0, 1);             //配置TX帧控制寄存器
            tx_status = TX_WAIT;                                        //发送状态标志，在中断回调函数变更
            int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);              //立即发送SYNC消息
            if(ret == DWT_ERROR)
            {
                dwt_forcetrxoff();
                state = STA_INIT_POLL_SYNC;
                break;
            }             
            while(tx_status == TX_WAIT);                             //等待发送成功
            tx_status = TX_WAIT;                                     //清标志     
            state = STA_INIT_POLL_SYNC;     
            break;
        }
#endif
        case STA_RECV_POLL_SYNC://接收处理poll消息
            if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_POLL)      //判断收到的数据是POLL
            {
                range_nb = rx_buffer[RANGE_NB_IDX];             //取range_nb，resp发送时发送相同的range_nb
                recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];  //取发送标签的ID
                sos = rx_buffer[POLL_MSG_SOS_IDX];
                if(sos > 1){sos = 0;}
                battery = rx_buffer[POLL_MSG_BATTERY_IDX];
                if(recv_tag_id >= inst_slot_number)             //标签ID如果大于标签总容量则退出
                {
                    state = STA_INIT_POLL_SYNC;
                    break;
                }
                
                for(int i = 0; i < 10; i++)  //接收用户字节
                {
                    user_data[i] = rx_buffer[POLL_MSG_USER_IDX + i];
                }

                range_time = portGetTickCnt();       //取得测距时间
                poll_rx_ts = get_rx_timestamp_u64(); //获得poll_rx时间戳

                sr = MAX_AHCHOR_NUMBER;

                rr = 0x01 << anc_id;
                state = STA_SORR_RESP;
                led_on(LED2);
                led_on(LED3);
            }
#if defined (ANCRANGE)
            else if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_SYNC)      //其他基站接收到基站A0的时间同步消息
            {
                uint32_t sync_time    = (uint32_t)rx_buffer[SYNC_MSG_TIME_IDX + 3] << 24;
                sync_time = sync_time | (uint32_t)rx_buffer[SYNC_MSG_TIME_IDX + 2] << 16;
                sync_time = sync_time | (uint32_t)rx_buffer[SYNC_MSG_TIME_IDX + 1] << 8;
                sync_time = sync_time | (uint32_t)rx_buffer[SYNC_MSG_TIME_IDX + 0];

                if(inst_dataRate == DWT_BR_110K)
                {
                    sync_time = sync_time + 2;//110k发送耗时约2ms补偿
                }
                sys_time_diff = HAL_GetTick() - sync_time;  //时间同步
                range_nb = 0;
                ancrange_flag = 1; //设置基站间测距模式开启 1=开始 2=运行中 0=停止
                instance_mode = TAG;  //切换当前角色控制为标签
                temp_dev_id = dev_id; //备份当前ID
                dev_id = 0;
                ancrange_count = ANC_RANGE_COUNT;
                set_instance();
                break;
                //state = STA_INIT_POLL_SYNC;    
            }
#endif
            else //非POLL数据重新开启接收POLL
            {
                state = STA_INIT_POLL_SYNC;
            }

            break;

        case STA_SORR_RESP://根据基站ID按顺序进行发送或接收resp消息
           
            if(sr > 0)// sr>0 处于resp阶段，按基站ID判断接收resp或发送resp
            {
                if(rr & 0x01)//当前该基站需发送resp
                {
                    rr = 0;
                    state = STA_SEND_RESP;
                }
                else//当前该基站需接收resp
                {
#if defined(ULM3) || defined (ULM3_PA)
                   dwt_configureframefilter(DWT_FF_DISABLE, 0); //关闭帧过滤，能够接收所有数据
#else                    
                   dwt_enableframefilter(DWT_FF_NOTYPE_EN); //关闭帧过滤，能够接收所有数据
#endif
                    //设置resp数据接收机开启时间
                    uint64_t resp_rx_time;
                    if(inst_dataRate == DWT_BR_110K)

                        resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_110K + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

                    else if(inst_dataRate == DWT_BR_6M8)

                        resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_6P8M + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

                    else if(inst_dataRate == DWT_BR_850K)

                        resp_rx_time = (poll_rx_ts + ((FIRST_RESP_SEND_850K + (MAX_AHCHOR_NUMBER - sr) * inst_data_interval) * UUS_TO_DWT_TIME));

                    resp_rx_time = resp_rx_time >> 8;
                    dwt_setdelayedtrxtime(resp_rx_time);         //设置接收机开启延时时间
                    dwt_setrxtimeout(inst_resp_rx_timeout);      //设置接收数据超时时间
                    dwt_setpreambledetecttimeout(PRE_TIMEOUT);   //设置接收前导码超时时间
                    int ret = dwt_rxenable(DWT_START_RX_DELAYED);          //延时开启接收机
                    if(ret == DWT_ERROR)
                    {
                        state = STA_INIT_POLL_SYNC;
                        break;
                    }   
                    rr = rr >> 1;
                    state = STA_WAIT_RESP;
                }
                sr = sr - 1; 
            }
            else//准备接收final
            {      
               //final数据的接收机开启时间，提前100us开启
#if defined(ULM3) || defined (ULM3_PA)
                uint64_t final_rx_time = (poll_rx_ts + inst_poll2final_time);
#else
                uint64_t final_rx_time = (poll_rx_ts + inst_poll2final_time);              
#endif
                final_rx_time = final_rx_time >> 8;
                dwt_setdelayedtrxtime((uint32)final_rx_time);  //设置接收机开启延时时间
                dwt_setrxtimeout(inst_final_rx_timeout);       //设置接收数据超时时间
                dwt_setpreambledetecttimeout(PRE_TIMEOUT);     //设置接收前导码超时时间
                int ret = dwt_rxenable(DWT_START_RX_DELAYED);            //延时开启接收机
                if(ret == DWT_ERROR)
                {
                    state = STA_INIT_POLL_SYNC;
                    break;
                }
                state = STA_WAIT_FINAL;
            }
            break;

        case STA_SEND_RESP: //打包发送resp消息
        {
            //设置resp消息发送时间
            uint64_t resp_tx_time;
            if(inst_dataRate == DWT_BR_110K)
                resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_110K + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_110K * UUS_TO_DWT_TIME));
            else if(inst_dataRate == DWT_BR_6M8)
                resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_6P8M + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_6P8M * UUS_TO_DWT_TIME));
            else if(inst_dataRate == DWT_BR_850K)   
                resp_tx_time = (poll_rx_ts + (FIRST_RESP_SEND_850K + anc_id * inst_data_interval) * UUS_TO_DWT_TIME + (ANC_RESP_SEND_BACK_850K * UUS_TO_DWT_TIME));
            resp_tx_time = resp_tx_time >> 8;
            dwt_setdelayedtrxtime((uint32)resp_tx_time);

            /* resp数据打包 */
            tx_resp_msg[SEQ_NB_IDX] = frame_seq_nb++;
            tx_resp_msg[PANID_IDX] = (uint8_t)PAN_ID; 
            tx_resp_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
            tx_resp_msg[RANGE_NB_IDX] = range_nb;
            tx_resp_msg[SENDER_SHORT_ADD_IDX] = anc_id;
            tx_resp_msg[RECEIVER_SHORT_ADD_IDX] = recv_tag_id;
            tx_resp_msg[FUNC_CODE_IDX] = FUNC_CODE_RESP;
            //tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id;

            //将上次的测距值打包在resp中发给标签
            //if(range_nb == prev_range[recv_tag_id].range_nb + 1)
            {
                tx_resp_msg[RESP_MSG_PREV_DIS_IDX]   = prev_range[recv_tag_id].distance >> 24;
                tx_resp_msg[RESP_MSG_PREV_DIS_IDX+1] = prev_range[recv_tag_id].distance >> 16;
                tx_resp_msg[RESP_MSG_PREV_DIS_IDX+2] = prev_range[recv_tag_id].distance >> 8;
                tx_resp_msg[RESP_MSG_PREV_DIS_IDX+3] = prev_range[recv_tag_id].distance;
            }

            if(anc_id == 0)//A0负责校准标签时序，防冲突
            {
                tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id | 0x80; //参与时序校准
                int error = 0;
                int currentSlotTime = 0;
                int expectedSlotTime = 0;
                int sframePeriod_ms = inst_one_slot_time * inst_slot_number; //sframePeriod_ms 为整个TWR周期的总时间= 单slot时间*slot个数(标签总容量)
                int slotDuration_ms = inst_one_slot_time; //slotDuration_ms 为单slot时间
                int tagSleepCorrection_ms = 0;
                
                currentSlotTime = range_time % sframePeriod_ms; //currentSlotTime 当前正在通信标签的实际slot
                expectedSlotTime = recv_tag_id * slotDuration_ms;  //expectedSlotTime 当前正在通信标签应该处于的slot
                error = expectedSlotTime - currentSlotTime;  //error 计算slot差异 用于校准

                if(error < (-(sframePeriod_ms>>1))) //if error is more negative than 0.5 period, add whole period to give up to 1.5 period sleep
                {
                    tagSleepCorrection_ms = (sframePeriod_ms + error);
                }
                else //the minimum Sleep time will be 0.5 period
                {
                    tagSleepCorrection_ms = error;
                }

                tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = (tagSleepCorrection_ms >> 8) & 0xFF;//高8位存11 
                tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = tagSleepCorrection_ms & 0xFF;//低8位存12
            }
            else
            {
                tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id & 0x7f;  //不参与时序校准
                tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = 0;
                tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = 0;
            }

            dwt_writetxdata(RESP_MSG_LEN + FCS_LEN, tx_resp_msg, 0); //数据写入DW3000数据缓冲区
            dwt_writetxfctrl(RESP_MSG_LEN + FCS_LEN, 0, 1); 
            tx_status = TX_WAIT; 
            int ret = dwt_starttx(DWT_START_TX_DELAYED);  //延时发送
            if(ret == DWT_ERROR)
            {
                dwt_forcetrxoff();
                state = STA_INIT_POLL_SYNC;
                break;
            }  
            while(tx_status == TX_WAIT);//等待发送结束，状态在中断回调函数中变更
            tx_status = TX_WAIT;
            state = STA_SORR_RESP;
            break;
        }
            

        case STA_WAIT_RESP:  //等待接收resp消息，在中断回调函数内rx_status状态变更
            if(rx_status == RX_OK)
            {
                rx_status = RX_WAIT;
                state = STA_RECV_RESP;
            }
            else if((rx_status == RX_TIMEOUT)||(rx_status == RX_ERROR))
            {
                rx_status = RX_WAIT;
                state = STA_SORR_RESP;
            }   
            break;

            case STA_RECV_RESP: //接收到其他基站的resp消息
            {
                if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_RESP)//正确接收到resp消息
                {
                    if(rx_buffer[RANGE_NB_IDX] == range_nb)//和当前测距具有相同的range_nb
                    {
                        if((rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f) == (group_id & 0x7f))//只取和自己组号相同的其他基站数据上报
                        {
                            uint8_t recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX]; //取基站ID
                            distance_report[recv_anc_id]  = (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX]   << 24;
                            distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+1] << 16;
                            distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+2] << 8;
                            distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+3];

                            group_report[recv_anc_id] = rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f;  //将最高bit置0，最高bit为校准基站标志位
                        }
                    }
                }
                state = STA_SORR_RESP;
            }
                break;
        

        case STA_WAIT_FINAL:  //等待接收final消息，在中断回调函数内rx_status状态变更
            if(rx_status == RX_OK)
            {
                rx_status = RX_WAIT;
#if defined(ULM3) || defined (ULM3_PA)
                if(dwt_readstsquality(&stsqual) < 0) //if STS is not good
                {
                    state = STA_INIT_POLL_SYNC;
                    range_status = RANGE_ERROR; 
                }
                else
#endif
                {
                    state = STA_RECV_FINAL;
                }

            }
            else if((rx_status == RX_TIMEOUT) || (rx_status == RX_ERROR))
            {
                rx_status = RX_WAIT;
                state = STA_INIT_POLL_SYNC;
                range_status = RANGE_ERROR; 
            }
            break;

        case STA_RECV_FINAL:  //接收到final消息，数据处理
                //验证function code为final，与poll相同的range number 和相同的tag id 则进行TWR测距计算 
            if ((rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_FINAL) && (rx_buffer[RANGE_NB_IDX] == range_nb) && (rx_buffer[SENDER_SHORT_ADD_IDX] == recv_tag_id))  
            {
                distance_report[anc_id] = prev_range[recv_tag_id].distance;  //将上次的测距值写入distance_report用于串口输出
                group_report[anc_id] = group_id & 0x7f;
                resp_valid = rx_buffer[FINAL_MSG_FINAL_VALID_IDX];
                if(rx_buffer[FINAL_MSG_A0_GROUP_ID_IDX + anc_id * 5] != (group_id & 0x7f)) //验证标签final内的基站组号和当前组号相同
                {
                    //printf("recv = %x, me = %x\n",rx_buffer[FINAL_MSG_A0_GROUP_ID_IDX + anc_id * 5],group_id & 0x7f);
                    resp_valid = resp_valid & (uint8_t)(~(0x01 << anc_id));   //设置该基站无效
                }
                if((resp_valid >> anc_id) & 0x01) //final消息中，本基站发送的resp消息是有效的,则进行距离计算
                {
                    uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                    uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                    double Ra, Rb, Da, Db;
                    int64_t tof_dtu;
                    double tof;

                    resp_tx_ts = get_tx_timestamp_u64();   //取得resp_tx时间戳
                    final_rx_ts = get_rx_timestamp_u64();  //取得final_rx时间戳

                    /* 从final消息中，取得poll_tx时间戳，resp_rx时间戳，final_tx时间戳 */
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP1_RX_TS_IDX + anc_id * (FINAL_MSG_TS_LEN + 1)], &resp_rx_ts);
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                    /* 计算飞行时间 */
                    poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                    resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                    final_rx_ts_32 = (uint32_t)final_rx_ts;
                    Ra = (double)(resp_rx_ts - poll_tx_ts);
                    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                    Da = (double)(final_tx_ts - resp_rx_ts);
                    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                    tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                    tof = (int32)tof_dtu; 
                    if (tof > 0x7FFFFFFF) 
                    {
                        tof -= 0x80000000;  
                    }

                    tof = tof * DWT_TIME_UNITS;
                    distance_now_m = tof * SPEED_OF_LIGHT;

                    distance_now_m = distance_now_m - dwt_getrangebias(inst_ch, (float)distance_now_m, inst_prf);

                    if(distance_now_m > 20000.000)  
                    {
                        distance_now_m = -1;
                    }
                    
                    distance_now_m = distance_now_m - (float)distance_offset_cm/100.0f;  //校准（PDOA模式）
                    //更新prev_range为本次测距值
                    prev_range[recv_tag_id].distance = distance_now_m * 1000;//单位转换为mm
                    prev_range[recv_tag_id].range_nb = range_nb;

#if defined(ULM3) || defined (ULM3_PA)
                    read_rx_power(&rx_power);   //读取接收功率
#else
                    dwt_rxdiag_t rx_diag;
                    dwt_readdiagnostics(&rx_diag);//读取信号强度等诊断信息
                    rx_power = rx_diag.rxPower;
#endif
                }
                else
                {
                    prev_range[recv_tag_id].distance = -1;
                }
                range_status = RANGE_TWR_OK;  //设置TWR成功测距标志，在dw_main.c里判断打包串口输出

                led_off(LED2);
                led_off(LED3); 
            }
            dwt_forcetrxoff();
            state = STA_INIT_POLL_SYNC;
            break;
        
        default:
            break;
    }
   
}
uint32_t g_bFrameCnt = 0;    //接收数据帧计数

/*! ------------------------------------------------------------------------------------------------------------------
* @fn anc_rx_ok_cb()
*
* @brief Callback to process RX good frame events
*
* @param  cb_data  callback data
*
* @return  none
*/
void anc_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_OK;
    if (cb_data->datalength <= FRAME_LEN_MAX_EX)  //接收数据
    {
       dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }
    if (cb_data->datalength > 16)  //接收数据
    {
        g_bFrameCnt++;
        // if(g_bFrameCnt%2000==0){
        //     uint32_t framNum=*((int *)(rx_buffer+FUNC_CODE_IDX));
        //     // if (g_bFrameCnt != framNum)
        //     // {
        //         printf("frame error %d %d\n",g_bFrameCnt,framNum);
        //     // }
        // }
        //dwt_rxenable(DWT_START_RX_IMMEDIATE);         //打开接收机，等待接收数据       
    }
    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn anc_rx_to_cb()
*
* @brief Callback to process RX timeout events
*
* @param  cb_data  callback data
*
* @return  none
*/
void anc_rx_to_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_TIMEOUT;
    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn anc_rx_err_cb()
*
* @brief Callback to process RX error events
*
* @param  cb_data  callback data
*
* @return  none
*/
void anc_rx_err_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_ERROR;
    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn anc_tx_conf_cb()
*
* @brief Callback to process TX confirmation events
*
* @param  cb_data  callback data
*
* @return  none
*/
void anc_tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    tx_status = TX_OK;
    UNUSED(cb_data);
}

#endif
