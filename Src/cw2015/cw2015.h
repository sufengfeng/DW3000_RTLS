#ifndef  __CW2015_H_
#define  __CW2015_H_

#define CW2015_ADDR             0xC4

/*CW2015 寄存器定义*/
#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_BATINFO             0x10
#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)
#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        //ATHD = 0%

extern uint8_t USE_CW2015;
extern uint8_t cw2015_init(void);
extern void cw2015_wakeup(void);
extern void cw2015_sleep(void);
extern uint8_t cw2015_read_battery(void);


#endif


