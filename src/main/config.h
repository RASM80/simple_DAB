//MCPWM config

#define mcpwm_t_period 1600
#define init_duty (uint32_t)(mcpwm_t_period/2)
#define max_dt (uint32_t)(mcpwm_t_period/2)

#define GPIO1 4   //LL
#define GPIO2 16  //LR
#define GPIO3 17  //RL
#define GPIO4 5   //RR


#define CHOP 1
#define POL 1
#define BUF 0
#define REFDET 0
#define DTA_STA 1


#define MEASURE_PIN 21



