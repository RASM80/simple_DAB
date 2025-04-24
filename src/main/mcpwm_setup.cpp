#include "mcpwm_setup.h"

static const char* TAG = "MCPWM_PWM";

static const int mcpwm_gpio[4] = {
  GPIO1, GPIO2, GPIO3, GPIO4
};
static mcpwm_gen_handle_t mcpwm_gen[4];
static mcpwm_timer_handle_t pwm_timer;
static mcpwm_dead_time_config_t dt_config_2 = {
  .posedge_delay_ticks = 0,
  .negedge_delay_ticks = 0,
};
static mcpwm_dead_time_config_t dt_config_4 = {
  .posedge_delay_ticks = 0,
  .negedge_delay_ticks = 0,
  .flags = { .invert_output = true },
};


void update_phase(double phase) {

  uint32_t pd = (phase / 180.0) * max_dt;

  REG_WRITE(0x3FF5E05C, (uint32_t)pd);
  REG_WRITE(0x3FF5E060, (uint32_t)pd);
  REG_WRITE(0x3FF5E094, (uint32_t)pd);
  REG_WRITE(0x3FF5E098, (uint32_t)pd);

  return;
}

void setup_mcpwm() {
  ESP_LOGI(TAG, "Initializing MCPWM module");


  // Create and configure MCPWM timer
  mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = 160000000,  // 1 MHz resolution
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = mcpwm_t_period,  // Calculate period ticks
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &pwm_timer));
  ESP_LOGI(TAG, "MCPWM Timer Done.");


  // Create and configure operators
  mcpwm_oper_handle_t pwm_opr[2];
  mcpwm_operator_config_t operator_config = {
    .group_id = 0,
    .flags = {
      .update_dead_time_on_tez = true,
    },
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &pwm_opr[0]));
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &pwm_opr[1]));
  // Connect operators to the same timer
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(pwm_opr[0], pwm_timer));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(pwm_opr[1], pwm_timer));
  ESP_LOGI(TAG, "MCPWM Operator Done.");


  // Create and configure comparators for PWM1 and PWM2
  mcpwm_cmpr_handle_t pwm_comparator[2];
  mcpwm_comparator_config_t comparator_config = {
    .flags = {
      .update_cmp_on_tez = true,
    },
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(pwm_opr[0], &comparator_config, &pwm_comparator[0]));
  ESP_ERROR_CHECK(mcpwm_new_comparator(pwm_opr[1], &comparator_config, &pwm_comparator[1]));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(pwm_comparator[0], init_duty));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(pwm_comparator[1], init_duty));
  ESP_LOGI(TAG, "MCPWM Comparator Done.");


  // Create and configure generators for PWM1 and PWM2
  mcpwm_generator_config_t generator_config;
  for (int i = 0; i < 4; i++) {
    generator_config.gen_gpio_num = mcpwm_gpio[i];
    if (i > 1) generator_config.flags.invert_pwm = false;
    ESP_ERROR_CHECK(mcpwm_new_generator(pwm_opr[i / 2], &generator_config, &mcpwm_gen[i]));


    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mcpwm_gen[i],
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mcpwm_gen[i],
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pwm_comparator[i / 2], MCPWM_GEN_ACTION_HIGH)));
  }


  ESP_LOGI(TAG, "MCPWM Generator Done.");

  //dt0 PWM_DT0_CFG_REG
  REG_WRITE(0x3FF5E058, 0x0002A711);

  //dt1 PWM_DT1_CFG_REG
  REG_WRITE(0x3FF5E090, 0x00028711);

  REG_WRITE(0x3FF5E05C, (uint32_t)400);
  REG_WRITE(0x3FF5E060, (uint32_t)400);
  REG_WRITE(0x3FF5E094, (uint32_t)400);
  REG_WRITE(0x3FF5E098, (uint32_t)400);
  //PWM_DT0_FED_CFG_REG 0x3FF5E05C
  //PWM_DT0_RED_CFG_REG 0x3FF5E060
  //PWM_DT1_FED_CFG_REG 0x3FF5E094
  //PWM_DT1_RED_CFG_REG 0x3FF5E098

  ESP_LOGI(TAG, "Force the output level to low, timer still running");
  for (int i = 0; i < 4; i++) {
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(mcpwm_gen[i], 0, true));
  }

  // Enable and start timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP));

  ESP_LOGI(TAG, "MCPWM setup complete.");
}


void mcpwm_force_lvl(bool lvl) {

  switch (lvl) {

    case false:
      for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(mcpwm_gen[i], -1, true));
      }
      break;

    case true:
      for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(mcpwm_gen[i], 0, true));
      }
      break;
  }
}
