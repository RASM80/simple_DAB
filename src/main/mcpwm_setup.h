#ifndef MCPWM_SETUP_H
#define MCPWM_SETUP_H


#include "driver/mcpwm_prelude.h"
#include "config.h"
#include "esp_log.h"
#include "esp_private/periph_ctrl.h"
#include "soc/soc.h"


void update_phase(double phase);
void setup_mcpwm();
void mcpwm_force_lvl(bool lvl);

#endif