/* ========================================
 *
 * LED helper functions for Belka
 * Code implementation (.c file)
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2019
 * (insert license later.)
 *
 * ========================================
*/

// Include Cypress' libraries, which we need for various things.
// These have their own include guards so are "safe" without an ifndef here.
#include <project.h>

// and we include the corresponding header file just for consistency in definitions.
#include "led_helper_fcns.h"

void set_led_constant() {
    // set a duty cycle of 100% by making the compare "max", i.e., the whole period.
    LED_PWM_WriteCompare(LED_PWM_ReadPeriod());
}

void set_led_blinkslow() {
    // 16 bit timer with period 1000 = 1 sec,
    LED_PWM_WritePeriod(300);
    // half duty
    LED_PWM_WriteCompare(LED_PWM_ReadPeriod()/2);
}

void set_led_blinkfast() {
    // 16 bit timer with period 1000 = 1 sec,
    LED_PWM_WritePeriod(100);
    // half duty
    LED_PWM_WriteCompare(LED_PWM_ReadPeriod()/2);
}

void set_led_dim() {
    // fast with low duty cycle smooths to a dim looking led
    LED_PWM_WritePeriod(10);
    LED_PWM_WriteCompare(1);
}

/* [] END OF FILE */
