/* ========================================
 *
 * LED helper functions for Belka
 * Header file
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2019
 * (insert license later.)
 *
 * ========================================
*/

/**
 * led_helper_fcns.h
 *
 * same usage as the UART helper libraries, but these are a nice way to get the LED to blink in different ways.
 */

// This is called an include guard. See https://en.wikipedia.org/wiki/Include_guard for more info.
// This is a macro also.
#ifndef LED_HELPER_FCNS_H
#define LED_HELPER_FCNS_H
    
// Cypress' project.h already has its own include guard, so we can safely
// do nested #include's here.
#include <project.h>

// Sets the LED to stay lit
void set_led_constant();

// sets the LED to blink slowly
void set_led_blinkslow();

// sets the LED to blink quickly
void set_led_blinkfast();

// sets the LED dim - fast PWM with low duty cycle
void set_led_dim();

#endif //LED_HELPER_FCNS_H

/* [] END OF FILE */
