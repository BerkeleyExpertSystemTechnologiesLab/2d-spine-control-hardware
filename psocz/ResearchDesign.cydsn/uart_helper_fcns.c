/* ========================================
 *
 * UART Helper Functions for 2D Spine Control Test
 * Code implementation (.c file)
 *
 * Copyright Berkeley Emergent Space Tensegrities Lab, 2018
 * (insert license later.)
 *
 * ========================================
*/

// Adapted from Cypress' example CE95277 ADC and UART,
// and from Andrew Sabelhaus' example code for ME235 S'18 at UC Berkeley.

// As one aside: floating point numbers don't seem to work on the PSoC unless
// specifically enabled. Follow https://community.cypress.com/docs/DOC-9389
// for enabling floats (newlib_nano = False, and heap size change to 0x1000.)

// Include Cypress' libraries, which we need for various things.
// These have their own include guards so are "safe" without an ifndef here.
#include <project.h>
// stdio provides the sprintf and sscanf for parsing UART input and producing output
#include "stdio.h"
// and we include the corresponding header file just for consistency in definitions.
#include "uart_helper_fcns.h"
// Also need the global data storage variables for writing to and from
#include "data_storage.h"
#include "string.h"

// Local variables which do not have scope outside this file:
// Transmit and receive buffers for the PSoC's UART will be strings,
// with the following number of characters:
#define TRANSMIT_LENGTH 128
#define RECEIVE_LENGTH 128
// Since these buffers will only be used in this helper function,
// we can declare them here.
char transmit_buffer[TRANSMIT_LENGTH];
char receive_buffer[RECEIVE_LENGTH];

// Zach's code/modifications
float degrees;
// ticks_per_rev for 30 Watt motors
float ticks_per_rev = 435356.467;
// 512 counts per turn. Abs. Gear reduction of 367.724
// This is more or less correct. It seems to be slightly under but negligible.
float ticks_per_rev_big = 189298.887;
int leg;
#define MAX_LEG_ANGLE 45
#define DEF_LEG_ANGLE 0
#define MIN_LEG_ANGLE -45

#define MAX_SPINE_LENGTH 3
#define DEF_SPINE_LENGTH 0
#define MIN_SPINE_LENGTH -3
#define MAX_SPINE_ROT_LENGTH 4
#define MIN_SPINE_ROT_LENGTH -4

// char cmd_str[2] = "00";

// We need to keep track of the number of characters received as they come in over UART,
// so that we can properly place them into the buffer.
// Needs to be global since used in a few different functions.
static uint8 num_chars_received = 0;

// Definition of the UART interrupt service routine.
// This stores the incoming data in a buffer, then calls the parser when command is done (newline.)
CY_ISR( Interrupt_Handler_UART_Receive ){
    // We assume this ISR is called when a byte is received.
    uint8 received_byte = UART_GetChar();
    
    // C allows us to "switch" on uint8s, since characters are also numbers via the ASCII table.
    // very convenient.
    // The switch-case statement makes it easy to do single-character commands.
    
    switch( received_byte )
    {
        case '\r':
            // flow downward, no specific lines of code for carriage return
        
        case '\n':
            // newline or carriage return received, so finally set the PWM parameters
            // First, terminate the string. This is for the use of sscanf below.
            receive_buffer[num_chars_received] = '\0';
            // Print back the newline/carriage return, to complete the "respond back to the terminal" code
            // Note: for files, linux needs \n whereas windows needs \r\n, but for terminals,
            // it seems that both want \r\n (CR+LF).
            UART_PutString("\r\n");
            // Call the helper function to actually set the PWM
            UART_Command_Parser();
            break;

        default:
            // The "default" case is "anything else", which is "store another character."
            // Add to the received buffer.
            receive_buffer[num_chars_received] = received_byte;
            // Respond back to the terminal
            UART_PutChar( received_byte );
            // We need to increment the counter. i++ does this without an equals sign for assignment
            num_chars_received++;
            
            
            // UART_PutString(received_byte);
            
            break;
        // end of case statement.
    }
}

/**
 * The command parser itself. See the welcome message for up-to-date list of commands.
 */
void UART_Command_Parser() {
    // First, get the command, and switch on it.
    char cmd = 0;
    
    // this should only read the first character 
    // sscanf(receive_buffer, "%c", &cmd);
    sscanf(receive_buffer, "%s" ,&cmd); // This recieves multiple characters
    // UART_PutString(&cmd);
    
    // sscanf(receive_buffer[0], "%c", &cmd);
    // sscanf(receive_buffer[1], "%c", &cmd);
    
    // When parsing a more complicated command below, we'll need to check
    // if the correct number of arguments were specified, and otherwise throw an error.

    uint8 num_filled;
    
    // switch on the command.
    // long length = strlen(&cmd);
    // char help[TRANSMIT_LENGTH]; //= sscanf("String length: %li/r/n",strlen(&cmd));
    // sprintf(help, "String length: %li\r\n", length);
    // UART_PutString(help);
    
    if(strlen(&cmd) == 1){
        switch(cmd) {
                
            case 'u':
                // The most important one! 
                // The floats can be put directly into the global variable.
    //            num_filled = sscanf(receive_buffer, "u %f %f %f %f", &current_control[0],
    //                   &current_control[1], &current_control[2], &current_control[3]);
                // we'll originally store the cm value.
                num_filled = sscanf(receive_buffer, "u %f %f %f %f", &control_in_cm[0],
                       &control_in_cm[1], &control_in_cm[2], &control_in_cm[3]);
                
                // Print out a message according to how much was parsed.
                if( num_filled == 4 ){
                    // Return the resulting data that was stored. First, in original input units:

                    // Calculate the control inputs in terms of encoder ticks.
                    // Assignment to an int automatically casts the float.

                    // Encoder res = 512 counts per turn, Gear res = 850.3056:1 reduction 
                    // Z~ I am not sure why this is defined in the if-statement. I defined it outside
                    //float ticks_per_rev = 435356.467;

                    // TO-DO: replace with the #define'd constants. More efficient.
                    
                    // current_control[0] = (-1*control_in_cm[0]*ticks_per_rev)/(2*PI*RADIUS);
                    current_control[0] = control_in_cm[0]*ticks_per_rev/360;
                    // Control input now in degrees for 60 watt motor
                    current_control[1] = control_in_cm[1]*ticks_per_rev_big/360;
                    // current_control[1] = (control_in_cm[1]*ticks_per_rev)/(2*PI*RADIUS);
                    current_control[2] = (control_in_cm[2]*ticks_per_rev)/(2*PI*RADIUS);
                    current_control[3] = control_in_cm[3]*ticks_per_rev_big/360;
                    // current_control[3] = (control_in_cm[3]*ticks_per_rev)/(2*PI*RADIUS);
                    
                    sprintf(transmit_buffer, "Stored an input, converted to encoder ticks, of %li, %li, %li, %li\r\n", current_control[0],
                        current_control[1], current_control[2], current_control[3]);
                    tensioning = 0;
                    controller_status = 1;
                    motor_1 = 1;
                    motor_2 = 1;
                    motor_3 = 1;
                    motor_4 = 1;
                    print = 1;
                }
                else {
                    // did not receive exactly 4 control inputs.
                    sprintf(transmit_buffer, "Error!! You typed %s, which gave %i control inputs when 4 were expected.\r\n", receive_buffer, num_filled);
                }
                break;
                
            // query the encoder ticks (current positions.)
            case 'e':
                // values are held in "count."
                sprintf(transmit_buffer, "Current encoder tick counts are %li, %li, %li, %li\r\n", QuadDec_Motor1_GetCounter(),
                   QuadDec_Motor2_GetCounter(), QuadDec_Motor3_GetCounter(), QuadDec_Motor4_GetCounter());
                break;
                
            // query current error signal (control input - encoder ticks.)
            case 'o':
                // values held in "error."
                sprintf(transmit_buffer, "Current error signals are:\r\n P = {%li, %li, %li, %li}, \r\n I = {%li, %li, %li, %li}, \r\n D = {%li, %li, %li, %li}\r\n", error[0],
                    error[1], error[2], error[3], integral_error[0], integral_error[1], integral_error[2], 
                    integral_error[3], deriv_error[0], deriv_error[1], deriv_error[2], deriv_error[3]);
                break;
                
            case 'p':
                // current PWM value
                sprintf(transmit_buffer, "Current control: %hu\r\n",PWM_1_ReadCompare());

                break;
                
            // Tensioning command for small adjustments / calibration.
            // Now uses the macros from data_storage.h
            case 't':    
                sscanf(receive_buffer, "t %f", &tension_control);
                if (tension_control == 1) {
                    first_loop_1 = 1;
                    motor_1 = 1;
                    current_control[0] = current_control[0] - T_TICKS_QD;
                }
                else if (tension_control == -1) {
                    first_loop_1 = 1;
                    motor_1 = 1;                
                    current_control[0] = current_control[0] + T_TICKS_QD;
                }
                else if (tension_control == 2) {
                    first_loop_2 = 1;
                    motor_2 = 1;
                    current_control[1] = current_control[1] + T_TICKS_QD;
                }
                else if (tension_control == -2) {
                    first_loop_2 = 1;
                    motor_2 = 1;
                    current_control[1] = current_control[1] - T_TICKS_QD;
                }         
                else if (tension_control == 3) {
                    first_loop_3 = 1;
                    motor_3 = 1;
                    current_control[2] = current_control[2] - T_TICKS_QD;
                }
                else if (tension_control == -3) {
                    first_loop_3 = 1;
                    motor_3 = 1;                
                    current_control[2] = current_control[2] + T_TICKS_QD;
                }
                else if (tension_control == 4) {
                    first_loop_4 = 1;
                    motor_4 = 1;
                    current_control[3] = current_control[3] + T_TICKS_QD;
                }
                else if (tension_control == -4) {
                    first_loop_4 = 1;
                    motor_4 = 1;                
                    current_control[3] = current_control[3] - T_TICKS_QD;
                }
                //tensioning = 1;
                controller_status = 1;
                sprintf(transmit_buffer, "Adjusted tensions. Control inputs are now %li, %li, %li, %li\r\n", current_control[0],
                        current_control[1], current_control[2], current_control[3]);
                break;
                
            // E-stop. "disable."
            // This function (a) turns off the PWM, (b) resets the control, (c) resets the encoder count.
            case 'd':
                PWM_1_WriteCompare(0);
                PWM_2_WriteCompare(0);
                PWM_3_WriteCompare(0);
                PWM_4_WriteCompare(0);
                PWM_1_Stop();
                PWM_2_Stop();
                PWM_3_Stop();
                PWM_4_Stop();
                
                current_control[0] = 0;
                current_control[1] = 0;
                current_control[2] = 0;
                current_control[3] = 0;
                count_3 = 0;
                count_4 = 0;
                
                // reset the quadrature encoder counts
                QuadDec_Motor1_SetCounter(0);
                QuadDec_Motor2_SetCounter(0);
                QuadDec_Motor3_SetCounter(0);
                QuadDec_Motor4_SetCounter(0);
                
                // also, reset all the error terms.
                error[0] = 0;
                error[1] = 0;
                error[2] = 0;
                error[3] = 0;
                integral_error[0] = 0;
                integral_error[1] = 0;
                integral_error[2] = 0;
                integral_error[3] = 0;
                
                sprintf(transmit_buffer, "Controls and encoder counts reset, PWM now of.\r\n");
                break;
                
            case 'q':
                // query the state of the store control commands.
                sprintf(transmit_buffer, "Current control inputs are (in encoder ticks): %li, %li, %li, %li\r\n", current_control[0],
                        current_control[1], current_control[2], current_control[3]);
                break;
                
            case 'w':
                // echo back the welcome message.
                UART_Welcome_Message();
                // and push an empty string to the transmit buffer.
                sprintf(transmit_buffer, "\r\n");
                break;
                
            case 'c':
                // Clear the UART's buffers. This shouldn't ever need to be used,
                // but is provided in case "something bad happens"
                UART_ClearRxBuffer();
                UART_ClearTxBuffer();
                // still need to specify some message to send back to the terminal.
                sprintf(transmit_buffer, "Cleared RX and TX buffers for the UART on the PSoC.\r\n");
                break;
            
            case 'n':
                // enaBle the PWMs, after having pressed d to turn them off.
                PWM_1_Enable();
                PWM_2_Enable();
                PWM_3_Enable();
                PWM_4_Enable();
                sprintf(transmit_buffer, "PWMs re-enabled.\r\n");
                break;
                
            default:
                // In any other case, report an error.
                sprintf(transmit_buffer, "Error! Command not recognized!\r\n");
                break;
            
        }
    }else{
            // LEGS:
            // left front leg max angle
            if(strcmp(&cmd,"lff")==0) {
                current_control[0] = MAX_LEG_ANGLE*ticks_per_rev/360;
                sprintf(transmit_buffer, "Current control inputs are (in encoder ticks): %li, %li, %li, %li\r\n", current_control[0],
                        current_control[1], current_control[2], current_control[3]);
            }
            // left front leg min angle
            else if(strcmp(&cmd,"lfb")==0) {
                current_control[0] = MIN_LEG_ANGLE*ticks_per_rev/360;
            }
            // left front leg zero angle
            else if(strcmp(&cmd,"lf")==0) {
                current_control[0] = DEF_LEG_ANGLE*ticks_per_rev/360;
            }
            
            // left back leg max angle
            else if(strcmp(&cmd,"lbf")==0) {
                current_control[2] = MAX_LEG_ANGLE*ticks_per_rev/360;
            }
            // left back leg min angle
            else if(strcmp(&cmd,"lbb")==0) {
                current_control[2] = MIN_LEG_ANGLE*ticks_per_rev/360;
            }
            // left front leg zero angle
            else if(strcmp(&cmd,"lb")==0) {
                current_control[2] = DEF_LEG_ANGLE*ticks_per_rev/360;
            }
            
            // Right front leg max angle
            else if(strcmp(&cmd,"rff")==0) {
            current_control[0] = MAX_LEG_ANGLE*ticks_per_rev/360;
            }
            // Right front leg min angle
            else if(strcmp(&cmd,"rfb")==0) {
            current_control[0] = MIN_LEG_ANGLE*ticks_per_rev/360;
            }
            // Right front leg zero angle
            else if(strcmp(&cmd,"rf")==0) {
            current_control[0] = DEF_LEG_ANGLE*ticks_per_rev/360;
            }
            
            // Right back leg max angle
            else if(strcmp(&cmd,"rbf")==0) {
                current_control[2] = MAX_LEG_ANGLE*ticks_per_rev/360;
            }
            // Right front leg min angle
            else if(strcmp(&cmd,"rbb")==0) {
                current_control[2] = MIN_LEG_ANGLE*ticks_per_rev/360;
            }
            // Right front leg zero angle
            else if(strcmp(&cmd,"rb")==0) {
                current_control[2] = DEF_LEG_ANGLE*ticks_per_rev/360;
            }

            // Need a left/right command and CW/CCW command
            // SPINE:
            if(strcmp(&cmd,"scw")==0) {
                current_control[1] = MIN_SPINE_ROT_LENGTH*ticks_per_rev/(2*PI*RADIUS);
                current_control[3] = MAX_SPINE_ROT_LENGTH*ticks_per_rev/(2*PI*RADIUS);
                sprintf(transmit_buffer, "Current control inputs are (in encoder ticks): %li, %li, %li, %li\r\n", current_control[0],
                current_control[1], current_control[2], current_control[3]);
            }
            else if(strcmp(&cmd,"sccw")==0) {
                current_control[3] = MIN_SPINE_ROT_LENGTH*ticks_per_rev/(2*PI*RADIUS);
                current_control[1] = MAX_SPINE_ROT_LENGTH*ticks_per_rev/(2*PI*RADIUS);
            }
            
            if(strcmp(&cmd,"sl")==0) {
                current_control[1] = MIN_SPINE_LENGTH*ticks_per_rev/(2*PI*RADIUS);
                current_control[3] = MAX_SPINE_LENGTH*ticks_per_rev/(2*PI*RADIUS);
                sprintf(transmit_buffer, "Current control inputs are (in encoder ticks): %li, %li, %li, %li\r\n", current_control[0],
                current_control[1], current_control[2], current_control[3]);
            }
            else if(strcmp(&cmd,"sr")==0) {
                current_control[3] = MIN_SPINE_LENGTH*ticks_per_rev/(2*PI*RADIUS);
                current_control[1] = MAX_SPINE_LENGTH*ticks_per_rev/(2*PI*RADIUS);
            }
            
            if(strcmp(&cmd,"sd")==0) {
                current_control[1] = DEF_SPINE_LENGTH*ticks_per_rev/(2*PI*RADIUS);
                current_control[3] = DEF_SPINE_LENGTH*ticks_per_rev/(2*PI*RADIUS);
                sprintf(transmit_buffer, "Current control inputs are (in encoder ticks): %li, %li, %li, %li\r\n", current_control[0],
                current_control[1], current_control[2], current_control[3]);
            }
            tensioning = 0;
            controller_status = 1;
            motor_1 = 1;
            motor_2 = 1;
            motor_3 = 1;
            motor_4 = 1;
            print = 1;

    }
    
    
    // Write the resulting message.
    UART_PutString(transmit_buffer);
    // and reset the counter into the receive buffer so that 
    // the ISR overwrites the last command.
    num_chars_received = 0;
}

// The welcome message.
// UPDATE THIS when new functionality is added.
void UART_Welcome_Message(){
    
    UART_PutString("\r\n2D Spine Controller Test.\r\n");
    UART_PutString("Copyright 2018 Berkeley Emergent Space Tensegrities Lab.\r\n");
    UART_PutString("Usage: send strings of the form (char) (optional_args). Currently supported:\r\n");
    UART_PutString("(NOTE: THESE MUST BE FOLLOWED EXACTLY, with exact spacing.)\r\n\n");
    UART_PutString("q = Query currently-stored control input\r\n");
    UART_PutString("d = Disable / emergency stop. The Big Red Button. Resets all counts (encoder, control.)\r\n");
    UART_PutString("w = echo back this Welcome message\r\n");
    UART_PutString("c = Clear all UART tx/rx buffers on the PSoC \r\n");
    UART_PutString("u float float float float = assign U, control input\r\n");
    UART_PutString("t{-}int = Tensioning for calibration. Small steps in each direction.\r\n");
    UART_PutString("              E.g. t-4 = motor 4, loosen.\r\n");
    UART_PutString("n = eNable all the PWMs for the motors (useful after d.)\r\n");
    UART_PutString("e = query Encoder ticks (current motor positions.)\r\n");
    UART_PutString("o = query eRror signal, control - encoder ticks.\r\n\n");
    UART_PutString("Recommended use pattern:\r\n");
    UART_PutString("c to reset buffer, u 0 0 0 0 to loosen the cables, then pin the vertebra in place,\r\n");
    UART_PutString("t to tension appropriately, d to set the zero point, then finally send u commands.\r\n\n");
    //UART_PutString("Remember to set your terminal's newline to LF or automatic detection. (TeraTerm: Setup -> Terminal -> New-line).\n\n");
}

/* [] END OF FILE */
