#ifndef input_control_h
#define input_control_h
#include <stdbool.h>
#include "../camera/camera.h"
typedef struct {
    //Keyboard Movement Inputs
    bool w_key, a_key, s_key, d_key;
    //Mouse Status Inputs
    bool mouse_1;
    double last_x_input, last_y_input;
} input_status;
//Initialise input state to zeroing
void initialise_input (input_state *input);

#endif
