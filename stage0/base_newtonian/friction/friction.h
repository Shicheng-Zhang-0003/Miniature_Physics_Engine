#include <gtk/gtk.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include "rlgl.h"
#include "vector.h"
#include "physics.h"

//Type: Non-Comp: Header File
//Header file for defining friction force execution functions

//Kinetic Friction: Ff = uk * Fn * vectoring
void app_friction_force_kinetic (Particle *object_in_motion, float mu_kinetic) {
    float inst_spd = Vector2Length (object_und_friction -> velocity); //Isolate instatenous velocity at the time
    if (inst_spd > 0.0001f) { //Value Variable in order to achieve different precision statements
        Vector2 Fn = {0.0f, (p -> mass_obj) * 9.81f}; //Gravitation --> Force Normal
        float mag_Fn = Vector2Length (Fn); //Isolate raw scalar magnitude of Fn
        Vector2 friction_vector = Vector2Normalize (Vector2Scale ((object_in_motion -> inst_spd), -1.0f)); //Reverse Vector of Friction relative to motion.
        Vector2 friction_force = Vector2Scale (friction_vector, mu_kinetic * mag_Fn); //Compute actuated Friction Force
        ApplyForce (object_in_motion, friction_force); //Application
    }
} //Do NOT Apply Kinetic Friction to Static Friction situations or bodies

//Static Friction: Ff = u * Fn
void app_friction_force_static (Particle *object_static, float mu_static) {
    //Determine if object is already in motion --> static bad_call
    float inst_spd = Vector2Length (object_static -> velocity);
    if (inst_spd > 0.05f) {return;} /* Can be change to scale according to accuracy and precision (DCE)
        Also a smaller threshold for movement to prevent oscillation changes */ 
    
    Vector2 pending_fa = Vector2Scale (p -> acceleration, p -> mass); //Compute pending net force already experienced by the object (Fnet w/o friction application)
    float pfa_magnitude = Vector2Length (pending_fa); //Magnitude calculated of pending force applied
    
    //Compute largest amount of static friction currently pending and available
    float normal_magnitude = (p -> mass) * 9.81f; //g value
    float max_static_friction = mu_static * normal_magnitude;

    if (pfa_magnitude <= max_statitc_friction + 1e-6f) { //Values Variable for some surfaces and movements (DCE)
        //Sttaic friction cancels applied force --> no movement
        //Same opposite magnitude to cancel any form of acceleration
        Vector2 cancellation_friction = Vector2Scale (pending_fa, -1.0f); //Redirection of applied force (friction) (DCE)
        ApplyForce (object_static, cancellation_friction); //Application to object
        (p -> velocity) = (Vector2) {0.0f, 0.0f}; //Erase all residual velocity from decimal point calibration errors (marked with DCE)
    } else {
        //Applied Force overpowers static friction --> object begins sliding motion
        //The friction is applied to the opposite direction to institute Fnet
        if (pfa_magnitude > 1e-6f) { //Values variable for some surfaces and movements (DCE)
            Vector2 opposite_direction = Vector2Normalize (Vector2Scale (pending_fa, -1.0f)); //Applied to opposite direction
            Vector2 max_ff_applied = Vector2Scale (opposite_direction, max_static_friction); 
            ApplyForce (object_static, max_ff_applied); //Apply Fnet to object
        }
    }
}

//Usage: app_friction_force_static (&particle, 0.1f); change last value for mu_static
//Usage: app_friction_force_kinetic (&particle, 0.1f);, change last value for mu_kinetic
