#include <gtk/gtk.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include "rlgl.h"
#include "vector.h"
#include "physics.h"

//Type: Non-Comp: Header File
//Header file for declaring functions deriving grativty and gravitational acceleration

//Gravity close range (defined g value)
//Fg = mg
void app_gravity_base (Particle *object, float g) { //Gravity udner set acceleration (g)
    Vector2 gravity = {0.0f, (object -> mass) * g}; //Applied Downwards (Vector)
    ApplyForce (object, gravity); //Apply to object 
}

//Gravity two defined large scale bodies (undefined g value)
//Fg = Gm1m2r^-2
#define GRAVITATIONAL_CONSTANT 6.67430e-3f
void app_gravity_complex (Particle *planet1, Particle *planet2, float G) { //Gravity under two body rotational/unset acceleration
    Vector2 delta_complex = Vector2Subtract ((planet2 -> position), (planet1 -> position)); //Delta Movement vectoring for position and displacement
    float radius_squared = delta_complex.x * delta_complex.x + delta_complex.y * delta_complex.y;

    //No div by zero for smaller radius, limit radius to ultra small decimal for small object attraction
    if (radius_squared < 1e-6f) {return;} //Variable according to limtis of rendering as well as required situations/compromise
    //From here we can also handle collision nature in the same way

    float radius = sqrtf (r_squared);
    float magnitude_force = G * (planet1 -> mass) * (planet2 -> mass) / radius_squared;

    //Direction of movement: planet1 --> planet2, planet2 --> planet1
    Vector2 direction_mov = Vector2Scale (delta, 1.0f / r); //Inverse proportionality from mass and delta movement according to mass vectoring using Pythagoras

    //Force on planet1 --> planet2
    Vector2 force_upon_planet1 = Vector2Scale (direction_mov, magnitude_force); //Positive movement in this particular case even if inverted by mass
                                                                            
    //Force equal on return to planet1
    Vector2 force_upon_planet2 = Vector2Scale (force_upon_planet1, -1.0f); //Equal Magnitude except opposite vectoring

    ApplyForce (planet1, force_upon_planet1), ApplyForce (planet2, force_upon_planet2); //Apply both grativational forces to either object
}

//Usage: app_gravity_base (&particle, 98.1f) (look 9.81f for typo)
//Usage: app_gravity_complex (&particle1, &particle2, GRAV_CONSTANT) 
