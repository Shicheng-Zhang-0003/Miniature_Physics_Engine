#include <gtk/gtk.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include "rlgl.h"
#include "vector.h"
#include "physics.h"

//Type: Non-Comp: Header File
//Header File for applying the application of spring force

//Spring Force
//Fs = -kx
void app_spring_force (Particle *object, Vector2 rest_position, float k) { //Spring Coefficient
    Vector2 displacement = Vector2Subtract ((p -> position), rest_position); //Calculate x for Fs = -kx
    Vector2 spring_force = Vector2Scale (displacement, -k); //Calculation for Fs Applied
    ApplyForce (object, spring_force); //Apply to object in question
}

//Usage: app_spring_force (&particle, (Vector2) {400, 300}, 0.1f);
//Second Value: anchor values towards center of the system
//Third Value: k for Fs = -kx
