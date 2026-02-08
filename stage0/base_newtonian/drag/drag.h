#include <gtk/gtk.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include "rlgl.h"
#include "vector.h"
#include "physics.h"

//Type: Non-Comp: Header File
//Header file for defining aerodynamic drag relation on objects

void app_linear_drag (Particle *object, float b) { //b = coefficient of drag
    Vector2 force_drag = Vector2Scale ((object -> velocity), -b);
    ApplyForce (object, force_drag);
}

//Usage: app_linear_drag (&particle, 0.05f); 
