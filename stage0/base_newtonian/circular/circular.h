#include <gtk/gtk.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include "rlgl.h"
#include "vector.h"
#include "physics.h"

//Type: Non-Comp: Header File
//Header File for declaration for Centripetal force declarations 

void app_centripetal (Particle *rotating_object, Vector2 centerpoint, float strength) {
    Vector2 lock_centerpoint = Vector2Subtract (centerpoint, (rotating_object -> position)); //Get Radius of rotating system
    float radius = Vector2Length (lock_centerpoint); //Numeric value
    if (r > 0.0001f) { //Ensure non division by zero value
        Vector2 direction = Vector2Normalize (lock_centerpoint); //Find rotational inwazrd vector to pinpoint central pivot
        float velocity_circular = Vector2Length (p -> velocity); //Pinpoint instantaneous Vc
        Vector2 centripetal = Vector2Scale (direction, strength * (rotating_object -> mass) * velocity_circular * velocity_circular / radius); //Calculate actual centripetal force (mv^2r^-1)

        ApplyForce (rotating_object, centripetal); //Apply force to object
    }
}

//Usage: app_centripetal (&particle, (Vector2) {0, 0}, 1.0f);, change strength to balance out ratio transfer values.
