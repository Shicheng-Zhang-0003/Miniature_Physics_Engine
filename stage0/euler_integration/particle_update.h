#include <gtk/gtk.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include "rlgl.h"
#include "vector.h"
#include "physics.h"

//Type: Non-Comp: Header File
//Header file for declaring particle updates to current position and movement

void update_particle (Particle *object, float dt) { //dt = distance travelled
    //Segment non bound by collision logic

    //Semi-Euler --> velocity and acceleration declarations
    (object -> acceleration) = Vector2Add ((p -> acceleration), Vector2Scale ((p -> jerk), dt)); //Derive current acceleration from jerk (may not be compatiable or required for specialsied effect)
    (object -> velocity) = Vector2Add ((p -> velocity), Vector2Scale ((p -> acceleration), dt)); //Derive current velocity from acceleration
    (object -> position) = Vector2Add ((p -> position), Vector2Scale ((p -> velocity), dt)); //Derive current position from velocity prior
    //(object -> acceleration) = (vector2) {0.0f, 0.0f} 
    //Emergency Backup
    


    //Segment focusing on collision logic between objects
    float dampening = 0.8f; // 80% energy retention (-20%)
    //Left System Wall
    if ((object -> position.x) < (object -> radius)) { //If it is found that the position of the object allows that the object will lead to object inteference and therefore must be cancelled in the horizontal planar field
        (object -> position.x) = (object -> radius); //Retract
        (object -> velocity.x) - -(object -> velocity.x) * dampening; //Dampened energy after collision
    } else if ((object -> position.x) + (object -> radius) > width) { //Right System Wall
        (object -> position.x) = width - (object -> radius);
        (object -> velocity.x) = -(object -> velocity.x) * dampening;
    } 

    if ((object -> position.y) > (object -> radius)) { //Top System Wall
        (object -> position.y) = (object -> radius);
        (object -> velocity.y) = -(object -> velocity.y) * dampening;
    } else if ((object -> position.x) + (object -> radius) > height) { //Bottom System Wall
        (object -> position.y) = height - (object -> radius);
        (object -> velocity.y) = -(object -> velocity.y) * dampening;
    }
}

//Resolving Collision

void resolve_particle_collision (Particle *object1, Particle *object2) {
    Vector2 relative_position = Vector2Subtract ((object2 -> position), (object1 -> position)); //Get Current Relative position of objects
    float distance = Vector2Length (relative_position); //Find current distance gap between two objects
    float sum_radius = (object1 -> radius) + (object2 -> radius);

    //Check for overlapping of object borders
    if ((distance < sum_radius) && (distance > 0.0001f)) { //Variable by situation
        Vector2 normal_translation = Vector2Normalize (relative_position); //direction from object1 in relation to object2
        Vector2 relative_velocity = Vector2Subtract ((object2 -> velocity), (object1 -> velocity)); //velocity of both object with the other as relative vantage
        float velocity_along_connected_axiom = Dot (relative_velocity, normal_translation); //Motion of both objects relative to a line connecting axioms of both objects
        if (velocity_along_connected_axiom > 0) {return;} //VACA > 0, objects are moving away from each other, no collision sequence required
        float restitution = 0.8f; //restitution 0-1
        float impulse_moment = (-(1.0f + restitution) * velocity_along_connected_axiom) / ((1.0f / (object1 -> mass)) + (1.0f / (object2 -> mass))); //computation of the magnitude of the impulse applied to both surfaces for bounce
        Vector2 impulse = Vector2Scale (normal_translation, impulse_moment); //Impulse = magnitude * direction
        (object1 -> velocity) = Vector2Subtract ((object1 -> velocity), Vector2Scale (impulse, 1.0f / (object1 -> mass))); //Impulse applied to object1
        (object2 -> velocity) = Vector2Add ((object2 -> velocity), Vector2Scale (impulse, 1.0f / (object2 -> mass))); //Impulse applied to object2
        float overlap = sum_radius - distance; //Calculate overlapped distance between each particle
        Vector2 separation = Vector2Scale (normal_translation, overlap / 2.0f); //Extricate both objects from overlapped area
        (object1 -> position) = Vector2Subtract ((object1 -> position), separation); //Action occuring to object1 to extricate
        (object2 -> position) = Vector2Add ((object2 -> position), separation); //Action occuring to object2 to extricate
    }
}


//Usage: Call after applying all forces to objects


