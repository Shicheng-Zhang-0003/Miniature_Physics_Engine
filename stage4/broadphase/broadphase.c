#include "broadphase.h"
int broadphase_generate_pairing (broadphase_pair *output_pairs, int maximum_pairs) {
    int counter = 0;
    for (int a = 0; a < object_count; a++) {
        for (int b = a + 1; b < object_count; b++) {
            if (counter >= maximum_pairs) {break;}
            rigidbody *ra = &obj_per_scene [a];
            rigidbody *rb = &obj_per_scene [b];
            //Check the distance squared between the objects against a control value with a slight tolerance margin.
            float margin = (ra->radius + rb->radius) * 1.1;
            vector3 delta_value = vector3_subtraction (rb->position, ra->position);
            float distance_squared = vector3_length_squared (delta_value);
            if (distance_squared <= (margin * margin)) {
                output_pairs [counter].a = a;
                output_pairs [counter].b = b;
                counter += 1;
            }
        }
    } return counter;
}
