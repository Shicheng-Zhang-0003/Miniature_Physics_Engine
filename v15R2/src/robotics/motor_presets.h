/* MPE_FTC_071: FTC motor presets */
#ifndef motor_presets_h
#define motor_presets_h
#include "motor.h"

typedef enum {
    MOTOR_GB_5203_19_2, /* goBILDA Yellow Jacket 19.2:1 */
    MOTOR_GB_5203_30, /* goBILDA Yellow Jacket 30:1 */
    MOTOR_GB_5203_43_7, /* goBILDA Yellow Jacket 43.7:1 */
    MOTOR_GB_5203_71, /* goBILDA Yellow Jacket 71:1 */
    MOTOR_REV_CORE_HEX, /* REV Core Hex Motor */
    MOTOR_COUNT
} motor_preset_id;

void motor_preset_apply(motor *m, motor_preset_id id);
const char *motor_preset_name(motor_preset_id id);

#endif /* motor_presets_h */
