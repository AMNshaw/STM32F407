#include <stdlib.h>

#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/modules/kinematics_base.h"
#include "Agv_kinematics/mecanum_config.h"

/**
 * private declarations
 */

typedef struct {
    const AgvMecanumConfig* cfg;

} KineMecanumImpl;

static int Kinematics_mecanum_destroy(AgvKinematicsBase* base);

static int mecanum_calculate_wheels_vel(AgvKinematicsBase* base,
                                        const Twist2D* cmd_vel_in,
                                        WheelsVel* wheels_vel_out);

static int mecanum_calculate_odom(AgvKinematicsBase* base,
                                  const WheelsAng* wheels_ang_in,
                                  const WheelsVel* wheels_vel_in,
                                  Odometry* odom_out);

int forward_kine(const AgvMecanumConfig* cfg, const Wheels4F* wheels_in,
                 XYYaw* body_out);

int inverse_kine(const AgvMecanumConfig* cfg, const Twist2D* body_in,
                 Wheels4F* wheels_out);
/**
 * private definitions
 */

int Kinematics_mecanum_create(AgvKinematicsBase* out,
                              const AgvMecanumConfig* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;
    KineMecanumImpl* impl = (KineMecanumImpl*)malloc(sizeof(KineMecanumImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;

    impl->cfg = cfg;

    out->impl = impl;
    out->calculate_wheels_vel = mecanum_calculate_wheels_vel;
    out->calculate_odom = mecanum_calculate_odom;
    out->destroy = Kinematics_mecanum_destroy;

    return AGV_OK;
}

static int Kinematics_mecanum_destroy(AgvKinematicsBase* out) {
    if (!out) return AGV_OK;
    KineMecanumImpl* impl = (KineMecanumImpl*)out->impl;
    if (impl) {
        free(impl);
    }

    out->impl = NULL;
    out->calculate_wheels_vel = NULL;
    out->calculate_odom = NULL;
    out->destroy = NULL;

    return AGV_OK;
}

static int mecanum_calculate_wheels_vel(AgvKinematicsBase* base,
                                        const Twist2D* cmd_vel_in,
                                        WheelsVel* wheels_vel_out) {
    if (!base || !cmd_vel_in || !wheels_vel_out) return AGV_ERR_INVALID_ARG;

    KineMecanumImpl* impl = (KineMecanumImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;

    return inverse_kine(impl->cfg, cmd_vel_in, wheels_vel_out);
}

static int mecanum_calculate_odom(AgvKinematicsBase* base,
                                  const WheelsAng* wheels_ang_in,
                                  const WheelsVel* wheels_vel_in,
                                  Odometry* odom_out) {
    if (!base || !wheels_ang_in || !wheels_vel_in || !odom_out)
        return AGV_ERR_INVALID_ARG;
    KineMecanumImpl* impl = (KineMecanumImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;

    int code = AGV_OK;

    code = forward_kine(impl->cfg, wheels_ang_in, &odom_out->pose);
    if (code != AGV_OK) return code;
    code = forward_kine(impl->cfg, wheels_vel_in, &odom_out->twist);

    return code;
}

int forward_kine(const AgvMecanumConfig* cfg, const Wheels4F* wheels_in,
                 XYYaw* body_out) {
    if (!cfg || !wheels_in || !body_out) return AGV_ERR_INVALID_ARG;

    body_out->x = (wheels_in->data[0] + wheels_in->data[1] +
                   wheels_in->data[2] + wheels_in->data[3]) *
                  cfg->wheel_radius / 4;

    body_out->y = (-wheels_in->data[0] + wheels_in->data[1] +
                   wheels_in->data[2] - wheels_in->data[3]) *
                  cfg->wheel_radius / 4;

    body_out->yaw = (-wheels_in->data[0] + wheels_in->data[1] -
                     wheels_in->data[2] + wheels_in->data[3]) *
                    cfg->wheel_radius / (4 * (cfg->L + cfg->W));

    return AGV_OK;
}

int inverse_kine(const AgvMecanumConfig* cfg, const Twist2D* body_in,
                 Wheels4F* wheels_out) {
    if (!cfg || !body_in || !wheels_out) return AGV_ERR_INVALID_ARG;

    float tmp = cfg->L + cfg->W;

    wheels_out->data[0] =
        (body_in->x - body_in->y - tmp * (body_in->yaw)) / cfg->wheel_radius;
    wheels_out->data[1] =
        (body_in->x + body_in->y + tmp * (body_in->yaw)) / cfg->wheel_radius;
    wheels_out->data[2] =
        (body_in->x + body_in->y - tmp * (body_in->yaw)) / cfg->wheel_radius;
    wheels_out->data[3] =
        (body_in->x - body_in->y + tmp * (body_in->yaw)) / cfg->wheel_radius;

    return AGV_OK;
}