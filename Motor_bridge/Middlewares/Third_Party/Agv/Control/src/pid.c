#include <math.h>
#include <stdio.h>

#include "Agv_control/pid_config.h"
#include "Agv_core/agv_types.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/modules/control_law_base.h"
#include "Agv_core/utils.h"
#include "Agv_factory/control_law_builder.h"
#include "FreeRTOS.h"
#include "semphr.h"

/**
 * private declarations
 */

typedef struct {
    float e_integral;
    float e_prev;
} PidState;

typedef struct {
    const AgvCtrlPidConfig* cfg;

    Twist2D vel_des;
    Twist2D vel_curr;

    SemaphoreHandle_t mutex_cmd;
    SemaphoreHandle_t mutex_vel;

    TickType_t tick_last;

    PidState pid_st_x;
    PidState pid_st_y;
    PidState pid_st_yaw;

} CtrlPidImpl;

static int Ctrl_pid_destroy(AgvControlLawBase* out);

static int pid_set_des_vel(AgvControlLawBase* base, const Twist2D* cmd_in);

static int pid_set_curr_vel(AgvControlLawBase* base, const Twist2D* curr_vel);

static int pid_get_ctrl_cmd(AgvControlLawBase* base, Twist2D* cmd_out);

// helpers

float pid_virtual_acc(float curr, float des, float dt, float max_acc,
                      PidState* pid_st, const PidConfig* gains);

/**
 * Private definitions
 */

int Ctrl_pid_create(AgvControlLawBase* out, const AgvCtrlPidConfig* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;

    CtrlPidImpl* impl = (CtrlPidImpl*)malloc(sizeof(CtrlPidImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;
    impl->cfg = cfg;

    const char* name = "PID ctrl";

    impl->mutex_cmd = xSemaphoreCreateMutex();
    if (impl->mutex_cmd == NULL) {
        free(impl);
        return AGV_ERR_MUTEX_FAIL;
    }

    impl->mutex_vel = xSemaphoreCreateMutex();
    if (impl->mutex_vel == NULL) {
        vSemaphoreDelete(impl->mutex_cmd);
        free(impl);
        return AGV_ERR_MUTEX_FAIL;
    }
    impl->tick_last = xTaskGetTickCount();
    memset(&impl->vel_des, 0, sizeof(Twist2D));
    memset(&impl->vel_curr, 0, sizeof(Twist2D));

    impl->pid_st_x.e_integral = 0;
    impl->pid_st_x.e_prev = 0;
    impl->pid_st_y.e_integral = 0;
    impl->pid_st_y.e_prev = 0;
    impl->pid_st_yaw.e_integral = 0;
    impl->pid_st_yaw.e_prev = 0;

    out->name = name;
    out->impl = impl;
    out->set_des_vel = pid_set_des_vel;
    out->set_curr_vel = pid_set_curr_vel;
    out->get_ctrl_cmd = pid_get_ctrl_cmd;
    out->destroy = Ctrl_pid_destroy;

    LOG(out->name, "Control law module created");
    return AGV_OK;
}

static int Ctrl_pid_destroy(AgvControlLawBase* out) {
    if (!out) return AGV_OK;

    CtrlPidImpl* impl = (CtrlPidImpl*)out->impl;
    if (impl) {
        if (impl->mutex_cmd) {
            vSemaphoreDelete(impl->mutex_cmd);
            impl->mutex_cmd = NULL;
        }
        if (impl->mutex_vel) {
            vSemaphoreDelete(impl->mutex_vel);
            impl->mutex_vel = NULL;
        }

        free(impl);
    }

    out->impl = NULL;
    out->set_des_vel = NULL;
    out->set_curr_vel = NULL;
    out->get_ctrl_cmd = NULL;
    out->destroy = NULL;

    return AGV_OK;
}

static int pid_set_des_vel(AgvControlLawBase* base, const Twist2D* cmd_in) {
    if (!base || !cmd_in) return AGV_ERR_INVALID_ARG;
    CtrlPidImpl* impl = (CtrlPidImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    xSemaphoreTake(impl->mutex_cmd, portMAX_DELAY);
    memcpy(&impl->vel_des, cmd_in, sizeof(impl->vel_des));
    xSemaphoreGive(impl->mutex_cmd);

    return AGV_OK;
}

static int pid_set_curr_vel(AgvControlLawBase* base, const Twist2D* curr_vel) {
    if (!base || !curr_vel) return AGV_ERR_INVALID_ARG;
    CtrlPidImpl* impl = (CtrlPidImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    xSemaphoreTake(impl->mutex_vel, portMAX_DELAY);
    memcpy(&impl->vel_curr, curr_vel, sizeof(impl->vel_curr));
    xSemaphoreGive(impl->mutex_vel);

    return AGV_OK;
}

static int pid_get_ctrl_cmd(AgvControlLawBase* base, Twist2D* cmd_out) {
    if (!base || !cmd_out) return AGV_ERR_INVALID_ARG;
    CtrlPidImpl* impl = (CtrlPidImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;
    const AgvCtrlPidConfig* cfg = impl->cfg;

    Twist2D curr, des;
    xSemaphoreTake(impl->mutex_vel, portMAX_DELAY);
    xSemaphoreTake(impl->mutex_cmd, portMAX_DELAY);
    curr = impl->vel_curr;
    des = impl->vel_des;
    xSemaphoreGive(impl->mutex_cmd);
    xSemaphoreGive(impl->mutex_vel);

    TickType_t tick_now = xTaskGetTickCount();
    float dt = (float)(tick_now - impl->tick_last) * (float)portTICK_PERIOD_MS /
               1000.0f;

    Twist2D cmd;
    cmd.x = pid_virtual_acc(curr.x, des.x, dt, cfg->max_acc.x, &impl->pid_st_x,
                            &cfg->gain_x);
    cmd.y = pid_virtual_acc(curr.y, des.y, dt, cfg->max_acc.y, &impl->pid_st_y,
                            &cfg->gain_y);
    cmd.yaw = pid_virtual_acc(curr.yaw, des.yaw, dt, cfg->max_acc.yaw,
                              &impl->pid_st_yaw, &cfg->gain_yaw);

    // static int time = 0;
    // time++;
    // if (time == 25) {
    //     time = 0;
    //     LOG(base->name, "curr: %f %f %f", curr.x, curr.y, curr.yaw);
    //     LOG(base->name, "des: %f %f %f", des.x, des.y, des.yaw);
    //     LOG(base->name, "cmd: %f %f %f", cmd.x, cmd.y, cmd.yaw);
    // }

    impl->tick_last = tick_now;

    *cmd_out = cmd;

    return AGV_OK;
}

float pid_virtual_acc(float curr, float des, float dt, float max_acc,
                      PidState* pid_st, const PidConfig* gains) {
    if (dt <= 0) dt = 0.0001f;

    float e = des - curr;
    pid_st->e_integral += e * dt;
    const float I_MAX = 10.0f;
    if (pid_st->e_integral > I_MAX) pid_st->e_integral = I_MAX;
    if (pid_st->e_integral < -I_MAX) pid_st->e_integral = -I_MAX;

    float de_dt = (e - pid_st->e_prev) / dt;
    pid_st->e_prev = e;

    float a =
        gains->kp * e + gains->ki * pid_st->e_integral + gains->kd * de_dt;

    if (max_acc > 0.0f) {
        if (a > max_acc) a = max_acc;
        if (a < -max_acc) a = -max_acc;
    }

    float cmd = curr + a * dt;
    return cmd;
}