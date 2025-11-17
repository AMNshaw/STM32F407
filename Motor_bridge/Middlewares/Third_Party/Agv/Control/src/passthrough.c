#include <stdio.h>

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
    const AgvCtrlPassthroughConfig* cfg;

    Twist2D vel_des;
    Twist2D vel_curr;

    SemaphoreHandle_t mutex_cmd;
    SemaphoreHandle_t mutex_vel;
} CtrlPssthrghImpl;

static int Ctrl_passthrough_destroy(AgvControlLawBase* out);

static int passthrough_set_des_vel(AgvControlLawBase* base,
                                   const Twist2D* cmd_in);

static int passthrough_set_curr_vel(AgvControlLawBase* base,
                                    const Twist2D* curr_vel);

static int passthrough_get_ctrl_cmd(AgvControlLawBase* base, Twist2D* cmd_out);

/**
 * Private definitions
 */

int Ctrl_passthrough_create(AgvControlLawBase* out,
                            const AgvCtrlPassthroughConfig* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;

    CtrlPssthrghImpl* impl =
        (CtrlPssthrghImpl*)malloc(sizeof(CtrlPssthrghImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;
    impl->cfg = cfg;

    char* name = "Passthrough ctrl";

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

    out->name = name;
    out->impl = impl;
    out->set_des_vel = passthrough_set_des_vel;
    out->set_curr_vel = passthrough_set_curr_vel;
    out->get_ctrl_cmd = passthrough_get_ctrl_cmd;
    out->destroy = Ctrl_passthrough_destroy;

    LOG(out->name, "Control law module created");
    return AGV_OK;
}

static int Ctrl_passthrough_destroy(AgvControlLawBase* out) {
    if (!out) return AGV_OK;

    CtrlPssthrghImpl* impl =
        (CtrlPssthrghImpl*)malloc(sizeof(CtrlPssthrghImpl));
    if (impl) {
        if (impl->mutex_cmd) {
            vSemaphoreDelete(impl->mutex_cmd);
            impl->mutex_cmd = NULL;
        }

        free(impl);
    }

    out->impl = NULL;
    out->set_des_vel = NULL;

    return AGV_OK;
}

static int passthrough_set_des_vel(AgvControlLawBase* base,
                                   const Twist2D* cmd_in) {
    if (!base || !cmd_in) return AGV_ERR_INVALID_ARG;
    CtrlPssthrghImpl* impl = (CtrlPssthrghImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    xSemaphoreTake(impl->mutex_cmd, portMAX_DELAY);
    memcpy(&impl->vel_des, cmd_in, sizeof(impl->vel_des));
    xSemaphoreGive(impl->mutex_cmd);

    return AGV_OK;
}

static int passthrough_set_curr_vel(AgvControlLawBase* base,
                                    const Twist2D* curr_vel) {
    if (!base || !curr_vel) return AGV_ERR_INVALID_ARG;
    CtrlPssthrghImpl* impl = (CtrlPssthrghImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    xSemaphoreTake(impl->mutex_vel, portMAX_DELAY);
    memcpy(&impl->vel_curr, curr_vel, sizeof(impl->vel_curr));
    xSemaphoreGive(impl->mutex_vel);

    return AGV_OK;
}

static int passthrough_get_ctrl_cmd(AgvControlLawBase* base, Twist2D* cmd_out) {
    if (!base || !cmd_out) return AGV_ERR_INVALID_ARG;
    CtrlPssthrghImpl* impl = (CtrlPssthrghImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    Twist2D cmd_curr;
    xSemaphoreTake(impl->mutex_cmd, portMAX_DELAY);
    memcpy(&cmd_curr, &impl->vel_des, sizeof(impl->vel_des));
    xSemaphoreGive(impl->mutex_cmd);

    *cmd_out = cmd_curr;

    return AGV_OK;
}
