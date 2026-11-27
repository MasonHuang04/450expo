#include "app_state.h"

void app_controller_init(AppController *controller)
{
    if (controller == 0) return;
    controller->state = APP_STATE_IDLE;
    controller->frame_count = 0;
    controller->error_count = 0;
}

void app_controller_step(AppController *controller,
                         int rx_ready, int rx_error)
{
    if (controller == 0) return;
    if (rx_error) {
        controller->error_count++;
        controller->state = APP_STATE_ERROR;
        return;
    }
    switch (controller->state) {
    case APP_STATE_IDLE:
        controller->state = APP_STATE_ARM_RX;
        break;
    case APP_STATE_ARM_RX:
        controller->state = APP_STATE_WAIT_RX;
        break;
    case APP_STATE_WAIT_RX:
        if (rx_ready) controller->state = APP_STATE_PROCESS_RX;
        break;
    case APP_STATE_PROCESS_RX:
        controller->state = APP_STATE_REPORT;
        break;
    case APP_STATE_REPORT:
        controller->frame_count++;
        controller->state = APP_STATE_IDLE;
        break;
    case APP_STATE_ERROR:
        break;
    default:
        controller->state = APP_STATE_ERROR;
        break;
    }
}
