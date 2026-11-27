#ifndef APP_STATE_H
#define APP_STATE_H

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_ARM_RX,
    APP_STATE_WAIT_RX,
    APP_STATE_PROCESS_RX,
    APP_STATE_REPORT,
    APP_STATE_ERROR
} AppState;

typedef struct {
    AppState state;
    unsigned frame_count;
    unsigned error_count;
} AppController;

void app_controller_init(AppController *controller);
void app_controller_step(AppController *controller,
                         int rx_ready, int rx_error);

#endif
