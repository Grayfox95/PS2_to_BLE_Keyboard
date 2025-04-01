#include <inttypes.h>
#include "freertos/queue.h"

#define PS2_CODE_DELAY_MS 200
#define INACTIVITY_TIMEOUT 120000000
#define COFFEE_TIMEOUT 60000000

typedef enum
{
  MAKE_CODE,
  BREAK_CODE,
  ACK_CODE,
  ERROR_CODE
} CODE_TYPE;

typedef enum
{
  PS2_RESULT_FAIL = -1,
  PS2_RESULT_WAITING = 0,
  PS2_RESULT_SUCCESS = 1,
  PS2_RESULT_ACK
} PS2_RESULT_E;

typedef enum
{
  PS2_NONE,
  PS2_IDLE,
  PS2_COMMUNICATION_INHIBITED,
  PS2_HOST_RTS,
  PS2_HOST_TRANSMISSION,
  PS2_HOST_RETRANSMISSION,
  PS2_HOST_RECEPTION
}PS2_STATE_TYPE;

typedef enum
{
  PS2_TX_FAILED = -2,
  PS2_TX_ABORT = -1,
  PS2_TX_COMPLETE = 0,
}PS2_TX_EVENT_TYPE;

typedef struct
{
    uint8_t value;
    int64_t time;
}data_bit_t;


typedef struct
{
    CODE_TYPE type;
    bool extended_f;
    uint8_t code;
}ps2_code_t;

extern QueueHandle_t ps2_tx_code_queue; // Contains each ps2 code arrived from keyboard
extern QueueHandle_t ps2_led_code_queue;

void ps2_data_task(void* arg);
void ps2_update_led(uint8_t ps2_led_status);
