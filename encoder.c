#include "encoder.h"
#include <ti/devices/msp/msp.h>

Encoder_Data_t g_Encoder = {0};

static volatile int32_t left_pulse_count = 0;
static volatile int32_t right_pulse_count = 0;

void Encoder_Init(void)
{
    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);
    DL_GPIO_enableInterrupt(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending_group = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    if (pending_group & DL_INTERRUPT_GROUP1_IIDX_GPIOB) {
        uint32_t gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);

        if (gpio_status & DL_GPIO_PIN_7) {
            uint8_t phase_a = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7);
            uint8_t phase_b = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9);

            if (phase_a != phase_b) {
                left_pulse_count--;
            } else {
                left_pulse_count++;
            }
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7);
        }

        if (gpio_status & DL_GPIO_PIN_6) {
            uint8_t phase_a = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6);
            uint8_t phase_b = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8);

            if (phase_a != phase_b) {
                right_pulse_count++;
            } else {
                right_pulse_count--;
            }
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6);
        }
    }
}

void Encoder_UpdateData_10ms(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    g_Encoder.speed_left = left_pulse_count;
    g_Encoder.speed_right = right_pulse_count;
    left_pulse_count = 0;
    right_pulse_count = 0;

    __set_PRIMASK(primask);

    g_Encoder.pulses_left += g_Encoder.speed_left;
    g_Encoder.pulses_right += g_Encoder.speed_right;

    float avg_pulses = (float)(g_Encoder.pulses_left + g_Encoder.pulses_right) / 2.0f;
    g_Encoder.distance_cm = avg_pulses * PULSE_TO_CM;
}

void Encoder_Clear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    g_Encoder.pulses_left = 0;
    g_Encoder.pulses_right = 0;
    g_Encoder.speed_left = 0;
    g_Encoder.speed_right = 0;
    g_Encoder.distance_cm = 0.0f;
    left_pulse_count = 0;
    right_pulse_count = 0;

    __set_PRIMASK(primask);
}
