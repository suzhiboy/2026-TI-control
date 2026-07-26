#include "electromagnet.h"

static bool electromagnet_on = false;

void Electromagnet_Init(void)
{
    /* GPIO is initialized by SYSCFG_DL_init().
     * Ensure the pin starts in the OFF (low) state. */
    DL_GPIO_clearPins(GPIO_ELECTROMAGNET_PORT, GPIO_ELECTROMAGNET_CTRL_PIN);
    electromagnet_on = false;
}

void Electromagnet_On(void)
{
    DL_GPIO_setPins(GPIO_ELECTROMAGNET_PORT, GPIO_ELECTROMAGNET_CTRL_PIN);
    electromagnet_on = true;
}

void Electromagnet_Off(void)
{
    DL_GPIO_clearPins(GPIO_ELECTROMAGNET_PORT, GPIO_ELECTROMAGNET_CTRL_PIN);
    electromagnet_on = false;
}

void Electromagnet_Set(bool state)
{
    if (state) {
        Electromagnet_On();
    } else {
        Electromagnet_Off();
    }
}

bool Electromagnet_GetState(void)
{
    return electromagnet_on;
}
