#include "ecu.h"
#include "mcal.h"
#include <sys/types.h>
#include "dtc_codes_bsw.h"
#include <unistd.h>

/**
 *  @brief function get by console the status and number of the PIN set it.
 *  @return SUCCESS(0), FAIL(1)
 */

uint8_t read_console(void)
{
    uint8_t pin = 0;
    uint8_t status = 0;
    uint8_t ret = ECU_SUCCESS;
    if (read_pint_status(&pin, &status) == ECU_FAIL)
    {
        REPORT_ERROR("read_pin_status FAIL\n", DTC_READ_PIN_FAIL);
        ret = ECU_FAIL;
    }

    if (ret == ECU_SUCCESS)
    {
        if (set_pin_status(status, pin) == ECU_FAIL)
        {
            REPORT_ERROR("read_console.set_pin_status FAIL\n", DTC_READ_CONSOLE_FAIL);
            ret = ECU_FAIL;
        }
    }

    return ret;
}
