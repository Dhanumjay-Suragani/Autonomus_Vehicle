/*
 * ultrasonic.c
 *
 *  Created on: Oct 5, 2026
 *      Author: dhanu
 */


/*
 * ultrasonic.c
 * HC-SR04 ultrasonic sensor
 *
 * TRIG = PC12
 * ECHO = PE5
 */

#include "ultrasonic.h"

#define ULTRASONIC_TIMEOUT_US    30000U

/* ---------------------------------------------------------
 * DWT microsecond timing
 * --------------------------------------------------------- */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;

    uint32_t ticks =
        (HAL_RCC_GetHCLKFreq() / 1000000U) * us;

    while ((DWT->CYCCNT - start) < ticks)
    {
        /* wait */
    }
}

/* ---------------------------------------------------------
 * Initialize ultrasonic
 * --------------------------------------------------------- */

void Ultrasonic_Init(void)
{
    DWT_Init();

    HAL_GPIO_WritePin(
        US_TRIG_GPIO_Port,
        US_TRIG_Pin,
        GPIO_PIN_RESET
    );
}

/* ---------------------------------------------------------
 * Read distance
 *
 * Returns:
 *   distance in cm
 *
 * Returns -1.0f if ECHO timeout occurs.
 * --------------------------------------------------------- */

float Ultrasonic_ReadDistanceCm(void)
{
    uint32_t start;
    uint32_t timeout_start;
    uint32_t pulse_cycles;
    uint32_t pulse_us;

    /* Make sure TRIG starts LOW */
    HAL_GPIO_WritePin(
        US_TRIG_GPIO_Port,
        US_TRIG_Pin,
        GPIO_PIN_RESET
    );

    Delay_us(2);

    /* 10 us trigger pulse */
    HAL_GPIO_WritePin(
        US_TRIG_GPIO_Port,
        US_TRIG_Pin,
        GPIO_PIN_SET
    );

    Delay_us(10);

    HAL_GPIO_WritePin(
        US_TRIG_GPIO_Port,
        US_TRIG_Pin,
        GPIO_PIN_RESET
    );

    /* -----------------------------------------------------
     * Wait for ECHO to become HIGH
     * ----------------------------------------------------- */

    timeout_start = DWT->CYCCNT;

    while (HAL_GPIO_ReadPin(
               US_ECHO_GPIO_Port,
               US_ECHO_Pin
           ) == GPIO_PIN_RESET)
    {
        uint32_t elapsed =
            DWT->CYCCNT - timeout_start;

        uint32_t timeout_cycles =
            (HAL_RCC_GetHCLKFreq() / 1000000U)
            * ULTRASONIC_TIMEOUT_US;

        if (elapsed > timeout_cycles)
        {
            return -1.0f;
        }
    }

    /* ECHO went HIGH */
    start = DWT->CYCCNT;

    /* -----------------------------------------------------
     * Measure ECHO HIGH pulse width
     * ----------------------------------------------------- */

    while (HAL_GPIO_ReadPin(
               US_ECHO_GPIO_Port,
               US_ECHO_Pin
           ) == GPIO_PIN_SET)
    {
        uint32_t elapsed =
            DWT->CYCCNT - start;

        uint32_t timeout_cycles =
            (HAL_RCC_GetHCLKFreq() / 1000000U)
            * ULTRASONIC_TIMEOUT_US;

        if (elapsed > timeout_cycles)
        {
            return -1.0f;
        }
    }

    pulse_cycles = DWT->CYCCNT - start;

    /* Convert CPU cycles to microseconds */
    pulse_us =
        pulse_cycles /
        (HAL_RCC_GetHCLKFreq() / 1000000U);

    /* Distance = time * speed / 2
     *
     * speed of sound ≈ 0.0343 cm/us
     */

    return ((float)pulse_us * 0.0343f) / 2.0f;
}
