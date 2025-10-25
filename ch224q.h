#ifndef __WCH_CH224Q_H__
#define __WCH_CH224Q_H__

#define CH224Q_USE_SW_IIC

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

#ifdef CH224Q_USE_SW_IIC
#include "esp_sw_iic.h"
#else
#include <driver/i2c_master.h>
#endif

#ifdef CH224Q_USE_SW_IIC
typedef sw_iic_desc_t ch224q_i2cdev_handle_t;
#else
typedef i2c_master_dev_handle_t ch224q_i2cdev_handle_t;
#endif

// charger portcol active
#define CH224_ACTIVE_UCPD_EPR (1U << 4)
#define CH224_ACTIVE_UCPD_SPR (1U << 3)
#define CH224_ACTIVE_QC3 (1U << 2)
#define CH224_ACTIVE_QC2 (1U << 1)
#define CH224_ACTIVE_BC (1U)
#define CH224_ACTIVE_NONE (0U)

// voltage request flag
#define CH224_FLAG_NONE (0)
#define CH224_FLAG_ENABLE_REQUEST_HV (1U) // more than 20v
#define CH224_FLAG_REQUEST_USE_AVS (1U << 4)
#define CH224_FLAG_REQUEST_USE_PPS (1U << 5)

typedef enum
{
    CH224_OK = 0,
    CH224_ERR_IO,
    CH224_ERR_ARG,
    CH224_ERR_VOLTAGE_NOT_SUPPORT,
    CH224_MAX,
} ch224_status_t;

typedef enum
{
    CH224_PT_NONE = 0,
    CH224_PT_BC,
    CH224_PT_QC2,
    CH224_PT_QC3,
    CH224_PT_UCPD,
    CH224_PT_UCPD_EPR,
    CH224_PT_MAX,
} ch224_protocol_t;

ch224_status_t ch224_get_handshake_status(ch224_protocol_t *status);

ch224_status_t ch224_request_voltage(uint16_t mv, uint8_t flag);

ch224_status_t ch224_get_available_current(uint16_t *ma);

ch224_status_t ch224_get_request_voltage(uint16_t *mv, uint8_t *flag);

ch224_status_t ch224_get_pdo(uint8_t *buf);

void ch224q_debug();
void ch224q_init(SemaphoreHandle_t *iic_free, ch224q_i2cdev_handle_t *handle);

#endif