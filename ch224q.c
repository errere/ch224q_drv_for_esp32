#include "ch224q.h"
#include "esp_log.h"

static const char TAG[] = "ch224";

// request voltage
#define CH224_REQUEST_VOLTAGE_5V (0)
#define CH224_REQUEST_VOLTAGE_9V (1)
#define CH224_REQUEST_VOLTAGE_12V (2)
#define CH224_REQUEST_VOLTAGE_15V (3)
#define CH224_REQUEST_VOLTAGE_20V (4)
#define CH224_REQUEST_VOLTAGE_28V (5)
#define CH224_REQUEST_VOLTAGE_PPS (6)
#define CH224_REQUEST_VOLTAGE_AVS (7)

#define CH224_RETURN_ON_FALSE(x, ret, xtag, msg, ...) \
    do                                                \
    {                                                 \
        if (!(x))                                     \
        {                                             \
            ESP_LOGE(xtag, msg, ##__VA_ARGS__);       \
            return ret;                               \
        }                                             \
    } while (0)

static SemaphoreHandle_t *pSemaphoreIICFree;
static ch224q_i2cdev_handle_t *i2c_handle;

static esp_err_t ch224q_write_reg(uint8_t addr, uint8_t dat)
{
    esp_err_t ret = ESP_OK;
    if (pSemaphoreIICFree != 0)
    {
        xSemaphoreTake(*pSemaphoreIICFree, portMAX_DELAY);
    }

    uint8_t cmdbuf[2] = {addr, dat};

#ifdef CH224Q_USE_SW_IIC
    uint8_t io_ret = swi2c_write_operation(i2c_handle, 0x22, cmdbuf, 2);
    if (io_ret)
        ret = ESP_FAIL;
#else
    ret = i2c_master_transmit(*i2c_handle, cmdbuf, 2, -1);
#endif // CH224Q_USE_SW_IIC

    if (pSemaphoreIICFree != 0)
    {
        xSemaphoreGive(*pSemaphoreIICFree);
    }

    return ret;
} // ch224q_write_reg

static esp_err_t ch224q_read_reg(uint8_t addr, uint8_t *dat)
{
    esp_err_t ret = ESP_OK;
    if (pSemaphoreIICFree != 0)
    {
        xSemaphoreTake(*pSemaphoreIICFree, portMAX_DELAY);
    }

#ifdef CH224Q_USE_SW_IIC
    uint8_t io_ret = swi2c_read_operation(i2c_handle, 0x22, &addr, 1, dat, 1);
    if (io_ret)
        ret = ESP_FAIL;
#else
    ret = i2c_master_transmit_receive(*i2c_handle, &addr, 1, dat, 1, -1);
#endif // CH224Q_USE_SW_IIC

    if (pSemaphoreIICFree != 0)
    {
        xSemaphoreGive(*pSemaphoreIICFree);
    }

    return ret;
} // ch224q_read_reg

static esp_err_t ch224q_read_regs(uint8_t addr, uint8_t *dat, uint8_t len)
{
    esp_err_t ret = ESP_OK;
    if (pSemaphoreIICFree != 0)
    {
        xSemaphoreTake(*pSemaphoreIICFree, portMAX_DELAY);
    }

#ifdef CH224Q_USE_SW_IIC
    uint8_t io_ret = swi2c_read_operation(i2c_handle, 0x22, &addr, 1, dat, len);
    if (io_ret)
        ret = ESP_FAIL;
#else
    ret = i2c_master_transmit_receive(*i2c_handle, &addr, 1, dat, len, -1);
#endif // CH224Q_USE_SW_IIC

    if (pSemaphoreIICFree != 0)
    {
        xSemaphoreGive(*pSemaphoreIICFree);
    }

    return ret;
} // ch224q_read_regs

/*======================================================================================*/

/**
 * @brief get handshake status
 * @param status [out]handshake protocol
 * @note ch224q reg 0x09
 * @note use seq epr->spr->qc3->qc2->bc
 * @return CH224_OK if ok
 */
ch224_status_t ch224_get_handshake_status(ch224_protocol_t *status)
{
    uint8_t reg;
    esp_err_t io_ret = ch224q_read_reg(0x09, &reg);

    if (reg & CH224_ACTIVE_UCPD_EPR)
    {
        *status = CH224_PT_UCPD_EPR;
    }
    else if (reg & CH224_ACTIVE_UCPD_SPR)
    {
        *status = CH224_PT_UCPD;
    }
    else if (reg & CH224_ACTIVE_QC3)
    {
        *status = CH224_PT_QC3;
    }
    else if (reg & CH224_ACTIVE_QC2)
    {
        *status = CH224_PT_QC2;
    }
    else if (reg & CH224_ACTIVE_BC)
    {
        *status = CH224_PT_BC;
    }
    else
    {
        *status = CH224_PT_NONE;
    }
    return (io_ret == ESP_OK) ? ESP_OK : ESP_FAIL;
} // ch224_get_handshake_status

/**
 * @brief set request voltage
 * @param mv [in]requested voltage in mv
 * @param flag [in]function flag
 * @note ch224q reg 0x0a,0x51,0x52,0x53 , avs->pps->std
 * @return CH224_OK if ok
 */
ch224_status_t ch224_request_voltage(uint16_t mv, uint8_t flag)
{
    // hv
    if (mv > 20 * 1000)
    {
        // hv
        if (!(flag & CH224_FLAG_ENABLE_REQUEST_HV))
        {
            ESP_LOGW(TAG, "hv not enable");
            return CH224_ERR_VOLTAGE_NOT_SUPPORT;
        } // CH224_FLAG_ENABLE_REQUEST_HV
    } // 20*1000
    // too hv
    if (mv > 28 * 1000)
    {
        ESP_LOGE(TAG, "voltage higher than 28V(%dmV)", mv);
        return CH224_ERR_ARG;
    }
    // request
    if ((flag & CH224_FLAG_REQUEST_USE_AVS))
    {
        ESP_LOGI(TAG, "avs %dmV", mv);
        ESP_LOGE(TAG, "avs has bug,disable it");
        return CH224_ERR_VOLTAGE_NOT_SUPPORT;
        // request avs(0x51,0x52)
        mv = mv / 100;
        uint8_t reg = mv & 0xff; // lsb8
        //  0x51、0x52：AVS 电压配置寄存器高八位、AVS 电压配置寄存器低八位
        // 电压配置寄存器 0~7 为请求电压低 8 位，8~14 位为请求电压高 7 位，最高位为使能位。
        // 配置时先写入低八位，后将高 7 位和使能位（置 1）一并写入，首次申请 AVS 时先配置电压，
        // 后将电压控制寄存器配置为 AVS 模式，后续调压直接修改 AVS 电压配置寄存器即可。
        CH224_RETURN_ON_FALSE((ch224q_write_reg(0x52, reg) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x52);
        reg = (mv >> 8) & 0xff; // hsb8 + en
        reg = reg | 0x80;
        CH224_RETURN_ON_FALSE((ch224q_write_reg(0x51, reg) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x51);
        // set avs mode
        CH224_RETURN_ON_FALSE((ch224q_write_reg(0x0a, CH224_REQUEST_VOLTAGE_AVS) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);
    } // avs
    else if (flag & CH224_FLAG_REQUEST_USE_PPS)
    {
        ESP_LOGI(TAG, "pps %dmV", mv);
        // 0x53：PPS 电压配置寄存器
        // 首次申请 PPS 时先配置电压，然后将电压控制寄存器配置为 PPS 模式，后续调压直接修改 PPS 电压配置寄存器即可。
        mv = mv / 100;
        uint8_t reg = mv & 0xff;
        CH224_RETURN_ON_FALSE((ch224q_write_reg(0x53, reg) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x51);
        // set pps mode
        CH224_RETURN_ON_FALSE((ch224q_write_reg(0x0a, CH224_REQUEST_VOLTAGE_PPS) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);

    } // pps
    else
    {
        ESP_LOGI(TAG, "std %dmV", mv);
        switch (mv)
        {
        case 5000:
            // 5v
            CH224_RETURN_ON_FALSE((ch224q_write_reg(0x0a, CH224_REQUEST_VOLTAGE_5V) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);
            break;
        case 9000:
            // 9v
            CH224_RETURN_ON_FALSE((ch224q_write_reg(0x0a, CH224_REQUEST_VOLTAGE_9V) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);
            break;
        case 12000:
            // 12v
            CH224_RETURN_ON_FALSE((ch224q_write_reg(0x0a, CH224_REQUEST_VOLTAGE_12V) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);
            break;
        case 15000:
            // 15v
            CH224_RETURN_ON_FALSE((ch224q_write_reg(0x0a, CH224_REQUEST_VOLTAGE_15V) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);
            break;
        case 20000:
            // 20v
            CH224_RETURN_ON_FALSE((ch224q_write_reg(0x0a, CH224_REQUEST_VOLTAGE_20V) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);
            break;
        case 28000:
            // 28v
            CH224_RETURN_ON_FALSE((ch224q_write_reg(0x0a, CH224_REQUEST_VOLTAGE_28V) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);
            break;
        default:
            ESP_LOGE(TAG, "no standard voltage(%dmV) in standard mode", mv);
            return CH224_ERR_ARG;
            break;
        }
    } // standard

    return CH224_OK;
} // ch224_request_voltage

/**
 * @brief get requested current
 * @param ma [out]requested current in ma
 * @note ch224q reg 0x50
 * @return CH224_OK if ok
 */
ch224_status_t ch224_get_available_current(uint16_t *ma)
{
    uint8_t reg;
    esp_err_t io_ret = ch224q_read_reg(0x50, &reg);
    *ma = reg * 50;
    return (io_ret == ESP_OK) ? ESP_OK : ESP_FAIL;
} // ch224_get_available_current

/**
 * @brief get requested voltage
 * @param mv [out]requested current in ma
 * @param flag [out]pps or avs
 * @return CH224_OK if ok
 */
ch224_status_t ch224_get_request_voltage(uint16_t *mv, uint8_t *flag)
{
    uint8_t reg;
    *flag = 0;
    CH224_RETURN_ON_FALSE((ch224q_read_reg(0x0a, &reg) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x0a);
    if (reg == 6)
    {
        // pps
        CH224_RETURN_ON_FALSE((ch224q_read_reg(0x53, &reg) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x53);
        *mv = reg * 100;
        *flag = CH224_FLAG_REQUEST_USE_PPS;
    }
    else if (reg == 7)
    {
        // avs
        CH224_RETURN_ON_FALSE((ch224q_read_reg(0x52, &reg) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x52);
        *mv = reg;
        CH224_RETURN_ON_FALSE((ch224q_read_reg(0x53, &reg) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x53);
        *mv = ((uint16_t)(reg & 0x7f)) << 8;
        *mv = *mv * 100;
        *flag = CH224_FLAG_REQUEST_USE_AVS;
    }
    else
    {
        switch (reg)
        {
        case 0:
            // 5v
            *mv = 5000;
            break;
        case 1:
            // 9v
            *mv = 9000;
            break;
        case 2:
            // 12v
            *mv = 12000;
            break;
        case 3:
            // 15v
            *mv = 15000;
            break;
        case 4:
            // 20v
            *mv = 20000;
            break;
        case 5:
            // 28v
            *mv = 28000;
            break;
        default:
            *mv = 5000;
            ESP_LOGW(TAG, "unknown reg");
            return CH224_ERR_VOLTAGE_NOT_SUPPORT;
            break;
        }
    }
    return CH224_OK;
} // ch224_get_request_voltage

/**
 * @brief get pdo if in pdmode
 * @param buf [out]SRCCAP ,need a 96 bytes buffer
 * @note ch224q reg 0x60->0x8f
 * @return CH224_OK if ok , CH224_ERR_ARG if not in pd mode
 */
ch224_status_t ch224_get_pdo(uint8_t *buf)
{

    uint8_t reg;
    CH224_RETURN_ON_FALSE((ch224q_read_reg(0x09, &reg) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x09);

    if (!(reg & 0x18))
    {
        ESP_LOGW(TAG, "not in pd/pps mode");
        return CH224_ERR_VOLTAGE_NOT_SUPPORT;
    } // no pd

    CH224_RETURN_ON_FALSE((ch224q_read_regs(0x60, buf, 96) == ESP_OK), CH224_ERR_IO, TAG, "io err%d", 0x60);
    return CH224_OK;

} // ch224_get_pdo

void ch224q_debug()
{
    uint8_t x;
    ch224q_read_reg(0x0a, &x);

    ESP_LOGI(TAG, "0a = %02x", x);
}

void ch224q_init(SemaphoreHandle_t *iic_free, ch224q_i2cdev_handle_t *handle)
{
    pSemaphoreIICFree = iic_free;
    i2c_handle = handle;
}

// eof