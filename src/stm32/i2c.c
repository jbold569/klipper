// I2C functions on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_STM32F1
#include "board/misc.h" // timer_is_before
#include "command.h" // shutdown
#include "gpio.h" // i2c_setup
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown
#include "board/irq.h" //irq_disable
#include "i2ccmds.h"   // I2C_BUS_SUCCESS

// The timeout for read/write transactions
#define I2C_TIMEOUT_US 5000
// What portion of the timeout to reserve at the start of a transaction
// in case we need to kick-start I2C in some way.
#define I2C_TIMEOUT_START_ERRATA_US 5000

struct i2c_info {
    I2C_TypeDef *i2c;
    uint8_t scl_pin, sda_pin;
};

DECL_ENUMERATION("i2c_bus", "i2c1", 0);
DECL_CONSTANT_STR("BUS_PINS_i2c1", "PB6,PB7");
DECL_ENUMERATION("i2c_bus", "i2c1a", 1);
DECL_CONSTANT_STR("BUS_PINS_i2c1a", "PB8,PB9");
DECL_ENUMERATION("i2c_bus", "i2c2", 2);
DECL_CONSTANT_STR("BUS_PINS_i2c2", "PB10,PB11");
#if CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
DECL_ENUMERATION("i2c_bus", "i2c3", 3);
DECL_CONSTANT_STR("BUS_PINS_i2c3", "PA8,PC9");
  #if CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4x5
DECL_ENUMERATION("i2c_bus", "i2c2a", 4);
DECL_CONSTANT_STR("BUS_PINS_i2c2a", "PH4,PH5");
DECL_ENUMERATION("i2c_bus", "i2c3a", 5);
DECL_CONSTANT_STR("BUS_PINS_i2c3a", "PH7,PH8");
DECL_ENUMERATION("i2c_bus", "i2c2_PF1_PF0", 6);
DECL_CONSTANT_STR("BUS_PINS_i2c2_PF1_PF0", "PF1,PF0");
  #endif
#endif

static const struct i2c_info i2c_bus[] = {
    { I2C1, GPIO('B', 6), GPIO('B', 7) },
    { I2C1, GPIO('B', 8), GPIO('B', 9) },
    { I2C2, GPIO('B', 10), GPIO('B', 11) },
#if CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
    { I2C3, GPIO('A', 8), GPIO('C', 9) },
  #if CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4x5
    { I2C2, GPIO('H', 4), GPIO('H', 5) },
    { I2C3, GPIO('H', 7), GPIO('H', 8) },
    { I2C2, GPIO('F', 1), GPIO('F', 0) },
  #endif
#endif
};

// Helper function to calculate configured I2C rate from registers
static uint32_t
i2c_get_configured_rate(I2C_TypeDef *i2c)
{
    uint32_t pclk = get_pclock_frequency((uint32_t)i2c);
    uint32_t ccr = i2c->CCR & 0xFFF;  // 12-bit CCR value
    uint32_t fast_mode = (i2c->CCR >> 15) & 1;  // F/S bit
    uint32_t duty_16_9 = (i2c->CCR >> 14) & 1;  // DUTY bit
    
    uint32_t rate;
    
    if (fast_mode) {
        // Fast mode
        if (duty_16_9) {
            // 16:9 duty cycle
            // f_scl = f_pclk1 / (25 × CCR)
            rate = pclk / (25 * ccr);
        } else {
            // 2:1 duty cycle
            // f_scl = f_pclk1 / (3 × CCR)
            rate = pclk / (3 * ccr);
        }
    } else {
        // Standard mode
        // f_scl = f_pclk1 / (2 × CCR)
        rate = pclk / (2 * ccr);
    }
    
    return rate;
}

static void
i2c_init(I2C_TypeDef *i2c, uint32_t rate)
{
    uint32_t pclk = get_pclock_frequency((uint32_t)i2c);
    uint32_t pclk_mhz = pclk / 1000000;
    
    i2c->CR2 = pclk_mhz;
    
    uint32_t ccr_val;
    uint32_t trise_val;
    
    // Standard mode: ≤100 kHz
    if (rate <= 100000) {
        ccr_val = pclk / (2 * rate);
        if (ccr_val < 4)
            ccr_val = 4;
        trise_val = pclk_mhz + 1;
        i2c->CCR = ccr_val & 0xFFF;
    }
    // Fast mode: 100 kHz < rate ≤ 400 kHz
    else if (rate <= 400000) {
        ccr_val = pclk / (3 * rate);
        if (ccr_val < 1)
            ccr_val = 1;
        trise_val = (pclk_mhz * 300) / 1000 + 1;
        i2c->CCR = (1 << 15) | (ccr_val & 0xFFF);
    }
    // Rate too high - cap at 400 kHz
    else {
        ccr_val = pclk / (3 * 400000);
        if (ccr_val < 1)
            ccr_val = 1;
        trise_val = (pclk_mhz * 300) / 1000 + 1;
        i2c->CCR = (1 << 15) | (ccr_val & 0xFFF);
    }
    
    i2c->TRISE = trise_val & 0x3F;
    i2c->CR1 = I2C_CR1_PE;
}

// Work around stm2 errata causing busy bit to be stuck
// STM32F103xC/D/E Errata Sheet (ES0340), Section 2.9.7:
// "I2C analog filter may provide wrong value, locking BUSY flag and 
//  preventing master mode entry"
// https://www.st.com/resource/en/errata_sheet/es0340-stm32f101xcde-stm32f103xcde-device-errata-stmicroelectronics.pdf
static int
i2c_stm32f1_busy_errata(I2C_TypeDef *i2c, uint32_t rate)
{
    if (!CONFIG_MACH_STM32F1)
        return 0;

    const struct i2c_info *ii =
      container_of((I2C_TypeDef * const *)i2c, struct i2c_info, i2c);

    // Step 1: Disable I2C peripheral
    i2c->CR1 &= ~I2C_CR1_PE;
    
    // Clear status registers
    (void)i2c->SR1;
    (void)i2c->SR2;

    struct gpio_out scl_gpio = {
      .regs = digital_regs[GPIO2PORT(ii->scl_pin)],
      .bit = GPIO2BIT(ii->scl_pin)
    };

    struct gpio_out sda_gpio = {
      .regs = digital_regs[GPIO2PORT(ii->sda_pin)],
      .bit = GPIO2BIT(ii->sda_pin)
    };

    GPIO_TypeDef *scl_regs = digital_regs[GPIO2PORT(ii->scl_pin)];
    GPIO_TypeDef *sda_regs = digital_regs[GPIO2PORT(ii->sda_pin)];

    // Step 2: Configure GPIO as Output Open-Drain, High
    gpio_peripheral(ii->scl_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
    gpio_peripheral(ii->sda_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);

    // Step 3: Check SCL and SDA High level
    uint8_t scl = !!(scl_regs->IDR & (1 << GPIO2BIT(ii->scl_pin)));
    uint8_t sda = !!(sda_regs->IDR & (1 << GPIO2BIT(ii->sda_pin)));
    if (!scl || !sda) {
        output("I2C errata: Step 3 failed - SCL=%u SDA=%u expected SCL=1 SDA=1", scl, sda);
        return -1;
    }

    // Step 4: Set SDA Low
    gpio_out_write(sda_gpio, 0);
    
    // Step 5: Check SDA Low level
    sda = !!(sda_regs->IDR & (1 << GPIO2BIT(ii->sda_pin)));
    if (sda) {
        output("I2C errata: Step 5 failed - SDA=%u expected SDA=0", sda);
        return -2;
    }

    // Step 6: Set SCL Low
    gpio_out_write(scl_gpio, 0);
    
    // Step 7: Check SCL Low level
    scl = !!(scl_regs->IDR & (1 << GPIO2BIT(ii->scl_pin)));
    if (scl) {
        output("I2C errata: Step 7 failed - SCL=%u expected SCL=0", scl);
        return -3;
    }

    // Step 8: Set SCL High
    gpio_out_write(scl_gpio, 1);
    
    // Step 9: Check SCL High level
    scl = !!(scl_regs->IDR & (1 << GPIO2BIT(ii->scl_pin)));
    if (!scl) {
        output("I2C errata: Step 9 failed - SCL=%u expected SCL=1", scl);
        return -4;
    }

    // Step 10: Set SDA High
    gpio_out_write(sda_gpio, 1);
    
    // Step 11: Check SDA High level
    sda = !!(sda_regs->IDR & (1 << GPIO2BIT(ii->sda_pin)));
    if (!sda) {
        output("I2C errata: Step 11 failed - SDA=%u expected SDA=1", sda);
        return -5;
    }

    // Step 12: Configure as Alternate Function Open-Drain
    gpio_peripheral(ii->scl_pin, GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN, 1);
    gpio_peripheral(ii->sda_pin, GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN, 1);

    // Step 13: Set SWRST bit
    i2c->CR1 = I2C_CR1_SWRST;

    // Step 14: Clear SWRST bit
    i2c->CR1 = 0;

    // Step 15: Re-initialize I2C peripheral
    i2c_init(i2c, rate);
    
    return 0;
}

struct i2c_config
i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr)
{
    // Lookup requested i2c bus
    if (bus >= ARRAY_SIZE(i2c_bus))
        shutdown("Unsupported i2c bus");
    const struct i2c_info *ii = &i2c_bus[bus];
    I2C_TypeDef *i2c = ii->i2c;

    if (!is_enabled_pclock((uint32_t)i2c)) {
        // Enable i2c clock and gpio
        enable_pclock((uint32_t)i2c);
        i2c_stm32f1_busy_errata(i2c, rate);
        gpio_peripheral(ii->scl_pin, GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN, 1);
        gpio_peripheral(ii->sda_pin, GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN, 1);
        i2c->CR1 = I2C_CR1_SWRST;
        i2c->CR1 = 0;

        // Set rate frequency and enable
        i2c_init(i2c, rate);
    }

    return (struct i2c_config){ .i2c=i2c, .addr=addr<<1 };
}

static int
i2c_wait(I2C_TypeDef *i2c, uint32_t set, uint32_t clear, uint32_t timeout)
{
    for (;;) {
        uint32_t sr1 = i2c->SR1, sr2 = i2c->SR2;

        if ((sr1 & set) == set && (sr1 & clear) == 0)
            return I2C_BUS_SUCCESS;

        if (sr1 & I2C_SR1_AF)
            return I2C_BUS_NACK;

        if (timer_is_before(timer_read_time(), timeout))
            continue;

        if (sr2 & I2C_SR2_BUSY)
            return I2C_BUS_BUSY;

        if (sr1 & I2C_SR1_BERR)
            return I2C_BUS_ERR;

        return I2C_BUS_TIMEOUT;
    }
}

static int
i2c_start(I2C_TypeDef *i2c, uint8_t addr, uint8_t xfer_len,
          uint32_t timeout)
{
    int ret = 0;
    int retries = 2;
    uint32_t start_timeout = timeout - I2C_TIMEOUT_START_ERRATA_US;

restart:
    i2c->CR1 = I2C_CR1_START | I2C_CR1_PE;
    ret = i2c_wait(i2c, I2C_SR1_SB, 0, start_timeout);

    // On this chip, there's a condition where the I2C
    // lines can get wedged. Try to resolve this at the start
    // of a transaction.
    if (CONFIG_MACH_STM32F1 && ret == I2C_BUS_BUSY) {
        if (retries--) {
            uint32_t rate = i2c_get_configured_rate(i2c);
            if (i2c_stm32f1_busy_errata(i2c, rate) != 0) {
                output("I2C: Errata workaround attempt (%i) FAILED", retries+1);
            }
            // reset the original deadline
            start_timeout = timeout;
            goto restart;
        }
    }

    if (ret != I2C_BUS_SUCCESS)
        return ret;

    i2c->DR = addr;
    if (addr & 0x01)
        i2c->CR1 |= I2C_CR1_ACK;

    ret = i2c_wait(i2c, I2C_SR1_ADDR, 0, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    irqstatus_t flag = irq_save();
    uint32_t sr2 = i2c->SR2;
    if (addr & 0x01 && xfer_len == 1)
        i2c->CR1 = I2C_CR1_STOP | I2C_CR1_PE;
    irq_restore(flag);
    if (!(sr2 & I2C_SR2_MSL))
        shutdown("Failed to send i2c addr");
    return ret;
}

static void
i2c_send_byte(I2C_TypeDef *i2c, uint8_t b, uint32_t timeout, int *error)
{
    i2c->DR = b;
    *error = i2c_wait(i2c, I2C_SR1_TXE, 0, timeout);
}

static uint8_t
i2c_read_byte(I2C_TypeDef *i2c, uint32_t timeout, uint8_t remaining, int *error)
{
    *error = i2c_wait(i2c, I2C_SR1_RXNE, 0, timeout);
    if (*error != I2C_BUS_SUCCESS)
        return 0;

    irqstatus_t flag = irq_save();
    uint8_t b = i2c->DR;
    if (remaining == 1)
        i2c->CR1 = I2C_CR1_STOP | I2C_CR1_PE;
    irq_restore(flag);
    return b;
}

static void
i2c_stop(I2C_TypeDef *i2c, uint32_t timeout)
{
    i2c->CR1 = I2C_CR1_STOP | I2C_CR1_PE;
    i2c_wait(i2c, 0, I2C_SR1_TXE, timeout);
}

int
i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write)
{
    int ret = 0;
    I2C_TypeDef *i2c = config.i2c;
    uint32_t timeout = timer_read_time() + timer_from_us(I2C_TIMEOUT_US);

    ret = i2c_start(i2c, config.addr, write_len, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;
    while (write_len--) {
        i2c_send_byte(i2c, *write++, timeout, &ret);
        if (ret != I2C_BUS_SUCCESS)
            break;
    }
    i2c_stop(i2c, timeout);
    return ret;
}

int
i2c_read(struct i2c_config config, uint8_t reg_len, uint8_t *reg
         , uint8_t read_len, uint8_t *read)
{
    int ret = 0;
    I2C_TypeDef *i2c = config.i2c;
    uint32_t timeout = timer_read_time() + timer_from_us(I2C_TIMEOUT_US);
    uint8_t addr = config.addr | 0x01;

    if (reg_len) {
        // write the register
        ret = i2c_start(i2c, config.addr, reg_len, timeout);
        if (ret != I2C_BUS_SUCCESS)
            return ret;
        while (reg_len--) {
            i2c_send_byte(i2c, *reg++, timeout, &ret);
            if (ret != I2C_BUS_SUCCESS)
                break;
        }
        if (ret != I2C_BUS_SUCCESS)
            return ret;
    }
    // start/re-start and read data
    ret = i2c_start(i2c, addr, read_len, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;
    while (read_len--) {
        *read++ = i2c_read_byte(i2c, timeout, read_len, &ret);
        if (ret != I2C_BUS_SUCCESS)
            break;
    }
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    return i2c_wait(i2c, 0, I2C_SR1_RXNE, timeout);
}
