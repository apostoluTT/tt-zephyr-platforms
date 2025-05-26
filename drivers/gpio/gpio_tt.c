/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <errno.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_tt);

#define GPIOS_PER_REG 16

#define RESET_UNIT_GPIO_PAD_TRIEN_CNTL_REG_ADDR  0x800301A0
#define RESET_UNIT_GPIO_PAD_RXEN_CNTL_REG_ADDR   0x800301AC
#define RESET_UNIT_GPIO_PAD_DATA_REG_ADDR        0x800301B4

#define RESET_UNIT_GPIO2_PAD_TRIEN_CNTL_REG_ADDR 0x80030240
#define RESET_UNIT_GPIO2_PAD_RXEN_CNTL_REG_ADDR  0x8003025C
#define RESET_UNIT_GPIO2_PAD_DATA_REG_ADDR       0x80030254

#define RESET_UNIT_GPIO3_PAD_TRIEN_CNTL_REG_ADDR 0x80030580
#define RESET_UNIT_GPIO3_PAD_RXEN_CNTL_REG_ADDR  0x8003058C
#define RESET_UNIT_GPIO3_PAD_DATA_REG_ADDR       0x80030594

#define RESET_UNIT_GPIO4_PAD_TRIEN_CNTL_REG_ADDR 0x800305A0
#define RESET_UNIT_GPIO4_PAD_RXEN_CNTL_REG_ADDR  0x800305AC
#define RESET_UNIT_GPIO4_PAD_DATA_REG_ADDR       0x800305B4

struct gpio_tt_config {
	const struct gpio_driver_config common;
};

struct gpio_tt_data {
	struct gpio_driver_data common;

	uint8_t port_index;

	uint32_t trien_addr;
	uint32_t rxen_addr;

	uint32_t pad_data_addr;
	gpio_port_pins_t output_enabled;

	struct k_spinlock lock;
};

static uint32_t read_reg(uint32_t addr)
{
	return *((uint32_t volatile *)addr);
}

static void write_reg(uint32_t addr, uint32_t val)
{
	*((uint32_t volatile *)addr) = val;
}

static uint32_t get_trien_address(uint32_t id)
{
	if (id < GPIOS_PER_REG) {
		return RESET_UNIT_GPIO_PAD_TRIEN_CNTL_REG_ADDR;
	} else if (id < GPIOS_PER_REG * 2) {
		return RESET_UNIT_GPIO2_PAD_TRIEN_CNTL_REG_ADDR;
	} else if (id < GPIOS_PER_REG * 3) {
		return RESET_UNIT_GPIO3_PAD_TRIEN_CNTL_REG_ADDR;
	} else {
		return RESET_UNIT_GPIO4_PAD_TRIEN_CNTL_REG_ADDR;
	}
}

static uint32_t get_rxen_address(uint32_t id)
{
	if (id < GPIOS_PER_REG) {
		return RESET_UNIT_GPIO_PAD_RXEN_CNTL_REG_ADDR;
	} else if (id < GPIOS_PER_REG * 2) {
		return RESET_UNIT_GPIO2_PAD_RXEN_CNTL_REG_ADDR;
	} else if (id < GPIOS_PER_REG * 3) {
		return RESET_UNIT_GPIO3_PAD_RXEN_CNTL_REG_ADDR;
	} else {
		return RESET_UNIT_GPIO4_PAD_RXEN_CNTL_REG_ADDR;
	}
}

static uint32_t get_pad_data_address(uint32_t id)
{
	if (id < GPIOS_PER_REG) {
		return RESET_UNIT_GPIO_PAD_DATA_REG_ADDR;
	} else if (id < GPIOS_PER_REG * 2) {
		return RESET_UNIT_GPIO2_PAD_DATA_REG_ADDR;
	} else if (id < GPIOS_PER_REG * 3) {
		return RESET_UNIT_GPIO3_PAD_DATA_REG_ADDR;
	} else {
		return RESET_UNIT_GPIO4_PAD_DATA_REG_ADDR;
	}
}

static int gpio_tt_pin_configure(const struct device *port, gpio_pin_t pin,
								 pio_flags_t flags)
{
    return -ENOSYS;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_tt_pin_get_config(const struct device *port, gpio_pin_t pin,
				    			  gpio_flags_t *out_flags)
{
    return -ENOSYS;
}
#endif /* CONFIG_GPIO_GET_CONFIG */

static int gpio_tt_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
    return -ENOSYS;
}

static int gpio_tt_port_set_masked_raw(const struct device *port,
				      				   gpio_port_pins_t pins)
{
    return -ENOSYS;
}

static int gpio_tt_port_set_bits_raw(const struct device *port,
				      				 gpio_port_pins_t pins)
{
	struct gpio_tt_data *data = (struct gpio_tt_data *)port->data;

	uint32_t reg;

	switch (data->port_index) {
	case 0: reg = RESET_UNIT_GPIO_PAD_DATA_REG_ADDR; break;
	case 1: reg = RESET_UNIT_GPIO2_PAD_DATA_REG_ADDR; break;
	case 2: reg = RESET_UNIT_GPIO3_PAD_DATA_REG_ADDR; break;
	case 3: reg = RESET_UNIT_GPIO4_PAD_DATA_REG_ADDR; break;
	default: return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	pins &= data->output_enabled;
	
	uint32_t val = sys_read32(reg);
	val |= pins;
	
	sys_write32(val, reg);

	k_spin_unlock(&data->lock, key);

    return 0;
}

static int gpio_tt_port_clear_bits_raw(const struct device *port,
									   gpio_port_pins_t pins)
{
    return -ENOSYS;
}

static int gpio_tt_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
    return -ENOSYS;
}

static int gpio_tt_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
										   enum gpio_int_mode mode,
										   enum gpio_int_trig trig)
{
    return -ENOSYS;
}

static int gpio_tt_manage_callback(const struct device *port,
				    			   struct gpio_callback *cb, bool set)
{
    return -ENOSYS;
}

static int gpio_tt_get_pending_int(const struct device *dev)
{
    return -ENOSYS;
}

#ifdef CONFIG_GPIO_GET_DIRECTION
static int gpio_tt_port_get_direction(const struct device *port, gpio_port_pins_t map,
									  gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
    return -ENOSYS;
}
#endif /* CONFIG_GPIO_GET_DIRECTION */

static DEVICE_API(gpio, gpio_tt_driver) = {
	.pin_configure = gpio_tt_pin_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_tt_pin_get_config,
#endif /* CONFIG_GPIO_GET_CONFIG */
	.port_get_raw = gpio_tt_port_get_raw,
	.port_set_masked_raw = gpio_tt_port_set_masked_raw,
	.port_set_bits_raw = gpio_tt_port_set_bits_raw,
	.port_clear_bits_raw = gpio_tt_port_clear_bits_raw,
	.port_toggle_bits = gpio_tt_port_toggle_bits,
	.pin_interrupt_configure = gpio_tt_pin_interrupt_configure,
	.manage_callback = gpio_tt_manage_callback,
	.get_pending_int = gpio_tt_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_tt_port_get_direction,
#endif /* CONFIG_GPIO_GET_DIRECTION */
};

static int gpio_tt_init(const struct device *dev)
{
	return -ENOSYS;
}

#define DEFINE_gpio_tt(_num)						\
									\
	static gpio_flags_t						\
		gpio_tt_flags_##_num[DT_INST_PROP(_num, ngpios)];	\
									\
	static const struct gpio_tt_config gpio_tt_config_##_num = {\
		.common = {						\
			.port_pin_mask =				\
				GPIO_PORT_PIN_MASK_FROM_DT_INST(_num),	\
		},							\
		.num_pins = DT_INST_PROP(_num, ngpios),			\
	};								\
	BUILD_ASSERT(							\
		DT_INST_PROP(_num, ngpios) <= GPIO_MAX_PINS_PER_PORT,	\
		"Too many ngpios");					\
									\
	static struct gpio_tt_data gpio_tt_data_##_num = {		\
		.flags = gpio_tt_flags_##_num,			\
	};								\
									\
	PM_DEVICE_DT_INST_DEFINE(_num, gpio_tt_pm_device_pm_action);	\
									\
	DEVICE_DT_INST_DEFINE(_num, gpio_tt_init,			\
			    PM_DEVICE_DT_INST_GET(_num),		\
			    &gpio_tt_data_##_num,			\
			    &gpio_tt_config_##_num, POST_KERNEL,	\
			    CONFIG_GPIO_INIT_PRIORITY,			\
			    &gpio_tt_driver);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_gpio_tt)
