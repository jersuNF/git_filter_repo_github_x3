/*
 * Copyright (c) 2021 Nofence AS
 */

#include "charging.h"
#include <drivers/gpio.h>
#include <drivers/adc.h>

#define MODULE charging
#include <logging/log.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_CHARGING_LOG_LEVEL);

#define PIN_HIGH 1
#define PIN_LOW 0

#define CHARGING_DCDC_NODE DT_ALIAS(charging_dcdc)
#define CHARGING_DCDC_LABEL DT_GPIO_LABEL(CHARGING_DCDC_NODE, gpios)
#define CHARGING_DCDC_PIN DT_GPIO_PIN(CHARGING_DCDC_NODE, gpios)
#define CHARGING_DCDC_FLAGS DT_GPIO_FLAGS(CHARGING_DCDC_NODE, gpios)

#define CHARGING_LOAD_NODE DT_ALIAS(charging_load)
#define CHARGING_LOAD_LABEL DT_GPIO_LABEL(CHARGING_LOAD_NODE, gpios)
#define CHARGING_LOAD_PIN DT_GPIO_PIN(CHARGING_LOAD_NODE, gpios)
#define CHARGING_LOAD_FLAGS DT_GPIO_FLAGS(CHARGING_LOAD_NODE, gpios)

#define Vdcdc (205ul * 10000ul) //205 = 2,05V with resolution
#define CURRENT_SENSE_RESISTOR 39
#define CURRENT_SENSE_GAIN 50 //MAX9634FEKS+
#define MAX_CURRENT_VALUE_mA                                                   \
	((Vdcdc / CURRENT_SENSE_GAIN) /                                        \
	 CURRENT_SENSE_RESISTOR) //In mA -> for MAX9634TERS+ this value is 683mA

const struct device *charging_dcdc_dev;
const struct device *charging_load_dev;
const struct device *charging_adc_dev;

static bool charging_ok;
#define BUFFER_SIZE 6
static int16_t raw_data;

#define CHARGING_ADC_DEVICE_NAME DT_LABEL(DT_ALIAS(charging_current))
#define CHARGING_ADC_RESOLUTION 10
#define CHARGING_ADC_GAIN ADC_GAIN_1_6
#define CHARGING_ADC_REFERENCE ADC_REF_INTERNAL
#define CHARGING_ADC_ACQUISITION_TIME                                          \
	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define CHARGING_ADC_CHANNEL 4

// the channel configuration with channel not yet filled in
static struct adc_channel_cfg charging_channel_cfg = {
	.gain = CHARGING_ADC_GAIN,
	.reference = CHARGING_ADC_REFERENCE,
	.acquisition_time = CHARGING_ADC_ACQUISITION_TIME,
	.channel_id = 0, // gets set during init
	.differential = 0,
#if CONFIG_ADC_CONFIGURABLE_INPUTS
	.input_positive = 0, // gets set during init
#endif
};

// initialize the adc channel
int init_charging_adc(void)
{
	int ret;
	charging_adc_dev = device_get_binding(CHARGING_ADC_DEVICE_NAME);
	if (charging_adc_dev != NULL) {
		charging_channel_cfg.channel_id = CHARGING_ADC_CHANNEL;
#if CONFIG_ADC_CONFIGURABLE_INPUTS
		charging_channel_cfg.input_positive = CHARGING_ADC_CHANNEL + 1,
#endif
		ret = adc_channel_setup(charging_adc_dev,
					&charging_channel_cfg);
		if (ret != 0) {
			LOG_ERR("Setup of charging adc channel failed with code %d",
				ret);
			return -ENODEV;
		}
	}

	memset(&raw_data, 0, sizeof(raw_data));
	return 0;
}

static int charging_setup(const struct device *arg)
{
	int err = init_charging_adc();

	charging_ok = (err == 0);
	LOG_INF("Charging ADC setup %s", charging_ok ? "complete" : "error");
	return err;
}

int read_analog_charging_channel(void)
{
	const struct adc_sequence sequence = {
		.options = NULL, // extra samples and callback
		.channels = BIT(4), // bit mask of channels to read
		.buffer = &raw_data, // where to put samples read
		.buffer_size = sizeof(raw_data),
		.resolution = CHARGING_ADC_RESOLUTION, // desired resolution
		.oversampling = 0, // don't oversample
		.calibrate = false // don't calibrate
	};

	int32_t sample_value = -ENOENT;

	if (charging_ok) {
		int ret = adc_read(charging_adc_dev, &sequence);
		if (ret == 0) {
			sample_value = raw_data;
		}

		adc_raw_to_millivolts(adc_ref_internal(charging_adc_dev),
				      charging_channel_cfg.gain,
				      sequence.resolution, &sample_value);
	}

	return sample_value;
}

int init_charging_module(void)
{
	int err;
	charging_load_dev = device_get_binding(CHARGING_LOAD_LABEL);
	if (charging_load_dev == NULL) {
		LOG_ERR("Error get device %s", log_strdup(CHARGING_LOAD_LABEL));
		return -ENODEV;
	}
	/* Configure pin with flags */
	err = gpio_pin_configure(charging_load_dev, CHARGING_LOAD_PIN,
				 GPIO_OUTPUT_ACTIVE | CHARGING_LOAD_FLAGS);
	/* Set pin state to low */
	err = gpio_pin_set(charging_load_dev, CHARGING_LOAD_PIN, PIN_LOW);

	charging_dcdc_dev = device_get_binding(CHARGING_DCDC_LABEL);
	if (charging_dcdc_dev == NULL) {
		LOG_ERR("Error get device %s", log_strdup(CHARGING_DCDC_LABEL));
		return -ENODEV;
	}
	/* Configure pin with flags */
	err = gpio_pin_configure(charging_dcdc_dev, CHARGING_DCDC_PIN,
				 GPIO_OUTPUT_ACTIVE | CHARGING_DCDC_FLAGS);
	/* Set pin state to low */
	err = gpio_pin_set(charging_dcdc_dev, CHARGING_DCDC_PIN, PIN_LOW);

	return err;
}

int start_charging(void)
{
	int err;
	if (!device_is_ready(charging_load_dev)) {
		/* Not ready, do not use */
		LOG_ERR("CHARGING_LOAD_DEV not ready or proper initialized");
		return -ENODEV;
	}

	err = gpio_pin_set(charging_load_dev, CHARGING_LOAD_PIN, PIN_HIGH);
	if (err < 0) {
		LOG_ERR("Could not enable charging load pin");
		return err;
	}

	/* Wait to give switch time to enable */
	k_sleep(K_MSEC(50));

	if (!device_is_ready(charging_dcdc_dev)) {
		/* Not ready, do not use */
		LOG_ERR("CHARGING_DCDC_DEV not ready or proper initialized");
		return -ENODEV;
	}

	err = gpio_pin_set(charging_dcdc_dev, CHARGING_DCDC_PIN, PIN_HIGH);
	if (err < 0) {
		LOG_ERR("Could not enable charging dcdc pin");
		return err;
	}

	return 0;
}

int stop_charging(void)
{
	int err;
	if (!device_is_ready(charging_dcdc_dev)) {
		/* Not ready, do not use */
		LOG_ERR("CHARGING_DCDC_DEV not ready or proper initialized");
		return -ENODEV;
	}
	err = gpio_pin_set(charging_dcdc_dev, CHARGING_DCDC_PIN, PIN_LOW);
	if (err < 0) {
		LOG_ERR("Could not disable charging dcdc pin");
		return err;
	}
	/* Wait to give switch time to disable */
	k_sleep(K_MSEC(50));

	if (!device_is_ready(charging_load_dev)) {
		/* Not ready, do not use */
		LOG_ERR("CHARGING_LOAD_DEV not ready or proper initialized");
		return -ENODEV;
	}

	err = gpio_pin_set(charging_load_dev, CHARGING_LOAD_PIN, PIN_LOW);
	if (err < 0) {
		LOG_ERR("Could not disable charging load pin");
		return err;
	}

	return 0;
}

int charging_current(void)
{
	return 0;
}

/* Initialze battery setup on startup */
SYS_INIT(charging_setup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);