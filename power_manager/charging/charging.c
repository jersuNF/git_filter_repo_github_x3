/*
 * Copyright (c) 2021 Nofence AS
 */

#include "charging.h"
#include "battery.h"

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

const struct device *charging_dcdc_dev;
const struct device *charging_load_dev;
const struct device *charging_adc_dev;

static bool charging_ok;
#define BUFFER_SIZE 6
static int16_t raw_data;

#define CHARGING_ADC_DEVICE_NAME DT_LABEL(DT_ALIAS(charging_current))
#define CHARGING_ADC_RESOLUTION 14
#define CHARGING_ADC_GAIN ADC_GAIN_1
#define CHARGING_ADC_REFERENCE ADC_REF_INTERNAL
#define CHARGING_ADC_ACQUISITION_TIME                                          \
	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define CHARGING_ADC_CHANNEL 4

#define CURRENT_SENSE_RESISTOR 0.039f // In Ohm
#define CURRENT_SENSE_GAIN 50.0f //MAX9634FEKS+
#define CURRENT_OFFSET 3.5f // mA

//***DO NOT CHANGE***
#define MAX_CURRENT_VALUE_mA                                                   \
	((Vdcdc / CURRENT_SENSE_GAIN) /                                        \
	 CURRENT_SENSE_RESISTOR) //In mA -> for MAX9634TERS+ this value is 683mA

/** @brief Moving average for current defined outside function */
MovAvg IchrgSMA;

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
		charging_channel_cfg.input_positive =
			SAADC_CH_PSELP_PSELP_AnalogInput0 +
			CHARGING_ADC_CHANNEL;
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

void init_current_moving_average(void)
{
	IchrgSMA.average = 0;
	IchrgSMA.N = 0;
	IchrgSMA.total = 0;
	IchrgSMA.MAX_SAMPLES = CONFIG_CURRENT_MOVING_AVERAGE_SAMPLES;
}

int charging_setup(void)
{
	int err = init_charging_adc();
	charging_ok = (err == 0);
	LOG_INF("Charging ADC setup %s", charging_ok ? "complete" : "error");
	init_current_moving_average();

	return err;
}

int read_analog_charging_channel(void)
{
	struct adc_sequence sequence = {
		.options = NULL, // extra samples and callback
		.channels = BIT(
			CHARGING_ADC_CHANNEL), // bit mask of channels to read
		.buffer = &raw_data, // where to put samples read
		.buffer_size = sizeof(raw_data),
		.resolution = CHARGING_ADC_RESOLUTION, // desired resolution
		.oversampling = 8,
		.calibrate = true // don't calibrate
	};

	int32_t sample_value;
	float result;

	if (charging_ok) {
		LOG_INF("Charging okay");
		int ret = adc_read(charging_adc_dev, &sequence);
		sequence.calibrate = false;
		if (ret == 0) {
			sample_value = raw_data;
			adc_raw_to_millivolts(
				adc_ref_internal(charging_adc_dev),
				charging_channel_cfg.gain, sequence.resolution,
				&sample_value);
			result = ((float)sample_value /
				  (CURRENT_SENSE_GAIN *
				   CURRENT_SENSE_RESISTOR)) -
				 CURRENT_OFFSET;

			LOG_INF("Charging: %f", result);
			return (int)result;
		}
	} else {
		return -ENOENT;
	}
	return 0;
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

int current_sample_averaged(void)
{
	int current_ma = read_analog_charging_channel();
	if (current_ma < 0) {
		LOG_ERR("Failed to read solar charging current: %d",
			current_ma);
		return -ENOENT;
	}
	uint16_t avg_current_value =
		approx_moving_average(&IchrgSMA, current_ma);
	return avg_current_value;
}