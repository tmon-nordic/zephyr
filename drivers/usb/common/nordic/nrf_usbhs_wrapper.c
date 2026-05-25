/*
 * SPDX-FileCopyrightText: Copyright Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/regulator.h>
#include <nrf_usbhs_wrapper.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbhs, LOG_LEVEL_INF);

/*
 * +-------------+        +-----------+
 * |BC12 Disabled| <--->  |BC12 Idle  | <------
 * +-------------+        +-----------+       |
 *                             ^            +-------------+
 *                  VBUS Event |            |BC12 Finished|
 *                             v            +-------------+
 *                        +-----------+       ^
 *                        |BC12 Active|  -----
 *                        +-----------+
 */
enum wrapper_bc12_state {
	BC12_DISABLED,
	BC12_IDLE,
	BC12_ACTIVE,
	BC12_FINISHED,
};

static __maybe_unused const char *const bc12_state_str[] = {
	"BC12 Disabled",
	"BC12 Idle",
	"BC12 Active",
	"BC12 Finished",
};

struct usbhs_wrapper_config {
	NRF_USBHS_Type *base;
	const struct device *vregusb_dev;
};

struct nrf_usbhs_wrapper_data {
	const struct device *dev;
	struct k_event events;
	regulator_callback_t bc12_cb;
	const void *bc12_cb_data;
	regulator_callback_t udc_cb;
	const void *udc_cb_data;
	struct k_spinlock lock;
	enum wrapper_bc12_state bc12_state;
	bool udc_enabled;
	bool vbus_present;
	bool vbus_state_changed;
	bool dcp;
};

static void vregusb_event_cb(const struct device *vreg_dev,
			     const struct regulator_event *const evt,
			     const void *const user_data)
{
	const struct device *const dev = user_data;
	const struct usbhs_wrapper_config *const config = dev->config;
	struct nrf_usbhs_wrapper_data *const data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);

	data->vbus_state_changed = true;

	if (evt->type == REGULATOR_VOLTAGE_DETECTED) {
		data->vbus_present = true;
		LOG_DBG("VBUS detected, %s, udc: %u",
			bc12_state_str[data->bc12_state], data->udc_enabled);

		if (!IS_ENABLED(CONFIG_USB_BC12_NRF_USBHS) ||
		    data->bc12_state == BC12_DISABLED) {
			/* USB is not configured for Portable Device, post the
			 * event immediately.
			 */
			LOG_DBG("USB BC12 driver disabled, post PHY ready");
			k_event_post(&data->events, NRF_USBHS_VBUS_READY);
		}
	}

	if (evt->type == REGULATOR_VOLTAGE_REMOVED) {
		data->vbus_present = false;
		LOG_DBG("VBUS removed, %s, udc: %u",
			bc12_state_str[data->bc12_state], data->udc_enabled);
		k_event_set_masked(&data->events, 0, UINT32_MAX);
	}

	k_spin_unlock(&data->lock, key);

	if (data->bc12_cb != NULL && data->bc12_state != BC12_DISABLED) {
		data->bc12_cb(config->vregusb_dev, evt, data->bc12_cb_data);
	}

	if (data->udc_cb != NULL) {
		data->udc_cb(config->vregusb_dev, evt, data->udc_cb_data);
	}
}

void nrf_usbhs_wrapper_start(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;
	NRF_USBHS_Type *wrapper = config->base;

	wrapper->TASKS_START = 1UL;
}

void nrf_usbhs_wrapper_stop(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;
	NRF_USBHS_Type *wrapper = config->base;

	wrapper->TASKS_STOP = 1UL;
}

/* Caller must hold wrapper spinlock */
static void wrapper_enable_core_internal(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;
	NRF_USBHS_Type *wrapper = config->base;

	if (wrapper->ENABLE) {
		return;
	}

	/* Power up peripheral */
	wrapper->ENABLE = USBHS_ENABLE_CORE_Msk;

	/* Set role to Device, force D+ pull-up off by overriding VBUS valid signal */
	wrapper->PHY.OVERRIDEVALUES = USBHS_PHY_OVERRIDEVALUES_ID_Msk;
	wrapper->PHY.INPUTOVERRIDE = USBHS_PHY_INPUTOVERRIDE_ID_Msk |
				     USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk |
				     USBHS_PHY_INPUTOVERRIDE_SUSPENDM0_Msk;

	/* Release PHY power-on reset */
	wrapper->ENABLE = USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk;
}

/* Caller must hold wrapper spinlock */
static void wrapper_post_enable_internal(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;
	struct nrf_usbhs_wrapper_data *const data = dev->data;
	NRF_USBHS_Type *wrapper = config->base;

	__ASSERT(wrapper->ENABLE == (USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk),
		 "Wrapper ENABLE in unexpected state 0x%08x", wrapper->ENABLE);

	if (IS_ENABLED(CONFIG_USB_BC12_NRF_USBHS) && data->bc12_state == BC12_ACTIVE) {
		/* Device cannot present itself on the bus, overrides will be
		 * released once BC1.2 detection finishes.
		 */
		return;
	}

	/* Allow D+ pull-up to become active. */
	wrapper->PHY.INPUTOVERRIDE = USBHS_PHY_INPUTOVERRIDE_ID_Msk;
	wrapper->PHY.OVERRIDEVALUES = USBHS_PHY_OVERRIDEVALUES_ID_Msk;

	if (IS_ENABLED(CONFIG_USB_BC12_NRF_USBHS)) {
		/* D+ pull-up is now enabled, so we can disable VDP_SRC in case
		 * it is left enabled after BC1.2 detection (DCP detected).
		 * For SDP and CDP this has no effect.
		 */
		wrapper->PHY.BATTCHRG = 0;
	}

	LOG_DBG("Post enable OVERRIDEVALUES 0x%08x INPUTOVERRIDE 0x%08x",
		wrapper->PHY.OVERRIDEVALUES, wrapper->PHY.INPUTOVERRIDE);
}

/* Caller must hold wrapper spinlock */
static void usbhs_wrapper_disable_check(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;
	struct nrf_usbhs_wrapper_data *const data = dev->data;
	NRF_USBHS_Type *wrapper = config->base;

	if (data->bc12_state == BC12_ACTIVE) {
		return;
	}

	if (data->dcp) {
		/* Make sure VDP_SRC is enabled before we disable D+ pull-up */
		wrapper->PHY.BATTCHRG = USBHS_PHY_BATTCHRG_VDATSRCENB0_Msk;
	}

	/* Set role to Device, force D+ pull-up off by overriding VBUS valid signal */
	wrapper->PHY.INPUTOVERRIDE = USBHS_PHY_INPUTOVERRIDE_ID_Msk |
				     USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk;
	wrapper->PHY.OVERRIDEVALUES = USBHS_PHY_OVERRIDEVALUES_ID_Msk;

	if (data->dcp) {
		/* Do not power off peripheral as it would disable+ pull-up */
		return;
	}

	if (data->udc_enabled) {
		/* Do not disable peripheral as it would reset core */
		return;
	}

	wrapper->ENABLE = 0UL;

	/* Busy looping with spinlock held is the simplest method to guarantee
	 * that PHY POR is held for at least 10 us.
	 */
	k_busy_wait(10);
}

void nrf_usbhs_wrapper_bc12_vbus_detected(const struct device *dev)
{
	struct nrf_usbhs_wrapper_data *const data = dev->data;

	K_SPINLOCK(&data->lock) {
		wrapper_enable_core_internal(dev);
		data->bc12_state = BC12_ACTIVE;
		LOG_DBG("New BC12 state %s, udc: %u",
			bc12_state_str[data->bc12_state], data->udc_enabled);
	}
}

void nrf_usbhs_wrapper_bc12_finished(const struct device *dev, const bool dcp)
{
	struct nrf_usbhs_wrapper_data *const data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);

	__ASSERT(data->bc12_state == BC12_ACTIVE, "BC12 detection is not active");
	data->bc12_state = BC12_FINISHED;
	data->dcp = dcp;

	LOG_DBG("New BC12 state %s, udc: %u, DCP: %u",
		bc12_state_str[data->bc12_state], data->udc_enabled, dcp);

	if (data->vbus_present) {
		if (data->udc_enabled) {
			/* Cease suppressing PHY clock and D+ pull-up */
			wrapper_post_enable_internal(dev);
		}

		k_event_post(&data->events, NRF_USBHS_VBUS_READY);
	}

	k_spin_unlock(&data->lock, key);
}

void nrf_usbhs_wrapper_bc12_vbus_removed(const struct device *dev)
{
	struct nrf_usbhs_wrapper_data *const data = dev->data;

	K_SPINLOCK(&data->lock) {
		data->bc12_state = BC12_IDLE;
		data->dcp = false;
		LOG_DBG("New BC12 state %s, udc: %u",
			bc12_state_str[data->bc12_state], data->udc_enabled);
		usbhs_wrapper_disable_check(dev);
	}
}

void nrf_usbhs_wrapper_bc12_pd_enabled(const struct device *dev, const bool enabled)
{
	struct nrf_usbhs_wrapper_data *const data = dev->data;

	K_SPINLOCK(&data->lock) {
		if (enabled) {
			data->bc12_state = BC12_IDLE;
			if (!data->vbus_present) {
				usbhs_wrapper_disable_check(dev);
			}
		} else {
			data->bc12_state = BC12_DISABLED;
			data->dcp = false;
			if (data->udc_enabled) {
				/* Enable D+ pull-up because it could have been
				 * disabled in BC12 callback on VBUS removal and
				 * there won't be any more BC12 calls.
				 */
				wrapper_post_enable_internal(dev);
			} else {
				usbhs_wrapper_disable_check(dev);
			}
		}
	}
}

int nrf_usbhs_wrapper_udc_pre_enable(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;
	struct nrf_usbhs_wrapper_data *const data = dev->data;
	NRF_USBHS_Type *wrapper = config->base;
	int err = 0;

	K_SPINLOCK(&data->lock) {
		if (!data->vbus_present) {
			LOG_DBG("VBUS is not present");
			err = -EAGAIN;
			K_SPINLOCK_BREAK;
		}

		if (data->bc12_state == BC12_ACTIVE) {
			LOG_DBG("BC12 detection is in progress");
			err = -EAGAIN;
			K_SPINLOCK_BREAK;
		}

		/* VBUS must be present and remain stable during startup.
		 * Here is the starting point for where the stability is required.
		 * This is the only place where vbus_state_changed can be set to false.
		 */
		data->vbus_state_changed = false;

		wrapper_enable_core_internal(dev);

		/* Allow PHY clock */
		wrapper->PHY.INPUTOVERRIDE &= ~USBHS_PHY_INPUTOVERRIDE_SUSPENDM0_Msk;

		LOG_DBG("Pre enable OVERRIDEVALUES 0x%08x INPUTOVERRIDE 0x%08x",
			wrapper->PHY.OVERRIDEVALUES, wrapper->PHY.INPUTOVERRIDE);

		data->udc_enabled = true;
	}

	if (err) {
		return err;
	}

	/* Wait for PHY clock to start */
	k_busy_wait(45);

	/* Release DWC2 reset */
	nrf_usbhs_wrapper_start(dev);

	/* Wait for clock to start to avoid hang on too early register read */
	k_msleep(1);

	K_SPINLOCK(&data->lock) {
		if (data->vbus_state_changed) {
			/* VBUS was not stable, try again when it gets stable */
			LOG_DBG("VBUS was not stable");
			err = -EAGAIN;
			data->udc_enabled = false;
			usbhs_wrapper_disable_check(dev);
		}
	}

	if (err) {
		LOG_DBG("VBUS was not stable during startup");
	}

	return err;
}

void nrf_usbhs_wrapper_udc_post_enable(const struct device *dev)
{
	struct nrf_usbhs_wrapper_data *const data = dev->data;

	K_SPINLOCK(&data->lock) {
		LOG_DBG("Post-enable UDC, %s, udc: %u",
			bc12_state_str[data->bc12_state], data->udc_enabled);
		wrapper_post_enable_internal(dev);
	}
}

void nrf_usbhs_wrapper_udc_disable(const struct device *dev)
{
	struct nrf_usbhs_wrapper_data *const data = dev->data;

	K_SPINLOCK(&data->lock) {
		LOG_DBG("Disable UDC, %s, udc: %u",
			bc12_state_str[data->bc12_state], data->udc_enabled);

		data->udc_enabled = false;
		usbhs_wrapper_disable_check(dev);
	}
}

struct k_event *nrf_usbhs_wrapper_get_events_ptr(const struct device *dev)
{
	struct nrf_usbhs_wrapper_data *const data = dev->data;

	return &data->events;
}

void nrf_usbhs_wrapper_set_bc12_cb(const struct device *dev,
				   regulator_callback_t cb,
				   const void *const user_data)
{
	struct nrf_usbhs_wrapper_data *const data = dev->data;

	K_SPINLOCK(&data->lock) {
		data->bc12_cb = cb;
		data->bc12_cb_data = user_data;
	}
}

void nrf_usbhs_wrapper_set_udc_cb(const struct device *dev,
				  regulator_callback_t cb,
				  const void *const user_data)
{
	const struct usbhs_wrapper_config *const config = dev->config;
	struct nrf_usbhs_wrapper_data *const data = dev->data;

	K_SPINLOCK(&data->lock) {
		data->udc_cb = cb;
		data->udc_cb_data = user_data;

		if (data->vbus_present) {
			struct regulator_event evt = {
				.type = REGULATOR_VOLTAGE_DETECTED,
			};

			data->udc_cb(config->vregusb_dev, &evt, data->udc_cb_data);
		}
	}
}

int nrf_usbhs_wrapper_vreg_enable(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;

	return regulator_enable(config->vregusb_dev);
}

int nrf_usbhs_wrapper_vreg_disable(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;

	return regulator_disable(config->vregusb_dev);
}

static int usbhs_wrapper_init(const struct device *dev)
{
	const struct usbhs_wrapper_config *const config = dev->config;
	struct nrf_usbhs_wrapper_data *const data = dev->data;
	int err;

	k_event_init(&data->events);

	err = regulator_set_callback(config->vregusb_dev, vregusb_event_cb, dev);
	if (err) {
		LOG_ERR("Failed to set regulator callback");
		return err;
	}

	return 0;
}

#define DT_DRV_COMPAT nordic_nrf_usbhs_wrapper

#define NRF_USBHS_WRAPPER_DEFINE(n)						\
	static const struct usbhs_wrapper_config usbhs_wrapper_config_##n = {	\
		.base = (void *)(DT_INST_REG_ADDR(n)),				\
		.vregusb_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, regulator)),	\
	};									\
										\
	static struct nrf_usbhs_wrapper_data usbhs_wrapper_data_##n;		\
										\
	DEVICE_DT_INST_DEFINE(n, usbhs_wrapper_init, NULL,			\
		      &usbhs_wrapper_data_##n, &usbhs_wrapper_config_##n,	\
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
		      NULL);

DT_INST_FOREACH_STATUS_OKAY(NRF_USBHS_WRAPPER_DEFINE)
