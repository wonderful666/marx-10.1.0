#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/usb/ch9.h>
#include <linux/mfd/hisi_pmic.h>
#include <linux/usb/audio.h>
#include <linux/version.h>
#include <linux/extcon.h>
#include <huawei_platform/usb/hw_pd_dev.h>
#include <linux/hisi/usb/hisi_usb.h>
#include <linux/hisi/usb/hisi_usb_helper.h>
#include <linux/hisi/usb/hisi_usb_reg_cfg.h>
#include <linux/hisi/usb/dwc3_usb_interface.h>
#include "dwc3-hisi.h"
#include "dwc3-hifi-usb.h"
#include "hisi_usb_bc12.h"
#include "hisi_usb_debug.h"
#include "hisi_usb_hw.h"

#ifdef CONFIG_HUAWEI_CHARGER_AP
#include <huawei_platform/power/huawei_charger.h>
#endif

struct hisi_dwc3_device *hisi_dwc3_dev;
const struct hisi_usb_phy *g_hisi_usb_phy;
static BLOCKING_NOTIFIER_HEAD(charger_type_notifier);

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE
};

static int hisi_dwc3_phy_init(struct hisi_dwc3_device *hisi_dwc,
		unsigned int host_mode);
static int hisi_dwc3_phy_shutdown(struct hisi_dwc3_device *hisi_dwc);
static int hisi_dwc3_usb20_phy_init(struct hisi_dwc3_device *hisi_dwc3, unsigned int combophy_flag);
static int hisi_dwc3_usb20_phy_shutdown(struct hisi_dwc3_device *hisi_dwc,
		unsigned int combophy_flag, unsigned int keep_power);
#ifdef CONFIG_SND
extern int usbaudio_nv_is_ready(void);
#else
static inline int usbaudio_nv_is_ready(void){return 1;}
#endif

bool hisi_usb_state_is_host(void)
{
	struct hisi_dwc3_device *hisi_dwc3 = hisi_dwc3_dev;

	if (hisi_dwc3)
		return (hisi_dwc3->state == USB_STATE_HOST);

	else
		return false;
}

int hisi_dwc3_is_fpga(void)
{
	if (!hisi_dwc3_dev) {
		usb_err("usb driver not probed!\n");
		return 0;
	}

	return !!hisi_dwc3_dev->fpga_flag;
}

void hisi_usb_unreset_phy_if_fpga(void)
{
	unsigned gpio;

	if (!hisi_dwc3_dev || hisi_dwc3_dev->fpga_phy_reset_gpio < 0)
		return;

	gpio = (unsigned)hisi_dwc3_dev->fpga_phy_reset_gpio;

	gpio_direction_output(gpio, 1);
	udelay(100);

	gpio_direction_output(gpio, 0);
	udelay(100);
}

void hisi_usb_switch_sharedphy_if_fpga(int to_hifi)
{
	if (!hisi_dwc3_dev || hisi_dwc3_dev->fpga_phy_switch_gpio < 0)
		return;

	gpio_direction_output((unsigned)hisi_dwc3_dev->fpga_phy_switch_gpio,
			!!to_hifi);
	udelay(100);
}

void dwc3_core_disable_pipe_clock(void)
{
	if (!hisi_dwc3_dev || !hisi_dwc3_dev->core_ops) {
		usb_err("[USB.CP0] usb driver not setup!\n");
		return;
	}
	/* disable pipe clock use for DP4 and must use usb2 */
	hisi_dwc3_dev->core_ops->disable_pipe_clock();
}

int dwc3_core_enable_u3(void)
{
	if (!hisi_dwc3_dev || !hisi_dwc3_dev->core_ops) {
		usb_err("[USB.CP0] usb driver not setup!\n");
		return -ENODEV;
	}
	return hisi_dwc3_dev->core_ops->enable_u3();
}

int dwc3_core_logic_analyzer_trace_set(unsigned int value)
{
	if (!hisi_dwc3_dev || !hisi_dwc3_dev->core_ops) {
		usb_err("[USB.CP0] usb driver not setup!\n");
		return -ENODEV;
	}
	hisi_dwc3_dev->core_ops->logic_analyzer_trace_set(value);

	return 0;
}

enum usb_device_speed hisi_dwc3_get_dt_host_maxspeed(void)
{
	enum usb_device_speed speed = USB_SPEED_SUPER;
	struct device *dev = NULL;
	const char *maximum_speed = NULL;
	int err;

	if (!hisi_dwc3_dev) {
		usb_err("hisi_dwc3_dev is null\n");
		return speed;
	}

	dev = &hisi_dwc3_dev->pdev->dev;

	err = device_property_read_string(dev, "host-maximum-speed", &maximum_speed);
	if (err < 0)
		return speed;

	return usb_speed_to_string(maximum_speed, strlen(maximum_speed));
}

int hisi_charger_type_notifier_register(struct notifier_block *nb)
{
	if (!nb)
		return -EINVAL;
	return blocking_notifier_chain_register(&charger_type_notifier, nb);
}
EXPORT_SYMBOL_GPL(hisi_charger_type_notifier_register);

int hisi_charger_type_notifier_unregister(struct notifier_block *nb)
{
	if (!nb)
		return -EINVAL;
	return blocking_notifier_chain_unregister(&charger_type_notifier, nb);
}
EXPORT_SYMBOL_GPL(hisi_charger_type_notifier_unregister);

void hisi_charger_type_notify(enum hisi_charger_type charger_type, void *data)
{
	int ret;

	ret = blocking_notifier_call_chain(&charger_type_notifier, charger_type,
			data);
	if (ret)
		usb_err("cb return %d\n", ret);
}

static const char *event_type_string(enum otg_dev_event_type event)
{
	static const char *const hisi_usb_event_strings[] = {
		[CHARGER_CONNECT_EVENT]		= "CHARGER_CONNECT",
		[CHARGER_DISCONNECT_EVENT]	= "CHARGER_DISCONNECT",
		[ID_FALL_EVENT]			= "OTG_CONNECT",
		[ID_RISE_EVENT]			= "OTG_DISCONNECT",
		[START_HIFI_USB]		= "START_HIFI_USB",
		[START_HIFI_USB_RESET_VBUS]	= "START_HIFI_USB_RESET_VBUS",
		[STOP_HIFI_USB]			= "STOP_HIFI_USB",
		[STOP_HIFI_USB_RESET_VBUS]	= "STOP_HIFI_USB_RESET_VBUS",
#ifdef CONFIG_USB_DWC3_NYET_ABNORMAL
		[START_AP_USE_HIFIUSB]		= "START_AP_USE_HIFIUSB",
		[STOP_AP_USE_HIFIUSB]		= "STOP_AP_USE_HIFIUSB",
#endif
		[HIFI_USB_HIBERNATE]		= "HIFI_USB_HIBERNATE",
		[HIFI_USB_WAKEUP]		= "HIFI_USB_WAKEUP",
		[DISABLE_USB3_PORT]		= "DISABLE_USB3_PORT",
		[NONE_EVENT]			= "NONE",
	};

	if (event > NONE_EVENT)
		return "illegal event";

	return hisi_usb_event_strings[event];
}

enum hisi_charger_type hisi_get_charger_type(void)
{
	if (!hisi_dwc3_dev) {
		usb_err("hisi_dwc3 not yet probed!\n");
		return CHARGER_TYPE_NONE;
	}

	usb_info("type: %s\n", charger_type_string(hisi_dwc3_dev->charger_type));
	return hisi_dwc3_dev->charger_type;
}
EXPORT_SYMBOL_GPL(hisi_get_charger_type);

static void set_vbus_power(struct hisi_dwc3_device *hisi_dwc3, unsigned int is_on)
{
	enum hisi_charger_type new;

	if (0 == is_on)
		new = CHARGER_TYPE_NONE;
	else
		new = PLEASE_PROVIDE_POWER;

	if (hisi_dwc3->charger_type != new) {
		usb_dbg("set port power %d\n", is_on);
		hisi_dwc3->charger_type = new;
		notify_charger_type(hisi_dwc3);
	}

	if (hisi_dwc3->fpga_otg_drv_vbus_gpio > 0) {
		gpio_direction_output(hisi_dwc3->fpga_otg_drv_vbus_gpio, !!is_on);
		usb_dbg("turn %s drvvbus for fpga\n", is_on ? "on" : "off");
	}
}

/*lint -save -e454 -e455 -e456 */
void hisi_dwc3_wake_lock(struct hisi_dwc3_device *hisi_dwc3)
{
	if (!hisi_dwc3->wake_lock.active) {
		usb_dbg("usb otg wake lock\n");
		__pm_stay_awake(&hisi_dwc3->wake_lock);
	}
}

void hisi_dwc3_wake_unlock(struct hisi_dwc3_device *hisi_dwc3)
{
	if (hisi_dwc3->wake_lock.active) {
		usb_dbg("usb otg wake unlock\n");
		__pm_relax(&hisi_dwc3->wake_lock);
	}
}

/*
 * create event queue
 * event_queue: event queue handle
 * count: set the queue max node
 */
int event_queue_creat(struct hiusb_event_queue *event_queue, unsigned int count)
{
	if (!event_queue) {
		usb_err("[event_queue_creat]bad argument (0x%pK)\n", event_queue);
		return -EINVAL;
	}

	count = (count >= MAX_EVENT_COUNT ? MAX_EVENT_COUNT : count);
	event_queue->max_event = count;
	event_queue->num_event = (count >= EVENT_QUEUE_UNIT ? EVENT_QUEUE_UNIT : count);

	event_queue->event = kzalloc(event_queue->num_event * sizeof(enum otg_dev_event_type), GFP_KERNEL);
	if (!event_queue->event) {
		usb_err("[event_queue_creat]:Can't alloc space:%d!\n", event_queue->num_event);
		return -ENOMEM;
	}

	event_queue->enpos = 0;
	event_queue->depos = 0;
	event_queue->overlay = 0;
	event_queue->overlay_index = 0;

	return 0;
}

void event_queue_destroy(struct hiusb_event_queue *event_queue)
{
	if (!event_queue) {
		return ;
	}

	kfree(event_queue->event);
	event_queue->event = NULL;
	event_queue->enpos = 0;
	event_queue->depos = 0;
	event_queue->num_event = 0;
	event_queue->max_event = 0;
	event_queue->overlay = 0;
	event_queue->overlay_index = 0;
}

/*
 * check if the queue is full
 * return true means full, false is not.
 */
int event_queue_isfull(struct hiusb_event_queue *event_queue)
{
	if (!event_queue) {
		return -EINVAL;
	}

	return (((event_queue->enpos + 1) % event_queue->num_event) == (event_queue->depos));
}

/*
 * check if the queue is full
 * return true means empty, false or not.
 */
int event_queue_isempty(struct hiusb_event_queue *event_queue)
{
	if (!event_queue) {
		return -EINVAL;
	}

	return (event_queue->enpos == event_queue->depos);
}

static inline void event_queue_set_overlay(struct hiusb_event_queue *event_queue)
{
	if (event_queue->overlay) {
		return;
	}
	event_queue->overlay = 1;
	event_queue->overlay_index = event_queue->enpos;
}

static inline void event_queue_clear_overlay(struct hiusb_event_queue *event_queue)
{
	event_queue->overlay = 0;
	event_queue->overlay_index = 0;
}

/*
 * put the new event en queue
 * if the event_queue is full, return -ENOSPC
 */
int event_enqueue(struct hiusb_event_queue *event_queue, enum otg_dev_event_type event)
{
	/* no need verify argument, isfull will check it */
	if (event_queue_isfull(event_queue)) {
		usb_err("event queue full!\n");
		return -ENOSPC;
	}

	if (event_queue->overlay) {
		if (event_queue->overlay_index == event_queue->enpos) {
			event_queue->enpos = ((event_queue->enpos + 1) % event_queue->num_event);
		}

		if (event_queue_isempty(event_queue)) {
			usb_err("overlay and queue isempty? just enqueue!\n");
			event_queue->overlay_index = ((event_queue->overlay_index + 1) % event_queue->num_event);
			event_queue->enpos = ((event_queue->enpos + 1) % event_queue->num_event);
			event_queue->overlay = 0;
		}

		event_queue->event[event_queue->overlay_index] = event;
	} else {
		event_queue->event[event_queue->enpos] = event;
		event_queue->enpos = ((event_queue->enpos + 1) % event_queue->num_event);
	}

	return 0;
}

/*
 * get event frome event_queue
 * this function never return fail
 * if the event_queue is empty, return NONE_EVENT
 */
enum otg_dev_event_type event_dequeue(struct hiusb_event_queue *event_queue)
{
	enum otg_dev_event_type event;

	/* no need verify argument, isempty will check it */
	if (event_queue_isempty(event_queue)) {
		return NONE_EVENT;
	}

	event = event_queue->event[event_queue->depos];
	event_queue->depos = ((event_queue->depos + 1) % event_queue->num_event);

	return event;
}

static inline int dwc3_event_work_is_sync(struct hisi_dwc3_device *hisi_dwc)
{
	return hisi_dwc->is_hanle_event_sync;
}

static void hisi_dwc3_check_voltage(struct hisi_dwc3_device *hisi_dwc)
{
	usb_dbg("+\n");

	if (hisi_dwc->check_voltage) {
		/*first dplus pulldown 15k*/
		hisi_usb_dpdm_pulldown(hisi_dwc);
#ifdef CONFIG_HUAWEI_CHARGER_AP
		/*second call charger's API to check voltage */
		water_detect_event_notify(WD_NE_DETECT_BY_USB_DP_DN, NULL);
#endif
		/*third dplus pullup*/
		hisi_usb_dpdm_pullup(hisi_dwc);
	}
	usb_dbg("-\n");
}

static int start_device(struct hisi_dwc3_device *hisi_dwc)
{
	int ret;

	/* due to detect charger type, must resume hisi_dwc */
	ret = hisi_dwc3_phy_init(hisi_dwc, 0);
	if (ret) {
		usb_err("hisi_dwc3_phy_init failed (ret %d)\n", ret);
		return ret;
	}

	/*if the platform support,it need check voltage*/
	hisi_dwc3_check_voltage(hisi_dwc);

	/* detect charger type */
	hisi_dwc->charger_type = hisi_usb_detect_charger_type(hisi_dwc);
	notify_charger_type(hisi_dwc);

	/* In some cases, DCP is detected as SDP wrongly. To avoid this,
	 * start bc_again delay work to detect charger type once more.
	 * If later the enum process is executed, then it's a real SDP, so
	 * the work will be canceled.
	 */
	if (bc_again_allowed(hisi_dwc))
		schedule_bc_again(hisi_dwc);

	/* do not start peripheral if real charger connected */
	if (enumerate_allowed(hisi_dwc)) {
		if (hisi_dwc->fpga_usb_mode_gpio > 0) {
			gpio_direction_output((unsigned)hisi_dwc->fpga_usb_mode_gpio, 0);
			usb_dbg("switch to device mode\n");
		}

		/* start peripheral */
		ret = extcon_set_state_sync(hisi_dwc->edev,
				EXTCON_USB, true);
		if (ret) {
			usb_err("start peripheral error\n");
			return ret;
		}
	} else {
		usb_dbg("it need notify USB_CONNECT_DCP while a real charger connected\n");
		hisi_dwc->speed = USB_CONNECT_DCP;
		if (!queue_work(system_power_efficient_wq,
						&hisi_dwc->speed_change_work)) {
			usb_err("schedule speed_change_work wait:%d\n", hisi_dwc->speed);
		}
	}

	hisi_dwc->state = USB_STATE_DEVICE;

	if (sleep_allowed(hisi_dwc))
		hisi_dwc3_wake_unlock(hisi_dwc);
	else
		hisi_dwc3_wake_lock(hisi_dwc);

	usb_dbg("hisi usb status: OFF -> DEVICE\n");

	return 0;
}

static void stop_device(struct hisi_dwc3_device *hisi_dwc)
{
	int ret;

	hisi_usb_disable_vdp_src(hisi_dwc);

	/* peripheral not started, if real charger connected */
	if (enumerate_allowed(hisi_dwc)) {
		/* stop peripheral */
		ret = extcon_set_state_sync(hisi_dwc->edev,
				EXTCON_USB, false);
		if (ret) {
			usb_err("stop peripheral error\n");
			return;
		}
	}

	cancel_bc_again(hisi_dwc, 0);

	/* usb cable disconnect, notify no charger */
	hisi_dwc->charger_type = CHARGER_TYPE_NONE;
	notify_charger_type(hisi_dwc);

	hisi_dwc->state = USB_STATE_OFF;
	hisi_dwc3_wake_unlock(hisi_dwc);

	ret = hisi_dwc3_phy_shutdown(hisi_dwc);
	if (ret)
		usb_err("hisi_dwc3_phy_shutdown failed (ret %d)\n", ret);

	usb_dbg("hisi usb status: DEVICE -> OFF\n");
}

static int start_host(struct hisi_dwc3_device *hisi_dwc)
{
	int ret;

	ret = hisi_dwc3_phy_init(hisi_dwc, 1);
	if (ret) {
		usb_err("hisi_dwc3_phy_init failed (ret %d)\n", ret);
		return ret;
	}

	if (hisi_dwc->fpga_usb_mode_gpio > 0) {
		gpio_direction_output((unsigned)hisi_dwc->fpga_usb_mode_gpio, 1);
		usb_dbg("switch to host mode\n");
	}

	/* start host */
	ret = extcon_set_state_sync(hisi_dwc->edev,
			EXTCON_USB_HOST, true);
	if (ret) {
		usb_err("start host error\n");
		return ret;
	}

	return 0;
}

static int stop_host(struct hisi_dwc3_device *hisi_dwc)
{
	int ret;

	/* stop host */
	ret = extcon_set_state_sync(hisi_dwc->edev,
			EXTCON_USB_HOST, false);
	if (ret) {
		usb_err("stop host error\n");
		return ret;
	}

	ret = hisi_dwc3_phy_shutdown(hisi_dwc);
	if (ret)
		usb_err("hisi_dwc3_phy_shutdown failed (ret %d)\n", ret);

	return ret;
}

static int start_audio_usb(struct hisi_dwc3_device *hisi_dwc, unsigned int combophy_flag)
{
	if (hisi_dwc3_usb20_phy_init(hisi_dwc, combophy_flag)) {
		usb_err("audio usb phy init failed\n");
		return -EBUSY;
	}

	if (start_hifi_usb()) {
		if (hisi_dwc3_usb20_phy_shutdown(hisi_dwc, combophy_flag, 0))
			WARN_ON(1);
		return -EBUSY;
	}

	return 0;
}

static void stop_audio_usb(struct hisi_dwc3_device *hisi_dwc, unsigned int combophy_flag)
{
	stop_hifi_usb();
	if (hisi_dwc3_usb20_phy_shutdown(hisi_dwc, combophy_flag, 0))
		WARN_ON(1);
}

static void handle_start_hifi_usb_event(struct hisi_dwc3_device *hisi_dwc,
					int reset_vbus)
{
	switch (hisi_dwc->state) {
	case USB_STATE_OFF:
		set_vbus_power(hisi_dwc, 1);
		if (start_audio_usb(hisi_dwc, 1)) {
			usb_err("start_audio_usb failed\n");
			if (start_host(hisi_dwc)) {
				usb_err("start_host failed\n");
			} else {
				hisi_dwc->state = USB_STATE_HOST;
				hisi_dwc3_wake_lock(hisi_dwc);
				usb_dbg("hisi usb_status: OFF -> HOST\n");
			}
			return;
		}

		hisi_dwc->state = USB_STATE_HIFI_USB;
		hisi_dwc3_wake_unlock(hisi_dwc);
		pd_dpm_wakelock_ctrl(PD_WAKE_UNLOCK);
		usb_dbg("hisi usb state: OFF -> HIFI_USB\n");
		usb_dbg("hisi usb start hifi usb time: %d ms\n",
				jiffies_to_msecs(jiffies
				- hisi_dwc->start_host_time_stamp));
		break;

	case USB_STATE_HOST:
		if (reset_vbus) {
			set_vbus_power(hisi_dwc, 0);
			pd_dpm_vbus_ctrl(CHARGER_TYPE_NONE);
		}

		if (stop_host(hisi_dwc)) {
			usb_err("stop_host failed\n");
			return;
		}

		if (reset_vbus) {
			set_vbus_power(hisi_dwc, 1);
			pd_dpm_vbus_ctrl(PLEASE_PROVIDE_POWER);
		}

		if (start_audio_usb(hisi_dwc, 1)) {
			usb_err("start_audio_usb failed\n");
			if (start_host(hisi_dwc))
				usb_err("start_host failed\n");
			return;
		}

		hisi_dwc->state = USB_STATE_HIFI_USB;
		hisi_dwc3_wake_unlock(hisi_dwc);
		pd_dpm_wakelock_ctrl(PD_WAKE_UNLOCK);
		usb_dbg("hisi usb state: HOST -> HIFI_USB\n");

		usb_dbg("hisi usb start hifi usb time: %d ms\n",
				jiffies_to_msecs(jiffies
				- hisi_dwc->start_hifiusb_time_stamp));
		break;
	default:
		usb_dbg("event %d in state %d\n", START_HIFI_USB, hisi_dwc->state);
		break;
	}
	hisi_dwc->start_hifiusb_complete_time_stamp = jiffies;
}

static void handle_stop_hifi_usb_event(struct hisi_dwc3_device *hisi_dwc,
					int reset_vbus)
{
	int ret;

	switch (hisi_dwc->state) {
	case USB_STATE_HIFI_USB:
		hisi_dwc3_wake_lock(hisi_dwc);
		pd_dpm_wakelock_ctrl(PD_WAKE_LOCK);

		stop_audio_usb(hisi_dwc, 1);

		if (reset_vbus) {
			set_vbus_power(hisi_dwc, 0);
			pd_dpm_vbus_ctrl(CHARGER_TYPE_NONE);
		}

		if (!hisi_dwc->hifi_ip_first)
			msleep(1500);

		if (start_host(hisi_dwc)) {
			usb_err("start_host failed\n");
			return;
		}

		if (reset_vbus) {
			set_vbus_power(hisi_dwc, 1);
			pd_dpm_vbus_ctrl(PLEASE_PROVIDE_POWER);
		}

		hisi_dwc->state = USB_STATE_HOST;
		usb_dbg("hisi usb state: HIFI_USB -> HOST\n");

		break;
	case USB_STATE_HIFI_USB_HIBERNATE:
		hisi_dwc3_wake_lock(hisi_dwc);
		pd_dpm_wakelock_ctrl(PD_WAKE_LOCK);

		/* phy was closed in this state */
		stop_hifi_usb();

		msleep(1500);

		ret = hisi_dwc3_usb20_phy_init(hisi_dwc, 0);
		if (ret)
			usb_err("shared_phy_init error ret %d\n", ret);
		ret = hisi_dwc3_usb20_phy_shutdown(hisi_dwc, 1, 0);
		if (ret)
			usb_err("shared_phy_shutdown error ret %d\n", ret);

		if (start_host(hisi_dwc)) {
			usb_err("start_host failed\n");
			return;
		}

		hisi_dwc->state = USB_STATE_HOST;
		usb_dbg("hisi usb state: HIFI_USB_HIBERNATE -> HOST\n");

		break;
	default:
		usb_dbg("event %d in state %d\n", STOP_HIFI_USB, hisi_dwc->state);
		break;
	}
}

static void handle_start_hifi_usb_hibernate(struct hisi_dwc3_device *hisi_dwc)
{
	switch (hisi_dwc->state) {
	case USB_STATE_HIFI_USB:
		if (hifi_usb_hibernate()) {
			WARN_ON(1);
			return;
		}

		if (hisi_dwc3_usb20_phy_shutdown(hisi_dwc, 0, 1)) {
			WARN_ON(1);
			return;
		}

		hisi_dwc->state = USB_STATE_HIFI_USB_HIBERNATE;
		usb_dbg("hisi usb state: HIFI_USB -> HIFI_USB_HIBERNATE\n");
		msleep(50); /* debounce of suspend state */
		break;
	default:
		usb_dbg("event %d in state %d\n", HIFI_USB_HIBERNATE, hisi_dwc->state);
		break;
	}
}

static void handle_start_hifi_usb_wakeup(struct hisi_dwc3_device *hisi_dwc)
{
	switch (hisi_dwc->state) {
	case USB_STATE_HIFI_USB_HIBERNATE:
		if (hisi_dwc3_usb20_phy_init(hisi_dwc, 0)) {
			WARN_ON(1);
			return;
		}

		if (hifi_usb_revive()) {
			WARN_ON(1);
			return;
		}

		hisi_dwc->state = USB_STATE_HIFI_USB;
		usb_dbg("hisi usb state: HIFI_USB_HIBERNATE -> HIFI_USB\n");
		break;
	default:
		usb_dbg("event %d in state %d\n", HIFI_USB_WAKEUP, hisi_dwc->state);
		break;
	}

	hifi_usb_complete_wakeup();
}

/* Caution: this function must be called in "hisi_dwc3->lock"
 * Currently, this function called only by hisi_usb_resume */
int hisi_usb_wakeup_hifi_usb(void)
{
	struct hisi_dwc3_device *dev = hisi_dwc3_dev;
	if (!dev)
		return -ENOENT;

	handle_start_hifi_usb_wakeup(dev);
	return 0;
}

static void handle_charger_connect_event(struct hisi_dwc3_device *hisi_dwc)
{
	if (USB_STATE_DEVICE == hisi_dwc->state) {
		usb_dbg("Already in device mode, do nothing\n");
	} else if (USB_STATE_OFF == hisi_dwc->state) {
		if (start_device(hisi_dwc))
			usb_err("start_device error\n");
	} else if (USB_STATE_HOST == hisi_dwc->state) {
		usb_dbg("Charger connect intrrupt in HOST mode\n");
	} else if (USB_STATE_HIFI_USB == hisi_dwc->state) {
		usb_dbg("vbus power in hifi usb state\n");
	} else {
		usb_dbg("can not handle_charger_connect_event in mode %s\n",
				hisi_usb_state_string(hisi_dwc->state));
	}
}

static void handle_charger_disconnect_event(struct hisi_dwc3_device *hisi_dwc)
{
	if (USB_STATE_OFF == hisi_dwc->state) {
		usb_dbg("Already in off mode, do nothing\n");
	} else if (USB_STATE_DEVICE == hisi_dwc->state) {
		stop_device(hisi_dwc);
	} else if (USB_STATE_HOST == hisi_dwc->state) {
		usb_dbg("Charger disconnect intrrupt in HOST mode\n");
	} else if (USB_STATE_HIFI_USB == hisi_dwc->state) {
		/* lose power ??? */
		usb_dbg("vbus disconnect event in hifi usb state\n");
	} else {
		usb_dbg("can not handle_charger_disconnect_event in mode %s\n",
				hisi_usb_state_string(hisi_dwc->state));
	}
}

static void handle_start_arm_usb_event(struct hisi_dwc3_device *hisi_dwc)
{
	switch (hisi_dwc->state) {
	case USB_STATE_OFF:
		set_vbus_power(hisi_dwc, 1);
		if (start_host(hisi_dwc))
			set_vbus_power(hisi_dwc, 0);

		hisi_dwc->state = USB_STATE_HOST;
		hisi_dwc3_wake_lock(hisi_dwc);
		usb_dbg("hisi usb_status: OFF -> HOST\n");
		break;
	default:
		usb_dbg("event %d in state %d\n", ID_FALL_EVENT, hisi_dwc->state);
		break;
	}
}

static void handle_id_fall_event(struct hisi_dwc3_device *hisi_dwc)
{
	/*
	 * 1. hifi ip first feature controlled by device tree.
	 * 2. hifi usb need to wait for usbauddio nv.
	 */
	if (hisi_usb_otg_use_hifi_ip_first()
		&& (TCPC_USB31_CONNECTED == hisi_dwc->mode_type)) {
		usb_dbg("use hifi ip first\n");
		handle_start_hifi_usb_event(hisi_dwc, 0);

	} else {
		usb_dbg("use arm ip first\n");
		handle_start_arm_usb_event(hisi_dwc);
	}

}

static void handle_id_rise_event(struct hisi_dwc3_device *hisi_dwc)
{
	switch (hisi_dwc->state) {
	case USB_STATE_HOST:
		set_vbus_power(hisi_dwc, 0);
		if (stop_host(hisi_dwc))
			usb_err("stop_host failed\n");

		hisi_dwc->state = USB_STATE_OFF;
		hisi_dwc3_wake_unlock(hisi_dwc);
		usb_dbg("hiusb_status: HOST -> OFF\n");

		reset_hifi_usb();
		break;
	case USB_STATE_HIFI_USB:
		set_vbus_power(hisi_dwc, 0);
		stop_audio_usb(hisi_dwc, 0);

		hisi_dwc->state = USB_STATE_OFF;
		hisi_dwc3_wake_unlock(hisi_dwc);
		usb_dbg("hisi usb state: HIFI_USB -> OFF\n");

		reset_hifi_usb();

		usb_dbg("hisi usb stop hifi usb time: %d ms\n",
				jiffies_to_msecs(jiffies
					- hisi_dwc->stop_host_time_stamp));
		break;

	case USB_STATE_HIFI_USB_HIBERNATE:
		set_vbus_power(hisi_dwc, 0);
		/* phy was closed in this state */
		stop_hifi_usb();

		hisi_dwc->state = USB_STATE_OFF;
		hisi_dwc3_wake_unlock(hisi_dwc);
		usb_dbg("hisi usb state: HIFI_USB_HIBERNATE -> OFF\n");

		reset_hifi_usb();

		usb_dbg("hisi usb stop hifi usb time: %d ms\n",
				jiffies_to_msecs(jiffies
				- hisi_dwc->stop_host_time_stamp));
		break;

#ifdef CONFIG_USB_DWC3_NYET_ABNORMAL
	case USB_STATE_AP_USE_HIFIUSB:
		set_vbus_power(hisi_dwc, 0);
		ap_stop_use_hifiusb();

		if (hisi_dwc3_usb20_phy_shutdown(hisi_dwc, 0, 0))
			WARN_ON(1);

		hisi_dwc->state = USB_STATE_OFF;
		hisi_dwc3_wake_unlock(hisi_dwc);
		usb_dbg("hisi usb state: AP_USE_HIFI_USB -> OFF\n");

		break;
#endif
	default:
		usb_dbg("event %d in state %d\n", ID_RISE_EVENT, hisi_dwc->state);
		break;
	}
}

#ifdef CONFIG_USB_DWC3_NYET_ABNORMAL
static int dwc3_tcpc_is_usb_only(struct hisi_dwc3_device *hisi_dwc)
{
	if (hisi_dwc->usb_phy && hisi_dwc->usb_phy->tcpc_is_usb_only)
		return hisi_dwc->usb_phy->tcpc_is_usb_only();

	return 0;
}

static void handle_start_ap_use_hifiusb(struct hisi_dwc3_device *hisi_dwc)
{
	int is_usb_only;

	switch (hisi_dwc->state) {
	case USB_STATE_HOST:
		if (stop_host(hisi_dwc)) {
			usb_err("stop_host failed\n");
			return;
		}

		is_usb_only = dwc3_tcpc_is_usb_only(hisi_dwc);
		if (is_usb_only) {
			usb_dbg("combophy is in usb only mode, do vbus reset\n");
			set_vbus_power(hisi_dwc, 0);
			pd_dpm_vbus_ctrl(CHARGER_TYPE_NONE);
			msleep(30);
		}

		if (hisi_dwc3_usb20_phy_init(hisi_dwc, 0)) {
			usb_err("audio usb phy init failed\n");
		} else if (ap_start_use_hifiusb()) {
			usb_err("start ap use hifiusb failed");
			if (!hisi_dwc3_usb20_phy_shutdown(hisi_dwc, 0, 0)) {
				if (start_host(hisi_dwc))
					usb_err("start_host failed\n");
			} else
				usb_err("shard phy shutdown failed\n");
		} else {
			hisi_dwc->state = USB_STATE_AP_USE_HIFIUSB;
			usb_dbg("hisi usb state: HOST -> AP_USE_HIFI_USB\n");
		}

		if (is_usb_only) {
			msleep(30);
			set_vbus_power(hisi_dwc, 1);
			pd_dpm_vbus_ctrl(PLEASE_PROVIDE_POWER);
		}

		break;
	default:
		usb_dbg("event %d in state %d\n", START_AP_USE_HIFIUSB,
				hisi_dwc->state);
		break;
	}
}

static void handle_stop_ap_use_hifiusb(struct hisi_dwc3_device *hisi_dwc)
{
	switch (hisi_dwc->state) {
	case USB_STATE_AP_USE_HIFIUSB:
		ap_stop_use_hifiusb();

		if (hisi_dwc3_usb20_phy_shutdown(hisi_dwc, 0, 0))
			WARN_ON(1);

		if (start_host(hisi_dwc)) {
			usb_err("start_host failed\n");
			return;
		}

		hisi_dwc->state = USB_STATE_HOST;
		usb_dbg("hisi usb state: AP_USE_HIFI_USB -> HOST\n");

		break;
	default:
		usb_dbg("event %d in state %d\n", STOP_AP_USE_HIFIUSB,
				hisi_dwc->state);
		break;
	}
}
#endif

static void hisi_usb_disable_usb3(struct hisi_dwc3_device *hisi_dwc)
{
	if (hisi_dwc->usb_phy && hisi_dwc->usb_phy->disable_usb3)
		hisi_dwc->usb_phy->disable_usb3();
}

static void handle_disable_usb3(struct hisi_dwc3_device *hisi_dwc)
{
	int ret;

	if (hisi_dwc->state != USB_STATE_HOST) {
		usb_dbg("event %d in state %d\n", DISABLE_USB3_PORT,
				hisi_dwc->state);
		return;
	}

	if (!hisi_dwc->usb_phy || !hisi_dwc->usb_phy->disable_usb3)
		return;

	/* stop host */
	ret = extcon_set_state_sync(hisi_dwc->edev,
			EXTCON_USB_HOST, false);
	if (ret) {
		usb_err("stop host error\n");
		return;
	}

	hisi_usb_disable_usb3(hisi_dwc);

	/* start host */
	ret = extcon_set_state_sync(hisi_dwc->edev,
			EXTCON_USB_HOST, true);
	if (ret) {
		usb_err("start host error\n");
		return;
	}
}

static void handle_event(struct hisi_dwc3_device *hisi_dwc, enum otg_dev_event_type event)
{
	int reset_vbus = 0;

	usb_err("type: %s\n", event_type_string(event));

	if (event == START_HIFI_USB_RESET_VBUS) {
		event = START_HIFI_USB;
		reset_vbus = 1;
	}

	if (event == STOP_HIFI_USB_RESET_VBUS) {
		event = STOP_HIFI_USB;
		reset_vbus = 1;
	}

	switch (event) {
	case CHARGER_CONNECT_EVENT:
		handle_charger_connect_event(hisi_dwc);
		hisi_dwc->start_device_complete_time_stamp = jiffies;
		break;

	case CHARGER_DISCONNECT_EVENT:
		handle_charger_disconnect_event(hisi_dwc);
		break;

	case ID_FALL_EVENT:
		handle_id_fall_event(hisi_dwc);
		hisi_dwc->start_host_complete_time_stamp = jiffies;
		break;

	case ID_RISE_EVENT:
		handle_id_rise_event(hisi_dwc);
		hisi_dwc->stop_host_complete_time_stamp = jiffies;
		break;

	case START_HIFI_USB:
		handle_start_hifi_usb_event(hisi_dwc, reset_vbus);
		break;

	case STOP_HIFI_USB:
		handle_stop_hifi_usb_event(hisi_dwc, reset_vbus);
		hisi_dwc->stop_hifiusb_complete_time_stamp = jiffies;
		break;

#ifdef CONFIG_USB_DWC3_NYET_ABNORMAL
	case START_AP_USE_HIFIUSB:
		handle_start_ap_use_hifiusb(hisi_dwc);
		break;

	case STOP_AP_USE_HIFIUSB:
		handle_stop_ap_use_hifiusb(hisi_dwc);
		break;
#endif

	case HIFI_USB_HIBERNATE:
		handle_start_hifi_usb_hibernate(hisi_dwc);
		break;

	case HIFI_USB_WAKEUP:
		hisi_dwc3_wake_lock(hisi_dwc);
		handle_start_hifi_usb_wakeup(hisi_dwc);
		hisi_dwc3_wake_unlock(hisi_dwc);
		break;

	case DISABLE_USB3_PORT:
		handle_disable_usb3(hisi_dwc);
		break;

	default:
		usb_dbg("illegal event type!\n");
		break;
	}
}

static void event_work(struct work_struct *work)
{
	unsigned long flags;
	enum otg_dev_event_type event;

	struct hisi_dwc3_device *hisi_dwc = container_of(work,
				    struct hisi_dwc3_device, event_work);

	usb_err("+\n");
	mutex_lock(&hisi_dwc->lock);

	while (!event_queue_isempty(&hisi_dwc->event_queue)) {
		spin_lock_irqsave(&(hisi_dwc->event_lock), flags);
		event = event_dequeue(&hisi_dwc->event_queue);
		spin_unlock_irqrestore(&(hisi_dwc->event_lock), flags);

		handle_event(hisi_dwc, event);
	}

	event_queue_clear_overlay(&hisi_dwc->event_queue);

	mutex_unlock(&hisi_dwc->lock);

	if (dwc3_event_work_is_sync(hisi_dwc)) {
		usb_err("sync & complete\n");
		complete(&hisi_dwc->event_completion);
	}
	usb_err("-\n");
	return;
}

static void hisi_dwc3_speed_change_work(struct work_struct *work)
{
	struct hisi_dwc3_device *hisi_dwc = container_of(work,
				    struct hisi_dwc3_device, speed_change_work);
	usb_dbg("+\n");
	if (hisi_dwc->fpga_flag) {
		usb_dbg("- fpga platform, don't notify speed\n");
		return ;
	}

	if (hisi_dwc->use_new_frame && hisi_dwc->notify_speed) {
		usb_dbg("+device speed is %d\n", hisi_dwc->speed);

#ifdef CONFIG_TCPC_CLASS
		if ((hisi_dwc->speed != USB_CONNECT_HOST) &&
				(hisi_dwc->speed != USB_CONNECT_DCP))
			hw_usb_set_usb_speed(hisi_dwc->speed);
#endif

		usb_dbg("-\n");
	} else if (hisi_dwc->usb_phy && hisi_dwc->usb_phy->notify_speed) {
		hisi_dwc->usb_phy->notify_speed(hisi_dwc->speed);
	}
	usb_dbg("-\n");
}

void hisi_dwc3_platform_host_quirks(void)
{
	usb_dbg("+\n");
	if (!hisi_dwc3_dev || !hisi_dwc3_dev->core_ops) {
		usb_err("USB drv not probe!\n");
		return;
	}

	/* BugNo: 9001202031 */
	if (hisi_dwc3_dev->quirk_disable_usb2phy_suspend) {
		hisi_dwc3_dev->core_ops->disable_usb2phy_suspend();
	}

	/* BugNo: 9001205968 */
	if (hisi_dwc3_dev->quirk_clear_svc_opp_per_hs) {
		hisi_dwc3_dev->core_ops->clear_svc_opp_per_hs();
	}

	/* BugNo: 9001208988 */
	if (hisi_dwc3_dev->quirk_disable_rx_thres_cfg) {
		hisi_dwc3_dev->core_ops->disable_rx_thres_cfg();
	}

	/* BugNo: 9001212079 */
	if (hisi_dwc3_dev->quirk_set_svc_opp_per_hs_sep) {
		hisi_dwc3_dev->core_ops->set_svc_opp_per_hs_sep();
	}

	/* BugNo: 9001227814 */
	if (hisi_dwc3_dev->quirk_adjust_dtout) {
		hisi_dwc3_dev->core_ops->adjust_dtout();
	}

	/* BugNo: 9001238552 */
	if (hisi_dwc3_dev->quirk_force_disable_host_lpm) {
		hisi_dwc3_dev->core_ops->force_disable_host_lpm();
	}

	/* BugNo: 9001162113 */
	if (hisi_dwc3_dev->quirk_enable_hst_imm_retry) {
		hisi_dwc3_dev->core_ops->enable_hst_imm_retry();
	}

	usb_dbg("-\n");
}

EXPORT_SYMBOL_GPL(hisi_dwc3_platform_host_quirks);

void hisi_dwc3_platform_device_quirks(void)
{
	usb_dbg("+\n");
	if (!hisi_dwc3_dev || !hisi_dwc3_dev->core_ops) {
		usb_err("USB drv not probe!\n");
		return;
	}

	/* BugNo: 9001169999 */
	if (hisi_dwc3_dev->quirk_enable_p4_gate) {
		hisi_dwc3_dev->core_ops->enable_p4_gate();
	}

	usb_dbg("-\n");
}
EXPORT_SYMBOL_GPL(hisi_dwc3_platform_device_quirks);

static enum otg_dev_event_type hifi_usb_event_filter(
					enum otg_dev_event_type event)
{
	if (event == START_HIFI_USB_RESET_VBUS)
		return START_HIFI_USB;
	else if (event == STOP_HIFI_USB_RESET_VBUS)
		return STOP_HIFI_USB;
	else
		return event;
}

static int id_rise_event_check(enum otg_dev_event_type last_event)
{
	if ((last_event == ID_FALL_EVENT)
		    || (last_event == START_HIFI_USB)
		    || (last_event == STOP_HIFI_USB)

#ifdef CONFIG_USB_DWC3_NYET_ABNORMAL
		    || (last_event == START_AP_USE_HIFIUSB)
		    || (last_event == STOP_AP_USE_HIFIUSB)
#endif
		    || (last_event == HIFI_USB_HIBERNATE)
		    || (last_event == HIFI_USB_WAKEUP)
		    || (last_event == DISABLE_USB3_PORT))
		return 1;
	else
		return 0;
}

static int id_fall_event_check(enum otg_dev_event_type last_event)
{
	if ((last_event == CHARGER_DISCONNECT_EVENT)
			|| (last_event == ID_RISE_EVENT))
		return 1;
	else
		return 0;
}

static int start_hifi_usb_event_check(enum otg_dev_event_type last_event)
{
	if ((last_event == ID_FALL_EVENT)
		    || (last_event == STOP_HIFI_USB)
		    || (last_event == START_HIFI_USB)) /* start hifiusb maybe failed, allow retry */
		return 1;
	else
		return 0;
}

static int stop_hifi_usb_event_check(enum otg_dev_event_type last_event)
{
	if ((last_event == START_HIFI_USB)
		    || (last_event == HIFI_USB_WAKEUP)
		    || (last_event == HIFI_USB_HIBERNATE)
		    || (last_event == ID_FALL_EVENT))
		return 1;
	else
		return 0;

}

static int hifi_usb_hibernate_event_check(enum otg_dev_event_type last_event)
{
	if ((last_event == START_HIFI_USB)
		    || (last_event == HIFI_USB_WAKEUP)
		    || (last_event == ID_FALL_EVENT))
		return 1;
	return 0;
}

static int hifi_usb_wakeup_event_check(enum otg_dev_event_type last_event)
{
	if ((last_event == HIFI_USB_HIBERNATE)
			|| (last_event == HIFI_USB_WAKEUP))
		return 1;
	return 0;
}

static int event_check(enum otg_dev_event_type last_event,
		enum otg_dev_event_type new_event)
{
	int ret = 0;

	if (last_event == NONE_EVENT)
		return 1;

	last_event = hifi_usb_event_filter(last_event);
	new_event = hifi_usb_event_filter(new_event);

	switch (new_event) {
	case CHARGER_CONNECT_EVENT:
		if ((last_event == CHARGER_DISCONNECT_EVENT)
				|| (last_event == ID_RISE_EVENT))
			ret = 1;
		break;
	case CHARGER_DISCONNECT_EVENT:
		if (last_event == CHARGER_CONNECT_EVENT)
			ret = 1;
		break;
	case ID_FALL_EVENT:
		ret = id_fall_event_check(last_event);
		break;
	case ID_RISE_EVENT:
		ret = id_rise_event_check(last_event);
		break;
	case START_HIFI_USB:
		ret = start_hifi_usb_event_check(last_event);
		break;
	case STOP_HIFI_USB:
		ret = stop_hifi_usb_event_check(last_event);
		break;
	case HIFI_USB_HIBERNATE:
		ret = hifi_usb_hibernate_event_check(last_event);
		break;
	case HIFI_USB_WAKEUP:
		ret = hifi_usb_wakeup_event_check(last_event);
		break;

#ifdef CONFIG_USB_DWC3_NYET_ABNORMAL
	case START_AP_USE_HIFIUSB:
		if ((last_event == ID_FALL_EVENT)
				|| (last_event == STOP_HIFI_USB)
				|| (last_event == STOP_AP_USE_HIFIUSB))
			ret = 1;
		break;
	case STOP_AP_USE_HIFIUSB:
		if (last_event == START_AP_USE_HIFIUSB)
			ret = 1;
		break;
#endif

	case DISABLE_USB3_PORT:
		if (last_event == ID_FALL_EVENT ||
				last_event == STOP_HIFI_USB)
			ret = 1;
		break;
	default:
		break;
	}
	return ret;
}

static void save_event_time_stamp(struct hisi_dwc3_device *hisi_dwc3,
			enum otg_dev_event_type event)
{
	if (event == CHARGER_CONNECT_EVENT)
		hisi_dwc3->start_device_time_stamp = jiffies;
	else if (event == ID_FALL_EVENT)
		hisi_dwc3->start_host_time_stamp = jiffies;
	else if (event == ID_RISE_EVENT)
		hisi_dwc3->stop_host_time_stamp = jiffies;
	else if ((event == START_HIFI_USB)
			|| (event == START_HIFI_USB_RESET_VBUS))
		hisi_dwc3->start_hifiusb_time_stamp = jiffies;
	else if ((event == STOP_HIFI_USB)
			|| (event == STOP_HIFI_USB_RESET_VBUS))
		hisi_dwc3->stop_hifiusb_time_stamp = jiffies;
}

static int hisi_usb_vbus_value(void)
{
	return hisi_pmic_get_vbus_status();
}

/*
 * return 0 means event was accepted, others means event was rejected.
 */
int hisi_usb_otg_event(enum otg_dev_event_type event)
{
	int ret = 0;
#ifdef CONFIG_USB_DWC3_DUAL_ROLE
	unsigned long flags;
	struct hisi_dwc3_device *hisi_dwc3 = hisi_dwc3_dev;

	if (!hisi_dwc3)
		return -ENODEV;

	if (hisi_dwc3->eventmask) {
		usb_dbg("eventmask enabled, mask all events.\n");
		return -EPERM;
	}

	spin_lock_irqsave(&(hisi_dwc3->event_lock), flags);

	if (event_check(hisi_dwc3->event, event)) {
		usb_dbg("event: %s\n", event_type_string(event));
		hisi_dwc3->event = event;

		save_event_time_stamp(hisi_dwc3, event);

		if ((CHARGER_CONNECT_EVENT == event)
				|| (CHARGER_DISCONNECT_EVENT == event))
			hisi_dwc3_wake_lock(hisi_dwc3);

		if (!event_enqueue(&hisi_dwc3->event_queue, event)) {
			if (!queue_work(system_power_efficient_wq,
					&hisi_dwc3->event_work)) {
				usb_err("schedule event_work wait:%d\n", event);
			}
		} else {
			usb_err("hisi_usb_otg_event can't enqueue event:%d\n", event);
			ret = -EBUSY;
		}

		if ((ID_RISE_EVENT == event) || (CHARGER_DISCONNECT_EVENT == event)) {
			event_queue_set_overlay(&hisi_dwc3->event_queue);

			usb_dbg("it need notify USB_SPEED_UNKNOWN to app while usb plugout\n");
			hisi_dwc3->speed = USB_SPEED_UNKNOWN;
			if (!queue_work(system_power_efficient_wq,
							&hisi_dwc3->speed_change_work)) {
				usb_err("schedule speed_change_work wait:%d\n", hisi_dwc3->speed);
			}
		}
	} else {
		usb_err("last event: [%s], event [%s] was rejected.\n",
				event_type_string(hisi_dwc3->event), event_type_string(event));
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&(hisi_dwc3->event_lock), flags);
#endif
	return ret;
}
EXPORT_SYMBOL_GPL(hisi_usb_otg_event);

/*lint -save -e578 */
static inline void dwc3_event_work_set_sync(struct hisi_dwc3_device *hisi_dwc)
{
	/* set sync flag */
	hisi_dwc->is_hanle_event_sync = 1;
}

static inline void dwc3_event_work_clr_sync(struct hisi_dwc3_device *hisi_dwc)
{
	hisi_dwc->is_hanle_event_sync = 0;
}

static DEFINE_MUTEX(sync_event_lock);

int hisi_usb_otg_use_hifi_ip_first()
{
	struct hisi_dwc3_device *hisi_dwc3 = hisi_dwc3_dev;

	/*
	 * just check if usb module probe.
	 */
	if (!hisi_dwc3) {
		usb_err("usb module not probe\n");
		return 0;
	}

	return ((hisi_dwc3->hifi_ip_first)
				&& (0 == get_hifi_usb_retry_count())
				&& (0 == get_never_hifi_usb_value())
				&& (0 == usbaudio_nv_is_ready()));
}
EXPORT_SYMBOL_GPL(hisi_usb_otg_use_hifi_ip_first);

int hisi_usb_otg_get_typec_orien()
{
	struct hisi_dwc3_device *hisi_dwc3 = hisi_dwc3_dev;

	/*
	 * just check if usb module probe.
	 */
	if (!hisi_dwc3) {
		usb_err("usb module not probe\n");
		return 0;
	}

	return hisi_dwc3->plug_orien;
}
EXPORT_SYMBOL_GPL(hisi_usb_otg_get_typec_orien);

int hisi_usb_otg_event_sync(TCPC_MUX_CTRL_TYPE mode_type, enum otg_dev_event_type event, TYPEC_PLUG_ORIEN_E typec_orien)
{
	int ret = -EINVAL;
	struct hisi_dwc3_device *hisi_dwc3 = hisi_dwc3_dev;

	usb_info("+.\n");

	/*
	 * just check if usb module probe.
	 */
	if (!hisi_dwc3) {
		usb_err("usb module not probe\n");
		return -EBUSY;
	}

	/*
	 * step 0: check if in interrupt context.
	 */
	if (in_interrupt()) {
		usb_err("Hard interrupt context.\n");
		WARN_ON_ONCE(1);
		return ret;
	}

	/*
	 * step 1: check input event.
	 */
	if (NONE_EVENT < event) { /*lint !e685 */
		usb_err("unknow event:%d.\n", event);
		return ret;
	}

	/*
	 * step 2:check eventmask
	 */
	if (hisi_dwc3->eventmask) {
		usb_err("eventmask enabled, sync mask all events.\n");
		return ret;
	}

	mutex_lock(&sync_event_lock);
	dwc3_event_work_set_sync(hisi_dwc3);

	hisi_dwc3->mode_type = mode_type;
	hisi_dwc3->plug_orien = typec_orien;

	ret = hisi_usb_otg_event(event);
	if (0 <= ret) {
		usb_info("start wait.\n");
		ret = 0;
		/* wait time greater than (XHCI_CMD_DEFAULT_TIMEOUT * 2) */
		if(!wait_for_completion_timeout(&hisi_dwc3->event_completion,
						msecs_to_jiffies(10500))) {
			usb_err("usb task timeout!\n");
			ret = -EAGAIN;
		}
	} else {
		usb_err("no need wait [ret=%d]\n", ret);
	}

	dwc3_event_work_clr_sync(hisi_dwc3);
	mutex_unlock(&sync_event_lock);

	usb_info("-.\n");

	return ret;
}
EXPORT_SYMBOL_GPL(hisi_usb_otg_event_sync);

int hisi_dwc3_is_powerdown(void)
{
	int power_flag = get_hisi_dwc3_power_flag();
	return ((USB_POWER_OFF == power_flag) || (USB_POWER_HOLD == power_flag));
}

#ifdef CONFIG_HISI_DEBUG_FS
void usb_start_dump(void)
{
	if (!queue_work(system_power_efficient_wq, &hisi_dwc3_dev->usb_core_reg_dump_work)) {
		usb_err("[USB.DBG] usb linkstate work not run!\n");
	}
}

static void usb_core_reg_dump_work(struct work_struct *work)
{
	struct hisi_dwc3_device *hisi_dwc = container_of(work,
				    struct hisi_dwc3_device, usb_core_reg_dump_work);
	if (!hisi_dwc->core_ops)
		return;

	hisi_dwc->core_ops->link_state_print();
}
#endif


static void hisi_dwc3_cmd_tmo_dbg_print(struct hisi_dwc3_device *hisi_dwc)
{
	usb_dbg("+\n");
	if (hisi_dwc->usb_phy && hisi_dwc->usb_phy->cmd_tmo_dbg_print)
		hisi_dwc->usb_phy->cmd_tmo_dbg_print();
	usb_dbg("-\n");
}

static int device_event_notifier_fn(struct notifier_block *nb,
			unsigned long event, void *para)
{
	struct hisi_dwc3_device *hisi_dwc = container_of(nb,
			struct hisi_dwc3_device, event_nb);
	enum usb_device_speed  speed;
	unsigned long flags;
	usb_dbg("+\n");

	switch (event) {
	case DEVICE_EVENT_CONNECT_DONE:
		speed = *(enum usb_device_speed  *)para;

		/*
		 * Keep VDP_SRC if speed is USB_SPEED_SUPER
		 * and charger_type is CHARGER_TYPE_CDP.
		*/
		if (hisi_dwc->charger_type == CHARGER_TYPE_CDP &&
				speed == USB_SPEED_SUPER)
			hisi_usb_enable_vdp_src(hisi_dwc);
		break;

	case DEVICE_EVENT_PULLUP: /*lint !e142 */
		/* Disable VDP_SRC for communicaton on D+ */
		hisi_usb_disable_vdp_src(hisi_dwc);
		break;

	case DEVICE_EVENT_CMD_TMO: /*lint !e142 */
		hisi_dwc3_cmd_tmo_dbg_print(hisi_dwc);
		break;

	case DEVICE_EVENT_SETCONFIG: /*lint !e142 */
		speed = *(enum usb_device_speed  *)para;
		spin_lock_irqsave(&hisi_dwc->bc_again_lock, flags);/*lint !e550*/
		if (hisi_dwc->charger_type == CHARGER_TYPE_UNKNOWN) {
			hisi_dwc->charger_type = CHARGER_TYPE_SDP;
		}

		hisi_dwc->speed = speed;
		if (!queue_work(system_power_efficient_wq,
						&hisi_dwc->speed_change_work)) {
			usb_err("schedule speed_change_work wait:%d\n", hisi_dwc->speed);
		}

		spin_unlock_irqrestore(&hisi_dwc->bc_again_lock, flags);/*lint !e550*/
		break;

	default:
		break;
	}

	usb_dbg("-\n");
	return 0;
}/*lint !e715*/

/*
 * Huawei st310 is a super-speed mass storage device. A call may cause
 * disconnection. Once it disconnected, force USB as high-speed.
 */
static void huawei_st310_quirk(struct usb_device *udev)
{
	int typec_state = PD_DPM_USB_TYPEC_DETACHED;

#define VID_HUAWEI_ST310 0x12D1
#define PID_HUAWEI_ST310 0x3B40

	if (udev->descriptor.idVendor == VID_HUAWEI_ST310 &&
			udev->descriptor.idProduct == PID_HUAWEI_ST310 &&
			udev->speed == USB_SPEED_SUPER) {
#ifdef CONFIG_TCPC_CLASS
		pd_dpm_get_typec_state(&typec_state);
#endif
		if (typec_state != PD_DPM_USB_TYPEC_DETACHED)
			hisi_usb_otg_event(DISABLE_USB3_PORT);
	}
}

static int xhci_notifier_fn(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct usb_device *udev = (struct usb_device *)data;

	usb_dbg("+\n");

	if (!udev) {
		usb_dbg("udev is null,just return\n");
		return 0;
	}

	if ((action == USB_DEVICE_ADD) && (USB_CLASS_HUB == udev->descriptor.bDeviceClass)) {
		usb_dbg("usb hub don't notify\n");
		return 0;
	}

	if (((action == USB_DEVICE_ADD) || (action == USB_DEVICE_REMOVE))
		&& ((udev->parent != NULL) && (udev->parent->parent == NULL))) {
		usb_dbg("xhci device speed is %d action %s\n", udev->speed,
			(action == USB_DEVICE_ADD)?"USB_DEVICE_ADD":"USB_DEVICE_REMOVE");

		/*only device plug out while phone is host mode,not the usb cable*/
		if (action == USB_DEVICE_REMOVE)
			hisi_dwc3_dev->speed = USB_CONNECT_HOST;
		else
			hisi_dwc3_dev->speed = udev->speed;

		if (action == USB_DEVICE_ADD)
			hisi_dwc3_dev->device_add_time_stamp = jiffies;

		if (!queue_work(system_power_efficient_wq,
						&hisi_dwc3_dev->speed_change_work)) {
			usb_err("schedule speed_change_work wait:%d\n", hisi_dwc3_dev->speed);
		}

		/* disable usb3.0 quirk for Huawei ST310-S1 */
		if (action == USB_DEVICE_REMOVE) {
			huawei_st310_quirk(udev);
		}

	}

	usb_dbg("-\n");
	return 0;
}

/**
 * get_usb_state() - get current USB cable state.
 * @hisi_dwc: the instance pointer of struct hisi_dwc3_device
 *
 * return current USB cable state according to VBUS status and ID status.
 */
static enum usb_state get_usb_state(struct hisi_dwc3_device *hisi_dwc)
{
        if (hisi_dwc->fpga_flag) {
                usb_dbg("this is fpga platform, usb is device mode\n");
                return USB_STATE_DEVICE;
        }

        if (hisi_usb_vbus_value() == 0)
                return USB_STATE_OFF;
        else
                return USB_STATE_DEVICE;
}

static void get_phy_param(struct hisi_dwc3_device *hisi_dwc3)
{
	struct device *dev = &hisi_dwc3->pdev->dev;

	/* hs phy param for device mode */
	if (of_property_read_u32(dev->of_node, "eye_diagram_param",
			&(hisi_dwc3->eye_diagram_param))) {
		usb_dbg("get eye diagram param form dt failed, use default value\n");
		hisi_dwc3->eye_diagram_param = 0x1c466e3;
	}
	usb_dbg("eye diagram param: 0x%x\n", hisi_dwc3->eye_diagram_param);

	/* hs phy param for host mode */
	if (of_property_read_u32(dev->of_node, "eye_diagram_host_param",
			&(hisi_dwc3->eye_diagram_host_param))) {
		usb_dbg("get eye diagram host param form dt failed, use default value\n");
		hisi_dwc3->eye_diagram_host_param = 0x1c466e3;
	}
	usb_dbg("eye diagram host param: 0x%x\n", hisi_dwc3->eye_diagram_host_param);

	/* tx_vboost_lvl */
	if (of_property_read_u32(dev->of_node, "usb3_phy_tx_vboost_lvl",
			&(hisi_dwc3->usb3_phy_tx_vboost_lvl))) {
		usb_dbg("get usb3_phy_tx_vboost_lvl form dt failed, use default value\n");
		hisi_dwc3->usb3_phy_tx_vboost_lvl = VBOOST_LVL_DEFAULT_PARAM;
	}
	usb_dbg("usb3_phy_tx_vboost_lvl: %d\n", hisi_dwc3->usb3_phy_tx_vboost_lvl);

	if (of_property_read_u32(dev->of_node, "vdp_src_disable",
		&(hisi_dwc3->vdp_src_disable))) {
		usb_dbg("get vdp_src_disable form dt failed, use default value\n");
		hisi_dwc3->vdp_src_disable = 0;
	}
	usb_dbg("vdp_src_disable: %d\n", hisi_dwc3->vdp_src_disable);
}

static int hisi_dwc3_phy_init(struct hisi_dwc3_device *hisi_dwc,
		unsigned int host_mode)
{
	unsigned int eye_diagram_param;
	int ret = -ENODEV;

	if (hisi_dwc->use_new_frame) {
		ret = hisi_usb2_phy_init(hisi_dwc->usb2_phy, host_mode);
		if (ret) {
			usb_err("hisi_usb2_phy_init failed\n");
			return ret;
		}

		ret = hisi_usb_controller_init(hisi_dwc);
		if (ret) {
			usb_err("hisi_usb_controller_init failed\n");
			if (hisi_usb2_phy_exit(hisi_dwc->usb2_phy))
				usb_err("hisi_usb2_phy_exit failed\n");
		}

		set_hisi_dwc3_power_flag(USB_POWER_ON);
		hisi_usb_unreset_phy_if_fpga();
	} else if (hisi_dwc->usb_phy->init) {
		if (host_mode)
			eye_diagram_param = hisi_dwc->eye_diagram_host_param;
		else
			eye_diagram_param = hisi_dwc->eye_diagram_param;
		return hisi_dwc->usb_phy->init(hisi_dwc->support_dp,
				eye_diagram_param,
				hisi_dwc->usb3_phy_tx_vboost_lvl);
	}

	return ret;
}

static inline int hisi_dwc3_phy_shutdown(struct hisi_dwc3_device *hisi_dwc)
{
	int ret = -ENODEV;

	if (hisi_dwc->use_new_frame) {
		set_hisi_dwc3_power_flag(USB_POWER_HOLD);
		ret = hisi_usb_controller_exit(hisi_dwc);
		if (ret) {
			usb_err("hisi_usb_controller_exit failed\n");
			return ret;
		}

		ret = hisi_usb2_phy_exit(hisi_dwc->usb2_phy);
		if (ret) {
			usb_err("hisi_usb2_phy_exit failed\n");
			if (hisi_usb_controller_init(hisi_dwc))
				usb_err("hisi_usb_controller_init failed\n");
		}
		set_hisi_dwc3_power_flag(USB_POWER_OFF);
	} else if (hisi_dwc->usb_phy->shutdown) {
		return hisi_dwc->usb_phy->shutdown(hisi_dwc->support_dp);
	}
	return ret;
}

/* Currently this function only called in hifi usb mode */
static int hisi_dwc3_usb20_phy_init(struct hisi_dwc3_device *hisi_dwc,
		unsigned int combophy_flag)
{
	int ret = 0;

	usb_dbg("+\n");
	if (hisi_dwc->use_new_frame) {
		ret = hisi_usb2_shared_phy_init(hisi_dwc->usb2_phy);
		if (ret) {
			usb_err("hisi_usb2_phy_init failed\n");
			return ret;
		}

		ret = hisi_usb_controller_init(hisi_dwc);
		if (ret) {
			usb_err("hisi_usb_controller_init failed\n");
			if (hisi_usb2_phy_exit(hisi_dwc->usb2_phy))
				usb_err("hisi_usb2_phy_exit failed\n");
		}
		hisi_usb_unreset_phy_if_fpga();
	} else if (hisi_dwc->usb_phy->shared_phy_init) {
		return hisi_dwc->usb_phy->shared_phy_init(hisi_dwc->support_dp,
				hisi_dwc->eye_diagram_host_param,
				combophy_flag);
	} else {
		WARN_ON(1);
	}
	usb_dbg("-\n");
	return ret;
}

static int hisi_dwc3_usb20_phy_shutdown(struct hisi_dwc3_device *hisi_dwc,
		unsigned int combophy_flag, unsigned int keep_power)
{
	int ret = 0;

	usb_dbg("+\n");
	if (hisi_dwc->use_new_frame) {
		ret = hisi_usb_controller_exit(hisi_dwc);
		if (ret) {
			usb_err("hisi_usb_controller_exit failed\n");
			return ret;
		}
		ret = hisi_usb2_shared_phy_exit(hisi_dwc->usb2_phy, keep_power);
		if (ret) {
			usb_err("hisi_usb2_shared_phy_exit failed\n");
			if (hisi_usb_controller_init(hisi_dwc))
				usb_err("hisi_usb_controller_init failed\n");
		}
	} else if (hisi_dwc->usb_phy->shared_phy_shutdown) {
		return hisi_dwc->usb_phy->shared_phy_shutdown(hisi_dwc->support_dp,
				combophy_flag, keep_power);
	} else {
		WARN_ON(1);
	}
	usb_dbg("-\n");
	return ret;
}

static void get_resource_for_fpga(struct hisi_dwc3_device *hisi_dwc3)
{
	struct device *dev = &hisi_dwc3->pdev->dev;

	hisi_dwc3->fpga_usb_mode_gpio = -1;
	hisi_dwc3->fpga_otg_drv_vbus_gpio = -1;
	hisi_dwc3->fpga_phy_reset_gpio = -1;
	hisi_dwc3->fpga_phy_switch_gpio = -1;

	if (of_property_read_u32(dev->of_node, "fpga_flag",
			    &(hisi_dwc3->fpga_flag))) {
		hisi_dwc3->fpga_flag = 0;
	}

	if (hisi_dwc3->fpga_flag == 0)
		return;

	usb_dbg("this is fpga platform\n");

	hisi_dwc3->fpga_usb_mode_gpio = of_get_named_gpio(dev->of_node,
			"fpga_usb_mode_gpio", 0);
	hisi_dwc3->fpga_otg_drv_vbus_gpio = of_get_named_gpio(dev->of_node,
			"fpga_otg_drv_vbus_gpio", 0);
	hisi_dwc3->fpga_phy_reset_gpio = of_get_named_gpio(dev->of_node,
			"fpga_phy_reset_gpio", 0);
	hisi_dwc3->fpga_phy_switch_gpio = of_get_named_gpio(dev->of_node,
			"fpga_phy_switch_gpio", 0);

	usb_dbg("fpga usb gpio info:"
		"usb_mode=%d, dr_vbus=%d, phy_reset=%d, phy_switch=%d\n",
			hisi_dwc3->fpga_usb_mode_gpio,
			hisi_dwc3->fpga_otg_drv_vbus_gpio,
			hisi_dwc3->fpga_phy_reset_gpio,
			hisi_dwc3->fpga_phy_switch_gpio);
}

static void get_quirks_dts(struct hisi_dwc3_device *hisi_dwc3)
{
	struct device *dev = &hisi_dwc3->pdev->dev;

	hisi_dwc3->quirk_enable_hst_imm_retry = device_property_read_bool(dev,
				"quirk_enable_hst_imm_retry");
	hisi_dwc3->quirk_disable_rx_thres_cfg = device_property_read_bool(dev,
				"quirk_disable_rx_thres_cfg");
	hisi_dwc3->quirk_disable_usb2phy_suspend = device_property_read_bool(dev,
				"quirk_disable_usb2phy_suspend");
	hisi_dwc3->quirk_clear_svc_opp_per_hs = device_property_read_bool(dev,
				"quirk_clear_svc_opp_per_hs");
	hisi_dwc3->quirk_set_svc_opp_per_hs_sep = device_property_read_bool(dev,
				"quirk_set_svc_opp_per_hs_sep");
	hisi_dwc3->quirk_adjust_dtout = device_property_read_bool(dev,
				"quirk_adjust_dtout");
	hisi_dwc3->quirk_force_disable_host_lpm = device_property_read_bool(dev,
				"quirk_force_disable_host_lpm");
	hisi_dwc3->quirk_enable_p4_gate = device_property_read_bool(dev,
				"quirk_enable_p4_gate");
}

/**
 * get_resource() - prepare resources
 * @hisi_dwc3: the instance pointer of struct hisi_dwc3_device
 *
 * 1. get registers base address and map registers region.
 * 2. get regulator handler.
 */
static int get_resource(struct hisi_dwc3_device *hisi_dwc3)
{
	struct device *dev = &hisi_dwc3->pdev->dev;

	get_phy_param(hisi_dwc3);

	get_resource_for_fpga(hisi_dwc3);

	get_quirks_dts(hisi_dwc3);

	if (of_property_read_u32(dev->of_node, "dma_mask_bit",
				&(hisi_dwc3->dma_mask_bit))) {
		hisi_dwc3->dma_mask_bit = 32;
	}

	if (of_property_read_u32(dev->of_node, "hifi_ip_first",
				&(hisi_dwc3->hifi_ip_first))) {
		hisi_dwc3->hifi_ip_first = 0;
	}

#ifdef CONFIG_CONTEXTHUB_PD
	if (of_property_read_u32(dev->of_node, "usb_support_dp",
				&(hisi_dwc3->support_dp))) {
		usb_err("usb driver not support dp\n");
		hisi_dwc3->support_dp = 0;
	}
#else
	hisi_dwc3->support_dp = 0;
#endif

	if (of_property_read_u32(dev->of_node, "usb_support_check_voltage",
				&(hisi_dwc3->check_voltage))) {
		usb_err("usb driver not support check voltage\n");
		hisi_dwc3->check_voltage = 0;
	}

	if (of_property_read_u32(dev->of_node, "set_hi_impedance",
				&(hisi_dwc3->set_hi_impedance))) {
		usb_err("usb driver not support set_hi_impedance\n");
		hisi_dwc3->set_hi_impedance = 0;
	}

	return 0;
}

static void request_gpio(int *gpio)
{
	int ret;

	if (*gpio < 0)
		return;

	ret = gpio_request((unsigned)(*gpio), NULL);
	if (ret) {
		usb_err("request gpio %d failed\n", *gpio);
		*gpio = -1;
	}
}

static void request_gpios_for_fpga(struct hisi_dwc3_device *hisi_dwc)
{
	request_gpio(&hisi_dwc->fpga_usb_mode_gpio);
	request_gpio(&hisi_dwc->fpga_otg_drv_vbus_gpio);
	request_gpio(&hisi_dwc->fpga_phy_reset_gpio);
	request_gpio(&hisi_dwc->fpga_phy_switch_gpio);
}

int hisi_usb_dwc3_register_phy(const struct hisi_usb_phy *phy)
{
	if (g_hisi_usb_phy)
		return -EBUSY;

	if (!phy)
		return -EINVAL;

	g_hisi_usb_phy = phy;

	return 0;
}

int hisi_usb_dwc3_unregister_phy(const struct hisi_usb_phy *phy)
{
	if (g_hisi_usb_phy != phy)
		return -EINVAL;

	g_hisi_usb_phy = NULL;

	return 0;
}

static int hisi_dwc3_init_state(struct hisi_dwc3_device *hisi_dwc)
{
	int ret = 0;

	/* default device state  */
	hisi_dwc->state = USB_STATE_DEVICE;

#ifdef CONFIG_USB_DWC3_DUAL_ROLE
	if (hisi_dwc->fpga_flag != 0) {
		/* if vbus is on, detect charger type */
		if (hisi_usb_vbus_value()) {
			hisi_dwc->charger_type =
				hisi_usb_detect_charger_type(hisi_dwc);
			notify_charger_type(hisi_dwc);
		}

		if (sleep_allowed(hisi_dwc))
			hisi_dwc3_wake_unlock(hisi_dwc);
		else
			hisi_dwc3_wake_lock(hisi_dwc);

		if (enumerate_allowed(hisi_dwc)) {
			/* start peripheral */
			ret = extcon_set_state_sync(hisi_dwc->edev,
					EXTCON_USB, true);
			if (ret) {
				hisi_dwc3_wake_unlock(hisi_dwc);
				usb_err("start peripheral error\n");
				return ret;
			}
		}

		hisi_dwc->event = CHARGER_CONNECT_EVENT;

		if (get_usb_state(hisi_dwc) == USB_STATE_OFF) {
			usb_dbg("init state: OFF\n");
			hisi_usb_otg_event(CHARGER_DISCONNECT_EVENT);
		}
	} else {
		if (!hisi_usb_vbus_value()) {
			hisi_dwc->charger_type = CHARGER_TYPE_NONE;
			/* vdp src may enable in fastboot, force disable */
			hisi_dwc->vdp_src_enable = 1;
			hisi_usb_disable_vdp_src(hisi_dwc);
		}
		hisi_dwc->state = USB_STATE_OFF;
		hisi_dwc->event = CHARGER_DISCONNECT_EVENT;

		ret = hisi_dwc3_phy_shutdown(hisi_dwc);
		if (ret)
			usb_err("hisi_dwc3_phy_shutdown failed (ret %d)\n", ret);

		hisi_dwc3_wake_unlock(hisi_dwc);
	}
#endif
	return ret;
}

static int hisi_dwc3_remove_child(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);

	if (data)
		usb_dbg("unused data not NULL!\n");
	platform_device_unregister(pdev);
	return 0;
}

static int hisi_usb_get_hardware(struct hisi_dwc3_device *hisi_usb)
{
	struct platform_device *pdev = hisi_usb->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;

	hisi_usb->notify_speed = of_property_read_bool(node,
			"notify_speed");

	hisi_usb->use_new_frame = of_property_read_bool(node,
			"use_new_frame");

	if (hisi_usb->use_new_frame) {
		return hisi_usb_get_hw_res(hisi_usb, dev);
	} else {
		if (!g_hisi_usb_phy || !g_hisi_usb_phy->otg_bc_reg_base) {
			usb_err("phy is NULL\n");
			return -EPROBE_DEFER;
		}

		hisi_usb->usb_phy = g_hisi_usb_phy;
	}

	return 0;
}

static int hisi_dwc3_probe(struct platform_device *pdev)
{
	int ret;
	struct hisi_dwc3_device *hisi_dwc = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;

	usb_dbg("+\n");

	/* [first] check arg & create dwc control struct */
	hisi_dwc = devm_kzalloc(dev, sizeof(*hisi_dwc), GFP_KERNEL);
	if (!hisi_dwc) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, hisi_dwc);
	hisi_dwc->pdev = pdev;

	ret = hisi_usb_get_hardware(hisi_dwc);
	if (ret) {
		usb_err("get hardware failed ret %d\n", ret);
		return ret;
	}

	hisi_dwc->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(hisi_dwc->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		ret = PTR_ERR(hisi_dwc->edev); /*lint !e429*/
		goto put_hw_res;
	}

	ret = devm_extcon_dev_register(dev, hisi_dwc->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		goto put_hw_res;
	}

	hisi_dwc3_dev = hisi_dwc;

	ret = get_resource(hisi_dwc);
	if (ret) {
		dev_err(&pdev->dev, "get resource failed!\n");
		goto err_set_dwc3_null;
	}
	hisi_dwc->core_ops = get_usb3_core_ops();
	request_gpios_for_fpga(hisi_dwc);

	hisi_dwc->dma_mask_bit = hisi_dwc->dma_mask_bit > 64 ?
					64 : hisi_dwc->dma_mask_bit;
	dev->coherent_dma_mask = DMA_BIT_MASK(hisi_dwc->dma_mask_bit);
	dev->dma_mask = &dev->coherent_dma_mask;

	/* create sysfs files. */
	ret = create_attr_file(dev);
	if (ret) {
		dev_err(&pdev->dev, "create_attr_file failed!\n");
		goto err_set_dwc3_null;
	}

	/* initialize */
	hisi_dwc->charger_type = CHARGER_TYPE_SDP;
	hisi_dwc->fake_charger_type = CHARGER_TYPE_NONE;
	hisi_dwc->event = NONE_EVENT;
	hisi_dwc->eventmask = 0;
	hisi_dwc->is_hanle_event_sync = 0;
	hisi_dwc->mode_type = TCPC_NC;
	spin_lock_init(&hisi_dwc->event_lock);
	INIT_WORK(&hisi_dwc->event_work, event_work);
	INIT_WORK(&hisi_dwc->speed_change_work, hisi_dwc3_speed_change_work);
	mutex_init(&hisi_dwc->lock);
	wakeup_source_init(&hisi_dwc->wake_lock, "usb_wake_lock");
	event_queue_creat(&hisi_dwc->event_queue, MAX_EVENT_COUNT);
	init_completion(&hisi_dwc->event_completion);
	hisi_dwc->xhci_nb.notifier_call = xhci_notifier_fn;
	usb_register_notify(&hisi_dwc->xhci_nb);
	hisi_dwc->event_nb.notifier_call = device_event_notifier_fn;
	ret = dwc3_device_event_notifier_register(&hisi_dwc->event_nb);
	if (ret) {
		usb_err("dwc3_device_event_notifier_register failed\n");
		goto err_remove_attr;
	}

#ifdef CONFIG_HISI_DEBUG_FS
	INIT_WORK(&hisi_dwc->usb_core_reg_dump_work, usb_core_reg_dump_work);
#endif /* hisi debug */

	/*
	 * enable runtime pm.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		usb_err("hisi_dwc3 pm_runtime_get_sync failed %d\n", ret);
		goto err_notifier_unregister;
	}

	pm_runtime_forbid(dev);

	ret = hisi_usb_bc_init(hisi_dwc);
	if (ret) {
		usb_err("hisi_usb_bc_init failed\n");
		goto err_pm_put;
	}

	/* power on */
	ret = hisi_dwc3_phy_init(hisi_dwc, 0);
	if (ret) {
		usb_err("hisi_dwc3_phy_init failed!\n");
		goto err_bc_exit;
	}

	/*
	 * probe child deivces
	 */
	ret = of_platform_populate(node, NULL, NULL, dev);
	if (ret) {
		usb_err("register dwc3 failed!\n");
		goto err_phy_exit;
	}

	ret = hisi_dwc3_init_state(hisi_dwc);
	if (ret) {
		usb_err("hisi_dwc3_init_state failed!\n");
		goto err_remove_child;
	}

	pm_runtime_allow(dev);
	usb_dbg("-\n");

	return 0;

err_remove_child:
	device_for_each_child(dev, NULL, hisi_dwc3_remove_child);

err_phy_exit:
	if (hisi_dwc3_phy_shutdown(hisi_dwc))
		usb_err("hisi_dwc3_phy_shutdown failed\n");

err_bc_exit:
	hisi_usb_bc_exit(hisi_dwc);

err_pm_put:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

err_notifier_unregister:
	dwc3_device_event_notifier_unregister(&hisi_dwc->event_nb);
	hisi_dwc->event_nb.notifier_call = NULL;

err_remove_attr:
	event_queue_destroy(&hisi_dwc->event_queue);
	remove_attr_file(dev);

err_set_dwc3_null:
	hisi_dwc3_dev = NULL;

put_hw_res:
	hisi_usb_put_hw_res(hisi_dwc);

	return ret;
}

static int hisi_dwc3_remove(struct platform_device *pdev)
{
	struct hisi_dwc3_device *hisi_dwc3 = platform_get_drvdata(pdev);
	int ret;

	if (!hisi_dwc3) {
		usb_err("hisi_dwc3 NULL\n");
		return -EBUSY;
	}

	device_for_each_child(&pdev->dev, NULL, hisi_dwc3_remove_child);

	hisi_usb_bc_exit(hisi_dwc3);

	usb_unregister_notify(&hisi_dwc3->xhci_nb);
	ret = hisi_dwc3_phy_shutdown(hisi_dwc3);
	if (ret) {
		usb_err("hisi_dwc3_phy_shutdown error\n");
	}
	hisi_dwc3->usb_phy = NULL;

	event_queue_destroy(&hisi_dwc3->event_queue);

	hisi_usb_put_hw_res(hisi_dwc3);

	remove_attr_file(&pdev->dev);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
/*lint -save -e454 -e455 */
static int hisi_dwc3_prepare(struct device *dev)
{
	struct hisi_dwc3_device *hisi_dwc = platform_get_drvdata(
				to_platform_device(dev));
	int ret = 0;

	usb_dbg("+\n");

	if (!hisi_dwc)
		return -ENODEV;

	mutex_lock(&hisi_dwc->lock);

	switch (hisi_dwc->state) {
	case USB_STATE_OFF:
		usb_info("off state.\n");
		break;
	case USB_STATE_DEVICE:
		usb_info("device state.\n");

		if (!enumerate_allowed(hisi_dwc)) {
			usb_dbg("connected is a real charger\n");
			hisi_usb_disable_vdp_src(hisi_dwc);
		}

		break;
	case USB_STATE_HOST:
		usb_dbg("host mode, should not go to sleep!\n");
		ret = 0;
		break;
	case USB_STATE_HIFI_USB:
	case USB_STATE_HIFI_USB_HIBERNATE:
		break;
	default:
		usb_err("illegal state!\n");
		ret = -EFAULT;
		goto error;
	}

	usb_dbg("-\n");
	return ret;
error:
	mutex_unlock(&hisi_dwc->lock);
	return ret;
}

static void hisi_dwc3_complete(struct device *dev)
{
	struct hisi_dwc3_device *hisi_dwc = platform_get_drvdata(
				to_platform_device(dev));
	usb_dbg("+\n");

	if (!hisi_dwc) {
		usb_err("hisi_dwc NULL !\n");
		return;
	}

	switch (hisi_dwc->state) {
	case USB_STATE_OFF:
		usb_dbg("off state.\n");
		break;
	case USB_STATE_DEVICE:
		usb_dbg("device state.charger_type[%d]\n", hisi_dwc->charger_type);

		if (sleep_allowed(hisi_dwc))
			hisi_dwc3_wake_unlock(hisi_dwc);
		else
			hisi_dwc3_wake_lock(hisi_dwc);

		/* do not start peripheral if real charger connected */
		if (!enumerate_allowed(hisi_dwc))
			usb_dbg("a real charger connected\n");

		break;
	case USB_STATE_HOST:
		usb_err("host mode, should not go to sleep!\n");
		break;
	case USB_STATE_HIFI_USB:
	case USB_STATE_HIFI_USB_HIBERNATE:
		/* keep audio usb power on */
		break;
	default:
		usb_err("illegal state!\n");
		break;
	}

	mutex_unlock(&hisi_dwc->lock);
	usb_dbg("-\n");
}
/*lint -restore */

static int hisi_dwc3_suspend(struct device *dev)
{
	struct hisi_dwc3_device *hisi_dwc3 = platform_get_drvdata(to_platform_device(dev));
	int ret = 0;

	usb_dbg("+\n");

	if (!hisi_dwc3) {
		usb_err("hisi_dwc3 NULL\n");
		return -EBUSY;
	}

	if (USB_STATE_DEVICE == hisi_dwc3->state) {
		ret = hisi_dwc3_phy_shutdown(hisi_dwc3);
		if (ret)
			usb_err("hisi_dwc3_phy_shutdown failed\n");
	} else {
		usb_dbg("hisi_dwc3 in state %s\n",
				hisi_usb_state_string(hisi_dwc3->state));
	}

	usb_dbg("-\n");

	return ret;
}

static int hisi_dwc3_resume(struct device *dev)
{
	struct hisi_dwc3_device *hisi_dwc3 = platform_get_drvdata(to_platform_device(dev));
	int ret = 0;

	usb_dbg("+\n");

	if (!hisi_dwc3) {
		usb_err("hisi_dwc3 NULL\n");
		return -EBUSY;
	}

	if (USB_STATE_DEVICE == hisi_dwc3->state) {
		ret = hisi_dwc3_phy_init(hisi_dwc3, 0);
		if (ret)
			usb_err("hisi_dwc3_phy_init failed\n");

		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
	} else {
		usb_dbg("hisi_dwc3 in state %s\n",
				hisi_usb_state_string(hisi_dwc3->state));
	}

	usb_dbg("-\n");

	return ret;
}
#endif

static int hisi_dwc3_runtime_suspend(struct device *dev)
{
	usb_dbg("+\n");

	return 0;
}

static int hisi_dwc3_runtime_resume(struct device *dev)
{
	usb_dbg("+\n");

	return 0;
}

static int hisi_dwc3_runtime_idle(struct device *dev)
{
	usb_dbg("+\n");

	return 0;
}
#endif

const struct dev_pm_ops hisi_dwc3_dev_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.prepare	= hisi_dwc3_prepare,
	.complete	= hisi_dwc3_complete,
#endif
	SET_SYSTEM_SLEEP_PM_OPS(hisi_dwc3_suspend, hisi_dwc3_resume)
	SET_RUNTIME_PM_OPS(hisi_dwc3_runtime_suspend, hisi_dwc3_runtime_resume,
			hisi_dwc3_runtime_idle)
};

static const struct of_device_id hisi_dwc3_match[] = {
	{ .compatible = "hisilicon,dwc3-usb" },
	{},
};
MODULE_DEVICE_TABLE(of, dwc3_may_match);

static struct platform_driver hisi_dwc3_driver = {
	.probe		= hisi_dwc3_probe,
	.remove		= hisi_dwc3_remove,
	.driver		= {
		.name	= "hisi-dwc3-usb",
		.of_match_table = of_match_ptr(hisi_dwc3_match),
		.pm	= &hisi_dwc3_dev_pm_ops,
	},
};
module_platform_driver(hisi_dwc3_driver);
