/*
 * hi64xx_vad.c -- hi64xx vad driver
 *
 * Copyright (c) 2014 Hisilicon Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mutex.h>
#include <sound/soc.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/hisi/hi64xx/hi64xx_vad.h>
#include <linux/hisi/hi64xx/hi64xx_irq.h>
#include <hi64xx_algo_interface.h>
#include <linux/hisi/hi64xx/hi64xx_regs.h>
#include <linux/hisi/hi64xx/hi64xx_mad.h>
#include "soundtrigger_event.h"

#define HI6402_DSP_VAD_CMD  (BASE_ADDR_PAGE_CFG + 0x73)

struct hi64xx_vad_platform_data {
	struct snd_soc_codec	*codec;;
	struct hi64xx_irq		*irq;
	struct wakeup_source	soundtrigger_wake_lock;
	unsigned short			fast_mode_enable;
};

static struct hi64xx_vad_platform_data *vad_data = NULL;

static irqreturn_t hi64xx_sound_trigger_handler(int irq, void *data)
{
	struct hi64xx_vad_platform_data *pdata =
		(struct hi64xx_vad_platform_data *)data;
	unsigned int soundtrigger_event = 0;

	__pm_wakeup_event(&pdata->soundtrigger_wake_lock, 1000);

	/* clr VAD INTR */
	snd_soc_write(pdata->codec, HI64XX_VAD_INT_SET, 0);
	snd_soc_write(pdata->codec, HI64XX_REG_IRQ_1, 1<<HI64XX_VAD_BIT);

	soundtrigger_event = snd_soc_read(pdata->codec, HI6402_DSP_VAD_CMD);
	pr_info("%s,soundtrigger_event = %d, fast_mode_enable = %d\n",
		__FUNCTION__, soundtrigger_event, vad_data->fast_mode_enable);

	if (0 == vad_data->fast_mode_enable) {
		hw_soundtrigger_event_input(soundtrigger_event);
		pr_info("soundtrigger input[%d].\n",soundtrigger_event);
	} else {
		hw_soundtrigger_event_uevent(soundtrigger_event);
		pr_info("soundtrigger uevent[%d].\n",soundtrigger_event);
	}

	return IRQ_HANDLED;
}

int hi64xx_fast_mode_set(unsigned short enable)
{
	if (!vad_data) {
		pr_err("vad data is null\n");
		return -EINVAL;
	}

	vad_data->fast_mode_enable = enable;

	return 0;
}
EXPORT_SYMBOL(hi64xx_fast_mode_set);

int hi64xx_vad_init(struct snd_soc_codec *codec, struct hi64xx_irq *irq)
{
	int ret = 0;

	pr_info("%s enter\n",__FUNCTION__);

	vad_data = (struct hi64xx_vad_platform_data*)kzalloc(sizeof(struct hi64xx_vad_platform_data), GFP_KERNEL);
	if (NULL == vad_data) {
		pr_err("cannot allocate hisi 6402 vad platform data\n");
		return -ENOMEM;
	}

	vad_data->codec = codec;
	vad_data->irq = irq;
	vad_data->fast_mode_enable = 0;

	wakeup_source_init(&vad_data->soundtrigger_wake_lock, "hisi-6402-soundtrigger");

	/* irq request : sound triger */
	ret = hi64xx_irq_request_irq(vad_data->irq, IRQ_VAD, hi64xx_sound_trigger_handler, "sound_triger", vad_data);
	if (0 > ret) {
		pr_err("request irq for sound triger err\n");
	}

	pr_info("%s : hi64xx vad probe ok \n", __FUNCTION__);

	return ret;
}
EXPORT_SYMBOL(hi64xx_vad_init);

int hi64xx_vad_deinit()
{
	if (!vad_data)
		return -EINVAL;

	wakeup_source_trash(&vad_data->soundtrigger_wake_lock);

	if (vad_data->irq)
		hi64xx_irq_free_irq(vad_data->irq, IRQ_VAD, vad_data);

	kfree(vad_data);
	vad_data = NULL;

	return 0;
}
EXPORT_SYMBOL(hi64xx_vad_deinit);

MODULE_DESCRIPTION("hi64xx_vad driver");
MODULE_LICENSE("GPL");
