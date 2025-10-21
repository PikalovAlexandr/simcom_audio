// SPDX-License-Identifier: GPL-2.0
/*
 * SIMCom SIM7600 USB Audio bridge to ALSA
 *
 * Adapted for Linux 4.4–4.9 (Android 10, RK3399)
 *
 * Provides ALSA PCM interface to USB bulk endpoints of SIM7600 modem.
 * Playback (OUT endpoint 0x05)  - uplink to modem
 * Capture  (IN  endpoint 0x84)  - downlink from modem
 *
 * PCM format: 8 kHz, 16-bit mono
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#define SIMCOM_VENDOR_ID  0x1E0E
#define SIMCOM_PRODUCT_ID 0x9001

#define SIMCOM_PCM_EP_OUT 0x05  /* bulk OUT to modem (uplink) */
#define SIMCOM_PCM_EP_IN  0x84  /* bulk IN from modem (downlink) */

#define SIMCOM_PCM_RATE   8000
#define SIMCOM_PCM_CHANS  1
#define SIMCOM_PCM_BITS   16

struct simcom_audio {
	struct usb_device *udev;
	struct snd_card *card;
	struct snd_pcm *pcm;
	int running;
	spinlock_t lock;
};

/* ========================================================= */
/* PCM OPERATIONS                                            */
/* ========================================================= */

static int simcom_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw = (struct snd_pcm_hardware) {
		.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rates = SNDRV_PCM_RATE_8000,
		.rate_min = SIMCOM_PCM_RATE,
		.rate_max = SIMCOM_PCM_RATE,
		.channels_min = SIMCOM_PCM_CHANS,
		.channels_max = SIMCOM_PCM_CHANS,
		.buffer_bytes_max = 64 * 1024,
		.period_bytes_min = 320,
		.period_bytes_max = 4096,
		.periods_min = 2,
		.periods_max = 64,
	};

	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	return 0;
}

static int simcom_pcm_close(struct snd_pcm_substream *substream)
{
	return 0;
}

static int simcom_pcm_hw_params(struct snd_pcm_substream *s,
				struct snd_pcm_hw_params *params)
{
	return snd_pcm_lib_malloc_pages(s, params_buffer_bytes(params));
}

static int simcom_pcm_hw_free(struct snd_pcm_substream *s)
{
	return snd_pcm_lib_free_pages(s);
}

/* Simple blocking I/O loops for prototype use */
static int simcom_pcm_playback(struct snd_pcm_substream *substream)
{
	struct simcom_audio *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int xfer;
	int ret;

	while (chip->running) {
		u8 *buf = runtime->dma_area;
		xfer = frames_to_bytes(runtime, runtime->period_size);

		ret = usb_bulk_msg(chip->udev,
				   usb_sndbulkpipe(chip->udev, SIMCOM_PCM_EP_OUT),
				   buf, xfer, &xfer, 1000);
		if (ret)
			break;

		snd_pcm_period_elapsed(substream);
	}
	return 0;
}

static int simcom_pcm_capture(struct snd_pcm_substream *substream)
{
	struct simcom_audio *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int xfer;
	int ret;

	while (chip->running) {
		u8 *buf = runtime->dma_area;
		xfer = frames_to_bytes(runtime, runtime->period_size);

		ret = usb_bulk_msg(chip->udev,
				   usb_rcvbulkpipe(chip->udev, SIMCOM_PCM_EP_IN),
				   buf, xfer, &xfer, 1000);
		if (ret)
			break;

		snd_pcm_period_elapsed(substream);
	}
	return 0;
}

static int simcom_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct simcom_audio *chip = snd_pcm_substream_chip(substream);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		chip->running = 1;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			simcom_pcm_playback(substream);
		else
			simcom_pcm_capture(substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		chip->running = 0;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t simcom_pcm_pointer(struct snd_pcm_substream *s)
{
	return 0;
}

static int simcom_pcm_prepare(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    snd_printk(KERN_INFO "simcom_audio: prepare stream %d\n", substream->stream);
    chip->running = 0;
    return 0;
}



static const struct snd_pcm_ops simcom_pcm_ops = {
	.open      = simcom_pcm_open,
	.close     = simcom_pcm_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = simcom_pcm_hw_params,
	.hw_free   = simcom_pcm_hw_free,
	.prepare   = simcom_pcm_prepare,
	.trigger   = simcom_pcm_trigger,
	.pointer   = simcom_pcm_pointer,
};

/* ========================================================= */
/* USB DRIVER CORE                                           */
/* ========================================================= */

static int simcom_audio_probe(struct usb_interface *intf,
			      const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct simcom_audio *chip;
	struct snd_card *card;
	int ret;

	ret = snd_card_new(&intf->dev, -1, "SIMCOM", THIS_MODULE,
			   sizeof(*chip), &card);
	if (ret < 0)
		return ret;

	chip = card->private_data;
	chip->udev = usb_get_dev(udev);
	chip->card = card;
	spin_lock_init(&chip->lock);

	strcpy(card->driver, "simcom_audio");
	strcpy(card->shortname, "SIMCom USB Audio");
	strcpy(card->longname, "SIMCom 7600 USB PCM (8kHz/16bit)");

	ret = snd_pcm_new(card, "SIMCom PCM", 0, 1, 1, &chip->pcm);
	if (ret < 0)
		goto error;

	snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_PLAYBACK, &simcom_pcm_ops);
	snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_CAPTURE, &simcom_pcm_ops);

	/* Preallocate DMA buffer for all substreams (legacy API) */
	snd_pcm_lib_preallocate_pages_for_all(chip->pcm,
		SNDRV_DMA_TYPE_CONTINUOUS,
		snd_dma_continuous_data(GFP_KERNEL),
		64*1024, 64*1024);

	ret = snd_card_register(card);
	if (ret == 0) {
		dev_info(&intf->dev, "SIMCom USB Audio registered\n");
		usb_set_intfdata(intf, chip);
		return 0;
	}

error:
	snd_card_free(card);
	return ret;
}

static void simcom_audio_disconnect(struct usb_interface *intf)
{
	struct simcom_audio *chip = usb_get_intfdata(intf);
	if (chip && chip->card)
		snd_card_free(chip->card);
	dev_info(&intf->dev, "SIMCom USB Audio disconnected\n");
}

static const struct usb_device_id simcom_audio_id[] = {
	{ USB_DEVICE(SIMCOM_VENDOR_ID, SIMCOM_PRODUCT_ID) },
	{}
};
MODULE_DEVICE_TABLE(usb, simcom_audio_id);

static struct usb_driver simcom_audio_driver = {
	.name = "simcom_audio",
	.probe = simcom_audio_probe,
	.disconnect = simcom_audio_disconnect,
	.id_table = simcom_audio_id,
};

module_usb_driver(simcom_audio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexandr Pikalov");
MODULE_DESCRIPTION("SIMCom 7600 USB Audio ALSA driver");
