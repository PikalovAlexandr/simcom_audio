// SPDX-License-Identifier: GPL-2.0
/*
 * SIMCom USB Audio Driver (patched version)
 * for SIM7600 / SIM7500 / SIM7100 modules
 *
 * - Compatible with ALSA core for Android 10 on rk3399
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#define SIMCOM_USB_VENDOR_ID  0x1E0E
#define SIMCOM_USB_PRODUCT_ID 0x9001

struct simcom_audio {
	struct usb_device *udev;
	struct snd_card *card;
	struct snd_pcm *pcm;
	int audio_enabled;
	struct mutex lock;
};

static struct snd_pcm_hardware simcom_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
	.rate_min = 8000,
	.rate_max = 16000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 64 * 1024,
	.period_bytes_min = 160,
	.period_bytes_max = 1024,
	.periods_min = 2,
	.periods_max = 128,
};

static int simcom_pcm_open(struct snd_pcm_substream *substream)
{
	struct simcom_audio *dev = snd_pcm_substream_chip(substream);
	dev_info(&dev->udev->dev, "PCM open: stream=%s\n",
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");
	substream->runtime->hw = simcom_pcm_hw;
	return 0;
}

static int simcom_pcm_close(struct snd_pcm_substream *substream)
{
	struct simcom_audio *dev = snd_pcm_substream_chip(substream);
	dev_info(&dev->udev->dev, "PCM close: stream=%s\n",
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");
	return 0;
}

static int simcom_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int simcom_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int simcom_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct simcom_audio *dev = snd_pcm_substream_chip(substream);
	dev_info(&dev->udev->dev, "PCM prepare: %s\n",
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");
	return 0;
}

static int simcom_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct simcom_audio *dev = snd_pcm_substream_chip(substream);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		dev_info(&dev->udev->dev, "PCM trigger START (%s)\n",
			substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		dev_info(&dev->udev->dev, "PCM trigger STOP (%s)\n",
			substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t simcom_pcm_pointer(struct snd_pcm_substream *substream)
{
	return 0;
}

static const struct snd_pcm_ops simcom_pcm_ops = {
	.open = simcom_pcm_open,
	.close = simcom_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = simcom_pcm_hw_params,
	.hw_free = simcom_pcm_hw_free,
	.prepare = simcom_pcm_prepare,
	.trigger = simcom_pcm_trigger,
	.pointer = simcom_pcm_pointer,
};

static ssize_t audio_enabled_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct simcom_audio *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->audio_enabled);
}

static ssize_t audio_enabled_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct simcom_audio *chip = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&chip->lock);
	chip->audio_enabled = !!val;
	dev_info(dev, "Audio %s via sysfs\n", val ? "ENABLED" : "DISABLED");
	mutex_unlock(&chip->lock);

	return count;
}

static DEVICE_ATTR_RW(audio_enabled);

static int simcom_audio_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct simcom_audio *chip;
	struct snd_card *card;
	int err;

	dev_info(&interface->dev,
		 "Probing SIMCom USB Audio device (VID:PID %04x:%04x)\n",
		 id->idVendor, id->idProduct);

	err = snd_card_new(&interface->dev, -1, NULL, THIS_MODULE,
			   sizeof(*chip), &card);
	if (err < 0)
		return err;

	chip = card->private_data;
	chip->udev = usb_get_dev(interface_to_usbdev(interface));
	chip->card = card;
	mutex_init(&chip->lock);

	snprintf(card->driver, sizeof(card->driver), "SIMCOM-Audio");
	snprintf(card->shortname, sizeof(card->shortname), "SIM7600 Voice");
	snprintf(card->longname, sizeof(card->longname),
		 "SIMCom SIM7600 USB Audio Modem at bus %d device %d",
		 chip->udev->bus->busnum, chip->udev->devnum);

	dev_info(&interface->dev, "Creating PCM (playback=1, capture=1)\n");

	err = snd_pcm_new(card, "SIM7600 PCM", 0, 1, 1, &chip->pcm);
	if (err < 0) {
		dev_err(&interface->dev,
			"Failed to create PCM device: %d\n", err);
		snd_card_free(card);
		return err;
	}

	/* --- ВАЖНО: создаём оба потока явно --- */
	snd_pcm_new_stream(chip->pcm, SNDRV_PCM_STREAM_PLAYBACK, 1);
	snd_pcm_new_stream(chip->pcm, SNDRV_PCM_STREAM_CAPTURE, 1);
   
    // Назначаем операции для обоих направлений 
	snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_PLAYBACK, &simcom_pcm_ops);
	snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_CAPTURE,  &simcom_pcm_ops);

	chip->pcm->private_data = chip;

	dev_info(&interface->dev,
		 "PCM streams registered: playback & capture\n");

	/* Регистрируем карту, чтобы card->card_dev стал валиден */
	err = snd_card_register(card);
	if (err < 0) {
		dev_err(&interface->dev,
			"snd_card_register failed: %d\n", err);
		snd_card_free(card);
		return err;
	}

	/* Теперь можно создавать sysfs-атрибут */
	err = device_create_file(&card->card_dev, &dev_attr_audio_enabled);
	if (err)
		dev_warn(&interface->dev,
			 "Failed to create sysfs audio_enabled: %d\n", err);

	usb_set_intfdata(interface, chip);

	dev_info(&interface->dev,
		 "SIMCom Audio driver loaded successfully\n");

	return 0;
}


static void simcom_audio_disconnect(struct usb_interface *interface)
{
	struct simcom_audio *chip = usb_get_intfdata(interface);

	dev_info(&interface->dev, "Disconnecting SIMCom Audio\n");
	if (!chip)
		return;

	device_remove_file(&chip->card->card_dev, &dev_attr_audio_enabled);
	snd_card_free(chip->card);
	usb_set_intfdata(interface, NULL);
	usb_put_dev(chip->udev);
}

static const struct usb_device_id simcom_audio_id_table[] = {
	{ USB_DEVICE(SIMCOM_USB_VENDOR_ID, SIMCOM_USB_PRODUCT_ID) },
	{}
};
MODULE_DEVICE_TABLE(usb, simcom_audio_id_table);

static struct usb_driver simcom_audio_driver = {
	.name = "simcom_audio",
	.probe = simcom_audio_probe,
	.disconnect = simcom_audio_disconnect,
	.id_table = simcom_audio_id_table,
};

module_usb_driver(simcom_audio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexandr_Pikalov");
MODULE_DESCRIPTION("SIMCom USB Audio driver");
