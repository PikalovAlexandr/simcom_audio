// SPDX-License-Identifier: GPL-2.0
/*
 * SIMCom SIM7600 USB Audio bridge to ALSA
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <linux/jiffies.h>

#define SIMCOM_VENDOR_ID  0x1E0E
#define SIMCOM_PRODUCT_ID 0x9001

#define SIMCOM_PCM_EP_OUT 0x05
#define SIMCOM_PCM_EP_IN  0x88

#define SIMCOM_PCM_RATE   8000
#define SIMCOM_PCM_CHANS  1
#define SIMCOM_PCM_BITS   16

#define NUM_URBS 4

struct simcom_urb_context {
    struct snd_pcm_substream *substream;
    unsigned int offset;  // current byte offset in runtime->dma_area for this URB
    void *buf;            // coherent buffer for USB transfer
    dma_addr_t dma;       // DMA address for coherent buffer
    unsigned int period_bytes; // cached period size in bytes for this URB
};

struct simcom_audio {
    struct usb_device *udev;
    struct snd_card *card;
    struct snd_pcm *pcm;
    
    struct urb *urb_in[NUM_URBS];
    struct urb *urb_out[NUM_URBS];
    struct simcom_urb_context urb_out_ctx[NUM_URBS];
    struct simcom_urb_context urb_in_ctx[NUM_URBS];
    
    spinlock_t lock;
    struct snd_pcm_substream *playback_substream;
    struct snd_pcm_substream *capture_substream;
    
    int playback_running;
    int capture_running;
};

/* ========================================================= */
/* HARDWARE DEFINITIONS                                      */
/* ========================================================= */

/* Playback: External MPU to Module
 * Requirement: 640 bytes (320 samples) every 40ms
 * At 8kHz: 0.04s × 8000 samples/s × 2 bytes = 640 bytes
 */
static struct snd_pcm_hardware simcom_playback_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED |
            SNDRV_PCM_INFO_BLOCK_TRANSFER),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000,
    .rate_min = 8000,
    .rate_max = 8000,
    .channels_min = 1,             
    .channels_max = 1,
    .buffer_bytes_max = 1024*64,
    .period_bytes_min = 640,   /* 320 samples × 16 bits */
    .period_bytes_max = 640,
    .periods_min = 2,
    .periods_max = 4,
};

/* Capture: Module to External MPU
 * Requirement: 1600 bytes (800 samples) every 100ms
 * At 8kHz: 0.1s × 8000 samples/s × 2 bytes = 1600 bytes
 */
static struct snd_pcm_hardware simcom_capture_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED |
            SNDRV_PCM_INFO_BLOCK_TRANSFER),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000,
    .rate_min = 8000,
    .rate_max = 8000,
    .channels_min = 1,
    .channels_max = 1,
    .buffer_bytes_max = 1024*64,
    .period_bytes_min = 1600,  /* 800 samples × 16 bits */
    .period_bytes_max = 1600,
    .periods_min = 2,
    .periods_max = 4,
};

/* ========================================================= */
/* URB COMPLETION HANDLERS                                   */
/* ========================================================= */

static void simcom_playback_urb_complete(struct urb *urb)
{
    struct simcom_urb_context *ctx = urb->context;
    struct snd_pcm_substream *substream;
    struct simcom_audio *chip;
    struct snd_pcm_runtime *runtime;
    unsigned long flags;
    unsigned int period_bytes;
    unsigned int buffer_bytes;
    int ret;

    if (!urb || !ctx)
        return;

    substream = ctx->substream;
    if (!substream)
        return;

    chip = snd_pcm_substream_chip(substream);
    if (!chip)
        return;

    runtime = substream->runtime;
    if (!runtime)
        return;

    pr_debug("simcom_audio: playback URB complete, status=%d len=%u\n", urb->status, urb->actual_length);
    
    if (urb->status < 0) {
        if (urb->status != -ENOENT && urb->status != -ECONNRESET)
            pr_err("simcom_audio: playback URB error: %d\n", urb->status);
        return;
    }

    period_bytes = frames_to_bytes(runtime, runtime->period_size);
    buffer_bytes = frames_to_bytes(runtime, runtime->buffer_size);

    spin_lock_irqsave(&chip->lock, flags);

    if (!chip->playback_running) {
        spin_unlock_irqrestore(&chip->lock, flags);
        return;
    }

    /* Inform ALSA one period has been processed */
    snd_pcm_period_elapsed(substream);

    /* Prepare next period: advance offset and copy from ALSA ring to URB buf */
    ctx->offset += period_bytes;
    if (ctx->offset >= buffer_bytes)
        ctx->offset -= buffer_bytes;

    /* Copy next period from ALSA ring buffer to the coherent USB buffer */
    memcpy(ctx->buf, runtime->dma_area + ctx->offset, period_bytes);
    urb->transfer_buffer = ctx->buf;
    urb->transfer_buffer_length = period_bytes;

    ret = usb_submit_urb(urb, GFP_ATOMIC);
    if (ret < 0)
        pr_err("simcom_audio: failed to resubmit playback URB: %d\n", ret);
    else
        pr_debug("simcom_audio: playback URB resubmitted, next_off=%u len=%u\n", ctx->offset, period_bytes);

    spin_unlock_irqrestore(&chip->lock, flags);
}

static void simcom_capture_urb_complete(struct urb *urb)
{
    struct simcom_urb_context *ctx = urb->context;
    struct snd_pcm_substream *substream;
    struct simcom_audio *chip;
    struct snd_pcm_runtime *runtime;
    unsigned long flags;
    unsigned int period_bytes;
    unsigned int buffer_bytes;
    int ret;

    if (!urb || !ctx)
        return;

    substream = ctx->substream;
    if (!substream)
        return;

    chip = snd_pcm_substream_chip(substream);
    if (!chip)
        return;

    runtime = substream->runtime;
    if (!runtime)
        return;

    if (urb->status < 0) {
        if (urb->status != -ENOENT && urb->status != -ECONNRESET)
            pr_err("simcom_audio: capture URB error: %d\n", urb->status);
        return;
    }

    pr_info("simcom_audio: capture URB complete ok, len=%u\n", urb->actual_length);

    period_bytes = frames_to_bytes(runtime, runtime->period_size);
    buffer_bytes = frames_to_bytes(runtime, runtime->buffer_size);

    spin_lock_irqsave(&chip->lock, flags);

    if (!chip->capture_running) {
        spin_unlock_irqrestore(&chip->lock, flags);
        return;
    }

    /* Copy captured data from URB coherent buffer into ALSA ring buffer */
    memcpy(runtime->dma_area + ctx->offset, urb->transfer_buffer, min_t(unsigned int, urb->actual_length, period_bytes));

    /* Inform ALSA one period is available */
    snd_pcm_period_elapsed(substream);

    /* Advance to next period */
    ctx->offset += period_bytes;
    if (ctx->offset >= buffer_bytes)
        ctx->offset -= buffer_bytes;

    urb->transfer_buffer = ctx->buf;
    urb->transfer_buffer_length = period_bytes;

    ret = usb_submit_urb(urb, GFP_ATOMIC);
    if (ret < 0)
        pr_err("simcom_audio: failed to resubmit capture URB: %d\n", ret);
    else
        pr_info("simcom_audio: capture URB resubmitted, next_off=%u len=%u\n", ctx->offset, period_bytes);

    spin_unlock_irqrestore(&chip->lock, flags);
}

/* ========================================================= */
/* URB MANAGEMENT                                            */
/* ========================================================= */

static void simcom_audio_free_urbs(struct simcom_audio *chip)
{
    int i;
    
    for (i = 0; i < NUM_URBS; i++) {
        if (chip->urb_in[i]) {
            usb_kill_urb(chip->urb_in[i]);
            usb_free_urb(chip->urb_in[i]);
            chip->urb_in[i] = NULL;
        }
        if (chip->urb_out[i]) {
            usb_kill_urb(chip->urb_out[i]);
            usb_free_urb(chip->urb_out[i]);
            chip->urb_out[i] = NULL;
        }
    }
}

static int simcom_audio_create_playback_urbs(struct simcom_audio *chip)
{
    int i;
    struct snd_pcm_runtime *runtime = chip->playback_substream->runtime;
    unsigned int period_bytes = frames_to_bytes(runtime, runtime->period_size);
    unsigned int buffer_bytes = frames_to_bytes(runtime, runtime->buffer_size);
    
    for (i = 0; i < NUM_URBS; i++) {
        struct urb *urb = usb_alloc_urb(0, GFP_KERNEL);
        struct simcom_urb_context *ctx;
        unsigned int initial_offset;
        if (!urb)
            return -ENOMEM;

        ctx = &chip->urb_out_ctx[i];
        ctx->substream = chip->playback_substream;
        initial_offset = (i * period_bytes) % buffer_bytes;
        ctx->offset = initial_offset;

        /* Allocate coherent buffer for USB transfer and prefill first period */
        ctx->buf = usb_alloc_coherent(chip->udev, period_bytes, GFP_KERNEL, &ctx->dma);
        if (!ctx->buf) {
            usb_free_urb(urb);
            return -ENOMEM;
        }
        ctx->period_bytes = period_bytes;
        memcpy(ctx->buf, runtime->dma_area + initial_offset, period_bytes);

        usb_fill_bulk_urb(urb, chip->udev,
                          usb_sndbulkpipe(chip->udev, SIMCOM_PCM_EP_OUT),
                          ctx->buf,
                          period_bytes,
                          simcom_playback_urb_complete,
                          ctx);
        urb->transfer_dma = ctx->dma;
        urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

        chip->urb_out[i] = urb;
    }
    return 0;
}

static int simcom_audio_create_capture_urbs(struct simcom_audio *chip)
{
    int i;
    struct snd_pcm_runtime *runtime = chip->capture_substream->runtime;
    unsigned int period_bytes = frames_to_bytes(runtime, runtime->period_size);
    unsigned int buffer_bytes = frames_to_bytes(runtime, runtime->buffer_size);
    
    for (i = 0; i < NUM_URBS; i++) {
        struct urb *urb = usb_alloc_urb(0, GFP_KERNEL);
        struct simcom_urb_context *ctx;
        unsigned int initial_offset;
        if (!urb)
            return -ENOMEM;

        ctx = &chip->urb_in_ctx[i];
        ctx->substream = chip->capture_substream;
        initial_offset = (i * period_bytes) % buffer_bytes;
        ctx->offset = initial_offset;

        /* Allocate coherent buffer for USB transfer */
        ctx->buf = usb_alloc_coherent(chip->udev, period_bytes, GFP_KERNEL, &ctx->dma);
        if (!ctx->buf) {
            usb_free_urb(urb);
            return -ENOMEM;
        }
        ctx->period_bytes = period_bytes;

        usb_fill_bulk_urb(urb, chip->udev,
                          usb_rcvbulkpipe(chip->udev, SIMCOM_PCM_EP_IN),
                          ctx->buf,
                          period_bytes,
                          simcom_capture_urb_complete,
                          ctx);
        urb->transfer_dma = ctx->dma;
        urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

        chip->urb_in[i] = urb;
    }
    return 0;
}

/* ========================================================= */
/* PCM OPERATORS                                             */
/* ========================================================= */

static int simcom_playback_open(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;

    if (!chip) {
        pr_err("simcom_audio: chip is NULL in playback open\n");
        return -ENODEV;
    }

    runtime->hw = simcom_playback_hw;
    chip->playback_substream = substream;
    
    pr_info("simcom_audio: playback opened (8kHz, mono)\n");
    return 0;
}

static int simcom_playback_close(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    int i;

    if (!chip)
        return -ENODEV;

    // Останавливаем URBs вне spinlock
    if (chip->playback_running) {
        chip->playback_running = 0;
        for (i = 0; i < NUM_URBS; i++) {
            if (chip->urb_out[i])
                usb_kill_urb(chip->urb_out[i]);
        }
    }
    
    chip->playback_substream = NULL;
    pr_info("simcom_audio: playback closed\n");
    return 0;
}

static int simcom_playback_prepare(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    int ret;
    struct snd_pcm_runtime *runtime = substream->runtime;
    
    if (!chip) {
        pr_err("simcom_audio: chip is NULL in playback prepare\n");
        return -ENODEV;
    }

    ret = simcom_audio_create_playback_urbs(chip);
    if (ret < 0) {
        pr_err("simcom_audio: failed to create playback URBs\n");
        return ret;
    }

    pr_info("simcom_audio: playback prepare: buf_size=%zu period_size=%lu period_bytes=%zu\n",
        (size_t)frames_to_bytes(runtime, runtime->buffer_size),
        (unsigned long)runtime->period_size,
        (size_t)frames_to_bytes(runtime, runtime->period_size));
    return 0;
}

static int simcom_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    unsigned long flags;
    int i, ret = 0;

    if (!chip)
        return -ENODEV;

    spin_lock_irqsave(&chip->lock, flags);

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        chip->playback_running = 1;
        
    for (i = 0; i < NUM_URBS; i++) {
        if (chip->urb_out[i]) {
            pr_info("simcom_audio: submitting playback URB %d\n", i);
            ret = usb_submit_urb(chip->urb_out[i], GFP_ATOMIC);
            if (ret < 0) {
                pr_err("simcom_audio: failed to submit playback URB %d: %d\n", i, ret);
                break;
            } else {
                pr_info("simcom_audio: playback URB %d submitted successfully\n", i);
            }
        }
    }
    pr_info("simcom_audio: playback started\n");
        break;

    case SNDRV_PCM_TRIGGER_STOP:
        chip->playback_running = 0;
        // Только устанавливаем флаг, URBs остановим позже
        pr_info("simcom_audio: playback stopped (flag set)\n");
        break;

    default:
        ret = -EINVAL;
    }

    spin_unlock_irqrestore(&chip->lock, flags);
    return ret;
}

static int simcom_capture_open(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;

    if (!chip) {
        pr_err("simcom_audio: chip is NULL in capture open\n");
        return -ENODEV;
    }

    runtime->hw = simcom_capture_hw;
    chip->capture_substream = substream;
    
    pr_info("simcom_audio: capture opened (8kHz, mono)\n");
    return 0;
}

static int simcom_capture_close(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    int i;

    if (!chip)
        return -ENODEV;

    // Останавливаем URBs вне spinlock
    if (chip->capture_running) {
        chip->capture_running = 0;
        for (i = 0; i < NUM_URBS; i++) {
            if (chip->urb_in[i])
                usb_kill_urb(chip->urb_in[i]);
        }
    }
    
    chip->capture_substream = NULL;
    pr_info("simcom_audio: capture closed\n");
    return 0;
}

static int simcom_capture_prepare(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    int ret;
    
    if (!chip) {
        pr_err("simcom_audio: chip is NULL in capture prepare\n");
        return -ENODEV;
    }

    ret = simcom_audio_create_capture_urbs(chip);
    if (ret < 0) {
        pr_err("simcom_audio: failed to create capture URBs\n");
        return ret;
    }

    pr_info("simcom_audio: capture prepare (8kHz, mono)\n");
    return 0;
}

static int simcom_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    unsigned long flags;
    int i, ret = 0;

    if (!chip)
        return -ENODEV;

    spin_lock_irqsave(&chip->lock, flags);

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        chip->capture_running = 1;
        
        for (i = 0; i < NUM_URBS; i++) {
            if (chip->urb_in[i]) {
                ret = usb_submit_urb(chip->urb_in[i], GFP_ATOMIC);
                if (ret < 0) {
                    pr_err("simcom_audio: failed to submit capture URB: %d\n", ret);
                    break;
                }
            }
        }
        pr_info("simcom_audio: capture started\n");
        break;

    case SNDRV_PCM_TRIGGER_STOP:
        chip->capture_running = 0;
        // Только устанавливаем флаг, URBs остановим позже
        pr_info("simcom_audio: capture stopped (flag set)\n");
        break;

    default:
        ret = -EINVAL;
    }

    spin_unlock_irqrestore(&chip->lock, flags);
    return ret;
}

static int simcom_pcm_hw_params(struct snd_pcm_substream *substream,
                               struct snd_pcm_hw_params *hw_params)
{
    return snd_pcm_lib_malloc_pages(substream, 
                                   params_buffer_bytes(hw_params));
}

static int simcom_pcm_hw_free(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    int i;
    
    if (chip) {
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            for (i = 0; i < NUM_URBS; i++) {
                if (chip->urb_out[i]) {
                    usb_kill_urb(chip->urb_out[i]);
                    if (chip->urb_out_ctx[i].buf)
                        usb_free_coherent(chip->udev, chip->urb_out_ctx[i].period_bytes,
                                          chip->urb_out_ctx[i].buf, chip->urb_out_ctx[i].dma);
                    chip->urb_out_ctx[i].buf = NULL;
                    usb_free_urb(chip->urb_out[i]);
                    chip->urb_out[i] = NULL;
                }
            }
        } else {
            for (i = 0; i < NUM_URBS; i++) {
                if (chip->urb_in[i]) {
                    usb_kill_urb(chip->urb_in[i]);
                    if (chip->urb_in_ctx[i].buf)
                        usb_free_coherent(chip->udev, chip->urb_in_ctx[i].period_bytes,
                                          chip->urb_in_ctx[i].buf, chip->urb_in_ctx[i].dma);
                    chip->urb_in_ctx[i].buf = NULL;
                    usb_free_urb(chip->urb_in[i]);
                    chip->urb_in[i] = NULL;
                }
            }
        }
    }
    
    return snd_pcm_lib_free_pages(substream);
}

static snd_pcm_uframes_t simcom_pcm_pointer(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip;
    struct snd_pcm_runtime *runtime;
    struct timespec64 now;
    u64 delta_ns;
    u64 frames;
    
    chip = snd_pcm_substream_chip(substream);
    runtime = substream->runtime;
    
    if (!chip || !runtime)
        return 0;
    
    ktime_get_ts64(&now);
    
    if (runtime->status->state == SNDRV_PCM_STATE_RUNNING) {
        /* Вычисляем разницу во времени в наносекундах */
        delta_ns = (now.tv_sec - runtime->trigger_tstamp.tv_sec) * NSEC_PER_SEC +
                   (now.tv_nsec - runtime->trigger_tstamp.tv_nsec);
        
        /* Конвертируем время в фреймы */
        frames = (delta_ns * runtime->rate) / NSEC_PER_SEC;
        return frames % runtime->buffer_size;
    }
    
    return 0;
}

/* ========================================================= */
/* OPERATORS STRUCTURES                                      */
/* ========================================================= */

static struct snd_pcm_ops simcom_playback_ops = {
    .open = simcom_playback_open,
    .close = simcom_playback_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = simcom_pcm_hw_params,
    .hw_free = simcom_pcm_hw_free,
    .prepare = simcom_playback_prepare,
    .trigger = simcom_playback_trigger,
    .pointer = simcom_pcm_pointer,
};

static struct snd_pcm_ops simcom_capture_ops = {
    .open = simcom_capture_open,
    .close = simcom_capture_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = simcom_pcm_hw_params,
    .hw_free = simcom_pcm_hw_free,
    .prepare = simcom_capture_prepare,
    .trigger = simcom_capture_trigger,
    .pointer = simcom_pcm_pointer,
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
    int ifnum = intf->altsetting[0].desc.bInterfaceNumber;
    struct usb_host_interface *alt = intf->cur_altsetting;
    int chosen_alt = -1;

    ret = snd_card_new(&intf->dev, -1, "SIMCOM", THIS_MODULE,
               sizeof(*chip), &card);
    if (ret < 0)
        return ret;

    chip = card->private_data;
    chip->udev = usb_get_dev(udev);
    chip->card = card;
    spin_lock_init(&chip->lock);

    /* Ensure correct interface/altsetting with bulk endpoints is selected */
    if (alt) {
        int a;
        for (a = 0; a < intf->num_altsetting; a++) {
            struct usb_host_interface *alts = &intf->altsetting[a];
            int i;
            bool has_in = false, has_out = false;
            pr_info("simcom_audio: probing alt %d, eps %u\n", alts->desc.bAlternateSetting, alts->desc.bNumEndpoints);
            for (i = 0; i < alts->desc.bNumEndpoints; i++) {
                struct usb_endpoint_descriptor *ep = &alts->endpoint[i].desc;
                pr_info("simcom_audio: alt %d EP %d addr=0x%02x attr=0x%02x maxp=%u\n",
                    alts->desc.bAlternateSetting, i, ep->bEndpointAddress, ep->bmAttributes, le16_to_cpu(ep->wMaxPacketSize));
                if (usb_endpoint_is_bulk_in(ep) && ep->bEndpointAddress == SIMCOM_PCM_EP_IN)
                    has_in = true;
                if (usb_endpoint_is_bulk_out(ep) && ep->bEndpointAddress == SIMCOM_PCM_EP_OUT)
                    has_out = true;
            }
            if (has_in && has_out) {
                chosen_alt = alts->desc.bAlternateSetting;
                break;
            }
        }
    }
    if (chosen_alt < 0)
        chosen_alt = 0;

    ret = usb_set_interface(udev, ifnum, chosen_alt);
    if (ret < 0) {
        pr_err("simcom_audio: usb_set_interface alt=%d failed: %d\n", chosen_alt, ret);
        goto error;
    }
    pr_info("simcom_audio: using alt %d for audio endpoints\n", chosen_alt);

    strcpy(card->driver, "simcom_audio");
    strcpy(card->shortname, "SIMCom USB Audio");
    strcpy(card->longname, "SIMCom 7600 USB PCM (8kHz/16bit)");

    ret = snd_pcm_new(card, "SIMCom PCM", 0, 1, 1, &chip->pcm);
    if (ret < 0)
        goto error;

    chip->pcm->private_data = chip;
    strcpy(chip->pcm->name, "SIMCom PCM");

    snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_PLAYBACK, &simcom_playback_ops);
    snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_CAPTURE, &simcom_capture_ops);

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
    if (chip) {
        simcom_audio_free_urbs(chip);
        if (chip->card)
            snd_card_free(chip->card);
    }
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