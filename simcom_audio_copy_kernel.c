// SPDX-License-Identifier: GPL-2.0
/*
 * SIMCom SIM7600 USB Audio bridge to ALSA
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>

#define SIMCOM_VENDOR_ID  0x1E0E
#define SIMCOM_PRODUCT_ID 0x9001

#define SIMCOM_PCM_EP_OUT 0x05
#define SIMCOM_PCM_EP_IN  0x88

#define SIMCOM_PCM_RATE   8000
#define SIMCOM_PCM_CHANS  1
#define SIMCOM_PCM_BITS   16

#define NUM_URBS 4

struct simcom_audio {
    struct usb_device *udev;
    struct snd_card *card;
    struct snd_pcm *pcm;
    
    struct urb *urb_in[NUM_URBS];
    struct urb *urb_out[NUM_URBS];
    
    spinlock_t lock;
    struct snd_pcm_substream *playback_substream;
    struct snd_pcm_substream *capture_substream;
    
    // Аппаратный буфер для передачи данных через USB
    void *hw_buffer;
    size_t hw_buffer_size;
    
    unsigned int playback_hw_ptr;  // позиция в аппаратном буфере
    unsigned int appl_ptr;         // позиция приложения
    int playback_running;
    int capture_running;
};

/* ========================================================= */
/* HARDWARE DEFINITIONS                                      */
/* ========================================================= */

static struct snd_pcm_hardware simcom_playback_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED |
            SNDRV_PCM_INFO_BLOCK_TRANSFER |
            SNDRV_PCM_INFO_SYNC_START),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000,
    .rate_min = 8000,
    .rate_max = 8000,
    .channels_min = 1,             
    .channels_max = 1,
    .buffer_bytes_max = 1024*64,
    .period_bytes_min = 640,
    .period_bytes_max = 640,
    .periods_min = 2,
    .periods_max = 4,
};

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
    .period_bytes_min = 1600,
    .period_bytes_max = 1600,
    .periods_min = 2,
    .periods_max = 4,
};

/* ========================================================= */
/* URB COMPLETION HANDLERS                                   */
/* ========================================================= */

static void simcom_playback_urb_complete(struct urb *urb)
{
    struct snd_pcm_substream *substream = urb->context;
    struct simcom_audio *chip;
    struct snd_pcm_runtime *runtime;
    unsigned long flags;
    int ret;
    
    if (!urb || !substream)
        return;
        
    chip = snd_pcm_substream_chip(substream);
    if (!chip)
        return;
        
    runtime = substream->runtime;
    if (!runtime)
        return;

    spin_lock_irqsave(&chip->lock, flags);
    
    if (!chip->playback_running) {
        spin_unlock_irqrestore(&chip->lock, flags);
        return;
    }
    
    if (urb->status == 0) {
        // Успешная передача - обновляем аппаратную позицию
        chip->playback_hw_ptr += urb->transfer_buffer_length;
        if (chip->playback_hw_ptr >= chip->hw_buffer_size)
            chip->playback_hw_ptr = 0;
            
        snd_pcm_period_elapsed(substream);
        
        // Заполняем следующий период данных
        if (chip->appl_ptr != chip->playback_hw_ptr) {
            unsigned int avail, to_copy;
            unsigned int period_bytes = urb->transfer_buffer_length;
            
            // Вычисляем доступные данные
            if (chip->appl_ptr >= chip->playback_hw_ptr) {
                avail = chip->appl_ptr - chip->playback_hw_ptr;
            } else {
                avail = chip->hw_buffer_size - chip->playback_hw_ptr;
            }
            
            to_copy = period_bytes;
            if (avail < period_bytes)
                to_copy = avail;
            
            if (to_copy > 0) {
                // Копируем данные из аппаратного буфера в URB
                memcpy(urb->transfer_buffer,
                       chip->hw_buffer + chip->playback_hw_ptr,
                       to_copy);
                
                // Если данных меньше чем период, заполняем остаток тишиной
                if (to_copy < period_bytes) {
                    memset(urb->transfer_buffer + to_copy, 0, 
                           period_bytes - to_copy);
                }
            } else {
                // Нет данных - заполняем тишиной
                memset(urb->transfer_buffer, 0, period_bytes);
            }
            
            // Переотправляем URB
            ret = usb_submit_urb(urb, GFP_ATOMIC);
            if (ret < 0) {
                pr_err("simcom_audio: failed to resubmit playback URB: %d\n", ret);
            }
        }
    } else if (urb->status != -ENOENT && urb->status != -ECONNRESET) {
        pr_err("simcom_audio: playback URB error: %d\n", urb->status);
    }
    
    spin_unlock_irqrestore(&chip->lock, flags);
}

static void simcom_capture_urb_complete(struct urb *urb)
{
    struct snd_pcm_substream *substream = urb->context;
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    
    if (!chip || !chip->capture_running)
        return;
        
    if (urb->status == 0 && urb->actual_length > 0) {
        snd_pcm_period_elapsed(substream);
        
        if (chip->capture_running) {
            if (usb_submit_urb(urb, GFP_ATOMIC) < 0)
                pr_err("simcom_audio: failed to resubmit capture URB\n");
        }
    } else if (urb->status != -ENOENT) {
        pr_err("simcom_audio: capture URB error: %d\n", urb->status);
    }
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
    
    for (i = 0; i < NUM_URBS; i++) {
        struct urb *urb;
        void *buffer;
        
        urb = usb_alloc_urb(0, GFP_KERNEL);
        if (!urb)
            return -ENOMEM;
            
        buffer = kmalloc(period_bytes, GFP_KERNEL);
        if (!buffer) {
            usb_free_urb(urb);
            return -ENOMEM;
        }
        
        // Инициализируем буфер тишиной
        memset(buffer, 0, period_bytes);
        
        usb_fill_bulk_urb(urb, 
                          chip->udev,
                          usb_sndbulkpipe(chip->udev, SIMCOM_PCM_EP_OUT),
                          buffer,
                          period_bytes,
                          simcom_playback_urb_complete,
                          chip->playback_substream);
        
        chip->urb_out[i] = urb;
    }
    return 0;
}

static int simcom_audio_create_capture_urbs(struct simcom_audio *chip)
{
    int i;
    struct snd_pcm_runtime *runtime = chip->capture_substream->runtime;
    unsigned int xfer = frames_to_bytes(runtime, runtime->period_size);
    
    for (i = 0; i < NUM_URBS; i++) {
        chip->urb_in[i] = usb_alloc_urb(0, GFP_KERNEL);
        if (!chip->urb_in[i])
            return -ENOMEM;
            
        usb_fill_bulk_urb(chip->urb_in[i], chip->udev,
                         usb_rcvbulkpipe(chip->udev, SIMCOM_PCM_EP_IN),
                         runtime->dma_area,
                         xfer,
                         simcom_capture_urb_complete,
                         chip->capture_substream);
    }
    return 0;
}

/* ========================================================= */
/* PCM COPY CALLBACKS - для External Hardware Buffers        */
/* ========================================================= */

static int simcom_playback_copy_user(struct snd_pcm_substream *substream,
                                    int channel, unsigned long pos,
                                    void __user *src, unsigned long count)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    unsigned long flags;
    
    if (!chip || !chip->hw_buffer)
        return -ENODEV;
    
    // Копируем данные из пользовательского пространства в аппаратный буфер
    if (copy_from_user(chip->hw_buffer + pos, src, count))
        return -EFAULT;
    
    spin_lock_irqsave(&chip->lock, flags);
    // Обновляем позицию приложения
    chip->appl_ptr = pos + count;
    if (chip->appl_ptr >= chip->hw_buffer_size)
        chip->appl_ptr -= chip->hw_buffer_size;
    spin_unlock_irqrestore(&chip->lock, flags);
        
    return 0;
}

static int simcom_playback_copy_kernel(struct snd_pcm_substream *substream,
                                      int channel, unsigned long pos,
                                      void *src, unsigned long count)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    unsigned long flags;
    
    if (!chip || !chip->hw_buffer)
        return -ENODEV;
    
    // Копируем данные из ядерного пространства в аппаратный буфер
    memcpy(chip->hw_buffer + pos, src, count);
    
    spin_lock_irqsave(&chip->lock, flags);
    // Обновляем позицию приложения
    chip->appl_ptr = pos + count;
    if (chip->appl_ptr >= chip->hw_buffer_size)
        chip->appl_ptr -= chip->hw_buffer_size;
    spin_unlock_irqrestore(&chip->lock, flags);
    
    return 0;
}

static int simcom_playback_fill_silence(struct snd_pcm_substream *substream,
                                       int channel, unsigned long pos,
                                       unsigned long count)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    unsigned long flags;
    
    if (!chip || !chip->hw_buffer)
        return -ENODEV;
    
    // Заполняем аппаратный буфер тишиной
    memset(chip->hw_buffer + pos, 0, count);
    
    spin_lock_irqsave(&chip->lock, flags);
    // Обновляем позицию приложения
    chip->appl_ptr = pos + count;
    if (chip->appl_ptr >= chip->hw_buffer_size)
        chip->appl_ptr -= chip->hw_buffer_size;
    spin_unlock_irqrestore(&chip->lock, flags);
    
    return 0;
}

static int simcom_capture_copy_user(struct snd_pcm_substream *substream,
                                   int channel, unsigned long pos,
                                   void __user *dst, unsigned long count)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;
    
    if (!chip || !runtime)
        return -ENODEV;
    
    // Для capture используем стандартный DMA буфер
    if (copy_to_user(dst, runtime->dma_area + pos, count))
        return -EFAULT;
        
    return 0;
}

static int simcom_capture_copy_kernel(struct snd_pcm_substream *substream,
                                     int channel, unsigned long pos,
                                     void *dst, unsigned long count)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;
    
    if (!chip || !runtime)
        return -ENODEV;
    
    // Для capture используем стандартный DMA буфер
    memcpy(dst, runtime->dma_area + pos, count);
    
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
    chip->playback_hw_ptr = 0;
    chip->appl_ptr = 0;
    
    return 0;
}

static int simcom_playback_close(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    int i;

    if (!chip)
        return -ENODEV;

    if (chip->playback_running) {
        chip->playback_running = 0;
        for (i = 0; i < NUM_URBS; i++) {
            if (chip->urb_out[i])
                usb_kill_urb(chip->urb_out[i]);
        }
    }
    
    chip->playback_substream = NULL;
    return 0;
}

static int simcom_playback_prepare(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    int ret;
    
    if (!chip) {
        pr_err("simcom_audio: chip is NULL in playback prepare\n");
        return -ENODEV;
    }

    // Сбрасываем позиции
    chip->playback_hw_ptr = 0;
    chip->appl_ptr = 0;

    ret = simcom_audio_create_playback_urbs(chip);
    if (ret < 0) {
        pr_err("simcom_audio: failed to create playback URBs\n");
        return ret;
    }

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
                // Заполняем URB начальными данными
                struct snd_pcm_runtime *runtime = substream->runtime;
                unsigned int period_bytes = frames_to_bytes(runtime, runtime->period_size);
                unsigned int to_copy;
                unsigned int avail;
                
                avail = chip->hw_buffer_size - chip->playback_hw_ptr;
                to_copy = period_bytes;
                if (avail < period_bytes)
                    to_copy = avail;
                
                if (to_copy > 0) {
                    memcpy(chip->urb_out[i]->transfer_buffer,
                           chip->hw_buffer + chip->playback_hw_ptr,
                           to_copy);
                    
                    if (to_copy < period_bytes) {
                        memset(chip->urb_out[i]->transfer_buffer + to_copy, 0,
                               period_bytes - to_copy);
                    }
                } else {
                    memset(chip->urb_out[i]->transfer_buffer, 0, period_bytes);
                }
                
                ret = usb_submit_urb(chip->urb_out[i], GFP_ATOMIC);
                if (ret < 0) {
                    pr_err("simcom_audio: failed to submit playback URB %d: %d\n", i, ret);
                    chip->playback_running = 0;
                    break;
                }
                
                // Обновляем позицию для следующего URB
                chip->playback_hw_ptr += period_bytes;
                if (chip->playback_hw_ptr >= chip->hw_buffer_size)
                    chip->playback_hw_ptr = 0;
            }
        }
        break;

    case SNDRV_PCM_TRIGGER_STOP:
        chip->playback_running = 0;
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
    
    return 0;
}

static int simcom_capture_close(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    int i;

    if (!chip)
        return -ENODEV;

    if (chip->capture_running) {
        chip->capture_running = 0;
        for (i = 0; i < NUM_URBS; i++) {
            if (chip->urb_in[i])
                usb_kill_urb(chip->urb_in[i]);
        }
    }
    
    chip->capture_substream = NULL;
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
        break;

    case SNDRV_PCM_TRIGGER_STOP:
        chip->capture_running = 0;
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
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    size_t buffer_size = params_buffer_bytes(hw_params);
    int ret;
    
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        // Для playback используем аппаратный буфер
        if (!chip->hw_buffer || chip->hw_buffer_size < buffer_size) {
            // Освобождаем старый буфер если нужен больший
            if (chip->hw_buffer) {
                kfree(chip->hw_buffer);
                chip->hw_buffer = NULL;
            }
            
            // Выделяем новый аппаратный буфер
            chip->hw_buffer = kzalloc(buffer_size, GFP_KERNEL);
            if (!chip->hw_buffer)
                return -ENOMEM;
                
            chip->hw_buffer_size = buffer_size;
        }
        
        // Для External Hardware Buffers не используем стандартный DMA буфер
        return 0;
    } else {
        // Для capture используем стандартный DMA буфер
        ret = snd_pcm_lib_malloc_pages(substream, buffer_size);
        if (ret < 0)
            return ret;
    }
    
    return 0;
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
                    if (chip->urb_out[i]->transfer_buffer)
                        kfree(chip->urb_out[i]->transfer_buffer);
                    usb_free_urb(chip->urb_out[i]);
                    chip->urb_out[i] = NULL;
                }
            }
        } else {
            for (i = 0; i < NUM_URBS; i++) {
                if (chip->urb_in[i]) {
                    usb_kill_urb(chip->urb_in[i]);
                    usb_free_urb(chip->urb_in[i]);
                    chip->urb_in[i] = NULL;
                }
            }
            
            // Освобождаем стандартный DMA буфер для capture
            return snd_pcm_lib_free_pages(substream);
        }
    }
    
    return 0;
}

static snd_pcm_uframes_t simcom_pcm_pointer(struct snd_pcm_substream *substream)
{
    struct simcom_audio *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;
    unsigned long flags;
    snd_pcm_uframes_t pos;
    
    if (!chip || !runtime)
        return 0;
    
    spin_lock_irqsave(&chip->lock, flags);
    
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        // Для playback возвращаем аппаратную позицию
        pos = bytes_to_frames(runtime, chip->playback_hw_ptr);
    } else {
        // Для capture используем стандартный механизм
        pos = bytes_to_frames(runtime, runtime->control->appl_ptr);
    }
    
    spin_unlock_irqrestore(&chip->lock, flags);
    return pos;
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
    .copy_user = simcom_playback_copy_user,
    .copy_kernel = simcom_playback_copy_kernel,
    .fill_silence = simcom_playback_fill_silence,
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
    .copy_user = simcom_capture_copy_user,
    .copy_kernel = simcom_capture_copy_kernel,
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

    chip->pcm->private_data = chip;
    strcpy(chip->pcm->name, "SIMCom PCM");

    snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_PLAYBACK, &simcom_playback_ops);
    snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_CAPTURE, &simcom_capture_ops);

    // Для playback не используем preallocate, так как используем аппаратный буфер
    // Для capture используем стандартный preallocate
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
    if (chip->hw_buffer) {
        kfree(chip->hw_buffer);
        chip->hw_buffer = NULL;
    }
    snd_card_free(card);
    return ret;
}

static void simcom_audio_disconnect(struct usb_interface *intf)
{
    struct simcom_audio *chip = usb_get_intfdata(intf);
    if (chip) {
        simcom_audio_free_urbs(chip);
        if (chip->hw_buffer) {
            kfree(chip->hw_buffer);
            chip->hw_buffer = NULL;
        }
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