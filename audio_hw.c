/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file    audio_hw.c
 * @brief
 *                 ALSA Audio Git Log
 * - V0.1.0:add alsa audio hal,just support 312x now.
 * - V0.2.0:remove unused variable.
 * - V0.3.0:turn off device when do_standby.
 * - V0.4.0:turn off device before open pcm.
 * - V0.4.1:Need to re-open the control to fix no sound when suspend.
 * - V0.5.0:Merge the mixer operation from legacy_alsa.
 * - V0.6.0:Merge speex denoise from legacy_alsa.
 * - V0.7.0:add copyright.
 * - V0.7.1:add support for box audio
 * - V0.7.2:add support for dircet output
 * - V0.8.0:update the direct output for box, add the DVI mode
 * - V1.0.0:stable version
 *
 * @author  RkAudio
 * @version 1.0.5
 * @date    2015-08-24
 */

//#define LOG_NDEBUG 0
#define LOG_TAG "AudioHardwareTiny"

#include "alsa_audio.h"
#include "audio_hw.h"
#include <system/audio.h>
#include "codec_config/config.h"
#include "audio_bitstream.h"
#include "audio_setting.h"
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdio.h>
#include <poll.h>
#include <termios.h>
#include <stdbool.h>
#define SNDRV_CARDS 8
#define SNDRV_DEVICES 8
#define SIMCOM_TX_FALLBACK_PROP   "persist.vendor.simcom.force_fallback_tx"

static inline bool simcom_tx_fallback_enabled(void)
{
    return property_get_bool(SIMCOM_TX_FALLBACK_PROP, true);
}

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define SND_CARDS_NODE          "/proc/asound/cards"
#define SIMCOM_CARD_ID_STRING   "SIMCOM"
#define SIMCOM_PCM_CARD_FALLBACK 0
#define SIMCOM_PCM_DEVICE       0
#define SIMCOM_AT_DEVICE_PATH    "/dev/ttyUSB3"
#define SIMCOM_PCM_READY_PROP    "vendor.simcom.pcm_ready"
#define SIMCOM_CPCM_DELAY_PROP   "persist.vendor.simcom.cpcmreg_delay_ms"
#define SIMCOM_CPCM_TIMEOUT_PROP "persist.vendor.simcom.cpcmreg_timeout_ms"
#define SIMCOM_PCM_WAIT_PROP     "persist.vendor.simcom.cpcmreg_wait_ms"
#define SIMCOM_CPCM_DEFAULT_DELAY_MS    3000
#define SIMCOM_CPCM_DEFAULT_TIMEOUT_MS  2000
#define SIMCOM_PCM_WAIT_DEFAULT_MS      6000
#define SIMCOM_POLL_STEP_MS      100
#define SIMCOM_CPCM_WRITE_RETRIES       3
#define SIMCOM_CPCM_WRITE_RETRY_DELAY_US 200000
#define SIMCOM_DUMP_RX_PROP      "persist.vendor.simcom.dump_rx"
#define SIMCOM_DUMP_TX_PROP      "persist.vendor.simcom.dump_tx"
#define SIMCOM_RX_DUMP_PATH      "/data/misc/audioserver/simcom_rx_dump.pcm"
#define SIMCOM_TX_DUMP_PATH      "/data/misc/audioserver/simcom_tx_dump.pcm"
#define OUT_SIMCOM_PCM(out)  ((out)->dev->simcom_tx_pcm)
#define IN_SIMCOM_PCM(in)    ((in)->dev->simcom_rx_pcm)
#define SIMCOM_TX_TARGET_RATE     8000

static int simcom_ensure_tx_resampler_buffer(struct stream_out *out, size_t frames, size_t channels);
static int simcom_ensure_tx_mono_buffer(struct stream_out *out, size_t frames);
static void simcom_release_tx_resampler(struct stream_out *out);
static int simcom_prepare_tx_resampler(struct stream_out *out, uint32_t in_rate, size_t channels);

static bool simcom_force_patch_enabled(void)
{
    return property_get_bool("persist.vendor.audio.simcom.force_patch", false);
}

struct simcom_cpcm_task {
    struct audio_device *adev;
    bool enable;
};

static void simcom_update_pcm_ready(struct audio_device *adev, bool ready)
{
    if (!adev) {
        return;
    }
    pthread_mutex_lock(&adev->simcom_modem_lock);
    adev->simcom_pcm_ready = ready;
    pthread_mutex_unlock(&adev->simcom_modem_lock);
    property_set(SIMCOM_PCM_READY_PROP, ready ? "1" : "0");
    ALOGI("SIMCOM: PCM ready flag -> %d", ready);
}

static int simcom_open_at_port(void)
{
    int fd = open(SIMCOM_AT_DEVICE_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        ALOGE("SIMCOM: failed to open %s (%s)", SIMCOM_AT_DEVICE_PATH, strerror(errno));
        return -errno;
    }

    struct termios cfg;
    if (tcgetattr(fd, &cfg) != 0) {
        ALOGE("SIMCOM: tcgetattr failed (%s)", strerror(errno));
        close(fd);
        return -errno;
    }

    cfmakeraw(&cfg);
    cfsetispeed(&cfg, B115200);
    cfsetospeed(&cfg, B115200);
    cfg.c_cflag |= (CLOCAL | CREAD);

    if (tcsetattr(fd, TCSANOW, &cfg) != 0) {
        ALOGE("SIMCOM: tcsetattr failed (%s)", strerror(errno));
        close(fd);
        return -errno;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static int simcom_send_cpcmreg_command(bool enable, int timeout_ms)
{
    int fd = simcom_open_at_port();
    if (fd < 0) {
        return fd;
    }

    const char *cmd = enable ? "AT+CPCMREG=1\r" : "AT+CPCMREG=0\r";
    ssize_t written = -1;
    int retries = SIMCOM_CPCM_WRITE_RETRIES;
    while (retries-- >= 0) {
        written = write(fd, cmd, strlen(cmd));
        if (written >= 0) {
            break;
        }
        if ((errno == EAGAIN || errno == EWOULDBLOCK) && retries >= 0) {
            ALOGW("SIMCOM: write '%s' got EAGAIN, retrying (%d)", cmd, retries);
            usleep(SIMCOM_CPCM_WRITE_RETRY_DELAY_US);
            continue;
        }
        ALOGE("SIMCOM: write '%s' failed (%s)", cmd, strerror(errno));
        close(fd);
        return -errno;
    }
    if (written < 0) {
        ALOGE("SIMCOM: write '%s' exhausted retries", cmd);
        close(fd);
        return -errno;
    }

    char response[128] = {0};
    size_t resp_len = 0;
    int remaining = timeout_ms > 0 ? timeout_ms : SIMCOM_CPCM_DEFAULT_TIMEOUT_MS;

    while (remaining > 0 && resp_len < sizeof(response) - 1) {
        struct pollfd pfd = {
            .fd = fd,
            .events = POLLIN,
        };
        int polled = poll(&pfd, 1, remaining < SIMCOM_POLL_STEP_MS ? remaining : SIMCOM_POLL_STEP_MS);
        if (polled < 0) {
            if (errno == EINTR) {
                continue;
            }
            ALOGE("SIMCOM: poll failed (%s)", strerror(errno));
            break;
        } else if (polled == 0) {
            remaining -= SIMCOM_POLL_STEP_MS;
            continue;
        }

        if (pfd.revents & POLLIN) {
            ssize_t r = read(fd, response + resp_len, sizeof(response) - 1 - resp_len);
            if (r > 0) {
                resp_len += r;
                response[resp_len] = '\0';
                if (strstr(response, "OK")) {
                    close(fd);
                    ALOGI("SIMCOM: '%s' acknowledged by modem", cmd);
                    return 0;
                }
                if (strstr(response, "ERROR")) {
                    ALOGE("SIMCOM: '%s' rejected: %s", cmd, response);
                    close(fd);
                    return -EIO;
                }
            }
        }
    }

    ALOGE("SIMCOM: timeout waiting for response to '%s' (resp='%s')", cmd, response);
    close(fd);
    return -ETIMEDOUT;
}

static void *simcom_cpcm_worker(void *opaque)
{
    struct simcom_cpcm_task *task = (struct simcom_cpcm_task *)opaque;
    if (!task || !task->adev) {
        ALOGE("SIMCOM: cpcm_worker: invalid task");
        free(task);
        return NULL;
    }

    struct audio_device *adev = task->adev;
    const bool enable = task->enable;
    free(task);

    ALOGI("SIMCOM: cpcm_worker started: enable=%d", enable ? 1 : 0);

    if (enable) {
        int delay_ms = property_get_int32(SIMCOM_CPCM_DELAY_PROP, SIMCOM_CPCM_DEFAULT_DELAY_MS);
        if (delay_ms > 0) {
            ALOGI("SIMCOM: delaying AT+CPCMREG=1 by %d ms", delay_ms);
            usleep(delay_ms * 1000);
        }
    }

    int timeout_ms = property_get_int32(SIMCOM_CPCM_TIMEOUT_PROP, SIMCOM_CPCM_DEFAULT_TIMEOUT_MS);
    ALOGI("SIMCOM: cpcm_worker: sending AT+CPCMREG=%d (timeout=%d ms)", enable ? 1 : 0, timeout_ms);
    int ret = simcom_send_cpcmreg_command(enable, timeout_ms);
    if (ret != 0) {
        ALOGE("SIMCOM: AT+CPCMREG=%d failed ret=%d", enable ? 1 : 0, ret);
    } else {
        ALOGI("SIMCOM: AT+CPCMREG=%d succeeded", enable ? 1 : 0);
    }

    simcom_update_pcm_ready(adev, enable && (ret == 0));
    ALOGI("SIMCOM: cpcm_worker finished: pcm_ready=%d", (enable && (ret == 0)) ? 1 : 0);
    return NULL;
}

static void simcom_schedule_cpcm_command(struct audio_device *adev, bool enable)
{
    if (!adev) {
        ALOGW("SIMCOM: simcom_schedule_cpcm_command called with NULL adev");
        return;
    }

    ALOGI("SIMCOM: simcom_schedule_cpcm_command: enable=%d (current voice_call_active=%d)", 
          enable ? 1 : 0, adev->voice_call_active ? 1 : 0);

    if (enable) {
        simcom_update_pcm_ready(adev, false);
    }

    struct simcom_cpcm_task *task = (struct simcom_cpcm_task *)calloc(1, sizeof(*task));
    if (!task) {
        ALOGE("SIMCOM: failed to allocate cpcm task");
        return;
    }
    task->adev = adev;
    task->enable = enable;

    pthread_t thread_id;
    int ret = pthread_create(&thread_id, NULL, simcom_cpcm_worker, task);
    if (ret != 0) {
        ALOGE("SIMCOM: pthread_create failed (%s)", strerror(ret));
        free(task);
        return;
    }
    pthread_detach(thread_id);
    ALOGI("SIMCOM: simcom_schedule_cpcm_command: worker thread started for AT+CPCMREG=%d", enable ? 1 : 0);
}

static bool simcom_is_pcm_ready(struct audio_device *adev)
{
    if (!adev) {
        return false;
    }
    pthread_mutex_lock(&adev->simcom_modem_lock);
    bool ready = adev->simcom_pcm_ready;
    pthread_mutex_unlock(&adev->simcom_modem_lock);
    return ready;
}

static bool simcom_wait_for_pcm_ready(struct audio_device *adev)
{
    int wait_ms = property_get_int32(SIMCOM_PCM_WAIT_PROP, SIMCOM_PCM_WAIT_DEFAULT_MS);
    if (wait_ms <= 0) {
        return simcom_is_pcm_ready(adev);
    }

    int waited = 0;
    while (waited < wait_ms) {
        if (simcom_is_pcm_ready(adev)) {
            return true;
        }
        usleep(SIMCOM_POLL_STEP_MS * 1000);
        waited += SIMCOM_POLL_STEP_MS;
    }
    return simcom_is_pcm_ready(adev);
}

static bool simcom_voice_mode_active(struct audio_device *adev);

static uint32_t simcom_select_voice_playback_route(audio_devices_t devices)
{
    if (devices & AUDIO_DEVICE_OUT_ALL_SCO) {
        return BLUETOOTH_INCALL_ROUTE;
    }
    if ((devices & AUDIO_DEVICE_OUT_SPEAKER) &&
        (devices & (AUDIO_DEVICE_OUT_WIRED_HEADPHONE | AUDIO_DEVICE_OUT_WIRED_HEADSET))) {
        return SPEAKER_HEADPHONE_INCALL_ROUTE;
    }
    if (devices & AUDIO_DEVICE_OUT_WIRED_HEADSET) {
        return HEADSET_INCALL_ROUTE;
    }
    if (devices & AUDIO_DEVICE_OUT_WIRED_HEADPHONE) {
        return HEADPHONE_INCALL_ROUTE;
    }
    if (devices & AUDIO_DEVICE_OUT_EARPIECE) {
        return EARPIECE_INCALL_ROUTE;
    }
    if (devices & AUDIO_DEVICE_OUT_SPEAKER) {
        return SPEAKER_INCALL_ROUTE;
    }
    return SPEAKER_INCALL_ROUTE;
}

static uint32_t simcom_select_voice_capture_route(audio_devices_t devices)
{
    if (devices & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET) {
        return BLUETOOTH_SOC_MIC_CAPTURE_ROUTE;
    }
    if (devices & AUDIO_DEVICE_IN_WIRED_HEADSET) {
        return HANDS_FREE_MIC_CAPTURE_ROUTE;
    }
    if (devices & AUDIO_DEVICE_IN_BACK_MIC) {
        return HANDS_FREE_MIC_CAPTURE_ROUTE;
    }
    return MAIN_MIC_CAPTURE_ROUTE;
}

static void simcom_apply_voice_routes_l(struct audio_device *adev)
{
    if (!simcom_voice_mode_active(adev)) {
        return;
    }

    audio_devices_t out_dev = adev->out_device;
    if (out_dev == AUDIO_DEVICE_NONE) {
        out_dev = AUDIO_DEVICE_OUT_SPEAKER;
    }

    uint32_t route = simcom_select_voice_playback_route(out_dev);
    ALOGI("SIMCOM: simcom_apply_voice_routes_l: out_dev=0x%x route=%u", out_dev, route);
    if (route == 0 ||
        (adev->simcom_voice_playback_route == route &&
         adev->simcom_forced_out_device == out_dev)) {
        ALOGI("SIMCOM: simcom_apply_voice_routes_l: skipping (route==0 or already applied)");
        return;
    }

    int card = adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card;
    if (card < 0) {
        ALOGW("SIMCOM: cannot apply voice route %u, invalid speaker card %d", route, card);
        return;
    }

    ALOGI("SIMCOM: simcom_apply_voice_routes_l: calling route_pcm_card_open(card=%d, route=%u)", card, route);
    route_pcm_card_open(card, route);
    adev->simcom_voice_playback_route = route;
    adev->simcom_forced_out_device = out_dev;
    ALOGI("SIMCOM: applied voice playback route %u for devices=0x%x (card=%d)",
          route, out_dev, card);

    audio_devices_t in_dev = adev->in_device;
    if (in_dev == AUDIO_DEVICE_NONE) {
        in_dev = AUDIO_DEVICE_IN_BUILTIN_MIC;
    }
    uint32_t capture_route = simcom_select_voice_capture_route(in_dev);
    if (capture_route != 0 &&
        (capture_route != adev->simcom_voice_capture_route ||
         adev->simcom_forced_in_device != in_dev)) {
        int in_card = adev->dev_in[SND_IN_SOUND_CARD_MIC].card;
        if (in_card < 0) {
            in_card = adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card;
        }
        if (in_card >= 0) {
            route_pcm_card_open(in_card, capture_route);
            adev->simcom_voice_capture_route = capture_route;
            adev->simcom_forced_in_device = in_dev;
            ALOGI("SIMCOM: applied voice capture route %u for devices=0x%x (card=%d)",
                  capture_route, in_dev, in_card);
        } else {
            ALOGW("SIMCOM: cannot apply voice capture route %u, invalid mic card %d",
                  capture_route, in_card);
        }
    }
}

static void simcom_reset_voice_routes_l(struct audio_device *adev)
{
    ALOGI("SIMCOM: simcom_reset_voice_routes_l called (playback_route=%d capture_route=%d)", 
          adev->simcom_voice_playback_route, adev->simcom_voice_capture_route);
    if (adev->simcom_voice_playback_route != 0) {
        int card = adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card;
        if (card >= 0) {
            // SIMCOM: Apply SPEAKER_NORMAL_ROUTE to reset mixer
            route_pcm_card_open(card, SPEAKER_NORMAL_ROUTE);
            ALOGI("SIMCOM: cleared voice playback route, applied SPEAKER_NORMAL_ROUTE (card=%d)", card);
        }
        adev->simcom_voice_playback_route = 0;
        adev->simcom_forced_out_device = AUDIO_DEVICE_NONE;
    }
    if (adev->simcom_voice_capture_route != 0) {
        int in_card = adev->dev_in[SND_IN_SOUND_CARD_MIC].card;
        if (in_card < 0) {
            in_card = adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card;
        }
        if (in_card >= 0) {
            route_pcm_card_open(in_card, CAPTURE_OFF_ROUTE);
            ALOGI("SIMCOM: cleared voice capture route (card=%d)", in_card);
        }
        adev->simcom_voice_capture_route = 0;
        adev->simcom_forced_in_device = AUDIO_DEVICE_NONE;
    }
}

static void simcom_rx_bus_init(struct simcom_rx_bus *bus)
{
    pthread_mutex_init(&bus->lock, NULL);
    pthread_cond_init(&bus->cond, NULL);
    bus->frame_ready = false;
    bus->producer_active = false;
    bus->frame_bytes = 0;
    bus->frame_gen = 0;
    bus->pending_consumers = 0;
}

static void simcom_rx_bus_reset(struct audio_device *adev)
{
    struct simcom_rx_bus *bus = &adev->simcom_rx_bus;
    pthread_mutex_lock(&bus->lock);
    bus->frame_ready = false;
    bus->producer_active = false;
    bus->frame_bytes = 0;
    bus->frame_gen = 0;
    bus->pending_consumers = 0;
    // SIMCOM: Clear buffer data to prevent stale data from previous call
    memset(bus->frame_buf, 0, sizeof(bus->frame_buf));
    pthread_cond_broadcast(&bus->cond);
    pthread_mutex_unlock(&bus->lock);
    ALOGI("SIMCOM: RX bus reset - buffer cleared");
}

static void simcom_rx_bus_drop_consumer(struct audio_device *adev)
{
    struct simcom_rx_bus *bus = &adev->simcom_rx_bus;
    pthread_mutex_lock(&bus->lock);
    if (bus->frame_ready && bus->pending_consumers > 0) {
        if (--bus->pending_consumers == 0) {
            bus->frame_ready = false;
            pthread_cond_broadcast(&bus->cond);
        }
    }
    pthread_mutex_unlock(&bus->lock);
}

static int simcom_rx_bus_acquire(struct stream_in *in, size_t bytes)
{
    struct audio_device *adev = in->dev;
    struct simcom_rx_bus *bus = &adev->simcom_rx_bus;

    if (bytes > SIMCOM_RX_BUS_MAX_BYTES) {
        ALOGE("SIMCOM: requested RX chunk %zu exceeds bus buffer %zu",
              bytes, (size_t)SIMCOM_RX_BUS_MAX_BYTES);
        return -EINVAL;
    }

    if (adev->simcom_rx_users <= 1) {
        return pcm_read(in->pcm, in->buffer, bytes);
    }

    pthread_mutex_lock(&bus->lock);
    while (true) {
        if (bus->frame_ready && bus->frame_bytes == bytes &&
            in->simcom_rx_last_gen != bus->frame_gen) {
            memcpy(in->buffer, bus->frame_buf, bytes);
            in->simcom_rx_last_gen = bus->frame_gen;
            if (bus->pending_consumers > 0 && --bus->pending_consumers == 0) {
                bus->frame_ready = false;
                pthread_cond_broadcast(&bus->cond);
            }
            pthread_mutex_unlock(&bus->lock);
            return 0;
        }

        if (!bus->frame_ready || bus->frame_bytes != bytes || bus->pending_consumers == 0) {
            if (bus->producer_active) {
                pthread_cond_wait(&bus->cond, &bus->lock);
                continue;
            }

            bus->producer_active = true;
            pthread_mutex_unlock(&bus->lock);
            int status = pcm_read(in->pcm, in->buffer, bytes);
            pthread_mutex_lock(&bus->lock);
            bus->producer_active = false;

            if (status != 0) {
                bus->frame_ready = false;
                pthread_cond_broadcast(&bus->cond);
                pthread_mutex_unlock(&bus->lock);
                return status;
            }

            memcpy(bus->frame_buf, in->buffer, bytes);
            bus->frame_bytes = bytes;
            bus->frame_gen++;
            bus->frame_ready = true;
            bus->pending_consumers = adev->simcom_rx_users;
            if (bus->pending_consumers == 0) {
                bus->pending_consumers = 1;
            }
            pthread_cond_broadcast(&bus->cond);

            if (bus->pending_consumers > 0 && --bus->pending_consumers == 0) {
                bus->frame_ready = false;
                pthread_cond_broadcast(&bus->cond);
            }
            in->simcom_rx_last_gen = bus->frame_gen;
            pthread_mutex_unlock(&bus->lock);
            return 0;
        }

        pthread_cond_wait(&bus->cond, &bus->lock);
    }
}

static bool simcom_detect_card(struct audio_device *adev);
static int simcom_wait_for_pcm_node(struct audio_device *adev, bool is_rx, bool wait_for_device);

static inline int simcom_pcm_card_index(const struct audio_device *adev)
{
    if (adev && adev->simcom_pcm_card >= 0) {
        return adev->simcom_pcm_card;
    }
    return SIMCOM_PCM_CARD_FALLBACK;
}

static inline int simcom_pcm_device_index(const struct audio_device *adev)
{
    if (adev) {
        return adev->simcom_pcm_device;
    }
    return SIMCOM_PCM_DEVICE;
}

static struct pcm_config simcom_pcm_config_tx = {
    .channels = 1,
    .rate = 8000,
    .period_size = 320,   /* 640 bytes @ S16 mono */
    .period_count = 8,
    .format = PCM_FORMAT_S16_LE,
    .start_threshold = 640,
    .stop_threshold = 1280,
    .silence_threshold = 0,
    .avail_min = 320,
};

static struct pcm_config simcom_pcm_config_rx = {
    .channels = 1,
    .rate = 8000,
    .period_size = 800,   /* 1600 bytes @ S16 mono */
    .period_count = 8,
    .format = PCM_FORMAT_S16_LE,
    .start_threshold = 1600,
    .stop_threshold = 3200,
    .silence_threshold = 0,
    .avail_min = 800,
};

static int simcom_acquire_tx_pcm(struct audio_device *adev, bool wait_for_device)
{
    if (!adev) {
        return -EINVAL;
    }

    if (!simcom_wait_for_pcm_ready(adev)) {
        ALOGW("SIMCOM: telephony TX requested before modem PCM ready");
        return -EAGAIN;
    }

    if (adev->simcom_tx_pcm) {
        adev->simcom_tx_users++;
        ALOGD("SIMCOM: telephony TX PCM reuse, users=%d", adev->simcom_tx_users);
        return 0;
    }

    if (!simcom_detect_card(adev)) {
        ALOGE("SIMCOM: telephony TX requested but SIMCOM device not detected");
        return -ENODEV;
    }

    int ret = simcom_wait_for_pcm_node(adev, false, wait_for_device);
    if (ret != 0) {
        return (ret == -ETIMEDOUT || ret == -ENOENT) ? -EAGAIN : ret;
    }

    const int card = simcom_pcm_card_index(adev);
    const int device = simcom_pcm_device_index(adev);
    const int attempts = wait_for_device ? 5 : 1;
    int last_err = -EIO;

    for (int attempt = 1; attempt <= attempts; ++attempt) {
        ALOGD("SIMCOM: opening shared telephony TX PCM (card %d device %d), attempt %d/%d",
              card, device, attempt, attempts);
        struct pcm *pcm_handle = pcm_open(card, device,
                                          PCM_OUT | PCM_MONOTONIC,
                                          &simcom_pcm_config_tx);
        if (pcm_handle && pcm_is_ready(pcm_handle)) {
            adev->simcom_tx_pcm = pcm_handle;
            adev->simcom_tx_users = 1;
            // SIMCOM: Prepare PCM to clear ALSA buffers and reset state
            pcm_prepare(pcm_handle);
            ALOGI("SIMCOM: telephony TX PCM ready on card %d device %d (prepared and cleared)", card, device);
            return 0;
        }

        int err = errno ? -errno : -ENODEV;
        last_err = err;
        if (pcm_handle) {
            ALOGE("SIMCOM: failed to open TX PCM: %s",
                  pcm_get_error(pcm_handle));
            pcm_close(pcm_handle);
        } else {
            ALOGE("SIMCOM: failed to allocate TX PCM handle (err=%d)", -err);
        }

        if (!wait_for_device ||
            (err != -EBUSY && err != -EAGAIN && err != -ENOENT && err != -EINVAL)) {
            break;
        }
        usleep(SIMCOM_WAIT_STEP_MS * 1000);
    }

    adev->simcom_tx_pcm = NULL;
    adev->simcom_tx_users = 0;
    return (last_err == 0) ? -ENODEV : last_err;
}

static void simcom_release_tx_pcm(struct audio_device *adev)
{
    if (adev->simcom_tx_users > 0) {
        adev->simcom_tx_users--;
        if (adev->simcom_tx_users == 0) {
            if (adev->simcom_tx_pcm) {
                pcm_close(adev->simcom_tx_pcm);
                adev->simcom_tx_pcm = NULL;
                ALOGI("SIMCOM: telephony TX PCM closed");
            }
        } else {
            ALOGD("SIMCOM: telephony TX PCM still in use (%d)", adev->simcom_tx_users);
        }
    }
}

static int simcom_acquire_rx_pcm(struct audio_device *adev, bool wait_for_device)
{
    if (!adev) {
        return -EINVAL;
    }

    if (!simcom_wait_for_pcm_ready(adev)) {
        ALOGW("SIMCOM: telephony RX requested before modem PCM ready");
        return -EAGAIN;
    }

    if (adev->simcom_rx_pcm) {
        adev->simcom_rx_users++;
        ALOGD("SIMCOM: telephony RX PCM reuse, users=%d", adev->simcom_rx_users);
        return 0;
    }

    if (!simcom_detect_card(adev)) {
        ALOGE("SIMCOM: telephony RX requested but SIMCOM device not detected");
        return -ENODEV;
    }

    int ret = simcom_wait_for_pcm_node(adev, true, wait_for_device);
    if (ret != 0) {
        return (ret == -ETIMEDOUT || ret == -ENOENT) ? -EAGAIN : ret;
    }

    const int card = simcom_pcm_card_index(adev);
    const int device = simcom_pcm_device_index(adev);
    const int attempts = wait_for_device ? 5 : 1;
    int last_err = -EIO;

    for (int attempt = 1; attempt <= attempts; ++attempt) {
        ALOGD("SIMCOM: opening shared telephony RX PCM (card %d device %d), attempt %d/%d",
              card, device, attempt, attempts);
        struct pcm *pcm_handle = pcm_open(card, device, PCM_IN, &simcom_pcm_config_rx);
        if (pcm_handle && pcm_is_ready(pcm_handle)) {
            adev->simcom_rx_pcm = pcm_handle;
            adev->simcom_rx_users = 1;
            simcom_rx_bus_reset(adev);
            // SIMCOM: Prepare PCM to clear ALSA buffers and reset state
            pcm_prepare(pcm_handle);
            ALOGI("SIMCOM: telephony RX PCM ready on card %d device %d (prepared and cleared)", card, device);
            return 0;
        }

        int err = errno ? -errno : -ENODEV;
        last_err = err;
        if (pcm_handle) {
            ALOGE("SIMCOM: failed to open RX PCM: %s",
                  pcm_get_error(pcm_handle));
            pcm_close(pcm_handle);
        } else {
            ALOGE("SIMCOM: failed to allocate RX PCM handle (err=%d)", -err);
        }

        if (!wait_for_device ||
            (err != -EBUSY && err != -EAGAIN && err != -ENOENT && err != -EINVAL)) {
            break;
        }
        usleep(SIMCOM_WAIT_STEP_MS * 1000);
    }

    adev->simcom_rx_pcm = NULL;
    adev->simcom_rx_users = 0;
    return (last_err == 0) ? -ENODEV : last_err;
}

static void simcom_release_rx_pcm(struct audio_device *adev)
{
    if (adev->simcom_rx_users > 0) {
        simcom_rx_bus_drop_consumer(adev);
        adev->simcom_rx_users--;
        if (adev->simcom_rx_users == 0) {
            if (adev->simcom_rx_pcm) {
                pcm_close(adev->simcom_rx_pcm);
                adev->simcom_rx_pcm = NULL;
                ALOGI("SIMCOM: telephony RX PCM closed");
            }
            simcom_rx_bus_reset(adev);
        } else {
            ALOGD("SIMCOM: telephony RX PCM still in use (%d)", adev->simcom_rx_users);
        }
    }
}

struct SurroundFormat {
    audio_format_t format;
    const char *value;
};


const struct SurroundFormat sSurroundFormat[] = {
    {AUDIO_FORMAT_AC3,"AUDIO_FORMAT_AC3"},
    {AUDIO_FORMAT_E_AC3,"AUDIO_FORMAT_E_AC3"},
    {AUDIO_FORMAT_DTS,"AUDIO_FORMAT_DTS"},
    {AUDIO_FORMAT_DTS_HD,"AUDIO_FORMAT_DTS_HD"},
    {AUDIO_FORMAT_AAC_LC,"AUDIO_FORMAT_AAC_LC"},
    {AUDIO_FORMAT_DOLBY_TRUEHD,"AUDIO_FORMAT_DOLBY_TRUEHD"},
    {AUDIO_FORMAT_AC4,"AUDIO_FORMAT_E_AC3_JOC"}
};

enum SOUND_CARD_OWNER{
    SOUND_CARD_HDMI = 0,
    SOUND_CARD_SPDIF = 1,
};

/*
 * mute audio datas when screen off or standby
 * The MediaPlayer no stop/pause when screen off, they may be just play in background,
 * so they still send audio datas to audio hal.
 * HDMI may disconnet and enter stanby status, this means no voice output on HDMI
 * but speaker/av and spdif still work, and voice may output on them.
 * Some customer need to mute the audio datas in this condition.
 * If need mute datas when screen off, define this marco.
 */
//#define MUTE_WHEN_SCREEN_OFF

/*
 * if current audio stream bitstream over hdmi,
 * and hdmi is removed and reconnected later,
 * the driver of hdmi may config it with pcm mode automatically,
 * which is according the implement of hdmi driver.
 * If hdmi driver implement in this way, in order to output audio
 * bitstream stream after hdmi reconnected,
 * we must close sound card of hdmi and reopen/config
 * it in bitstream mode. If need this, define this macro.
 */
#define AUDIO_BITSTREAM_REOPEN_HDMI

//#define ALSA_DEBUG
#ifdef ALSA_IN_DEBUG
FILE *in_debug;
#endif

int in_dump(const struct audio_stream *stream, int fd);
int out_dump(const struct audio_stream *stream, int fd);
static inline bool hasExtCodec();

/**
 * @brief get_output_device_id
 *
 * @param device
 *
 * @returns
 */
int get_output_device_id(audio_devices_t device)
{
    if (device == AUDIO_DEVICE_NONE)
        return OUT_DEVICE_NONE;

    if (popcount(device) == 2) {
        if ((device == (AUDIO_DEVICE_OUT_SPEAKER |
                        AUDIO_DEVICE_OUT_WIRED_HEADSET)) ||
                (device == (AUDIO_DEVICE_OUT_SPEAKER |
                            AUDIO_DEVICE_OUT_WIRED_HEADPHONE)))
            return OUT_DEVICE_SPEAKER_AND_HEADSET;
        else
            return OUT_DEVICE_NONE;
    }

    if (popcount(device) != 1)
        return OUT_DEVICE_NONE;

    switch (device) {
    case AUDIO_DEVICE_OUT_SPEAKER:
        return OUT_DEVICE_SPEAKER;
    case AUDIO_DEVICE_OUT_WIRED_HEADSET:
        return OUT_DEVICE_HEADSET;
    case AUDIO_DEVICE_OUT_WIRED_HEADPHONE:
        return OUT_DEVICE_HEADPHONES;
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO:
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET:
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT:
        return OUT_DEVICE_BT_SCO;
    default:
        return OUT_DEVICE_NONE;
    }
}

/**
 * @brief get_input_source_id
 *
 * @param source
 *
 * @returns
 */
int get_input_source_id(audio_source_t source)
{
    switch (source) {
    case AUDIO_SOURCE_DEFAULT:
        return IN_SOURCE_NONE;
    case AUDIO_SOURCE_MIC:
        return IN_SOURCE_MIC;
    case AUDIO_SOURCE_CAMCORDER:
        return IN_SOURCE_CAMCORDER;
    case AUDIO_SOURCE_VOICE_RECOGNITION:
        return IN_SOURCE_VOICE_RECOGNITION;
    case AUDIO_SOURCE_VOICE_COMMUNICATION:
        return IN_SOURCE_VOICE_COMMUNICATION;
    default:
        return IN_SOURCE_NONE;
    }
}

/**
 * @brief force_non_hdmi_out_standby
 * must be called with hw device outputs list, all out streams, and hw device mutexes locked
 *
 * @param adev
 */
static void force_non_hdmi_out_standby(struct audio_device *adev)
{
    enum output_type type;
    struct stream_out *out;
    for (type = 0; type < OUTPUT_TOTAL; ++type) {
        out = adev->outputs[type];
        if (type == OUTPUT_HDMI_MULTI|| !out)
            continue;
        /* This will never recurse more than 2 levels deep. */
        do_out_standby(out);
    }
}

static int simcom_parse_card_index(const char *line)
{
    if (!line) {
        return -1;
    }

    while (*line && !isdigit((unsigned char)*line)) {
        ++line;
    }
    if (!*line) {
        return -1;
    }

    char *end = NULL;
    long value = strtol(line, &end, 10);
    if (end == line || value < 0 || value >= SNDRV_CARDS) {
        return -1;
    }
    return (int)value;
}

static bool simcom_detect_card(struct audio_device *adev)
{
    if (!adev) {
        return false;
    }

    FILE *fp = fopen(SND_CARDS_NODE, "r");
    if (!fp) {
        ALOGW("SIMCOM: failed to open %s: %s", SND_CARDS_NODE, strerror(errno));
        adev->simcom_card_available = false;
        adev->simcom_pcm_card = -1;
        return false;
    }

    char line[256];
    int detected = -1;
    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, SIMCOM_CARD_ID_STRING)) {
            detected = simcom_parse_card_index(line);
            if (detected >= 0) {
                ALOGI("SIMCOM audio device detected on card %d: %s", detected, line);
            } else {
                ALOGW("SIMCOM: identifier found but card index parse failed: %s", line);
            }
            break;
        }
    }
    fclose(fp);

    if (detected >= 0) {
        if (adev->simcom_pcm_card != detected) {
            ALOGI("SIMCOM: card index updated %d -> %d",
                  adev->simcom_pcm_card, detected);
        }
        if (detected != SIMCOM_PCM_CARD_FALLBACK) {
            ALOGW("SIMCOM: card index is %d (expected %d); HAL will adapt dynamically",
                  detected, SIMCOM_PCM_CARD_FALLBACK);
        }
        adev->simcom_pcm_card = detected;
        adev->simcom_card_available = true;
        return true;
    }

    if (adev->simcom_card_available) {
        ALOGW("SIMCOM: audio device disappeared from %s", SND_CARDS_NODE);
    } else {
        ALOGW("SIMCOM: audio device not found in %s", SND_CARDS_NODE);
    }
    adev->simcom_card_available = false;
    adev->simcom_pcm_card = -1;
    return false;
}

static int simcom_wait_for_pcm_node(struct audio_device *adev, bool is_rx, bool wait_for_device)
{
    if (!adev) {
        return -EINVAL;
    }

    const int timeout_ms = is_rx ? SIMCOM_RX_TIMEOUT_MS : SIMCOM_TX_TIMEOUT_MS;
    int elapsed_ms = 0;
    char node_path[64];

    while (true) {
        const int card = simcom_pcm_card_index(adev);
        const int device = simcom_pcm_device_index(adev);
        snprintf(node_path, sizeof(node_path), "/dev/snd/pcmC%dD%d%c",
                 card, device, is_rx ? 'c' : 'p');

        if (access(node_path, F_OK) == 0) {
            if (wait_for_device && elapsed_ms > 0) {
                ALOGI("SIMCOM: %s PCM node %s ready after %d ms",
                      is_rx ? "RX" : "TX", node_path, elapsed_ms);
            }
            return 0;
        }

        if (!wait_for_device) {
            return -ENOENT;
        }

        if (elapsed_ms == 0) {
            ALOGW("SIMCOM: waiting for %s PCM node %s", is_rx ? "RX" : "TX", node_path);
        }

        if (elapsed_ms >= timeout_ms) {
            ALOGE("SIMCOM: %s PCM node %s not available within %d ms",
                  is_rx ? "RX" : "TX", node_path, timeout_ms);
            return -ETIMEDOUT;
        }

        usleep(SIMCOM_WAIT_STEP_MS * 1000);
        elapsed_ms += SIMCOM_WAIT_STEP_MS;
        simcom_detect_card(adev);
    }
}

static bool simcom_voice_mode_active(struct audio_device *adev)
{
    return adev->simcom_card_available &&
           adev->voice_call_active &&
           adev->mode == AUDIO_MODE_IN_CALL;
}

/**
 * @brief getOutputRouteFromDevice
 *
 * @param device
 *
 * @returns
 */
unsigned getOutputRouteFromDevice(uint32_t device)
{
    /*if (mMode != AudioSystem::MODE_RINGTONE && mMode != AudioSystem::MODE_NORMAL)
        return PLAYBACK_OFF_ROUTE;
    */
    switch (device) {
    case AUDIO_DEVICE_OUT_SPEAKER:
        return SPEAKER_NORMAL_ROUTE;
    case AUDIO_DEVICE_OUT_WIRED_HEADSET:
        return HEADSET_NORMAL_ROUTE;
    case AUDIO_DEVICE_OUT_WIRED_HEADPHONE:
        return HEADPHONE_NORMAL_ROUTE;
    case (AUDIO_DEVICE_OUT_SPEAKER|AUDIO_DEVICE_OUT_WIRED_HEADPHONE):
    case (AUDIO_DEVICE_OUT_SPEAKER|AUDIO_DEVICE_OUT_WIRED_HEADSET):
        return SPEAKER_HEADPHONE_NORMAL_ROUTE;
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO:
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET:
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT:
        return BLUETOOTH_NORMAL_ROUTE;
    case AUDIO_DEVICE_OUT_AUX_DIGITAL:
	return HDMI_NORMAL_ROUTE;
        //case AudioSystem::DEVICE_OUT_EARPIECE:
        //	return EARPIECE_NORMAL_ROUTE;
        //case AudioSystem::DEVICE_OUT_ANLG_DOCK_HEADSET:
        //case AudioSystem::DEVICE_OUT_DGTL_DOCK_HEADSET:
        //	return USB_NORMAL_ROUTE;
    default:
        return PLAYBACK_OFF_ROUTE;
    }
}

/**
 * @brief getVoiceRouteFromDevice
 *
 * @param device
 *
 * @returns
 */
uint32_t getVoiceRouteFromDevice(uint32_t device)
{
    ALOGE("not support now");
    return 0;
}

/**
 * @brief getInputRouteFromDevice
 *
 * @param device
 *
 * @returns
 */
uint32_t getInputRouteFromDevice(uint32_t device)
{
    /*if (mMicMute) {
        return CAPTURE_OFF_ROUTE;
    }*/
    ALOGE("%s:device:%x",__FUNCTION__,device);
    switch (device) {
    case AUDIO_DEVICE_IN_BUILTIN_MIC:
        return MAIN_MIC_CAPTURE_ROUTE;
    case AUDIO_DEVICE_IN_WIRED_HEADSET:
        return HANDS_FREE_MIC_CAPTURE_ROUTE;
    case AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET:
        return BLUETOOTH_SOC_MIC_CAPTURE_ROUTE;
    case AUDIO_DEVICE_IN_ANLG_DOCK_HEADSET:
        return USB_CAPTURE_ROUTE;
    case AUDIO_DEVICE_IN_HDMI:
        return HDMI_IN_CAPTURE_ROUTE;
    default:
        return CAPTURE_OFF_ROUTE;
    }
}

/**
 * @brief getRouteFromDevice
 *
 * @param device
 *
 * @returns
 */
uint32_t getRouteFromDevice(uint32_t device)
{
    if (device & AUDIO_DEVICE_BIT_IN)
        return getInputRouteFromDevice(device);
    else
        return getOutputRouteFromDevice(device);
}

struct dev_proc_info SPEAKER_OUT_NAME[] = /* add codes& dai name here*/
{
    {"realtekrt5616c", NULL,},
    {"realtekrt5651co", "rt5651-aif1",},
    {"realtekrt5670c", NULL,},
    {"realtekrt5672c", NULL,},
    {"realtekrt5678co", NULL,},
    {"rkhdmianalogsnd", NULL,},
    {"rockchipcx2072x", NULL,},
    {"rockchipes8316c", NULL,},
    {"rockchipes8323c", NULL,},
    {"rockchipes8388c", NULL,},
    {"rockchipes8396c", NULL,},
    {"rockchiprk", NULL, },
    {"rockchiprk809co", NULL,},
    {"rockchiprk817co", NULL,},
    {"rockchiprt5640c", "rt5640-aif1",},
    {"rockchiprt5670c", NULL,},
    {"rockchiprt5672c", NULL,},
    {NULL, NULL}, /* Note! Must end with NULL, else will cause crash */
};

struct dev_proc_info HDMI_OUT_NAME[] =
{
    {"realtekrt5651co", "i2s-hifi",},
    {"realtekrt5670co", "i2s-hifi",},
    {"rkhdmidpsound", NULL,},
    {"rockchiphdmi", NULL,},
    {"rockchiprt5640c", "i2s-hifi",},
    {NULL, NULL}, /* Note! Must end with NULL, else will cause crash */
};


struct dev_proc_info SPDIF_OUT_NAME[] =
{
    {"ROCKCHIPSPDIF", "dit-hifi",},
    {"rockchipspdif", NULL,},
    {"rockchipcdndp", NULL,},
    {NULL, NULL}, /* Note! Must end with NULL, else will cause crash */
};

struct dev_proc_info BT_OUT_NAME[] =
{
    {"rockchipbt", NULL,},
    {NULL, NULL}, /* Note! Must end with NULL, else will cause crash */
};

struct dev_proc_info SIMCOM_OUT_NAME[] =
{
    {SIMCOM_CARD_ID_STRING, NULL,},
    {"simcom_audio", NULL,},
    {"SIMCom", NULL,},
    {NULL, NULL},
};

struct dev_proc_info MIC_IN_NAME[] =
{
    {"realtekrt5616c", NULL,},
    {"realtekrt5651co", "rt5651-aif1",},
    {"realtekrt5670c", NULL,},
    {"realtekrt5672c", NULL,},
    {"realtekrt5678co", NULL,},
    {"rockchipes8316c", NULL,},
    {"rockchipes8323c", NULL,},
    {"rockchipes8396c", NULL,},
    {"rockchipes7210", NULL,},
    {"rockchipes7243", NULL,},
    {"rockchiprk", NULL, },
    {"rockchiprk809co", NULL,},
    {"rockchiprk817co", NULL,},
    {"rockchiprt5640c", NULL,},
    {"rockchiprt5670c", NULL,},
    {"rockchiprt5672c", NULL,},
    {NULL, NULL}, /* Note! Must end with NULL, else will cause crash */
};


struct dev_proc_info HDMI_IN_NAME[] =
{
    {"realtekrt5651co", "tc358749x-audio"},
    {"hdmiin", NULL},
    {NULL, NULL}, /* Note! Must end with NULL, else will cause crash */
};

struct dev_proc_info BT_IN_NAME[] =
{
    {"rockchipbt", NULL},
    {NULL, NULL}, /* Note! Must end with NULL, else will cause crash */
};

struct dev_proc_info SIMCOM_IN_NAME[] =
{
    {SIMCOM_CARD_ID_STRING, NULL,},
    {"simcom_audio", NULL,},
    {"SIMCom", NULL,},
    {NULL, NULL},
};

static int name_match(const char* dst, const char* src)
{
    int score = 0;
    // total equal
    if (!strcmp(dst, src)) {
        score = 100;
    } else  if (strstr(dst, src)) {
        // part equal
        score = 50;
    }

    return score;
}

static bool is_specified_out_sound_card(char *id, struct dev_proc_info *match)
{
    int i = 0;

    if (!match)
        return true; /* match any */

    while (match[i].cid) {
        if (!strcmp(id, match[i].cid)) {
            return true;
    }
        i++;
    }
    return false;
}

static bool dev_id_match(const char *info, const char *did)
{
    const char *deli = "id:";
    char *id;
    int idx = 0;

    if (!did)
        return true;
    if (!info)
        return false;
    /* find str like-> id: ff880000.i2s-rt5651-aif1 rt5651-aif1-0 */
    id = strstr(info, deli);
    if (!id)
        return false;
    id += strlen(deli);
    while(id[idx] != '\0') {
        if (id[idx] == '\r' ||id[idx] == '\n') {
            id[idx] = '\0';
            break;
    }
        idx ++;
    }
    if (strstr(id, did)) {
        ALOGE("match dai!!!: %s (dai=%s)", id, did ? did : "null");
        return true;
    }
    return false;
}

static bool get_specified_out_dev(struct dev_info *devinfo,
                                  int card,
                                  const char *id,
                                  struct dev_proc_info *match)
{
    int i = 0;
    int device;
    char str_device[32];
    char info[256];
    size_t len;
    FILE* file = NULL;
    int better = 0;
    int index = -1;

    /* parse card id */
    if (!match)
        return true; /* match any */
    while (match[i].cid) {
        int score = name_match(id, match[i].cid);
        if (score > better) {
            better = score;
            index = i;
        }
        i++;
    }

    if (index < 0)
        return false;

    if (!match[index].cid)
        return false;

    if (!match[index].did) { /* no exist dai info, exit */
        devinfo->card = card;
        devinfo->device = 0;
        ALOGD("%s card, got card=%d,device=%d", devinfo->id,
              devinfo->card, devinfo->device);
        return true;
    }

    /* parse device id */
    for (device = 0; device < SNDRV_DEVICES; device++) {
        sprintf(str_device, "proc/asound/card%d/pcm%dp/info", card, device);
        if (access(str_device, 0)) {
            ALOGD("No exist %s, break and finish parsing", str_device);
            break;
        }
        file = fopen(str_device, "r");
        if (!file) {
            ALOGD("Could reading %s property", str_device);
            continue;
        }
        len = fread(info, sizeof(char), sizeof(info)/sizeof(char), file);
        fclose(file);
        if (len == 0 || len > sizeof(info)/sizeof(char))
            continue;
        if (info[len - 1] == '\n') {
            len--;
            info[len] = '\0';
        }
        /* parse device dai */
        if (dev_id_match(info, match[index].did)) {
            devinfo->card = card;
            devinfo->device = device;
            ALOGD("%s card, got card=%d,device=%d", devinfo->id,
                  devinfo->card, devinfo->device);
        return true;
    }
    }
    return false;
}

static bool get_specified_in_dev(struct dev_info *devinfo,
                                 int card,
                                 const char *id,
                                 struct dev_proc_info *match)
{
    int i = 0;
    int device;
    char str_device[32];
    char info[256];
    size_t len;
    FILE* file = NULL;
    int better = 0;
    int index = -1;

    /* parse card id */
    if (!match)
        return true; /* match any */
    while (match[i].cid) {
        int score = name_match(id, match[i].cid);
        if (score > better) {
            better = score;
            index = i;
        }
        i++;
    }

    if (index < 0)
        return false;

    if (!match[index].cid)
        return false;

    if (!match[index].did) { /* no exist dai info, exit */
        devinfo->card = card;
        devinfo->device = 0;
        ALOGD("%s card, got card=%d,device=%d", devinfo->id,
              devinfo->card, devinfo->device);
        return true;
    }

    /* parse device id */
    for (device = 0; device < SNDRV_DEVICES; device++) {
        sprintf(str_device, "proc/asound/card%d/pcm%dc/info", card, device);
        if (access(str_device, 0)) {
            ALOGD("No exist %s, break and finish parsing", str_device);
            break;
        }
        file = fopen(str_device, "r");
        if (!file) {
            ALOGD("Could reading %s property", str_device);
            continue;
        }
        len = fread(info, sizeof(char), sizeof(info)/sizeof(char), file);
        fclose(file);
        if (len == 0 || len > sizeof(info)/sizeof(char))
            continue;
        if (info[len - 1] == '\n') {
            len--;
            info[len] = '\0';
        }
        /* parse device dai */
        if (dev_id_match(info, match[i].did)) {
            devinfo->card = card;
            devinfo->device = device;
            ALOGD("%s card, got card=%d,device=%d", devinfo->id,
                  devinfo->card, devinfo->device);
            return true;
        }
    }
    return false;
}

static bool is_specified_in_sound_card(char *id, struct dev_proc_info *match)
{
    int i = 0;

    /*
     * mic: diffrent product may have diffrent card name,modify codes here
     * for example: 0 [rockchiprk3328 ]: rockchip-rk3328 - rockchip-rk3328
     */
    if (!match)
        return true;/* match any */
    while (match[i].cid) {
        if (!strcmp(id, match[i].cid)) {
            return true;
  }
        i++;
    }
    return false;
}

static void set_default_dev_info( struct dev_info *info, int size, int rid)
{
    for(int i =0; i < size; i++) {
        if (rid) {
            info[i].id = NULL;
        }
        info[i].card = (int)SND_OUT_SOUND_CARD_UNKNOWN;
    }
}

static void dumpdev_info(const char *tag, struct dev_info  *devinfo, int count)
{
    ALOGD("dump %s device info", tag);
    for(int i = 0; i < count; i++) {
        if (devinfo[i].id && devinfo[i].card != SND_OUT_SOUND_CARD_UNKNOWN)
            ALOGD("dev_info %s  card=%d, device:%d", devinfo[i].id,
                  devinfo[i].card,
                  devinfo[i].device);
    }
}

/*
 * get sound card infor by parser node: /proc/asound/cards
 * the sound card number is not always the same value
 */
static void read_out_sound_card(struct stream_out *out)
{

    struct audio_device *device = NULL;
    int card = 0;
    char str[32];
    char id[20];
    size_t len;
    FILE* file = NULL;

    if((out == NULL) || (out->dev == NULL)) {
        return ;
    }
    device = out->dev;
    set_default_dev_info(device->dev_out, SND_OUT_SOUND_CARD_UNKNOWN, 0);
    for (card = 0; card < SNDRV_CARDS; card++) {
        sprintf(str, "proc/asound/card%d/id", card);
        if (access(str, 0)) {
            ALOGD("No exist %s, break and finish parsing", str);
            break;
        }
        file = fopen(str, "r");
        if (!file) {
            ALOGD("Could reading %s property", str);
            continue;
        }
        len = fread(id, sizeof(char), sizeof(id)/sizeof(char), file);
        fclose(file);
        if (len == 0 || len > sizeof(id)/sizeof(char))
            continue;
        if (id[len - 1] == '\n') {
            len--;
            id[len] = '\0';
        }
        ALOGD("card%d id:%s", card, id);
        get_specified_out_dev(&device->dev_out[SND_OUT_SOUND_CARD_SPEAKER], card, id, SPEAKER_OUT_NAME);
        get_specified_out_dev(&device->dev_out[SND_OUT_SOUND_CARD_HDMI], card, id, HDMI_OUT_NAME);
        get_specified_out_dev(&device->dev_out[SND_OUT_SOUND_CARD_SPDIF], card, id, SPDIF_OUT_NAME);
        get_specified_out_dev(&device->dev_out[SND_OUT_SOUND_CARD_BT], card, id, BT_OUT_NAME);
        get_specified_out_dev(&device->dev_out[SND_OUT_SOUND_CARD_SIMCOM], card, id, SIMCOM_OUT_NAME);
    }
    dumpdev_info("out", device->dev_out, SND_OUT_SOUND_CARD_MAX);
    return ;
}

/*
 * get sound card infor by parser node: /proc/asound/cards
 * the sound card number is not always the same value
 */
static void read_in_sound_card(struct stream_in *in)
{
    struct audio_device *device = NULL;
    int card = 0;
    char str[32];
    char id[20];
    size_t len;
    FILE* file = NULL;

    if((in == NULL) || (in->dev == NULL)){
        return ;
    }
    device = in->dev;
    set_default_dev_info(device->dev_in, SND_IN_SOUND_CARD_UNKNOWN, 0);
    for (card = 0; card < SNDRV_CARDS; card++) {
        sprintf(str, "proc/asound/card%d/id", card);
        if(access(str, 0)) {
            ALOGD("No exist %s, break and finish parsing", str);
                break;
        }
        file = fopen(str, "r");
        if (!file) {
            ALOGD("Could reading %s property", str);
            continue;
        }
        len = fread(id, sizeof(char), sizeof(id)/sizeof(char), file);
        fclose(file);
        if (len == 0 || len > sizeof(id)/sizeof(char))
            continue;
        if (id[len - 1] == '\n') {
            len--;
           id[len] = '\0';
        }
        get_specified_in_dev(&device->dev_in[SND_IN_SOUND_CARD_MIC], card, id, MIC_IN_NAME);
        /* set HDMI audio input info if need hdmi audio input */
        get_specified_in_dev(&device->dev_in[SND_IN_SOUND_CARD_HDMI], card, id, HDMI_IN_NAME);
        get_specified_in_dev(&device->dev_in[SND_IN_SOUND_CARD_BT], card, id, BT_IN_NAME);
        get_specified_in_dev(&device->dev_in[SND_IN_SOUND_CARD_SIMCOM], card, id, SIMCOM_IN_NAME);
    }
    dumpdev_info("in", device->dev_in, SND_IN_SOUND_CARD_MAX);
    return ;
}

static inline bool hasExtCodec()
{
    char line[80];
    bool ret = false;
    FILE *fd = fopen("proc/asound/cards","r");
    if(NULL != fd){
      memset(line, 0, 80);
      while((fgets(line,80,fd))!= NULL){
          line[80-1]='\0';
          if(strstr(line,"realtekrt5651co")){
              ret = true;
              break;
          }
      }
      fclose(fd);
    }
    return ret;
}

static bool is_bitstream(struct stream_out *out)
{
    if (out == NULL) {
        return false;
    }

    bool bitstream = false;
    if (out->output_direct) {
        switch(out->output_direct_mode) {
            case HBR:
            case NLPCM:
                bitstream = true;
                break;
            case LPCM:
            default:
                bitstream = false;
                break;
        }
    } else {
        if(out->output_direct_mode != LPCM) {
            ALOGD("%s: %d: error output_direct = false, but output_direct_mode != LPCM, this is error config",__FUNCTION__,__LINE__);
        }
    }

    return bitstream;
}

static bool is_multi_pcm(struct stream_out *out)
{
    if (out == NULL) {
        return false;
    }

    bool multi = false;
    if (out->output_direct && (out->output_direct_mode == LPCM) && (out->config.channels > 2)) {
        multi = true;
    }

    return multi;
}

/**
 * @brief mixer_mode_set
 * for rk3399 audio output mixer mode set
 * @param out
 *
 * @return
 */
static int mixer_mode_set(struct stream_out *out)
{
    int ret = 0;
    struct mixer *pMixer = NULL;
    struct mixer_ctl *pctl;
    struct audio_device *adev = out->dev;

    /*
     * set audio mode for hdmi
     * The driver of hdmi read the audio mode to identify
     * the type of audio stream according to audio mode.
     * 1) LPCM: the stream is pcm format
     * 2) NLPCM: the stream is bitstream format, AC3/EAC3/DTS use this format
     * 3) HDR: the stream is bitstream format, TrueHD/Atoms/DTS-HD/DTS-X use this format.
     */
    if (out->device & AUDIO_DEVICE_OUT_AUX_DIGITAL) {
        pMixer = mixer_open_legacy(adev->dev_out[SND_OUT_SOUND_CARD_HDMI].card);
        if (!pMixer) {
            ALOGE("mMixer is a null point %s %d,CARD = %d",__func__, __LINE__,adev->dev_out[SND_OUT_SOUND_CARD_HDMI].card);
            return ret;
        }
        pctl = mixer_get_control(pMixer,"AUDIO MODE",0 );
        if (pctl != NULL) {
            ALOGD("Now set mixer audio_mode is %d for drm",out->output_direct_mode);
            switch (out->output_direct_mode) {
            case HBR:
                ret = mixer_ctl_set_val(pctl , out->output_direct_mode);
                break;
            case NLPCM:
                ret = mixer_ctl_set_val(pctl , out->output_direct_mode);
                break;
            default:
                ret = mixer_ctl_set_val(pctl , out->output_direct_mode);
                break;
            }

            if (ret != 0) {
                ALOGE("set_controls() can not set ctl!");
                mixer_close_legacy(pMixer);
                return -EINVAL;
            }
        }
        mixer_close_legacy(pMixer);
    }

    return ret;
}


static void open_sound_card_policy(struct stream_out *out)
{
    if (out == NULL) {
        return ;
    }

    if (is_bitstream(out) || (is_multi_pcm(out))) {
        return ;
    }

    /*
     * In Box Product, ouput 2 channles pcm datas over hdmi,speaker and spdif simultaneous.
     * speaker can only support 44.1k or 48k
     */
    bool support = ((out->config.rate == 44100) || (out->config.rate == 48000));
    struct audio_device *adev = out->dev;
    if (support) {
        if(adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card != SND_OUT_SOUND_CARD_UNKNOWN) {
            out->device |= AUDIO_DEVICE_OUT_SPEAKER;
        }

        if(adev->dev_out[SND_OUT_SOUND_CARD_HDMI].card != SND_OUT_SOUND_CARD_UNKNOWN) {
            /*
             * hdmi is taken by direct/mulit pcm output
             */
            if(adev->outputs[OUTPUT_HDMI_MULTI] != NULL) {
                out->device &= ~AUDIO_DEVICE_OUT_AUX_DIGITAL;
            } else {
                out->device |= AUDIO_DEVICE_OUT_AUX_DIGITAL;
            }
        }

        if(adev->dev_out[SND_OUT_SOUND_CARD_SPDIF].card != SND_OUT_SOUND_CARD_UNKNOWN){
           out->device |= AUDIO_DEVICE_OUT_SPDIF;
        }
    }

    // some specail config for chips
#ifdef RK3288
    /*3288's hdmi & codec use the same i2s,so only config the codec card*/
    audio_devices_t devices = (AUDIO_DEVICE_OUT_AUX_DIGITAL|AUDIO_DEVICE_OUT_SPEAKER);
    if ((out->device & devices) == devices) {
        out->device &= ~AUDIO_DEVICE_OUT_AUX_DIGITAL;
    }
#endif
}

/**
 * @brief start_output_stream
 * must be called with hw device outputs list, output stream, and hw device mutexes locked
 *
 * @param out
 *
 * @returns
 */
static int start_output_stream(struct stream_out *out)
{
    struct audio_device *adev = out->dev;
    int ret = 0;
    int card = (int)SND_OUT_SOUND_CARD_UNKNOWN;
    int device = 0;
    // set defualt value to true for compatible with mid project
    bool disable = true;

if (!hasExtCodec()){
    /*
     * In Box Project, if output stream is 2 channels pcm,
     * the 2 channels pcm can simultaneous output over speaker,hdmi and spdif.
     * If hdmi is already opened in multi pcm mode or bitstream mode,
     * and new output stream want open it in
     * multi pcm mode or bitstream mode, disable it.
     * Otherwise, if new output stream is 2 channels pcm,
     * we can let output over speaker and spdif, this it
     * better than usleep in out_write function
     */
    disable = is_multi_pcm(out) || is_bitstream(out);
}

    if (out->is_simcom_voice) {
        if (!(out->device & AUDIO_DEVICE_OUT_TELEPHONY_TX)) {
            ALOGW("SIMCOM: telephony TX stream requested on non-Telephony device 0x%x, skipping pcm_open",
                  out->device);
            return 0;
        }
        if (out->simcom_attached) {
            ALOGD("SIMCOM: telephony TX stream already attached");
            return 0;
        }
        ALOGI("SIMCOM: start_output_stream Telephony TX (device=0x%x)", out->device);
        out->config = simcom_pcm_config_tx;
        ret = simcom_acquire_tx_pcm(adev, true);
        if (ret == 0) {
            out->simcom_attached = true;
            ALOGI("SIMCOM: telephony TX PCM attached (pcm=%p users=%d)",
                  OUT_SIMCOM_PCM(out), adev->simcom_tx_users);
            return 0;
        }
        if (ret == -EAGAIN) {
            ALOGW("SIMCOM: telephony TX PCM not ready yet, deferring attachment");
            return 0;
        }
        ALOGE("SIMCOM: telephony TX PCM acquire failed ret=%d", ret);
        return ret;
}

    ALOGD("%s:%d out = %p,device = 0x%x,outputs[OUTPUT_HDMI_MULTI] = %p",__FUNCTION__,__LINE__,out,out->device,adev->outputs[OUTPUT_HDMI_MULTI]);
    if (out == adev->outputs[OUTPUT_HDMI_MULTI]) {
        force_non_hdmi_out_standby(adev);
    } else if (adev->outputs[OUTPUT_HDMI_MULTI] &&
            !adev->outputs[OUTPUT_HDMI_MULTI]->standby) {
        out->disabled = true;
        return 0;
    }

    out->disabled = false;
    read_out_sound_card(out);

if (!hasExtCodec()){
    open_sound_card_policy(out);
}

    out_dump(out, 0);
    // SIMCOM: Activate route with voice call route if in call mode
    uint32_t route;
    if (adev->mode == AUDIO_MODE_IN_CALL && adev->voice_call_active) {
        // SIMCOM: Use SPEAKER_HEADPHONE_INCALL_ROUTE if both speaker and headphone are selected
        if ((out->device & AUDIO_DEVICE_OUT_SPEAKER) && 
            (out->device & (AUDIO_DEVICE_OUT_WIRED_HEADPHONE | AUDIO_DEVICE_OUT_WIRED_HEADSET))) {
            route = SPEAKER_HEADPHONE_INCALL_ROUTE;
            ALOGI("SIMCOM: start_output_stream using SPEAKER_HEADPHONE_INCALL_ROUTE for device=0x%x", out->device);
        } else if (out->device & AUDIO_DEVICE_OUT_SPEAKER) {
            route = SPEAKER_INCALL_ROUTE;
        } else if (out->device & AUDIO_DEVICE_OUT_WIRED_HEADPHONE) {
            route = HEADPHONE_INCALL_ROUTE;
        } else if (out->device & AUDIO_DEVICE_OUT_WIRED_HEADSET) {
            route = HEADSET_INCALL_ROUTE;
        } else {
            route = getRouteFromDevice(out->device);
        }
        ALOGI("SIMCOM: start_output_stream activating voice route=%u for device=0x%x", route, out->device);
    } else {
        route = getRouteFromDevice(out->device);
    }
    ALOGI("SIMCOM: start_output_stream activating route=%u for device=0x%x (card=%d)", 
          route, out->device, adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card);
    route_pcm_card_open(adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card, route);

    // SIMCOM: For bypass_pcm streams (voice call RX path), PCM is opened normally
    // No special handling needed - just continue to open PCM devices below

    if (out->device & AUDIO_DEVICE_OUT_AUX_DIGITAL) {
        if (adev->owner[SOUND_CARD_HDMI] == NULL) {
            card = adev->dev_out[SND_OUT_SOUND_CARD_HDMI].card;
            device =adev->dev_out[SND_OUT_SOUND_CARD_HDMI].device;
            if (card != (int)SND_OUT_SOUND_CARD_UNKNOWN) {
if (!hasExtCodec()){			
#ifdef USE_DRM
            ret = mixer_mode_set(out);

            if (ret!=0) {
                ALOGE("mixer mode set error,ret=%d!",ret);
            }
#endif
}
                out->pcm[SND_OUT_SOUND_CARD_HDMI] = pcm_open(card, device,
                                                    PCM_OUT | PCM_MONOTONIC, &out->config);
                if (out->pcm[SND_OUT_SOUND_CARD_HDMI] &&
                        !pcm_is_ready(out->pcm[SND_OUT_SOUND_CARD_HDMI])) {
                    ALOGE("pcm_open(PCM_CARD_HDMI) failed: %s, card number = %d",
                          pcm_get_error(out->pcm[SND_OUT_SOUND_CARD_HDMI]),card);
                    pcm_close(out->pcm[SND_OUT_SOUND_CARD_HDMI]);
                    return -ENOMEM;
                }
if (!hasExtCodec()){
                if(is_multi_pcm(out) || is_bitstream(out)){
                    adev->owner[SOUND_CARD_HDMI] = (int*)out;
                }
}
            }
        } else {
            ALOGD("The current HDMI is DVI mode");
            out->device |= AUDIO_DEVICE_OUT_SPEAKER;
        }
    }

    if (out->device & (AUDIO_DEVICE_OUT_SPEAKER |
                       AUDIO_DEVICE_OUT_WIRED_HEADSET |
                       AUDIO_DEVICE_OUT_WIRED_HEADPHONE)) {
        card = adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card;
        device = adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].device;
        ALOGI("SIMCOM: start_output_stream opening PCM for device=0x%x card=%d device_index=%d bypass_pcm=%d",
              out->device, card, device, out->bypass_pcm ? 1 : 0);
        if(card != (int)SND_OUT_SOUND_CARD_UNKNOWN) {
			if (out->device & (AUDIO_DEVICE_OUT_SPEAKER | AUDIO_DEVICE_OUT_WIRED_HEADSET |AUDIO_DEVICE_OUT_WIRED_HEADPHONE)) {	
            out->pcm[SND_OUT_SOUND_CARD_SPEAKER] = pcm_open(card, device,
                                          PCM_OUT | PCM_MONOTONIC, &out->config);
            if (out->pcm[SND_OUT_SOUND_CARD_SPEAKER] && !pcm_is_ready(out->pcm[SND_OUT_SOUND_CARD_SPEAKER])) {
                ALOGE("pcm_open(PCM_CARD) failed: %s,card number = %d",
                      pcm_get_error(out->pcm[SND_OUT_SOUND_CARD_SPEAKER]),card);
                pcm_close(out->pcm[SND_OUT_SOUND_CARD_SPEAKER]);
                return -ENOMEM;
            }
            if (out->pcm[SND_OUT_SOUND_CARD_SPEAKER]) {
                ALOGI("SIMCOM: PCM for speaker/headphone opened successfully (pcm=%p card=%d device=%d)",
                      out->pcm[SND_OUT_SOUND_CARD_SPEAKER], card, device);
            }
        }
			else{
				card = adev->dev_out[SND_OUT_SOUND_CARD_HDMI].card;
				out->pcm[SND_OUT_SOUND_CARD_HDMI] = pcm_open(card, device,
												PCM_OUT | PCM_MONOTONIC, &out->config);
				if (out->pcm[SND_OUT_SOUND_CARD_HDMI] &&
						!pcm_is_ready(out->pcm[SND_OUT_SOUND_CARD_HDMI])) {
					ALOGE("pcm_open(PCM_CARD_HDMI) failed: %s, card number = %d",
						  pcm_get_error(out->pcm[SND_OUT_SOUND_CARD_HDMI]),card);
					pcm_close(out->pcm[SND_OUT_SOUND_CARD_HDMI]);
					return -ENOMEM;
				}
			}
        }

    }

    // SIMCOM: For bypass_pcm streams, return early after opening PCM devices
    if (out->bypass_pcm) {
        ALOGI("SIMCOM: voice call stream - AudioFlinger patch drives PCM (pcm devices opened for playback)");
        return 0;
    }

    if (out->device & AUDIO_DEVICE_OUT_SPDIF) {
        if (adev->owner[SOUND_CARD_SPDIF] == NULL){
            card = adev->dev_out[SND_OUT_SOUND_CARD_SPDIF].card;
            device = adev->dev_out[SND_OUT_SOUND_CARD_SPDIF].device;
            if(card != (int)SND_OUT_SOUND_CARD_UNKNOWN) {
                out->pcm[SND_OUT_SOUND_CARD_SPDIF] = pcm_open(card, device,
                                                    PCM_OUT | PCM_MONOTONIC, &out->config);

                if (out->pcm[SND_OUT_SOUND_CARD_SPDIF] &&
                        !pcm_is_ready(out->pcm[SND_OUT_SOUND_CARD_SPDIF])) {
                    ALOGE("pcm_open(PCM_CARD_SPDIF) failed: %s,card number = %d",
                          pcm_get_error(out->pcm[SND_OUT_SOUND_CARD_SPDIF]),card);
                    pcm_close(out->pcm[SND_OUT_SOUND_CARD_SPDIF]);
                    return -ENOMEM;
                }

if (!hasExtCodec()){
                if(is_multi_pcm(out) || is_bitstream(out)){
                    adev->owner[SOUND_CARD_SPDIF] = (int*)out;
                }
}
            }
        }
    }
		if (out->device & AUDIO_DEVICE_OUT_ALL_SCO) {
	#ifdef BT_AP_SCO // HARD CODE FIXME
			card = adev->dev_out[SND_OUT_SOUND_CARD_BT].card;
			device = adev->dev_out[SND_OUT_SOUND_CARD_BT].device;
			ALOGD("pcm_open bt card number = %d",card);
			if(card != (int)SND_OUT_SOUND_CARD_UNKNOWN) {
				out->pcm[SND_OUT_SOUND_CARD_BT] = pcm_open(card, 0,
											PCM_OUT | PCM_MONOTONIC, &pcm_config_ap_sco);
				ret = create_resampler(out->config.rate,
									   pcm_config_ap_sco.rate,
									   2,
									   RESAMPLER_QUALITY_DEFAULT,
									   NULL,
									   &out->resampler);
				if (ret != 0) {
					ret = -EINVAL;
				}
			}
	#endif
		}
	

    adev->out_device |= out->device;
    ALOGD("%s:%d, out = %p",__FUNCTION__,__LINE__,out);
    return 0;
}

/**
 * @brief get_next_buffer
 *
 * @param buffer_provider
 * @param buffer
 *
 * @returns
 */
static int get_next_buffer(struct resampler_buffer_provider *buffer_provider,
                           struct resampler_buffer* buffer)
{
    struct stream_in *in;
    size_t i,size;

    if (buffer_provider == NULL || buffer == NULL)
        return -EINVAL;

    in = (struct stream_in *)((char *)buffer_provider -
                              offsetof(struct stream_in, buf_provider));

    if (in->pcm == NULL) {
        buffer->raw = NULL;
        buffer->frame_count = 0;
        in->read_status = -ENODEV;
        return -ENODEV;
    }

    if (in->frames_in == 0) {
        size = pcm_frames_to_bytes(in->pcm,pcm_get_buffer_size(in->pcm));
        size_t period_bytes = pcm_frames_to_bytes(in->pcm, in->config->period_size);
        if (in->is_simcom_voice) {
            in->read_status = simcom_rx_bus_acquire(in, period_bytes);
        } else {
            in->read_status = pcm_read(in->pcm, (void*)in->buffer, period_bytes);
        }
        if (in->read_status != 0) {
            ALOGE("get_next_buffer() pcm_read error %d", in->read_status);
            buffer->raw = NULL;
            buffer->frame_count = 0;
            return in->read_status;
        }

        //fwrite(in->buffer,pcm_frames_to_bytes(in->pcm,pcm_get_buffer_size(in->pcm)),1,in_debug);
        in->frames_in = in->config->period_size;

        /* Do stereo to mono conversion in place by discarding right channel */
        if ((in->channel_mask == AUDIO_CHANNEL_IN_MONO)
                &&(in->config->channels == 2)) {
            //ALOGE("channel_mask = AUDIO_CHANNEL_IN_MONO");
            for (i = 0; i < in->frames_in; i++)
                in->buffer[i] = in->buffer[i * 2];
        }
    }

    //ALOGV("pcm_frames_to_bytes(in->pcm,pcm_get_buffer_size(in->pcm)):%d",size);
    buffer->frame_count = (buffer->frame_count > in->frames_in) ?
                          in->frames_in : buffer->frame_count;
    buffer->i16 = in->buffer +
                  (in->config->period_size - in->frames_in) *
                  audio_channel_count_from_in_mask(in->channel_mask);

    return in->read_status;

}

/**
 * @brief release_buffer
 *
 * @param buffer_provider
 * @param buffer
 */
static void release_buffer(struct resampler_buffer_provider *buffer_provider,
                           struct resampler_buffer* buffer)
{
    struct stream_in *in;

    if (buffer_provider == NULL || buffer == NULL)
        return;

    in = (struct stream_in *)((char *)buffer_provider -
                              offsetof(struct stream_in, buf_provider));

    in->frames_in -= buffer->frame_count;
}

#define STR_32KHZ "32KHZ"
#define STR_44_1KHZ "44.1KHZ"
#define STR_48KHZ "48KHZ"
/**
 * @brief get_hdmiin_audio_rate
 * @param
 * @return hdmiin audio rate
 */
static int get_hdmiin_audio_rate(struct audio_device *adev)
{
    int rate = 44100;
    char value[PROPERTY_VALUE_MAX] = "";
    property_get("vendor.hdmiin.audiorate", value, STR_44_1KHZ);

    if ( 0 == strncmp(value, STR_32KHZ, strlen(STR_32KHZ)) ){
        rate = 32000;
    } else if ( 0 == strncmp(value, STR_44_1KHZ, strlen(STR_44_1KHZ)) ){
        rate = 44100;
    } else if ( 0 == strncmp(value, STR_48KHZ, strlen(STR_48KHZ)) ){
        rate = 48000;
    } else {
        rate = atoi(value);
        if (rate <= 0)
            rate = 44100;
    }

    // if hdmiin connect to codec, use 44100 sample rate
    if (adev->dev_out[SND_IN_SOUND_CARD_HDMI].card
            == adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card)
        rate = 44100;

    return rate;
}

int create_resampler_helper(struct stream_in *in, uint32_t in_rate)
{
    int ret = 0;
    if (in->resampler) {
        release_resampler(in->resampler);
        in->resampler = NULL;
    }

    in->buf_provider.get_next_buffer = get_next_buffer;
    in->buf_provider.release_buffer = release_buffer;
    ALOGD("create resampler, channel %d, rate %d => %d",
                    audio_channel_count_from_in_mask(in->channel_mask),
                    in_rate, in->requested_rate);
    ret = create_resampler(in_rate,
                    in->requested_rate,
                    audio_channel_count_from_in_mask(in->channel_mask),
                    RESAMPLER_QUALITY_DEFAULT,
                    &in->buf_provider,
                    &in->resampler);
    if (ret != 0) {
        ret = -EINVAL;
    }

    return ret;
}

/**
 * @brief start_input_stream
 * must be called with input stream and hw device mutexes locked
 *
 * @param in
 *
 * @returns
 */
static int start_input_stream(struct stream_in *in)
{
    struct audio_device *adev = in->dev;
    int  ret = 0;
    int card = 0;
    int device = 0;

    in_dump(in, 0);
    
    if (in->bypass_pcm) {
        ALOGI("SIMCOM: voice call input stream - AudioFlinger patch drives PCM (pcm=NULL enforced)");
        in->pcm = NULL;
        return 0;
    }

    if (in->is_simcom_voice) {
        // Reset audio detection state for new call
        in->simcom_audio_detected = false;
        in->simcom_silence_count = 0;
        in->simcom_audio_confirm_count = 0;
        ALOGD("SIMCOM: reset audio detection state for new call");
        
        if (in->simcom_attached) {
            ALOGD("SIMCOM: telephony RX stream already attached");
            goto simcom_post_open;
        }
        ALOGI("SIMCOM: start_input_stream Telephony RX (device=0x%x)", in->device);
        in->config = &simcom_pcm_config_rx;
        ret = simcom_acquire_rx_pcm(adev, true);
        if (ret == 0) {
            in->pcm = IN_SIMCOM_PCM(in);
            in->simcom_attached = true;
            ALOGI("SIMCOM: telephony RX PCM attached (pcm=%p users=%d)",
                  IN_SIMCOM_PCM(in), adev->simcom_rx_users);
        } else if (ret == -EAGAIN) {
            ALOGW("SIMCOM: telephony RX PCM not ready yet, deferring attachment");
            in->pcm = NULL;
            in->simcom_attached = false;
        } else {
            ALOGE("SIMCOM: telephony RX PCM acquire failed ret=%d", ret);
            return ret;
        }
        if (in->resampler) {
            release_resampler(in->resampler);
            in->resampler = NULL;
        }
        goto simcom_post_open;
    }

    read_in_sound_card(in);
    route_pcm_card_open(adev->dev_in[SND_IN_SOUND_CARD_MIC].card,
                        getRouteFromDevice(in->device | AUDIO_DEVICE_BIT_IN));
#ifdef RK3399_LAPTOP //HARD CODE FIXME
    if ((in->device & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET)
           /*&&  (adev->mode == AUDIO_MODE_IN_COMMUNICATION)*/) {
        in->config = &pcm_config_in_bt;
        card = adev->dev_in[SND_IN_SOUND_CARD_BT].card;
        device =  adev->dev_in[SND_IN_SOUND_CARD_BT].device;
        if(card != SND_IN_SOUND_CARD_UNKNOWN){
            in->pcm = pcm_open(card, device, PCM_IN, in->config);
            if (in->resampler) {
                release_resampler(in->resampler);

                in->buf_provider.get_next_buffer = get_next_buffer;
                in->buf_provider.release_buffer = release_buffer;

                ret = create_resampler(in->config->rate,
                                       in->requested_rate,
                                       audio_channel_count_from_in_mask(in->channel_mask),
                                       RESAMPLER_QUALITY_DEFAULT,
                                       &in->buf_provider,
                                       &in->resampler);
                if (ret != 0) {
                    ret = -EINVAL;
                }
            }
        } else {
            ALOGE("%s: %d,the card number of bt is = %d",__FUNCTION__,__LINE__,card);
            return -EINVAL;
        }
    } else {
        in->config = &pcm_config_in;
        card = adev->dev_in[SND_IN_SOUND_CARD_MIC].card;
        device =  adev->dev_in[SND_IN_SOUND_CARD_MIC].device;
        if (card != SND_IN_SOUND_CARD_UNKNOWN) {
            in->pcm = pcm_open(card, device, PCM_IN, in->config);

            if (in->resampler) {
                release_resampler(in->resampler);

                in->buf_provider.get_next_buffer = get_next_buffer;
                in->buf_provider.release_buffer = release_buffer;

                ret = create_resampler(in->config->rate,
                                       in->requested_rate,
                                       audio_channel_count_from_in_mask(in->channel_mask),
                                       RESAMPLER_QUALITY_DEFAULT,
                                       &in->buf_provider,
                                       &in->resampler);
                if (ret != 0) {
                    ret = -EINVAL;
                }
            }
        } else {
            ALOGE("%s: %d,the card number of mic is %d",__FUNCTION__,__LINE__,card);
            return -EINVAL;
        }
    }
#else
    card = (int)adev->dev_in[SND_IN_SOUND_CARD_HDMI].card;
    if (in->device & AUDIO_DEVICE_IN_HDMI && (card != (int)SND_OUT_SOUND_CARD_UNKNOWN)) {
        in->config->rate = get_hdmiin_audio_rate(adev);
        in->pcm = pcm_open(card, PCM_DEVICE, PCM_IN, in->config);
        ALOGD("open HDMIIN %d", card);
        if (in->resampler) {
            release_resampler(in->resampler);
            in->resampler = NULL;
        }

        // if hdmiin connect to codec, don't resample
        if (in->config->rate != in->requested_rate) {
            ret = create_resampler_helper(in, in->config->rate);
        }
    } else if (in->device & AUDIO_DEVICE_IN_BUILTIN_MIC ||
               in->device & AUDIO_DEVICE_IN_WIRED_HEADSET) {
        card = adev->dev_in[SND_IN_SOUND_CARD_MIC].card;
        device =  adev->dev_in[SND_IN_SOUND_CARD_MIC].device;
        in->pcm = pcm_open(card, device, PCM_IN, in->config);
    } else {
        card = adev->dev_in[SND_IN_SOUND_CARD_BT].card;
        device = adev->dev_in[SND_IN_SOUND_CARD_BT].device;
        in->pcm = pcm_open(card, device, PCM_IN, in->config);
    }
#endif
    if (in->pcm && !pcm_is_ready(in->pcm)) {
        ALOGE("pcm_open() failed: %s", pcm_get_error(in->pcm));
        pcm_close(in->pcm);
        return -ENOMEM;
    }

simcom_post_open:
    /* if no supported sample rate is available, use the resampler */
    if (in->resampler)
        in->resampler->reset(in->resampler);

    in->frames_in = 0;
    adev->input_source = in->input_source;
    adev->in_device = in->device;
    adev->in_channel_mask = in->channel_mask;


    /* initialize volume ramp */
    in->ramp_frames = (CAPTURE_START_RAMP_MS * in->requested_rate) / 1000;
    in->ramp_step = (uint16_t)(USHRT_MAX / in->ramp_frames);
    in->ramp_vol = 0;;


    return 0;
}

/**
 * @brief get_input_buffer_size
 *
 * @param sample_rate
 * @param format
 * @param channel_count
 * @param is_low_latency
 *
 * @returns
 */
static size_t get_input_buffer_size(unsigned int sample_rate,
                                    audio_format_t format,
                                    unsigned int channel_count,
                                    bool is_low_latency)
{
    const struct pcm_config *config = is_low_latency ?
                                              &pcm_config_in_low_latency : &pcm_config_in;
    size_t size;

    /*
     * take resampling into account and return the closest majoring
     * multiple of 16 frames, as audioflinger expects audio buffers to
     * be a multiple of 16 frames
     */
    size = (config->period_size * sample_rate) / config->rate;
    size = ((size + 15) / 16) * 16;

    return size * channel_count * audio_bytes_per_sample(format);
}


/**
 * @brief read_frames
 * read_frames() reads frames from kernel driver, down samples to capture rate
 * if necessary and output the number of frames requested to the buffer specified
 *
 * @param in
 * @param buffer
 * @param frames
 *
 * @returns
 */
static ssize_t read_frames(struct stream_in *in, void *buffer, ssize_t frames)
{
    ssize_t frames_wr = 0;
    size_t frame_size = audio_stream_in_frame_size(&in->stream);

    while (frames_wr < frames) {
        size_t frames_rd = frames - frames_wr;
        if (in->resampler != NULL) {
            in->resampler->resample_from_provider(in->resampler,
                                                  (int16_t *)((char *)buffer +
                                                          frames_wr * frame_size),
                                                  &frames_rd);
        } else {
            struct resampler_buffer buf = {
                { raw : NULL, },
frame_count :
                frames_rd,
            };
            get_next_buffer(&in->buf_provider, &buf);
            if (buf.raw != NULL) {
                memcpy((char *)buffer +
                       frames_wr * frame_size,
                       buf.raw,
                       buf.frame_count * frame_size);
                frames_rd = buf.frame_count;
            }
            release_buffer(&in->buf_provider, &buf);
        }
        /* in->read_status is updated by getNextBuffer() also called by
         * in->resampler->resample_from_provider() */
        if (in->read_status != 0)
            return in->read_status;

        frames_wr += frames_rd;
    }
    return frames_wr;
}

/**
 * @brief out_get_sample_rate
 *
 * @param stream
 *
 * @returns
 */
static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    char value[PROPERTY_VALUE_MAX];
    property_get("vendor.vts_test", value, NULL);

    if (strcmp(value, "true") == 0) {
        return out->aud_config.sample_rate;
    } else {
        return out->config.rate;
    }

}

/**
 * @brief out_set_sample_rate
 *
 * @param stream
 * @param rate
 *
 * @returns
 */
static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    return -ENOSYS;
}

/**
 * @brief out_get_buffer_size
 *
 * @param stream
 *
 * @returns
 */
static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;

    return out->config.period_size *
           audio_stream_out_frame_size((const struct audio_stream_out *)stream);
}

/**
 * @brief out_get_channels
 *
 * @param stream
 *
 * @returns
 */
static audio_channel_mask_t out_get_channels(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    char value[PROPERTY_VALUE_MAX];
    property_get("vendor.vts_test", value, NULL);

    if (strcmp(value, "true") == 0){
        return out->aud_config.channel_mask;
    } else {
        return out->channel_mask;
    }

}

/**
 * @brief out_get_format
 *
 * @param stream
 *
 * @returns
 */
static audio_format_t out_get_format(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    char value[PROPERTY_VALUE_MAX];
    property_get("vendor.vts_test", value, NULL);

    if (strcmp(value, "true") == 0){
        return out->aud_config.format;
    } else {
        return AUDIO_FORMAT_PCM_16_BIT;
    }

}

/**
 * @brief out_set_format
 *
 * @param stream
 * @param format
 *
 * @returns
 */
static int out_set_format(struct audio_stream *stream, audio_format_t format)
{
    return -ENOSYS;
}

/**
 * @brief output_devices
 * Return the set of output devices associated with active streams
 * other than out.  Assumes out is non-NULL and out->dev is locked.
 *
 * @param out
 *
 * @returns
 */
static audio_devices_t output_devices(struct stream_out *out)
{
    struct audio_device *dev = out->dev;
    enum output_type type;
    audio_devices_t devices = AUDIO_DEVICE_NONE;

    for (type = 0; type < OUTPUT_TOTAL; ++type) {
        struct stream_out *other = dev->outputs[type];
        if (other && (other != out) && !other->standby) {
            // TODO no longer accurate
            /* safe to access other stream without a mutex,
             * because we hold the dev lock,
             * which prevents the other stream from being closed
             */
            devices |= other->device;
        }
    }

    return devices;
}

/**
 * @brief do_out_standby
 * must be called with hw device outputs list, all out streams, and hw device mutex locked
 *
 * @param out
 */
static void do_out_standby(struct stream_out *out)
{
    struct audio_device *adev = out->dev;
    int i;
    ALOGD("%s,out = %p,device = 0x%x",__FUNCTION__,out,out->device);
    if (!out->standby) {
        if (out->is_simcom_voice && out->simcom_attached) {
            simcom_release_tx_pcm(adev);
            out->simcom_attached = false;
        }
        for (i = 0; i < SND_OUT_SOUND_CARD_MAX; i++) {
            if (out->pcm[i]) {
                pcm_close(out->pcm[i]);
                out->pcm[i] = NULL;
            }
        }
        out->is_simcom_voice = false;
        out->bypass_pcm = false;
        out->standby = true;
        out->nframes = 0;
        if (out == adev->outputs[OUTPUT_HDMI_MULTI]) {
            /* force standby on low latency output stream so that it can reuse HDMI driver if
             * necessary when restarted */
            force_non_hdmi_out_standby(adev);
        }
if (!hasExtCodec()){
#ifdef USE_DRM
        mixer_mode_set(out);
#endif
}
        /* re-calculate the set of active devices from other streams */
        adev->out_device = output_devices(out);

#ifdef AUDIO_3A
        if (adev->voice_api != NULL) {
            adev->voice_api->flush();
        }
#endif
        // SIMCOM: Close mixer only if no active output devices remain
        // This prevents interrupting playback when other streams are active
        if (!adev->out_device) {
            ALOGI("SIMCOM: do_out_standby: no active output devices, closing mixer");
        route_pcm_close(PLAYBACK_OFF_ROUTE);
        ALOGD("close device");
        } else {
            ALOGI("SIMCOM: do_out_standby: active output devices remain (0x%x), keeping mixer open",
                  adev->out_device);
        }

        /* Skip resetting the mixer if no output device is active */
        // SIMCOM: For normal playback (not voice call), activate route using route_pcm_card_open with correct sound card
        if (adev->out_device && !(adev->mode == AUDIO_MODE_IN_CALL && adev->voice_call_active)) {
            uint32_t route = getRouteFromDevice(adev->out_device);
            ALOGI("SIMCOM: do_out_standby activating route=%u for device=0x%x (card=%d)", 
                  route, adev->out_device, adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card);
            route_pcm_card_open(adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card, route);
            ALOGD("change device");
        } else if (adev->mode == AUDIO_MODE_IN_CALL && adev->voice_call_active) {
            // SIMCOM: For voice call mode, use out->device instead of adev->out_device
            uint32_t device_for_route = out->device;
            if (device_for_route) {
                uint32_t route;
                // SIMCOM: Use voice call route if in call mode
                if (device_for_route & AUDIO_DEVICE_OUT_SPEAKER) {
                    route = SPEAKER_INCALL_ROUTE;
                } else if (device_for_route & AUDIO_DEVICE_OUT_WIRED_HEADPHONE) {
                    route = HEADPHONE_INCALL_ROUTE;
                } else if (device_for_route & AUDIO_DEVICE_OUT_WIRED_HEADSET) {
                    route = HEADSET_INCALL_ROUTE;
                } else {
                    route = getRouteFromDevice(device_for_route);
                }
                ALOGI("SIMCOM: do_out_standby using voice route=%u for device=0x%x (out->device=0x%x adev->out_device=0x%x)",
                      route, device_for_route, out->device, adev->out_device);
                route_pcm_card_open(adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card, route);
            }
            ALOGI("SIMCOM: do_out_standby route activated for voice call");
        }
if (!hasExtCodec()){
        if(adev->owner[SOUND_CARD_HDMI] == (int*)out){
            adev->owner[SOUND_CARD_HDMI] = NULL;
        }

        if(adev->owner[SOUND_CARD_SPDIF] == (int*)out){
            adev->owner[SOUND_CARD_SPDIF] = NULL;
        }
}
        simcom_release_tx_resampler(out);
    }
}

/**
 * @brief lock_all_outputs
 * lock outputs list, all output streams, and device
 *
 * @param adev
 */
static void lock_all_outputs(struct audio_device *adev)
{
    enum output_type type;
    pthread_mutex_lock(&adev->lock_outputs);
    for (type = 0; type < OUTPUT_TOTAL; ++type) {
        struct stream_out *out = adev->outputs[type];
        if (out)
            pthread_mutex_lock(&out->lock);
    }
    pthread_mutex_lock(&adev->lock);
}

/**
 * @brief unlock_all_outputs
 * unlock device, all output streams (except specified stream), and outputs list
 *
 * @param adev
 * @param except
 */
static void unlock_all_outputs(struct audio_device *adev, struct stream_out *except)
{
    /* unlock order is irrelevant, but for cleanliness we unlock in reverse order */
    pthread_mutex_unlock(&adev->lock);
    enum output_type type = OUTPUT_TOTAL;
    do {
        struct stream_out *out = adev->outputs[--type];
        if (out && out != except)
            pthread_mutex_unlock(&out->lock);
    } while (type != (enum output_type) 0);
    pthread_mutex_unlock(&adev->lock_outputs);
}

/**
 * @brief out_standby
 *
 * @param stream
 *
 * @returns
 */
static int out_standby(struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;

    lock_all_outputs(adev);

    do_out_standby(out);

    unlock_all_outputs(adev, NULL);

    return 0;
}

/**
 * @brief out_dump
 *
 * @param stream
 * @param fd
 *
 * @returns
 */
int out_dump(const struct audio_stream *stream, int fd)
{
    struct stream_out *out = (struct stream_out *)stream;

    ALOGD("out->Device     : 0x%x", out->device);
    ALOGD("out->SampleRate : %d", out->config.rate);
    ALOGD("out->Channels   : %d", out->config.channels);
    ALOGD("out->Formate    : %d", out->config.format);
    ALOGD("out->PreiodSize : %d", out->config.period_size);
    return 0;
}
/**
 * @brief out_set_parameters
 *
 * @param stream
 * @param kvpairs
 *
 * @returns
 */
static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;
    struct str_parms *parms;
    char value[32];
    int ret;
    int status = 0;
    unsigned int val;

    ALOGD("%s: %s (mode=%d, call_active=%d)",
          __func__, kvpairs, adev->mode, adev->voice_call_active);

    parms = str_parms_create_str(kvpairs);

    //set channel_mask
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_CHANNELS,
                            value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        out->aud_config.channel_mask = val;
    }
    // set sample rate
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_SAMPLING_RATE,
                            value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        out->aud_config.sample_rate = val;
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));
    lock_all_outputs(adev);
    if (ret >= 0) {
        val = atoi(value);
        /* Don't switch HDMI audio in box products */
        if ((val != 0) && ((out->device & val) != val) ||
            (val != 0) && !(out->device & AUDIO_DEVICE_OUT_HDMI)) {
            /* Force standby if moving to/from SPDIF or if the output
             * device changes when in SPDIF mode */
            if (((val & AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET) ^
                    (adev->out_device & AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET)) ||
                    (adev->out_device & AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET)) {
                do_out_standby(out);
            }

            /* force output standby to start or stop SCO pcm stream if needed */
            if ((val & AUDIO_DEVICE_OUT_ALL_SCO) ^
                    (out->device & AUDIO_DEVICE_OUT_ALL_SCO)) {
                do_out_standby(out);
            }

            if (!out->standby && (out == adev->outputs[OUTPUT_HDMI_MULTI] ||
                                  !adev->outputs[OUTPUT_HDMI_MULTI] ||
                                  adev->outputs[OUTPUT_HDMI_MULTI]->standby)) {
                adev->out_device = output_devices(out) | val;
#ifndef RK3228
                do_out_standby(out);
#endif
            }
            out->device = val;
        }
        
        // SIMCOM: Don't update out_device during patch creation to avoid blocking
        // Route will be activated in out_write() when first data is written
    }
    unlock_all_outputs(adev, NULL);

    if (adev->mode == AUDIO_MODE_IN_CALL && adev->voice_call_active) {
        pthread_mutex_lock(&adev->lock);
        simcom_apply_voice_routes_l(adev);
        pthread_mutex_unlock(&adev->lock);
    }

    str_parms_destroy(parms);

    ALOGV("%s: exit: status(%d)", __func__, status);
    return status;

}

/*
 * function: get support formats
 * Query supported formats. The response is a '|' separated list of strings from audio_format_t enum
 *  e.g: "sup_formats=AUDIO_FORMAT_PCM_16_BIT"
 */

static int stream_get_parameter_formats(const struct audio_stream *stream,
                                    struct str_parms *query,
                                    struct str_parms *reply)
{
    struct stream_out *out = (struct stream_out *)stream;
    int avail = 1024;
    char value[avail];
    if (str_parms_has_key(query, AUDIO_PARAMETER_STREAM_SUP_FORMATS)) {
        memset(value,0,avail);
        // set support pcm 16 bit default
        strcat(value, "AUDIO_FORMAT_PCM_16_BIT");
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_FORMATS, value);
        return 0;
    }

    return -1;
}


/*
 * function: get support channels
 * Query supported channel masks. The response is a '|' separated list of strings from
 * audio_channel_mask_t enum
 * e.g: "sup_channels=AUDIO_CHANNEL_OUT_STEREO|AUDIO_CHANNEL_OUT_MONO"
 */

static int stream_get_parameter_channels(struct str_parms *query,
                                    struct str_parms *reply,
                                    audio_channel_mask_t *supported_channel_masks)
{
    char value[1024];
    size_t i, j;
    bool first = true;

    if(str_parms_has_key(query, AUDIO_PARAMETER_STREAM_SUP_CHANNELS)){
        value[0] = '\0';
        i = 0;
        /* the last entry in supported_channel_masks[] is always 0 */
        while (supported_channel_masks[i] != 0) {
            for (j = 0; j < ARRAY_SIZE(channels_name_to_enum_table); j++) {
                if (channels_name_to_enum_table[j].value == supported_channel_masks[i]) {
                    if (!first) {
                        strcat(value, "|");
                    }

                    strcat(value, channels_name_to_enum_table[j].name);
                    first = false;
                    break;
                }
            }
            i++;
        }
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_CHANNELS, value);
        return 0;
    }

    return -1;
}

/*
 * function: get support sample_rates
 * Query supported sampling rates. The response is a '|' separated list of integer values
 * e.g: ""sup_sampling_rates=44100|48000"
 */

static int stream_get_parameter_rates(struct str_parms *query,
                                struct str_parms *reply,
                                uint32_t *supported_sample_rates)
{
    char value[256];
    int ret = -1;

    if (str_parms_has_key(query, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES)) {
        value[0] = '\0';
        int cursor = 0;
        int i = 0;
        while(supported_sample_rates[i]){
            int avail = sizeof(value) - cursor;
            ret = snprintf(value + cursor, avail, "%s%d",
                           cursor > 0 ? "|" : "",
                           supported_sample_rates[i]);

            if (ret < 0 || ret > avail){
                value[cursor] = '\0';
                break;
            }

            cursor += ret;
            ++i;
        }
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, value);
        return 0;
    }
    return -1;
}

/**
 * @brief out_get_parameters
 *
 * @param stream
 * @param keys
 *
 * @returns
 */
static char * out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    ALOGD("%s: keys = %s", __func__, keys);

    struct stream_out *out = (struct stream_out *)stream;
    struct str_parms *query = str_parms_create_str(keys);
    char *str = NULL;
    struct str_parms *reply = str_parms_create();

    if (stream_get_parameter_formats(stream,query,reply) == 0) {
        str = str_parms_to_str(reply);
    } else if (stream_get_parameter_channels(query, reply, &out->supported_channel_masks[0]) == 0) {
        str = str_parms_to_str(reply);
    } else if (stream_get_parameter_rates(query, reply, &out->supported_sample_rates[0]) == 0) {
        str = str_parms_to_str(reply);
    } else {
        ALOGD("%s,str_parms_get_str failed !",__func__);
        str = strdup("");
    }

    str_parms_destroy(query);
    str_parms_destroy(reply);

    ALOGV("%s,exit -- str = %s",__func__,str);
    return str;
}

/**
 * @brief out_get_latency
 *
 * @param stream
 *
 * @returns
 */
static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
    struct stream_out *out = (struct stream_out *)stream;

    return (out->config.period_size * out->config.period_count * 1000) /
           out->config.rate;
}

/**
 * @brief out_set_volume
 *
 * @param stream
 * @param left
 * @param right
 *
 * @returns
 */
static int out_set_volume(struct audio_stream_out *stream, float left,
                          float right)
{
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;

    /* The mutex lock is not needed, because the client
     * is not allowed to close the stream concurrently with this API
     *  pthread_mutex_lock(&adev->lock_outputs);
     */
    bool is_HDMI = out == adev->outputs[OUTPUT_HDMI_MULTI];
    /*  pthread_mutex_unlock(&adev->lock_outputs); */
    if (is_HDMI) {
        /* only take left channel into account: the API is for stereo anyway */
        out->muted = (left == 0.0f);
        return 0;
    }
    return -ENOSYS;
}
/**
 * @brief dump_out_data
 *
 * @param buffer bytes
 */
static void dump_out_data(const void* buffer,size_t bytes)
{
    char value[PROPERTY_VALUE_MAX];
    property_get("vendor.audio.record", value, "0");
    int size = atoi(value);
    if (size <= 0)
        return ;

    ALOGD("dump pcm file.");
    static FILE* fd = NULL;
    static int offset = 0;
    if (fd == NULL) {
        fd=fopen("/data/misc/audioserver/debug.pcm","wb+");
        if(fd == NULL) {
            ALOGD("DEBUG open /data/debug.pcm ,errno = %s",strerror(errno));
            offset = 0;
        }
    }

    if (fd != NULL) {
        fwrite(buffer,bytes,1,fd);
        offset += bytes;
        fflush(fd);
        if(offset >= size*1024*1024) {
            fclose(fd);
            fd = NULL;
            offset = 0;
            property_set("vendor.audio.record", "0");
            ALOGD("TEST playback pcmfile end");
        }
    }
}

static void dump_in_data(const void* buffer, size_t bytes)
{
    static int offset = 0;
    static FILE* fd = NULL;
    char value[PROPERTY_VALUE_MAX];
    property_get("vendor.audio.record.in", value, "0");
    int size = atoi(value);
    if (size > 0) {
        if(fd == NULL) {
            fd=fopen("/data/misc/audioserver/debug_in.pcm","wb+");
            if(fd == NULL) {
                ALOGD("DEBUG open /data/misc/audioserver/debug_in.pcm ,errno = %s",strerror(errno));
            } else {
                ALOGD("dump pcm to file /data/misc/audioserver/debug_in.pcm");
            }
            offset = 0;
        }
    }

    if (fd != NULL) {
        ALOGD("dump in pcm %zu bytes", bytes);
        fwrite(buffer,bytes,1,fd);
        offset += bytes;
        fflush(fd);
        if (offset >= size*1024*1024) {
            fclose(fd);
            fd = NULL;
            offset = 0;
            property_set("vendor.audio.record.in", "0");
            ALOGD("TEST record pcmfile end");
        }
    }
}

static void simcom_maybe_dump_buffer(bool is_rx, const void *buffer, size_t bytes)
{
    const char *prop = is_rx ? SIMCOM_DUMP_RX_PROP : SIMCOM_DUMP_TX_PROP;
    const char *path = is_rx ? SIMCOM_RX_DUMP_PATH : SIMCOM_TX_DUMP_PATH;

    if (!property_get_bool(prop, false)) {
        return;
    }

    int fd = open(path, O_CREAT | O_WRONLY | O_APPEND | O_CLOEXEC, 0666);
    if (fd < 0) {
        ALOGE("SIMCOM: failed to open dump file %s: %s", path, strerror(errno));
        return;
    }

    ssize_t ret = write(fd, buffer, bytes);
    if (ret < 0) {
        ALOGE("SIMCOM: failed to write dump %s: %s", path, strerror(errno));
    }
    close(fd);
}

static void simcom_log_pcm_snapshot(const char *tag, const void *buffer, size_t bytes)
{
    if (!buffer || bytes < sizeof(int16_t)) {
        ALOGD("SIMCOM: %s PCM bytes=%zu (insufficient data)", tag, bytes);
        return;
    }

    const int16_t *samples = (const int16_t *)buffer;
    const size_t sample_count = bytes / sizeof(int16_t);
    const size_t inspect = sample_count < 64 ? sample_count : 64;
    bool all_zero = true;
    int16_t peak = 0;

    for (size_t i = 0; i < inspect; ++i) {
        const int16_t val = samples[i];
        if (val != 0) {
            all_zero = false;
        }
        const int16_t abs_val = val < 0 ? -val : val;
        if (abs_val > peak) {
            peak = abs_val;
        }
    }

    const int16_t first = samples[0];
    const int16_t mid = samples[sample_count / 2];
    const int16_t last = samples[sample_count - 1];

    ALOGD("SIMCOM: %s PCM bytes=%zu samples=%zu peak=%d first/mid/last=%d/%d/%d zero=%d",
          tag, bytes, sample_count, peak, first, mid, last, all_zero);

    bool is_rx = (strstr(tag, "RX") != NULL);
    simcom_maybe_dump_buffer(is_rx, buffer, bytes);
}

// Threshold for detecting real audio vs silence (16-bit PCM, range -32768 to 32767)
// Peak amplitude must exceed this to be considered real audio
// Temporarily disabled to test if detection causes clicks
#define SIMCOM_AUDIO_DETECT_THRESHOLD 50  // Lowered threshold to detect earlier
// Number of consecutive non-silence buffers required to confirm audio detection
#define SIMCOM_AUDIO_DETECT_CONFIRM_COUNT 3
// Maximum silence buffers before resetting detection (to allow re-detection)
#define SIMCOM_SILENCE_MAX_COUNT 100

/**
 * Detect if buffer contains real audio signal (not silence/zeros)
 * Returns true if audio detected, false if silence
 */
static bool simcom_detect_real_audio(const void *buffer, size_t bytes)
{
    if (!buffer || bytes < sizeof(int16_t)) {
        return false;
    }

    const int16_t *samples = (const int16_t *)buffer;
    const size_t sample_count = bytes / sizeof(int16_t);
    int16_t peak = 0;

    // Check entire buffer for peak amplitude
    for (size_t i = 0; i < sample_count; ++i) {
        const int16_t abs_val = samples[i] < 0 ? -samples[i] : samples[i];
        if (abs_val > peak) {
            peak = abs_val;
        }
    }

    // Consider real audio if peak exceeds threshold
    return (peak >= SIMCOM_AUDIO_DETECT_THRESHOLD);
}

/**
 * Update audio detection state for SIMCOM RX stream
 * Returns true if real audio is confirmed (should pass through),
 * false if still in silence period (should be suppressed)
 */
static bool simcom_update_audio_detection(struct stream_in *in, const void *buffer, size_t bytes)
{
    if (!in->is_simcom_voice) {
        return true;  // Not SIMCOM voice stream, always pass through
    }

    // If audio already detected, check if we're back to silence
    if (in->simcom_audio_detected) {
        if (simcom_detect_real_audio(buffer, bytes)) {
            in->simcom_silence_count = 0;
            return true;  // Real audio confirmed, pass through
        } else {
            in->simcom_silence_count++;
            // Allow brief silence periods but reset if too long
            if (in->simcom_silence_count > SIMCOM_SILENCE_MAX_COUNT) {
                ALOGI("SIMCOM: RX audio lost, resetting detection (silence_count=%u)",
                      in->simcom_silence_count);
                in->simcom_audio_detected = false;
                in->simcom_silence_count = 0;
                return false;
            }
            return true;  // Brief silence allowed, still pass through
        }
    }

    // Audio not yet detected, check current buffer
    if (simcom_detect_real_audio(buffer, bytes)) {
        // Increment confirmation counter
        in->simcom_audio_confirm_count++;
        
        if (in->simcom_audio_confirm_count >= SIMCOM_AUDIO_DETECT_CONFIRM_COUNT) {
            // Audio confirmed, enable pass-through
            in->simcom_audio_detected = true;
            in->simcom_silence_count = 0;
            in->simcom_audio_confirm_count = 0;
            ALOGI("SIMCOM: RX real audio detected! Enabling playback (peak detected in %u buffers)",
                  SIMCOM_AUDIO_DETECT_CONFIRM_COUNT);
            return true;
        }
        // Need more confirmations
        return false;
    } else {
        // Still silence, reset confirmation counter
        in->simcom_audio_confirm_count = 0;
        in->simcom_silence_count++;
        return false;
    }
}

static int simcom_ensure_tx_resampler_buffer(struct stream_out *out, size_t frames, size_t channels)
{
    const size_t required = frames * channels * sizeof(int16_t);
    if (required == 0) {
        return -EINVAL;
    }
    if (out->simcom_resampler_buffer_size < required) {
        int16_t *buf = realloc(out->simcom_resampler_buffer, required);
        if (buf == NULL) {
            ALOGE("SIMCOM: failed to allocate TX resampler buffer size=%zu", required);
            return -ENOMEM;
        }
        out->simcom_resampler_buffer = buf;
        out->simcom_resampler_buffer_size = required;
    }
    return 0;
}

static int simcom_ensure_tx_mono_buffer(struct stream_out *out, size_t frames)
{
    const size_t required = frames * sizeof(int16_t);
    if (required == 0) {
        return -EINVAL;
    }
    if (out->simcom_mono_buffer_size < required) {
        int16_t *buf = realloc(out->simcom_mono_buffer, required);
        if (buf == NULL) {
            ALOGE("SIMCOM: failed to allocate TX mono buffer size=%zu", required);
            return -ENOMEM;
        }
        out->simcom_mono_buffer = buf;
        out->simcom_mono_buffer_size = required;
    }
    return 0;
}

static void simcom_release_tx_resampler(struct stream_out *out)
{
    if (out == NULL) {
        return;
    }
    if (out->simcom_resampler) {
        release_resampler(out->simcom_resampler);
        out->simcom_resampler = NULL;
    }
    // SIMCOM: Clear buffers before freeing to prevent stale data
    if (out->simcom_resampler_buffer) {
        memset(out->simcom_resampler_buffer, 0, out->simcom_resampler_buffer_size);
        free(out->simcom_resampler_buffer);
        out->simcom_resampler_buffer = NULL;
        out->simcom_resampler_buffer_size = 0;
    }
    if (out->simcom_mono_buffer) {
        memset(out->simcom_mono_buffer, 0, out->simcom_mono_buffer_size);
        free(out->simcom_mono_buffer);
        out->simcom_mono_buffer = NULL;
        out->simcom_mono_buffer_size = 0;
    }
    out->simcom_resampler_in_rate = 0;
    ALOGD("SIMCOM: TX resampler and buffers released and cleared");
}

static int simcom_prepare_tx_resampler(struct stream_out *out, uint32_t in_rate, size_t channels)
{
    if (in_rate == 0 || in_rate == SIMCOM_TX_TARGET_RATE) {
        if (out->simcom_resampler) {
            release_resampler(out->simcom_resampler);
            out->simcom_resampler = NULL;
        }
        out->simcom_resampler_in_rate = 0;
        return 0;
    }
    if (out->simcom_resampler &&
        out->simcom_resampler_in_rate != in_rate) {
        release_resampler(out->simcom_resampler);
        out->simcom_resampler = NULL;
        out->simcom_resampler_in_rate = 0;
    }
    if (out->simcom_resampler == NULL) {
        int ret = create_resampler(in_rate,
                                   SIMCOM_TX_TARGET_RATE,
                                   channels,
                                   RESAMPLER_QUALITY_DEFAULT,
                                   NULL,
                                   &out->simcom_resampler);
        if (ret != 0) {
            ALOGE("SIMCOM: failed to create TX resampler %u->%u ret=%d",
                  in_rate, SIMCOM_TX_TARGET_RATE, ret);
            return ret;
        }
        out->simcom_resampler_in_rate = in_rate;
    }
    return 0;
}

/**
 * @brief reset_bitstream_buf
 *
 * @param out
 */
static void reset_bitstream_buf(struct stream_out *out)
{
    if (is_bitstream(out)) {
        if(out->config.format == PCM_FORMAT_S24_LE) {
            if (out->bitstream_buffer) {
                free (out->bitstream_buffer);
                out->bitstream_buffer = NULL;
            }
        }
    }
}

static void check_hdmi_reconnect(struct stream_out *out)
{
    if (out == NULL) {
        return ;
    }

    struct audio_device *adev = out->dev;
    lock_all_outputs(adev);
    /*
     * if snd_reopen is set to true, this means we need to reopen sound card.
     * There are a situation, we need to do this:
     *   current stream is bistream over hdmi, and hdmi is unpluged and plug later,
     *   the driver of hdmi may init the hdmi in pcm mode automatically, according the
     *   implement of driver of hdmi. If we contiune send bitstream to hdmi open in pcm mode,
     *   hdmi may make noies or mute.
     */
    if (out->snd_reopen && !out->standby)
    {
        /*
         * standby sound cards
         * the driver of hdmi will auto init with last configurations,
         * so, we don't need close and reopen sound card of hdmi here.
         * If driver of hdmi not config the hdmi with last output configurations,
         * please open this codes to close and reopen sound card of hdmi.
         */
  //      do_out_standby(out);
  //      reset_bitstream_buf(out);
    }
    unlock_all_outputs(adev,NULL);
    /*
     * audio hal recived the msg of hdmi plugin, and other part of sdk will reviced it too.
     * Other part(maybe hwc) will config hdmi after it reviced the msg.
     * Audio must wait other part(maybe hwc) codes config hdmi finish, before send bitstream datas to hdmi
     */
    if (out->snd_reopen && is_bitstream(out) && (out->device == AUDIO_DEVICE_OUT_AUX_DIGITAL)) {
#ifdef USE_DRM
        const char* PATH = "/sys/class/drm/card0-HDMI-A-1/enabled";
#else
        const char* PATH = "/sys/class/display/HDMI/enabled";
#endif
        if (access(PATH, R_OK) != 0) {
            /*
             * in most test, the time is 700~800ms between received msg of hdmi plug in
             * and hdmi init finish, so we sleep 1 sec here if no way to get the status of hdmi.
             */
            usleep(1000000);
        } else {
            /*
             * read this node to judge the status of hdmi is config finish?
             */
            char buffer[1024];
            int counter  = 200;
            FILE* file = NULL;
            while (counter >= 0 && ((file = fopen(PATH,"r")) != NULL)) {
                int size = fread(buffer,1,sizeof(buffer),file);
                if(size >= 0) {
                    if(strstr(buffer,"enabled")) {
                        fclose(file);
                        usleep(10000);
                        break;
                    }
                }
                usleep(10000);
                counter --;
                fclose(file);
            }
        }
        ALOGD("%s: out = %p",__FUNCTION__,out);
        out->snd_reopen = false;
    }
}

static void out_mute_data(struct stream_out *out,void* buffer,size_t bytes)
{
    struct audio_device *adev = out->dev;
    bool mute = false;

#ifdef MUTE_WHEN_SCREEN_OFF
    mute = adev->screenOff;
#endif
    // for some special customer
    char value[PROPERTY_VALUE_MAX];
    property_get("vendor.audio.mute", value, "false");
    if (!strcasecmp(value,"true")) {
        mute = true;
    }

    if (out->muted || mute){
        memset((void *)buffer, 0, bytes);
    }
}

static int fill_hdmi_bistream(struct stream_out *out,void* buffer,size_t insize)
{
    int size = 2*insize;
    if (out->bitstream_buffer == NULL) {
        out->bitstream_buffer = (char *)malloc(size);
        ALOGD("new bitstream buffer!");
    }
    memset(out->bitstream_buffer, 0x00, size);
    fill_hdmi_bitstream_buf((void *)buffer, (void *)out->bitstream_buffer,(void*)out->channel_buffer, (int)insize);
    return size;
}

static int bitstream_write_data(struct stream_out *out,void* buffer,size_t bytes)
{
    if ((out == NULL) || (buffer == NULL) || (bytes <= 0)) {
        ALOGD("%s: %d, input parameter is invalid",__FUNCTION__,__LINE__);
        return -1;
    }

    struct audio_device *adev = out->dev;
    int ret = 0;
    if ((out->device & AUDIO_DEVICE_OUT_AUX_DIGITAL) && (is_multi_pcm(out) || is_bitstream(out))) {
        int card = adev->dev_out[SND_OUT_SOUND_CARD_HDMI].card;
        if ((card != SND_OUT_SOUND_CARD_UNKNOWN) && (out->pcm[SND_OUT_SOUND_CARD_HDMI] != NULL)) {
            if(out->config.format == PCM_FORMAT_S16_LE){
                out_mute_data(out,buffer,bytes);
                dump_out_data(buffer, bytes);
                ret = pcm_write(out->pcm[SND_OUT_SOUND_CARD_HDMI], (void *)buffer, bytes);
            }else if(out->config.format == PCM_FORMAT_S24_LE){
                int size = fill_hdmi_bistream(out,buffer,bytes);
                out_mute_data(out,(void*)out->bitstream_buffer,size);
                dump_out_data((void*)out->bitstream_buffer, size);
                ret = pcm_write(out->pcm[SND_OUT_SOUND_CARD_HDMI], (void *)out->bitstream_buffer, size);
            }
        } else {
            ALOGD("%s: %d: HDMI sound card not open",__FUNCTION__,__LINE__);
            ret = -1;
        }
    }

    return ret;
}

/**
 * @brief out_write
 *
 * @param stream
 * @param buffer
 * @param bytes
 *
 * @returns
 */

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    int ret = 0;
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;
    size_t newbytes = bytes * 2;
    int i,card;
    /* FIXME This comment is no longer correct
     * acquiring hw device mutex systematically is useful if a low
     * priority thread is waiting on the output stream mutex - e.g.
     * executing out_set_parameters() while holding the hw device
     * mutex
     */

	if (!hasExtCodec()){
	    check_hdmi_reconnect(out);
	}

    pthread_mutex_lock(&out->lock);
    if (out->standby) {
        pthread_mutex_unlock(&out->lock);
        lock_all_outputs(adev);
        if (!out->standby) {
            unlock_all_outputs(adev, out);
            goto false_alarm;
        }
        ret = start_output_stream(out);
        if (ret < 0) {
            unlock_all_outputs(adev, NULL);
            goto final_exit;
        }
        out->standby = false;
        unlock_all_outputs(adev, out);
    }
false_alarm:

    // SIMCOM: Log out_write entry for debugging (only first call and when bypass_pcm changes)
    // Removed frequent logging to avoid system overload

    // SIMCOM: Для bypass_pcm потоков (voice call RX path) данные приходят через патч AudioFlinger
    // Нужно записать эти данные в PCM для динамика/гарнитуры для воспроизведения
    if (out->bypass_pcm) {
        // Для voice call RX path: данные из модема → PCM для динамика/гарнитуры
        if (out->device & (AUDIO_DEVICE_OUT_SPEAKER |
                           AUDIO_DEVICE_OUT_WIRED_HEADSET |
                           AUDIO_DEVICE_OUT_WIRED_HEADPHONE)) {
            // SIMCOM: Check data from patch before writing
            int16_t *patch_data = (int16_t *)buffer;
            size_t patch_samples = bytes / sizeof(int16_t);
            int16_t patch_peak = 0;
            for (size_t s = 0; s < patch_samples && s < 100; s++) {
                int16_t val = patch_data[s] < 0 ? -patch_data[s] : patch_data[s];
                if (val > patch_peak) patch_peak = val;
            }
            ALOGI("SIMCOM: bypass_pcm data from patch: bytes=%zu peak=%d device=0x%x rate=%u channels=%u",
                  bytes, patch_peak, out->device, out->config.rate, out->config.channels);
            
            if (out->pcm[SND_OUT_SOUND_CARD_SPEAKER] != NULL && 
                pcm_is_ready(out->pcm[SND_OUT_SOUND_CARD_SPEAKER])) {
                ret = pcm_write(out->pcm[SND_OUT_SOUND_CARD_SPEAKER], buffer, bytes);
                if (ret) {
                    ALOGW("SIMCOM: bypass_pcm pcm_write to speaker failed ret=%d err=%s",
                          ret, pcm_get_error(out->pcm[SND_OUT_SOUND_CARD_SPEAKER]));
                    ret = 0; // Non-fatal, continue
                } else {
                    ALOGV("SIMCOM: bypass_pcm out_write: %zu bytes written to speaker PCM", bytes);
                }
            } else {
                ALOGW("SIMCOM: bypass_pcm out_write: speaker PCM not ready (pcm=%p), dropping %zu bytes",
                      out->pcm[SND_OUT_SOUND_CARD_SPEAKER], bytes);
                ret = 0;
            }
        } else {
            ALOGD("SIMCOM: bypass_pcm out_write: %zu bytes passed through AudioFlinger patch (device=0x%x)",
                  bytes, out->device);
            ret = 0;
        }
        goto exit;
    }

    if (out->disabled) {
        ret = -EPIPE;
        ALOGD("%s: %d: error out = %p",__FUNCTION__,__LINE__,out);
        goto exit;
    }


#ifdef AUDIO_3A
    if (adev->voice_api != NULL) {
        int ret = 0;
        adev->voice_api->queuePlaybackBuffer(buffer, bytes);
        ret = adev->voice_api->getPlaybackBuffer(buffer, bytes);
        if (ret < 0) {
            memset((char *)buffer, 0x00, bytes);
        }
    }
#endif

    /* Write to all active PCMs */
    if ((out->device & AUDIO_DEVICE_OUT_AUX_DIGITAL) && is_bitstream(out)) {
if (!hasExtCodec()){
        ret = bitstream_write_data(out,(void*)buffer,bytes);
        if(ret < 0) {
            goto exit;
        }
}
    } else {
        if (out->is_simcom_voice) {
            if (!out->simcom_attached || !OUT_SIMCOM_PCM(out)) {
                pthread_mutex_lock(&adev->lock);
                int attach_ret = simcom_acquire_tx_pcm(adev, false);
                if (attach_ret == 0) {
                    out->simcom_attached = true;
                    ALOGI("SIMCOM: out_write re-attached TX PCM (pcm=%p users=%d)",
                          OUT_SIMCOM_PCM(out), adev->simcom_tx_users);
                }
                pthread_mutex_unlock(&adev->lock);
                if (attach_ret != 0 || !OUT_SIMCOM_PCM(out)) {
                    ALOGW("SIMCOM: TX PCM not ready (ret=%d), dropping %zu bytes", attach_ret, bytes);
                    ret = 0;
                    goto exit;
                }
            }

            if (buffer && bytes > 0) {
                simcom_log_pcm_snapshot("TX->modem", buffer, bytes);
            }
            ALOGV("SIMCOM: out_write telephony pcm=%p bytes=%zu", OUT_SIMCOM_PCM(out), bytes);
            ret = pcm_write(OUT_SIMCOM_PCM(out), (void *)buffer, bytes);
            if (ret) {
                ALOGE("SIMCOM: out_write pcm_write failed ret=%d err=%s",
                      ret, pcm_get_error(OUT_SIMCOM_PCM(out)));
            }
            goto exit;
        }

        if (simcom_voice_mode_active(adev) && simcom_tx_fallback_enabled() &&
                !out->is_simcom_voice &&
                buffer != NULL && bytes > 0) {
            pthread_mutex_lock(&adev->lock);
            if (adev->simcom_tx_pcm == NULL) {
                simcom_acquire_tx_pcm(adev, true);
            }
            pthread_mutex_unlock(&adev->lock);
            if (adev->simcom_tx_pcm != NULL) {
                uint32_t in_rate = out->aud_config.sample_rate ?
                        out->aud_config.sample_rate : out->config.rate;
                if (in_rate == 0) {
                    in_rate = out->config.rate;
                }
                size_t in_channels = audio_channel_count_from_out_mask(out->channel_mask);
                if (in_channels == 0) {
                    in_channels = out->config.channels;
                }
                const int16_t *tx_src = (const int16_t *)buffer;
                size_t tx_frames = bytes / (in_channels * sizeof(int16_t));
                size_t tx_channels = in_channels;
                
                // SIMCOM: Step 1 - Convert to mono first (before resampling for efficiency)
                if (tx_frames > 0 && in_channels > 1) {
                    if (simcom_ensure_tx_mono_buffer(out, tx_frames) == 0) {
                        int16_t *mono = out->simcom_mono_buffer;
                        for (size_t f = 0; f < tx_frames; ++f) {
                            int32_t acc = 0;
                            for (size_t c = 0; c < in_channels; ++c) {
                                acc += tx_src[f * in_channels + c];
                            }
                            mono[f] = (int16_t)(acc / (int32_t)in_channels);
                        }
                        tx_src = mono;
                        tx_channels = 1; // Now mono
                    } else {
                        tx_frames = 0;
                    }
                }
                
                // SIMCOM: Step 2 - Resample mono signal (more efficient than resampling stereo)
                if (tx_frames > 0 && in_rate != SIMCOM_TX_TARGET_RATE) {
                    int resample_ret = simcom_prepare_tx_resampler(out, in_rate, tx_channels);
                    if (resample_ret == 0 && out->simcom_resampler != NULL) {
                        size_t tmp_in = tx_frames;
                        size_t tmp_out = (size_t)(((uint64_t)tx_frames * SIMCOM_TX_TARGET_RATE) / in_rate) + 1;
                        if (simcom_ensure_tx_resampler_buffer(out, tmp_out, tx_channels) == 0) {
                            out->simcom_resampler->resample_from_input(out->simcom_resampler,
                                                                       (int16_t *)tx_src, &tmp_in,
                                                                       out->simcom_resampler_buffer,
                                                                       &tmp_out);
                            tx_src = out->simcom_resampler_buffer;
                            tx_frames = tmp_out;
                        } else {
                            tx_frames = 0;
                        }
                    } else if (resample_ret != 0) {
                        ALOGW("SIMCOM: TX resampler unavailable for rate %u->%u, skipping modem feed", 
                              in_rate, SIMCOM_TX_TARGET_RATE);
                        tx_frames = 0;
                    }
                }
                if (tx_frames > 0) {
                    // SIMCOM: Check PCM readiness before write to avoid blocking/deadlock
                    if (!pcm_is_ready(adev->simcom_tx_pcm)) {
                        ALOGW("SIMCOM: TX PCM not ready, skipping write. Error: %s",
                              pcm_get_error(adev->simcom_tx_pcm));
                        // Try to reacquire PCM device
                        pthread_mutex_lock(&adev->lock);
                        if (adev->simcom_tx_pcm) {
                            pcm_close(adev->simcom_tx_pcm);
                            adev->simcom_tx_pcm = NULL;
                        }
                        simcom_acquire_tx_pcm(adev, false);
                        pthread_mutex_unlock(&adev->lock);
                    } else {
                        size_t tx_bytes = tx_frames * sizeof(int16_t);
                        simcom_log_pcm_snapshot("TX->modem (fallback)", tx_src, tx_bytes);
                        int tx_ret = pcm_write(adev->simcom_tx_pcm, tx_src, tx_bytes);
                        if (tx_ret < 0) {
                            ALOGW("SIMCOM: fallback pcm_write failed ret=%d err=%s (non-fatal, continuing)",
                                  tx_ret, pcm_get_error(adev->simcom_tx_pcm));
                            // Don't block on PCM write errors - continue with main stream
                        }
                    }
                }
            }
        }

        out_mute_data(out,(void*)buffer,bytes);
        dump_out_data(buffer, bytes);
        ret = -1;
        // SIMCOM: Log when writing to speaker PCM for voice call
        if (adev->mode == AUDIO_MODE_IN_CALL && 
            out->device & (AUDIO_DEVICE_OUT_SPEAKER | AUDIO_DEVICE_OUT_WIRED_HEADSET | AUDIO_DEVICE_OUT_WIRED_HEADPHONE)) {
            if (out->pcm[SND_OUT_SOUND_CARD_SPEAKER]) {
                ALOGI("SIMCOM: out_write voice call RX: device=0x%x bytes=%zu pcm=%p ready=%d",
                      out->device, bytes, out->pcm[SND_OUT_SOUND_CARD_SPEAKER],
                      pcm_is_ready(out->pcm[SND_OUT_SOUND_CARD_SPEAKER]) ? 1 : 0);
            }
        }
        for (i = 0; i < SND_OUT_SOUND_CARD_MAX; i++)
            if (out->pcm[i]) {
#ifdef BT_AP_SCO
                if (i == SND_OUT_SOUND_CARD_BT) {
                    // HARD CODE FIXME 48000 stereo -> 8000 stereo
                    size_t inFrameCount = bytes/2/2;
                    size_t outFrameCount = inFrameCount/(out->config.rate/pcm_config_ap_sco.rate);
                    int16_t out_buffer[outFrameCount*2];
                    memset(out_buffer, 0x00, outFrameCount*2);

                    out->resampler->resample_from_input(out->resampler,
                                                        (const int16_t *)buffer,
                                                        &inFrameCount,
                                                        out_buffer,
                                                        &outFrameCount);

                    ret = pcm_write(out->pcm[i], (void *)out_buffer, outFrameCount*2*2);
                    if (ret != 0)
                        break;
                } else {
#else
                {
#endif
if (!hasExtCodec()){
                    /*
                     * do not write hdmi/spdif snd sound if they are taken by other bitstream/multi channle pcm stream
                     */
                    if(((i == SND_OUT_SOUND_CARD_HDMI) && (adev->owner[SOUND_CARD_HDMI] != (int*)out) && (adev->owner[SOUND_CARD_HDMI] != NULL)) ||
                        ((i == SND_OUT_SOUND_CARD_SPDIF) && (adev->owner[SOUND_CARD_SPDIF] != (int*)out) && (adev->owner[SOUND_CARD_SPDIF] != NULL))){
                        continue;
                    }
}
                    // SIMCOM: Log pcm_write for voice call to diagnose audio issues
                    if (adev->mode == AUDIO_MODE_IN_CALL && 
                        i == SND_OUT_SOUND_CARD_SPEAKER) {
                        int16_t *data = (int16_t *)buffer;
                        size_t samples = bytes / sizeof(int16_t);
                        int16_t peak = 0;
                        for (size_t s = 0; s < samples && s < 100; s++) {
                            int16_t val = data[s] < 0 ? -data[s] : data[s];
                            if (val > peak) peak = val;
                        }
                        ALOGI("SIMCOM: pcm_write to speaker: card=%d device=%d bytes=%zu rate=%u channels=%u format=%u pcm=%p peak=%d",
                              adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card,
                              adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].device,
                              bytes, out->config.rate, out->config.channels, out->config.format,
                              out->pcm[i], peak);
}
                    ret = pcm_write(out->pcm[i], (void *)buffer, bytes);
                    if (ret != 0) {
                        if (adev->mode == AUDIO_MODE_IN_CALL && 
                            i == SND_OUT_SOUND_CARD_SPEAKER) {
                            ALOGE("SIMCOM: pcm_write to speaker failed: ret=%d err=%s",
                                  ret, pcm_get_error(out->pcm[i]));
                        }
                        break;
                    } else if (adev->mode == AUDIO_MODE_IN_CALL && 
                               i == SND_OUT_SOUND_CARD_SPEAKER) {
                        // SIMCOM: Log successful pcm_write for voice call
                        ALOGV("SIMCOM: pcm_write to speaker succeeded: bytes=%zu ret=%d", bytes, ret);
                    }
                }
            }
    }
exit:
    pthread_mutex_unlock(&out->lock);
final_exit:
    {
        // For PCM we always consume the buffer and return #bytes regardless of ret.
        out->written += bytes / (out->config.channels * sizeof(short));
        out->nframes = out->written;
    }
    if (ret != 0) {
        ALOGD("AudioData write  error , keep slience! ret = %d", ret);
        usleep(bytes * 1000000 / audio_stream_out_frame_size(stream) /
               out_get_sample_rate(&stream->common));
    }

    return bytes;
}

/**
 * @brief out_get_render_position
 *
 * @param stream
 * @param dsp_frames
 *
 * @returns
 */
static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    struct stream_out *out = (struct stream_out *)stream;

    *dsp_frames = out->nframes;
    return 0;
}

/**
 * @brief out_add_audio_effect
 *
 * @param stream
 * @param effect
 *
 * @returns
 */
static int out_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

/**
 * @brief out_remove_audio_effect
 *
 * @param stream
 * @param effect
 *
 * @returns
 */
static int out_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

/**
 * @brief out_get_next_write_timestamp
 *
 * @param stream
 * @param timestamp
 *
 * @returns
 */
static int out_get_next_write_timestamp(const struct audio_stream_out *stream,
                                        int64_t *timestamp)
{
    ALOGV("%s: %d Entered", __FUNCTION__, __LINE__);
    return -ENOSYS;
}

/**
 * @brief out_get_presentation_position
 *
 * @param stream
 * @param frames
 * @param timestamp
 *
 * @returns
 */
static int out_get_presentation_position(const struct audio_stream_out *stream,
        uint64_t *frames, struct timespec *timestamp)
{
    struct stream_out *out = (struct stream_out *)stream;
    int ret = -1;

    pthread_mutex_lock(&out->lock);

    int i;
    // There is a question how to implement this correctly when there is more than one PCM stream.
    // We are just interested in the frames pending for playback in the kernel buffer here,
    // not the total played since start.  The current behavior should be safe because the
    // cases where both cards are active are marginal.
    for (i = 0; i < SND_OUT_SOUND_CARD_MAX; i++)
        if (out->pcm[i]) {
            size_t avail;
            //ALOGD("===============%s,%d==============",__FUNCTION__,__LINE__);
            if (pcm_get_htimestamp(out->pcm[i], &avail, timestamp) == 0) {
                size_t kernel_buffer_size = out->config.period_size * out->config.period_count;
                //ALOGD("===============%s,%d==============",__FUNCTION__,__LINE__);
                // FIXME This calculation is incorrect if there is buffering after app processor
                int64_t signed_frames = out->written - kernel_buffer_size + avail;
                //signed_frames -= 17;
                //ALOGV("============singed_frames:%lld=======",signed_frames);
                //ALOGV("============timestamp:%lld==========",timestamp);
                // It would be unusual for this value to be negative, but check just in case ...
                if (signed_frames >= 0) {
                    *frames = signed_frames;
                    ret = 0;
                }
                break;
            }
        }
    pthread_mutex_unlock(&out->lock);

    return ret;
}

/**
 * @brief in_get_sample_rate
 * audio_stream_in implementation
 *
 * @param stream
 *
 * @returns
 */
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;
    //ALOGV("%s:get requested_rate : %d ",__FUNCTION__,in->requested_rate);
    return in->requested_rate;
}

/**
 * @brief in_set_sample_rate
 *
 * @param stream
 * @param rate
 *
 * @returns
 */
static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    return 0;
}

/**
 * @brief in_get_channels
 *
 * @param stream
 *
 * @returns
 */
static audio_channel_mask_t in_get_channels(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;

    //ALOGV("%s:get channel_mask : %d ",__FUNCTION__,in->channel_mask);
    return in->channel_mask;
}


/**
 * @brief in_get_buffer_size
 *
 * @param stream
 *
 * @returns
 */
static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;

    return get_input_buffer_size(in->requested_rate,
                                 AUDIO_FORMAT_PCM_16_BIT,
                                 audio_channel_count_from_in_mask(in_get_channels(stream)),
                                 (in->flags & AUDIO_INPUT_FLAG_FAST) != 0);
}

/**
 * @brief in_get_format
 *
 * @param stream
 *
 * @returns
 */
static audio_format_t in_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

/**
 * @brief in_set_format
 *
 * @param stream
 * @param format
 *
 * @returns
 */
static int in_set_format(struct audio_stream *stream, audio_format_t format)
{
    return -ENOSYS;
}

/**
 * @brief do_in_standby
 * must be called with in stream and hw device mutex locked
 *
 * @param in
 */
static void do_in_standby(struct stream_in *in)
{
    struct audio_device *adev = in->dev;

    if (!in->standby) {
        if (in->is_simcom_voice && in->simcom_attached) {
            simcom_release_rx_pcm(adev);
            in->pcm = NULL;
            in->simcom_attached = false;
        } else {
        pcm_close(in->pcm);
        in->pcm = NULL;
        }

        if (in->device & AUDIO_DEVICE_IN_HDMI) {
            route_pcm_close(HDMI_IN_CAPTURE_OFF_ROUTE);
        }

        in->dev->input_source = AUDIO_SOURCE_DEFAULT;
        in->dev->in_device = AUDIO_DEVICE_NONE;
        in->dev->in_channel_mask = 0;
        in->standby = true;
        route_pcm_close(CAPTURE_OFF_ROUTE);
        in->simcom_rx_last_gen = 0;
        
        // Reset audio detection state when going to standby
        if (in->is_simcom_voice) {
            in->simcom_audio_detected = false;
            in->simcom_silence_count = 0;
            in->simcom_audio_confirm_count = 0;
        }
        
        // SIMCOM: Clear resampler buffer to prevent stale data
        if (in->buffer) {
            memset(in->buffer, 0, pcm_frames_to_bytes(in->pcm, in->config->period_size));
            ALOGD("SIMCOM: cleared input resampler buffer");
        }
        in->frames_in = 0;
    }

}

/**
 * @brief in_standby
 *
 * @param stream
 *
 * @returns
 */
static int in_standby(struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;

    pthread_mutex_lock(&in->lock);
    pthread_mutex_lock(&in->dev->lock);

    do_in_standby(in);

    pthread_mutex_unlock(&in->dev->lock);
    pthread_mutex_unlock(&in->lock);

    return 0;
}

/**
 * @brief in_dump
 *
 * @param stream
 * @param fd
 *
 * @returns
 */
int in_dump(const struct audio_stream *stream, int fd)
{
    struct stream_in *in = (struct stream_in *)stream;

    ALOGD("in->Device     : 0x%x", in->device);
    ALOGD("in->SampleRate : %d", in->config->rate);
    ALOGD("in->Channels   : %d", in->config->channels);
    ALOGD("in->Formate    : %d", in->config->format);
    ALOGD("in->PreiodSize : %d", in->config->period_size);

    return 0;
}

/**
 * @brief in_set_parameters
 *
 * @param stream
 * @param kvpairs
 *
 * @returns
 */
static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct stream_in *in = (struct stream_in *)stream;
    struct audio_device *adev = in->dev;
    struct str_parms *parms;
    char value[32];
    int ret;
    int status = 0;
    unsigned int val;
    bool apply_now = false;

    ALOGV("%s: kvpairs = %s", __func__, kvpairs);
    parms = str_parms_create_str(kvpairs);

    //set channel_mask
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_CHANNELS,
                            value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        in->channel_mask = val;
    }
     // set sample rate
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_SAMPLING_RATE,
                            value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        in->requested_rate = val;
    }

    pthread_mutex_lock(&in->lock);
    pthread_mutex_lock(&adev->lock);
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_INPUT_SOURCE,
                            value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        /* no audio source uses val == 0 */
        if ((in->input_source != val) && (val != 0)) {
            in->input_source = val;
            apply_now = !in->standby;
        }
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));
    if (ret >= 0) {
        /* strip AUDIO_DEVICE_BIT_IN to allow bitwise comparisons */
        val = atoi(value) & ~AUDIO_DEVICE_BIT_IN;
        /* no audio device uses val == 0 */
        if ((in->device != val) && (val != 0)) {
            /* force output standby to start or stop SCO pcm stream if needed */
            if ((val & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET) ^
                    (in->device & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET)) {
                do_in_standby(in);
            }
            in->device = val;
            apply_now = !in->standby;
        }
    }

    if (apply_now) {
        adev->input_source = in->input_source;
        adev->in_device = in->device;
        route_pcm_open(getRouteFromDevice(in->device | AUDIO_DEVICE_BIT_IN));
    }

    pthread_mutex_unlock(&adev->lock);
    pthread_mutex_unlock(&in->lock);

    str_parms_destroy(parms);

    ALOGV("%s: exit: status(%d)", __func__, status);
    return status;

}

/**
 * @brief in_get_parameters
 *
 * @param stream
 * @param keys
 *
 * @returns
 */
static char * in_get_parameters(const struct audio_stream *stream,
                                const char *keys)
{
    ALOGD("%s: keys = %s", __func__, keys);

    struct stream_in *in = (struct stream_in *)stream;
    struct str_parms *query = str_parms_create_str(keys);
    char *str = NULL;
    struct str_parms *reply = str_parms_create();

    if (stream_get_parameter_formats(stream,query,reply) == 0) {
        str = str_parms_to_str(reply);
    } else if (stream_get_parameter_channels(query, reply, &in->supported_channel_masks[0]) == 0) {
        str = str_parms_to_str(reply);
    } else if (stream_get_parameter_rates(query, reply, &in->supported_sample_rates[0]) == 0) {
        str = str_parms_to_str(reply);
    } else {
        ALOGD("%s,str_parms_get_str failed !",__func__);
        str = strdup("");
    }

    str_parms_destroy(query);
    str_parms_destroy(reply);

    ALOGV("%s,exit -- str = %s",__func__,str);
    return str;
}

/**
 * @brief in_set_gain
 *
 * @param stream
 * @param gain
 *
 * @returns
 */
static int in_set_gain(struct audio_stream_in *stream, float gain)
{
    return 0;
}

/**
 * @brief in_apply_ramp
 *
 * @param in
 * @param buffer
 * @param frames
 */
static void in_apply_ramp(struct stream_in *in, int16_t *buffer, size_t frames)
{
    size_t i;
    uint16_t vol = in->ramp_vol;
    uint16_t step = in->ramp_step;

    frames = (frames < in->ramp_frames) ? frames : in->ramp_frames;

    if (in->channel_mask == AUDIO_CHANNEL_IN_MONO)
        for (i = 0; i < frames; i++) {
            buffer[i] = (int16_t)((buffer[i] * vol) >> 16);
            vol += step;
        }
    else
        for (i = 0; i < frames; i++) {
            buffer[2*i] = (int16_t)((buffer[2*i] * vol) >> 16);
            buffer[2*i + 1] = (int16_t)((buffer[2*i + 1] * vol) >> 16);
            vol += step;
        }


    in->ramp_vol = vol;
    in->ramp_frames -= frames;
}

/**
 * @brief in_read
 *
 * @param stream
 * @param buffer
 * @param bytes
 *
 * @returns
 */
static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
                       size_t bytes)
{
    int ret = 0;
    struct stream_in *in = (struct stream_in *)stream;
    struct audio_device *adev = in->dev;
    size_t frames_rq = bytes / audio_stream_in_frame_size(stream);

    // SIMCOM: Log in_read entry for telephony RX streams
    if (in->is_simcom_voice) {
        static int log_counter = 0;
        if ((log_counter++ % 50) == 0) { // Log every 50th call to avoid spam
            ALOGI("SIMCOM: in_read called: is_simcom_voice=1 bypass_pcm=%d standby=%d bytes=%zu device=0x%x",
                  in->bypass_pcm ? 1 : 0, in->standby ? 1 : 0, bytes, in->device);
        }
    }

    if (in->device & AUDIO_DEVICE_IN_HDMI) {
        unsigned int rate = get_hdmiin_audio_rate(adev);
        if(rate != in->config->rate){
            ALOGD("HDMI-In: rate is changed: %d -> %d, restart input stream",
                    in->config->rate, rate);
            do_in_standby(in);
        }
    }
    /*
     * acquiring hw device mutex systematically is useful if a low
     * priority thread is waiting on the input stream mutex - e.g.
     * executing in_set_parameters() while holding the hw device
     * mutex
     */
    pthread_mutex_lock(&in->lock);
    if (in->standby) {
        pthread_mutex_lock(&adev->lock);
        ret = start_input_stream(in);
        pthread_mutex_unlock(&adev->lock);
        if (ret < 0)
            goto exit;
        in->standby = false;
#ifdef AUDIO_3A
        if (adev->voice_api != NULL) {
            adev->voice_api->start();
        }
#endif
    }

    // Для Telephony устройств в patch mode данные передаются через патч AudioFlinger
    // HAL не должен открывать PCM напрямую, но должен передавать данные через stream
    if (in->bypass_pcm) {
        // В patch mode AudioFlinger управляет PCM, HAL просто передает данные
        // Заполняем буфер нулями (или данными через патч) и возвращаем количество байт
        ALOGD("SIMCOM: bypass_pcm in_read: %zu bytes passed through AudioFlinger patch", bytes);
        memset(buffer, 0, bytes);
        ret = bytes;  // Возвращаем количество байт, как будто прочитали
        goto exit;
    }

    /*if (in->num_preprocessors != 0)
        ret = process_frames(in, buffer, frames_rq);
      else */
    //ALOGV("%s:frames_rq:%d",__FUNCTION__,frames_rq);
    if (in->is_simcom_voice) {
        if (!IN_SIMCOM_PCM(in)) {
            pthread_mutex_lock(&adev->lock);
            int attach_ret = simcom_acquire_rx_pcm(adev, false);
            if (attach_ret == 0) {
                in->simcom_attached = true;
                in->pcm = IN_SIMCOM_PCM(in);
                ALOGI("SIMCOM: in_read re-attached RX PCM (pcm=%p users=%d)",
                      IN_SIMCOM_PCM(in), adev->simcom_rx_users);
            } else {
                ALOGW("SIMCOM: in_read failed to acquire RX PCM (ret=%d)", attach_ret);
            }
            pthread_mutex_unlock(&adev->lock);
        }

        if (!IN_SIMCOM_PCM(in)) {
            ALOGW("SIMCOM: in_read PCM not ready, returning silence (%zu bytes)", bytes);
            memset(buffer, 0, bytes);
            if (buffer && bytes > 0) {
                simcom_log_pcm_snapshot("RX<-modem (pcm not ready)", buffer, bytes);
            }
            ret = bytes;
            goto exit;
        }
        
        // SIMCOM: Log PCM readiness for RX
        if (IN_SIMCOM_PCM(in) && !pcm_is_ready(IN_SIMCOM_PCM(in))) {
            ALOGW("SIMCOM: in_read RX PCM not ready: %s", pcm_get_error(IN_SIMCOM_PCM(in)));
        }
    }

    int retry = 0;
    do {
    ret = read_frames(in, buffer, frames_rq);
        if (ret >= 0) {
        ret = 0;
            break;
        }

        const bool can_retry = in->is_simcom_voice &&
                               (ret == -EAGAIN || ret == -EBUSY || ret == -EINTR);
        if (!can_retry) {
            ALOGE("SIMCOM: in_read read_frames failed ret=%d pcm=%p", ret, in->pcm);
            break;
        }

        if (retry == 0) {
            ALOGW("SIMCOM: in_read pcm_read busy (ret=%d) pcm=%p, will retry up to %d times",
                  ret, in->pcm, SIMCOM_RX_READ_MAX_RETRIES);
        }

        if (retry >= SIMCOM_RX_READ_MAX_RETRIES) {
            ALOGE("SIMCOM: in_read still busy after %d retries, returning silence",
                  SIMCOM_RX_READ_MAX_RETRIES);
            memset(buffer, 0, bytes);
            if (buffer && bytes > 0) {
                simcom_log_pcm_snapshot("RX<-modem (retry timeout)", buffer, bytes);
            }
            ret = 0;
            break;
        }

        retry++;
        usleep(SIMCOM_RX_READ_RETRY_US);
    } while (true);

    // SIMCOM RX: Detect real audio and suppress zeros during call establishment
    // Temporarily disable detection to test if it causes clicks
    // TODO: Re-enable after fixing resampling/fragmentation issue
    /*
    if (in->is_simcom_voice && buffer && bytes > 0 && ret == 0) {
        bool should_pass_through = simcom_update_audio_detection(in, buffer, bytes);
        
        if (!should_pass_through) {
            // Audio not yet detected or in silence period, suppress zeros
            memset(buffer, 0, bytes);
            
            // Log detection state periodically
            static int log_counter = 0;
            if ((log_counter++ % 100) == 0) {
                ALOGD("SIMCOM: RX audio detection: detected=%d confirm=%u silence=%u (suppressing zeros)",
                      in->simcom_audio_detected ? 1 : 0,
                      in->simcom_audio_confirm_count,
                      in->simcom_silence_count);
            }
        } else {
            // Real audio confirmed, log periodically
            static int log_counter = 0;
            if ((log_counter++ % 200) == 0) {
                ALOGD("SIMCOM: RX audio confirmed, passing through");
            }
        }
        
        simcom_log_pcm_snapshot("RX<-modem", buffer, bytes);
    }
    */
    // Log RX data without blocking
    if (in->is_simcom_voice && buffer && bytes > 0) {
        simcom_log_pcm_snapshot("RX<-modem", buffer, bytes);
    }
    dump_in_data(buffer, bytes);

#ifdef AUDIO_3A
    do {
        if (adev->voice_api != NULL) {
            int ret  = 0;
            ret = adev->voice_api->quueCaputureBuffer(buffer, bytes);
            if (ret < 0) break;
            ret = adev->voice_api->getCapureBuffer(buffer, bytes);
            if (ret < 0) memset(buffer, 0x00, bytes);
        }
    } while (0);
#endif

    //if (in->ramp_frames > 0)
    //    in_apply_ramp(in, buffer, frames_rq);

    /*
     * Instead of writing zeroes here, we could trust the hardware
     * to always provide zeroes when muted.
     */
    //if (ret == 0 && adev->mic_mute)
    //    memset(buffer, 0, bytes);

    if (in->device & AUDIO_DEVICE_IN_HDMI) {
        goto exit;
    }

#ifdef SPEEX_DENOISE_ENABLE
    if(!adev->mic_mute && ret== 0) {
        int index = 0;
        int startPos = 0;
        spx_int16_t* data = (spx_int16_t*) buffer;

        int channel_count = audio_channel_count_from_out_mask(in->channel_mask);
        int curFrameSize = bytes/(channel_count*sizeof(int16_t));
        long ch;
        ALOGV("channel_count:%d",channel_count);
        if(curFrameSize != in->mSpeexFrameSize)
            ALOGD("the current request have some error mSpeexFrameSize %d bytes %d ",in->mSpeexFrameSize, bytes);

        while(curFrameSize >= startPos+in->mSpeexFrameSize) {
            if( 2 == channel_count) {
                for(index = startPos; index< startPos +in->mSpeexFrameSize ; index++ )
                    in->mSpeexPcmIn[index-startPos] = data[index*channel_count]/2 + data[index*channel_count+1]/2;
            } else {
                for(index = startPos; index< startPos +in->mSpeexFrameSize ; index++ )
                    in->mSpeexPcmIn[index-startPos] = data[index*channel_count];
            }
            speex_preprocess_run(in->mSpeexState,in->mSpeexPcmIn);
#ifndef TARGET_RK2928
            for(ch = 0 ; ch < channel_count; ch++)
                for(index = startPos; index< startPos + in->mSpeexFrameSize ; index++ ) {
                    data[index*channel_count+ch] = in->mSpeexPcmIn[index-startPos];
                }
#else
            for(index = startPos; index< startPos + in->mSpeexFrameSize ; index++ ) {
                int tmp = (int)in->mSpeexPcmIn[index-startPos]+ in->mSpeexPcmIn[index-startPos]/2;
                data[index*channel_count+0] = tmp > 32767 ? 32767 : (tmp < -32768 ? -32768 : tmp);
            }
            for(int ch = 1 ; ch < channel_count; ch++)
                for(index = startPos; index< startPos + in->mSpeexFrameSize ; index++ ) {
                    data[index*channel_count+ch] = data[index*channel_count+0];
                }
#endif
            startPos += in->mSpeexFrameSize;
        }
    }
#endif

#ifdef ALSA_IN_DEBUG
        fwrite(buffer, bytes, 1, in_debug);
#endif
exit:
    if (ret < 0) {
        usleep(bytes * 1000000 / audio_stream_in_frame_size(stream) /
               in_get_sample_rate(&stream->common));
        do_in_standby(in);
    }

    pthread_mutex_unlock(&in->lock);
    return bytes;
}

/**
 * @brief in_get_input_frames_lost
 *
 * @param stream
 *
 * @returns
 */
static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream)
{
    return 0;
}

/**
 * @brief in_add_audio_effect
 *
 * @param stream
 * @param effect
 *
 * @returns
 */
static int in_add_audio_effect(const struct audio_stream *stream,
                               effect_handle_t effect)
{
    struct stream_in *in = (struct stream_in *)stream;
    effect_descriptor_t descr;
    if ((*effect)->get_descriptor(effect, &descr) == 0) {

        pthread_mutex_lock(&in->lock);
        pthread_mutex_lock(&in->dev->lock);


        pthread_mutex_unlock(&in->dev->lock);
        pthread_mutex_unlock(&in->lock);
    }

    return 0;
}

/**
 * @brief in_remove_audio_effect
 *
 * @param stream
 * @param effect
 *
 * @returns
 */
static int in_remove_audio_effect(const struct audio_stream *stream,
                                  effect_handle_t effect)
{
    struct stream_in *in = (struct stream_in *)stream;
    effect_descriptor_t descr;
    if ((*effect)->get_descriptor(effect, &descr) == 0) {

        pthread_mutex_lock(&in->lock);
        pthread_mutex_lock(&in->dev->lock);


        pthread_mutex_unlock(&in->dev->lock);
        pthread_mutex_unlock(&in->lock);
    }

    return 0;
}

static int adev_get_microphones(const struct audio_hw_device *dev,
                         struct audio_microphone_characteristic_t *mic_array,
                         size_t *mic_count)
{
    struct audio_device *adev = (struct audio_device *)dev;
    size_t actual_mic_count = 0;

    int card_no = 0;

    char snd_card_node_id[100]={0};
    char snd_card_node_cap[100]={0};
    char address[32] = "bottom";

    do{
        sprintf(snd_card_node_id, "/proc/asound/card%d/id", card_no);
        if (access(snd_card_node_id,F_OK) == -1) break;

        sprintf(snd_card_node_cap, "/proc/asound/card%d/pcm0c/info", card_no);
        if (access(snd_card_node_cap,F_OK) == -1) continue;

        actual_mic_count++;
    }while(++card_no);

    mic_array->device = -2147483644;
    strcpy(mic_array->address,address);

    ALOGD("%s,get capture mic actual_mic_count =%d",__func__,actual_mic_count);
    *mic_count = actual_mic_count;
    return 0;
}

static int in_get_active_microphones(const struct audio_stream_in *stream,
                         struct audio_microphone_characteristic_t *mic_array,
                         size_t *mic_count)
{
    struct stream_in *in = (struct stream_in *)stream;
    struct audio_device *adev = in->dev;
    pthread_mutex_lock(&in->lock);
    pthread_mutex_lock(&adev->lock);

    size_t actual_mic_count = 0;
    int card_no = 0;

    char snd_card_node_id[100]={0};
    char snd_card_node_cap[100]={0};
    char snd_card_info[100]={0};
    char snd_card_state[255]={0};

    do{
        sprintf(snd_card_node_id, "/proc/asound/card%d/id", card_no);
        if (access(snd_card_node_id,F_OK) == -1) break;

        sprintf(snd_card_node_cap, "/proc/asound/card%d/pcm0c/info", card_no);
        if (access(snd_card_node_cap,F_OK) == -1) {
            continue;
        } else {
            sprintf(snd_card_info, "/proc/asound/card%d/pcm0c/sub0/status", card_no);
            int fd;
            fd = open(snd_card_info, O_RDONLY);
            if (fd < 0) {
                ALOGE("%s,failed to open node: %s", __func__, snd_card_info);
            } else {
                int length = read(fd, snd_card_state, sizeof(snd_card_state) -1);
                snd_card_state[length] = 0;
                if (strcmp(snd_card_state, "closed") != 0) actual_mic_count++;
            }
            close(fd);
        }
    } while(++card_no);

    pthread_mutex_unlock(&adev->lock);
    pthread_mutex_unlock(&in->lock);

    ALOGD("%s,get active mic actual_mic_count =%d", __func__, actual_mic_count);
    *mic_count = actual_mic_count;
    return 0;
}

/*
 * get support channels mask of hdmi from parsing edid of hdmi
 */
static void get_hdmi_support_channels_masks(struct stream_out *out)
{
    if(out == NULL)
        return ;

    int channels = get_hdmi_audio_speaker_allocation(&out->hdmi_audio);
    switch (channels) {
    case AUDIO_CHANNEL_OUT_5POINT1:
        ALOGD("%s: HDMI Support 5.1 channels pcm",__FUNCTION__);
        out->supported_channel_masks[0] = AUDIO_CHANNEL_OUT_5POINT1;
        out->supported_channel_masks[1] = AUDIO_CHANNEL_OUT_STEREO;
        break;
    case AUDIO_CHANNEL_OUT_7POINT1:
        ALOGD("%s: HDMI Support 7.1 channels pcm",__FUNCTION__);
        out->supported_channel_masks[0] = AUDIO_CHANNEL_OUT_5POINT1;
        out->supported_channel_masks[1] = AUDIO_CHANNEL_OUT_7POINT1;
        break;
    case AUDIO_CHANNEL_OUT_STEREO:
    default:
        ALOGD("%s: HDMI Support 2 channels pcm",__FUNCTION__);
        out->supported_channel_masks[0] = AUDIO_CHANNEL_OUT_STEREO;
        out->supported_channel_masks[1] = AUDIO_CHANNEL_OUT_MONO;
        break;
    }
}

/**
 * @brief adev_open_output_stream
 *
 * @param dev
 * @param handle
 * @param devices
 * @param flags
 * @param config
 * @param stream_out
 * @param __unused
 *
 * @returns
 */
static int adev_open_output_stream(struct audio_hw_device *dev,
                                   audio_io_handle_t handle,
                                   audio_devices_t devices,
                                   audio_output_flags_t flags,
                                   struct audio_config *config,
                                   struct audio_stream_out **stream_out,
                                   const char *address __unused)
{
    struct audio_device *adev = (struct audio_device *)dev;
    struct stream_out *out;
    int ret;
    enum output_type type = OUTPUT_LOW_LATENCY;
    bool isPcm = audio_is_linear_pcm(config->format);

    bool telephony_tx = (devices & AUDIO_DEVICE_OUT_TELEPHONY_TX);
    bool call_mode_active = simcom_voice_mode_active(adev);
    bool force_patch = simcom_force_patch_enabled();
    bool voip_rx_flag = (flags & AUDIO_OUTPUT_FLAG_VOIP_RX);
    bool telephony_voice_stream = telephony_tx || voip_rx_flag;

    ALOGD("audio hal adev_open_output_stream devices = 0x%x, flags = %d, samplerate = %d,format = 0x%x",
          devices, flags, config->sample_rate,config->format);

    if (telephony_voice_stream || call_mode_active) {
        if (telephony_voice_stream) {
            type = OUTPUT_SIMCOM_VOICE;
        }
        if (telephony_tx && !adev->simcom_card_available) {
            ALOGW("SIMCOM: telephony TX requested but SIMCOM device not detected");
        }
        ALOGD("SIMCOM: forcing output stream request to 8kHz mono PCM16 for voice call");
        config->sample_rate = 8000;
        config->channel_mask = AUDIO_CHANNEL_OUT_MONO;
        config->format = AUDIO_FORMAT_PCM_16_BIT;
    }

    out = (struct stream_out *)calloc(1, sizeof(struct stream_out));
    if (!out)
        return -ENOMEM;

    /*get default supported channel_mask*/
    memset(out->supported_channel_masks, 0, sizeof(out->supported_channel_masks));
    out->supported_channel_masks[0] = AUDIO_CHANNEL_OUT_STEREO;
    out->supported_channel_masks[1] = AUDIO_CHANNEL_OUT_MONO;
    /*get default supported sample_rate*/
    memset(out->supported_sample_rates, 0, sizeof(out->supported_sample_rates));
    out->supported_sample_rates[0] = 44100;
    out->supported_sample_rates[1] = 48000;
    out->supported_sample_rates[2] = 8000;

    out->is_simcom_voice = telephony_tx;
    out->bypass_pcm = (force_patch && call_mode_active && !telephony_tx);
    if (voip_rx_flag) {
        out->bypass_pcm = true;
    }
    out->simcom_attached = false;
    
    // SIMCOM: Log bypass_pcm setting for voice call streams (always log, not just when call_mode_active)
    ALOGI("SIMCOM: adev_open_output_stream: force_patch=%d call_mode_active=%d telephony_tx=%d voip_rx=%d bypass_pcm=%d device=0x%x adev->mode=%d adev->voice_call_active=%d",
          force_patch ? 1 : 0, call_mode_active ? 1 : 0, telephony_tx ? 1 : 0,
          voip_rx_flag ? 1 : 0, out->bypass_pcm ? 1 : 0,
          devices, adev->mode, adev->voice_call_active ? 1 : 0);

    if (telephony_tx && force_patch) {
        ALOGI("SIMCOM: Telephony Tx requested - NO PCM in adev_open (patch mode)");
        memset(out->pcm, 0, sizeof(out->pcm));
        out->bypass_pcm = false;
    } else if (telephony_tx) {
        ALOGI("SIMCOM: Telephony Tx requested - direct PCM mode (force_patch disabled)");
    } else if (call_mode_active && force_patch) {
        // SIMCOM: For bypass_pcm streams (voice call RX path), PCM will be opened in start_output_stream
        // Don't zero PCM here - it needs to be opened for speaker/headphone playback
        ALOGI("SIMCOM: Voice call active, bypass_pcm=true - PCM will be opened in start_output_stream for playback");
        // Do NOT zero PCM - it will be opened for speaker/headphone output
    }

    if(config != NULL)
        memcpy(&(out->aud_config),config,sizeof(struct audio_config));
    out->requested_rate = config != NULL ? config->sample_rate : out->config.rate;
    out->channel_mask = AUDIO_CHANNEL_OUT_STEREO;
    if (devices == AUDIO_DEVICE_NONE)
        devices = AUDIO_DEVICE_OUT_SPEAKER;
    out->device = devices;
    /*
     * set output_direct_mode to LPCM, means data is not multi pcm or bitstream datas.
     * set output_direct to false, means data is 2 channels pcm
     */
    out->output_direct_mode = LPCM;
    out->output_direct = false;
    out->snd_reopen = false;
    out->channel_buffer = NULL;
    out->bitstream_buffer = NULL;

    init_hdmi_audio(&out->hdmi_audio);
    if(devices & AUDIO_DEVICE_OUT_AUX_DIGITAL) {
        parse_hdmi_audio(&out->hdmi_audio);
        get_hdmi_support_channels_masks(out);
    }

    if (out->is_simcom_voice) {
        out->config = simcom_pcm_config_tx;
        out->pcm_device = SIMCOM_PCM_DEVICE;
    } else if (out->bypass_pcm) {
        out->config = simcom_pcm_config_tx;
        out->pcm_device = SIMCOM_PCM_DEVICE;
    } else if (flags & AUDIO_OUTPUT_FLAG_DIRECT) {
        if (devices & AUDIO_DEVICE_OUT_AUX_DIGITAL) {
            if (config->format == AUDIO_FORMAT_IEC61937) {
                ALOGD("%s:out = %p HDMI Bitstream",__FUNCTION__,out);
                out->channel_mask = config->channel_mask;
                if (isValidSamplerate(config->sample_rate)) {
                    out->config = pcm_config_direct;
                    out->config.rate = config->sample_rate;
                    out->output_direct = true;
                    int channel = audio_channel_count_from_out_mask(config->channel_mask);

                    if (channel == 8 && config->sample_rate == 192000) {
                        out->output_direct_mode = HBR;
                    } else {
                        out->output_direct_mode = NLPCM;
                    }

                    if (channel == 8) {
                        out->config = pcm_config_direct;
                        out->config.rate = config->sample_rate;
                    } else if (config->sample_rate >= 176400) {
                        out->config.period_size = 1024 * 4;
                    } else {
                        out->config.period_size = 2048;
                    }
                    type = OUTPUT_HDMI_MULTI;
                } else {
                    out->config = pcm_config;
                    out->config.rate = 44100;
                    ALOGE("hdmi bitstream samplerate %d unsupport", config->sample_rate);
                }
                out->config.channels = audio_channel_count_from_out_mask(config->channel_mask);
                if (out->config.channels < 2)
                    out->config.channels = 2;
                out->pcm_device = PCM_DEVICE;
                out->device = AUDIO_DEVICE_OUT_AUX_DIGITAL;
            } else if (isPcm){ // multi pcm
                if (config->sample_rate == 0)
                    config->sample_rate = HDMI_MULTI_DEFAULT_SAMPLING_RATE;
                if (config->channel_mask == 0)
                    config->channel_mask = AUDIO_CHANNEL_OUT_5POINT1;

                int layout = get_hdmi_audio_speaker_allocation(&out->hdmi_audio);
                unsigned int mask = (layout&config->channel_mask);
                ALOGD("%s:out = %p HDMI multi pcm: layout = 0x%x,mask = 0x%x",
                    __FUNCTION__,out,layout,mask);
                // current hdmi allocation(speaker) only support MONO or STEREO
                if(mask <= (int)AUDIO_CHANNEL_OUT_STEREO) {
                    ALOGD("%s:out = %p input stream is multi pcm,channle mask = 0x%x,but hdmi not support,mixer it to stereo output",
                        __FUNCTION__,out,config->channel_mask);
                    out->channel_mask = AUDIO_CHANNEL_OUT_STEREO;
                    out->config = pcm_config;
                    out->pcm_device = PCM_DEVICE;
                    type = OUTPUT_LOW_LATENCY;
                    out->device = AUDIO_DEVICE_OUT_AUX_DIGITAL;
                    out->output_direct = false;
                } else {
                    /*
                     * maybe input audio stream is 7.1 channels,
                     * but hdmi only support 5.1, we also output 7.1 for default.
                     * Is better than output 2 channels after mixer?
                     * If customer like output 2 channles data after mixer,
                     * modify codes here
                     */
                    out->channel_mask = config->channel_mask;
                    out->config = pcm_config_hdmi_multi;
                    out->config.rate = config->sample_rate;
                    out->config.channels = audio_channel_count_from_out_mask(config->channel_mask);
                    out->pcm_device = PCM_DEVICE;
                    type = OUTPUT_HDMI_MULTI;
                    out->device = AUDIO_DEVICE_OUT_AUX_DIGITAL;
                    out->output_direct = true;
                }
            } else {
                ALOGD("Not any bitstream mode!");
            }
        } else if ((devices & AUDIO_DEVICE_OUT_SPDIF)
                      && (config->format == AUDIO_FORMAT_IEC61937)) {
            ALOGD("%s:out = %p Spdif Bitstream",__FUNCTION__,out);
            out->channel_mask = config->channel_mask;
            out->config = pcm_config_direct;
            if ((config->sample_rate == 48000) ||
                    (config->sample_rate == 32000) ||
                    (config->sample_rate == 44100)) {
                out->config.rate = config->sample_rate;
                out->config.format = PCM_FORMAT_S16_LE;
                out->config.period_size = 2048;
            } else {
                out->config.rate = 44100;
                ALOGE("spdif passthrough samplerate %d is unsupport",config->sample_rate);
            }
            out->config.channels = audio_channel_count_from_out_mask(config->channel_mask);
            devices = AUDIO_DEVICE_OUT_SPDIF;
            out->pcm_device = PCM_DEVICE;
            out->output_direct = true;
            type = OUTPUT_HDMI_MULTI;
            out->device = AUDIO_DEVICE_OUT_SPDIF;
            out->output_direct_mode = NLPCM;
        } else {
            out->config = pcm_config;
            out->pcm_device = PCM_DEVICE;
            type = OUTPUT_LOW_LATENCY;
        }
    } else if (flags & AUDIO_OUTPUT_FLAG_DEEP_BUFFER) {
        out->config = pcm_config_deep;
        out->pcm_device = PCM_DEVICE_DEEP;
        type = OUTPUT_DEEP_BUF;
    } else {
        // Убрано дублирующее условие !out->is_simcom_voice - уже обработано выше
        out->config = pcm_config;
        out->pcm_device = PCM_DEVICE;
        type = OUTPUT_LOW_LATENCY;
    }

    /*
      * the ip of hdmi need convert 16 bits to 21 bits(except rk3128's ip) if bitstream over hdmi
      */
    if (is_bitstream(out) && (devices & AUDIO_DEVICE_OUT_AUX_DIGITAL)) {
        /*
         * the ip of hdmi need convert 16 bits to 21 bits(except rk3128's ip)
         */
        out->config.format = PCM_FORMAT_S24_LE;
#ifdef RK3128
        out->config.format = PCM_FORMAT_S16_LE;
#endif
        if(out->config.format == PCM_FORMAT_S24_LE){
            out->channel_buffer = malloc(CHASTA_SUB_NUM);
            initchnsta(out->channel_buffer);
            setChanSta(out->channel_buffer,out->config.rate, out->config.channels);
        }
    } else {
        out->config.format = PCM_FORMAT_S16_LE;
    }

    ALOGD("out->config.rate = %d, out->config.channels = %d out->config.format = %d",
          out->config.rate, out->config.channels, out->config.format);

    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    out->stream.common.add_audio_effect = out_add_audio_effect;
    out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;
    out->stream.get_next_write_timestamp = out_get_next_write_timestamp;
    out->stream.get_presentation_position = out_get_presentation_position;

    out->dev = adev;

    out->standby = true;
    out->nframes = 0;

    pthread_mutex_lock(&adev->lock_outputs);
    // Для Telephony устройств в patch mode разрешаем множественные stream'ы
    // AudioFlinger может открывать несколько stream'ов для патчей
    if (adev->outputs[type]) {
        bool is_telephony_new = (out->is_simcom_voice || out->bypass_pcm);
        bool is_telephony_existing = (adev->outputs[type]->is_simcom_voice || adev->outputs[type]->bypass_pcm);
        
        if (is_telephony_new && is_telephony_existing) {
            // Оба stream'а Telephony - разрешаем замену
            ALOGD("SIMCOM: replacing existing Telephony stream in outputs[%d] (new: is_simcom=%d bypass=%d, existing: is_simcom=%d bypass=%d)",
                  type, out->is_simcom_voice, out->bypass_pcm,
                  adev->outputs[type]->is_simcom_voice, adev->outputs[type]->bypass_pcm);
        } else if (!is_telephony_new) {
            // Новый stream НЕ Telephony, а существующий занят - блокируем
            ALOGD("SIMCOM: blocking non-Telephony stream, outputs[%d] already in use", type);
        pthread_mutex_unlock(&adev->lock_outputs);
        ret = -EBUSY;
        goto err_open;
        } else {
            // Новый Telephony, но существующий НЕ Telephony - это не должно происходить, но разрешаем
            ALOGW("SIMCOM: replacing non-Telephony stream with Telephony in outputs[%d]", type);
        }
    }
    adev->outputs[type] = out;
    pthread_mutex_unlock(&adev->lock_outputs);

    *stream_out = &out->stream;

    return 0;

err_open:
    if (out != NULL) {
        destory_hdmi_audio(&out->hdmi_audio);
        free(out);
    }
    *stream_out = NULL;
    return ret;
}

/**
 * @brief adev_close_output_stream
 *
 * @param dev
 * @param stream
 */
static void adev_close_output_stream(struct audio_hw_device *dev,
                                     struct audio_stream_out *stream)
{
    struct audio_device *adev;
    enum output_type type;

    ALOGD("adev_close_output_stream!");
    out_standby(&stream->common);
    adev = (struct audio_device *)dev;
    pthread_mutex_lock(&adev->lock_outputs);
    for (type = 0; type < OUTPUT_TOTAL; ++type) {
        if (adev->outputs[type] == (struct stream_out *) stream) {
            adev->outputs[type] = NULL;
            break;
        }
    }
    {
        struct stream_out *out = (struct stream_out *)stream;
        if (out->bitstream_buffer != NULL) {
            free(out->bitstream_buffer);
            out->bitstream_buffer = NULL;
        }

        if (out->channel_buffer != NULL) {
            free(out->channel_buffer);
            out->channel_buffer = NULL;
        }

        destory_hdmi_audio(&out->hdmi_audio);
        simcom_release_tx_resampler(out);
    }
    pthread_mutex_unlock(&adev->lock_outputs);
    free(stream);
}

/**
 * @brief adev_set_parameters
 *
 * @param dev
 * @param kvpairs
 *
 * @returns
 */
static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    struct audio_device *adev = (struct audio_device *)dev;
    struct str_parms *parms = NULL;
    char value[32] = "";
    /*
     * ret is the result of str_parms_get_str,
     * if no paramter which str_parms_get_str to get, it will return result < 0 always.
     * For example: kvpairs = connect=1024 is coming
     *              str_parms_get_str(parms, AUDIO_PARAMETER_KEY_SCREEN_STATE,value, sizeof(value))
     *              will return result < 0,this means no screen_state in parms
     */
    int ret = 0;

    /*
     * status is the result of one process,
     * For example: kvpairs = screen_state=on is coming,
     *              str_parms_get_str(parms, AUDIO_PARAMETER_KEY_SCREEN_STATE,value, sizeof(value))
     *              will return result >= 0,this means screen is on, we can do something,
     *              if the things we do is correct, we set status = 0, or status < 0 means fail.
     */
    int status = 0;

    ALOGD("%s: kvpairs = %s", __func__, kvpairs);
    parms = str_parms_create_str(kvpairs);
    pthread_mutex_lock(&adev->lock);

    // screen state off/on
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_SCREEN_STATE, // screen_state
                            value, sizeof(value));
    if (ret >= 0) {
        if(strcmp(value,"on") == 0){
            adev->screenOff = false;
        } else if(strcmp(value,"off") == 0){
            adev->screenOff = true;
        }
    }

#ifdef AUDIO_BITSTREAM_REOPEN_HDMI
	if (!hasExtCodec()){
    // hdmi reconnect
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_DEVICE_CONNECT, // hdmi reconnect
                            value, sizeof(value));
    if (ret >= 0) {
        int device = atoi(value);
        if(device == (int)AUDIO_DEVICE_OUT_AUX_DIGITAL){
            struct stream_out *out = adev->outputs[OUTPUT_HDMI_MULTI];
            if((out != NULL) && is_bitstream(out) && (out->device == AUDIO_DEVICE_OUT_AUX_DIGITAL)) {
                ALOGD("%s: hdmi connect when audio stream is output over hdmi, do something,out = %p",__FUNCTION__,out);
                out->snd_reopen = true;
            }
	        }
        }
    }
#endif

    ret = str_parms_get_str(parms, "simcom_voice_call", value, sizeof(value));
    if (ret >= 0) {
        if (!strcmp(value, "start")) {
            if (!adev->simcom_card_available) {
                ALOGE("SIMCOM: start requested but SIMCOM card missing");
                status = -ENODEV;
            } else {
                adev->voice_call_active = true;
                ALOGI("SIMCOM: voice call flag set");
                if (adev->mode == AUDIO_MODE_IN_CALL) {
                    simcom_apply_voice_routes_l(adev);
                }
            }
        } else if (!strcmp(value, "stop")) {
            adev->voice_call_active = false;
            ALOGI("SIMCOM: voice call flag cleared");
            simcom_reset_voice_routes_l(adev);
        } else {
            ALOGW("SIMCOM: unknown simcom_voice_call value: %s", value);
            status = -EINVAL;
        }
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));
    if (ret < 0) {
        ret = str_parms_get_str(parms, "routing", value, sizeof(value));
    }
    if (ret >= 0) {
        audio_devices_t routed_dev = (audio_devices_t)atoi(value);
        if (routed_dev != 0) {
            adev->out_device = routed_dev;
            if (simcom_voice_mode_active(adev)) {
                simcom_apply_voice_routes_l(adev);
            }
        }
    }

    pthread_mutex_unlock(&adev->lock);
    str_parms_destroy(parms);
    return status;
}

/*
 * get support formats for bitstream
 * There is no stand interface in andorid to get the formats can be bistream,
 * so we extend get parameter to report formats
 */
static int get_support_bitstream_formats(struct str_parms *query,
                                    struct str_parms *reply)
{
    int avail = 1024;
    char value[avail];

    struct hdmi_audio_infors hdmi_edid;
    init_hdmi_audio(&hdmi_edid);
    const char* AUDIO_PARAMETER_STREAM_SUP_BITSTREAM_FORMAT = "sup_bitstream_formats";
    if (str_parms_has_key(query, AUDIO_PARAMETER_STREAM_SUP_BITSTREAM_FORMAT)) {
        memset(value,0,avail);

        // get the format can be bistream?
        if(parse_hdmi_audio(&hdmi_edid) >= 0){
            int cursor = 0;
            for(int i = 0; i < ARRAY_SIZE(sSurroundFormat); i++){
                if(is_support_format(&hdmi_edid,sSurroundFormat[i].format)){
                    avail -= cursor;
                    int length = snprintf(value + cursor, avail, "%s%s",
                                   cursor > 0 ? "|" : "",
                                   sSurroundFormat[i].value);
                    if (length < 0 || length >= avail) {
                        break;
                    }
                    cursor += length;
                }
            }
        }

        destory_hdmi_audio(&hdmi_edid);
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_BITSTREAM_FORMAT, value);
        return 0;
    }

    return -1;
}

/**
 * @brief adev_get_parameters
 *
 * @param dev
 * @param keys
 *
 * @returns
 */
static char * adev_get_parameters(const struct audio_hw_device *dev,
                                  const char *keys)
{
    struct audio_device *adev = (struct audio_device *)dev;
    struct str_parms *parms = str_parms_create_str(keys);
    struct str_parms *reply = str_parms_create();
    char *str = NULL;
    ALOGD("%s: keys = %s",__FUNCTION__,keys);
    if (str_parms_has_key(parms, "ec_supported")) {
        str_parms_destroy(parms);
        parms = str_parms_create_str("ec_supported=yes");
        str = str_parms_to_str(parms);
    } else if (get_support_bitstream_formats(parms,reply) == 0) {
        str = str_parms_to_str(reply);
    } else {
        str = strdup("");
    }

    str_parms_destroy(parms);
    str_parms_destroy(reply);

    return str;
}

/**
 * @brief adev_init_check
 *
 * @param dev
 *
 * @returns
 */
static int adev_init_check(const struct audio_hw_device *dev)
{
    return 0;
}

/**
 * @brief adev_set_voice_volume
 *
 * @param dev
 * @param volume
 *
 * @returns
 */
static int adev_set_voice_volume(struct audio_hw_device *dev, float volume)
{
    int ret = 0;
    struct audio_device *adev = (struct audio_device *)dev;
    if(adev->mode == AUDIO_MODE_IN_CALL) {
        if (volume < 0.0) {
            volume = 0.0;
        } else if (volume > 1.0) {
            volume = 1.0;
        }

        const char *mixer_ctl_name = "Speaker Playback Volume";
        ret = route_set_voice_volume(mixer_ctl_name,volume);
    }

    return ret;
}

/**
 * @brief adev_set_master_volume
 *
 * @param dev
 * @param volume
 *
 * @returns
 */
static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    return -ENOSYS;
}

/**
 * @brief adev_set_mode
 *
 * @param dev
 * @param mode
 *
 * @returns
 */
static int adev_set_mode(struct audio_hw_device *dev, audio_mode_t mode)
{
    struct audio_device *adev = (struct audio_device *)dev;
    audio_mode_t previous = adev->mode;
    bool now_call = (mode == AUDIO_MODE_IN_CALL);

    ALOGD("Audio mode changing: %d -> %d, SIMCOM available: %d, call active: %d",
          previous, mode, adev->simcom_card_available, adev->voice_call_active);
    
    // SIMCOM: Detailed logging for mode changes
    if (mode == AUDIO_MODE_IN_CALL) {
        ALOGI("SIMCOM: adev_set_mode: ENTERING IN_CALL mode (previous=%d)", previous);
    } else if (previous == AUDIO_MODE_IN_CALL && mode != AUDIO_MODE_IN_CALL) {
        ALOGI("SIMCOM: adev_set_mode: EXITING IN_CALL mode (new=%d)", mode);
    }
    
    pthread_mutex_lock(&adev->lock);
    adev->mode = mode;

    if (now_call) {
        // SIMCOM: Always reset PCM ready and send AT+CPCMREG=1 when entering call mode
        // This ensures the modem is properly initialized even if voice_call_active was already true
        if (!adev->voice_call_active) {
            ALOGI("SIMCOM: entered call mode, enabling voice flag for fallback path");
            adev->voice_call_active = true;
        } else {
            ALOGI("SIMCOM: already in call mode, resetting PCM ready and re-sending AT+CPCMREG=1");
        }
        simcom_schedule_cpcm_command(adev, true);
        
        // SIMCOM: Ensure sound cards are initialized before applying voice routes
        // This prevents "invalid speaker card -1" error on subsequent calls
        if (adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card < 0) {
            // Force re-scan of sound cards
            ALOGI("SIMCOM: speaker card not initialized (card=%d), forcing re-scan",
                  adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card);
            
            // Manually scan for RT5651 codec (card=2)
            FILE *fp = fopen("/proc/asound/card2/id", "r");
            if (fp) {
                char id[32];
                if (fgets(id, sizeof(id), fp)) {
                    if (strstr(id, "realtekrt5651co")) {
                        adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card = 2;
                        adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].device = 0;
                        ALOGI("SIMCOM: manually initialized speaker card=2 device=0");
                    }
                }
                fclose(fp);
            }
        }
        
        // Apply voice routes if speaker card is now valid
        if (adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].card >= 0) {
            simcom_apply_voice_routes_l(adev);
        } else {
            ALOGW("SIMCOM: speaker card still invalid, voice routes not applied");
        }
    } else if (!now_call && adev->voice_call_active) {
        ALOGI("SIMCOM: exited call mode, clearing voice flag (previous=%d new=%d)",
              previous, mode);
        adev->voice_call_active = false;
        simcom_schedule_cpcm_command(adev, false);
        simcom_reset_voice_routes_l(adev);
    } else {
        ALOGD("SIMCOM: adev_set_mode: no action needed (now_call=%d voice_call_active=%d)", 
              now_call ? 1 : 0, adev->voice_call_active ? 1 : 0);
        if (!now_call) {
            simcom_reset_voice_routes_l(adev);
        }
    }

    pthread_mutex_unlock(&adev->lock);
    return 0;
}

/**
 * @brief adev_set_mic_mute
 *
 * @param dev
 * @param state
 *
 * @returns
 */
static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    struct audio_device *adev = (struct audio_device *)dev;

    adev->mic_mute = state;

    return 0;
}

/**
 * @brief adev_get_mic_mute
 *
 * @param dev
 * @param state
 *
 * @returns
 */
static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    struct audio_device *adev = (struct audio_device *)dev;

    *state = adev->mic_mute;

    return 0;
}

/**
 * @brief adev_get_input_buffer_size
 *
 * @param dev
 * @param config
 *
 * @returns
 */
static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev,
        const struct audio_config *config)
{

    return get_input_buffer_size(config->sample_rate, config->format,
                                 audio_channel_count_from_in_mask(config->channel_mask),
                                 false /* is_low_latency: since we don't know, be conservative */);
}

/**
 * @brief adev_open_input_stream
 *
 * @param dev
 * @param handle
 * @param devices
 * @param config
 * @param stream_in
 * @param flags
 * @param __unused
 * @param __unused
 *
 * @returns
 */
static int adev_open_input_stream(struct audio_hw_device *dev,
                                  audio_io_handle_t handle,
                                  audio_devices_t devices,
                                  struct audio_config *config,
                                  struct audio_stream_in **stream_in,
                                  audio_input_flags_t flags,
                                  const char *address __unused,
                                  audio_source_t source __unused)
{
    struct audio_device *adev = (struct audio_device *)dev;
    struct stream_in *in;
    int ret;

    bool telephony_rx = (devices & AUDIO_DEVICE_IN_TELEPHONY_RX);
    bool call_mode_active = simcom_voice_mode_active(adev);

    ALOGI("SIMCOM: adev_open_input_stream: devices=0x%x telephony_rx=%d call_mode_active=%d flags=%d rate=%d channels=0x%x",
           devices, telephony_rx ? 1 : 0, call_mode_active ? 1 : 0, flags, config->sample_rate, config->channel_mask);
    ALOGD("audio hal adev_open_input_stream devices = 0x%x, flags = %d, config->samplerate = %d,config->channel_mask = %x",
           devices, flags, config->sample_rate,config->channel_mask);

    *stream_in = NULL;
#ifdef ALSA_IN_DEBUG
    in_debug = fopen("/data/debug.pcm","wb");//please touch /data/debug.pcm first
#endif
    /* Respond with a request for mono if a different format is given. */
    //ALOGV("%s:config->channel_mask %d",__FUNCTION__,config->channel_mask);
    if (telephony_rx || call_mode_active) {
        config->sample_rate = 8000;
        config->channel_mask = AUDIO_CHANNEL_IN_MONO;
        config->format = AUDIO_FORMAT_PCM_16_BIT;
    } else if (config->channel_mask != AUDIO_CHANNEL_IN_STEREO) {
        config->channel_mask = AUDIO_CHANNEL_IN_STEREO;
        ALOGE("%s:channel is not support",__FUNCTION__);
        return -EINVAL;
    }

    in = (struct stream_in *)calloc(1, sizeof(struct stream_in));
    if (!in)
        return -ENOMEM;

    /*get default supported channel_mask*/
    memset(in->supported_channel_masks, 0, sizeof(in->supported_channel_masks));
    in->supported_channel_masks[0] = AUDIO_CHANNEL_IN_STEREO;
    in->supported_channel_masks[1] = AUDIO_CHANNEL_IN_MONO;
    /*get default supported sample_rate*/
    memset(in->supported_sample_rates, 0, sizeof(in->supported_sample_rates));
    in->supported_sample_rates[0] = 44100;
    in->supported_sample_rates[1] = 48000;
    in->supported_sample_rates[2] = 8000;

    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;
    in->stream.get_active_microphones = in_get_active_microphones;

    in->dev = adev;
    in->is_simcom_voice = telephony_rx;
    in->bypass_pcm = call_mode_active && !telephony_rx;
    in->simcom_attached = false;
    if (telephony_rx) {
        ALOGI("SIMCOM: Telephony Rx input - dedicated SIMCOM PCM stream (bypass_pcm=false)");
        in->bypass_pcm = false;
    }
    ALOGI("SIMCOM: adev_open_input_stream: is_simcom_voice=%d bypass_pcm=%d standby=%d",
          in->is_simcom_voice ? 1 : 0, in->bypass_pcm ? 1 : 0, in->standby ? 1 : 0);

    in->standby = true;
    in->requested_rate = config->sample_rate;
    in->input_source = AUDIO_SOURCE_DEFAULT;
    /* strip AUDIO_DEVICE_BIT_IN to allow bitwise comparisons */
    in->device = devices & ~AUDIO_DEVICE_BIT_IN;
    in->io_handle = handle;
    in->channel_mask = config->channel_mask;
    if (in->device & AUDIO_DEVICE_IN_HDMI) {
        ALOGD("HDMI-In: use low latency");
        flags |= AUDIO_INPUT_FLAG_FAST;
    }
    in->flags = flags;
    struct pcm_config *pcm_config = flags & AUDIO_INPUT_FLAG_FAST ?
                                            &pcm_config_in_low_latency : &pcm_config_in;
    if (in->is_simcom_voice) {
        pcm_config = &simcom_pcm_config_rx;
    } else if (in->bypass_pcm) {
        pcm_config = &simcom_pcm_config_rx;
    }
#ifdef BT_AP_SCO
    if (/*adev->mode == AUDIO_MODE_IN_COMMUNICATION && */in->device & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET) {
        pcm_config = &pcm_config_in_bt;
    }
#endif

    in->config = pcm_config;

    in->buffer = malloc(pcm_config->period_size * pcm_config->channels
                        * audio_stream_in_frame_size(&in->stream));
#ifdef SPEEX_DENOISE_ENABLE
    in->mSpeexState = NULL;
    in->mSpeexFrameSize = 0;
    in->mSpeexPcmIn = NULL;
#endif

    if (!in->buffer) {
        ret = -ENOMEM;
        goto err_malloc;
    }

    if (in->requested_rate != pcm_config->rate) {
        in->buf_provider.get_next_buffer = get_next_buffer;
        in->buf_provider.release_buffer = release_buffer;

        ALOGD("pcm_config->rate:%d,in->requested_rate:%d,in->channel_mask:%d",
              pcm_config->rate,in->requested_rate,audio_channel_count_from_in_mask(in->channel_mask));
        ret = create_resampler(pcm_config->rate,
                               in->requested_rate,
                               audio_channel_count_from_in_mask(in->channel_mask),
                               RESAMPLER_QUALITY_DEFAULT,
                               &in->buf_provider,
                               &in->resampler);
        if (ret != 0) {
            ret = -EINVAL;
            goto err_resampler;
        }
    }

    if (in->device&AUDIO_DEVICE_IN_HDMI) {
        goto out;
    }

#ifdef AUDIO_3A
    ALOGD("voice process has opened, try to create voice process!");
    adev->voice_api = rk_voiceprocess_create(DEFAULT_PLAYBACK_SAMPLERATE,
                                             DEFAULT_PLAYBACK_CHANNELS,
                                             in->requested_rate,
                                             audio_channel_count_from_in_mask(in->channel_mask));
    if (adev->voice_api == NULL) {
        ALOGE("crate voice process failed!");
    }
#endif

#ifdef SPEEX_DENOISE_ENABLE
    uint32_t size;
    int denoise = 1;
    int noiseSuppress = -24;
    int channel_count = audio_channel_count_from_out_mask(config->channel_mask);

    size = in_get_buffer_size(in);
    in->mSpeexFrameSize = size/(channel_count * sizeof(int16_t));
    ALOGD("in->mSpeexFrameSize:%d in->requested_rate:%d",in->mSpeexFrameSize, in->requested_rate);
    in->mSpeexPcmIn = malloc(sizeof(int16_t)*in->mSpeexFrameSize);
    if(!in->mSpeexPcmIn) {
        ALOGE("speexPcmIn malloc failed");
        goto err_speex_malloc;
    }
    in->mSpeexState = speex_preprocess_state_init(in->mSpeexFrameSize, in->requested_rate);
    if(in->mSpeexState == NULL) {
        ALOGE("speex error");
        goto err_speex_malloc;
    }

    speex_preprocess_ctl(in->mSpeexState, SPEEX_PREPROCESS_SET_DENOISE, &denoise);
    speex_preprocess_ctl(in->mSpeexState, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &noiseSuppress);

#endif

out:
    *stream_in = &in->stream;
    return 0;

err_speex_malloc:
#ifdef SPEEX_DENOISE_ENABLE
    free(in->mSpeexPcmIn);
#endif
err_resampler:
    free(in->buffer);
err_malloc:
    free(in);
    return ret;
}

/**
 * @brief adev_close_input_stream
 *
 * @param dev
 * @param stream
 */
static void adev_close_input_stream(struct audio_hw_device *dev,
                                    struct audio_stream_in *stream)
{
    struct stream_in *in = (struct stream_in *)stream;
    struct audio_device *adev = (struct audio_device *)dev;

    ALOGD("%s",__FUNCTION__);

    in_standby(&stream->common);
    if (in->resampler) {
        release_resampler(in->resampler);
        in->resampler = NULL;
    }

#ifdef ALSA_IN_DEBUG
    fclose(in_debug);
#endif
#ifdef AUDIO_3A
    if (adev->voice_api != NULL) {
        rk_voiceprocess_destory();
        adev->voice_api = NULL;
    }
#endif

#ifdef SPEEX_DENOISE_ENABLE
    if (in->mSpeexState) {
        speex_preprocess_state_destroy(in->mSpeexState);
    }
    if(in->mSpeexPcmIn) {
        free(in->mSpeexPcmIn);
    }
#endif
    free(in->buffer);
    free(stream);
}

/**
 * @brief adev_dump
 *
 * @param device
 * @param fd
 *
 * @returns
 */
static int adev_dump(const audio_hw_device_t *device, int fd)
{
    return 0;
}

/**
 * @brief adev_close
 *
 * @param device
 *
 * @returns
 */
static int adev_close(hw_device_t *device)
{
    struct audio_device *adev = (struct audio_device *)device;

    //audio_route_free(adev->ar);
    route_uninit();
    property_set(SIMCOM_PCM_READY_PROP, "0");
    pthread_mutex_destroy(&adev->simcom_modem_lock);

    free(device);
    return 0;
}

static void adev_open_init(struct audio_device *adev)
{
    ALOGD("%s",__func__);
    int i = 0;
    adev->mic_mute = false;
    adev->screenOff = false;

#ifdef AUDIO_3A
    adev->voice_api = NULL;
#endif

    adev->input_source = AUDIO_SOURCE_DEFAULT;

    for(i =0; i < OUTPUT_TOTAL; i++){
        adev->outputs[i] = NULL;
    }
    set_default_dev_info(adev->dev_out, SND_OUT_SOUND_CARD_MAX, 1);
    set_default_dev_info(adev->dev_in, SND_IN_SOUND_CARD_MAX, 1);
    adev->dev_out[SND_OUT_SOUND_CARD_SPEAKER].id = "SPEAKER";
    adev->dev_out[SND_OUT_SOUND_CARD_HDMI].id = "HDMI";
    adev->dev_out[SND_OUT_SOUND_CARD_SPDIF].id = "SPDIF";
    adev->dev_out[SND_OUT_SOUND_CARD_BT].id = "BT";
    adev->dev_out[SND_OUT_SOUND_CARD_SIMCOM].id = "SIMCOM_OUT";
    adev->dev_in[SND_IN_SOUND_CARD_MIC].id = "MIC";
    adev->dev_in[SND_IN_SOUND_CARD_BT].id = "BT";
    adev->dev_in[SND_IN_SOUND_CARD_SIMCOM].id = "SIMCOM_IN";
    adev->owner[0] = NULL;
    adev->owner[1] = NULL;
    simcom_rx_bus_init(&adev->simcom_rx_bus);

    char value[PROPERTY_VALUE_MAX];
    if (property_get("vendor.audio.period_size", value, NULL) > 0) {
        pcm_config.period_size = atoi(value);
        pcm_config_in.period_size = pcm_config.period_size;
    }
    if (property_get("vendor.audio.in_period_size", value, NULL) > 0)
        pcm_config_in.period_size = atoi(value);

    adev->voice_call_active = false;
    adev->simcom_card_available = false;
    adev->simcom_pcm_card = -1;
    adev->simcom_pcm_device = SIMCOM_PCM_DEVICE;
    adev->simcom_forced_out_device = AUDIO_DEVICE_NONE;
    adev->simcom_voice_playback_route = 0;
    adev->simcom_forced_in_device = AUDIO_DEVICE_NONE;
    adev->simcom_voice_capture_route = 0;
}

/**
 * @brief adev_open
 *
 * @param module
 * @param name
 * @param device
 *
 * @returns
 */
static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
    struct audio_device *adev;
    int ret;

    ALOGD(AUDIO_HAL_VERSION);

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = calloc(1, sizeof(struct audio_device));
    if (!adev)
        return -ENOMEM;

    adev->hw_device.common.tag = HARDWARE_DEVICE_TAG;
    adev->hw_device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
    adev->hw_device.common.module = (struct hw_module_t *) module;
    adev->hw_device.common.close = adev_close;

    adev->hw_device.init_check = adev_init_check;
    adev->hw_device.set_voice_volume = adev_set_voice_volume;
    adev->hw_device.set_master_volume = adev_set_master_volume;
    adev->hw_device.set_mode = adev_set_mode;
    adev->hw_device.set_mic_mute = adev_set_mic_mute;
    adev->hw_device.get_mic_mute = adev_get_mic_mute;
    adev->hw_device.set_parameters = adev_set_parameters;
    adev->hw_device.get_parameters = adev_get_parameters;
    adev->hw_device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->hw_device.open_output_stream = adev_open_output_stream;
    adev->hw_device.close_output_stream = adev_close_output_stream;
    adev->hw_device.open_input_stream = adev_open_input_stream;
    adev->hw_device.close_input_stream = adev_close_input_stream;
    adev->hw_device.dump = adev_dump;
    adev->hw_device.get_microphones = adev_get_microphones;
    //adev->ar = audio_route_init(MIXER_CARD, NULL);
    //route_init();
    /* adev->cur_route_id initial value is 0 and such that first device
     * selection is always applied by select_devices() */
    *device = &adev->hw_device.common;

    pthread_mutex_init(&adev->simcom_modem_lock, NULL);
    adev->simcom_pcm_ready = false;
    property_set(SIMCOM_PCM_READY_PROP, "0");

    adev_open_init(adev);
    adev->simcom_card_available = simcom_detect_card(adev);
    if (!adev->simcom_card_available) {
        ALOGW("SIMCOM audio device not found - voice calls unavailable");
    }
    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AUDIO_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AUDIO_HARDWARE_MODULE_ID,
        .name = "Manta audio HW HAL",
        .author = "The Android Open Source Project",
        .methods = &hal_module_methods,
    },
};
