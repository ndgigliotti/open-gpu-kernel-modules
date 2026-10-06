/*
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "os-interface.h"
#include "nv-linux.h"
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>

#if defined(NV_LINUX_ACPI_EVENTS_SUPPORTED) && IS_ENABLED(CONFIG_POWER_SUPPLY)
static DEFINE_SPINLOCK(nv_power_source_lock);
static DEFINE_MUTEX(nv_power_source_stop_lock);
static NvBool nv_power_source_notifier_registered;
static NvBool nv_power_source_reboot_registered;
static struct workqueue_struct *nv_power_source_wq;
static NvU64 nv_power_source_epoch;
static NvU64 nv_power_source_request_epoch;
static NvU64 nv_power_source_retry_epoch;
static unsigned int nv_power_source_retries;
#define NV_POWER_SOURCE_RETRY_LIMIT 8
static int nv_power_source_status = -ENODEV;
static void nv_power_source_work(struct work_struct *work);
static DECLARE_WORK(nv_power_source_sample_work, nv_power_source_work);
static void nv_power_source_retry_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(nv_power_source_retry, nv_power_source_retry_work);

struct nv_power_source_sample {
    NvBool readable;
    NvBool failed;
};

static int nv_power_source_read_one(struct power_supply *psy, void *data)
{
    struct nv_power_source_sample *sample = data;
    union power_supply_propval value;
    int status;

    // A battery's presence alone does not establish the charger state.
    if (psy->desc->type == POWER_SUPPLY_TYPE_BATTERY)
        return 0;

    status = power_supply_get_property(psy, POWER_SUPPLY_PROP_SCOPE, &value);
    if (status == 0 && value.intval == POWER_SUPPLY_SCOPE_DEVICE)
        return 0;
    if (status != 0 && status != -EINVAL && status != -ENODATA)
    {
        sample->failed = NV_TRUE;
        return 0;
    }

    status = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
    if (status != 0)
    {
        sample->failed = NV_TRUE;
        return 0;
    }
    sample->readable = NV_TRUE;
    return value.intval > 0;
}

#if !defined(NV_POWER_SUPPLY_FOR_EACH_PSY_PRESENT)
static int nv_power_source_read_device(struct device *dev, void *data)
{
    return nv_power_source_read_one(dev_get_drvdata(dev), data);
}
#endif

static int nv_power_source_read(void)
{
    struct nv_power_source_sample sample = { NV_FALSE, NV_FALSE };
    int status;

#if defined(NV_POWER_SUPPLY_FOR_EACH_PSY_PRESENT)
    status = power_supply_for_each_psy(&sample, nv_power_source_read_one);
#elif defined(NV_POWER_SUPPLY_FOR_EACH_DEVICE_PRESENT)
    status = power_supply_for_each_device(&sample, nv_power_source_read_device);
#else
    status = class_for_each_device(power_supply_class, NULL, &sample,
                                  nv_power_source_read_device);
#endif
    if (status > 0)
        return 1;
    if (status < 0 || sample.failed || !sample.readable)
        return -ENODEV;
    return 0;
}

static void nv_power_source_queue_locked(void)
{
    if (nv_power_source_wq != NULL)
        queue_work(nv_power_source_wq, &nv_power_source_sample_work);
}

void nv_power_source_refresh(void)
{
    unsigned long flags;

    spin_lock_irqsave(&nv_power_source_lock, flags);
    nv_power_source_epoch++;
    nv_power_source_request_epoch = nv_power_source_epoch;
    nv_power_source_status = -ENODEV;
    nv_power_source_retries = 0;
    nv_power_source_queue_locked();
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
}

void nv_power_source_refresh_device(nv_state_t *nv)
{
    unsigned long flags;

    spin_lock_irqsave(&nv_power_source_lock, flags);
    if (nv != NULL)
        nv->power_source_valid = NV_FALSE;
    nv_power_source_epoch++;
    nv_power_source_request_epoch = nv_power_source_epoch;
    nv_power_source_status = -ENODEV;
    nv_power_source_retries = 0;
    nv_power_source_queue_locked();
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
}

// Only validated, successful common RM setters call this under API/GPU locks.
void NV_API_CALL nv_power_source_report(nv_state_t *nv, NvU32 source,
                                       NvBool from_provider)
{
    unsigned long flags;

    spin_lock_irqsave(&nv_power_source_lock, flags);
    nv->power_source_value = source;
    nv->power_source_valid = NV_TRUE;
    if (!from_provider)
    {
        // Reject an older sample without immediately undoing this report.
        nv_power_source_epoch++;
        nv_power_source_status = -ENODEV;
    }
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
}

NvBool NV_API_CALL nv_power_source_sample_is_current(nv_state_t *nv,
                                                    NvU32 source, NvU64 epoch)
{
    unsigned long flags;
    NvBool sample_current;

    spin_lock_irqsave(&nv_power_source_lock, flags);
    sample_current = nv_power_source_wq != NULL &&
              nv_power_source_epoch == epoch && nv_power_source_status >= 0 &&
              (!nv->power_source_valid || nv->power_source_value != source);
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
    return sample_current && nv_acpi_uses_power_supply();
}

static void nv_power_source_retry_work(struct work_struct *work)
{
    unsigned long flags;

    spin_lock_irqsave(&nv_power_source_lock, flags);
    if (nv_power_source_retry_epoch == nv_power_source_epoch &&
        nv_power_source_request_epoch == nv_power_source_epoch)
        nv_power_source_queue_locked();
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
}

static void nv_power_source_schedule_retry(NvU64 epoch)
{
    unsigned long flags;
    unsigned int shift;

    spin_lock_irqsave(&nv_power_source_lock, flags);
    if (nv_power_source_wq != NULL && epoch == nv_power_source_epoch &&
        nv_power_source_retries < NV_POWER_SOURCE_RETRY_LIMIT)
    {
        shift = nv_power_source_retries < 4 ? nv_power_source_retries : 4;
        nv_power_source_retry_epoch = epoch;
        nv_power_source_retries++;
        mod_delayed_work(nv_power_source_wq, &nv_power_source_retry,
                         msecs_to_jiffies(100U << shift));
    }
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
}

NV_STATUS nv_power_source_get(NvU32 *ac_plugged)
{
    unsigned long flags;
    int status;

    spin_lock_irqsave(&nv_power_source_lock, flags);
    status = nv_power_source_wq == NULL ? -ENODEV : nv_power_source_status;
    // Re-sample even a valid cache: removal need not emit a change event.
    nv_power_source_retries = 0;
    nv_power_source_request_epoch = nv_power_source_epoch;
    nv_power_source_queue_locked();
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
    if (status < 0)
        return NV_ERR_NOT_SUPPORTED;
    *ac_plugged = status > 0;
    return NV_OK;
}

#if defined(NV_PM_RUNTIME_AVAILABLE)
static int nv_power_source_pin_active(struct device *dev)
{
#if defined(NV_PM_RUNTIME_GET_IF_ACTIVE_PRESENT)
    return pm_runtime_get_if_active(dev);
#elif defined(NV_PM_RUNTIME_GET_IF_ACTIVE_HAS_USAGE_COUNT)
    return pm_runtime_get_if_active(dev, NV_TRUE);
#else
    unsigned long flags;
    int status = 0;

    // Older kernels lack get_if_active. Serialize the non-waking reference
    // with runtime-state transitions, including the zero-PM-count case.
    spin_lock_irqsave(&dev->power.lock, flags);
    if (dev->power.disable_depth == 0 &&
        dev->power.runtime_status == RPM_ACTIVE)
    {
        pm_runtime_get_noresume(dev);
        status = 1;
    }
    spin_unlock_irqrestore(&dev->power.lock, flags);
    return status;
#endif
}
#endif

static NvBool nv_power_source_dispatch(NvU32 source, NvU64 epoch)
{
    nv_linux_state_t *nvl;
    nvidia_stack_t *sp = NULL;
    nv_state_t *nv;
    NV_STATUS status;
    NvBool retry = NV_FALSE;
    DECLARE_BITMAP(visited, NV_MINOR_DEVICE_NUMBER_REGULAR_MAX + 1);

#if defined(NV_VGPU_KVM_BUILD)
    // Late vGPU probe failure tears down a listed device without ldata_lock.
    // Cache queries remain available, but live dispatch needs that lifetime lock.
    return NV_FALSE;
#endif

    // Waiting here can prevent the freezable worker from reaching quiescence.
    if (!down_read_trylock(&nv_system_pm_lock))
        return NV_TRUE;
    if (nv_kmem_cache_alloc_stack(&sp) != 0)
    {
        retry = NV_TRUE;
        goto pm_unlock;
    }

    bitmap_zero(visited, NV_MINOR_DEVICE_NUMBER_REGULAR_MAX + 1);
    for (;;)
    {
        LOCK_NV_LINUX_DEVICES();
        for (nvl = nv_linux_devices; nvl != NULL; nvl = nvl->next)
        {
            // Platform-display teardown can precede its ldata_lock.
            // Only PCI removal serializes teardown with our lifetime lock.
            if (nvl->pci_dev == NULL)
                continue;
            if (test_bit(nvl->minor_num, visited))
                continue;
            set_bit(nvl->minor_num, visited);
            if (down_trylock(&nvl->ldata_lock) == 0)
                break;
            retry = NV_TRUE;
        }
        UNLOCK_NV_LINUX_DEVICES();
        if (nvl == NULL)
            break;

        // PCI removal needs ldata_lock before unlink/teardown.
        // Do not retain list pointers after releasing this lifetime lock.
        nv = NV_STATE_PTR(nvl);
        if ((nv->flags & NV_FLAG_INITIALIZED) == 0 ||
            (nv->flags & NV_FLAG_SUSPENDED) != 0 ||
            atomic64_read(&nvl->usage_count) == 0)
            goto device_unlock;
#if defined(NV_PM_RUNTIME_AVAILABLE)
        // Client use is separate from the fine-RTD3 runtime-PM counter.
        if (nv_power_source_pin_active(nvl->dev) <= 0)
        {
            retry = NV_TRUE;
            goto device_unlock;
        }
#endif
        status = rm_power_source_update(sp, nv, source, epoch);
#if defined(NV_PM_RUNTIME_AVAILABLE)
        pm_runtime_put(nvl->dev);
#endif
        if (status != NV_OK)
        {
            retry = NV_TRUE;
            NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
                          "power-source refresh failed: 0x%x\n", status);
        }
device_unlock:
        up(&nvl->ldata_lock);
    }
    nv_kmem_cache_free_stack(sp);
pm_unlock:
    up_read(&nv_system_pm_lock);
    return retry;
}

static void nv_power_source_work(struct work_struct *work)
{
    unsigned long flags;
    NvU64 epoch;
    int status;

    spin_lock_irqsave(&nv_power_source_lock, flags);
    if (nv_power_source_wq == NULL ||
        nv_power_source_request_epoch != nv_power_source_epoch)
    {
        spin_unlock_irqrestore(&nv_power_source_lock, flags);
        return;
    }
    epoch = nv_power_source_request_epoch;
    spin_unlock_irqrestore(&nv_power_source_lock, flags);

    if (!nv_acpi_uses_power_supply())
        return;

    // Provider I/O precedes every GPU reference and RM/API/GPU lock.
    status = nv_power_source_read();
    spin_lock_irqsave(&nv_power_source_lock, flags);
    if (nv_power_source_wq == NULL || epoch != nv_power_source_epoch)
    {
        spin_unlock_irqrestore(&nv_power_source_lock, flags);
        return;
    }
    nv_power_source_status = status;
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
    if (status < 0)
        nv_power_source_schedule_retry(epoch);
    else if (nv_acpi_uses_power_supply() &&
             nv_power_source_dispatch(!status, epoch))
        nv_power_source_schedule_retry(epoch);
}

static int nv_power_source_changed(struct notifier_block *nb,
                                   unsigned long event, void *data)
{
    struct power_supply *psy = data;

    if (event == PSY_EVENT_PROP_CHANGED &&
        (psy == NULL || psy->desc->type != POWER_SUPPLY_TYPE_BATTERY))
        nv_power_source_refresh();
    return NOTIFY_DONE;
}

static struct notifier_block nv_power_source_nb = {
    .notifier_call = nv_power_source_changed,
};

static void nv_power_source_stop(void)
{
    unsigned long flags;
    struct workqueue_struct *wq;

    mutex_lock(&nv_power_source_stop_lock);
    spin_lock_irqsave(&nv_power_source_lock, flags);
    wq = nv_power_source_wq;
    nv_power_source_wq = NULL;
    nv_power_source_status = -ENODEV;
    nv_power_source_epoch++;
    spin_unlock_irqrestore(&nv_power_source_lock, flags);
    // No RM/device lock is held while in-flight sampling/dispatch drains.
    if (wq != NULL)
    {
        // Pending work cannot execute while hibernation keeps this queue frozen.
        cancel_delayed_work_sync(&nv_power_source_retry);
        cancel_work_sync(&nv_power_source_sample_work);
        destroy_workqueue(wq);
    }
    mutex_unlock(&nv_power_source_stop_lock);
}

static int nv_power_source_reboot(struct notifier_block *nb,
                                  unsigned long event, void *data)
{
    nv_power_source_stop();
    return NOTIFY_DONE;
}

static struct notifier_block nv_power_source_reboot_nb = {
    .notifier_call = nv_power_source_reboot,
    .priority = INT_MAX,
};

int nv_power_source_init(void)
{
    int status;

    nv_power_source_wq = alloc_ordered_workqueue("nvidia_power_source",
                                               WQ_FREEZABLE);
    if (nv_power_source_wq == NULL)
        return -ENOMEM;
    status = power_supply_reg_notifier(&nv_power_source_nb);
    if (status != 0)
        goto stop;
    nv_power_source_notifier_registered = NV_TRUE;
    status = register_reboot_notifier(&nv_power_source_reboot_nb);
    if (status != 0)
        goto unregister_supply;
    nv_power_source_reboot_registered = NV_TRUE;
    nv_power_source_refresh();
    return 0;

unregister_supply:
    power_supply_unreg_notifier(&nv_power_source_nb);
    nv_power_source_notifier_registered = NV_FALSE;
stop:
    nv_power_source_stop();
    return status;
}

void nv_power_source_exit(void)
{
    if (nv_power_source_reboot_registered)
    {
        unregister_reboot_notifier(&nv_power_source_reboot_nb);
        nv_power_source_reboot_registered = NV_FALSE;
    }
    if (nv_power_source_notifier_registered)
    {
        power_supply_unreg_notifier(&nv_power_source_nb);
        nv_power_source_notifier_registered = NV_FALSE;
    }
    nv_power_source_stop();
}
#else
int nv_power_source_init(void) { return 0; }
void nv_power_source_exit(void) { }
void nv_power_source_refresh(void) { }
void nv_power_source_refresh_device(nv_state_t *nv) { }
void NV_API_CALL nv_power_source_report(nv_state_t *nv, NvU32 source,
                                       NvBool from_provider) { }
NvBool NV_API_CALL nv_power_source_sample_is_current(nv_state_t *nv,
                                                    NvU32 source, NvU64 epoch)
{
    return NV_FALSE;
}
NV_STATUS nv_power_source_get(NvU32 *ac_plugged)
{
    return NV_ERR_NOT_SUPPORTED;
}
#endif
