// SPDX-License-Identifier: GPL-2.0
/* Opt-in persistent C2H stream reception. The on-wire packet ABI is unchanged. */
#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mm.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include "libxdma.h"
#include "xdma_c2h_pool.h"
#include "c2h_pool_math.h"

#define C2H_POOL_DEFAULT_SLOTS 128U
#define C2H_POOL_MAX_SLOTS 1024U
#define C2H_POOL_MAX_PACKET (64U * 1024U)

extern unsigned int c2h_timeout_ms;

struct xdma_c2h_bank {
	void *payload;
	dma_addr_t payload_dma;
	size_t payload_bytes;
	struct xdma_desc *desc;
	dma_addr_t desc_dma;
	size_t desc_bytes;
	struct xdma_result *result;
	dma_addr_t result_dma;
	size_t result_bytes;
	struct xdma_poll_wb *wb;
	dma_addr_t wb_dma;
	u32 consumed;
};

struct xdma_c2h_pool {
	struct xdma_c2h_bank bank[2];
	struct xdma_c2h_pool_stats stats;
	struct xdma_c2h_pool_diagnostics diagnostics;
	u32 flags;
	u32 bank_count;
	u32 read_bank;
	u32 active_bank;
	u32 wb_stride;
	atomic_t mappings;
	bool launched;
	bool quarantine_pin;
	int error;
};

/* All entry points run under XENGINE_BUSY_BIT, including teardown. */
static int c2h_pool_quiesce(struct xdma_engine *engine)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	u32 status;
	int rv;

	iowrite32(XDMA_CTRL_RUN_STOP, &engine->regs->control_w1c);
	/* The register read flushes the posted stop before polling BUSY. */
	ioread32(&engine->regs->control);
	rv = read_poll_timeout(ioread32, status, !(status & XDMA_STAT_BUSY),
			       10, 5000000, false, &engine->regs->status);
	if (rv) {
		pool->error = -ETIMEDOUT;
		/* Never free a possible DMA target or permit module unload here. */
		if (!pool->quarantine_pin) {
			__module_get(THIS_MODULE);
			pool->quarantine_pin = true;
		}
		pr_err("%s did not become idle; retaining C2H DMA memory and pinning module\n",
		       engine->name);
		return rv;
	}
	dma_rmb();
	engine->running = 0;
	return 0;
}

static void c2h_pool_free(struct xdma_engine *engine,
			  struct xdma_c2h_pool *pool)
{
	struct device *dev = &engine->xdev->pdev->dev;
	u32 i;

	for (i = 0; i < pool->bank_count; ++i) {
		struct xdma_c2h_bank *bank = &pool->bank[i];

		if (bank->wb)
			dma_free_coherent(dev, PAGE_SIZE, bank->wb, bank->wb_dma);
		if (bank->result)
			dma_free_coherent(dev, bank->result_bytes,
					  bank->result, bank->result_dma);
		if (bank->desc)
			dma_free_coherent(dev, bank->desc_bytes, bank->desc, bank->desc_dma);
		if (bank->payload)
			dma_free_coherent(dev, bank->payload_bytes,
					  bank->payload, bank->payload_dma);
	}
	if (pool->quarantine_pin)
		module_put(THIS_MODULE);
	kfree(pool);
}

static int c2h_pool_alloc_bank(struct xdma_engine *engine,
		struct xdma_c2h_pool *pool, struct xdma_c2h_bank *bank)
{
	struct device *dev = &engine->xdev->pdev->dev;
	u32 i, slots = pool->stats.slots;
	u32 packet_bytes = pool->stats.packet_bytes;

	bank->consumed = slots;
	bank->payload_bytes = PAGE_ALIGN((size_t)slots * packet_bytes);
	bank->desc_bytes = PAGE_ALIGN((size_t)slots * sizeof(*bank->desc));
	bank->result_bytes = PAGE_ALIGN((size_t)slots * sizeof(*bank->result));
	bank->payload = dma_alloc_coherent(dev, bank->payload_bytes,
					   &bank->payload_dma, GFP_KERNEL);
	bank->desc = dma_alloc_coherent(dev, bank->desc_bytes, &bank->desc_dma,
					GFP_KERNEL);
	bank->result = dma_alloc_coherent(dev, bank->result_bytes,
					  &bank->result_dma, GFP_KERNEL);
	bank->wb = dma_alloc_coherent(dev, PAGE_SIZE, &bank->wb_dma, GFP_KERNEL);
	if (!bank->payload || !bank->desc || !bank->result || !bank->wb)
		return -ENOMEM;
	if ((bank->desc_dma & (XDMA_PAGE_SIZE - 1)) ||
	    (engine->addr_align && bank->payload_dma % engine->addr_align))
		return -EINVAL;
	/* All mmap-visible storage, including page padding, starts zeroed. */
	memset(bank->payload, 0, bank->payload_bytes);
	memset(bank->result, 0, bank->result_bytes);
	memset(bank->wb, 0, PAGE_SIZE);
	memset(bank->desc, 0, bank->desc_bytes);
	for (i = 0; i < slots; ++i) {
		struct xdma_desc *desc = &bank->desc[i];
		dma_addr_t dst = bank->payload_dma + (size_t)i * packet_bytes;
		dma_addr_t result = bank->result_dma + i * sizeof(*bank->result);
		u32 control = DESC_MAGIC;

		/* This only coalesces completed-descriptor notifications. Stream
		 * result WB remains enabled and has one actual length per packet. */
		if (xdma_c2h_pool_wb_notify(i, slots, pool->wb_stride))
			control |= XDMA_DESC_COMPLETED;
		desc->bytes = cpu_to_le32(packet_bytes);
		desc->src_addr_lo = cpu_to_le32(lower_32_bits(result));
		desc->src_addr_hi = cpu_to_le32(upper_32_bits(result));
		desc->dst_addr_lo = cpu_to_le32(lower_32_bits(dst));
		desc->dst_addr_hi = cpu_to_le32(upper_32_bits(dst));
		if (i + 1 < slots) {
			dma_addr_t next = bank->desc_dma + (i + 1) * sizeof(*desc);

			desc->next_lo = cpu_to_le32(lower_32_bits(next));
			desc->next_hi = cpu_to_le32(upper_32_bits(next));
			if (pool->flags & XDMA_C2H_POOL_ADJACENT)
				control |= xdma_c2h_pool_adjacent(bank->desc_dma,
					i + 1, slots, engine->xdev->max_read_request_size)
					<< DESC_ADJ_SHIFT;
		} else {
			control |= XDMA_DESC_STOPPED;
		}
		desc->control = cpu_to_le32(control);
	}
	return 0;
}

int xdma_c2h_pool_setup(struct xdma_engine *engine, unsigned long arg)
{
#ifndef XDMA_POLL_MODE
	return -EOPNOTSUPP;
#else
	struct xdma_c2h_pool_config config;
	struct xdma_c2h_pool *pool;
	struct sgdma_common_regs *common;
	u32 i, slots, stride;
	int rv;

	BUILD_BUG_ON(sizeof(struct xdma_c2h_pool_layout) != 40);
	BUILD_BUG_ON(sizeof(struct xdma_c2h_pool_diagnostics) != 40);
	BUILD_BUG_ON(sizeof(struct xdma_c2h_pool_advance) != 16);
	BUILD_BUG_ON(sizeof(struct xdma_desc) != 32);
	BUILD_BUG_ON(sizeof(struct xdma_result) != 32);
	if (engine->dir != DMA_FROM_DEVICE || !engine->streaming)
		return -EOPNOTSUPP;
	if (engine->c2h_pool)
		return -EBUSY;
	if (copy_from_user(&config, (void __user *)arg, sizeof(config)))
		return -EFAULT;
#ifndef CONFIG_X86
	if (config.flags & XDMA_C2H_POOL_MMAP)
		return -EOPNOTSUPP;
#endif
	slots = config.slots ? config.slots : C2H_POOL_DEFAULT_SLOTS;
	stride = (config.flags & XDMA_C2H_POOL_WB_STRIDE_MASK) >>
		 XDMA_C2H_POOL_WB_STRIDE_SHIFT;
	if (!stride)
		stride = 1;
	if (config.version != XDMA_C2H_POOL_ABI ||
	    (config.flags & ~(XDMA_C2H_POOL_WAIT_BATCH | XDMA_C2H_POOL_ADJACENT |
		XDMA_C2H_POOL_MMAP | XDMA_C2H_POOL_DOUBLE_BUFFER |
		XDMA_C2H_POOL_WB_STRIDE_MASK)) ||
	    ((config.flags & XDMA_C2H_POOL_MMAP) &&
	     (config.flags & XDMA_C2H_POOL_DOUBLE_BUFFER)) ||
	    stride > 128 || (stride & (stride - 1)) ||
	    config.packet_bytes < 64 || config.packet_bytes > C2H_POOL_MAX_PACKET ||
	    config.packet_bytes % 64 || slots > C2H_POOL_MAX_SLOTS ||
	    (engine->len_granularity && config.packet_bytes % engine->len_granularity) ||
	    (engine->addr_align && config.packet_bytes % engine->addr_align))
		return -EINVAL;
	if (ioread32(&engine->regs->status) & XDMA_STAT_BUSY)
		return -EBUSY;
	common = (struct sgdma_common_regs *)(engine->xdev->bar[
		engine->xdev->config_bar_idx] + 6 * TARGET_SPACING);
	if (ioread32(&common->credit_mode_enable) & (1U << (16 + engine->channel)))
		return -EOPNOTSUPP;

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return -ENOMEM;
	pool->stats.packet_bytes = config.packet_bytes;
	pool->stats.slots = slots;
	pool->flags = config.flags;
	pool->wb_stride = stride;
	pool->bank_count = (config.flags & XDMA_C2H_POOL_DOUBLE_BUFFER) ? 2 : 1;
	pool->diagnostics.version = XDMA_C2H_POOL_ABI;
	pool->diagnostics.banks = pool->bank_count;
	pool->diagnostics.wb_stride = pool->wb_stride;
	atomic_set(&pool->mappings, 0);
	for (i = 0; i < pool->bank_count; ++i) {
		rv = c2h_pool_alloc_bank(engine, pool, &pool->bank[i]);
		if (rv) {
			c2h_pool_free(engine, pool);
			return rv;
		}
	}
	/* No DMA runs until the first read or explicit mmap RESTART. */
	engine->c2h_pool = pool;
	return 0;
#endif
}

/* Read a bank-specific count. Its WB stays stable while another bank runs. */
static int c2h_pool_completed(struct xdma_c2h_pool *pool,
			     struct xdma_c2h_bank *bank)
{
	u32 wb = le32_to_cpu(READ_ONCE(bank->wb->completed_desc_count));
	u32 completed = wb & WB_COUNT_MASK;

	if ((wb & WB_ERR_MASK) || completed > pool->stats.slots ||
	    completed < bank->consumed)
		return -EIO;
	dma_rmb();
	return completed;
}

static int c2h_pool_launch(struct xdma_engine *engine, u32 index)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	struct xdma_c2h_bank *bank = &pool->bank[index];
	int completed = 0, rv;

	if (pool->launched) {
		completed = c2h_pool_completed(pool, &pool->bank[pool->active_bank]);
		if (completed < 0)
			return completed;
	}
	/* No bank may be overwritten while it has unconsumed packets. The
	 * hardware must also finish the previous finite chain before launch. */
	if (!xdma_c2h_pool_can_launch(pool->stats.slots, bank->consumed,
				    pool->launched, completed))
		return -EBUSY;
	if (pool->launched) {
		rv = c2h_pool_quiesce(engine);
		if (rv)
			return rv;
	}
	memset(bank->result, 0, bank->result_bytes);
	WRITE_ONCE(bank->wb->completed_desc_count, cpu_to_le32(0));
	dma_wmb();
	ioread32(&engine->regs->status_rc);
	iowrite32(lower_32_bits(bank->wb_dma), &engine->regs->poll_mode_wb_lo);
	iowrite32(upper_32_bits(bank->wb_dma), &engine->regs->poll_mode_wb_hi);
	/* Bit 27 DISABLES stream result WB; enable it only for this path. */
	iowrite32(XDMA_CTRL_STM_MODE_WB, &engine->regs->control_w1c);
	iowrite32(lower_32_bits(bank->desc_dma), &engine->sgdma_regs->first_desc_lo);
	iowrite32(upper_32_bits(bank->desc_dma), &engine->sgdma_regs->first_desc_hi);
	iowrite32((pool->flags & XDMA_C2H_POOL_ADJACENT) ?
		  xdma_c2h_pool_adjacent(bank->desc_dma, 0, pool->stats.slots,
				 engine->xdev->max_read_request_size) : 0,
		  &engine->sgdma_regs->first_desc_adjacent);
	bank->consumed = 0;
	pool->active_bank = index;
	pool->launched = true;
	pool->stats.launches++;
	engine->running = 1;
	iowrite32(XDMA_CTRL_RUN_STOP, &engine->regs->control_w1s);
	return 0;
}

/* Validate actual lengths, independently of the requested descriptor size. */
static int c2h_pool_validate(struct xdma_engine *engine,
		struct xdma_c2h_bank *bank, u32 packets)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	u32 i;

	for (i = 0; i < packets; ++i) {
		struct xdma_result *result = &bank->result[bank->consumed + i];
		u32 status = le32_to_cpu(READ_ONCE(result->status));
		u32 length;

		if (!status)
			break;
		dma_rmb();
		length = le32_to_cpu(READ_ONCE(result->length));
		if ((status >> 16) != C2H_WB || !(status & RX_STATUS_EOP) ||
		    length != pool->stats.packet_bytes) {
			pr_err("%s invalid C2H result %u: status=%08x length=%u expected=%u\n",
			       engine->name, bank->consumed + i, status, length,
			       pool->stats.packet_bytes);
			return -EPROTO;
		}
	}
	return i;
}

/* A completed-descriptor WB gates access to every result and payload. */
static int c2h_pool_ready(struct xdma_engine *engine, u32 max_packets)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	struct xdma_c2h_bank *bank = &pool->bank[pool->read_bank];
	u32 available;
	int completed = c2h_pool_completed(pool, bank);
	int validated;

	if (completed < 0)
		return completed;
	if ((pool->flags & XDMA_C2H_POOL_WAIT_BATCH) &&
	    completed - bank->consumed <
		min(max_packets, pool->stats.slots - bank->consumed))
		return 0;
	available = min(max_packets, (u32)completed - bank->consumed);
	validated = c2h_pool_validate(engine, bank, available);
	if (validated < 0)
		return validated;
	if ((pool->flags & XDMA_C2H_POOL_WAIT_BATCH) && validated < available)
		return 0;
	return validated;
}

/* Start a free spare bank before copying the completed read bank. The two
 * finite chains never link to each other, so hardware cannot wrap or reuse
 * a slot until this driver has observed its userspace consumption. */
static int c2h_pool_prefetch(struct xdma_engine *engine)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	u32 spare = pool->read_bank ^ 1U;
	int completed, target;

	if (pool->bank_count != 2 || pool->active_bank != pool->read_bank)
		return 0;
	completed = c2h_pool_completed(pool, &pool->bank[pool->active_bank]);
	if (completed < 0)
		return completed;
	target = xdma_c2h_pool_prefetch_bank(pool->bank_count, pool->read_bank,
		pool->active_bank, pool->stats.slots, completed,
		pool->bank[spare].consumed);
	if (target < 0)
		return 0;
	completed = c2h_pool_launch(engine, target);
	if (!completed) {
		pool->diagnostics.prefetch_launches++;
		pool->diagnostics.prefetch_unconsumed_packets +=
			pool->stats.slots - pool->bank[pool->read_bank].consumed;
	}
	return completed;
}

ssize_t xdma_c2h_pool_read(struct xdma_engine *engine, char __user *buf,
			 size_t count)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	struct xdma_c2h_bank *bank;
	unsigned long deadline;
	u32 max_packets, timeout;
	int ready, rv;
	size_t bytes;

	if (!pool)
		return -EINVAL;
	if (pool->flags & XDMA_C2H_POOL_MMAP)
		return -EOPNOTSUPP;
	if (count < pool->stats.packet_bytes || count % pool->stats.packet_bytes)
		return -EINVAL;
	pool->stats.reads++;
	if (pool->error)
		return pool->error;
	max_packets = min_t(size_t, count / pool->stats.packet_bytes, pool->stats.slots);
	bank = &pool->bank[pool->read_bank];
	if (bank->consumed == pool->stats.slots) {
		if (pool->bank_count == 2 && pool->launched) {
			pool->read_bank ^= 1U;
			pool->diagnostics.bank_switches++;
		}
		bank = &pool->bank[pool->read_bank];
		if (bank->consumed == pool->stats.slots) {
			rv = c2h_pool_launch(engine, pool->read_bank);
			if (rv)
				goto fail;
		}
	}
	timeout = READ_ONCE(c2h_timeout_ms);
	deadline = jiffies + msecs_to_jiffies(timeout);
	for (;;) {
		ready = c2h_pool_ready(engine, max_packets);
		if (ready > 0)
			break;
		if (ready < 0) {
			rv = ready;
			goto fail;
		}
		if (xdma_device_test_offline(engine->xdev)) {
			rv = -ENODEV;
			goto fail;
		}
		/* Leave the live chain for STOP/close after receiver cancellation. */
		if (signal_pending(current))
			return -EINTR;
		if (timeout && time_after_eq(jiffies, deadline)) {
			rv = -ETIMEDOUT;
			goto fail;
		}
		cpu_relax();
		cond_resched();
	}
	rv = c2h_pool_prefetch(engine);
	if (rv)
		goto fail;
	bytes = (size_t)ready * pool->stats.packet_bytes;
	if (copy_to_user(buf, (char *)bank->payload +
			 (size_t)bank->consumed * pool->stats.packet_bytes, bytes)) {
		pool->stats.errors++;
		return -EFAULT;
	}
	bank->consumed += ready;
	pool->stats.packets += ready;
	pool->stats.bytes += bytes;
	if (bytes < count)
		pool->stats.partial_reads++;
	return bytes;
fail:
	pool->stats.errors++;
	pool->error = rv;
	c2h_pool_quiesce(engine);
	return rv;
}

int xdma_c2h_pool_stats(struct xdma_engine *engine, unsigned long arg)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;

	if (!pool)
		return -ENODATA;
	return copy_to_user((void __user *)arg, &pool->stats,
			    sizeof(pool->stats)) ? -EFAULT : 0;
}

int xdma_c2h_pool_diagnostics(struct xdma_engine *engine, unsigned long arg)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;

	if (!pool)
		return -ENODATA;
	return copy_to_user((void __user *)arg, &pool->diagnostics,
			    sizeof(pool->diagnostics)) ? -EFAULT : 0;
}

int xdma_c2h_pool_layout(struct xdma_engine *engine, unsigned long arg)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	struct xdma_c2h_pool_layout layout = { 0 };

	if (!pool || !(pool->flags & XDMA_C2H_POOL_MMAP))
		return -EOPNOTSUPP;
	layout.version = XDMA_C2H_POOL_ABI;
	layout.packet_bytes = pool->stats.packet_bytes;
	layout.slots = pool->stats.slots;
	layout.result_stride = sizeof(*pool->bank[0].result);
	layout.payload_bytes = pool->bank[0].payload_bytes;
	layout.result_bytes = pool->bank[0].result_bytes;
	layout.wb_bytes = PAGE_SIZE;
	return copy_to_user((void __user *)arg, &layout, sizeof(layout)) ? -EFAULT : 0;
}

int xdma_c2h_pool_advance(struct xdma_engine *engine, unsigned long arg)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	struct xdma_c2h_pool_advance advance;
	u32 i, wb, completed;
	int rv;

	if (!pool || !(pool->flags & XDMA_C2H_POOL_MMAP))
		return -EOPNOTSUPP;
	if (copy_from_user(&advance, (void __user *)arg, sizeof(advance)))
		return -EFAULT;
	if (advance.version != XDMA_C2H_POOL_ABI || advance.reserved ||
	    (advance.flags & ~XDMA_C2H_POOL_ADVANCE_RESTART) ||
	    !xdma_c2h_pool_ack_valid(pool->bank[0].consumed, pool->stats.slots,
		advance.packets, advance.flags & XDMA_C2H_POOL_ADVANCE_RESTART))
		return -EINVAL;
	if (pool->error)
		return pool->error;
	/* Before the first launch no hardware completion count exists. */
	if (!pool->launched) {
		if (advance.packets || !(advance.flags & XDMA_C2H_POOL_ADVANCE_RESTART))
			return -EINVAL;
		return c2h_pool_launch(engine, 0);
	}
	wb = le32_to_cpu(READ_ONCE(pool->bank[0].wb->completed_desc_count));
	completed = wb & WB_COUNT_MASK;
	if ((wb & WB_ERR_MASK) || completed > pool->stats.slots ||
	    completed < pool->bank[0].consumed) {
		rv = -EIO;
		goto fail;
	}
	if (advance.packets > completed - pool->bank[0].consumed)
		return -EAGAIN;
	dma_rmb();
	for (i = 0; i < advance.packets; ++i) {
		struct xdma_result *result = &pool->bank[0].result[pool->bank[0].consumed + i];
		u32 status = le32_to_cpu(READ_ONCE(result->status));
		u32 length = le32_to_cpu(READ_ONCE(result->length));

		if ((status >> 16) != C2H_WB || !(status & RX_STATUS_EOP) ||
		    length != pool->stats.packet_bytes) {
			pr_err("%s invalid acknowledged C2H result %u: status=%08x length=%u\n",
			       engine->name, pool->bank[0].consumed + i, status, length);
			rv = -EPROTO;
			goto fail;
		}
	}
	/* The caller acknowledges only after dispatch/copying has finished.
	 * A whole-chain ACK is the only permission to overwrite a DMA slot. */
	pool->bank[0].consumed += advance.packets;
	pool->stats.packets += advance.packets;
	pool->stats.bytes += (u64)advance.packets * pool->stats.packet_bytes;
	if (advance.flags & XDMA_C2H_POOL_ADVANCE_RESTART) {
		rv = c2h_pool_launch(engine, 0);
		if (rv)
			goto fail;
	}
	return 0;
fail:
	pool->stats.errors++;
	pool->error = rv;
	c2h_pool_quiesce(engine);
	return rv;
}

static void c2h_pool_vma_open(struct vm_area_struct *vma)
{
	struct xdma_c2h_pool *pool = vma->vm_private_data;

	atomic_inc(&pool->mappings);
}

static void c2h_pool_vma_close(struct vm_area_struct *vma)
{
	struct xdma_c2h_pool *pool = vma->vm_private_data;

	atomic_dec(&pool->mappings);
}

static const struct vm_operations_struct c2h_pool_vm_ops = {
	.open = c2h_pool_vma_open,
	.close = c2h_pool_vma_close,
};

int xdma_c2h_pool_mmap(struct xdma_engine *engine, struct vm_area_struct *vma)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	struct device *dev = &engine->xdev->pdev->dev;
	void *address;
	dma_addr_t dma_address;
	size_t bytes;
	int rv;

	if (!pool || !(pool->flags & XDMA_C2H_POOL_MMAP))
		return -EOPNOTSUPP;
	if (pool->error)
		return pool->error;
	if (vma->vm_flags & VM_WRITE)
		return -EACCES;
	if (!(vma->vm_flags & VM_MAYSHARE))
		return -EINVAL;
	switch (vma->vm_pgoff) {
	case XDMA_C2H_POOL_MAP_PAYLOAD:
		address = pool->bank[0].payload;
		dma_address = pool->bank[0].payload_dma;
		bytes = pool->bank[0].payload_bytes;
		break;
	case XDMA_C2H_POOL_MAP_RESULTS:
		address = pool->bank[0].result;
		dma_address = pool->bank[0].result_dma;
		bytes = pool->bank[0].result_bytes;
		break;
	case XDMA_C2H_POOL_MAP_WB:
		address = pool->bank[0].wb;
		dma_address = pool->bank[0].wb_dma;
		bytes = PAGE_SIZE;
		break;
	default:
		return -EINVAL;
	}
	if (vma->vm_end - vma->vm_start != bytes)
		return -EINVAL;
#if LINUX_VERSION_CHECK(6, 3, 0)
	vm_flags_clear(vma, VM_MAYWRITE);
	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
#else
	vma->vm_flags &= ~VM_MAYWRITE;
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
#endif
	/* The public offset selects an allocation, not a page within it. */
	vma->vm_pgoff = 0;
	rv = dma_mmap_coherent(dev, vma, address, dma_address, bytes);
	if (rv)
		return rv;
	vma->vm_ops = &c2h_pool_vm_ops;
	vma->vm_private_data = pool;
	/* mmap does not call vm_ops->open for this initial VMA. Fork and
	 * VMA splits do; their file references also pin this module. */
	c2h_pool_vma_open(vma);
	return 0;
}

int xdma_c2h_pool_stop(struct xdma_engine *engine)
{
	struct xdma_c2h_pool *pool = engine->c2h_pool;
	int rv;

	if (!pool)
		return 0;
	if (atomic_read(&pool->mappings))
		return -EBUSY;
	rv = c2h_pool_quiesce(engine);
	if (rv)
		return rv;
	/* Restore the original legacy engine writeback configuration only
	 * after no DMA target in this pool can be used by the hardware. */
	iowrite32(XDMA_CTRL_STM_MODE_WB, &engine->regs->control_w1s);
#ifdef XDMA_POLL_MODE
	iowrite32(lower_32_bits(engine->poll_mode_wb.dma_addr),
		  &engine->regs->poll_mode_wb_lo);
	iowrite32(upper_32_bits(engine->poll_mode_wb.dma_addr),
		  &engine->regs->poll_mode_wb_hi);
#endif
	ioread32(&engine->regs->control);
	engine->c2h_pool = NULL;
	c2h_pool_free(engine, pool);
	return 0;
}
