/* SPDX-License-Identifier: GPL-2.0 */
#ifndef XDMA_C2H_POOL_MATH_H
#define XDMA_C2H_POOL_MATH_H

/* Adjacency describes the descriptor at the supplied NEXT address. Keep
 * each fetch within MRRS, a hardware 64-descriptor block, and a 4 KiB page. */
static inline unsigned int xdma_c2h_pool_adjacent(unsigned long long base,
		unsigned int index, unsigned int slots, unsigned int mrrs)
{
	unsigned long long address = base + (unsigned long long)index * 32;
	unsigned int block = mrrs / 32;
	unsigned int count, limit;

	if (index >= slots || !block)
		return 0;
	if (block > 64)
		block = 64;
	count = slots - index;
	limit = block - (unsigned int)((address & 4095) / 32) % block;
	if (count > limit)
		count = limit;
	limit = 64 - (unsigned int)((address & 4095) / 32) % 64;
	if (count > limit)
		count = limit;
	limit = (4096 - (unsigned int)(address & 4095)) / 32;
	if (count > limit)
		count = limit;
	return count ? count - 1 : 0;
}

/* Validate an ACK without modifying the pool. A restart can only follow
 * acknowledgment of every remaining slot in the finite chain. */
static inline int xdma_c2h_pool_ack_valid(unsigned int consumed,
		unsigned int slots, unsigned int packets, unsigned int restart)
{
	if (consumed > slots || packets > slots - consumed)
		return 0;
	return !restart || consumed + packets == slots;
}

/* Always publish the last descriptor, even for a partial stride. */
static inline int xdma_c2h_pool_wb_notify(unsigned int index,
		unsigned int slots, unsigned int stride)
{
	return (index + 1) % stride == 0 || index + 1 == slots;
}

/* A finite chain may be reused only after userspace consumed all its slots
 * and the previous active hardware chain completed. */
static inline int xdma_c2h_pool_can_launch(unsigned int slots,
		unsigned int target_consumed, unsigned int active_started,
		unsigned int active_completed)
{
	return target_consumed == slots &&
		(!active_started || active_completed == slots);
}

/* Launch the spare bank only after the active bank has finished, while
 * the read bank is still held by userspace. Return -1 if it is unsafe. */
static inline int xdma_c2h_pool_prefetch_bank(unsigned int banks,
		unsigned int read_bank, unsigned int active_bank,
		unsigned int slots, unsigned int completed,
		unsigned int spare_consumed)
{
	if (banks != 2 || read_bank != active_bank || completed != slots ||
	    spare_consumed != slots)
		return -1;
	return read_bank ^ 1U;
}

#endif
