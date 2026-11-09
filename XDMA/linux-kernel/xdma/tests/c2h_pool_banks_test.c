/* SPDX-License-Identifier: GPL-2.0 */
/* Simulate an asynchronous producer and slower userspace consumer, using
 * the production launch/prefetch gates. Payload sequence and ownership
 * assertions detect reorder, overwrite, and lack of actual overlap. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../c2h_pool_math.h"

struct model_bank {
	unsigned int consumed;
	unsigned int completed;
	uint64_t payload[1024];
};

static uint32_t rng = 7;
static unsigned int random_below(unsigned int limit)
{
	rng = rng * 1664525U + 1013904223U;
	return rng % limit;
}

static void run_pipeline(unsigned int slots, unsigned int stride)
{
	struct model_bank bank[2] = { 0 };
	unsigned int read_bank = 0, active = 0, launched = 0, overlap = 0;
	uint64_t produced = 0, consumed = 0;
	unsigned int iterations = 0;

	bank[0].consumed = bank[1].consumed = slots;
	while (consumed < (uint64_t)slots * 100) {
		struct model_bank *reader = &bank[read_bank];
		unsigned int available, count, i, published;
		int target;

		assert(++iterations < slots * 10000);
		if (reader->consumed == slots) {
			if (launched)
				read_bank ^= 1;
			reader = &bank[read_bank];
			if (reader->consumed == slots) {
				assert(xdma_c2h_pool_can_launch(slots, reader->consumed,
					launched, bank[active].completed));
				reader->consumed = reader->completed = 0;
				active = read_bank;
				launched = 1;
			}
		}
		/* The producer progresses independently while earlier packets
		 * are being decoded; result publication uses the actual stride. */
		count = 1 + random_below(slots);
		while (count-- && bank[active].completed < slots) {
			unsigned int pos = bank[active].completed++;
			bank[active].payload[pos] = produced++;
		}
		published = reader->completed == slots ? slots :
			reader->completed / stride * stride;
		assert(published >= reader->consumed);
		available = published - reader->consumed;
		if (!available)
			continue;
		count = 1 + random_below(available);
		target = xdma_c2h_pool_prefetch_bank(2, read_bank, active, slots,
			bank[active].completed, bank[read_bank ^ 1].consumed);
		if (target >= 0) {
			struct model_bank *spare = &bank[target];
			assert(reader->consumed < slots);
			assert(xdma_c2h_pool_can_launch(slots, spare->consumed,
				launched, bank[active].completed));
			spare->consumed = spare->completed = 0;
			active = target;
			overlap++;
			/* New DMA may write the spare before old data is consumed. */
			spare->payload[spare->completed++] = produced++;
		}
		for (i = 0; i < count; ++i)
			assert(reader->payload[reader->consumed++] == consumed++);
	}
	assert(overlap >= 99);
}

int main(void)
{
	unsigned int slots, stride, i;
	const unsigned int sizes[] = { 1, 7, 128, 512, 1024 };

	assert(!xdma_c2h_pool_can_launch(128, 127, 0, 0));
	assert(!xdma_c2h_pool_can_launch(128, 128, 1, 127));
	assert(xdma_c2h_pool_can_launch(128, 128, 0, 0));
	assert(xdma_c2h_pool_can_launch(128, 128, 1, 128));
	assert(xdma_c2h_pool_prefetch_bank(2, 0, 0, 128, 128, 128) == 1);
	assert(xdma_c2h_pool_prefetch_bank(2, 1, 1, 128, 128, 128) == 0);
	assert(xdma_c2h_pool_prefetch_bank(1, 0, 0, 128, 128, 128) < 0);
	assert(xdma_c2h_pool_prefetch_bank(2, 0, 1, 128, 128, 128) < 0);
	assert(xdma_c2h_pool_prefetch_bank(2, 0, 0, 128, 127, 128) < 0);
	assert(xdma_c2h_pool_prefetch_bank(2, 0, 0, 128, 128, 127) < 0);
	for (i = 0; i < sizeof(sizes) / sizeof(*sizes); ++i) {
		slots = sizes[i];
		for (stride = 1; stride <= 128; stride *= 2) {
			unsigned int notifications = 0, last = 0, index;
			for (index = 0; index < slots; ++index)
				if (xdma_c2h_pool_wb_notify(index, slots, stride)) {
					assert(index + 1 - last <= stride);
					last = index + 1;
					notifications++;
				}
			assert(last == slots);
			assert(notifications == (slots + stride - 1) / stride);
			run_pipeline(slots, stride);
		}
	}
	puts("C2H finite-bank ownership, overlap, ordering and WB stride: PASS");
	return 0;
}
