/* SPDX-License-Identifier: GPL-2.0 */
/* Standalone invariants for the production adjacency and ACK helpers. */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include "../c2h_pool_math.h"

int main(void)
{
	unsigned int mrrs, offset, slots, index;

	assert(xdma_c2h_pool_adjacent(0, 0, 128, 128) == 3);
	assert(xdma_c2h_pool_adjacent(0, 1, 128, 128) == 2);
	assert(xdma_c2h_pool_adjacent(0, 2, 128, 128) == 1);
	assert(xdma_c2h_pool_adjacent(0, 3, 128, 128) == 0);
	assert(xdma_c2h_pool_adjacent(0, 4, 128, 128) == 3);
	assert(xdma_c2h_pool_adjacent(0, 127, 128, 4096) == 0);
	assert(xdma_c2h_pool_adjacent(0, 128, 128, 128) == 0);
	assert(xdma_c2h_pool_adjacent(0, 0, 128, 0) == 0);
	for (mrrs = 32; mrrs <= 4096; mrrs *= 2) {
		unsigned int block = mrrs / 32 < 64 ? mrrs / 32 : 64;
		for (offset = 0; offset < 4096; offset += 32)
			for (slots = 1; slots <= 128; ++slots)
				for (index = 0; index < slots; ++index) {
					unsigned long long addr = 0x100000ULL + offset + index * 32;
					unsigned int count = xdma_c2h_pool_adjacent(
						0x100000ULL + offset, index, slots, mrrs) + 1;
					assert(count <= slots - index);
					assert(count * 32 <= mrrs);
					assert(count <= block - ((addr & 4095) / 32) % block);
					assert(count <= 64 - ((addr & 4095) / 32) % 64);
					assert(count * 32 <= 4096 - (addr & 4095));
				}
	}
	/* The larger finite chains cross both hardware block and page edges. */
	for (mrrs = 128; mrrs <= 4096; mrrs *= 2)
		for (slots = 512; slots <= 1024; slots *= 2)
			for (index = 0; index < slots; ++index) {
				unsigned long long addr = 0x100000ULL + index * 32;
				unsigned int count = xdma_c2h_pool_adjacent(
					0x100000ULL, index, slots, mrrs) + 1;
				assert(count <= slots - index);
				assert(count * 32 <= mrrs);
				assert(count <= 64 - ((addr & 4095) / 32) % 64);
				assert(count * 32 <= 4096 - (addr & 4095));
			}
	assert(xdma_c2h_pool_ack_valid(128, 128, 0, 1)); /* first launch */
	assert(xdma_c2h_pool_ack_valid(0, 128, 128, 1));
	assert(xdma_c2h_pool_ack_valid(7, 128, 3, 0));
	assert(xdma_c2h_pool_ack_valid(7, 128, 121, 1));
	assert(!xdma_c2h_pool_ack_valid(7, 128, 120, 1));
	assert(!xdma_c2h_pool_ack_valid(7, 128, 122, 0));
	assert(!xdma_c2h_pool_ack_valid(129, 128, 0, 0));
	assert(!xdma_c2h_pool_ack_valid(0, 128, UINT_MAX, 1));
	puts("C2H adjacency and ACK invariants: PASS");
	return 0;
}
