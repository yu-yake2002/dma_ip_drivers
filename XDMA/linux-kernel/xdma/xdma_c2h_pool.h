/* Versioned, fixed-width UAPI for opt-in persistent XDMA C2H reception. */
#ifndef XDMA_C2H_POOL_UAPI_H
#define XDMA_C2H_POOL_UAPI_H
#include <linux/ioctl.h>
#include <linux/types.h>

#define XDMA_C2H_POOL_ABI 1U
/* Wait for the requested packet count, bounded by the remaining chain. */
#define XDMA_C2H_POOL_WAIT_BATCH (1U << 0)
#define XDMA_C2H_POOL_ADJACENT (1U << 1)
#define XDMA_C2H_POOL_MMAP (1U << 2)
#define XDMA_C2H_POOL_DOUBLE_BUFFER (1U << 3)
#define XDMA_C2H_POOL_WB_STRIDE_SHIFT 8U
#define XDMA_C2H_POOL_WB_STRIDE_MASK (0xffU << XDMA_C2H_POOL_WB_STRIDE_SHIFT)
struct xdma_c2h_pool_config {
  __u32 version;
  __u32 packet_bytes;
  __u32 slots;
  __u32 flags;
};
struct xdma_c2h_pool_stats {
  __u64 launches;
  __u64 reads;
  __u64 packets;
  __u64 bytes;
  __u64 errors;
  __u64 partial_reads;
  __u32 packet_bytes;
  __u32 slots;
};
/* mmap page-offset selectors: payload=0, results=1, poll writeback=2. */
#define XDMA_C2H_POOL_MAP_PAYLOAD 0U
#define XDMA_C2H_POOL_MAP_RESULTS 1U
#define XDMA_C2H_POOL_MAP_WB 2U
struct xdma_c2h_pool_layout {
  __u32 version;
  __u32 packet_bytes;
  __u32 slots;
  __u32 result_stride;
  __u64 payload_bytes;
  __u64 result_bytes;
  __u64 wb_bytes;
};
#define XDMA_C2H_POOL_ADVANCE_RESTART (1U << 0)
struct xdma_c2h_pool_advance {
  __u32 version;
  __u32 packets;
  __u32 flags;
  __u32 reserved;
};
struct xdma_c2h_pool_diagnostics {
  __u32 version;
  __u32 banks;
  __u32 wb_stride;
  __u32 reserved;
  __u64 bank_switches;
  __u64 prefetch_launches;
  __u64 prefetch_unconsumed_packets;
};
#define XDMA_IOCTL_C2H_POOL_DIAGNOSTICS _IOR('x', 21, struct xdma_c2h_pool_diagnostics)
#define XDMA_IOCTL_C2H_POOL_LAYOUT _IOR('x', 19, struct xdma_c2h_pool_layout)
#define XDMA_IOCTL_C2H_POOL_ADVANCE _IOW('x', 20, struct xdma_c2h_pool_advance)
#define XDMA_IOCTL_C2H_POOL_SETUP _IOW('x', 16, struct xdma_c2h_pool_config)
#define XDMA_IOCTL_C2H_POOL_STATS _IOR('x', 17, struct xdma_c2h_pool_stats)
#define XDMA_IOCTL_C2H_POOL_STOP _IO('x', 18)
#endif
