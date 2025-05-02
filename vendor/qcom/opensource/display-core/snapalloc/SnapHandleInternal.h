// Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef __SNAP_HANDLE_INTERNAL_H__
#define __SNAP_HANDLE_INTERNAL_H__

#include "SnapTypes.h"

#include <unistd.h>
#include <memory>

namespace snapalloc {

inline bool isSnapHandleEmpty(SnapHandle *handle) {
  return (handle->num_ints == 0 || handle->num_fds == 0);
}

class SnapHandleInternal : public SnapHandle {
 public:
  int fd;
  int fd_metadata;
  int flags;
  int aligned_width_in_bytes;
  int aligned_width_in_pixels;
  int aligned_height;
  int unaligned_width;
  int unaligned_height;
  vendor_qti_hardware_display_common_PixelFormat format;
  int buffer_type;
  unsigned int layer_count;
  uint64_t id;
  vendor_qti_hardware_display_common_BufferUsage usage =
      vendor_qti_hardware_display_common_BufferUsage::CPU_READ_NEVER;
  unsigned int size;
  uint64_t base;
  uint64_t base_metadata;
  uint64_t fb_id;
  unsigned int reserved_size;
  unsigned int custom_content_md_reserved_size;
  uint64_t pixel_format_modifier;
  uint64_t reserved_region_base;
  uint64_t custom_content_md_region_base;
  static const int kNumFds = 2;
  unsigned int flush = false;
  // Lock count to ensure nested lock/unlock situations are handled correctly
  int lock_count = 0;

 private:
  int ref_count = 0;

 public:
  void Init(int fd, int meta_fd, int flags, int width_in_bytes, int width_in_pixels, int height,
            int uw, int uh, vendor_qti_hardware_display_common_PixelFormat format, int buf_type,
            unsigned int size, vendor_qti_hardware_display_common_BufferUsage usage) {
    this->fd = fd;
    this->fd_metadata = meta_fd;
    this->flags = flags;
    this->aligned_width_in_bytes = width_in_bytes;
    this->aligned_width_in_pixels = width_in_pixels;
    this->aligned_height = height;
    this->unaligned_width = uw;
    this->unaligned_height = uh;
    this->format = format;
    this->buffer_type = buf_type;
    this->layer_count = 1;
    this->id = 0;
    this->usage = usage;
    this->size = size;
    this->base = 0;
    this->base_metadata = 0;
    this->version = static_cast<int>(sizeof(SnapHandle));
    this->num_ints = NumInts();
    this->num_fds = kNumFds;
  }

  SnapHandleInternal() {}

  ~SnapHandleInternal() {}

  int GetRefCount() { return ref_count; }
  void IncRef() { ++ref_count; }
  bool DecRef() { return --ref_count == 0; }
  void ResetRefCount() { ref_count = 0; }
  static inline int NumInts() {
    return (((sizeof(SnapHandleInternal) - sizeof(SnapHandle)) / sizeof(int)) - kNumFds);
  }

  static int validate(SnapHandle *h) {
    if (!h || h->version != sizeof(SnapHandle) || h->num_ints != NumInts() || h->num_fds != kNumFds) {
      DLOGE("Invalid SnapHandleInternal (at %p): ver(%d/%zu) ints(%d/%d) fds(%d/%d)", h,
            h ? h->version : -1, sizeof(SnapHandle), h ? h->num_ints : -1, NumInts(),
            h ? h->num_fds : -1, kNumFds);
      return -1;
    }

    return 0;
  }
};

}  // namespace snapalloc

#endif  // __SNAP_HANDLE_INTERNAL_H__