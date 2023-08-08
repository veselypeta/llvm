//===--------- device.hpp - HIP Adapter -----------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===-----------------------------------------------------------------===//
#pragma once

#include "common.hpp"

#include <ur/ur.hpp>

/// UR device mapping to a hipDevice_t.
/// Includes an observer pointer to the platform,
/// and implements the reference counting semantics since
/// HIP objects are not refcounted.
struct ur_device_handle_t_ {
private:
  using native_type = hipDevice_t;

  native_type HIPDevice;
  std::atomic_uint32_t RefCount;
  ur_platform_handle_t Platform;
  hipCtx_t HIPContext;


  size_t MaxAllocSize{0};

public:
  ur_device_handle_t_(native_type HipDevice, hipCtx_t Context,
                      ur_platform_handle_t Platform)
      : HIPDevice(HipDevice), RefCount{1}, Platform(Platform),
        HIPContext(Context) {
          // Max size of memory object allocation in bytes.
          // The minimum value is max(min(1024 × 1024 ×
          // 1024, 1/4th of CL_DEVICE_GLOBAL_MEM_SIZE),
          // 32 × 1024 × 1024) for devices that are not of type
          // CL_DEVICE_TYPE_CUSTOM.

          size_t Global = 0;
          //detail::ur::assertion(hipDeviceTotalMem(&Global, HIPDevice->get()) ==
                                //hipSuccess);
          UR_CHECK_ERROR(hipDeviceTotalMem(&Global, HIPDevice));

          auto QuarterGlobal = static_cast<uint32_t>(Global / 4u);

          MaxAllocSize = std::max(std::min(1024u * 1024u * 1024u, QuarterGlobal),
                                  32u * 1024u * 1024u);
        }

  ~ur_device_handle_t_() {
    UR_CHECK_ERROR(hipDevicePrimaryCtxRelease(HIPDevice));
  }

  native_type get() const noexcept { return HIPDevice; };

  uint32_t getReferenceCount() const noexcept { return RefCount; }

  ur_platform_handle_t getPlatform() const noexcept { return Platform; };

  hipCtx_t getNativeContext() { return HIPContext; };

  size_t getMaxAllocSize() const noexcept { return MaxAllocSize; };
};

int getAttribute(ur_device_handle_t Device, hipDeviceAttribute_t Attribute);
