//===--------- virtual_memory.hpp - HIP Adapter ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
#pragma once
#include "common.hpp"

struct ur_physical_mem_handle_t_ {
  using NativeType = hipMemGenericAllocationHandle_t;
  ur_physical_mem_handle_t_(NativeType handle)
      : PhysicalMem(handle), ReferenceCount(1) {}
  ~ur_physical_mem_handle_t_() { (void)hipMemRelease(PhysicalMem); }

  uint32_t IncrementReferenceCount() noexcept { return ++ReferenceCount; }
  uint32_t DecrementReferenceCount() noexcept { return --ReferenceCount; }
  uint32_t GetReferenceCount() const noexcept { return ReferenceCount; }
  NativeType get() { return PhysicalMem; }

private:
  NativeType PhysicalMem;
  std::atomic_uint32_t ReferenceCount;
};
