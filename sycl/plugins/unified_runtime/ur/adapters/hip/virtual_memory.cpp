//===--------- virtual_memory.cpp - HIP Adapter ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "virtual_memory.hpp"
#include "common.hpp"
#include "context.hpp"

UR_APIEXPORT ur_result_t UR_APICALL urVirtualMemGranularityGetInfo(
    ur_context_handle_t hContext, ur_device_handle_t hDevice,
    ur_virtual_mem_granularity_info_t propName, size_t propSize,
    void *pPropValue, size_t *pPropSizeRet) {

  ur_result_t Result = UR_RESULT_SUCCESS;
  hipMemAllocationGranularity_flags GranularityFlags;
  switch (propName) {
  case UR_VIRTUAL_MEM_GRANULARITY_INFO_MINIMUM:
    GranularityFlags = hipMemAllocationGranularityRecommended;
    break;

  case UR_VIRTUAL_MEM_GRANULARITY_INFO_RECOMMENDED:
    GranularityFlags = hipMemAllocationGranularityRecommended;
    break;

  default:
    return UR_RESULT_ERROR_INVALID_ENUMERATION;
  }

  try {
    ScopedContext Active(hDevice);
    UrReturnHelper ReturnValue(propSize, pPropValue, pPropSizeRet);
    size_t HipGranularity = 0;

    // choose some sensible defaults
    hipMemAllocationProp AllocProps = {};
    AllocProps.type = hipMemAllocationTypePinned;
    AllocProps.location.type = hipMemLocationTypeDevice;
    AllocProps.location.id = hDevice->get();

    Result = UR_CHECK_ERROR(hipMemGetAllocationGranularity(
        &HipGranularity, &AllocProps, GranularityFlags));
    return ReturnValue(HipGranularity);
  } catch (ur_result_t Err) {
    Result = Err;
  } catch (...) {
    Result = UR_RESULT_ERROR_UNKNOWN;
  }
  return Result;
}

UR_APIEXPORT ur_result_t UR_APICALL
urVirtualMemReserve(ur_context_handle_t hContext, const void *pStart,
                    size_t size, void **ppStart) {
  ur_result_t Result = UR_RESULT_SUCCESS;
  try {
    ScopedContext Active(hContext->getDevice());
    Result = UR_CHECK_ERROR(
        hipMemAddressReserve(ppStart, size, 0, const_cast<void *>(pStart), 0));
  } catch (ur_result_t Err) {
    Result = Err;
  } catch (...) {
    Result = UR_RESULT_ERROR_UNKNOWN;
  }

  return Result;
}

UR_APIEXPORT ur_result_t UR_APICALL urVirtualMemFree(
    ur_context_handle_t hContext, const void *pStart, size_t size) {
  ur_result_t Result = UR_RESULT_SUCCESS;
  try {
    ScopedContext Active(hContext->getDevice());
    Result =
        UR_CHECK_ERROR(hipMemAddressFree(const_cast<void *>(pStart), size));
  } catch (ur_result_t Err) {
    Result = Err;
  } catch (...) {
    Result = UR_RESULT_ERROR_UNKNOWN;
  }
  return Result;
}

UR_APIEXPORT ur_result_t UR_APICALL
urVirtualMemMap(ur_context_handle_t hContext, const void *pStart, size_t size,
                ur_physical_mem_handle_t hPhysicalMem, size_t offset,
                ur_virtual_mem_access_flags_t flags) {
  std::ignore = flags;
  ur_result_t Result = UR_RESULT_SUCCESS;
  try {
    ScopedContext Active(hContext->getDevice());
    Result = UR_CHECK_ERROR(hipMemMap(const_cast<void *>(pStart), size, offset,
                                      hPhysicalMem->get(), 0));

  } catch (ur_result_t Err) {
    Result = Err;
  } catch (...) {
    Result = UR_RESULT_ERROR_UNKNOWN;
  }
  return Result;
}

UR_APIEXPORT ur_result_t UR_APICALL urVirtualMemUnmap(
    ur_context_handle_t hContext, const void *pStart, size_t size) {

  ur_result_t Result = UR_RESULT_SUCCESS;
  try {
    ScopedContext Active(hContext->getDevice());
    Result = UR_CHECK_ERROR(hipMemUnmap(const_cast<void *>(pStart), size));
  } catch (ur_result_t Err) {
    Result = Err;
  } catch (...) {
    Result = UR_RESULT_ERROR_UNKNOWN;
  }
  return Result;
}

UR_APIEXPORT ur_result_t UR_APICALL
urVirtualMemSetAccess(ur_context_handle_t hContext, const void *pStart,
                      size_t size, ur_virtual_mem_access_flags_t flags) {

  ur_result_t Result = UR_RESULT_SUCCESS;
  try {
    ScopedContext Active(hContext->getDevice());

    hipMemAccessFlags HipFlags{};
    if (flags & UR_VIRTUAL_MEM_ACCESS_FLAG_NONE) {
      HipFlags = hipMemAccessFlagsProtNone;
    } else if (flags & UR_VIRTUAL_MEM_ACCESS_FLAG_READ_ONLY) {
      HipFlags = hipMemAccessFlagsProtRead;
    } else if (flags & UR_VIRTUAL_MEM_ACCESS_FLAG_READ_WRITE) {
      HipFlags = hipMemAccessFlagsProtReadWrite;
    }

    hipMemAccessDesc AccessDesc{};
    AccessDesc.location.id = hContext->getDevice()->get();
    AccessDesc.location.type = hipMemLocationTypeDevice;
    AccessDesc.flags = HipFlags;
    Result = UR_CHECK_ERROR(
        hipMemSetAccess(const_cast<void *>(pStart), size, &AccessDesc, 1));
  } catch (ur_result_t Err) {
    Result = Err;
  } catch (...) {
    Result = UR_RESULT_ERROR_UNKNOWN;
  }
  return Result;
}

UR_APIEXPORT ur_result_t UR_APICALL
urVirtualMemGetInfo(ur_context_handle_t hContext, const void *pStart,
                    size_t size, ur_virtual_mem_info_t propName,
                    size_t propSize, void *pPropValue, size_t *pPropSizeRet) {
  ur_result_t Result = UR_RESULT_SUCCESS;
  try {
    ScopedContext Active(hContext->getDevice());
    UrReturnHelper ReturnValue(propSize, pPropValue, pPropSizeRet);

    switch (propName) {
    case UR_VIRTUAL_MEM_INFO_ACCESS_MODE: {
      long long unsigned int HipAccessFlags{};
      hipMemLocation HipLocation{};
      HipLocation.id = hContext->getDevice()->get();
      HipLocation.type = hipMemLocationTypeDevice;
      detail::ur::assertion(hipMemGetAccess(&HipAccessFlags, &HipLocation,
                                            const_cast<void *>(pStart)) ==
                            hipSuccess);
      ur_virtual_mem_access_flags_t OutFlags{};
      switch (HipAccessFlags) {
      case hipMemAccessFlagsProtNone:
        OutFlags |= UR_VIRTUAL_MEM_ACCESS_FLAG_NONE;
        break;
      case hipMemAccessFlagsProtRead:
        OutFlags |= UR_VIRTUAL_MEM_ACCESS_FLAG_READ_ONLY;
        break;
      case hipMemAccessFlagsProtReadWrite:
        OutFlags |= UR_VIRTUAL_MEM_ACCESS_FLAG_READ_WRITE;
        break;
      default:
        break;
      }
      return ReturnValue(OutFlags);
    }

    default:
      return UR_RESULT_ERROR_INVALID_ENUMERATION;
    }

  } catch (ur_result_t Err) {
    Result = Err;
  } catch (...) {
    Result = UR_RESULT_ERROR_UNKNOWN;
  }

  return Result;
}

UR_APIEXPORT ur_result_t UR_APICALL urPhysicalMemCreate(
    ur_context_handle_t hContext, ur_device_handle_t hDevice, size_t size,
    const ur_physical_mem_properties_t *pProperties,
    ur_physical_mem_handle_t *phPhysicalMem) {

  ur_result_t Result = UR_RESULT_SUCCESS;
  try {
    // ScopedContext Active(hDevice);
    std::unique_ptr<ur_physical_mem_handle_t_> PhysicalMem{nullptr};

    hipMemGenericAllocationHandle_t GenericAlloc{};
    hipMemAllocationProp AllocProps = {};
    AllocProps.requestedHandleType =
        hipMemAllocationHandleType::hipMemHandleTypePosixFileDescriptor;
    AllocProps.win32HandleMetaData = nullptr;
    AllocProps.type = hipMemAllocationTypePinned;
    AllocProps.location.type = hipMemLocationTypeDevice;
    AllocProps.location.id = hDevice->get();
    Result = UR_CHECK_ERROR(hipMemCreate(&GenericAlloc, size, &AllocProps, 0));

    PhysicalMem = std::unique_ptr<ur_physical_mem_handle_t_>{
        new ur_physical_mem_handle_t_{GenericAlloc}};

    *phPhysicalMem = PhysicalMem.release();
  } catch (ur_result_t Err) {
    Result = Err;
  } catch (...) {
    Result = UR_RESULT_ERROR_UNKNOWN;
  }
  return Result;
}

UR_APIEXPORT ur_result_t UR_APICALL
urPhysicalMemRetain(ur_physical_mem_handle_t hPhysicalMem) {
  UR_ASSERT(hPhysicalMem->GetReferenceCount() > 0,
            UR_RESULT_ERROR_INVALID_MEM_OBJECT);
  hPhysicalMem->IncrementReferenceCount();
  return UR_RESULT_SUCCESS;
}

UR_APIEXPORT ur_result_t UR_APICALL
urPhysicalMemRelease(ur_physical_mem_handle_t hPhysicalMem) {
  UR_ASSERT(hPhysicalMem->GetReferenceCount() > 0,
            UR_RESULT_ERROR_INVALID_MEM_OBJECT);
  if (hPhysicalMem->DecrementReferenceCount() == 0) {
    std::unique_ptr<ur_physical_mem_handle_t_> PhysicalMem{hPhysicalMem};
  }
  return UR_RESULT_SUCCESS;
}
