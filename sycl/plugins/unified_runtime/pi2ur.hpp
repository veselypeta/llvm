//===---------------- pi2ur.hpp - PI API to UR API  --------------------==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===------------------------------------------------------------------===//
#pragma once

#include "ur_api.h"
#include <sycl/detail/pi.h>
#include <ur/ur.hpp>

#include <cassert>

// Map of UR error codes to PI error codes
static pi_result ur2piResult(ur_result_t urResult) {
  if (urResult == UR_RESULT_SUCCESS)
    return PI_SUCCESS;

  switch (urResult) {
  case UR_RESULT_ERROR_UNKNOWN:
    return PI_ERROR_UNKNOWN;
  case UR_RESULT_ERROR_DEVICE_LOST:
    return PI_ERROR_DEVICE_NOT_FOUND;
  case UR_RESULT_ERROR_INVALID_OPERATION:
    return PI_ERROR_INVALID_OPERATION;
  case UR_RESULT_ERROR_INVALID_PLATFORM:
    return PI_ERROR_INVALID_PLATFORM;
  case UR_RESULT_ERROR_INVALID_ARGUMENT:
    return PI_ERROR_INVALID_ARG_VALUE;
  case UR_RESULT_ERROR_INVALID_VALUE:
    return PI_ERROR_INVALID_VALUE;
  case UR_RESULT_ERROR_INVALID_EVENT:
    return PI_ERROR_INVALID_EVENT;
  case UR_RESULT_ERROR_INVALID_BINARY:
    return PI_ERROR_INVALID_BINARY;
  case UR_RESULT_ERROR_INVALID_KERNEL_NAME:
    return PI_ERROR_INVALID_KERNEL_NAME;
  case UR_RESULT_ERROR_INVALID_FUNCTION_NAME:
    return PI_ERROR_BUILD_PROGRAM_FAILURE;
  case UR_RESULT_ERROR_INVALID_WORK_GROUP_SIZE:
    return PI_ERROR_INVALID_WORK_GROUP_SIZE;
  case UR_RESULT_ERROR_OUT_OF_DEVICE_MEMORY:
    return PI_ERROR_OUT_OF_RESOURCES;
  case UR_RESULT_ERROR_OUT_OF_HOST_MEMORY:
    return PI_ERROR_OUT_OF_HOST_MEMORY;
  case UR_RESULT_ERROR_PROGRAM_BUILD_FAILURE:
    return PI_ERROR_BUILD_PROGRAM_FAILURE;
  case UR_RESULT_ERROR_UNINITIALIZED:
    return PI_ERROR_UNINITIALIZED;
  case UR_RESULT_ERROR_INVALID_ENUMERATION:
    return PI_ERROR_INVALID_VALUE;
  default:
    return PI_ERROR_UNKNOWN;
  };
}

// Early exits on any error
#define HANDLE_ERRORS(urCall)                                                  \
  if (auto Result = urCall)                                                    \
    return ur2piResult(Result);

// A version of return helper that returns pi_result and not ur_result_t
class ReturnHelper : public UrReturnHelper {
public:
  using UrReturnHelper::UrReturnHelper;

  template <class T> pi_result operator()(const T &t) {
    return ur2piResult(UrReturnHelper::operator()(t));
  }
  // Array return value
  template <class T> pi_result operator()(const T *t, size_t s) {
    return ur2piResult(UrReturnHelper::operator()(t, s));
  }
  // Array return value where element type is differrent from T
  template <class RetType, class T> pi_result operator()(const T *t, size_t s) {
    return ur2piResult(UrReturnHelper::operator()<RetType>(t, s));
  }
};

// A version of return helper that supports conversion through a map
class ConvertHelper : public ReturnHelper {
  using ReturnHelper::ReturnHelper;

public:
  // Convert the value using a conversion map
  template <typename TypeUR, typename TypePI>
  pi_result convert(std::function<TypePI(TypeUR)> Func) {
    *param_value_size_ret = sizeof(TypePI);

    // There is no value to convert.
    if (!param_value)
      return PI_SUCCESS;

    auto pValueUR = static_cast<TypeUR *>(param_value);
    auto pValuePI = static_cast<TypePI *>(param_value);

    // Cannot convert to a smaller storage type
    PI_ASSERT(sizeof(TypePI) >= sizeof(TypeUR), PI_ERROR_UNKNOWN);

    *pValuePI = Func(*pValueUR);
    return PI_SUCCESS;
  }

  // Convert the array (0-terminated) using a conversion map
  template <typename TypeUR, typename TypePI>
  pi_result convertArray(std::function<TypePI(TypeUR)> Func) {
    // Cannot convert to a smaller element storage type
    PI_ASSERT(sizeof(TypePI) >= sizeof(TypeUR), PI_ERROR_UNKNOWN);
    *param_value_size_ret *= sizeof(TypePI) / sizeof(TypeUR);

    // There is no value to convert. Adjust to a possibly bigger PI storage.
    if (!param_value)
      return PI_SUCCESS;

    PI_ASSERT(*param_value_size_ret % sizeof(TypePI) == 0, PI_ERROR_UNKNOWN);

    // Make a copy of the input UR array as we may possibly overwrite
    // following elements while converting previous ones (if extending).
    auto ValueUR = new char[*param_value_size_ret];
    auto pValueUR = reinterpret_cast<TypeUR *>(ValueUR);
    auto pValuePI = static_cast<TypePI *>(param_value);
    memcpy(pValueUR, param_value, *param_value_size_ret);

    while (pValueUR) {
      if (*pValueUR == 0) {
        *pValuePI = 0;
        break;
      }

      *pValuePI = Func(*pValueUR);
      ++pValuePI;
      ++pValueUR;
    }

    delete[] ValueUR;
    return PI_SUCCESS;
  }

  // Convert the bitset using a conversion map
  template <typename TypeUR, typename TypePI>
  pi_result convertBitSet(std::function<TypePI(TypeUR)> Func) {
    // There is no value to convert.
    if (!param_value)
      return PI_SUCCESS;

    auto pValuePI = static_cast<TypePI *>(param_value);
    auto pValueUR = static_cast<TypeUR *>(param_value);

    // Cannot handle biteset large than size_t
    PI_ASSERT(sizeof(TypeUR) <= sizeof(size_t), PI_ERROR_UNKNOWN);
    size_t In = *pValueUR;
    TypePI Out = 0;

    size_t Val;
    while ((Val = In & -In)) { // Val is the rightmost set bit in In
      In &= In - 1;            // Reset the rightmost set bit

      // Convert the Val alone and merge it into Out
      *pValueUR = TypeUR(Val);
      if (auto Res = convert(Func))
        return Res;
      Out |= *pValuePI;
    }
    *pValuePI = TypePI(Out);
    return PI_SUCCESS;
  }
};

// Translate UR platform info values to PI info values
inline pi_result ur2piPlatformInfoValue(ur_platform_info_t ParamName,
                                        size_t ParamValueSizePI,
                                        size_t *ParamValueSizeUR,
                                        void *ParamValue) {

  ConvertHelper Value(ParamValueSizePI, ParamValue, ParamValueSizeUR);

  switch (ParamName) {
  case UR_PLATFORM_INFO_EXTENSIONS:
  case UR_PLATFORM_INFO_NAME:
  case UR_PLATFORM_INFO_PROFILE:
  case UR_PLATFORM_INFO_VENDOR_NAME:
  case UR_PLATFORM_INFO_VERSION:
    // These ones do not need ur2pi translations
    break;
  case UR_PLATFORM_INFO_BACKEND: {
    auto ConvertFunc = [](ur_platform_backend_t UrValue) {
      switch (UrValue) {
      case UR_PLATFORM_BACKEND_UNKNOWN:
        return PI_EXT_PLATFORM_BACKEND_UNKNOWN;
      case UR_PLATFORM_BACKEND_LEVEL_ZERO:
        return PI_EXT_PLATFORM_BACKEND_LEVEL_ZERO;
      case UR_PLATFORM_BACKEND_OPENCL:
        return PI_EXT_PLATFORM_BACKEND_OPENCL;
      case UR_PLATFORM_BACKEND_CUDA:
        return PI_EXT_PLATFORM_BACKEND_CUDA;
      case UR_PLATFORM_BACKEND_HIP:
        return PI_EXT_PLATFORM_BACKEND_HIP;
      default:
        die("UR_PLATFORM_INFO_BACKEND: unhandled value");
      }
    };
    return Value.convert<ur_platform_backend_t, pi_platform_backend>(
        ConvertFunc);
  }
  default:
    return PI_ERROR_UNKNOWN;
  }

  if (ParamValueSizePI && ParamValueSizePI != *ParamValueSizeUR) {
    fprintf(stderr, "UR PlatformInfoType=%d PI=%d but UR=%d\n", ParamName,
            (int)ParamValueSizePI, (int)*ParamValueSizeUR);
    die("ur2piPlatformInfoValue: size mismatch");
  }

  return PI_SUCCESS;
}

// Handle mismatched PI and UR type return sizes for info queries
inline pi_result fixupInfoValueTypes(size_t ParamValueSizeRetUR,
                                     size_t *ParamValueSizeRetPI,
                                     size_t ParamValueSize, void *ParamValue) {
  if (ParamValueSizeRetUR == 1 && ParamValueSize == 4) {
    // extend bool to pi_bool (uint32_t)
    if (ParamValue) {
      auto *ValIn = static_cast<bool *>(ParamValue);
      auto *ValOut = static_cast<pi_bool *>(ParamValue);
      *ValOut = static_cast<pi_bool>(*ValIn);
    }
    if (ParamValueSizeRetPI) {
      *ParamValueSizeRetPI = sizeof(pi_bool);
    }
  }

  return PI_SUCCESS;
}

template <typename TypeOut, typename TypeFlag>
inline pi_result
ConvertInputBitfield(pi_bitfield in, TypeOut *out,
                     const std::unordered_map<pi_bitfield, TypeFlag> &map) {
  *out = 0;
  for (auto &[FlagPI, FlagUR] : map) {
    if (in & FlagPI) {
      *out |= FlagUR;
    }
  }

  return PI_SUCCESS;
}

// Convert bitfield flags from PI to UR for MemFlags
inline pi_result pi2urMemFlags(pi_mem_flags piFlags, ur_mem_flags_t *urFlags) {
  static const std::unordered_map<pi_mem_flags, ur_mem_flags_t> MemFlagsMap = {
      {PI_MEM_FLAGS_ACCESS_RW, UR_MEM_FLAG_READ_WRITE},
      {PI_MEM_ACCESS_READ_ONLY, UR_MEM_FLAG_READ_ONLY},
      {PI_MEM_FLAGS_HOST_PTR_USE, UR_MEM_FLAG_USE_HOST_POINTER},
      {PI_MEM_FLAGS_HOST_PTR_COPY, UR_MEM_FLAG_ALLOC_COPY_HOST_POINTER},
      {PI_MEM_FLAGS_HOST_PTR_ALLOC, UR_MEM_FLAG_ALLOC_HOST_POINTER},
  };

  return ConvertInputBitfield(piFlags, urFlags, MemFlagsMap);
}

// Convert bitfield flags from PI to UR for MapFlags
inline pi_result pi2urMapFlags(pi_mem_flags piFlags, ur_mem_flags_t *urFlags) {
  static const std::unordered_map<pi_bitfield, ur_map_flag_t> MapFlagsMap = {
      {PI_MAP_READ, UR_MAP_FLAG_READ},
      {PI_MAP_WRITE, UR_MAP_FLAG_WRITE},
  };
  return ConvertInputBitfield(piFlags, urFlags, MapFlagsMap);
}

// Translate UR device info values to PI info values
inline pi_result ur2piDeviceInfoValue(ur_device_info_t ParamName,
                                      size_t ParamValueSizePI,
                                      size_t *ParamValueSizeUR,
                                      void *ParamValue) {

  ConvertHelper Value(ParamValueSizePI, ParamValue, ParamValueSizeUR);

  if (ParamName == UR_DEVICE_INFO_TYPE) {
    auto ConvertFunc = [](ur_device_type_t UrValue) {
      switch (UrValue) {
      case UR_DEVICE_TYPE_DEFAULT:
        return PI_DEVICE_TYPE_DEFAULT;
      case UR_DEVICE_TYPE_CPU:
        return PI_DEVICE_TYPE_CPU;
      case UR_DEVICE_TYPE_GPU:
        return PI_DEVICE_TYPE_GPU;
      case UR_DEVICE_TYPE_FPGA:
        return PI_DEVICE_TYPE_ACC;
      default:
        die("UR_DEVICE_INFO_TYPE: unhandled value");
      }
    };
    return Value.convert<ur_device_type_t, pi_device_type>(ConvertFunc);
  } else if (ParamName == UR_DEVICE_INFO_QUEUE_PROPERTIES) {
    auto ConvertFunc = [](ur_queue_flag_t UrValue) {
      switch (UrValue) {
      case UR_QUEUE_FLAG_OUT_OF_ORDER_EXEC_MODE_ENABLE:
        return PI_QUEUE_FLAG_OUT_OF_ORDER_EXEC_MODE_ENABLE;
      case UR_QUEUE_FLAG_PROFILING_ENABLE:
        return PI_QUEUE_FLAG_PROFILING_ENABLE;
      case UR_QUEUE_FLAG_ON_DEVICE:
        return PI_QUEUE_FLAG_ON_DEVICE;
      case UR_QUEUE_FLAG_ON_DEVICE_DEFAULT:
        return PI_QUEUE_FLAG_ON_DEVICE_DEFAULT;
      default:
        die("UR_DEVICE_INFO_QUEUE_PROPERTIES: unhandled value");
      }
    };
    return Value.convertBitSet<ur_queue_flag_t, pi_queue_properties>(
        ConvertFunc);
  } else if (ParamName == UR_DEVICE_INFO_EXECUTION_CAPABILITIES) {
    auto ConvertFunc = [](ur_device_exec_capability_flag_t UrValue) {
      switch (UrValue) {
      case UR_DEVICE_EXEC_CAPABILITY_FLAG_KERNEL:
        return PI_DEVICE_EXEC_CAPABILITIES_KERNEL;
      case UR_DEVICE_EXEC_CAPABILITY_FLAG_NATIVE_KERNEL:
        return PI_DEVICE_EXEC_CAPABILITIES_NATIVE_KERNEL;
      default:
        die("UR_DEVICE_INFO_EXECUTION_CAPABILITIES: unhandled value");
      }
    };
    return Value
        .convertBitSet<ur_device_exec_capability_flag_t, pi_queue_properties>(
            ConvertFunc);
  } else if (ParamName == UR_DEVICE_INFO_PARTITION_AFFINITY_DOMAIN) {
    auto ConvertFunc = [](ur_device_affinity_domain_flag_t UrValue) {
      switch (UrValue) {
      case UR_DEVICE_AFFINITY_DOMAIN_FLAG_NUMA:
        return PI_DEVICE_AFFINITY_DOMAIN_NUMA;
      case UR_DEVICE_AFFINITY_DOMAIN_FLAG_NEXT_PARTITIONABLE:
        return PI_DEVICE_AFFINITY_DOMAIN_NEXT_PARTITIONABLE;
      default:
        die("UR_DEVICE_INFO_PARTITION_AFFINITY_DOMAIN: unhandled value");
      }
    };
    return Value.convertBitSet<ur_device_affinity_domain_flag_t,
                               pi_device_affinity_domain>(ConvertFunc);
  } else if (ParamName == UR_DEVICE_INFO_PARTITION_TYPE) {
    auto ConvertFunc = [](ur_device_partition_property_t UrValue) {
      switch (UrValue) {
      case UR_DEVICE_PARTITION_BY_AFFINITY_DOMAIN:
        return PI_DEVICE_PARTITION_BY_AFFINITY_DOMAIN;
      case UR_DEVICE_PARTITION_BY_CSLICE:
        return PI_EXT_INTEL_DEVICE_PARTITION_BY_CSLICE;
      case (ur_device_partition_property_t)
          UR_DEVICE_AFFINITY_DOMAIN_FLAG_NEXT_PARTITIONABLE:
        return (pi_device_partition_property)
            PI_DEVICE_AFFINITY_DOMAIN_NEXT_PARTITIONABLE;
      default:
        die("UR_DEVICE_INFO_PARTITION_TYPE: unhandled value");
      }
    };
    return Value.convertArray<ur_device_partition_property_t,
                              pi_device_partition_property>(ConvertFunc);
  } else if (ParamName == UR_DEVICE_INFO_PARTITION_PROPERTIES) {
    auto ConvertFunc = [](ur_device_partition_property_t UrValue) {
      switch (UrValue) {
      case UR_DEVICE_PARTITION_BY_AFFINITY_DOMAIN:
        return PI_DEVICE_PARTITION_BY_AFFINITY_DOMAIN;
      case UR_DEVICE_PARTITION_BY_CSLICE:
        return PI_EXT_INTEL_DEVICE_PARTITION_BY_CSLICE;
      default:
        die("UR_DEVICE_INFO_PARTITION_PROPERTIES: unhandled value");
      }
    };
    return Value.convertArray<ur_device_partition_property_t,
                              pi_device_partition_property>(ConvertFunc);
  } else if (ParamName == UR_DEVICE_INFO_LOCAL_MEM_TYPE) {
    auto ConvertFunc = [](ur_device_local_mem_type_t UrValue) {
      switch (UrValue) {
      case UR_DEVICE_LOCAL_MEM_TYPE_LOCAL:
        return PI_DEVICE_LOCAL_MEM_TYPE_LOCAL;
      case UR_DEVICE_LOCAL_MEM_TYPE_GLOBAL:
        return PI_DEVICE_LOCAL_MEM_TYPE_GLOBAL;
      default:
        die("UR_DEVICE_INFO_LOCAL_MEM_TYPE: unhandled value");
      }
    };
    return Value.convert<ur_device_local_mem_type_t, pi_device_local_mem_type>(
        ConvertFunc);
  } else if (ParamName == UR_DEVICE_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES ||
             ParamName == UR_DEVICE_INFO_ATOMIC_FENCE_ORDER_CAPABILITIES) {
    auto ConvertFunc = [](ur_memory_order_capability_flag_t UrValue) {
      switch (UrValue) {
      case UR_MEMORY_ORDER_CAPABILITY_FLAG_RELAXED:
        return PI_MEMORY_ORDER_RELAXED;
      case UR_MEMORY_ORDER_CAPABILITY_FLAG_ACQUIRE:
        return PI_MEMORY_ORDER_ACQUIRE;
      case UR_MEMORY_ORDER_CAPABILITY_FLAG_RELEASE:
        return PI_MEMORY_ORDER_RELEASE;
      case UR_MEMORY_ORDER_CAPABILITY_FLAG_ACQ_REL:
        return PI_MEMORY_ORDER_ACQ_REL;
      case UR_MEMORY_ORDER_CAPABILITY_FLAG_SEQ_CST:
        return PI_MEMORY_ORDER_SEQ_CST;
      default:
        die("UR_DEVICE_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES: unhandled "
            "value");
      }
    };
    return Value.convertBitSet<ur_memory_order_capability_flag_t,
                               pi_memory_order_capabilities>(ConvertFunc);
  } else if (ParamName == UR_DEVICE_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES ||
             ParamName == UR_DEVICE_INFO_ATOMIC_FENCE_SCOPE_CAPABILITIES) {
    auto ConvertFunc = [](ur_memory_scope_capability_flag_t UrValue) {
      switch (UrValue) {
      case UR_MEMORY_SCOPE_CAPABILITY_FLAG_WORK_ITEM:
        return PI_MEMORY_SCOPE_WORK_ITEM;
      case UR_MEMORY_SCOPE_CAPABILITY_FLAG_SUB_GROUP:
        return PI_MEMORY_SCOPE_SUB_GROUP;
      case UR_MEMORY_SCOPE_CAPABILITY_FLAG_WORK_GROUP:
        return PI_MEMORY_SCOPE_WORK_GROUP;
      case UR_MEMORY_SCOPE_CAPABILITY_FLAG_DEVICE:
        return PI_MEMORY_SCOPE_DEVICE;
      case UR_MEMORY_SCOPE_CAPABILITY_FLAG_SYSTEM:
        return PI_MEMORY_SCOPE_SYSTEM;
      default:
        die("UR_DEVICE_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES: unhandled "
            "value");
      }
    };
    return Value.convertBitSet<ur_memory_scope_capability_flag_t,
                               pi_memory_scope_capabilities>(ConvertFunc);
  } else {
    // TODO: what else needs a UR-PI translation?
  }

  if (ParamValueSizePI && ParamValueSizePI != *ParamValueSizeUR) {
    fprintf(stderr, "UR DeviceInfoType=%d PI=%d but UR=%d\n", ParamName,
            (int)ParamValueSizePI, (int)*ParamValueSizeUR);
    die("ur2piDeviceInfoValue: size mismatch");
  }
  return PI_SUCCESS;
}

namespace pi2ur {

inline pi_result piTearDown(void *) {
  HANDLE_ERRORS(urTearDown(nullptr));
  return PI_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// Platform
inline pi_result piPlatformsGet(pi_uint32 num_entries, pi_platform *platforms,
                                pi_uint32 *num_platforms) {

  urInit(0);
  uint32_t Count = num_entries;
  auto phPlatforms = reinterpret_cast<ur_platform_handle_t *>(platforms);
  HANDLE_ERRORS(urPlatformGet(Count, phPlatforms, num_platforms));
  return PI_SUCCESS;
}

inline pi_result piPlatformGetInfo(pi_platform platform,
                                   pi_platform_info ParamName,
                                   size_t ParamValueSize, void *ParamValue,
                                   size_t *ParamValueSizeRet) {
  ur_platform_info_t InfoType;
  switch (ParamName) {
  case PI_PLATFORM_INFO_EXTENSIONS:
    InfoType = UR_PLATFORM_INFO_NAME;
    break;
  case PI_PLATFORM_INFO_NAME:
    InfoType = UR_PLATFORM_INFO_NAME;
    break;
  case PI_PLATFORM_INFO_PROFILE:
    InfoType = UR_PLATFORM_INFO_PROFILE;
    break;
  case PI_PLATFORM_INFO_VENDOR:
    InfoType = UR_PLATFORM_INFO_VENDOR_NAME;
    break;
  case PI_PLATFORM_INFO_VERSION:
    InfoType = UR_PLATFORM_INFO_VERSION;
    break;
  case PI_EXT_PLATFORM_INFO_BACKEND:
    InfoType = UR_PLATFORM_INFO_BACKEND;
    break;
  default:
    return PI_ERROR_UNKNOWN;
  }

  size_t SizeInOut = ParamValueSize;
  auto hPlatform = reinterpret_cast<ur_platform_handle_t>(platform);
  HANDLE_ERRORS(urPlatformGetInfo(hPlatform, InfoType, SizeInOut, ParamValue,
                                  ParamValueSizeRet));

  ur2piPlatformInfoValue(InfoType, ParamValueSize, &SizeInOut, ParamValue);

  return PI_SUCCESS;
}

inline pi_result piPluginGetLastError(char **ppMesage) {
  // PI doesn't take a platform, but UR does. For now just get the first
  // platform, which should be fine to do for the existing plugins.
  ur_platform_handle_t hPlatform;
  urPlatformGet(1, &hPlatform, nullptr);

  if (hPlatform) {
    HANDLE_ERRORS(
        urGetLastResult(hPlatform, const_cast<const char **>(ppMesage)));
  }

  // No platforms means no error was set, so we can return success
  return PI_SUCCESS;
}

inline pi_result piPluginGetBackendOption(pi_platform Platform,
                                          const char *FrontendOption,
                                          const char **PlatformOption) {

  auto UrPlatform = reinterpret_cast<ur_platform_handle_t>(Platform);
  HANDLE_ERRORS(
      urPlatformGetBackendOption(UrPlatform, FrontendOption, PlatformOption));

  return PI_SUCCESS;
}

// Platform
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Device
inline pi_result piDevicesGet(pi_platform Platform, pi_device_type DeviceType,
                              pi_uint32 NumEntries, pi_device *Devices,
                              pi_uint32 *NumDevices) {
  ur_device_type_t Type;
  switch (DeviceType) {
  case PI_DEVICE_TYPE_ALL:
    Type = UR_DEVICE_TYPE_ALL;
    break;
  case PI_DEVICE_TYPE_GPU:
    Type = UR_DEVICE_TYPE_GPU;
    break;
  case PI_DEVICE_TYPE_CPU:
    Type = UR_DEVICE_TYPE_CPU;
    break;
  case PI_DEVICE_TYPE_ACC:
    Type = UR_DEVICE_TYPE_FPGA;
    break;
  case PI_DEVICE_TYPE_DEFAULT:
    Type = UR_DEVICE_TYPE_DEFAULT;
    break;
  default:
    return PI_ERROR_UNKNOWN;
  }

  uint32_t Count = NumEntries;
  auto hPlatform = reinterpret_cast<ur_platform_handle_t>(Platform);
  auto phDevices = reinterpret_cast<ur_device_handle_t *>(Devices);
  HANDLE_ERRORS(urDeviceGet(hPlatform, Type, Count, phDevices, NumDevices));
  return PI_SUCCESS;
}

inline pi_result piDeviceRetain(pi_device Device) {
  auto hDevice = reinterpret_cast<ur_device_handle_t>(Device);
  HANDLE_ERRORS(urDeviceRetain(hDevice));
  return PI_SUCCESS;
}

inline pi_result piDeviceRelease(pi_device Device) {
  auto hDevice = reinterpret_cast<ur_device_handle_t>(Device);
  HANDLE_ERRORS(urDeviceRelease(hDevice));
  return PI_SUCCESS;
}

inline pi_result piDeviceGetInfo(pi_device Device, pi_device_info ParamName,
                                 size_t ParamValueSize, void *ParamValue,
                                 size_t *ParamValueSizeRet) {

  ur_device_info_t InfoType;
  switch (ParamName) {
  case PI_DEVICE_INFO_TYPE:
    InfoType = UR_DEVICE_INFO_TYPE;
    break;
  case PI_DEVICE_INFO_PARENT_DEVICE:
    InfoType = UR_DEVICE_INFO_PARENT_DEVICE;
    break;
  case PI_DEVICE_INFO_PLATFORM:
    InfoType = UR_DEVICE_INFO_PLATFORM;
    break;
  case PI_DEVICE_INFO_VENDOR_ID:
    InfoType = UR_DEVICE_INFO_VENDOR_ID;
    break;
  case PI_DEVICE_INFO_UUID:
    InfoType = UR_DEVICE_INFO_UUID;
    break;
  case PI_DEVICE_INFO_ATOMIC_64:
    InfoType = UR_DEVICE_INFO_ATOMIC_64;
    break;
  case PI_DEVICE_INFO_EXTENSIONS:
    InfoType = UR_DEVICE_INFO_EXTENSIONS;
    break;
  case PI_DEVICE_INFO_NAME:
    InfoType = UR_DEVICE_INFO_NAME;
    break;
  case PI_DEVICE_INFO_COMPILER_AVAILABLE:
    InfoType = UR_DEVICE_INFO_COMPILER_AVAILABLE;
    break;
  case PI_DEVICE_INFO_LINKER_AVAILABLE:
    InfoType = UR_DEVICE_INFO_LINKER_AVAILABLE;
    break;
  case PI_DEVICE_INFO_MAX_COMPUTE_UNITS:
    InfoType = UR_DEVICE_INFO_MAX_COMPUTE_UNITS;
    break;
  case PI_DEVICE_INFO_MAX_WORK_ITEM_DIMENSIONS:
    InfoType = UR_DEVICE_INFO_MAX_WORK_ITEM_DIMENSIONS;
    break;
  case PI_DEVICE_INFO_MAX_WORK_GROUP_SIZE:
    InfoType = UR_DEVICE_INFO_MAX_WORK_GROUP_SIZE;
    break;
  case PI_DEVICE_INFO_MAX_WORK_ITEM_SIZES:
    InfoType = UR_DEVICE_INFO_MAX_WORK_ITEM_SIZES;
    break;
  case PI_DEVICE_INFO_MAX_CLOCK_FREQUENCY:
    InfoType = UR_DEVICE_INFO_MAX_CLOCK_FREQUENCY;
    break;
  case PI_DEVICE_INFO_ADDRESS_BITS:
    InfoType = UR_DEVICE_INFO_ADDRESS_BITS;
    break;
  case PI_DEVICE_INFO_MAX_MEM_ALLOC_SIZE:
    InfoType = UR_DEVICE_INFO_MAX_MEM_ALLOC_SIZE;
    break;
  case PI_DEVICE_INFO_GLOBAL_MEM_SIZE:
    InfoType = UR_DEVICE_INFO_GLOBAL_MEM_SIZE;
    break;
  case PI_DEVICE_INFO_LOCAL_MEM_SIZE:
    InfoType = UR_DEVICE_INFO_LOCAL_MEM_SIZE;
    break;
  case PI_DEVICE_INFO_IMAGE_SUPPORT:
    InfoType = UR_DEVICE_INFO_IMAGE_SUPPORTED;
    break;
  case PI_DEVICE_INFO_HOST_UNIFIED_MEMORY:
    InfoType = UR_DEVICE_INFO_HOST_UNIFIED_MEMORY;
    break;
  case PI_DEVICE_INFO_AVAILABLE:
    InfoType = UR_DEVICE_INFO_AVAILABLE;
    break;
  case PI_DEVICE_INFO_VENDOR:
    InfoType = UR_DEVICE_INFO_VENDOR;
    break;
  case PI_DEVICE_INFO_DRIVER_VERSION:
    InfoType = UR_DEVICE_INFO_DRIVER_VERSION;
    break;
  case PI_DEVICE_INFO_VERSION:
    InfoType = UR_DEVICE_INFO_VERSION;
    break;
  case PI_DEVICE_INFO_PARTITION_MAX_SUB_DEVICES:
    InfoType = UR_DEVICE_INFO_PARTITION_MAX_SUB_DEVICES;
    break;
  case PI_DEVICE_INFO_REFERENCE_COUNT:
    InfoType = UR_DEVICE_INFO_REFERENCE_COUNT;
    break;
  case PI_DEVICE_INFO_PARTITION_PROPERTIES:
    InfoType = UR_DEVICE_INFO_PARTITION_PROPERTIES;
    break;
  case PI_DEVICE_INFO_PARTITION_AFFINITY_DOMAIN:
    InfoType = UR_DEVICE_INFO_PARTITION_AFFINITY_DOMAIN;
    break;
  case PI_DEVICE_INFO_PARTITION_TYPE:
    InfoType = UR_DEVICE_INFO_PARTITION_TYPE;
    break;
  case PI_DEVICE_INFO_OPENCL_C_VERSION:
    InfoType = UR_EXT_DEVICE_INFO_OPENCL_C_VERSION;
    break;
  case PI_DEVICE_INFO_PREFERRED_INTEROP_USER_SYNC:
    InfoType = UR_DEVICE_INFO_PREFERRED_INTEROP_USER_SYNC;
    break;
  case PI_DEVICE_INFO_PRINTF_BUFFER_SIZE:
    InfoType = UR_DEVICE_INFO_PRINTF_BUFFER_SIZE;
    break;
  case PI_DEVICE_INFO_PROFILE:
    InfoType = UR_DEVICE_INFO_PROFILE;
    break;
  case PI_DEVICE_INFO_BUILT_IN_KERNELS:
    InfoType = UR_DEVICE_INFO_BUILT_IN_KERNELS;
    break;
  case PI_DEVICE_INFO_QUEUE_PROPERTIES:
    InfoType = UR_DEVICE_INFO_QUEUE_PROPERTIES;
    break;
  case PI_DEVICE_INFO_EXECUTION_CAPABILITIES:
    InfoType = UR_DEVICE_INFO_EXECUTION_CAPABILITIES;
    break;
  case PI_DEVICE_INFO_ENDIAN_LITTLE:
    InfoType = UR_DEVICE_INFO_ENDIAN_LITTLE;
    break;
  case PI_DEVICE_INFO_ERROR_CORRECTION_SUPPORT:
    InfoType = UR_DEVICE_INFO_ERROR_CORRECTION_SUPPORT;
    break;
  case PI_DEVICE_INFO_PROFILING_TIMER_RESOLUTION:
    InfoType = UR_DEVICE_INFO_PROFILING_TIMER_RESOLUTION;
    break;
  case PI_DEVICE_INFO_LOCAL_MEM_TYPE:
    InfoType = UR_DEVICE_INFO_LOCAL_MEM_TYPE;
    break;
  case PI_DEVICE_INFO_MAX_CONSTANT_ARGS:
    InfoType = UR_DEVICE_INFO_MAX_CONSTANT_ARGS;
    break;
  case PI_DEVICE_INFO_MAX_CONSTANT_BUFFER_SIZE:
    InfoType = UR_DEVICE_INFO_MAX_CONSTANT_BUFFER_SIZE;
    break;
  case PI_DEVICE_INFO_GLOBAL_MEM_CACHE_TYPE:
    InfoType = UR_DEVICE_INFO_GLOBAL_MEM_CACHE_TYPE;
    break;
  case PI_DEVICE_INFO_GLOBAL_MEM_CACHELINE_SIZE:
    InfoType = UR_DEVICE_INFO_GLOBAL_MEM_CACHELINE_SIZE;
    break;
  case PI_DEVICE_INFO_GLOBAL_MEM_CACHE_SIZE:
    InfoType = UR_DEVICE_INFO_GLOBAL_MEM_CACHE_SIZE;
    break;
  case PI_DEVICE_INFO_MAX_PARAMETER_SIZE:
    InfoType = UR_DEVICE_INFO_MAX_PARAMETER_SIZE;
    break;
  case PI_DEVICE_INFO_MEM_BASE_ADDR_ALIGN:
    InfoType = UR_DEVICE_INFO_MEM_BASE_ADDR_ALIGN;
    break;
  case PI_DEVICE_INFO_MAX_SAMPLERS:
    InfoType = UR_DEVICE_INFO_MAX_SAMPLERS;
    break;
  case PI_DEVICE_INFO_MAX_READ_IMAGE_ARGS:
    InfoType = UR_DEVICE_INFO_MAX_READ_IMAGE_ARGS;
    break;
  case PI_DEVICE_INFO_MAX_WRITE_IMAGE_ARGS:
    InfoType = UR_DEVICE_INFO_MAX_WRITE_IMAGE_ARGS;
    break;
  case PI_DEVICE_INFO_SINGLE_FP_CONFIG:
    InfoType = UR_DEVICE_INFO_SINGLE_FP_CONFIG;
    break;
  case PI_DEVICE_INFO_HALF_FP_CONFIG:
    InfoType = UR_DEVICE_INFO_HALF_FP_CONFIG;
    break;
  case PI_DEVICE_INFO_DOUBLE_FP_CONFIG:
    InfoType = UR_DEVICE_INFO_DOUBLE_FP_CONFIG;
    break;
  case PI_DEVICE_INFO_IMAGE2D_MAX_WIDTH:
    InfoType = UR_DEVICE_INFO_IMAGE2D_MAX_WIDTH;
    break;
  case PI_DEVICE_INFO_IMAGE2D_MAX_HEIGHT:
    InfoType = UR_DEVICE_INFO_IMAGE2D_MAX_HEIGHT;
    break;
  case PI_DEVICE_INFO_IMAGE3D_MAX_WIDTH:
    InfoType = UR_DEVICE_INFO_IMAGE3D_MAX_WIDTH;
    break;
  case PI_DEVICE_INFO_IMAGE3D_MAX_HEIGHT:
    InfoType = UR_DEVICE_INFO_IMAGE3D_MAX_HEIGHT;
    break;
  case PI_DEVICE_INFO_IMAGE3D_MAX_DEPTH:
    InfoType = UR_DEVICE_INFO_IMAGE3D_MAX_DEPTH;
    break;
  case PI_DEVICE_INFO_IMAGE_MAX_BUFFER_SIZE:
    InfoType = UR_DEVICE_INFO_IMAGE_MAX_BUFFER_SIZE;
    break;
  case PI_DEVICE_INFO_NATIVE_VECTOR_WIDTH_CHAR:
    InfoType = UR_DEVICE_INFO_NATIVE_VECTOR_WIDTH_CHAR;
    break;
  case PI_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_CHAR:
    InfoType = UR_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_CHAR;
    break;
  case PI_DEVICE_INFO_NATIVE_VECTOR_WIDTH_SHORT:
    InfoType = UR_DEVICE_INFO_NATIVE_VECTOR_WIDTH_SHORT;
    break;
  case PI_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_SHORT:
    InfoType = UR_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_SHORT;
    break;
  case PI_DEVICE_INFO_NATIVE_VECTOR_WIDTH_INT:
    InfoType = UR_DEVICE_INFO_NATIVE_VECTOR_WIDTH_INT;
    break;
  case PI_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_INT:
    InfoType = UR_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_INT;
    break;
  case PI_DEVICE_INFO_NATIVE_VECTOR_WIDTH_LONG:
    InfoType = UR_DEVICE_INFO_NATIVE_VECTOR_WIDTH_LONG;
    break;
  case PI_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_LONG:
    InfoType = UR_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_LONG;
    break;
  case PI_DEVICE_INFO_NATIVE_VECTOR_WIDTH_FLOAT:
    InfoType = UR_DEVICE_INFO_NATIVE_VECTOR_WIDTH_FLOAT;
    break;
  case PI_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_FLOAT:
    InfoType = UR_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_FLOAT;
    break;
  case PI_DEVICE_INFO_NATIVE_VECTOR_WIDTH_DOUBLE:
    InfoType = UR_DEVICE_INFO_NATIVE_VECTOR_WIDTH_DOUBLE;
    break;
  case PI_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_DOUBLE:
    InfoType = UR_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_DOUBLE;
    break;
  case PI_DEVICE_INFO_NATIVE_VECTOR_WIDTH_HALF:
    InfoType = UR_DEVICE_INFO_NATIVE_VECTOR_WIDTH_HALF;
    break;
  case PI_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_HALF:
    InfoType = UR_DEVICE_INFO_PREFERRED_VECTOR_WIDTH_HALF;
    break;
  case PI_DEVICE_INFO_MAX_NUM_SUB_GROUPS:
    InfoType = UR_DEVICE_INFO_MAX_NUM_SUB_GROUPS;
    break;
  case PI_DEVICE_INFO_SUB_GROUP_INDEPENDENT_FORWARD_PROGRESS:
    InfoType = UR_DEVICE_INFO_SUB_GROUP_INDEPENDENT_FORWARD_PROGRESS;
    break;
  case PI_DEVICE_INFO_SUB_GROUP_SIZES_INTEL:
    InfoType = UR_DEVICE_INFO_SUB_GROUP_SIZES_INTEL;
    break;
  case PI_DEVICE_INFO_IL_VERSION:
    InfoType = UR_DEVICE_INFO_IL_VERSION;
    break;
  case PI_DEVICE_INFO_USM_HOST_SUPPORT:
    InfoType = UR_DEVICE_INFO_USM_HOST_SUPPORT;
    break;
  case PI_DEVICE_INFO_USM_DEVICE_SUPPORT:
    InfoType = UR_DEVICE_INFO_USM_DEVICE_SUPPORT;
    break;
  case PI_DEVICE_INFO_USM_SINGLE_SHARED_SUPPORT:
    InfoType = UR_DEVICE_INFO_USM_SINGLE_SHARED_SUPPORT;
    break;
  case PI_DEVICE_INFO_USM_CROSS_SHARED_SUPPORT:
    InfoType = UR_DEVICE_INFO_USM_CROSS_SHARED_SUPPORT;
    break;
  case PI_DEVICE_INFO_USM_SYSTEM_SHARED_SUPPORT:
    InfoType = UR_DEVICE_INFO_USM_SYSTEM_SHARED_SUPPORT;
    break;
  case PI_DEVICE_INFO_PCI_ADDRESS:
    InfoType = UR_DEVICE_INFO_PCI_ADDRESS;
    break;
  case PI_DEVICE_INFO_GPU_EU_COUNT:
    InfoType = UR_DEVICE_INFO_GPU_EU_COUNT;
    break;
  case PI_DEVICE_INFO_GPU_EU_SIMD_WIDTH:
    InfoType = UR_DEVICE_INFO_GPU_EU_SIMD_WIDTH;
    break;
  case PI_DEVICE_INFO_GPU_SUBSLICES_PER_SLICE:
    InfoType = UR_DEVICE_INFO_GPU_SUBSLICES_PER_SLICE;
    break;
  case PI_DEVICE_INFO_BUILD_ON_SUBDEVICE:
    InfoType = UR_DEVICE_INFO_BUILD_ON_SUBDEVICE;
    break;
  case PI_EXT_ONEAPI_DEVICE_INFO_MAX_WORK_GROUPS_3D:
    InfoType = UR_DEVICE_INFO_MAX_WORK_GROUPS_3D;
    break;
  case PI_DEVICE_INFO_IMAGE_MAX_ARRAY_SIZE:
    InfoType = UR_DEVICE_INFO_IMAGE_MAX_ARRAY_SIZE;
    break;
  case PI_DEVICE_INFO_DEVICE_ID:
    InfoType = UR_DEVICE_INFO_DEVICE_ID;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_FREE_MEMORY:
    InfoType = UR_DEVICE_INFO_GLOBAL_MEM_FREE;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_MEMORY_CLOCK_RATE:
    InfoType = UR_DEVICE_INFO_MEMORY_CLOCK_RATE;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_MEMORY_BUS_WIDTH:
    InfoType = UR_DEVICE_INFO_MEMORY_BUS_WIDTH;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_MAX_COMPUTE_QUEUE_INDICES:
    InfoType = UR_DEVICE_INFO_MAX_COMPUTE_QUEUE_INDICES;
    break;
  case PI_DEVICE_INFO_GPU_SLICES:
    InfoType = UR_DEVICE_INFO_GPU_EU_SLICES;
    break;
  case PI_DEVICE_INFO_GPU_EU_COUNT_PER_SUBSLICE:
    InfoType = UR_DEVICE_INFO_GPU_EU_COUNT_PER_SUBSLICE;
    break;
  case PI_DEVICE_INFO_GPU_HW_THREADS_PER_EU:
    InfoType = UR_DEVICE_INFO_GPU_HW_THREADS_PER_EU;
    break;
  case PI_DEVICE_INFO_MAX_MEM_BANDWIDTH:
    InfoType = UR_DEVICE_INFO_MAX_MEMORY_BANDWIDTH;
    break;
  case PI_EXT_ONEAPI_DEVICE_INFO_BFLOAT16_MATH_FUNCTIONS:
    InfoType = UR_DEVICE_INFO_BFLOAT16;
    break;
  case PI_EXT_DEVICE_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES:
    InfoType = UR_DEVICE_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES;
    break;
  case PI_EXT_DEVICE_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES:
    InfoType = UR_DEVICE_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES;
    break;
  case PI_EXT_DEVICE_INFO_ATOMIC_FENCE_ORDER_CAPABILITIES:
    InfoType = UR_DEVICE_INFO_ATOMIC_FENCE_ORDER_CAPABILITIES;
    break;
  case PI_EXT_DEVICE_INFO_ATOMIC_FENCE_SCOPE_CAPABILITIES:
    InfoType = UR_DEVICE_INFO_ATOMIC_FENCE_SCOPE_CAPABILITIES;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_MEM_CHANNEL_SUPPORT:
    InfoType = UR_DEVICE_INFO_MEM_CHANNEL_SUPPORT;
    break;
  case PI_DEVICE_INFO_IMAGE_SRGB:
    InfoType = UR_DEVICE_INFO_IMAGE_SRGB;
    break;
  case PI_DEVICE_INFO_BACKEND_VERSION: {
    InfoType = UR_DEVICE_INFO_BACKEND_RUNTIME_VERSION;
    break;
  }
  case PI_EXT_ONEAPI_DEVICE_INFO_CUDA_ASYNC_BARRIER:
    InfoType = UR_DEVICE_INFO_ASYNC_BARRIER;
    break;
  default:
    return PI_ERROR_UNKNOWN;
  };

  size_t UrParamValueSizeRet;
  auto hDevice = reinterpret_cast<ur_device_handle_t>(Device);
  HANDLE_ERRORS(urDeviceGetInfo(hDevice, InfoType, ParamValueSize, ParamValue,
                                &UrParamValueSizeRet));
  if (ParamValueSizeRet) {
    *ParamValueSizeRet = UrParamValueSizeRet;
  }

  ur2piDeviceInfoValue(InfoType, ParamValueSize, &ParamValueSize, ParamValue);
  fixupInfoValueTypes(UrParamValueSizeRet, ParamValueSizeRet, ParamValueSize,
                      ParamValue);

  return PI_SUCCESS;
}

inline pi_result piDevicePartition(
    pi_device Device, const pi_device_partition_property *Properties,
    pi_uint32 NumEntries, pi_device *SubDevices, pi_uint32 *NumSubDevices) {

  if (!Properties || !Properties[0])
    return PI_ERROR_INVALID_VALUE;

  ur_device_partition_property_t Property;
  switch (Properties[0]) {
  case PI_DEVICE_PARTITION_EQUALLY:
    Property = UR_DEVICE_PARTITION_EQUALLY;
    break;
  case PI_DEVICE_PARTITION_BY_COUNTS:
    Property = UR_DEVICE_PARTITION_BY_COUNTS;
    break;
  case PI_DEVICE_PARTITION_BY_AFFINITY_DOMAIN:
    Property = UR_DEVICE_PARTITION_BY_AFFINITY_DOMAIN;
    break;
  case PI_EXT_INTEL_DEVICE_PARTITION_BY_CSLICE:
    Property = UR_DEVICE_PARTITION_BY_CSLICE;
    break;
  default:
    return PI_ERROR_UNKNOWN;
  }

  // Some partitioning types require a value
  auto Value = uint32_t(Properties[1]);
  if (Property == UR_DEVICE_PARTITION_BY_AFFINITY_DOMAIN) {
    switch (Properties[1]) {
    case PI_DEVICE_AFFINITY_DOMAIN_NUMA:
      Value = UR_DEVICE_AFFINITY_DOMAIN_FLAG_NUMA;
      break;
    case PI_DEVICE_AFFINITY_DOMAIN_NEXT_PARTITIONABLE:
      Value = UR_DEVICE_AFFINITY_DOMAIN_FLAG_NEXT_PARTITIONABLE;
      break;
    default:
      return PI_ERROR_UNKNOWN;
    }
  }

  // Translate partitioning properties from PI-way
  // (array of uintptr_t values) to UR-way
  // (array of {uint32_t, uint32_t} pairs)
  //
  // TODO: correctly terminate the UR properties, see:
  // https://github.com/oneapi-src/unified-runtime/issues/183
  //
  ur_device_partition_property_t UrProperties[] = {
      ur_device_partition_property_t(Property), Value, 0};

  auto hDevice = reinterpret_cast<ur_device_handle_t>(Device);
  auto phSubDevices = reinterpret_cast<ur_device_handle_t *>(SubDevices);
  HANDLE_ERRORS(urDevicePartition(hDevice, UrProperties, NumEntries,
                                  phSubDevices, NumSubDevices));
  return PI_SUCCESS;
}

inline pi_result piGetDeviceAndHostTimer(pi_device Device, uint64_t *DeviceTime,
                                         uint64_t *HostTime) {
  auto UrDevice = reinterpret_cast<ur_device_handle_t>(Device);
  HANDLE_ERRORS(urDeviceGetGlobalTimestamps(UrDevice, DeviceTime, HostTime));
  return PI_SUCCESS;
}

inline pi_result piextEnqueueDeviceGlobalVariableWrite(
    pi_queue Queue, pi_program Program, const char *Name, pi_bool BlockingWrite,
    size_t Count, size_t Offset, const void *Src, pi_uint32 NumEventsInWaitList,
    const pi_event *EventsWaitList, pi_event *OutEvent) {
  ur_queue_handle_t UrQueue = reinterpret_cast<ur_queue_handle_t>(Queue);
  ur_program_handle_t UrProgram =
      reinterpret_cast<ur_program_handle_t>(Program);
  const ur_event_handle_t *UrEventsWaitList =
      reinterpret_cast<const ur_event_handle_t *>(EventsWaitList);
  ur_event_handle_t *UrEvent = reinterpret_cast<ur_event_handle_t *>(OutEvent);
  HANDLE_ERRORS(urEnqueueDeviceGlobalVariableWrite(
      UrQueue, UrProgram, Name, BlockingWrite, Count, Offset, Src,
      NumEventsInWaitList, UrEventsWaitList, UrEvent));

  return PI_SUCCESS;
}

inline pi_result piextEnqueueDeviceGlobalVariableRead(
    pi_queue Queue, pi_program Program, const char *Name, pi_bool BlockingRead,
    size_t Count, size_t Offset, void *Dst, pi_uint32 NumEventsInWaitList,
    const pi_event *EventsWaitList, pi_event *OutEvent) {
  ur_queue_handle_t UrQueue = reinterpret_cast<ur_queue_handle_t>(Queue);
  ur_program_handle_t UrProgram =
      reinterpret_cast<ur_program_handle_t>(Program);
  const ur_event_handle_t *UrEventsWaitList =
      reinterpret_cast<const ur_event_handle_t *>(EventsWaitList);

  ur_event_handle_t *UrEvent = reinterpret_cast<ur_event_handle_t *>(OutEvent);

  HANDLE_ERRORS(urEnqueueDeviceGlobalVariableRead(
      UrQueue, UrProgram, Name, BlockingRead, Count, Offset, Dst,
      NumEventsInWaitList, UrEventsWaitList, UrEvent));

  return PI_SUCCESS;
}

inline pi_result piextDeviceSelectBinary(pi_device Device,
                                         pi_device_binary *Binaries,
                                         pi_uint32 NumBinaries,
                                         pi_uint32 *SelectedBinaryInd) {

  auto UrDevice = reinterpret_cast<ur_device_handle_t>(Device);
  std::vector<ur_device_binary_t> UrBinaries(NumBinaries);

  for (uint32_t BinaryCount = 0; BinaryCount < NumBinaries; BinaryCount++) {
    if (strcmp(Binaries[BinaryCount]->DeviceTargetSpec,
               __SYCL_PI_DEVICE_BINARY_TARGET_UNKNOWN) == 0)
      UrBinaries[BinaryCount].pDeviceTargetSpec =
          UR_DEVICE_BINARY_TARGET_UNKNOWN;
    else if (strcmp(Binaries[BinaryCount]->DeviceTargetSpec,
                    __SYCL_PI_DEVICE_BINARY_TARGET_SPIRV32) == 0)
      UrBinaries[BinaryCount].pDeviceTargetSpec =
          UR_DEVICE_BINARY_TARGET_SPIRV32;
    else if (strcmp(Binaries[BinaryCount]->DeviceTargetSpec,
                    __SYCL_PI_DEVICE_BINARY_TARGET_SPIRV64) == 0)
      UrBinaries[BinaryCount].pDeviceTargetSpec =
          UR_DEVICE_BINARY_TARGET_SPIRV64;
    else if (strcmp(Binaries[BinaryCount]->DeviceTargetSpec,
                    __SYCL_PI_DEVICE_BINARY_TARGET_SPIRV64_X86_64) == 0)
      UrBinaries[BinaryCount].pDeviceTargetSpec =
          UR_DEVICE_BINARY_TARGET_SPIRV64_X86_64;
    else if (strcmp(Binaries[BinaryCount]->DeviceTargetSpec,
                    __SYCL_PI_DEVICE_BINARY_TARGET_SPIRV64_GEN) == 0)
      UrBinaries[BinaryCount].pDeviceTargetSpec =
          UR_DEVICE_BINARY_TARGET_SPIRV64_GEN;
    else if (strcmp(Binaries[BinaryCount]->DeviceTargetSpec,
                    __SYCL_PI_DEVICE_BINARY_TARGET_SPIRV64_FPGA) == 0)
      UrBinaries[BinaryCount].pDeviceTargetSpec =
          UR_DEVICE_BINARY_TARGET_SPIRV64_FPGA;
    else if (strcmp(Binaries[BinaryCount]->DeviceTargetSpec,
                    __SYCL_PI_DEVICE_BINARY_TARGET_NVPTX64) == 0)
      UrBinaries[BinaryCount].pDeviceTargetSpec =
          UR_DEVICE_BINARY_TARGET_NVPTX64;
    else if (strcmp(Binaries[BinaryCount]->DeviceTargetSpec,
                    __SYCL_PI_DEVICE_BINARY_TARGET_AMDGCN) == 0)
      UrBinaries[BinaryCount].pDeviceTargetSpec =
          UR_DEVICE_BINARY_TARGET_AMDGCN;
  }

  HANDLE_ERRORS(urDeviceSelectBinary(UrDevice, UrBinaries.data(), NumBinaries,
                                     SelectedBinaryInd));
  return PI_SUCCESS;
}

inline pi_result piextGetDeviceFunctionPointer(pi_device Device,
                                               pi_program Program,
                                               const char *FunctionName,
                                               pi_uint64 *FunctionPointerRet) {

  PI_ASSERT(Program, PI_ERROR_INVALID_PROGRAM);

  auto UrDevice = reinterpret_cast<ur_device_handle_t>(Device);

  ur_program_handle_t UrProgram =
      reinterpret_cast<ur_program_handle_t>(Program);

  void **FunctionPointer = reinterpret_cast<void **>(FunctionPointerRet);

  HANDLE_ERRORS(urProgramGetFunctionPointer(UrDevice, UrProgram, FunctionName,
                                            FunctionPointer));
  return PI_SUCCESS;
}

// Device
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Context
inline pi_result piContextCreate(const pi_context_properties *properties,
                                 pi_uint32 num_devices,
                                 const pi_device *devices,
                                 void (*pfn_notify)(const char *errinfo,
                                                    const void *private_info,
                                                    size_t cb, void *user_data),
                                 void *user_data, pi_context *retcontext) {
  // TODO: Implement callback (if needed)
  auto hDevices = reinterpret_cast<const ur_device_handle_t *>(devices);
  auto phContext = reinterpret_cast<ur_context_handle_t *>(retcontext);

  HANDLE_ERRORS(urContextCreate(num_devices, hDevices, nullptr, phContext));

  return PI_SUCCESS;
}

inline pi_result piContextGetInfo(pi_context context,
                                  pi_context_info param_name,
                                  size_t param_value_size, void *param_value,
                                  size_t *param_value_size_ret) {
  static std::unordered_map<pi_context_info, ur_context_info_t> InfoMapping = {
      {PI_CONTEXT_INFO_NUM_DEVICES, UR_CONTEXT_INFO_NUM_DEVICES},
      {PI_CONTEXT_INFO_DEVICES, UR_CONTEXT_INFO_DEVICES},
      {PI_CONTEXT_INFO_REFERENCE_COUNT, UR_CONTEXT_INFO_REFERENCE_COUNT},
      {PI_EXT_CONTEXT_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES,
       UR_CONTEXT_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES},
      {PI_EXT_CONTEXT_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES,
       UR_CONTEXT_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES},
      {PI_EXT_ONEAPI_CONTEXT_INFO_USM_MEMCPY2D_SUPPORT,
       UR_CONTEXT_INFO_USM_MEMCPY2D_SUPPORT},
      {PI_EXT_ONEAPI_CONTEXT_INFO_USM_FILL2D_SUPPORT,
       UR_CONTEXT_INFO_USM_FILL2D_SUPPORT}
  };

  // Special case as UR does not have USM memset2d, so it is never supported.
  // It was not implemented by any PI plugin anyway.
  if (param_name == PI_EXT_ONEAPI_CONTEXT_INFO_USM_MEMSET2D_SUPPORT) {
    if (param_value_size_ret) {
      *param_value_size_ret = 4;
    }
    if (param_value) {
      *(static_cast<uint32_t *>(param_value)) = 0u;
    }
    return PI_SUCCESS;
  }

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  size_t UrParamValueSizeRet;
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  HANDLE_ERRORS(urContextGetInfo(hContext, InfoType->second, param_value_size,
                                 param_value, &UrParamValueSizeRet));

  if (param_value_size_ret) {
    *param_value_size_ret = UrParamValueSizeRet;
  }

  fixupInfoValueTypes(UrParamValueSizeRet, param_value_size_ret,
                      param_value_size, param_value);

  return PI_SUCCESS;
}

inline pi_result piContextRetain(pi_context context) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  HANDLE_ERRORS(urContextRetain(hContext));

  return PI_SUCCESS;
}

inline pi_result piContextRelease(pi_context context) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  HANDLE_ERRORS(urContextRelease(hContext));

  return PI_SUCCESS;
}

inline pi_result piextDeviceGetNativeHandle(pi_device device,
                                            pi_native_handle *nativeHandle) {
  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);
  auto phNativeHandle = reinterpret_cast<ur_native_handle_t *>(nativeHandle);

  HANDLE_ERRORS(urDeviceGetNativeHandle(hDevice, phNativeHandle));

  return PI_SUCCESS;
}

inline pi_result
piextDeviceCreateWithNativeHandle(pi_native_handle nativeHandle,
                                  pi_platform platform, pi_device *device) {
  auto hNativeHandle = reinterpret_cast<ur_native_handle_t>(nativeHandle);
  auto hPlatform = reinterpret_cast<ur_platform_handle_t>(platform);
  auto phDevice = reinterpret_cast<ur_device_handle_t *>(device);

  HANDLE_ERRORS(
      urDeviceCreateWithNativeHandle(hNativeHandle, hPlatform, phDevice));

  return PI_SUCCESS;
}

inline pi_result piextContextSetExtendedDeleter(
    pi_context context, pi_context_extended_deleter func, void *user_data) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);

  HANDLE_ERRORS(urContextSetExtendedDeleter(hContext, func, user_data));

  return PI_SUCCESS;
}

inline pi_result piextContextGetNativeHandle(pi_context context,
                                             pi_native_handle *nativeHandle) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto phNativeHandle = reinterpret_cast<ur_native_handle_t *>(nativeHandle);

  HANDLE_ERRORS(urContextGetNativeHandle(hContext, phNativeHandle));

  return PI_SUCCESS;
}

inline pi_result piextContextCreateWithNativeHandle(
    pi_native_handle nativeHandle, pi_uint32 numDevices,
    const pi_device *devices, bool pluginOwnsNativeHandle,
    pi_context *context) {

  auto hNativeHandle = reinterpret_cast<ur_native_handle_t>(nativeHandle);
  auto phContext = reinterpret_cast<ur_context_handle_t *>(context);
  auto phDevices = reinterpret_cast<const ur_device_handle_t *>(devices);

  ur_context_native_properties_t properties{
      UR_STRUCTURE_TYPE_CONTEXT_NATIVE_PROPERTIES, nullptr,
      pluginOwnsNativeHandle};

  HANDLE_ERRORS(urContextCreateWithNativeHandle(
      hNativeHandle, numDevices, phDevices, &properties, phContext));

  return PI_SUCCESS;
}
// Context
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Program
inline pi_result piProgramCreate(pi_context, const void *, size_t,
                                 pi_program *) {
  die("piProgramCreate not implemented");
  return {};
}

inline pi_result piclProgramCreateWithSource(pi_context context,
                                             pi_uint32 count,
                                             const char **strings,
                                             const size_t *lengths,
                                             pi_program *ret_program) {
  die("piclProgramCreateWithSource doesn't have a mapping in UR");
  return {};
}

inline ur_result_t
mapPIMetadataToUR(const pi_device_binary_property *pi_metadata,
                  ur_program_metadata_t *ur_metadata) {
  ur_metadata->pName = (*pi_metadata)->Name;
  ur_metadata->size = (*pi_metadata)->ValSize;
  switch ((*pi_metadata)->Type) {
  case PI_PROPERTY_TYPE_UINT32:
    ur_metadata->type = UR_PROGRAM_METADATA_TYPE_UINT32;
    ur_metadata->value.data32 = (*pi_metadata)->ValSize;
    return UR_RESULT_SUCCESS;
  case PI_PROPERTY_TYPE_BYTE_ARRAY:
    ur_metadata->type = UR_PROGRAM_METADATA_TYPE_BYTE_ARRAY;
    ur_metadata->value.pData = (*pi_metadata)->ValAddr;
    return UR_RESULT_SUCCESS;
  case PI_PROPERTY_TYPE_STRING:
    ur_metadata->type = UR_PROGRAM_METADATA_TYPE_STRING;
    ur_metadata->value.pString =
        reinterpret_cast<char *>((*pi_metadata)->ValAddr);
    return UR_RESULT_SUCCESS;
  default:
    return UR_RESULT_ERROR_INVALID_VALUE;
  }
}

inline pi_result piProgramCreateWithBinary(
    pi_context context, pi_uint32 num_devices, const pi_device *device_list,
    const size_t *lengths, const unsigned char **binaries,
    size_t num_metadata_entries, const pi_device_binary_property *metadata,
    pi_int32 *binary_status, pi_program *program) {
  (void)binary_status;
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto hDevice = reinterpret_cast<ur_device_handle_t>(device_list[0]);
  auto hProgram = reinterpret_cast<ur_program_handle_t *>(program);

  std::unique_ptr<ur_program_metadata_t[]> pMetadatas(
      new ur_program_metadata_t[num_metadata_entries]);
  for (unsigned i = 0; i < num_metadata_entries; i++) {
    HANDLE_ERRORS(mapPIMetadataToUR(&metadata[i], &pMetadatas[i]));
  }

  ur_program_properties_t pProperties;
  pProperties.stype = UR_STRUCTURE_TYPE_PROGRAM_PROPERTIES;
  pProperties.pNext = nullptr;
  pProperties.count = num_metadata_entries;
  pProperties.pMetadatas = pMetadatas.get();

  HANDLE_ERRORS(urProgramCreateWithBinary(hContext, hDevice, lengths[0],
                                          binaries[0], &pProperties, hProgram));

  return PI_SUCCESS;
}

inline pi_result piProgramGetInfo(pi_program program,
                                  pi_program_info param_name,
                                  size_t param_value_size, void *param_value,
                                  size_t *param_value_size_ret) {
  static std::unordered_map<pi_program_info, ur_program_info_t> InfoMapping = {
      {PI_PROGRAM_INFO_REFERENCE_COUNT, UR_PROGRAM_INFO_REFERENCE_COUNT},
      {PI_PROGRAM_INFO_CONTEXT, UR_PROGRAM_INFO_CONTEXT},
      {PI_PROGRAM_INFO_NUM_DEVICES, UR_PROGRAM_INFO_NUM_DEVICES},
      {PI_PROGRAM_INFO_DEVICES, UR_PROGRAM_INFO_DEVICES},
      {PI_PROGRAM_INFO_SOURCE, UR_PROGRAM_INFO_SOURCE},
      {PI_PROGRAM_INFO_BINARY_SIZES, UR_PROGRAM_INFO_BINARY_SIZES},
      {PI_PROGRAM_INFO_BINARIES, UR_PROGRAM_INFO_BINARIES},
      {PI_PROGRAM_INFO_NUM_KERNELS, UR_PROGRAM_INFO_NUM_KERNELS},
      {PI_PROGRAM_INFO_KERNEL_NAMES, UR_PROGRAM_INFO_KERNEL_NAMES},
  };

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  HANDLE_ERRORS(urProgramGetInfo(hProgram, InfoType->second, param_value_size,
                                 param_value, param_value_size_ret));

  return PI_SUCCESS;
}

inline pi_result piProgramCompile(
    pi_program program, pi_uint32 num_devices, const pi_device *device_list,
    const char *options, pi_uint32 num_input_headers,
    const pi_program *input_headers, const char **header_include_names,
    void (*pfn_notify)(pi_program program, void *user_data), void *user_data) {
  (void)num_devices;
  (void)device_list;
  (void)num_input_headers;
  (void)input_headers;
  (void)header_include_names;
  (void)pfn_notify;

  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  ur_context_handle_t hContext{};

  HANDLE_ERRORS(urProgramCompile(hContext, hProgram, options));

  return PI_SUCCESS;
}

inline pi_result
piProgramBuild(pi_program program, pi_uint32 num_devices,
               const pi_device *device_list, const char *options,
               void (*pfn_notify)(pi_program program, void *user_data),
               void *user_data) {
  (void)num_devices;
  (void)device_list;
  (void)pfn_notify;

  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  ur_context_handle_t hContext{};

  HANDLE_ERRORS(urProgramBuild(hContext, hProgram, options));

  return PI_SUCCESS;
}

inline pi_result
piProgramLink(pi_context context, pi_uint32 num_devices,
              const pi_device *device_list, const char *options,
              pi_uint32 num_input_programs, const pi_program *input_programs,
              void (*pfn_notify)(pi_program program, void *user_data),
              void *user_data, pi_program *ret_program) {
  (void)num_devices;
  (void)device_list;
  (void)pfn_notify;
  (void)user_data;

  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto phPrograms =
      reinterpret_cast<const ur_program_handle_t *>(input_programs);
  auto phProgram = reinterpret_cast<ur_program_handle_t *>(ret_program);

  HANDLE_ERRORS(urProgramLink(hContext, num_input_programs, phPrograms, options,
                              phProgram));

  return PI_SUCCESS;
}

inline pi_result piProgramGetBuildInfo(pi_program program, pi_device device,
                                       pi_program_build_info param_name,
                                       size_t param_value_size,
                                       void *param_value,
                                       size_t *param_value_size_ret) {
  static std::unordered_map<pi_program_build_info, ur_program_build_info_t>
      InfoMapping = {
          {PI_PROGRAM_BUILD_INFO_STATUS, UR_PROGRAM_BUILD_INFO_STATUS},
          {PI_PROGRAM_BUILD_INFO_OPTIONS, UR_PROGRAM_BUILD_INFO_OPTIONS},
          {PI_PROGRAM_BUILD_INFO_LOG, UR_PROGRAM_BUILD_INFO_LOG}};

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);

  HANDLE_ERRORS(urProgramGetBuildInfo(hProgram, hDevice, InfoType->second,
                                      param_value_size, param_value,
                                      param_value_size_ret));

  return PI_SUCCESS;
}

inline pi_result piProgramRetain(pi_program program) {
  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  HANDLE_ERRORS(urProgramRetain(hProgram));

  return PI_SUCCESS;
}

inline pi_result piProgramRelease(pi_program program) {
  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  HANDLE_ERRORS(urProgramRelease(hProgram));

  return PI_SUCCESS;
}

inline pi_result piextProgramGetNativeHandle(pi_program program,
                                             pi_native_handle *nativeHandle) {
  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  auto phNativeHandle = reinterpret_cast<ur_native_handle_t *>(nativeHandle);
  HANDLE_ERRORS(urProgramGetNativeHandle(hProgram, phNativeHandle));

  return PI_SUCCESS;
}

inline pi_result
piextProgramCreateWithNativeHandle(pi_native_handle nativeHandle,
                                   pi_context context, bool ownNativeHandle,
                                   pi_program *program) {
  (void)ownNativeHandle;

  auto hNativeProgram = reinterpret_cast<ur_native_handle_t>(nativeHandle);
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto phProgram = reinterpret_cast<ur_program_handle_t *>(program);

  HANDLE_ERRORS(
      urProgramCreateWithNativeHandle(hNativeProgram, hContext, phProgram));

  return PI_SUCCESS;
}

inline pi_result piextProgramSetSpecializationConstant(pi_program program,
                                                       pi_uint32 spec_id,
                                                       size_t spec_size,
                                                       const void *spec_value) {
  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);

  ur_specialization_constant_info_t pSpecConstant{};
  pSpecConstant.id = spec_id;
  pSpecConstant.size = spec_size;
  pSpecConstant.pValue = spec_value;

  HANDLE_ERRORS(
      urProgramSetSpecializationConstants(hProgram, 1, &pSpecConstant));

  return PI_SUCCESS;
}
// Program
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Kernel
inline pi_result piKernelCreate(pi_program program, const char *kernel_name,
                                pi_kernel *kernel) {
  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  auto phKernel = reinterpret_cast<ur_kernel_handle_t *>(kernel);
  HANDLE_ERRORS(urKernelCreate(hProgram, kernel_name, phKernel));

  return PI_SUCCESS;
}

inline pi_result piKernelSetArg(pi_kernel kernel, pi_uint32 arg_index,
                                size_t arg_size, const void *arg_value) {
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  HANDLE_ERRORS(urKernelSetArgValue(hKernel, arg_index, arg_size, arg_value));

  return PI_SUCCESS;
}

inline pi_result piKernelGetInfo(pi_kernel kernel, pi_kernel_info param_name,
                                 size_t param_value_size, void *param_value,
                                 size_t *param_value_size_ret) {
  static std::unordered_map<pi_kernel_info, ur_kernel_info_t> InfoMapping = {
      {PI_KERNEL_INFO_FUNCTION_NAME, UR_KERNEL_INFO_FUNCTION_NAME},
      {PI_KERNEL_INFO_NUM_ARGS, UR_KERNEL_INFO_NUM_ARGS},
      {PI_KERNEL_INFO_REFERENCE_COUNT, UR_KERNEL_INFO_REFERENCE_COUNT},
      {PI_KERNEL_INFO_CONTEXT, UR_KERNEL_INFO_CONTEXT},
      {PI_KERNEL_INFO_PROGRAM, UR_KERNEL_INFO_PROGRAM},
      {PI_KERNEL_INFO_ATTRIBUTES, UR_KERNEL_INFO_ATTRIBUTES}};

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  HANDLE_ERRORS(urKernelGetInfo(hKernel, InfoType->second, param_value_size,
                                param_value, param_value_size_ret));

  return PI_SUCCESS;
}

inline pi_result piKernelGetGroupInfo(pi_kernel kernel, pi_device device,
                                      pi_kernel_group_info param_name,
                                      size_t param_value_size,
                                      void *param_value,
                                      size_t *param_value_size_ret) {
  static std::unordered_map<pi_kernel_group_info, ur_kernel_group_info_t>
      InfoMapping = {
          {PI_KERNEL_GROUP_INFO_GLOBAL_WORK_SIZE,
           UR_KERNEL_GROUP_INFO_GLOBAL_WORK_SIZE},
          {PI_KERNEL_GROUP_INFO_WORK_GROUP_SIZE,
           UR_KERNEL_GROUP_INFO_WORK_GROUP_SIZE},
          {PI_KERNEL_GROUP_INFO_COMPILE_WORK_GROUP_SIZE,
           UR_KERNEL_GROUP_INFO_COMPILE_WORK_GROUP_SIZE},
          {PI_KERNEL_GROUP_INFO_LOCAL_MEM_SIZE,
           UR_KERNEL_GROUP_INFO_LOCAL_MEM_SIZE},
          {PI_KERNEL_GROUP_INFO_PREFERRED_WORK_GROUP_SIZE_MULTIPLE,
           UR_KERNEL_GROUP_INFO_PREFERRED_WORK_GROUP_SIZE_MULTIPLE},
          {PI_KERNEL_GROUP_INFO_PRIVATE_MEM_SIZE,
           UR_KERNEL_GROUP_INFO_PRIVATE_MEM_SIZE},
      };

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  HANDLE_ERRORS(urKernelGetGroupInfo(hKernel, hDevice, InfoType->second,
                                     param_value_size, param_value,
                                     param_value_size_ret));

  return PI_SUCCESS;
}

inline pi_result piKernelGetSubGroupInfo(
    pi_kernel kernel, pi_device device, pi_kernel_sub_group_info param_name,
    size_t input_value_size, const void *input_value, size_t param_value_size,
    void *param_value, size_t *param_value_size_ret) {
  // Ignore unused parameters
  (void)input_value_size;
  (void)input_value;

  static std::unordered_map<pi_kernel_sub_group_info,
                            ur_kernel_sub_group_info_t>
      InfoMapping = {{PI_KERNEL_MAX_SUB_GROUP_SIZE,
                      UR_KERNEL_SUB_GROUP_INFO_MAX_SUB_GROUP_SIZE},
                     {PI_KERNEL_MAX_NUM_SUB_GROUPS,
                      UR_KERNEL_SUB_GROUP_INFO_MAX_NUM_SUB_GROUPS},
                     {PI_KERNEL_COMPILE_NUM_SUB_GROUPS,
                      UR_KERNEL_SUB_GROUP_INFO_COMPILE_NUM_SUB_GROUPS},
                     {PI_KERNEL_COMPILE_SUB_GROUP_SIZE_INTEL,
                      UR_KERNEL_SUB_GROUP_INFO_SUB_GROUP_SIZE_INTEL}};

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  HANDLE_ERRORS(urKernelGetSubGroupInfo(hKernel, hDevice, InfoType->second,
                                        param_value_size, param_value,
                                        param_value_size_ret));

  return PI_SUCCESS;
}

inline pi_result piKernelRetain(pi_kernel kernel) {
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  HANDLE_ERRORS(urKernelRetain(hKernel));

  return PI_SUCCESS;
}

inline pi_result piKernelRelease(pi_kernel kernel) {
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  HANDLE_ERRORS(urKernelRelease(hKernel));

  return PI_SUCCESS;
}

inline pi_result piKernelSetExecInfo(pi_kernel kernel,
                                     pi_kernel_exec_info param_name,
                                     size_t param_value_size,
                                     const void *param_value) {
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);

  static std::unordered_map<pi_kernel_exec_info, ur_kernel_exec_info_t>
      InfoMapping = {
          {PI_USM_INDIRECT_ACCESS, UR_KERNEL_EXEC_INFO_USM_INDIRECT_ACCESS},
          {PI_USM_PTRS, UR_KERNEL_EXEC_INFO_USM_PTRS}};

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  HANDLE_ERRORS(urKernelSetExecInfo(hKernel, InfoType->second, param_value_size,
                                    param_value));

  return PI_SUCCESS;
}

inline pi_result piKernelSetArgPointer(pi_kernel kernel, pi_uint32 arg_index,
                                       size_t arg_size, const void *arg_value) {
  (void)arg_size;
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  HANDLE_ERRORS(urKernelSetArgPointer(hKernel, arg_index, arg_value));

  return PI_SUCCESS;
}

inline pi_result piextKernelSetArgMemObj(pi_kernel kernel, pi_uint32 arg_index,
                                         const pi_mem *arg_value) {
  UR_ASSERT(arg_value, PI_ERROR_INVALID_VALUE);

  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  auto hBuffer = reinterpret_cast<const ur_mem_handle_t *>(arg_value);

  HANDLE_ERRORS(urKernelSetArgMemObj(hKernel, arg_index, *hBuffer));

  return PI_SUCCESS;
}

inline pi_result piKernelGetNativeHandle(pi_kernel kernel,
                                         pi_native_handle *nativeHandle) {
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  auto phNativeHandle = reinterpret_cast<ur_native_handle_t *>(nativeHandle);
  HANDLE_ERRORS(urKernelGetNativeHandle(hKernel, phNativeHandle));

  return PI_SUCCESS;
}

inline pi_result
piextKernelCreateWithNativeHandle(pi_native_handle native_handle,
                                  pi_context context, pi_program program,
                                  bool own_native_handle, pi_kernel *kernel) {
  (void)program;
  (void)own_native_handle;

  auto hNativeKernel = reinterpret_cast<ur_native_handle_t>(native_handle);
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  auto phKernel = reinterpret_cast<ur_kernel_handle_t *>(kernel);

  ur_kernel_native_properties_t properties{
      UR_STRUCTURE_TYPE_KERNEL_NATIVE_PROPERTIES, nullptr, own_native_handle};

  HANDLE_ERRORS(urKernelCreateWithNativeHandle(
      hNativeKernel, hContext, hProgram, &properties, phKernel));

  return PI_SUCCESS;
}

inline pi_result piextKernelSetArgSampler(pi_kernel kernel, pi_uint32 arg_index,
                                          const pi_sampler *arg_value) {
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  auto phArgValue = reinterpret_cast<const ur_sampler_handle_t *>(arg_value);

  HANDLE_ERRORS(urKernelSetArgSampler(hKernel, arg_index, *phArgValue));

  return PI_SUCCESS;
}
// Kernel
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Queue
inline ur_queue_flags_t ConvertQueueFlagsBitfield(pi_queue_properties flags) {
  ur_queue_flags_t Flags = 0;
  static std::unordered_map<pi_queue_properties, ur_queue_flag_t> FlagMap = {
      {PI_QUEUE_FLAG_OUT_OF_ORDER_EXEC_MODE_ENABLE,
       UR_QUEUE_FLAG_OUT_OF_ORDER_EXEC_MODE_ENABLE},
      {PI_QUEUE_FLAG_PROFILING_ENABLE, UR_QUEUE_FLAG_PROFILING_ENABLE},
      {PI_QUEUE_FLAG_ON_DEVICE, UR_QUEUE_FLAG_ON_DEVICE},
      {PI_QUEUE_FLAG_ON_DEVICE_DEFAULT, UR_QUEUE_FLAG_ON_DEVICE_DEFAULT},
      {PI_EXT_ONEAPI_QUEUE_FLAG_DISCARD_EVENTS, UR_QUEUE_FLAG_DISCARD_EVENTS},
      {PI_EXT_ONEAPI_QUEUE_FLAG_PRIORITY_LOW, UR_QUEUE_FLAG_PRIORITY_LOW},
      {PI_EXT_ONEAPI_QUEUE_FLAG_PRIORITY_HIGH, UR_QUEUE_FLAG_PRIORITY_HIGH},
  };
  for (auto &FlagPair : FlagMap) {
    if (flags & FlagPair.first) {
      Flags |= FlagPair.second;
    }
  }

  return Flags;
}

inline pi_result piQueueCreate(pi_context context, pi_device device,
                               pi_queue_properties properties,
                               pi_queue *queue) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);
  auto phQueue = reinterpret_cast<ur_queue_handle_t *>(queue);
  ur_queue_properties_t URProperties = {
      /*stype*/ UR_STRUCTURE_TYPE_QUEUE_PROPERTIES,
      /*pNext*/ nullptr,
      /*flags*/ ConvertQueueFlagsBitfield(properties),
  };

  HANDLE_ERRORS(urQueueCreate(hContext, hDevice, &URProperties, phQueue));

  return PI_SUCCESS;
}

inline pi_result piextQueueCreate(pi_context context, pi_device device,
                                  pi_queue_properties *properties,
                                  pi_queue *queue) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);
  auto phQueue = reinterpret_cast<ur_queue_handle_t *>(queue);

  assert(properties);
  // Expect flags mask to be passed first.
  assert(properties[0] == PI_QUEUE_FLAGS);
  if (properties[0] != PI_QUEUE_FLAGS)
    return PI_ERROR_INVALID_VALUE;
  pi_queue_properties Flags = properties[1];
  // Extra data isn't supported yet.
  assert(properties[2] == 0);
  if (properties[2] != 0)
    return PI_ERROR_INVALID_VALUE;

  ur_queue_properties_t Properties = {
      /*stype*/ UR_STRUCTURE_TYPE_QUEUE_PROPERTIES,
      /*pNext*/ nullptr,
      /*flags*/ ConvertQueueFlagsBitfield(Flags),
  };

  HANDLE_ERRORS(urQueueCreate(hContext, hDevice, &Properties, phQueue));

  return PI_SUCCESS;
}

inline pi_result piextQueueCreate2(pi_context context, pi_device device,
                                   pi_queue_properties *properties,
                                   pi_queue *queue) {
  return pi2ur::piextQueueCreate(context, device, properties, queue);
}

inline pi_result piQueueGetInfo(pi_queue command_queue,
                                pi_queue_info param_name,
                                size_t param_value_size, void *param_value,
                                size_t *param_value_size_ret) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);

  static std::unordered_map<pi_queue_info, ur_queue_info_t> InfoMapping = {
      {PI_QUEUE_INFO_CONTEXT, UR_QUEUE_INFO_CONTEXT},
      {PI_QUEUE_INFO_DEVICE, UR_QUEUE_INFO_DEVICE},
      {PI_QUEUE_INFO_DEVICE_DEFAULT, UR_QUEUE_INFO_DEVICE_DEFAULT},
      {PI_QUEUE_INFO_REFERENCE_COUNT, UR_QUEUE_INFO_REFERENCE_COUNT},
      {PI_QUEUE_INFO_SIZE, UR_QUEUE_INFO_SIZE},
      {PI_EXT_ONEAPI_QUEUE_INFO_EMPTY, UR_QUEUE_INFO_EMPTY},
  };

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  HANDLE_ERRORS(urQueueGetInfo(hQueue, InfoType->second, param_value_size,
                               param_value, param_value_size_ret));

  return PI_SUCCESS;
}

inline pi_result piQueueRetain(pi_queue command_queue) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  HANDLE_ERRORS(urQueueRetain(hQueue));

  return PI_SUCCESS;
}

inline pi_result piQueueRelease(pi_queue command_queue) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  HANDLE_ERRORS(urQueueRelease(hQueue));

  return PI_SUCCESS;
}

inline pi_result piQueueFinish(pi_queue command_queue) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  HANDLE_ERRORS(urQueueFinish(hQueue));

  return PI_SUCCESS;
}

inline pi_result piQueueFlush(pi_queue command_queue) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  HANDLE_ERRORS(urQueueFlush(hQueue));

  return PI_SUCCESS;
}

inline pi_result piextQueueGetNativeHandle(pi_queue queue,
                                           pi_native_handle *nativeHandle) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto phNativeHandle = reinterpret_cast<ur_native_handle_t *>(nativeHandle);
  HANDLE_ERRORS(urQueueGetNativeHandle(hQueue, phNativeHandle));

  return PI_SUCCESS;
}

inline pi_result piextQueueGetNativeHandle2(pi_queue queue,
                                            pi_native_handle *nativeHandle,
                                            int32_t *nativeHandleDesc) {

  (void)nativeHandleDesc;
  return pi2ur::piextQueueGetNativeHandle(queue, nativeHandle);
}

inline pi_result piextQueueCreateWithNativeHandle(pi_native_handle nativeHandle,
                                                  pi_context context,
                                                  pi_device device,
                                                  bool pluginOwnsNativeHandle,
                                                  pi_queue *queue) {
  auto hNativeHandle = reinterpret_cast<ur_native_handle_t>(nativeHandle);
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);
  auto phQueue = reinterpret_cast<ur_queue_handle_t *>(queue);

  ur_queue_native_properties_t Props = {
      UR_STRUCTURE_TYPE_QUEUE_NATIVE_PROPERTIES, nullptr,
      pluginOwnsNativeHandle};

  HANDLE_ERRORS(urQueueCreateWithNativeHandle(hNativeHandle, hContext, hDevice,
                                              &Props, phQueue));

  return PI_SUCCESS;
}

inline pi_result piextQueueCreateWithNativeHandle2(
    pi_native_handle nativeHandle, int32_t nativeHandleDesc, pi_context context,
    pi_device device, bool pluginOwnsNativeHandle,
    pi_queue_properties *Properties, pi_queue *queue) {
  (void)nativeHandleDesc;
  (void)Properties;
  return pi2ur::piextQueueCreateWithNativeHandle(nativeHandle, context, device,
                                                 pluginOwnsNativeHandle, queue);
}

// Queue
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Event
inline pi_result piEventCreate(pi_context context, pi_event *ret_event) {
  assert(false && "piEventCreate not implement in pi2ur");
  return PI_ERROR_INVALID_OPERATION;
}

inline pi_result piEventGetInfo(pi_event event, pi_event_info param_name,
                                size_t param_value_size, void *param_value,
                                size_t *param_value_size_ret) {
  auto hEvent = reinterpret_cast<ur_event_handle_t>(event);

  // NOTE: This map isn't static to prevent a failure in
  // Regression/static-buffer-dtor from the llvm-test-suite. If the map is
  // static then it may have been destroyed by the time this function is called
  // from the destructor of the static buffer object in the test.
  std::unordered_map<pi_event_info, ur_event_info_t> EventInfoMap = {
      {PI_EVENT_INFO_COMMAND_QUEUE, UR_EVENT_INFO_COMMAND_QUEUE},
      {PI_EVENT_INFO_CONTEXT, UR_EVENT_INFO_CONTEXT},
      {PI_EVENT_INFO_COMMAND_TYPE, UR_EVENT_INFO_COMMAND_TYPE},
      {PI_EVENT_INFO_COMMAND_EXECUTION_STATUS,
       UR_EVENT_INFO_COMMAND_EXECUTION_STATUS},
      {PI_EVENT_INFO_REFERENCE_COUNT, UR_EVENT_INFO_REFERENCE_COUNT},
  };
  auto EventInfoMapIt = EventInfoMap.find(param_name);
  if (EventInfoMapIt == EventInfoMap.end()) {
    return PI_ERROR_UNKNOWN;
  }

  HANDLE_ERRORS(urEventGetInfo(hEvent, EventInfoMapIt->second, param_value_size,
                               param_value, param_value_size_ret));

  return PI_SUCCESS;
}

inline pi_result piEventGetProfilingInfo(pi_event event,
                                         pi_profiling_info param_name,
                                         size_t param_value_size,
                                         void *param_value,
                                         size_t *param_value_size_ret) {
  auto hEvent = reinterpret_cast<ur_event_handle_t>(event);

  static std::unordered_map<pi_profiling_info, ur_profiling_info_t>
      EventProfilingInfoMap = {
          {PI_PROFILING_INFO_COMMAND_QUEUED, UR_PROFILING_INFO_COMMAND_QUEUED},
          {PI_PROFILING_INFO_COMMAND_SUBMIT, UR_PROFILING_INFO_COMMAND_SUBMIT},
          {PI_PROFILING_INFO_COMMAND_START, UR_PROFILING_INFO_COMMAND_START},
          {PI_PROFILING_INFO_COMMAND_END, UR_PROFILING_INFO_COMMAND_END},
      };
  auto EventProfilingInfoMapIt = EventProfilingInfoMap.find(param_name);
  if (EventProfilingInfoMapIt == EventProfilingInfoMap.end()) {
    return PI_ERROR_UNKNOWN;
  }

  HANDLE_ERRORS(urEventGetProfilingInfo(hEvent, EventProfilingInfoMapIt->second,
                                        param_value_size, param_value,
                                        param_value_size_ret));

  return PI_SUCCESS;
}

inline pi_result piEventsWait(pi_uint32 num_events,
                              const pi_event *event_list) {
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_list);

  HANDLE_ERRORS(urEventWait(num_events, phEventWaitList));

  return PI_SUCCESS;
}

inline pi_result piEventSetCallback(
    pi_event event, pi_int32 command_exec_callback_type,
    void (*pfn_notify)(pi_event event, pi_int32 event_command_status,
                       void *user_data),
    void *user_data) {

  auto hEvent = reinterpret_cast<ur_event_handle_t>(event);
  auto pfnNotifyUR = reinterpret_cast<ur_event_callback_t>(pfn_notify);

  // These values aren't documented in PI, but are assumed to match
  // OpenCL's clSetEventCallback, so we can cast to the UR enum directly
  auto urExecInfo =
      static_cast<ur_execution_info_t>(command_exec_callback_type);

  HANDLE_ERRORS(urEventSetCallback(hEvent, urExecInfo, pfnNotifyUR, user_data));

  return PI_SUCCESS;
}

inline pi_result piEventSetStatus(pi_event event, pi_int32 execution_status) {
  assert(false && "piEventSetStatus not implement in pi2ur");
  return PI_ERROR_INVALID_OPERATION;
}

inline pi_result piEventRetain(pi_event event) {
  auto hEvent = reinterpret_cast<ur_event_handle_t>(event);
  HANDLE_ERRORS(urEventRetain(hEvent));
  return PI_SUCCESS;
}

inline pi_result piEventRelease(pi_event event) {
  auto hEvent = reinterpret_cast<ur_event_handle_t>(event);
  HANDLE_ERRORS(urEventRelease(hEvent));
  return PI_SUCCESS;
}

inline pi_result piextEventGetNativeHandle(pi_event event,
                                           pi_native_handle *nativeHandle) {
  auto hEvent = reinterpret_cast<ur_event_handle_t>(event);
  auto phNativeEvent = reinterpret_cast<ur_native_handle_t *>(nativeHandle);
  HANDLE_ERRORS(urEventGetNativeHandle(hEvent, phNativeEvent));
  return PI_SUCCESS;
}

inline pi_result piextEventCreateWithNativeHandle(pi_native_handle nativeHandle,
                                                  pi_context context,
                                                  bool ownNativeHandle,
                                                  pi_event *event) {
  auto hNativeEvent = reinterpret_cast<ur_native_handle_t>(nativeHandle);
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);
  ur_event_native_properties_t Props = {
      UR_STRUCTURE_TYPE_EVENT_NATIVE_PROPERTIES, nullptr, ownNativeHandle};

  HANDLE_ERRORS(
      urEventCreateWithNativeHandle(hNativeEvent, hContext, &Props, phEvent));

  return PI_SUCCESS;
}

inline pi_result piEnqueueEventsWait(pi_queue command_queue,
                                     pi_uint32 num_events_in_wait_list,
                                     const pi_event *event_wait_list,
                                     pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueEventsWait(hQueue, num_events_in_wait_list,
                                    phEventWaitList, phEvent));

  return PI_SUCCESS;
}

// Event
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Enqueue
inline pi_result piEnqueueKernelLaunch(
    pi_queue queue, pi_kernel kernel, pi_uint32 work_dim,
    const size_t *global_work_offset, const size_t *global_work_size,
    const size_t *local_work_size, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {

  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueKernelLaunch(
      hQueue, hKernel, work_dim, global_work_offset, global_work_size,
      local_work_size, num_events_in_wait_list, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piEnqueueEventsWaitWithBarrier(
    pi_queue command_queue, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueEventsWaitWithBarrier(hQueue, num_events_in_wait_list,
                                               phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piEnqueueMemBufferRead(pi_queue command_queue, pi_mem buffer,
                                        pi_bool blocking_read, size_t offset,
                                        size_t size, void *ptr,
                                        pi_uint32 num_events_in_wait_list,
                                        const pi_event *event_wait_list,
                                        pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hBuffer = reinterpret_cast<ur_mem_handle_t>(buffer);
  auto urBlockingRead = static_cast<bool>(blocking_read);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueMemBufferRead(hQueue, hBuffer, urBlockingRead, offset,
                                       size, ptr, num_events_in_wait_list,
                                       phEventWaitList, phEvent));

  return PI_SUCCESS;
}

/// \TODO Not implemented
inline pi_result piEnqueueNativeKernel(pi_queue, void (*)(void *), void *,
                                       size_t, pi_uint32, const pi_mem *,
                                       const void **, pi_uint32,
                                       const pi_event *, pi_event *) {
  die("Doesn't map to an entry-point in UR");
  return {};
}

/// \TODO Not implemented in CUDA.
inline pi_result piEnqueueMemImageFill(pi_queue, pi_mem, const void *,
                                       const size_t *, const size_t *,
                                       pi_uint32, const pi_event *,
                                       pi_event *) {
  die("Doesn't map to an entry-point in UR");
  return {};
}

inline pi_result piEnqueueMemBufferReadRect(
    pi_queue command_queue, pi_mem buffer, pi_bool blocking_read,
    pi_buff_rect_offset buffer_offset, pi_buff_rect_offset host_offset,
    pi_buff_rect_region region, size_t buffer_row_pitch,
    size_t buffer_slice_pitch, size_t host_row_pitch, size_t host_slice_pitch,
    void *ptr, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hBuffer = reinterpret_cast<ur_mem_handle_t>(buffer);
  auto bufferOrigin = reinterpret_cast<ur_rect_offset_t *>(buffer_offset);
  auto hostOrigin = reinterpret_cast<ur_rect_offset_t *>(host_offset);
  auto ur_region = reinterpret_cast<ur_rect_region_t *>(region);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueMemBufferReadRect(
      hQueue, hBuffer, blocking_read, *bufferOrigin, *hostOrigin, *ur_region,
      buffer_row_pitch, buffer_slice_pitch, host_row_pitch, host_slice_pitch,
      ptr, num_events_in_wait_list, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piEnqueueMemBufferWrite(pi_queue command_queue, pi_mem buffer,
                                         pi_bool blocking_write, size_t offset,
                                         size_t size, const void *ptr,
                                         pi_uint32 num_events_in_wait_list,
                                         const pi_event *event_wait_list,
                                         pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hBuffer = reinterpret_cast<ur_mem_handle_t>(buffer);
  auto urBlockingWrite = static_cast<bool>(blocking_write);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueMemBufferWrite(
      hQueue, hBuffer, urBlockingWrite, offset, size, ptr,
      num_events_in_wait_list, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piEnqueueMemBufferWriteRect(
    pi_queue command_queue, pi_mem buffer, pi_bool blocking_write,
    pi_buff_rect_offset buffer_offset, pi_buff_rect_offset host_offset,
    pi_buff_rect_region region, size_t buffer_row_pitch,
    size_t buffer_slice_pitch, size_t host_row_pitch, size_t host_slice_pitch,
    const void *ptr, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hBuffer = reinterpret_cast<ur_mem_handle_t>(buffer);
  auto bufferOrigin = reinterpret_cast<ur_rect_offset_t *>(buffer_offset);
  auto hostOrigin = reinterpret_cast<ur_rect_offset_t *>(host_offset);
  auto ur_region = reinterpret_cast<ur_rect_region_t *>(region);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);
  auto pSrc = const_cast<void *>(ptr);

  HANDLE_ERRORS(urEnqueueMemBufferWriteRect(
      hQueue, hBuffer, blocking_write, *bufferOrigin, *hostOrigin, *ur_region,
      buffer_row_pitch, buffer_slice_pitch, host_row_pitch, host_slice_pitch,
      pSrc, num_events_in_wait_list, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result
piEnqueueMemBufferCopy(pi_queue command_queue, pi_mem src_buffer,
                       pi_mem dst_buffer, size_t src_offset, size_t dst_offset,
                       size_t size, pi_uint32 num_events_in_wait_list,
                       const pi_event *event_wait_list, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);
  auto hBufferSrc = reinterpret_cast<ur_mem_handle_t>(src_buffer);
  auto hBufferDst = reinterpret_cast<ur_mem_handle_t>(dst_buffer);

  HANDLE_ERRORS(urEnqueueMemBufferCopy(
      hQueue, hBufferSrc, hBufferDst, src_offset, dst_offset, size,
      num_events_in_wait_list, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piEnqueueMemBufferCopyRect(
    pi_queue command_queue, pi_mem src_buffer, pi_mem dst_buffer,
    pi_buff_rect_offset src_origin, pi_buff_rect_offset dst_origin,
    pi_buff_rect_region region, size_t src_row_pitch, size_t src_slice_pitch,
    size_t dst_row_pitch, size_t dst_slice_pitch,
    pi_uint32 num_events_in_wait_list, const pi_event *event_wait_list,
    pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);
  auto hBufferSrc = reinterpret_cast<ur_mem_handle_t>(src_buffer);
  auto hBufferDst = reinterpret_cast<ur_mem_handle_t>(dst_buffer);
  auto srcOrigin = reinterpret_cast<ur_rect_offset_t *>(src_origin);
  auto dstOrigin = reinterpret_cast<ur_rect_offset_t *>(dst_origin);
  auto ur_region = reinterpret_cast<ur_rect_region_t *>(region);

  HANDLE_ERRORS(urEnqueueMemBufferCopyRect(
      hQueue, hBufferSrc, hBufferDst, *srcOrigin, *dstOrigin, *ur_region,
      src_row_pitch, src_slice_pitch, dst_row_pitch, dst_slice_pitch,
      num_events_in_wait_list, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result
piEnqueueMemBufferFill(pi_queue command_queue, pi_mem buffer,
                       const void *pattern, size_t pattern_size, size_t offset,
                       size_t size, pi_uint32 num_events_in_wait_list,
                       const pi_event *event_wait_list, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hBuffer = reinterpret_cast<ur_mem_handle_t>(buffer);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueMemBufferFill(hQueue, hBuffer, pattern, pattern_size,
                                       offset, size, num_events_in_wait_list,
                                       phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piEnqueueMemImageRead(
    pi_queue command_queue, pi_mem image, pi_bool blocking_read,
    const size_t *origin, const size_t *region, size_t row_pitch,
    size_t slice_pitch, void *ptr, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hImage = reinterpret_cast<ur_mem_handle_t>(image);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  ur_rect_offset_t ur_origin{origin[0], origin[1], origin[2]};
  ur_rect_region_t ur_region{region[0], region[1], region[2]};

  HANDLE_ERRORS(urEnqueueMemImageRead(
      hQueue, hImage, blocking_read, ur_origin, ur_region, row_pitch,
      slice_pitch, ptr, num_events_in_wait_list, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result
piEnqueueMemImageWrite(pi_queue command_queue, pi_mem image,
                       pi_bool blocking_write, const size_t *origin,
                       const size_t *region, size_t input_row_pitch,
                       size_t input_slice_pitch, const void *ptr,
                       pi_uint32 num_events_in_wait_list,
                       const pi_event *event_wait_list, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hImage = reinterpret_cast<ur_mem_handle_t>(image);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  ur_rect_offset_t ur_origin{origin[0], origin[1], origin[2]};
  ur_rect_region_t ur_region{region[0], region[1], region[2]};

  auto pSrc = const_cast<void *>(ptr);

  HANDLE_ERRORS(urEnqueueMemImageWrite(
      hQueue, hImage, blocking_write, ur_origin, ur_region, input_row_pitch,
      input_slice_pitch, pSrc, num_events_in_wait_list, phEventWaitList,
      phEvent));

  return PI_SUCCESS;
}

inline pi_result
piEnqueueMemImageCopy(pi_queue command_queue, pi_mem src_image,
                      pi_mem dst_image, const size_t *src_origin,
                      const size_t *dst_origin, const size_t *region,
                      pi_uint32 num_events_in_wait_list,
                      const pi_event *event_wait_list, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hSrcImage = reinterpret_cast<ur_mem_handle_t>(src_image);
  auto hDstImage = reinterpret_cast<ur_mem_handle_t>(dst_image);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  ur_rect_offset_t ur_src_origin{src_origin[0], src_origin[1], src_origin[2]};
  ur_rect_offset_t ur_dst_origin{dst_origin[0], dst_origin[1], dst_origin[2]};
  ur_rect_region_t ur_region{region[0], region[1], region[2]};

  HANDLE_ERRORS(urEnqueueMemImageCopy(
      hQueue, hSrcImage, hDstImage, ur_src_origin, ur_dst_origin, ur_region,
      num_events_in_wait_list, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piEnqueueMemBufferMap(pi_queue command_queue, pi_mem buffer,
                                       pi_bool blocking_map,
                                       pi_map_flags map_flags, size_t offset,
                                       size_t size,
                                       pi_uint32 num_events_in_wait_list,
                                       const pi_event *event_wait_list,
                                       pi_event *event, void **ret_map) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);
  auto hBuffer = reinterpret_cast<ur_mem_handle_t>(buffer);

  ur_map_flags_t urFlags{};

  pi2urMapFlags(map_flags, &urFlags);

  HANDLE_ERRORS(urEnqueueMemBufferMap(hQueue, hBuffer, blocking_map, urFlags,
                                      offset, size, num_events_in_wait_list,
                                      phEventWaitList, phEvent, ret_map));

  return PI_SUCCESS;
}

inline pi_result piEnqueueMemUnmap(pi_queue command_queue, pi_mem memobj,
                                   void *mapped_ptr,
                                   pi_uint32 num_events_in_wait_list,
                                   const pi_event *event_wait_list,
                                   pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(command_queue);
  auto hMem = reinterpret_cast<ur_mem_handle_t>(memobj);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(event_wait_list);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueMemUnmap(hQueue, hMem, mapped_ptr,
                                  num_events_in_wait_list, phEventWaitList,
                                  phEvent));

  return PI_SUCCESS;
}

inline pi_result
piextEnqueueReadHostPipe(pi_queue queue, pi_program program,
                         const char *pipe_symbol, pi_bool blocking, void *ptr,
                         size_t size, pi_uint32 num_events_in_waitlist,
                         const pi_event *events_waitlist, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(events_waitlist);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueReadHostPipe(hQueue, hProgram, pipe_symbol, blocking,
                                      ptr, size, num_events_in_waitlist,
                                      phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result
piextEnqueueWriteHostPipe(pi_queue queue, pi_program program,
                          const char *pipe_symbol, pi_bool blocking, void *ptr,
                          size_t size, pi_uint32 num_events_in_waitlist,
                          const pi_event *events_waitlist, pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto hProgram = reinterpret_cast<ur_program_handle_t>(program);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(events_waitlist);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueWriteHostPipe(hQueue, hProgram, pipe_symbol, blocking,
                                       ptr, size, num_events_in_waitlist,
                                       phEventWaitList, phEvent));

  return PI_SUCCESS;
}

// Enqueue
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Sampler
inline pi_result piSamplerCreate(pi_context Context,
                                 const pi_sampler_properties *SamplerProperties,
                                 pi_sampler *RetSampler) {
  PI_ASSERT(Context, PI_ERROR_INVALID_CONTEXT);
  PI_ASSERT(RetSampler, PI_ERROR_INVALID_VALUE);

  ur_context_handle_t UrContext =
      reinterpret_cast<ur_context_handle_t>(Context);
  ur_sampler_desc_t UrProps{};
  UrProps.stype = UR_STRUCTURE_TYPE_SAMPLER_DESC;
  const pi_sampler_properties *CurProperty = SamplerProperties;
  while (*CurProperty != 0) {
    switch (*CurProperty) {
    case PI_SAMPLER_PROPERTIES_NORMALIZED_COORDS: {
      UrProps.normalizedCoords = static_cast<pi_bool>(*(++CurProperty));
    } break;

    case PI_SAMPLER_PROPERTIES_ADDRESSING_MODE: {
      pi_sampler_addressing_mode CurValueAddressingMode =
          static_cast<pi_sampler_addressing_mode>(*(++CurProperty));

      if (CurValueAddressingMode == PI_SAMPLER_ADDRESSING_MODE_MIRRORED_REPEAT)
        UrProps.addressingMode = UR_SAMPLER_ADDRESSING_MODE_MIRRORED_REPEAT;
      else if (CurValueAddressingMode == PI_SAMPLER_ADDRESSING_MODE_REPEAT)
        UrProps.addressingMode = UR_SAMPLER_ADDRESSING_MODE_REPEAT;
      else if (CurValueAddressingMode ==
               PI_SAMPLER_ADDRESSING_MODE_CLAMP_TO_EDGE)
        UrProps.addressingMode = UR_SAMPLER_ADDRESSING_MODE_CLAMP_TO_EDGE;
      else if (CurValueAddressingMode == PI_SAMPLER_ADDRESSING_MODE_CLAMP)
        UrProps.addressingMode = UR_SAMPLER_ADDRESSING_MODE_CLAMP;
      else if (CurValueAddressingMode == PI_SAMPLER_ADDRESSING_MODE_NONE)
        UrProps.addressingMode = UR_SAMPLER_ADDRESSING_MODE_NONE;
    } break;

    case PI_SAMPLER_PROPERTIES_FILTER_MODE: {
      pi_sampler_filter_mode CurValueFilterMode =
          static_cast<pi_sampler_filter_mode>(*(++CurProperty));

      if (CurValueFilterMode == PI_SAMPLER_FILTER_MODE_NEAREST)
        UrProps.filterMode = UR_SAMPLER_FILTER_MODE_NEAREST;
      else if (CurValueFilterMode == PI_SAMPLER_FILTER_MODE_LINEAR)
        UrProps.filterMode = UR_SAMPLER_FILTER_MODE_LINEAR;
    } break;

    default:
      break;
    }
    CurProperty++;
  }

  ur_sampler_handle_t *UrSampler =
      reinterpret_cast<ur_sampler_handle_t *>(RetSampler);

  HANDLE_ERRORS(urSamplerCreate(UrContext, &UrProps, UrSampler));

  return PI_SUCCESS;
}

inline pi_result piSamplerGetInfo(pi_sampler Sampler, pi_sampler_info ParamName,
                                  size_t ParamValueSize, void *ParamValue,
                                  size_t *ParamValueSizeRet) {
  static std::unordered_map<pi_sampler_info, ur_sampler_info_t> InfoMapping = {
      {PI_SAMPLER_INFO_REFERENCE_COUNT, UR_SAMPLER_INFO_REFERENCE_COUNT},
      {PI_SAMPLER_INFO_CONTEXT, UR_SAMPLER_INFO_CONTEXT},
      {PI_SAMPLER_INFO_NORMALIZED_COORDS, UR_SAMPLER_INFO_NORMALIZED_COORDS},
      {PI_SAMPLER_INFO_ADDRESSING_MODE, UR_SAMPLER_INFO_ADDRESSING_MODE},
      {PI_SAMPLER_INFO_FILTER_MODE, UR_SAMPLER_INFO_FILTER_MODE},
  };

  auto InfoType = InfoMapping.find(ParamName);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  size_t UrParamValueSizeRet;
  auto hSampler = reinterpret_cast<ur_sampler_handle_t>(Sampler);
  HANDLE_ERRORS(urSamplerGetInfo(hSampler, InfoType->second, ParamValueSize,
                                 ParamValue, &UrParamValueSizeRet));
  if (ParamValueSizeRet) {
    *ParamValueSizeRet = UrParamValueSizeRet;
  }
  fixupInfoValueTypes(UrParamValueSizeRet, ParamValueSizeRet, ParamValueSize,
                      ParamValue);

  return PI_SUCCESS;
}

inline pi_result piSamplerRetain(pi_sampler Sampler) {
  PI_ASSERT(Sampler, PI_ERROR_INVALID_SAMPLER);

  ur_sampler_handle_t UrSampler =
      reinterpret_cast<ur_sampler_handle_t>(Sampler);

  HANDLE_ERRORS(urSamplerRetain(UrSampler));

  return PI_SUCCESS;
}

inline pi_result piSamplerRelease(pi_sampler Sampler) {
  PI_ASSERT(Sampler, PI_ERROR_INVALID_SAMPLER);

  ur_sampler_handle_t UrSampler =
      reinterpret_cast<ur_sampler_handle_t>(Sampler);

  HANDLE_ERRORS(urSamplerRelease(UrSampler));

  return PI_SUCCESS;
}
// Sampler
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Memory
inline pi_result piMemBufferCreate(pi_context context, pi_mem_flags flags,
                                   size_t size, void *host_ptr, pi_mem *ret_mem,
                                   const pi_mem_properties *properties) {
  (void)properties;
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);

  ur_mem_flags_t urFlags{};
  pi2urMemFlags(flags, &urFlags);

  ur_buffer_alloc_location_properties_t bufferLocationProperties{
      UR_STRUCTURE_TYPE_BUFFER_ALLOC_LOCATION_PROPERTIES,
      nullptr,
      0,
  };

  ur_buffer_channel_properties_t bufferChannelProperties{
      UR_STRUCTURE_TYPE_BUFFER_CHANNEL_PROPERTIES,
      &bufferLocationProperties,
      0,
  };

  ur_buffer_properties_t bufferProperties{
      UR_STRUCTURE_TYPE_BUFFER_PROPERTIES,
      nullptr,
      host_ptr,
  };

  const pi_mem_properties *CurProperty = properties;
  while (CurProperty && *CurProperty != 0) {
    switch (*CurProperty) {
    case PI_MEM_PROPERTIES_ALLOC_BUFFER_LOCATION: {
      bufferLocationProperties.location =
          static_cast<uint32_t>(*(++CurProperty));
    } break;

    case PI_MEM_PROPERTIES_CHANNEL: {
      bufferChannelProperties.channel = static_cast<uint32_t>(*(++CurProperty));
      bufferProperties.pNext = &bufferChannelProperties;
    } break;

    default:
      break;
    }
    CurProperty++;
  }

  auto phRetMem = reinterpret_cast<ur_mem_handle_t *>(ret_mem);
  HANDLE_ERRORS(
      urMemBufferCreate(hContext, urFlags, size, &bufferProperties, phRetMem));
  return PI_SUCCESS;
}

inline pi_result piMemRetain(pi_mem mem) {
  auto hMem = reinterpret_cast<ur_mem_handle_t>(mem);
  HANDLE_ERRORS(urMemRetain(hMem));
  return PI_SUCCESS;
}

inline pi_result piMemRelease(pi_mem memObj) {
  auto hMem = reinterpret_cast<ur_mem_handle_t>(memObj);
  HANDLE_ERRORS(urMemRelease(hMem));
  return PI_SUCCESS;
}

inline pi_result piextMemGetNativeHandle(pi_mem mem,
                                         pi_native_handle *nativeHandle) {
  auto hMem = reinterpret_cast<ur_mem_handle_t>(mem);
  auto hNativeHandle = reinterpret_cast<ur_native_handle_t *>(nativeHandle);
  HANDLE_ERRORS(urMemGetNativeHandle(hMem, hNativeHandle));
  return PI_SUCCESS;
}

inline pi_result piextUSMFree(pi_context context, void *ptr) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);

  HANDLE_ERRORS(urUSMFree(hContext, ptr));

  return PI_SUCCESS;
}

inline pi_result piMemGetInfo(pi_mem mem, pi_mem_info memInfo, size_t size,
                              void *pMemInfo, size_t *pMemInfoSize) {
  auto hMem = reinterpret_cast<ur_mem_handle_t>(mem);

  static std::unordered_map<pi_mem_info, ur_mem_info_t> MemInfoMap = {
      {PI_MEM_CONTEXT, UR_MEM_INFO_SIZE},
      {PI_MEM_SIZE, UR_MEM_INFO_CONTEXT},
  };

  auto MemInfoMapIt = MemInfoMap.find(memInfo);
  if (MemInfoMapIt == MemInfoMap.end()) {
    return PI_ERROR_UNKNOWN;
  }

  HANDLE_ERRORS(
      urMemGetInfo(hMem, MemInfoMapIt->second, size, pMemInfo, pMemInfoSize));
  return PI_SUCCESS;
}

inline pi_result piextMemCreateWithNativeHandle(pi_native_handle nativeHandle,
                                                pi_context context,
                                                bool ownNativeHandle,
                                                pi_mem *mem) {
  auto hNativeHandle = reinterpret_cast<ur_native_handle_t>(nativeHandle);
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto hMem = reinterpret_cast<ur_mem_handle_t *>(mem);
  ur_mem_native_properties_t Props = {UR_STRUCTURE_TYPE_MEM_NATIVE_PROPERTIES,
                                      nullptr, ownNativeHandle};

  HANDLE_ERRORS(
      urMemBufferCreateWithNativeHandle(hNativeHandle, hContext, &Props, hMem));

  return PI_SUCCESS;
}

inline pi_result piMemImageCreate(pi_context context, pi_mem_flags flags,
                                  const pi_image_format *image_format,
                                  const pi_image_desc *image_desc,
                                  void *host_ptr, pi_mem *ret_mem) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto phMem = reinterpret_cast<ur_mem_handle_t *>(ret_mem);

  ur_mem_flags_t urFlags{};
  pi2urMemFlags(flags, &urFlags);

  static std::unordered_map<pi_image_channel_order, ur_image_channel_order_t>
      ImageChannelOrderMap = {
          {PI_IMAGE_CHANNEL_ORDER_A, UR_IMAGE_CHANNEL_ORDER_A},
          {PI_IMAGE_CHANNEL_ORDER_R, UR_IMAGE_CHANNEL_ORDER_R},
          {PI_IMAGE_CHANNEL_ORDER_RG, UR_IMAGE_CHANNEL_ORDER_RG},
          {PI_IMAGE_CHANNEL_ORDER_RA, UR_IMAGE_CHANNEL_ORDER_RA},
          {PI_IMAGE_CHANNEL_ORDER_RGB, UR_IMAGE_CHANNEL_ORDER_RGB},
          {PI_IMAGE_CHANNEL_ORDER_RGBA, UR_IMAGE_CHANNEL_ORDER_RGBA},
          {PI_IMAGE_CHANNEL_ORDER_BGRA, UR_IMAGE_CHANNEL_ORDER_BGRA},
          {PI_IMAGE_CHANNEL_ORDER_ARGB, UR_IMAGE_CHANNEL_ORDER_ARGB},
          {PI_IMAGE_CHANNEL_ORDER_ABGR, UR_IMAGE_CHANNEL_ORDER_ABGR},
          {PI_IMAGE_CHANNEL_ORDER_INTENSITY, UR_IMAGE_CHANNEL_ORDER_INTENSITY},
          {PI_IMAGE_CHANNEL_ORDER_LUMINANCE, UR_IMAGE_CHANNEL_ORDER_LUMINANCE},
          {PI_IMAGE_CHANNEL_ORDER_Rx, UR_IMAGE_CHANNEL_ORDER_RX},
          {PI_IMAGE_CHANNEL_ORDER_RGx, UR_IMAGE_CHANNEL_ORDER_RGX},
          {PI_IMAGE_CHANNEL_ORDER_RGBx, UR_IMAGE_CHANNEL_ORDER_RGBX},
          {PI_IMAGE_CHANNEL_ORDER_sRGBA, UR_IMAGE_CHANNEL_ORDER_SRGBA},
      };

  const auto &ChannelOrderIt =
      ImageChannelOrderMap.find(image_format->image_channel_order);
  if (ChannelOrderIt == ImageChannelOrderMap.end()) {
    return PI_ERROR_UNKNOWN;
  }
  ur_image_channel_order_t urImageChannelOrder = ChannelOrderIt->second;

  static std::unordered_map<pi_image_channel_type, ur_image_channel_type_t>
      ImageChannelTypeMap = {
          {PI_IMAGE_CHANNEL_TYPE_SNORM_INT8, UR_IMAGE_CHANNEL_TYPE_SNORM_INT8},
          {PI_IMAGE_CHANNEL_TYPE_SNORM_INT16,
           UR_IMAGE_CHANNEL_TYPE_SNORM_INT16},
          {PI_IMAGE_CHANNEL_TYPE_UNORM_INT8, UR_IMAGE_CHANNEL_TYPE_UNORM_INT8},
          {PI_IMAGE_CHANNEL_TYPE_UNORM_INT16,
           UR_IMAGE_CHANNEL_TYPE_UNORM_INT16},
          {PI_IMAGE_CHANNEL_TYPE_UNORM_SHORT_565,
           UR_IMAGE_CHANNEL_TYPE_UNORM_SHORT_565},
          {PI_IMAGE_CHANNEL_TYPE_UNORM_SHORT_555,
           UR_IMAGE_CHANNEL_TYPE_UNORM_SHORT_555},
          {PI_IMAGE_CHANNEL_TYPE_UNORM_INT_101010,
           UR_IMAGE_CHANNEL_TYPE_INT_101010},
          {PI_IMAGE_CHANNEL_TYPE_SIGNED_INT8,
           UR_IMAGE_CHANNEL_TYPE_SIGNED_INT8},
          {PI_IMAGE_CHANNEL_TYPE_SIGNED_INT16,
           UR_IMAGE_CHANNEL_TYPE_SIGNED_INT16},
          {PI_IMAGE_CHANNEL_TYPE_SIGNED_INT32,
           UR_IMAGE_CHANNEL_TYPE_SIGNED_INT32},
          {PI_IMAGE_CHANNEL_TYPE_UNSIGNED_INT8,
           UR_IMAGE_CHANNEL_TYPE_UNSIGNED_INT8},
          {PI_IMAGE_CHANNEL_TYPE_UNSIGNED_INT16,
           UR_IMAGE_CHANNEL_TYPE_UNSIGNED_INT16},
          {PI_IMAGE_CHANNEL_TYPE_UNSIGNED_INT32,
           UR_IMAGE_CHANNEL_TYPE_UNSIGNED_INT32},
          {PI_IMAGE_CHANNEL_TYPE_HALF_FLOAT, UR_IMAGE_CHANNEL_TYPE_HALF_FLOAT},
          {PI_IMAGE_CHANNEL_TYPE_FLOAT, UR_IMAGE_CHANNEL_TYPE_FLOAT},
      };
  const auto &ChannelTypeIt =
      ImageChannelTypeMap.find(image_format->image_channel_data_type);
  if (ChannelTypeIt == ImageChannelTypeMap.end()) {
    return PI_ERROR_UNKNOWN;
  }
  ur_image_channel_type_t urImageChannelType = ChannelTypeIt->second;

  ur_image_format_t urImageFormat{urImageChannelOrder, urImageChannelType};

  static std::unordered_map<pi_mem_type, ur_mem_type_t> ImageTypeMap = {
      {PI_MEM_TYPE_BUFFER, UR_MEM_TYPE_BUFFER},
      {PI_MEM_TYPE_IMAGE2D, UR_MEM_TYPE_IMAGE2D},
      {PI_MEM_TYPE_IMAGE3D, UR_MEM_TYPE_IMAGE3D},
      {PI_MEM_TYPE_IMAGE2D_ARRAY, UR_MEM_TYPE_IMAGE2D_ARRAY},
      {PI_MEM_TYPE_IMAGE1D, UR_MEM_TYPE_IMAGE1D},
      {PI_MEM_TYPE_IMAGE1D_ARRAY, UR_MEM_TYPE_IMAGE1D_ARRAY},
      {PI_MEM_TYPE_IMAGE1D_BUFFER, UR_MEM_TYPE_IMAGE1D_BUFFER},
  };
  const auto &ImageTypeIt = ImageTypeMap.find(image_desc->image_type);
  if (ImageTypeIt == ImageTypeMap.end()) {
    return PI_ERROR_UNKNOWN;
  }

  ur_image_desc_t urImageDesc{
      UR_STRUCTURE_TYPE_IMAGE_DESC,  nullptr,
      ImageTypeIt->second,           image_desc->image_width,
      image_desc->image_height,      image_desc->image_depth,
      image_desc->image_array_size,  image_desc->image_row_pitch,
      image_desc->image_slice_pitch, image_desc->num_mip_levels,
      image_desc->num_samples};

  HANDLE_ERRORS(urMemImageCreate(hContext, urFlags, &urImageFormat,
                                 &urImageDesc, host_ptr, phMem));
  return PI_SUCCESS;
}

inline pi_result piMemImageGetInfo(pi_mem mem, pi_image_info info, size_t size,
                                   void *pImgInfo, size_t *ret_size) {
  auto hMem = reinterpret_cast<ur_mem_handle_t>(mem);

  static std::unordered_map<pi_image_info, ur_image_info_t> ImageInfoMap = {
      {PI_IMAGE_INFO_FORMAT, UR_IMAGE_INFO_FORMAT},
      {PI_IMAGE_INFO_ELEMENT_SIZE, UR_IMAGE_INFO_ELEMENT_SIZE},
      {PI_IMAGE_INFO_ROW_PITCH, UR_IMAGE_INFO_ROW_PITCH},
      {PI_IMAGE_INFO_SLICE_PITCH, UR_IMAGE_INFO_SLICE_PITCH},
      {PI_IMAGE_INFO_WIDTH, UR_IMAGE_INFO_WIDTH},
      {PI_IMAGE_INFO_HEIGHT, UR_IMAGE_INFO_HEIGHT},
      {PI_IMAGE_INFO_DEPTH, UR_IMAGE_INFO_DEPTH},
  };

  auto ImageInfoMapIt = ImageInfoMap.find(info);
  if (ImageInfoMapIt == ImageInfoMap.end()) {
    return PI_ERROR_UNKNOWN;
  }

  HANDLE_ERRORS(urMemImageGetInfo(hMem, ImageInfoMapIt->second, size, pImgInfo,
                                  ret_size));
  return PI_SUCCESS;
}

inline pi_result piMemBufferPartition(pi_mem parent_buffer, pi_mem_flags flags,
                                      pi_buffer_create_type buffer_create_type,
                                      void *buffer_create_info,
                                      pi_mem *memObj) {
  auto hParentBuffer = reinterpret_cast<ur_mem_handle_t>(parent_buffer);
  auto hMemObj = reinterpret_cast<ur_mem_handle_t *>(memObj);

  ur_mem_flags_t urFlags{};
  pi2urMemFlags(flags, &urFlags);

  auto piBuffer = static_cast<pi_buffer_region>(buffer_create_info);

  const ur_buffer_region_t bufferRegion{UR_STRUCTURE_TYPE_BUFFER_REGION,
                                        nullptr, piBuffer->origin,
                                        piBuffer->size};

  HANDLE_ERRORS(urMemBufferPartition(hParentBuffer, urFlags,
                                     UR_BUFFER_CREATE_TYPE_REGION,
                                     &bufferRegion, hMemObj));

  return PI_SUCCESS;
}
// Memory
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// USM
inline pi_result piextUSMHostAlloc(void **result_ptr, pi_context context,
                                   pi_usm_mem_properties *properties,
                                   size_t size, pi_uint32 alignment) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);

  ur_usm_desc_t USMDesc{};
  USMDesc.stype = UR_STRUCTURE_TYPE_USM_DESC;
  USMDesc.align = alignment;

  HANDLE_ERRORS(urUSMHostAlloc(hContext, &USMDesc, nullptr, size, result_ptr));

  return PI_SUCCESS;
}

inline pi_result piextUSMDeviceAlloc(void **result_ptr, pi_context context,
                                     pi_device device,
                                     pi_usm_mem_properties *properties,
                                     size_t size, pi_uint32 alignment) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);

  ur_usm_desc_t USMDesc{};
  USMDesc.stype = UR_STRUCTURE_TYPE_USM_DESC;
  USMDesc.align = alignment;

  HANDLE_ERRORS(
      urUSMDeviceAlloc(hContext, hDevice, &USMDesc, nullptr, size, result_ptr));

  return PI_SUCCESS;
}

inline pi_result piextUSMSharedAlloc(void **result_ptr, pi_context context,
                                     pi_device device,
                                     pi_usm_mem_properties *properties,
                                     size_t size, pi_uint32 alignment) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  auto hDevice = reinterpret_cast<ur_device_handle_t>(device);

  ur_usm_desc_t USMDesc{};
  USMDesc.stype = UR_STRUCTURE_TYPE_USM_DESC;
  USMDesc.align = alignment;

  HANDLE_ERRORS(
      urUSMSharedAlloc(hContext, hDevice, &USMDesc, nullptr, size, result_ptr));

  return PI_SUCCESS;
}

inline pi_result piextUSMEnqueueMemset(pi_queue queue, void *ptr,
                                       pi_int32 value, size_t count,
                                       pi_uint32 num_events_in_waitlist,
                                       const pi_event *events_waitlist,
                                       pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(events_waitlist);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueUSMFill(hQueue, ptr, 1, &value, count,
                                 num_events_in_waitlist, phEventWaitList,
                                 phEvent));

  return PI_SUCCESS;
}

inline pi_result piextUSMEnqueueMemcpy(pi_queue queue, pi_bool blocking,
                                       void *dst_ptr, const void *src_ptr,
                                       size_t size,
                                       pi_uint32 num_events_in_waitlist,
                                       const pi_event *events_waitlist,
                                       pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(events_waitlist);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueUSMMemcpy(hQueue, blocking, dst_ptr, src_ptr, size,
                                   num_events_in_waitlist, phEventWaitList,
                                   phEvent));

  return PI_SUCCESS;
}

inline pi_result piextUSMEnqueuePrefetch(pi_queue queue, const void *ptr,
                                         size_t size,
                                         pi_usm_migration_flags flags,
                                         pi_uint32 num_events_in_waitlist,
                                         const pi_event *events_waitlist,
                                         pi_event *event) {
  (void)flags;
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(events_waitlist);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  // PI does not implement migration flags atm, and UR have only one default
  // migration flag.
  ur_usm_migration_flags_t urFlags{};

  HANDLE_ERRORS(urEnqueueUSMPrefetch(hQueue, ptr, size, urFlags,
                                     num_events_in_waitlist, phEventWaitList,
                                     phEvent));

  return PI_SUCCESS;
}

inline ur_usm_advice_flags_t
ConvertMemAdviseFlagsBitfield(pi_mem_advice flags) {
  ur_usm_advice_flags_t Flags{};
  static std::unordered_map<pi_mem_advice, ur_usm_advice_flags_t> FlagMap = {
      {PI_MEM_ADVICE_CUDA_SET_READ_MOSTLY, UR_USM_ADVICE_FLAG_SET_READ_MOSTLY},
      {PI_MEM_ADVICE_CUDA_UNSET_READ_MOSTLY,
       UR_USM_ADVICE_FLAG_CLEAR_READ_MOSTLY},
      {PI_MEM_ADVICE_CUDA_SET_PREFERRED_LOCATION,
       UR_USM_ADVICE_FLAG_SET_PREFERRED_LOCATION},
      {PI_MEM_ADVICE_CUDA_UNSET_PREFERRED_LOCATION,
       UR_USM_ADVICE_FLAG_CLEAR_PREFERRED_LOCATION},
      {PI_MEM_ADVICE_RESET, UR_USM_ADVICE_FLAG_DEFAULT}};
  for (auto &FlagPair : FlagMap) {
    if (flags & FlagPair.first) {
      Flags |= FlagPair.second;
    }
  }

  return Flags;
}

inline pi_result piextUSMEnqueueMemAdvise(pi_queue queue, const void *ptr,
                                          size_t length, pi_mem_advice advice,
                                          pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  ur_usm_advice_flags_t ur_advice = ConvertMemAdviseFlagsBitfield(advice);

  HANDLE_ERRORS(urEnqueueUSMAdvise(hQueue, ptr, length, ur_advice, phEvent));

  return PI_SUCCESS;
}

inline pi_result piextUSMEnqueueFill2D(pi_queue queue, void *ptr, size_t pitch,
                                       size_t pattern_size, const void *pattern,
                                       size_t width, size_t height,
                                       pi_uint32 num_events_in_waitlist,
                                       const pi_event *events_waitlist,
                                       pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(events_waitlist);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueUSMFill2D(hQueue, ptr, pitch, pattern_size, pattern,
                                   width, height, num_events_in_waitlist,
                                   phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piextUSMEnqueueMemset2D(pi_queue queue, void *ptr,
                                         size_t pitch, int value, size_t width,
                                         size_t height,
                                         pi_uint32 num_events_in_waitlist,
                                         const pi_event *events_waitlist,
                                         pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(events_waitlist);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueUSMFill2D(hQueue, ptr, pitch, 1, &value, width, height,
                                   num_events_in_waitlist, phEventWaitList,
                                   phEvent));

  return PI_SUCCESS;
}

inline pi_result piextUSMEnqueueMemcpy2D(pi_queue queue, pi_bool blocking,
                                         void *dst_ptr, size_t dst_pitch,
                                         const void *src_ptr, size_t src_pitch,
                                         size_t width, size_t height,
                                         pi_uint32 num_events_in_waitlist,
                                         const pi_event *events_waitlist,
                                         pi_event *event) {
  auto hQueue = reinterpret_cast<ur_queue_handle_t>(queue);
  auto phEventWaitList =
      reinterpret_cast<const ur_event_handle_t *>(events_waitlist);
  auto phEvent = reinterpret_cast<ur_event_handle_t *>(event);

  HANDLE_ERRORS(urEnqueueUSMMemcpy2D(
      hQueue, blocking, dst_ptr, dst_pitch, src_ptr, src_pitch, width, height,
      num_events_in_waitlist, phEventWaitList, phEvent));

  return PI_SUCCESS;
}

inline pi_result piextUSMGetMemAllocInfo(pi_context context, const void *ptr,
                                         pi_mem_alloc_info param_name,
                                         size_t param_value_size,
                                         void *param_value,
                                         size_t *param_value_size_ret) {
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);

  std::unordered_map<pi_mem_alloc_info, ur_usm_alloc_info_t> InfoMap = {
      {PI_MEM_ALLOC_TYPE, UR_USM_ALLOC_INFO_TYPE},
      {PI_MEM_ALLOC_BASE_PTR, UR_USM_ALLOC_INFO_BASE_PTR},
      {PI_MEM_ALLOC_SIZE, UR_USM_ALLOC_INFO_SIZE},
      {PI_MEM_ALLOC_DEVICE, UR_USM_ALLOC_INFO_DEVICE},
  };
  auto InfoMapIt = InfoMap.find(param_name);
  if (InfoMapIt == InfoMap.end()) {
    return PI_ERROR_UNKNOWN;
  }

  HANDLE_ERRORS(urUSMGetMemAllocInfo(hContext, ptr, InfoMapIt->second,
                                     param_value_size, param_value,
                                     param_value_size_ret));

  if (InfoMapIt->second == UR_USM_ALLOC_INFO_TYPE) {
    std::unordered_map<ur_usm_type_t, pi_usm_type> RetTypeMap = {
        {UR_USM_TYPE_UNKNOWN, PI_MEM_TYPE_UNKNOWN},
        {UR_USM_TYPE_DEVICE, PI_MEM_TYPE_DEVICE},
        {UR_USM_TYPE_HOST, PI_MEM_TYPE_HOST},
        {UR_USM_TYPE_SHARED, PI_MEM_TYPE_SHARED},
    };
    auto ur_mem_type = reinterpret_cast<ur_usm_type_t *>(param_value);
    auto RetTypeMapIt = RetTypeMap.find(*ur_mem_type);
    if (RetTypeMapIt == RetTypeMap.end()) {
      return PI_ERROR_UNKNOWN;
    }
    auto pi_mem_type = reinterpret_cast<pi_usm_type *>(param_value);
    *pi_mem_type = RetTypeMapIt->second;
  }

  return PI_SUCCESS;
}
// USM
///////////////////////////////////////////////////////////////////////////////

} // namespace pi2ur
