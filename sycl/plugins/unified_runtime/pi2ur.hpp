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
        return PI_EXT_PLATFORM_BACKEND_CUDA;
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
inline pi_result fixupInfoValueTypes(size_t ParamValueSizeUR,
                                     size_t *ParamValueSizeRetPI,
                                     void *ParamValue) {
  if (ParamValueSizeUR == 1) {
    // extend bool to pi_bool (uint32_t)
    auto *ValIn = static_cast<bool *>(ParamValue);
    auto *ValOut = static_cast<pi_bool *>(ParamValue);
    *ValOut = static_cast<pi_bool>(*ValIn);
    if (ParamValueSizeRetPI) {
      *ParamValueSizeRetPI = sizeof(pi_bool);
    }
  }

  return PI_SUCCESS;
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
  fixupInfoValueTypes(SizeInOut, ParamValueSizeRet, ParamValue);

  return PI_SUCCESS;
}

inline pi_result piPluginGetLastError(char **) { return PI_SUCCESS; }

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
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_BUILD_ON_SUBDEVICE;
    break;
  case PI_EXT_ONEAPI_DEVICE_INFO_MAX_WORK_GROUPS_3D:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_MAX_WORK_GROUPS_3D;
    break;
  case PI_DEVICE_INFO_IMAGE_MAX_ARRAY_SIZE:
    InfoType = (ur_device_info_t)UR_DEVICE_INFO_IMAGE_MAX_ARRAY_SIZE;
    break;
  case PI_DEVICE_INFO_DEVICE_ID:
    InfoType = (ur_device_info_t)UR_DEVICE_INFO_DEVICE_ID;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_FREE_MEMORY:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_FREE_MEMORY;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_MEMORY_CLOCK_RATE:
    InfoType = (ur_device_info_t)UR_DEVICE_INFO_MEMORY_CLOCK_RATE;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_MEMORY_BUS_WIDTH:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_MEMORY_BUS_WIDTH;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_MAX_COMPUTE_QUEUE_INDICES:
    InfoType = (ur_device_info_t)UR_DEVICE_INFO_MAX_COMPUTE_QUEUE_INDICES;
    break;
  case PI_DEVICE_INFO_GPU_SLICES:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_GPU_SLICES;
    break;
  case PI_DEVICE_INFO_GPU_EU_COUNT_PER_SUBSLICE:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_GPU_EU_COUNT_PER_SUBSLICE;
    break;
  case PI_DEVICE_INFO_GPU_HW_THREADS_PER_EU:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_GPU_HW_THREADS_PER_EU;
    break;
  case PI_DEVICE_INFO_MAX_MEM_BANDWIDTH:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_MAX_MEM_BANDWIDTH;
    break;
  case PI_EXT_ONEAPI_DEVICE_INFO_BFLOAT16_MATH_FUNCTIONS:
    InfoType = (ur_device_info_t)UR_DEVICE_INFO_BFLOAT16;
    break;
  case PI_EXT_DEVICE_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES:
    InfoType =
        (ur_device_info_t)UR_DEVICE_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES;
    break;
  case PI_EXT_DEVICE_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES:
    InfoType =
        (ur_device_info_t)UR_DEVICE_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES;
    break;
  case PI_EXT_DEVICE_INFO_ATOMIC_FENCE_ORDER_CAPABILITIES:
    InfoType = (ur_device_info_t)UR_DEVICE_INFO_ATOMIC_FENCE_ORDER_CAPABILITIES;
    break;
  case PI_EXT_DEVICE_INFO_ATOMIC_FENCE_SCOPE_CAPABILITIES:
    InfoType = (ur_device_info_t)UR_DEVICE_INFO_ATOMIC_FENCE_SCOPE_CAPABILITIES;
    break;
  case PI_EXT_INTEL_DEVICE_INFO_MEM_CHANNEL_SUPPORT:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_MEM_CHANNEL_SUPPORT;
    break;
  case PI_DEVICE_INFO_IMAGE_SRGB:
    InfoType = (ur_device_info_t)UR_DEVICE_INFO_IMAGE_SRGB;
    break;
  case PI_DEVICE_INFO_BACKEND_VERSION: {
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_BACKEND_VERSION;
    break;
  }
  case PI_EXT_ONEAPI_DEVICE_INFO_CUDA_ASYNC_BARRIER:
    InfoType = (ur_device_info_t)UR_EXT_DEVICE_INFO_CUDA_ASYNC_BARRIER;
    break;
  default:
    return PI_ERROR_UNKNOWN;
  };

  size_t SizeInOut = ParamValueSize;
  auto hDevice = reinterpret_cast<ur_device_handle_t>(Device);
  HANDLE_ERRORS(urDeviceGetInfo(hDevice, InfoType, SizeInOut, ParamValue,
                                ParamValueSizeRet));

  ur2piDeviceInfoValue(InfoType, ParamValueSize, &SizeInOut, ParamValue);
  fixupInfoValueTypes(SizeInOut, ParamValueSizeRet, ParamValue);

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
       (ur_context_info_t)UR_EXT_CONTEXT_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES},
      {PI_EXT_CONTEXT_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES,
       (ur_context_info_t)UR_EXT_CONTEXT_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES},
      {PI_EXT_ONEAPI_CONTEXT_INFO_USM_MEMCPY2D_SUPPORT,
       UR_CONTEXT_INFO_USM_MEMCPY2D_SUPPORT},
      {PI_EXT_ONEAPI_CONTEXT_INFO_USM_FILL2D_SUPPORT,
       UR_CONTEXT_INFO_USM_FILL2D_SUPPORT},
      {PI_EXT_ONEAPI_CONTEXT_INFO_USM_MEMSET2D_SUPPORT,
       (ur_context_info_t)UR_EXT_CONTEXT_INFO_USM_MEMSET2D_SUPPORT},
  };

  auto InfoType = InfoMapping.find(param_name);
  if (InfoType == InfoMapping.end()) {
    return PI_ERROR_UNKNOWN;
  }

  size_t SizeInOut = param_value_size;
  auto hContext = reinterpret_cast<ur_context_handle_t>(context);
  HANDLE_ERRORS(urContextGetInfo(hContext, InfoType->second, SizeInOut,
                                 param_value, param_value_size_ret));
  fixupInfoValueTypes(SizeInOut, param_value_size_ret, param_value);

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

  // Note that we ignore the devices and ownership arguments here. This is
  // enough for CUDA, HIP and OpenCL.

  HANDLE_ERRORS(urContextCreateWithNativeHandle(hNativeHandle, phContext));

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

inline pi_result piKernelSetArgPointer(pi_kernel kernel, pi_uint32 arg_index,
                                       size_t arg_size, const void *arg_value) {
  (void)arg_size;
  auto hKernel = reinterpret_cast<ur_kernel_handle_t>(kernel);
  HANDLE_ERRORS(urKernelSetArgPointer(hKernel, arg_index, arg_value));

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
  auto phKernel = reinterpret_cast<ur_kernel_handle_t *>(kernel);

  HANDLE_ERRORS(
      urKernelCreateWithNativeHandle(hNativeKernel, hContext, phKernel));

  return PI_SUCCESS;
}
// Kernel
///////////////////////////////////////////////////////////////////////////////
} // namespace pi2ur
