//==---------- pi_cuda.cpp - CUDA Plugin -----------------------------------==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

/// \file pi_cuda.cpp
/// Implementation of CUDA Plugin.
///
/// \ingroup sycl_pi_cuda

#include <pi_cuda.hpp>
#include <sycl/detail/cuda_definitions.hpp>
#include <sycl/detail/defines.hpp>
#include <sycl/detail/pi.hpp>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cuda.h>
#include <cuda_device_runtime_api.h>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <string_view>

// Forward declarations
void enableCUDATracing();
void disableCUDATracing();

namespace {
pi_result map_error(CUresult result) {
  switch (result) {
  case CUDA_SUCCESS:
    return PI_SUCCESS;
  case CUDA_ERROR_NOT_PERMITTED:
    return PI_ERROR_INVALID_OPERATION;
  case CUDA_ERROR_INVALID_CONTEXT:
    return PI_ERROR_INVALID_CONTEXT;
  case CUDA_ERROR_INVALID_DEVICE:
    return PI_ERROR_INVALID_DEVICE;
  case CUDA_ERROR_INVALID_VALUE:
    return PI_ERROR_INVALID_VALUE;
  case CUDA_ERROR_OUT_OF_MEMORY:
    return PI_ERROR_OUT_OF_HOST_MEMORY;
  case CUDA_ERROR_LAUNCH_OUT_OF_RESOURCES:
    return PI_ERROR_OUT_OF_RESOURCES;
  default:
    return PI_ERROR_UNKNOWN;
  }
}

// Returns plugin specific backend option.
// Current support is only for optimization options.
// Return empty string for cuda.
// TODO: Determine correct string to be passed.
pi_result cuda_piPluginGetBackendOption(pi_platform,
                                        const char *frontend_option,
                                        const char **backend_option) {
  using namespace std::literals;
  if (frontend_option == nullptr)
    return PI_ERROR_INVALID_VALUE;
  if (frontend_option == "-O0"sv || frontend_option == "-O1"sv ||
      frontend_option == "-O2"sv || frontend_option == "-O3"sv ||
      frontend_option == ""sv) {
    *backend_option = "";
    return PI_SUCCESS;
  }
  return PI_ERROR_INVALID_VALUE;
}

pi_result map_ur_error(ur_result_t result) {
  switch (result) {
  case UR_RESULT_SUCCESS:
    return PI_SUCCESS;
  case UR_RESULT_ERROR_INVALID_OPERATION:
    return PI_ERROR_INVALID_OPERATION;
  case UR_RESULT_ERROR_INVALID_CONTEXT:
    return PI_ERROR_INVALID_CONTEXT;
  case UR_RESULT_ERROR_INVALID_DEVICE:
    return PI_ERROR_INVALID_DEVICE;
  case UR_RESULT_ERROR_INVALID_VALUE:
    return PI_ERROR_INVALID_VALUE;
  case UR_RESULT_ERROR_OUT_OF_HOST_MEMORY:
    return PI_ERROR_OUT_OF_HOST_MEMORY;
  case UR_RESULT_ERROR_OUT_OF_RESOURCES:
    return PI_ERROR_OUT_OF_RESOURCES;
  default:
    return PI_ERROR_UNKNOWN;
  }
}

// Iterates over the event wait list, returns correct pi_result error codes.
// Invokes the callback for the latest event of each queue in the wait list.
// The callback must take a single pi_event argument and return a pi_result.
template <typename Func>
pi_result forLatestEvents(const pi_event *event_wait_list,
                          std::size_t num_events_in_wait_list, Func &&f) {

  if (event_wait_list == nullptr || num_events_in_wait_list == 0) {
    return PI_ERROR_INVALID_EVENT_WAIT_LIST;
  }

  // Fast path if we only have a single event
  if (num_events_in_wait_list == 1) {
    return f(event_wait_list[0]);
  }

  std::vector<pi_event> events{event_wait_list,
                               event_wait_list + num_events_in_wait_list};
  std::sort(events.begin(), events.end(), [](pi_event e0, pi_event e1) {
    // Tiered sort creating sublists of streams (smallest value first) in which
    // the corresponding events are sorted into a sequence of newest first.
    return e0->get_stream() < e1->get_stream() ||
           (e0->get_stream() == e1->get_stream() &&
            e0->get_event_id() > e1->get_event_id());
  });

  bool first = true;
  CUstream lastSeenStream = 0;
  for (pi_event event : events) {
    if (!event || (!first && event->get_stream() == lastSeenStream)) {
      continue;
    }

    first = false;
    lastSeenStream = event->get_stream();

    auto result = f(event);
    if (result != PI_SUCCESS) {
      return result;
    }
  }

  return PI_SUCCESS;
}

/// Converts CUDA error into PI error codes, and outputs error information
/// to stderr.
/// If PI_CUDA_ABORT env variable is defined, it aborts directly instead of
/// throwing the error. This is intended for debugging purposes.
/// \return PI_SUCCESS if \param result was CUDA_SUCCESS.
/// \throw pi_error exception (integer) if input was not success.
///
pi_result check_error(CUresult result, const char *function, int line,
                      const char *file) {
  if (result == CUDA_SUCCESS || result == CUDA_ERROR_DEINITIALIZED) {
    return PI_SUCCESS;
  }

  if (std::getenv("SYCL_PI_SUPPRESS_ERROR_MESSAGE") == nullptr) {
    const char *errorString = nullptr;
    const char *errorName = nullptr;
    cuGetErrorName(result, &errorName);
    cuGetErrorString(result, &errorString);
    std::stringstream ss;
    ss << "\nPI CUDA ERROR:"
       << "\n\tValue:           " << result
       << "\n\tName:            " << errorName
       << "\n\tDescription:     " << errorString
       << "\n\tFunction:        " << function << "\n\tSource Location: " << file
       << ":" << line << "\n"
       << std::endl;
    std::cerr << ss.str();
  }

  if (std::getenv("PI_CUDA_ABORT") != nullptr) {
    std::abort();
  }

  throw map_error(result);
}

/// \cond NODOXY
#define PI_CHECK_ERROR(result) check_error(result, __func__, __LINE__, __FILE__)

ScopedContext::ScopedContext(pi_context ctxt) {
  if (!ctxt) {
    throw PI_ERROR_INVALID_CONTEXT;
  }

  set_context(ctxt->get());
}

/// \cond NODOXY
template <typename T, typename Assign>
pi_result getInfoImpl(size_t param_value_size, void *param_value,
                      size_t *param_value_size_ret, T value, size_t value_size,
                      Assign &&assign_func) {

  if (param_value != nullptr) {

    if (param_value_size < value_size) {
      return PI_ERROR_INVALID_VALUE;
    }

    assign_func(param_value, value, value_size);
  }

  if (param_value_size_ret != nullptr) {
    *param_value_size_ret = value_size;
  }

  return PI_SUCCESS;
}

template <typename T>
pi_result getInfo(size_t param_value_size, void *param_value,
                  size_t *param_value_size_ret, T value) {

  auto assignment = [](void *param_value, T value, size_t value_size) {
    // Ignore unused parameter
    (void)value_size;

    *static_cast<T *>(param_value) = value;
  };

  return getInfoImpl(param_value_size, param_value, param_value_size_ret, value,
                     sizeof(T), assignment);
}

template <typename T>
pi_result getInfoArray(size_t array_length, size_t param_value_size,
                       void *param_value, size_t *param_value_size_ret,
                       T *value) {
  return getInfoImpl(param_value_size, param_value, param_value_size_ret, value,
                     array_length * sizeof(T), memcpy);
}

int getAttribute(pi_device device, CUdevice_attribute attribute) {
  int value;
  sycl::detail::pi::assertion(
      cuDeviceGetAttribute(&value, attribute, device->get()) == CUDA_SUCCESS);
  return value;
}
/// \endcond

// Determine local work sizes that result in uniform work groups.
// The default threadsPerBlock only require handling the first work_dim
// dimension.
void guessLocalWorkSize(_pi_device *device, size_t *threadsPerBlock,
                        const size_t *global_work_size,
                        const size_t maxThreadsPerBlock[3], pi_kernel kernel,
                        pi_uint32 local_size) {
  assert(threadsPerBlock != nullptr);
  assert(global_work_size != nullptr);
  assert(kernel != nullptr);
  int minGrid, maxBlockSize, gridDim[3];

  cuDeviceGetAttribute(&gridDim[1], CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_Y,
                       device->get());
  cuDeviceGetAttribute(&gridDim[2], CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_Z,
                       device->get());

  threadsPerBlock[1] = ((global_work_size[1] - 1) / gridDim[1]) + 1;
  threadsPerBlock[2] = ((global_work_size[2] - 1) / gridDim[2]) + 1;

  PI_CHECK_ERROR(cuOccupancyMaxPotentialBlockSize(
      &minGrid, &maxBlockSize, kernel->get(), NULL, local_size,
      maxThreadsPerBlock[0]));

  gridDim[0] = maxBlockSize / (threadsPerBlock[1] * threadsPerBlock[2]);

  threadsPerBlock[0] =
      std::min(maxThreadsPerBlock[0],
               std::min(global_work_size[0], static_cast<size_t>(gridDim[0])));

  // Find a local work group size that is a divisor of the global
  // work group size to produce uniform work groups.
  while (0u != (global_work_size[0] % threadsPerBlock[0])) {
    --threadsPerBlock[0];
  }
}

pi_result enqueueEventsWait(pi_queue command_queue, CUstream stream,
                            pi_uint32 num_events_in_wait_list,
                            const pi_event *event_wait_list) {
  if (!event_wait_list) {
    return PI_SUCCESS;
  }
  try {
    ScopedContext active(command_queue->get_context());

    auto result = forLatestEvents(
        event_wait_list, num_events_in_wait_list,
        [stream](pi_event event) -> pi_result {
          if (event->get_stream() == stream) {
            return PI_SUCCESS;
          } else {
            return PI_CHECK_ERROR(cuStreamWaitEvent(stream, event->get(), 0));
          }
        });

    if (result != PI_SUCCESS) {
      return result;
    }
    return PI_SUCCESS;
  } catch (pi_result err) {
    return err;
  } catch (...) {
    return PI_ERROR_UNKNOWN;
  }
}

template <typename PtrT>
void getUSMHostOrDevicePtr(PtrT usm_ptr, CUmemorytype *out_mem_type,
                           CUdeviceptr *out_dev_ptr, PtrT *out_host_ptr) {
  // do not throw if cuPointerGetAttribute returns CUDA_ERROR_INVALID_VALUE
  // checks with PI_CHECK_ERROR are not suggested
  CUresult ret = cuPointerGetAttribute(
      out_mem_type, CU_POINTER_ATTRIBUTE_MEMORY_TYPE, (CUdeviceptr)usm_ptr);
  assert((*out_mem_type != CU_MEMORYTYPE_ARRAY &&
          *out_mem_type != CU_MEMORYTYPE_UNIFIED) &&
         "ARRAY, UNIFIED types are not supported!");

  // pointer not known to the CUDA subsystem (possibly a system allocated ptr)
  if (ret == CUDA_ERROR_INVALID_VALUE) {
    *out_mem_type = CU_MEMORYTYPE_HOST;
    *out_dev_ptr = 0;
    *out_host_ptr = usm_ptr;

    // todo: resets the above "non-stick" error
  } else if (ret == CUDA_SUCCESS) {
    *out_dev_ptr = (*out_mem_type == CU_MEMORYTYPE_DEVICE)
                       ? reinterpret_cast<CUdeviceptr>(usm_ptr)
                       : 0;
    *out_host_ptr = (*out_mem_type == CU_MEMORYTYPE_HOST) ? usm_ptr : nullptr;
  } else {
    PI_CHECK_ERROR(ret);
  }
}

} // anonymous namespace

/// ------ Error handling, matching OpenCL plugin semantics.
namespace sycl {
__SYCL_INLINE_VER_NAMESPACE(_V1) {
namespace detail {
namespace pi {

// Report error and no return (keeps compiler from printing warnings).
// TODO: Probably change that to throw a catchable exception,
//       but for now it is useful to see every failure.
//
[[noreturn]] void die(const char *Message) {
  std::cerr << "pi_die: " << Message << std::endl;
  std::terminate();
}

// Reports error messages
void cuPrint(const char *Message) {
  std::cerr << "pi_print: " << Message << std::endl;
}

void assertion(bool Condition, const char *Message) {
  if (!Condition)
    die(Message);
}

} // namespace pi
} // namespace detail
} // __SYCL_INLINE_VER_NAMESPACE(_V1)
} // namespace sycl

//--------------
// PI object implementation

extern "C" {

// Required in a number of functions, so forward declare here
pi_result cuda_piEnqueueEventsWait(pi_queue command_queue,
                                   pi_uint32 num_events_in_wait_list,
                                   const pi_event *event_wait_list,
                                   pi_event *event);
pi_result cuda_piEnqueueEventsWaitWithBarrier(pi_queue command_queue,
                                              pi_uint32 num_events_in_wait_list,
                                              const pi_event *event_wait_list,
                                              pi_event *event);

} // extern "C"

/// \endcond

// makes all future work submitted to queue wait for all work captured in event.
pi_result enqueueEventWait(pi_queue queue, pi_event event) {
  // for native events, the cuStreamWaitEvent call is used.
  // This makes all future work submitted to stream wait for all
  // work captured in event.
  queue->for_each_stream([e = event->get()](CUstream s) {
    PI_CHECK_ERROR(cuStreamWaitEvent(s, e, 0));
  });
  return PI_SUCCESS;
}

//-- PI API implementation
extern "C" {
pi_result cuda_piContextGetInfo(pi_context context, pi_context_info param_name,
                                size_t param_value_size, void *param_value,
                                size_t *param_value_size_ret) {

  switch (param_name) {
  case PI_CONTEXT_INFO_NUM_DEVICES:
    return getInfo(param_value_size, param_value, param_value_size_ret, 1);
  case PI_CONTEXT_INFO_DEVICES:
    return getInfo(param_value_size, param_value, param_value_size_ret,
                   context->get_device());
  case PI_CONTEXT_INFO_REFERENCE_COUNT:
    return getInfo(param_value_size, param_value, param_value_size_ret,
                   context->get_reference_count());
  case PI_EXT_CONTEXT_INFO_ATOMIC_MEMORY_ORDER_CAPABILITIES:
  case PI_EXT_CONTEXT_INFO_ATOMIC_MEMORY_SCOPE_CAPABILITIES:
  case PI_EXT_CONTEXT_INFO_ATOMIC_FENCE_ORDER_CAPABILITIES:
  case PI_EXT_CONTEXT_INFO_ATOMIC_FENCE_SCOPE_CAPABILITIES: {
    // These queries should be dealt with in context_impl.cpp by calling the
    // queries of each device separately and building the intersection set.
    setErrorMessage("These queries should have never come here.",
                    UR_RESULT_ERROR_INVALID_ARGUMENT);
    return PI_ERROR_PLUGIN_SPECIFIC_ERROR;
  }
  case PI_EXT_ONEAPI_CONTEXT_INFO_USM_MEMCPY2D_SUPPORT:
    return getInfo<pi_bool>(param_value_size, param_value, param_value_size_ret,
                            true);
  case PI_EXT_ONEAPI_CONTEXT_INFO_USM_FILL2D_SUPPORT:
  case PI_EXT_ONEAPI_CONTEXT_INFO_USM_MEMSET2D_SUPPORT:
    // 2D USM operations currently not supported.
    return getInfo<pi_bool>(param_value_size, param_value, param_value_size_ret,
                            false);
  default:
    __SYCL_PI_HANDLE_UNKNOWN_PARAM_NAME(param_name);
  }

  return PI_ERROR_OUT_OF_RESOURCES;
}

/// \return If available, the first binary that is PTX
///
pi_result cuda_piextDeviceSelectBinary(pi_device device,
                                       pi_device_binary *binaries,
                                       pi_uint32 num_binaries,
                                       pi_uint32 *selected_binary) {
  // Ignore unused parameter
  (void)device;

  if (!binaries) {
    sycl::detail::pi::die("No list of device images provided");
  }
  if (num_binaries < 1) {
    sycl::detail::pi::die("No binary images in the list");
  }

  // Look for an image for the NVPTX64 target, and return the first one that is
  // found
  for (pi_uint32 i = 0; i < num_binaries; i++) {
    if (strcmp(binaries[i]->DeviceTargetSpec,
               __SYCL_PI_DEVICE_BINARY_TARGET_NVPTX64) == 0) {
      *selected_binary = i;
      return PI_SUCCESS;
    }
  }

  // No image can be loaded for the given device
  return PI_ERROR_INVALID_BINARY;
}

pi_result cuda_piextGetDeviceFunctionPointer(pi_device device,
                                             pi_program program,
                                             const char *func_name,
                                             pi_uint64 *func_pointer_ret) {
  // Check if device passed is the same the device bound to the context
  assert(device == program->get_context()->get_device());
  assert(func_pointer_ret != nullptr);

  CUfunction func;
  CUresult ret = cuModuleGetFunction(&func, program->get(), func_name);
  *func_pointer_ret = reinterpret_cast<pi_uint64>(func);
  pi_result retError = PI_SUCCESS;

  if (ret != CUDA_SUCCESS && ret != CUDA_ERROR_NOT_FOUND)
    retError = PI_CHECK_ERROR(ret);
  if (ret == CUDA_ERROR_NOT_FOUND) {
    *func_pointer_ret = 0;
    retError = PI_ERROR_INVALID_KERNEL_NAME;
  }

  return retError;
}

/// Creates a PI Memory object using a CUDA memory allocation.
/// Can trigger a manual copy depending on the mode.
/// \TODO Implement USE_HOST_PTR using cuHostRegister
///
pi_result cuda_piMemBufferCreate(pi_context context, pi_mem_flags flags,
                                 size_t size, void *host_ptr, pi_mem *ret_mem,
                                 const pi_mem_properties *properties) {
  // Need input memory object
  assert(ret_mem != nullptr);
  assert((properties == nullptr || *properties == 0) &&
         "no mem properties goes to cuda RT yet");
  // Currently, USE_HOST_PTR is not implemented using host register
  // since this triggers a weird segfault after program ends.
  // Setting this constant to true enables testing that behavior.
  const bool enableUseHostPtr = false;
  const bool performInitialCopy =
      (flags & PI_MEM_FLAGS_HOST_PTR_COPY) ||
      ((flags & PI_MEM_FLAGS_HOST_PTR_USE) && !enableUseHostPtr);
  pi_result retErr = PI_SUCCESS;
  pi_mem retMemObj = nullptr;

  try {
    ScopedContext active(context);
    CUdeviceptr ptr;
    _pi_mem::mem_::buffer_mem_::alloc_mode allocMode =
        _pi_mem::mem_::buffer_mem_::alloc_mode::classic;

    if ((flags & PI_MEM_FLAGS_HOST_PTR_USE) && enableUseHostPtr) {
      retErr = PI_CHECK_ERROR(
          cuMemHostRegister(host_ptr, size, CU_MEMHOSTREGISTER_DEVICEMAP));
      retErr = PI_CHECK_ERROR(cuMemHostGetDevicePointer(&ptr, host_ptr, 0));
      allocMode = _pi_mem::mem_::buffer_mem_::alloc_mode::use_host_ptr;
    } else if (flags & PI_MEM_FLAGS_HOST_PTR_ALLOC) {
      retErr = PI_CHECK_ERROR(cuMemAllocHost(&host_ptr, size));
      retErr = PI_CHECK_ERROR(cuMemHostGetDevicePointer(&ptr, host_ptr, 0));
      allocMode = _pi_mem::mem_::buffer_mem_::alloc_mode::alloc_host_ptr;
    } else {
      retErr = PI_CHECK_ERROR(cuMemAlloc(&ptr, size));
      if (flags & PI_MEM_FLAGS_HOST_PTR_COPY) {
        allocMode = _pi_mem::mem_::buffer_mem_::alloc_mode::copy_in;
      }
    }

    if (retErr == PI_SUCCESS) {
      pi_mem parentBuffer = nullptr;

      auto piMemObj = std::unique_ptr<_pi_mem>(
          new _pi_mem{context, parentBuffer, allocMode, ptr, host_ptr, size});
      if (piMemObj != nullptr) {
        retMemObj = piMemObj.release();
        if (performInitialCopy) {
          // Operates on the default stream of the current CUDA context.
          retErr = PI_CHECK_ERROR(cuMemcpyHtoD(ptr, host_ptr, size));
          // Synchronize with default stream implicitly used by cuMemcpyHtoD
          // to make buffer data available on device before any other PI call
          // uses it.
          if (retErr == PI_SUCCESS) {
            CUstream defaultStream = 0;
            retErr = PI_CHECK_ERROR(cuStreamSynchronize(defaultStream));
          }
        }
      } else {
        retErr = PI_ERROR_OUT_OF_HOST_MEMORY;
      }
    }
  } catch (pi_result err) {
    retErr = err;
  } catch (...) {
    retErr = PI_ERROR_OUT_OF_RESOURCES;
  }

  *ret_mem = retMemObj;

  return retErr;
}

/// Decreases the reference count of the Mem object.
/// If this is zero, calls the relevant CUDA Free function
/// \return PI_SUCCESS unless deallocation error
///
pi_result cuda_piMemRelease(pi_mem memObj) {
  assert((memObj != nullptr) && "PI_ERROR_INVALID_MEM_OBJECTS");

  pi_result ret = PI_SUCCESS;

  try {

    // Do nothing if there are other references
    if (memObj->decrement_reference_count() > 0) {
      return PI_SUCCESS;
    }

    // make sure memObj is released in case PI_CHECK_ERROR throws
    std::unique_ptr<_pi_mem> uniqueMemObj(memObj);

    if (memObj->is_sub_buffer()) {
      return PI_SUCCESS;
    }

    ScopedContext active(uniqueMemObj->get_context());

    if (memObj->mem_type_ == _pi_mem::mem_type::buffer) {
      switch (uniqueMemObj->mem_.buffer_mem_.allocMode_) {
      case _pi_mem::mem_::buffer_mem_::alloc_mode::copy_in:
      case _pi_mem::mem_::buffer_mem_::alloc_mode::classic:
        ret = PI_CHECK_ERROR(cuMemFree(uniqueMemObj->mem_.buffer_mem_.ptr_));
        break;
      case _pi_mem::mem_::buffer_mem_::alloc_mode::use_host_ptr:
        ret = PI_CHECK_ERROR(
            cuMemHostUnregister(uniqueMemObj->mem_.buffer_mem_.hostPtr_));
        break;
      case _pi_mem::mem_::buffer_mem_::alloc_mode::alloc_host_ptr:
        ret = PI_CHECK_ERROR(
            cuMemFreeHost(uniqueMemObj->mem_.buffer_mem_.hostPtr_));
      };
    } else if (memObj->mem_type_ == _pi_mem::mem_type::surface) {
      ret = PI_CHECK_ERROR(
          cuSurfObjectDestroy(uniqueMemObj->mem_.surface_mem_.get_surface()));
      ret = PI_CHECK_ERROR(
          cuArrayDestroy(uniqueMemObj->mem_.surface_mem_.get_array()));
    }

  } catch (pi_result err) {
    ret = err;
  } catch (...) {
    ret = PI_ERROR_OUT_OF_RESOURCES;
  }

  if (ret != PI_SUCCESS) {
    // A reported CUDA error is either an implementation or an asynchronous CUDA
    // error for which it is unclear if the function that reported it succeeded
    // or not. Either way, the state of the program is compromised and likely
    // unrecoverable.
    sycl::detail::pi::die(
        "Unrecoverable program state reached in cuda_piMemRelease");
  }

  return PI_SUCCESS;
}

/// Implements a buffer partition in the CUDA backend.
/// A buffer partition (or a sub-buffer, in OpenCL terms) is simply implemented
/// as an offset over an existing CUDA allocation.
///
pi_result cuda_piMemBufferPartition(pi_mem parent_buffer, pi_mem_flags flags,
                                    pi_buffer_create_type buffer_create_type,
                                    void *buffer_create_info, pi_mem *memObj) {
  assert((parent_buffer != nullptr) && "PI_ERROR_INVALID_MEM_OBJECT");
  assert(parent_buffer->is_buffer() && "PI_ERROR_INVALID_MEM_OBJECTS");
  assert(!parent_buffer->is_sub_buffer() && "PI_ERROR_INVALID_MEM_OBJECT");

  // Default value for flags means PI_MEM_FLAGS_ACCCESS_RW.
  if (flags == 0) {
    flags = PI_MEM_FLAGS_ACCESS_RW;
  }

  assert((flags == PI_MEM_FLAGS_ACCESS_RW) && "PI_ERROR_INVALID_VALUE");
  assert((buffer_create_type == PI_BUFFER_CREATE_TYPE_REGION) &&
         "PI_ERROR_INVALID_VALUE");
  assert((buffer_create_info != nullptr) && "PI_ERROR_INVALID_VALUE");
  assert(memObj != nullptr);

  const auto bufferRegion =
      *reinterpret_cast<pi_buffer_region>(buffer_create_info);
  assert((bufferRegion.size != 0u) && "PI_ERROR_INVALID_BUFFER_SIZE");

  assert((bufferRegion.origin <= (bufferRegion.origin + bufferRegion.size)) &&
         "Overflow");
  assert(((bufferRegion.origin + bufferRegion.size) <=
          parent_buffer->mem_.buffer_mem_.get_size()) &&
         "PI_ERROR_INVALID_BUFFER_SIZE");
  // Retained indirectly due to retaining parent buffer below.
  pi_context context = parent_buffer->context_;
  _pi_mem::mem_::buffer_mem_::alloc_mode allocMode =
      _pi_mem::mem_::buffer_mem_::alloc_mode::classic;

  assert(parent_buffer->mem_.buffer_mem_.ptr_ !=
         _pi_mem::mem_::buffer_mem_::native_type{0});
  _pi_mem::mem_::buffer_mem_::native_type ptr =
      parent_buffer->mem_.buffer_mem_.ptr_ + bufferRegion.origin;

  void *hostPtr = nullptr;
  if (parent_buffer->mem_.buffer_mem_.hostPtr_) {
    hostPtr = static_cast<char *>(parent_buffer->mem_.buffer_mem_.hostPtr_) +
              bufferRegion.origin;
  }

  std::unique_ptr<_pi_mem> retMemObj{nullptr};
  try {
    retMemObj = std::unique_ptr<_pi_mem>{new _pi_mem{
        context, parent_buffer, allocMode, ptr, hostPtr, bufferRegion.size}};
  } catch (pi_result err) {
    *memObj = nullptr;
    return err;
  } catch (...) {
    *memObj = nullptr;
    return PI_ERROR_OUT_OF_HOST_MEMORY;
  }

  *memObj = retMemObj.release();
  return PI_SUCCESS;
}

pi_result cuda_piMemGetInfo(pi_mem, pi_mem_info, size_t, void *, size_t *) {
  sycl::detail::pi::die("cuda_piMemGetInfo not implemented");
}

/// Gets the native CUDA handle of a PI mem object
///
/// \param[in] mem The PI mem to get the native CUDA object of.
/// \param[out] nativeHandle Set to the native handle of the PI mem object.
///
/// \return PI_SUCCESS
pi_result cuda_piextMemGetNativeHandle(pi_mem mem,
                                       pi_native_handle *nativeHandle) {
  *nativeHandle = static_cast<pi_native_handle>(mem->mem_.buffer_mem_.get());
  return PI_SUCCESS;
}

/// Created a PI mem object from a CUDA mem handle.
/// TODO: Implement this.
/// NOTE: The created PI object takes ownership of the native handle.
///
/// \param[in] nativeHandle The native handle to create PI mem object from.
/// \param[in] context The PI context of the memory allocation.
/// \param[in] ownNativeHandle Indicates if we own the native memory handle or
/// it came from interop that asked to not transfer the ownership to SYCL RT.
/// \param[out] mem Set to the PI mem object created from native handle.
///
/// \return TBD
pi_result cuda_piextMemCreateWithNativeHandle(pi_native_handle nativeHandle,
                                              pi_context context,
                                              bool ownNativeHandle,
                                              pi_mem *mem) {
  sycl::detail::pi::die(
      "Creation of PI mem from native handle not implemented");
  return {};
}

/// Created a PI image mem object from a CUDA image mem handle.
/// TODO: Implement this.
/// NOTE: The created PI object takes ownership of the native handle.
///
/// \param[in] pi_native_handle The native handle to create PI mem object from.
/// \param[in] pi_context The PI context of the memory allocation.
/// \param[in] ownNativeHandle Boolean indicates if we own the native memory
/// handle or it came from interop that asked to not transfer the ownership to
/// SYCL RT. \param[in] pi_image_format The format of the image. \param[in]
/// pi_image_desc The description information for the image. \param[out] pi_mem
/// Set to the PI mem object created from native handle.
///
/// \return TBD
pi_result cuda_piextMemImageCreateWithNativeHandle(pi_native_handle, pi_context,
                                                   bool,
                                                   const pi_image_format *,
                                                   const pi_image_desc *,
                                                   pi_mem *) {
  sycl::detail::pi::die(
      "Creation of PI mem from native image handle not implemented");
  return {};
}

pi_result cuda_piEnqueueMemBufferWrite(pi_queue command_queue, pi_mem buffer,
                                       pi_bool blocking_write, size_t offset,
                                       size_t size, const void *ptr,
                                       pi_uint32 num_events_in_wait_list,
                                       const pi_event *event_wait_list,
                                       pi_event *event) {

  assert(buffer != nullptr);
  assert(command_queue != nullptr);
  pi_result retErr = PI_SUCCESS;
  CUdeviceptr devPtr = buffer->mem_.buffer_mem_.get();
  std::unique_ptr<_pi_event> retImplEv{nullptr};

  try {
    ScopedContext active(command_queue->get_context());
    CUstream cuStream = command_queue->get_next_transfer_stream();

    retErr = enqueueEventsWait(command_queue, cuStream, num_events_in_wait_list,
                               event_wait_list);

    if (event) {
      retImplEv = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_WRITE, command_queue, cuStream));
      retImplEv->start();
    }

    retErr =
        PI_CHECK_ERROR(cuMemcpyHtoDAsync(devPtr + offset, ptr, size, cuStream));

    if (event) {
      retErr = map_ur_error(retImplEv->record());
    }

    if (blocking_write) {
      retErr = PI_CHECK_ERROR(cuStreamSynchronize(cuStream));
    }

    if (event) {
      *event = retImplEv.release();
    }
  } catch (pi_result err) {
    retErr = err;
  }
  return retErr;
}

pi_result cuda_piEnqueueMemBufferRead(pi_queue command_queue, pi_mem buffer,
                                      pi_bool blocking_read, size_t offset,
                                      size_t size, void *ptr,
                                      pi_uint32 num_events_in_wait_list,
                                      const pi_event *event_wait_list,
                                      pi_event *event) {

  assert(buffer != nullptr);
  assert(command_queue != nullptr);
  pi_result retErr = PI_SUCCESS;
  CUdeviceptr devPtr = buffer->mem_.buffer_mem_.get();
  std::unique_ptr<_pi_event> retImplEv{nullptr};

  try {
    ScopedContext active(command_queue->get_context());
    CUstream cuStream = command_queue->get_next_transfer_stream();

    retErr = enqueueEventsWait(command_queue, cuStream, num_events_in_wait_list,
                               event_wait_list);

    if (event) {
      retImplEv = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_READ, command_queue, cuStream));
      retImplEv->start();
    }

    retErr =
        PI_CHECK_ERROR(cuMemcpyDtoHAsync(ptr, devPtr + offset, size, cuStream));

    if (event) {
      retErr = map_ur_error(retImplEv->record());
    }

    if (blocking_read) {
      retErr = PI_CHECK_ERROR(cuStreamSynchronize(cuStream));
    }

    if (event) {
      *event = retImplEv.release();
    }

  } catch (pi_result err) {
    retErr = err;
  }
  return retErr;
}

pi_result cuda_piextKernelSetArgMemObj(pi_kernel kernel, pi_uint32 arg_index,
                                       const pi_mem *arg_value) {

  assert(kernel != nullptr);
  assert(arg_value != nullptr);

  pi_result retErr = PI_SUCCESS;
  try {
    pi_mem arg_mem = *arg_value;
    if (arg_mem->mem_type_ == _pi_mem::mem_type::surface) {
      CUDA_ARRAY3D_DESCRIPTOR arrayDesc;
      PI_CHECK_ERROR(cuArray3DGetDescriptor(
          &arrayDesc, arg_mem->mem_.surface_mem_.get_array()));
      if (arrayDesc.Format != CU_AD_FORMAT_UNSIGNED_INT32 &&
          arrayDesc.Format != CU_AD_FORMAT_SIGNED_INT32 &&
          arrayDesc.Format != CU_AD_FORMAT_HALF &&
          arrayDesc.Format != CU_AD_FORMAT_FLOAT) {
        setErrorMessage("PI CUDA kernels only support images with channel "
                        "types int32, uint32, float, and half.",
                        UR_RESULT_ERROR_ADAPTER_SPECIFIC);
        return PI_ERROR_PLUGIN_SPECIFIC_ERROR;
      }
      CUsurfObject cuSurf = arg_mem->mem_.surface_mem_.get_surface();
      kernel->set_kernel_arg(arg_index, sizeof(cuSurf), (void *)&cuSurf);
    } else {
      CUdeviceptr cuPtr = arg_mem->mem_.buffer_mem_.get();
      kernel->set_kernel_arg(arg_index, sizeof(CUdeviceptr), (void *)&cuPtr);
    }
  } catch (pi_result err) {
    retErr = err;
  }
  return retErr;
}

pi_result cuda_piextKernelSetArgSampler(pi_kernel kernel, pi_uint32 arg_index,
                                        const pi_sampler *arg_value) {

  assert(kernel != nullptr);
  assert(arg_value != nullptr);

  pi_result retErr = PI_SUCCESS;
  try {
    pi_uint32 samplerProps = (*arg_value)->props_;
    kernel->set_kernel_arg(arg_index, sizeof(pi_uint32), (void *)&samplerProps);
  } catch (pi_result err) {
    retErr = err;
  }
  return retErr;
}

pi_result cuda_piEnqueueKernelLaunch(
    pi_queue command_queue, pi_kernel kernel, pi_uint32 work_dim,
    const size_t *global_work_offset, const size_t *global_work_size,
    const size_t *local_work_size, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {

  // Preconditions
  assert(command_queue != nullptr);
  assert(command_queue->get_context() == kernel->get_context());
  assert(kernel != nullptr);
  assert(global_work_offset != nullptr);
  assert(work_dim > 0);
  assert(work_dim < 4);

  if (*global_work_size == 0) {
    return cuda_piEnqueueEventsWaitWithBarrier(
        command_queue, num_events_in_wait_list, event_wait_list, event);
  }

  // Set the number of threads per block to the number of threads per warp
  // by default unless user has provided a better number
  size_t threadsPerBlock[3] = {32u, 1u, 1u};
  size_t maxWorkGroupSize = 0u;
  size_t maxThreadsPerBlock[3] = {};
  bool providedLocalWorkGroupSize = (local_work_size != nullptr);
  pi_uint32 local_size = kernel->get_local_size();
  pi_result retError = PI_SUCCESS;

  try {
    // Set the active context here as guessLocalWorkSize needs an active context
    ScopedContext active(command_queue->get_context());
    {
      size_t *reqdThreadsPerBlock = kernel->reqdThreadsPerBlock_;
      maxWorkGroupSize = command_queue->device_->get_max_work_group_size();
      command_queue->device_->get_max_work_item_sizes(
          sizeof(maxThreadsPerBlock), maxThreadsPerBlock);

      if (providedLocalWorkGroupSize) {
        auto isValid = [&](int dim) {
          if (reqdThreadsPerBlock[dim] != 0 &&
              local_work_size[dim] != reqdThreadsPerBlock[dim])
            return PI_ERROR_INVALID_WORK_GROUP_SIZE;

          if (local_work_size[dim] > maxThreadsPerBlock[dim])
            return PI_ERROR_INVALID_WORK_GROUP_SIZE;
          // Checks that local work sizes are a divisor of the global work sizes
          // which includes that the local work sizes are neither larger than
          // the global work sizes and not 0.
          if (0u == local_work_size[dim])
            return PI_ERROR_INVALID_WORK_GROUP_SIZE;
          if (0u != (global_work_size[dim] % local_work_size[dim]))
            return PI_ERROR_INVALID_WORK_GROUP_SIZE;
          threadsPerBlock[dim] = local_work_size[dim];
          return PI_SUCCESS;
        };

        for (size_t dim = 0; dim < work_dim; dim++) {
          auto err = isValid(dim);
          if (err != PI_SUCCESS)
            return err;
        }
      } else {
        guessLocalWorkSize(reinterpret_cast<pi_device>(command_queue->device_),
                           threadsPerBlock, global_work_size,
                           maxThreadsPerBlock, kernel, local_size);
      }
    }

    if (maxWorkGroupSize <
        size_t(threadsPerBlock[0] * threadsPerBlock[1] * threadsPerBlock[2])) {
      return PI_ERROR_INVALID_WORK_GROUP_SIZE;
    }

    size_t blocksPerGrid[3] = {1u, 1u, 1u};

    for (size_t i = 0; i < work_dim; i++) {
      blocksPerGrid[i] =
          (global_work_size[i] + threadsPerBlock[i] - 1) / threadsPerBlock[i];
    }

    std::unique_ptr<_pi_event> retImplEv{nullptr};

    pi_uint32 stream_token;
    _pi_stream_guard guard;
    CUstream cuStream = command_queue->get_next_compute_stream(
        num_events_in_wait_list,
        reinterpret_cast<const ur_event_handle_t *>(event_wait_list), guard,
        &stream_token);
    CUfunction cuFunc = kernel->get();

    retError = enqueueEventsWait(command_queue, cuStream,
                                 num_events_in_wait_list, event_wait_list);

    // Set the implicit global offset parameter if kernel has offset variant
    if (kernel->get_with_offset_parameter()) {
      std::uint32_t cuda_implicit_offset[3] = {0, 0, 0};
      if (global_work_offset) {
        for (size_t i = 0; i < work_dim; i++) {
          cuda_implicit_offset[i] =
              static_cast<std::uint32_t>(global_work_offset[i]);
          if (global_work_offset[i] != 0) {
            cuFunc = kernel->get_with_offset_parameter();
          }
        }
      }
      kernel->set_implicit_offset_arg(sizeof(cuda_implicit_offset),
                                      cuda_implicit_offset);
    }

    auto &argIndices = kernel->get_arg_indices();

    if (event) {
      retImplEv = std::unique_ptr<_pi_event>(
          _pi_event::make_native(PI_COMMAND_TYPE_NDRANGE_KERNEL, command_queue,
                                 cuStream, stream_token));
      retImplEv->start();
    }

    // Set local mem max size if env var is present
    static const char *local_mem_sz_ptr =
        std::getenv("SYCL_PI_CUDA_MAX_LOCAL_MEM_SIZE");

    if (local_mem_sz_ptr) {
      int device_max_local_mem = 0;
      cuDeviceGetAttribute(
          &device_max_local_mem,
          CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_BLOCK_OPTIN,
          command_queue->get_device()->get());

      static const int env_val = std::atoi(local_mem_sz_ptr);
      if (env_val <= 0 || env_val > device_max_local_mem) {
        setErrorMessage("Invalid value specified for "
                        "SYCL_PI_CUDA_MAX_LOCAL_MEM_SIZE",
                        UR_RESULT_ERROR_ADAPTER_SPECIFIC);
        return PI_ERROR_PLUGIN_SPECIFIC_ERROR;
      }
      PI_CHECK_ERROR(cuFuncSetAttribute(
          cuFunc, CU_FUNC_ATTRIBUTE_MAX_DYNAMIC_SHARED_SIZE_BYTES, env_val));
    }

    retError = PI_CHECK_ERROR(cuLaunchKernel(
        cuFunc, blocksPerGrid[0], blocksPerGrid[1], blocksPerGrid[2],
        threadsPerBlock[0], threadsPerBlock[1], threadsPerBlock[2], local_size,
        cuStream, const_cast<void **>(argIndices.data()), nullptr));
    if (local_size != 0)
      kernel->clear_local_size();

    if (event) {
      retError = map_ur_error(retImplEv->record());
      *event = retImplEv.release();
    }
  } catch (pi_result err) {
    retError = err;
  }
  return retError;
}

/// \TODO Not implemented
pi_result cuda_piEnqueueNativeKernel(pi_queue, void (*)(void *), void *, size_t,
                                     pi_uint32, const pi_mem *, const void **,
                                     pi_uint32, const pi_event *, pi_event *) {
  sycl::detail::pi::die("Not implemented in CUDA backend");
  return {};
}

/// \TODO Not implemented
pi_result cuda_piMemImageCreate(pi_context context, pi_mem_flags flags,
                                const pi_image_format *image_format,
                                const pi_image_desc *image_desc, void *host_ptr,
                                pi_mem *ret_mem) {
  // Need input memory object
  assert(ret_mem != nullptr);
  const bool performInitialCopy = (flags & PI_MEM_FLAGS_HOST_PTR_COPY) ||
                                  ((flags & PI_MEM_FLAGS_HOST_PTR_USE));
  pi_result retErr = PI_SUCCESS;

  // We only support RBGA channel order
  // TODO: check SYCL CTS and spec. May also have to support BGRA
  if (image_format->image_channel_order !=
      pi_image_channel_order::PI_IMAGE_CHANNEL_ORDER_RGBA) {
    sycl::detail::pi::die(
        "cuda_piMemImageCreate only supports RGBA channel order");
  }

  // We have to use cuArray3DCreate, which has some caveats. The height and
  // depth parameters must be set to 0 produce 1D or 2D arrays. image_desc gives
  // a minimum value of 1, so we need to convert the answer.
  CUDA_ARRAY3D_DESCRIPTOR array_desc;
  array_desc.NumChannels = 4; // Only support 4 channel image
  array_desc.Flags = 0;       // No flags required
  array_desc.Width = image_desc->image_width;
  if (image_desc->image_type == PI_MEM_TYPE_IMAGE1D) {
    array_desc.Height = 0;
    array_desc.Depth = 0;
  } else if (image_desc->image_type == PI_MEM_TYPE_IMAGE2D) {
    array_desc.Height = image_desc->image_height;
    array_desc.Depth = 0;
  } else if (image_desc->image_type == PI_MEM_TYPE_IMAGE3D) {
    array_desc.Height = image_desc->image_height;
    array_desc.Depth = image_desc->image_depth;
  }

  // We need to get this now in bytes for calculating the total image size later
  size_t pixel_type_size_bytes;

  switch (image_format->image_channel_data_type) {
  case PI_IMAGE_CHANNEL_TYPE_UNORM_INT8:
  case PI_IMAGE_CHANNEL_TYPE_UNSIGNED_INT8:
    array_desc.Format = CU_AD_FORMAT_UNSIGNED_INT8;
    pixel_type_size_bytes = 1;
    break;
  case PI_IMAGE_CHANNEL_TYPE_SIGNED_INT8:
    array_desc.Format = CU_AD_FORMAT_SIGNED_INT8;
    pixel_type_size_bytes = 1;
    break;
  case PI_IMAGE_CHANNEL_TYPE_UNORM_INT16:
  case PI_IMAGE_CHANNEL_TYPE_UNSIGNED_INT16:
    array_desc.Format = CU_AD_FORMAT_UNSIGNED_INT16;
    pixel_type_size_bytes = 2;
    break;
  case PI_IMAGE_CHANNEL_TYPE_SIGNED_INT16:
    array_desc.Format = CU_AD_FORMAT_SIGNED_INT16;
    pixel_type_size_bytes = 2;
    break;
  case PI_IMAGE_CHANNEL_TYPE_HALF_FLOAT:
    array_desc.Format = CU_AD_FORMAT_HALF;
    pixel_type_size_bytes = 2;
    break;
  case PI_IMAGE_CHANNEL_TYPE_UNSIGNED_INT32:
    array_desc.Format = CU_AD_FORMAT_UNSIGNED_INT32;
    pixel_type_size_bytes = 4;
    break;
  case PI_IMAGE_CHANNEL_TYPE_SIGNED_INT32:
    array_desc.Format = CU_AD_FORMAT_SIGNED_INT32;
    pixel_type_size_bytes = 4;
    break;
  case PI_IMAGE_CHANNEL_TYPE_FLOAT:
    array_desc.Format = CU_AD_FORMAT_FLOAT;
    pixel_type_size_bytes = 4;
    break;
  default:
    sycl::detail::pi::die(
        "cuda_piMemImageCreate given unsupported image_channel_data_type");
  }

  // When a dimension isn't used image_desc has the size set to 1
  size_t pixel_size_bytes =
      pixel_type_size_bytes * 4; // 4 is the only number of channels we support
  size_t image_size_bytes = pixel_size_bytes * image_desc->image_width *
                            image_desc->image_height * image_desc->image_depth;

  ScopedContext active(context);
  CUarray image_array;
  retErr = PI_CHECK_ERROR(cuArray3DCreate(&image_array, &array_desc));

  try {
    if (performInitialCopy) {
      // We have to use a different copy function for each image dimensionality
      if (image_desc->image_type == PI_MEM_TYPE_IMAGE1D) {
        retErr = PI_CHECK_ERROR(
            cuMemcpyHtoA(image_array, 0, host_ptr, image_size_bytes));
      } else if (image_desc->image_type == PI_MEM_TYPE_IMAGE2D) {
        CUDA_MEMCPY2D cpy_desc;
        memset(&cpy_desc, 0, sizeof(cpy_desc));
        cpy_desc.srcMemoryType = CUmemorytype_enum::CU_MEMORYTYPE_HOST;
        cpy_desc.srcHost = host_ptr;
        cpy_desc.dstMemoryType = CUmemorytype_enum::CU_MEMORYTYPE_ARRAY;
        cpy_desc.dstArray = image_array;
        cpy_desc.WidthInBytes = pixel_size_bytes * image_desc->image_width;
        cpy_desc.Height = image_desc->image_height;
        retErr = PI_CHECK_ERROR(cuMemcpy2D(&cpy_desc));
      } else if (image_desc->image_type == PI_MEM_TYPE_IMAGE3D) {
        CUDA_MEMCPY3D cpy_desc;
        memset(&cpy_desc, 0, sizeof(cpy_desc));
        cpy_desc.srcMemoryType = CUmemorytype_enum::CU_MEMORYTYPE_HOST;
        cpy_desc.srcHost = host_ptr;
        cpy_desc.dstMemoryType = CUmemorytype_enum::CU_MEMORYTYPE_ARRAY;
        cpy_desc.dstArray = image_array;
        cpy_desc.WidthInBytes = pixel_size_bytes * image_desc->image_width;
        cpy_desc.Height = image_desc->image_height;
        cpy_desc.Depth = image_desc->image_depth;
        retErr = PI_CHECK_ERROR(cuMemcpy3D(&cpy_desc));
      }
    }

    // CUDA_RESOURCE_DESC is a union of different structs, shown here
    // https://docs.nvidia.com/cuda/cuda-driver-api/group__CUDA__TEXOBJECT.html
    // We need to fill it as described here to use it for a surface or texture
    // https://docs.nvidia.com/cuda/cuda-driver-api/group__CUDA__SURFOBJECT.html
    // CUDA_RESOURCE_DESC::resType must be CU_RESOURCE_TYPE_ARRAY and
    // CUDA_RESOURCE_DESC::res::array::hArray must be set to a valid CUDA array
    // handle.
    // CUDA_RESOURCE_DESC::flags must be set to zero

    CUDA_RESOURCE_DESC image_res_desc;
    image_res_desc.res.array.hArray = image_array;
    image_res_desc.resType = CU_RESOURCE_TYPE_ARRAY;
    image_res_desc.flags = 0;

    CUsurfObject surface;
    retErr = PI_CHECK_ERROR(cuSurfObjectCreate(&surface, &image_res_desc));

    auto piMemObj = std::unique_ptr<_pi_mem>(new _pi_mem{
        context, image_array, surface, image_desc->image_type, host_ptr});

    if (piMemObj == nullptr) {
      return PI_ERROR_OUT_OF_HOST_MEMORY;
    }

    *ret_mem = piMemObj.release();
  } catch (pi_result err) {
    cuArrayDestroy(image_array);
    return err;
  } catch (...) {
    cuArrayDestroy(image_array);
    return PI_ERROR_UNKNOWN;
  }

  return retErr;
}

/// \TODO Not implemented
pi_result cuda_piMemImageGetInfo(pi_mem, pi_image_info, size_t, void *,
                                 size_t *) {
  sycl::detail::pi::die("cuda_piMemImageGetInfo not implemented");
  return {};
}

pi_result cuda_piMemRetain(pi_mem mem) {
  assert(mem != nullptr);
  assert(mem->get_reference_count() > 0);
  mem->increment_reference_count();
  return PI_SUCCESS;
}

/// Enqueues a wait on the given CUstream for all events.
/// See \ref enqueueEventWait
/// TODO: Add support for multiple streams once the Event class is properly
/// refactored.
///
pi_result cuda_piEnqueueEventsWait(pi_queue command_queue,
                                   pi_uint32 num_events_in_wait_list,
                                   const pi_event *event_wait_list,
                                   pi_event *event) {
  return cuda_piEnqueueEventsWaitWithBarrier(
      command_queue, num_events_in_wait_list, event_wait_list, event);
}

/// Enqueues a wait on the given CUstream for all specified events (See
/// \ref enqueueEventWaitWithBarrier.) If the events list is empty, the enqueued
/// wait will wait on all previous events in the queue.
///
/// \param[in] command_queue A valid PI queue.
/// \param[in] num_events_in_wait_list Number of events in event_wait_list.
/// \param[in] event_wait_list Events to wait on.
/// \param[out] event Event for when all events in event_wait_list have finished
/// or, if event_wait_list is empty, when all previous events in the queue have
/// finished.
///
/// \return TBD
pi_result cuda_piEnqueueEventsWaitWithBarrier(pi_queue command_queue,
                                              pi_uint32 num_events_in_wait_list,
                                              const pi_event *event_wait_list,
                                              pi_event *event) {
  // This function makes one stream work on the previous work (or work
  // represented by input events) and then all future work waits on that stream.
  if (!command_queue) {
    return PI_ERROR_INVALID_QUEUE;
  }

  pi_result result;

  try {
    ScopedContext active(command_queue->get_context());
    pi_uint32 stream_token;
    _pi_stream_guard guard;
    CUstream cuStream = command_queue->get_next_compute_stream(
        num_events_in_wait_list,
        reinterpret_cast<const ur_event_handle_t *>(event_wait_list), guard,
        &stream_token);
    {
      std::lock_guard<std::mutex> guard(command_queue->barrier_mutex_);
      if (command_queue->barrier_event_ == nullptr) {
        PI_CHECK_ERROR(cuEventCreate(&command_queue->barrier_event_,
                                     CU_EVENT_DISABLE_TIMING));
      }
      if (num_events_in_wait_list == 0) { //  wait on all work
        if (command_queue->barrier_tmp_event_ == nullptr) {
          PI_CHECK_ERROR(cuEventCreate(&command_queue->barrier_tmp_event_,
                                       CU_EVENT_DISABLE_TIMING));
        }
        command_queue->sync_streams(
            [cuStream,
             tmp_event = command_queue->barrier_tmp_event_](CUstream s) {
              if (cuStream != s) {
                // record a new CUDA event on every stream and make one stream
                // wait for these events
                PI_CHECK_ERROR(cuEventRecord(tmp_event, s));
                PI_CHECK_ERROR(cuStreamWaitEvent(cuStream, tmp_event, 0));
              }
            });
      } else { // wait just on given events
        forLatestEvents(event_wait_list, num_events_in_wait_list,
                        [cuStream](pi_event event) -> pi_result {
                          if (event->get_queue()->has_been_synchronized(
                                  event->get_compute_stream_token())) {
                            return PI_SUCCESS;
                          } else {
                            return PI_CHECK_ERROR(
                                cuStreamWaitEvent(cuStream, event->get(), 0));
                          }
                        });
      }

      result = PI_CHECK_ERROR(
          cuEventRecord(command_queue->barrier_event_, cuStream));
      for (unsigned int i = 0;
           i < command_queue->compute_applied_barrier_.size(); i++) {
        command_queue->compute_applied_barrier_[i] = false;
      }
      for (unsigned int i = 0;
           i < command_queue->transfer_applied_barrier_.size(); i++) {
        command_queue->transfer_applied_barrier_[i] = false;
      }
    }
    if (result != PI_SUCCESS) {
      return result;
    }

    if (event) {
      *event = _pi_event::make_native(PI_COMMAND_TYPE_MARKER, command_queue,
                                      cuStream, stream_token);
      (*event)->start();
      (*event)->record();
    }

    return PI_SUCCESS;
  } catch (pi_result err) {
    return err;
  } catch (...) {
    return PI_ERROR_UNKNOWN;
  }
}

/// Creates a PI sampler object
///
/// \param[in] context The context the sampler is created for.
/// \param[in] sampler_properties The properties for the sampler.
/// \param[out] result_sampler Set to the resulting sampler object.
///
/// \return PI_SUCCESS on success. PI_ERROR_INVALID_VALUE if given an invalid
/// property
///         or if there is multiple of properties from the same category.
pi_result cuda_piSamplerCreate(pi_context context,
                               const pi_sampler_properties *sampler_properties,
                               pi_sampler *result_sampler) {
  std::unique_ptr<_pi_sampler> retImplSampl{new _pi_sampler(context)};

  bool propSeen[3] = {false, false, false};
  for (size_t i = 0; sampler_properties[i] != 0; i += 2) {
    switch (sampler_properties[i]) {
    case PI_SAMPLER_PROPERTIES_NORMALIZED_COORDS:
      if (propSeen[0]) {
        return PI_ERROR_INVALID_VALUE;
      }
      propSeen[0] = true;
      retImplSampl->props_ |= sampler_properties[i + 1];
      break;
    case PI_SAMPLER_PROPERTIES_FILTER_MODE:
      if (propSeen[1]) {
        return PI_ERROR_INVALID_VALUE;
      }
      propSeen[1] = true;
      retImplSampl->props_ |=
          (sampler_properties[i + 1] - PI_SAMPLER_FILTER_MODE_NEAREST) << 1;
      break;
    case PI_SAMPLER_PROPERTIES_ADDRESSING_MODE:
      if (propSeen[2]) {
        return PI_ERROR_INVALID_VALUE;
      }
      propSeen[2] = true;
      retImplSampl->props_ |=
          (sampler_properties[i + 1] - PI_SAMPLER_ADDRESSING_MODE_NONE) << 2;
      break;
    default:
      return PI_ERROR_INVALID_VALUE;
    }
  }

  if (!propSeen[0]) {
    retImplSampl->props_ |= PI_TRUE;
  }
  // Default filter mode to PI_SAMPLER_FILTER_MODE_NEAREST
  if (!propSeen[2]) {
    retImplSampl->props_ |=
        (PI_SAMPLER_ADDRESSING_MODE_CLAMP % PI_SAMPLER_ADDRESSING_MODE_NONE)
        << 2;
  }

  *result_sampler = retImplSampl.release();
  return PI_SUCCESS;
}

/// Gets information from a PI sampler object
///
/// \param[in] sampler The sampler to get the information from.
/// \param[in] param_name The name of the information to get.
/// \param[in] param_value_size The size of the param_value.
/// \param[out] param_value Set to information value.
/// \param[out] param_value_size_ret Set to the size of the information value.
///
/// \return PI_SUCCESS on success.
pi_result cuda_piSamplerGetInfo(pi_sampler sampler, pi_sampler_info param_name,
                                size_t param_value_size, void *param_value,
                                size_t *param_value_size_ret) {
  assert(sampler != nullptr);

  switch (param_name) {
  case PI_SAMPLER_INFO_REFERENCE_COUNT:
    return getInfo(param_value_size, param_value, param_value_size_ret,
                   sampler->get_reference_count());
  case PI_SAMPLER_INFO_CONTEXT:
    return getInfo(param_value_size, param_value, param_value_size_ret,
                   sampler->context_);
  case PI_SAMPLER_INFO_NORMALIZED_COORDS: {
    pi_bool norm_coords_prop = static_cast<pi_bool>(sampler->props_ & 0x1);
    return getInfo(param_value_size, param_value, param_value_size_ret,
                   norm_coords_prop);
  }
  case PI_SAMPLER_INFO_FILTER_MODE: {
    pi_sampler_filter_mode filter_prop = static_cast<pi_sampler_filter_mode>(
        ((sampler->props_ >> 1) & 0x1) + PI_SAMPLER_FILTER_MODE_NEAREST);
    return getInfo(param_value_size, param_value, param_value_size_ret,
                   filter_prop);
  }
  case PI_SAMPLER_INFO_ADDRESSING_MODE: {
    pi_sampler_addressing_mode addressing_prop =
        static_cast<pi_sampler_addressing_mode>(
            (sampler->props_ >> 2) + PI_SAMPLER_ADDRESSING_MODE_NONE);
    return getInfo(param_value_size, param_value, param_value_size_ret,
                   addressing_prop);
  }
  default:
    __SYCL_PI_HANDLE_UNKNOWN_PARAM_NAME(param_name);
  }
  return {};
}

/// Retains a PI sampler object, incrementing its reference count.
///
/// \param[in] sampler The sampler to increment the reference count of.
///
/// \return PI_SUCCESS.
pi_result cuda_piSamplerRetain(pi_sampler sampler) {
  assert(sampler != nullptr);
  sampler->increment_reference_count();
  return PI_SUCCESS;
}

/// Releases a PI sampler object, decrementing its reference count. If the
/// reference count reaches zero, the sampler object is destroyed.
///
/// \param[in] sampler The sampler to decrement the reference count of.
///
/// \return PI_SUCCESS.
pi_result cuda_piSamplerRelease(pi_sampler sampler) {
  assert(sampler != nullptr);

  // double delete or someone is messing with the ref count.
  // either way, cannot safely proceed.
  sycl::detail::pi::assertion(
      sampler->get_reference_count() != 0,
      "Reference count overflow detected in cuda_piSamplerRelease.");

  // decrement ref count. If it is 0, delete the sampler.
  if (sampler->decrement_reference_count() == 0) {
    delete sampler;
  }

  return PI_SUCCESS;
}

/// General 3D memory copy operation.
/// This function requires the corresponding CUDA context to be at the top of
/// the context stack
/// If the source and/or destination is on the device, src_ptr and/or dst_ptr
/// must be a pointer to a CUdeviceptr
static pi_result commonEnqueueMemBufferCopyRect(
    CUstream cu_stream, pi_buff_rect_region region, const void *src_ptr,
    const CUmemorytype_enum src_type, pi_buff_rect_offset src_offset,
    size_t src_row_pitch, size_t src_slice_pitch, void *dst_ptr,
    const CUmemorytype_enum dst_type, pi_buff_rect_offset dst_offset,
    size_t dst_row_pitch, size_t dst_slice_pitch) {

  assert(region != nullptr);
  assert(src_offset != nullptr);
  assert(dst_offset != nullptr);

  assert(src_type == CU_MEMORYTYPE_DEVICE || src_type == CU_MEMORYTYPE_HOST);
  assert(dst_type == CU_MEMORYTYPE_DEVICE || dst_type == CU_MEMORYTYPE_HOST);

  src_row_pitch = (!src_row_pitch) ? region->width_bytes + src_offset->x_bytes
                                   : src_row_pitch;
  src_slice_pitch =
      (!src_slice_pitch)
          ? ((region->height_scalar + src_offset->y_scalar) * src_row_pitch)
          : src_slice_pitch;
  dst_row_pitch = (!dst_row_pitch) ? region->width_bytes + dst_offset->x_bytes
                                   : dst_row_pitch;
  dst_slice_pitch =
      (!dst_slice_pitch)
          ? ((region->height_scalar + dst_offset->y_scalar) * dst_row_pitch)
          : dst_slice_pitch;

  CUDA_MEMCPY3D params = {};

  params.WidthInBytes = region->width_bytes;
  params.Height = region->height_scalar;
  params.Depth = region->depth_scalar;

  params.srcMemoryType = src_type;
  params.srcDevice = src_type == CU_MEMORYTYPE_DEVICE
                         ? *static_cast<const CUdeviceptr *>(src_ptr)
                         : 0;
  params.srcHost = src_type == CU_MEMORYTYPE_HOST ? src_ptr : nullptr;
  params.srcXInBytes = src_offset->x_bytes;
  params.srcY = src_offset->y_scalar;
  params.srcZ = src_offset->z_scalar;
  params.srcPitch = src_row_pitch;
  params.srcHeight = src_slice_pitch / src_row_pitch;

  params.dstMemoryType = dst_type;
  params.dstDevice = dst_type == CU_MEMORYTYPE_DEVICE
                         ? *static_cast<CUdeviceptr *>(dst_ptr)
                         : 0;
  params.dstHost = dst_type == CU_MEMORYTYPE_HOST ? dst_ptr : nullptr;
  params.dstXInBytes = dst_offset->x_bytes;
  params.dstY = dst_offset->y_scalar;
  params.dstZ = dst_offset->z_scalar;
  params.dstPitch = dst_row_pitch;
  params.dstHeight = dst_slice_pitch / dst_row_pitch;

  return PI_CHECK_ERROR(cuMemcpy3DAsync(&params, cu_stream));
}

pi_result cuda_piEnqueueMemBufferReadRect(
    pi_queue command_queue, pi_mem buffer, pi_bool blocking_read,
    pi_buff_rect_offset buffer_offset, pi_buff_rect_offset host_offset,
    pi_buff_rect_region region, size_t buffer_row_pitch,
    size_t buffer_slice_pitch, size_t host_row_pitch, size_t host_slice_pitch,
    void *ptr, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {

  assert(buffer != nullptr);
  assert(command_queue != nullptr);

  pi_result retErr = PI_SUCCESS;
  CUdeviceptr devPtr = buffer->mem_.buffer_mem_.get();
  std::unique_ptr<_pi_event> retImplEv{nullptr};

  try {
    ScopedContext active(command_queue->get_context());
    CUstream cuStream = command_queue->get_next_transfer_stream();

    retErr = enqueueEventsWait(command_queue, cuStream, num_events_in_wait_list,
                               event_wait_list);

    if (event) {
      retImplEv = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_READ_RECT, command_queue, cuStream));
      retImplEv->start();
    }

    retErr = commonEnqueueMemBufferCopyRect(
        cuStream, region, &devPtr, CU_MEMORYTYPE_DEVICE, buffer_offset,
        buffer_row_pitch, buffer_slice_pitch, ptr, CU_MEMORYTYPE_HOST,
        host_offset, host_row_pitch, host_slice_pitch);

    if (event) {
      retErr = map_ur_error(retImplEv->record());
    }

    if (blocking_read) {
      retErr = PI_CHECK_ERROR(cuStreamSynchronize(cuStream));
    }

    if (event) {
      *event = retImplEv.release();
    }

  } catch (pi_result err) {
    retErr = err;
  }
  return retErr;
}

pi_result cuda_piEnqueueMemBufferWriteRect(
    pi_queue command_queue, pi_mem buffer, pi_bool blocking_write,
    pi_buff_rect_offset buffer_offset, pi_buff_rect_offset host_offset,
    pi_buff_rect_region region, size_t buffer_row_pitch,
    size_t buffer_slice_pitch, size_t host_row_pitch, size_t host_slice_pitch,
    const void *ptr, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {

  assert(buffer != nullptr);
  assert(command_queue != nullptr);

  pi_result retErr = PI_SUCCESS;
  CUdeviceptr devPtr = buffer->mem_.buffer_mem_.get();
  std::unique_ptr<_pi_event> retImplEv{nullptr};

  try {
    ScopedContext active(command_queue->get_context());
    CUstream cuStream = command_queue->get_next_transfer_stream();
    retErr = enqueueEventsWait(command_queue, cuStream, num_events_in_wait_list,
                               event_wait_list);

    if (event) {
      retImplEv = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_WRITE_RECT, command_queue, cuStream));
      retImplEv->start();
    }

    retErr = commonEnqueueMemBufferCopyRect(
        cuStream, region, ptr, CU_MEMORYTYPE_HOST, host_offset, host_row_pitch,
        host_slice_pitch, &devPtr, CU_MEMORYTYPE_DEVICE, buffer_offset,
        buffer_row_pitch, buffer_slice_pitch);

    if (event) {
      retErr = map_ur_error(retImplEv->record());
    }

    if (blocking_write) {
      retErr = PI_CHECK_ERROR(cuStreamSynchronize(cuStream));
    }

    if (event) {
      *event = retImplEv.release();
    }

  } catch (pi_result err) {
    retErr = err;
  }
  return retErr;
}

pi_result cuda_piEnqueueMemBufferCopy(pi_queue command_queue, pi_mem src_buffer,
                                      pi_mem dst_buffer, size_t src_offset,
                                      size_t dst_offset, size_t size,
                                      pi_uint32 num_events_in_wait_list,
                                      const pi_event *event_wait_list,
                                      pi_event *event) {
  if (!command_queue) {
    return PI_ERROR_INVALID_QUEUE;
  }

  std::unique_ptr<_pi_event> retImplEv{nullptr};

  try {
    ScopedContext active(command_queue->get_context());
    pi_result result;

    auto stream = command_queue->get_next_transfer_stream();
    result = enqueueEventsWait(command_queue, stream, num_events_in_wait_list,
                               event_wait_list);

    if (event) {
      retImplEv = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_COPY, command_queue, stream));
      result = map_ur_error(retImplEv->start());
    }

    auto src = src_buffer->mem_.buffer_mem_.get() + src_offset;
    auto dst = dst_buffer->mem_.buffer_mem_.get() + dst_offset;

    result = PI_CHECK_ERROR(cuMemcpyDtoDAsync(dst, src, size, stream));

    if (event) {
      result = map_ur_error(retImplEv->record());
      *event = retImplEv.release();
    }

    return result;
  } catch (pi_result err) {
    return err;
  } catch (...) {
    return PI_ERROR_UNKNOWN;
  }
}

pi_result cuda_piEnqueueMemBufferCopyRect(
    pi_queue command_queue, pi_mem src_buffer, pi_mem dst_buffer,
    pi_buff_rect_offset src_origin, pi_buff_rect_offset dst_origin,
    pi_buff_rect_region region, size_t src_row_pitch, size_t src_slice_pitch,
    size_t dst_row_pitch, size_t dst_slice_pitch,
    pi_uint32 num_events_in_wait_list, const pi_event *event_wait_list,
    pi_event *event) {

  assert(src_buffer != nullptr);
  assert(dst_buffer != nullptr);
  assert(command_queue != nullptr);

  pi_result retErr = PI_SUCCESS;
  CUdeviceptr srcPtr = src_buffer->mem_.buffer_mem_.get();
  CUdeviceptr dstPtr = dst_buffer->mem_.buffer_mem_.get();
  std::unique_ptr<_pi_event> retImplEv{nullptr};

  try {
    ScopedContext active(command_queue->get_context());
    CUstream cuStream = command_queue->get_next_transfer_stream();
    retErr = enqueueEventsWait(command_queue, cuStream, num_events_in_wait_list,
                               event_wait_list);

    if (event) {
      retImplEv = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_COPY_RECT, command_queue, cuStream));
      retImplEv->start();
    }

    retErr = commonEnqueueMemBufferCopyRect(
        cuStream, region, &srcPtr, CU_MEMORYTYPE_DEVICE, src_origin,
        src_row_pitch, src_slice_pitch, &dstPtr, CU_MEMORYTYPE_DEVICE,
        dst_origin, dst_row_pitch, dst_slice_pitch);

    if (event) {
      retImplEv->record();
      *event = retImplEv.release();
    }

  } catch (pi_result err) {
    retErr = err;
  }
  return retErr;
}

pi_result cuda_piEnqueueMemBufferFill(pi_queue command_queue, pi_mem buffer,
                                      const void *pattern, size_t pattern_size,
                                      size_t offset, size_t size,
                                      pi_uint32 num_events_in_wait_list,
                                      const pi_event *event_wait_list,
                                      pi_event *event) {
  assert(command_queue != nullptr);

  auto args_are_multiples_of_pattern_size =
      (offset % pattern_size == 0) || (size % pattern_size == 0);

  auto pattern_is_valid = (pattern != nullptr);

  auto pattern_size_is_valid =
      ((pattern_size & (pattern_size - 1)) == 0) && // is power of two
      (pattern_size > 0) && (pattern_size <= 128);  // falls within valid range

  assert(args_are_multiples_of_pattern_size && pattern_is_valid &&
         pattern_size_is_valid);
  (void)args_are_multiples_of_pattern_size;
  (void)pattern_is_valid;
  (void)pattern_size_is_valid;

  std::unique_ptr<_pi_event> retImplEv{nullptr};

  try {
    ScopedContext active(command_queue->get_context());

    auto stream = command_queue->get_next_transfer_stream();
    pi_result result;
    result = enqueueEventsWait(command_queue, stream, num_events_in_wait_list,
                               event_wait_list);

    if (event) {
      retImplEv = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_FILL, command_queue, stream));
      result = map_ur_error(retImplEv->start());
    }

    auto dstDevice = buffer->mem_.buffer_mem_.get() + offset;
    auto N = size / pattern_size;

    // pattern size in bytes
    switch (pattern_size) {
    case 1: {
      auto value = *static_cast<const uint8_t *>(pattern);
      result = PI_CHECK_ERROR(cuMemsetD8Async(dstDevice, value, N, stream));
      break;
    }
    case 2: {
      auto value = *static_cast<const uint16_t *>(pattern);
      result = PI_CHECK_ERROR(cuMemsetD16Async(dstDevice, value, N, stream));
      break;
    }
    case 4: {
      auto value = *static_cast<const uint32_t *>(pattern);
      result = PI_CHECK_ERROR(cuMemsetD32Async(dstDevice, value, N, stream));
      break;
    }
    default: {
      // CUDA has no memset functions that allow setting values more than 4
      // bytes. PI API lets you pass an arbitrary "pattern" to the buffer
      // fill, which can be more than 4 bytes. We must break up the pattern
      // into 4 byte values, and set the buffer using multiple strided calls.
      // This means that one cuMemsetD2D32Async call is made for every 4 bytes
      // in the pattern.

      auto number_of_steps = pattern_size / sizeof(uint32_t);

      // we walk up the pattern in 4-byte steps, and call cuMemset for each
      // 4-byte chunk of the pattern.
      for (auto step = 0u; step < number_of_steps; ++step) {
        // take 4 bytes of the pattern
        auto value = *(static_cast<const uint32_t *>(pattern) + step);

        // offset the pointer to the part of the buffer we want to write to
        auto offset_ptr = dstDevice + (step * sizeof(uint32_t));

        // set all of the pattern chunks
        result = PI_CHECK_ERROR(
            cuMemsetD2D32Async(offset_ptr, pattern_size, value, 1, N, stream));
      }

      break;
    }
    }

    if (event) {
      result = map_ur_error(retImplEv->record());
      *event = retImplEv.release();
    }

    return result;
  } catch (pi_result err) {
    return err;
  } catch (...) {
    return PI_ERROR_UNKNOWN;
  }
}

static size_t imageElementByteSize(CUDA_ARRAY_DESCRIPTOR array_desc) {
  switch (array_desc.Format) {
  case CU_AD_FORMAT_UNSIGNED_INT8:
  case CU_AD_FORMAT_SIGNED_INT8:
    return 1;
  case CU_AD_FORMAT_UNSIGNED_INT16:
  case CU_AD_FORMAT_SIGNED_INT16:
  case CU_AD_FORMAT_HALF:
    return 2;
  case CU_AD_FORMAT_UNSIGNED_INT32:
  case CU_AD_FORMAT_SIGNED_INT32:
  case CU_AD_FORMAT_FLOAT:
    return 4;
  default:
    sycl::detail::pi::die("Invalid image format.");
    return 0;
  }
}

/// General ND memory copy operation for images (where N > 1).
/// This function requires the corresponding CUDA context to be at the top of
/// the context stack
/// If the source and/or destination is an array, src_ptr and/or dst_ptr
/// must be a pointer to a CUarray
static pi_result commonEnqueueMemImageNDCopy(
    CUstream cu_stream, pi_mem_type img_type, const size_t *region,
    const void *src_ptr, const CUmemorytype_enum src_type,
    const size_t *src_offset, void *dst_ptr, const CUmemorytype_enum dst_type,
    const size_t *dst_offset) {
  assert(region != nullptr);

  assert(src_type == CU_MEMORYTYPE_ARRAY || src_type == CU_MEMORYTYPE_HOST);
  assert(dst_type == CU_MEMORYTYPE_ARRAY || dst_type == CU_MEMORYTYPE_HOST);

  if (img_type == PI_MEM_TYPE_IMAGE2D) {
    CUDA_MEMCPY2D cpyDesc;
    memset(&cpyDesc, 0, sizeof(cpyDesc));
    cpyDesc.srcMemoryType = src_type;
    if (src_type == CU_MEMORYTYPE_ARRAY) {
      cpyDesc.srcArray = *static_cast<const CUarray *>(src_ptr);
      cpyDesc.srcXInBytes = src_offset[0];
      cpyDesc.srcY = src_offset[1];
    } else {
      cpyDesc.srcHost = src_ptr;
    }
    cpyDesc.dstMemoryType = dst_type;
    if (dst_type == CU_MEMORYTYPE_ARRAY) {
      cpyDesc.dstArray = *static_cast<CUarray *>(dst_ptr);
      cpyDesc.dstXInBytes = dst_offset[0];
      cpyDesc.dstY = dst_offset[1];
    } else {
      cpyDesc.dstHost = dst_ptr;
    }
    cpyDesc.WidthInBytes = region[0];
    cpyDesc.Height = region[1];
    return PI_CHECK_ERROR(cuMemcpy2DAsync(&cpyDesc, cu_stream));
  }
  if (img_type == PI_MEM_TYPE_IMAGE3D) {
    CUDA_MEMCPY3D cpyDesc;
    memset(&cpyDesc, 0, sizeof(cpyDesc));
    cpyDesc.srcMemoryType = src_type;
    if (src_type == CU_MEMORYTYPE_ARRAY) {
      cpyDesc.srcArray = *static_cast<const CUarray *>(src_ptr);
      cpyDesc.srcXInBytes = src_offset[0];
      cpyDesc.srcY = src_offset[1];
      cpyDesc.srcZ = src_offset[2];
    } else {
      cpyDesc.srcHost = src_ptr;
    }
    cpyDesc.dstMemoryType = dst_type;
    if (dst_type == CU_MEMORYTYPE_ARRAY) {
      cpyDesc.dstArray = *static_cast<CUarray *>(dst_ptr);
      cpyDesc.dstXInBytes = dst_offset[0];
      cpyDesc.dstY = dst_offset[1];
      cpyDesc.dstZ = dst_offset[2];
    } else {
      cpyDesc.dstHost = dst_ptr;
    }
    cpyDesc.WidthInBytes = region[0];
    cpyDesc.Height = region[1];
    cpyDesc.Depth = region[2];
    return PI_CHECK_ERROR(cuMemcpy3DAsync(&cpyDesc, cu_stream));
  }
  return PI_ERROR_INVALID_VALUE;
}

pi_result cuda_piEnqueueMemImageRead(
    pi_queue command_queue, pi_mem image, pi_bool blocking_read,
    const size_t *origin, const size_t *region, size_t row_pitch,
    size_t slice_pitch, void *ptr, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {
  // Ignore unused parameters
  (void)row_pitch;
  (void)slice_pitch;

  assert(command_queue != nullptr);
  assert(image != nullptr);
  assert(image->mem_type_ == _pi_mem::mem_type::surface);

  pi_result retErr = PI_SUCCESS;

  try {
    ScopedContext active(command_queue->get_context());
    CUstream cuStream = command_queue->get_next_transfer_stream();
    retErr = enqueueEventsWait(command_queue, cuStream, num_events_in_wait_list,
                               event_wait_list);

    CUarray array = image->mem_.surface_mem_.get_array();

    CUDA_ARRAY_DESCRIPTOR arrayDesc;
    retErr = PI_CHECK_ERROR(cuArrayGetDescriptor(&arrayDesc, array));

    int elementByteSize = imageElementByteSize(arrayDesc);

    size_t byteOffsetX = origin[0] * elementByteSize * arrayDesc.NumChannels;
    size_t bytesToCopy = elementByteSize * arrayDesc.NumChannels * region[0];

    pi_mem_type imgType = image->mem_.surface_mem_.get_image_type();
    if (imgType == PI_MEM_TYPE_IMAGE1D) {
      retErr = PI_CHECK_ERROR(
          cuMemcpyAtoHAsync(ptr, array, byteOffsetX, bytesToCopy, cuStream));
    } else {
      size_t adjustedRegion[3] = {bytesToCopy, region[1], region[2]};
      size_t srcOffset[3] = {byteOffsetX, origin[1], origin[2]};

      retErr = commonEnqueueMemImageNDCopy(
          cuStream, imgType, adjustedRegion, &array, CU_MEMORYTYPE_ARRAY,
          srcOffset, ptr, CU_MEMORYTYPE_HOST, nullptr);

      if (retErr != PI_SUCCESS) {
        return retErr;
      }
    }

    if (event) {
      auto new_event = _pi_event::make_native(PI_COMMAND_TYPE_IMAGE_READ,
                                              command_queue, cuStream);
      new_event->record();
      *event = new_event;
    }

    if (blocking_read) {
      retErr = PI_CHECK_ERROR(cuStreamSynchronize(cuStream));
    }
  } catch (pi_result err) {
    return err;
  } catch (...) {
    return PI_ERROR_UNKNOWN;
  }

  return retErr;
}

pi_result
cuda_piEnqueueMemImageWrite(pi_queue command_queue, pi_mem image,
                            pi_bool blocking_write, const size_t *origin,
                            const size_t *region, size_t input_row_pitch,
                            size_t input_slice_pitch, const void *ptr,
                            pi_uint32 num_events_in_wait_list,
                            const pi_event *event_wait_list, pi_event *event) {
  // Ignore unused parameters
  (void)blocking_write;
  (void)input_row_pitch;
  (void)input_slice_pitch;

  assert(command_queue != nullptr);
  assert(image != nullptr);
  assert(image->mem_type_ == _pi_mem::mem_type::surface);

  pi_result retErr = PI_SUCCESS;

  try {
    ScopedContext active(command_queue->get_context());
    CUstream cuStream = command_queue->get_next_transfer_stream();
    retErr = enqueueEventsWait(command_queue, cuStream, num_events_in_wait_list,
                               event_wait_list);

    CUarray array = image->mem_.surface_mem_.get_array();

    CUDA_ARRAY_DESCRIPTOR arrayDesc;
    retErr = PI_CHECK_ERROR(cuArrayGetDescriptor(&arrayDesc, array));

    int elementByteSize = imageElementByteSize(arrayDesc);

    size_t byteOffsetX = origin[0] * elementByteSize * arrayDesc.NumChannels;
    size_t bytesToCopy = elementByteSize * arrayDesc.NumChannels * region[0];

    pi_mem_type imgType = image->mem_.surface_mem_.get_image_type();
    if (imgType == PI_MEM_TYPE_IMAGE1D) {
      retErr = PI_CHECK_ERROR(
          cuMemcpyHtoAAsync(array, byteOffsetX, ptr, bytesToCopy, cuStream));
    } else {
      size_t adjustedRegion[3] = {bytesToCopy, region[1], region[2]};
      size_t dstOffset[3] = {byteOffsetX, origin[1], origin[2]};

      retErr = commonEnqueueMemImageNDCopy(
          cuStream, imgType, adjustedRegion, ptr, CU_MEMORYTYPE_HOST, nullptr,
          &array, CU_MEMORYTYPE_ARRAY, dstOffset);

      if (retErr != PI_SUCCESS) {
        return retErr;
      }
    }

    if (event) {
      auto new_event = _pi_event::make_native(PI_COMMAND_TYPE_IMAGE_WRITE,
                                              command_queue, cuStream);
      new_event->record();
      *event = new_event;
    }
  } catch (pi_result err) {
    return err;
  } catch (...) {
    return PI_ERROR_UNKNOWN;
  }

  return retErr;
}

pi_result cuda_piEnqueueMemImageCopy(pi_queue command_queue, pi_mem src_image,
                                     pi_mem dst_image, const size_t *src_origin,
                                     const size_t *dst_origin,
                                     const size_t *region,
                                     pi_uint32 num_events_in_wait_list,
                                     const pi_event *event_wait_list,
                                     pi_event *event) {
  assert(src_image->mem_type_ == _pi_mem::mem_type::surface);
  assert(dst_image->mem_type_ == _pi_mem::mem_type::surface);
  assert(src_image->mem_.surface_mem_.get_image_type() ==
         dst_image->mem_.surface_mem_.get_image_type());

  pi_result retErr = PI_SUCCESS;

  try {
    ScopedContext active(command_queue->get_context());
    CUstream cuStream = command_queue->get_next_transfer_stream();
    retErr = enqueueEventsWait(command_queue, cuStream, num_events_in_wait_list,
                               event_wait_list);

    CUarray srcArray = src_image->mem_.surface_mem_.get_array();
    CUarray dstArray = dst_image->mem_.surface_mem_.get_array();

    CUDA_ARRAY_DESCRIPTOR srcArrayDesc;
    retErr = PI_CHECK_ERROR(cuArrayGetDescriptor(&srcArrayDesc, srcArray));
    CUDA_ARRAY_DESCRIPTOR dstArrayDesc;
    retErr = PI_CHECK_ERROR(cuArrayGetDescriptor(&dstArrayDesc, dstArray));

    assert(srcArrayDesc.Format == dstArrayDesc.Format);
    assert(srcArrayDesc.NumChannels == dstArrayDesc.NumChannels);

    int elementByteSize = imageElementByteSize(srcArrayDesc);

    size_t dstByteOffsetX =
        dst_origin[0] * elementByteSize * srcArrayDesc.NumChannels;
    size_t srcByteOffsetX =
        src_origin[0] * elementByteSize * dstArrayDesc.NumChannels;
    size_t bytesToCopy = elementByteSize * srcArrayDesc.NumChannels * region[0];

    pi_mem_type imgType = src_image->mem_.surface_mem_.get_image_type();
    if (imgType == PI_MEM_TYPE_IMAGE1D) {
      retErr = PI_CHECK_ERROR(cuMemcpyAtoA(dstArray, dstByteOffsetX, srcArray,
                                           srcByteOffsetX, bytesToCopy));
    } else {
      size_t adjustedRegion[3] = {bytesToCopy, region[1], region[2]};
      size_t srcOffset[3] = {srcByteOffsetX, src_origin[1], src_origin[2]};
      size_t dstOffset[3] = {dstByteOffsetX, dst_origin[1], dst_origin[2]};

      retErr = commonEnqueueMemImageNDCopy(
          cuStream, imgType, adjustedRegion, &srcArray, CU_MEMORYTYPE_ARRAY,
          srcOffset, &dstArray, CU_MEMORYTYPE_ARRAY, dstOffset);

      if (retErr != PI_SUCCESS) {
        return retErr;
      }
    }

    if (event) {
      auto new_event = _pi_event::make_native(PI_COMMAND_TYPE_IMAGE_COPY,
                                              command_queue, cuStream);
      new_event->record();
      *event = new_event;
    }
  } catch (pi_result err) {
    return err;
  } catch (...) {
    return PI_ERROR_UNKNOWN;
  }

  return retErr;
}

/// \TODO Not implemented in CUDA.
pi_result cuda_piEnqueueMemImageFill(pi_queue, pi_mem, const void *,
                                     const size_t *, const size_t *, pi_uint32,
                                     const pi_event *, pi_event *) {
  sycl::detail::pi::die("cuda_piEnqueueMemImageFill not implemented");
  return {};
}

/// Implements mapping on the host using a BufferRead operation.
/// Mapped pointers are stored in the pi_mem object.
/// If the buffer uses pinned host memory a pointer to that memory is returned
/// and no read operation is done.
///
pi_result cuda_piEnqueueMemBufferMap(pi_queue command_queue, pi_mem buffer,
                                     pi_bool blocking_map,
                                     pi_map_flags map_flags, size_t offset,
                                     size_t size,
                                     pi_uint32 num_events_in_wait_list,
                                     const pi_event *event_wait_list,
                                     pi_event *event, void **ret_map) {
  assert(ret_map != nullptr);
  assert(command_queue != nullptr);
  assert(buffer != nullptr);
  assert(buffer->mem_type_ == _pi_mem::mem_type::buffer);

  pi_result ret_err = PI_ERROR_INVALID_OPERATION;
  const bool is_pinned = buffer->mem_.buffer_mem_.allocMode_ ==
                         _pi_mem::mem_::buffer_mem_::alloc_mode::alloc_host_ptr;

  // Currently no support for overlapping regions
  if (buffer->mem_.buffer_mem_.get_map_ptr() != nullptr) {
    return ret_err;
  }

  // Allocate a pointer in the host to store the mapped information
  auto hostPtr = buffer->mem_.buffer_mem_.map_to_ptr(offset, map_flags);
  *ret_map = buffer->mem_.buffer_mem_.get_map_ptr();
  if (hostPtr) {
    ret_err = PI_SUCCESS;
  }

  if (!is_pinned && ((map_flags & PI_MAP_READ) || (map_flags & PI_MAP_WRITE))) {
    // Pinned host memory is already on host so it doesn't need to be read.
    ret_err = cuda_piEnqueueMemBufferRead(
        command_queue, buffer, blocking_map, offset, size, hostPtr,
        num_events_in_wait_list, event_wait_list, event);
  } else {
    ScopedContext active(command_queue->get_context());

    if (is_pinned) {
      ret_err = cuda_piEnqueueEventsWait(command_queue, num_events_in_wait_list,
                                         event_wait_list, nullptr);
    }

    if (event) {
      try {
        *event = _pi_event::make_native(
            PI_COMMAND_TYPE_MEM_BUFFER_MAP, command_queue,
            command_queue->get_next_transfer_stream());
        (*event)->start();
        (*event)->record();
      } catch (pi_result error) {
        ret_err = error;
      }
    }
  }

  return ret_err;
}

/// Implements the unmap from the host, using a BufferWrite operation.
/// Requires the mapped pointer to be already registered in the given memobj.
/// If memobj uses pinned host memory, this will not do a write.
///
pi_result cuda_piEnqueueMemUnmap(pi_queue command_queue, pi_mem memobj,
                                 void *mapped_ptr,
                                 pi_uint32 num_events_in_wait_list,
                                 const pi_event *event_wait_list,
                                 pi_event *event) {
  pi_result ret_err = PI_SUCCESS;

  assert(command_queue != nullptr);
  assert(mapped_ptr != nullptr);
  assert(memobj != nullptr);
  assert(memobj->mem_type_ == _pi_mem::mem_type::buffer);
  assert(memobj->mem_.buffer_mem_.get_map_ptr() != nullptr);
  assert(memobj->mem_.buffer_mem_.get_map_ptr() == mapped_ptr);

  const bool is_pinned = memobj->mem_.buffer_mem_.allocMode_ ==
                         _pi_mem::mem_::buffer_mem_::alloc_mode::alloc_host_ptr;

  if (!is_pinned &&
      ((memobj->mem_.buffer_mem_.get_map_flags() & PI_MAP_WRITE) ||
       (memobj->mem_.buffer_mem_.get_map_flags() &
        PI_MAP_WRITE_INVALIDATE_REGION))) {
    // Pinned host memory is only on host so it doesn't need to be written to.
    ret_err = cuda_piEnqueueMemBufferWrite(
        command_queue, memobj, true,
        memobj->mem_.buffer_mem_.get_map_offset(mapped_ptr),
        memobj->mem_.buffer_mem_.get_size(), mapped_ptr,
        num_events_in_wait_list, event_wait_list, event);
  } else {
    ScopedContext active(command_queue->get_context());

    if (is_pinned) {
      ret_err = cuda_piEnqueueEventsWait(command_queue, num_events_in_wait_list,
                                         event_wait_list, nullptr);
    }

    if (event) {
      try {
        *event = _pi_event::make_native(
            PI_COMMAND_TYPE_MEM_BUFFER_UNMAP, command_queue,
            command_queue->get_next_transfer_stream());
        (*event)->start();
        (*event)->record();
      } catch (pi_result error) {
        ret_err = error;
      }
    }
  }

  memobj->mem_.buffer_mem_.unmap(mapped_ptr);
  return ret_err;
}

/// USM: Implements USM Host allocations using CUDA Pinned Memory
///
pi_result cuda_piextUSMHostAlloc(void **result_ptr, pi_context context,
                                 pi_usm_mem_properties *properties, size_t size,
                                 pi_uint32 alignment) {
  assert(result_ptr != nullptr);
  assert(context != nullptr);
  assert(properties == nullptr || *properties == 0);
  pi_result result = PI_SUCCESS;
  try {
    ScopedContext active(context);
    result = PI_CHECK_ERROR(cuMemAllocHost(result_ptr, size));
  } catch (pi_result error) {
    result = error;
  }

  assert(alignment == 0 ||
         (result == PI_SUCCESS &&
          reinterpret_cast<std::uintptr_t>(*result_ptr) % alignment == 0));
  return result;
}

/// USM: Implements USM device allocations using a normal CUDA device pointer
///
pi_result cuda_piextUSMDeviceAlloc(void **result_ptr, pi_context context,
                                   pi_device device,
                                   pi_usm_mem_properties *properties,
                                   size_t size, pi_uint32 alignment) {
  assert(result_ptr != nullptr);
  assert(context != nullptr);
  assert(device != nullptr);
  assert(properties == nullptr || *properties == 0);
  pi_result result = PI_SUCCESS;
  try {
    ScopedContext active(context);
    result = PI_CHECK_ERROR(cuMemAlloc((CUdeviceptr *)result_ptr, size));
  } catch (pi_result error) {
    result = error;
  }

  assert(alignment == 0 ||
         (result == PI_SUCCESS &&
          reinterpret_cast<std::uintptr_t>(*result_ptr) % alignment == 0));
  return result;
}

/// USM: Implements USM Shared allocations using CUDA Managed Memory
///
pi_result cuda_piextUSMSharedAlloc(void **result_ptr, pi_context context,
                                   pi_device device,
                                   pi_usm_mem_properties *properties,
                                   size_t size, pi_uint32 alignment) {
  assert(result_ptr != nullptr);
  assert(context != nullptr);
  assert(device != nullptr);
  assert(properties == nullptr || *properties == 0);
  pi_result result = PI_SUCCESS;
  try {
    ScopedContext active(context);
    result = PI_CHECK_ERROR(cuMemAllocManaged((CUdeviceptr *)result_ptr, size,
                                              CU_MEM_ATTACH_GLOBAL));
  } catch (pi_result error) {
    result = error;
  }

  assert(alignment == 0 ||
         (result == PI_SUCCESS &&
          reinterpret_cast<std::uintptr_t>(*result_ptr) % alignment == 0));
  return result;
}

/// USM: Frees the given USM pointer associated with the context.
///
pi_result cuda_piextUSMFree(pi_context context, void *ptr) {
  assert(context != nullptr);
  pi_result result = PI_SUCCESS;
  try {
    ScopedContext active(context);
    bool is_managed;
    unsigned int type;
    void *attribute_values[2] = {&is_managed, &type};
    CUpointer_attribute attributes[2] = {CU_POINTER_ATTRIBUTE_IS_MANAGED,
                                         CU_POINTER_ATTRIBUTE_MEMORY_TYPE};
    result = PI_CHECK_ERROR(cuPointerGetAttributes(
        2, attributes, attribute_values, (CUdeviceptr)ptr));
    assert(type == CU_MEMORYTYPE_DEVICE || type == CU_MEMORYTYPE_HOST);
    if (is_managed || type == CU_MEMORYTYPE_DEVICE) {
      // Memory allocated with cuMemAlloc and cuMemAllocManaged must be freed
      // with cuMemFree
      result = PI_CHECK_ERROR(cuMemFree((CUdeviceptr)ptr));
    } else {
      // Memory allocated with cuMemAllocHost must be freed with cuMemFreeHost
      result = PI_CHECK_ERROR(cuMemFreeHost(ptr));
    }
  } catch (pi_result error) {
    result = error;
  }
  return result;
}

pi_result cuda_piextUSMEnqueueMemset(pi_queue queue, void *ptr, pi_int32 value,
                                     size_t count,
                                     pi_uint32 num_events_in_waitlist,
                                     const pi_event *events_waitlist,
                                     pi_event *event) {
  assert(queue != nullptr);
  assert(ptr != nullptr);
  pi_result result = PI_SUCCESS;
  std::unique_ptr<_pi_event> event_ptr{nullptr};

  try {
    ScopedContext active(queue->get_context());
    pi_uint32 stream_token;
    _pi_stream_guard guard;
    CUstream cuStream = queue->get_next_compute_stream(
        num_events_in_waitlist,
        reinterpret_cast<const ur_event_handle_t *>(events_waitlist), guard,
        &stream_token);
    result = enqueueEventsWait(queue, cuStream, num_events_in_waitlist,
                               events_waitlist);
    if (event) {
      event_ptr = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_FILL, queue, cuStream, stream_token));
      event_ptr->start();
    }
    result = PI_CHECK_ERROR(cuMemsetD8Async(
        (CUdeviceptr)ptr, (unsigned char)value & 0xFF, count, cuStream));
    if (event) {
      result = map_ur_error(event_ptr->record());
      *event = event_ptr.release();
    }
  } catch (pi_result err) {
    result = err;
  }
  return result;
}

pi_result cuda_piextUSMEnqueueMemcpy(pi_queue queue, pi_bool blocking,
                                     void *dst_ptr, const void *src_ptr,
                                     size_t size,
                                     pi_uint32 num_events_in_waitlist,
                                     const pi_event *events_waitlist,
                                     pi_event *event) {
  assert(queue != nullptr);
  assert(dst_ptr != nullptr);
  assert(src_ptr != nullptr);
  pi_result result = PI_SUCCESS;

  std::unique_ptr<_pi_event> event_ptr{nullptr};

  try {
    ScopedContext active(queue->get_context());
    CUstream cuStream = queue->get_next_transfer_stream();
    result = enqueueEventsWait(queue, cuStream, num_events_in_waitlist,
                               events_waitlist);
    if (event) {
      event_ptr = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_COPY, queue, cuStream));
      event_ptr->start();
    }
    result = PI_CHECK_ERROR(cuMemcpyAsync(
        (CUdeviceptr)dst_ptr, (CUdeviceptr)src_ptr, size, cuStream));
    if (event) {
      result = map_ur_error(event_ptr->record());
    }
    if (blocking) {
      result = PI_CHECK_ERROR(cuStreamSynchronize(cuStream));
    }
    if (event) {
      *event = event_ptr.release();
    }
  } catch (pi_result err) {
    result = err;
  }
  return result;
}

pi_result cuda_piextUSMEnqueuePrefetch(pi_queue queue, const void *ptr,
                                       size_t size,
                                       pi_usm_migration_flags flags,
                                       pi_uint32 num_events_in_waitlist,
                                       const pi_event *events_waitlist,
                                       pi_event *event) {
  pi_device device =
      reinterpret_cast<pi_device>(queue->get_context()->get_device());

  // Certain cuda devices and Windows do not have support for some Unified
  // Memory features. cuMemPrefetchAsync requires concurrent memory access
  // for managed memory. Therfore, ignore prefetch hint if concurrent managed
  // memory access is not available.
  if (!getAttribute(device, CU_DEVICE_ATTRIBUTE_CONCURRENT_MANAGED_ACCESS)) {
    setErrorMessage("Prefetch hint ignored as device does not support "
                    "concurrent managed access",
                    UR_RESULT_SUCCESS);
    return PI_ERROR_PLUGIN_SPECIFIC_ERROR;
  }

  unsigned int is_managed;
  PI_CHECK_ERROR(cuPointerGetAttribute(
      &is_managed, CU_POINTER_ATTRIBUTE_IS_MANAGED, (CUdeviceptr)ptr));
  if (!is_managed) {
    setErrorMessage("Prefetch hint ignored as prefetch only works with USM",
                    UR_RESULT_SUCCESS);
    return PI_ERROR_PLUGIN_SPECIFIC_ERROR;
  }

  // flags is currently unused so fail if set
  if (flags != 0)
    return PI_ERROR_INVALID_VALUE;
  assert(queue != nullptr);
  assert(ptr != nullptr);
  pi_result result = PI_SUCCESS;
  std::unique_ptr<_pi_event> event_ptr{nullptr};

  try {
    ScopedContext active(queue->get_context());
    CUstream cuStream = queue->get_next_transfer_stream();
    result = enqueueEventsWait(queue, cuStream, num_events_in_waitlist,
                               events_waitlist);
    if (event) {
      event_ptr = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_MEM_BUFFER_COPY, queue, cuStream));
      event_ptr->start();
    }
    result = PI_CHECK_ERROR(
        cuMemPrefetchAsync((CUdeviceptr)ptr, size, device->get(), cuStream));
    if (event) {
      result = map_ur_error(event_ptr->record());
      *event = event_ptr.release();
    }
  } catch (pi_result err) {
    result = err;
  }
  return result;
}

/// USM: memadvise API to govern behavior of automatic migration mechanisms
pi_result cuda_piextUSMEnqueueMemAdvise(pi_queue queue, const void *ptr,
                                        size_t length, pi_mem_advice advice,
                                        pi_event *event) {
  assert(queue != nullptr);
  assert(ptr != nullptr);

  // Certain cuda devices and Windows do not have support for some Unified
  // Memory features. Passing CU_MEM_ADVISE_[UN]SET_PREFERRED_LOCATION and
  // CU_MEM_ADVISE_[UN]SET_ACCESSED_BY to cuMemAdvise on a GPU device requires
  // the GPU device to report a non-zero value for
  // CU_DEVICE_ATTRIBUTE_CONCURRENT_MANAGED_ACCESS. Therfore, ignore memory
  // advise if concurrent managed memory access is not available.
  if (advice == PI_MEM_ADVICE_CUDA_SET_PREFERRED_LOCATION ||
      advice == PI_MEM_ADVICE_CUDA_UNSET_PREFERRED_LOCATION ||
      advice == PI_MEM_ADVICE_CUDA_SET_ACCESSED_BY ||
      advice == PI_MEM_ADVICE_CUDA_UNSET_ACCESSED_BY ||
      advice == PI_MEM_ADVICE_RESET) {
    pi_device device =
        reinterpret_cast<pi_device>(queue->get_context()->get_device());
    if (!getAttribute(device, CU_DEVICE_ATTRIBUTE_CONCURRENT_MANAGED_ACCESS)) {
      setErrorMessage("Mem advise ignored as device does not support "
                      "concurrent managed access",
                      UR_RESULT_SUCCESS);
      return PI_ERROR_PLUGIN_SPECIFIC_ERROR;
    }

    // TODO: If ptr points to valid system-allocated pageable memory we should
    // check that the device also has the
    // CU_DEVICE_ATTRIBUTE_PAGEABLE_MEMORY_ACCESS property.
  }

  unsigned int is_managed;
  PI_CHECK_ERROR(cuPointerGetAttribute(
      &is_managed, CU_POINTER_ATTRIBUTE_IS_MANAGED, (CUdeviceptr)ptr));
  if (!is_managed) {
    setErrorMessage(
        "Memory advice ignored as memory advices only works with USM",
        UR_RESULT_SUCCESS);
    return PI_ERROR_PLUGIN_SPECIFIC_ERROR;
  }

  pi_result result = PI_SUCCESS;
  std::unique_ptr<_pi_event> event_ptr{nullptr};

  try {
    ScopedContext active(queue->get_context());

    if (event) {
      event_ptr = std::unique_ptr<_pi_event>(_pi_event::make_native(
          PI_COMMAND_TYPE_USER, queue, queue->get_next_transfer_stream()));
      event_ptr->start();
    }

    switch (advice) {
    case PI_MEM_ADVICE_CUDA_SET_READ_MOSTLY:
    case PI_MEM_ADVICE_CUDA_UNSET_READ_MOSTLY:
    case PI_MEM_ADVICE_CUDA_SET_PREFERRED_LOCATION:
    case PI_MEM_ADVICE_CUDA_UNSET_PREFERRED_LOCATION:
    case PI_MEM_ADVICE_CUDA_SET_ACCESSED_BY:
    case PI_MEM_ADVICE_CUDA_UNSET_ACCESSED_BY:
      result = PI_CHECK_ERROR(cuMemAdvise(
          (CUdeviceptr)ptr, length,
          (CUmem_advise)(advice - PI_MEM_ADVICE_CUDA_SET_READ_MOSTLY + 1),
          queue->get_context()->get_device()->get()));
      break;
    case PI_MEM_ADVICE_CUDA_SET_PREFERRED_LOCATION_HOST:
    case PI_MEM_ADVICE_CUDA_UNSET_PREFERRED_LOCATION_HOST:
    case PI_MEM_ADVICE_CUDA_SET_ACCESSED_BY_HOST:
    case PI_MEM_ADVICE_CUDA_UNSET_ACCESSED_BY_HOST:
      result = PI_CHECK_ERROR(cuMemAdvise(
          (CUdeviceptr)ptr, length,
          (CUmem_advise)(advice - PI_MEM_ADVICE_CUDA_SET_READ_MOSTLY + 1 -
                         (PI_MEM_ADVICE_CUDA_SET_PREFERRED_LOCATION_HOST -
                          PI_MEM_ADVICE_CUDA_SET_PREFERRED_LOCATION)),
          CU_DEVICE_CPU));
      break;
    case PI_MEM_ADVICE_RESET:
      PI_CHECK_ERROR(cuMemAdvise((CUdeviceptr)ptr, length,
                                 CU_MEM_ADVISE_UNSET_READ_MOSTLY,
                                 queue->get_context()->get_device()->get()));
      PI_CHECK_ERROR(cuMemAdvise((CUdeviceptr)ptr, length,
                                 CU_MEM_ADVISE_UNSET_PREFERRED_LOCATION,
                                 queue->get_context()->get_device()->get()));
      PI_CHECK_ERROR(cuMemAdvise((CUdeviceptr)ptr, length,
                                 CU_MEM_ADVISE_UNSET_ACCESSED_BY,
                                 queue->get_context()->get_device()->get()));
      break;
    default:
      sycl::detail::pi::die("Unknown advice");
    }
    if (event) {
      result = map_ur_error(event_ptr->record());
      *event = event_ptr.release();
    }
  } catch (pi_result err) {
    result = err;
  } catch (...) {
    result = PI_ERROR_UNKNOWN;
  }
  return result;
}

// TODO: Implement this. Remember to return true for
//       PI_EXT_ONEAPI_CONTEXT_INFO_USM_FILL2D_SUPPORT when it is implemented.
pi_result cuda_piextUSMEnqueueFill2D(pi_queue, void *, size_t, size_t,
                                     const void *, size_t, size_t, pi_uint32,
                                     const pi_event *, pi_event *) {
  sycl::detail::pi::die("piextUSMEnqueueFill2D: not implemented");
  return {};
}

// TODO: Implement this. Remember to return true for
//       PI_EXT_ONEAPI_CONTEXT_INFO_USM_MEMSET2D_SUPPORT when it is implemented.
pi_result cuda_piextUSMEnqueueMemset2D(pi_queue, void *, size_t, int, size_t,
                                       size_t, pi_uint32, const pi_event *,
                                       pi_event *) {
  sycl::detail::pi::die("cuda_piextUSMEnqueueMemset2D: not implemented");
  return {};
}

/// 2D Memcpy API
///
/// \param queue is the queue to submit to
/// \param blocking is whether this operation should block the host
/// \param dst_ptr is the location the data will be copied
/// \param dst_pitch is the total width of the destination memory including
/// padding
/// \param src_ptr is the data to be copied
/// \param dst_pitch is the total width of the source memory including padding
/// \param width is width in bytes of each row to be copied
/// \param height is height the columns to be copied
/// \param num_events_in_waitlist is the number of events to wait on
/// \param events_waitlist is an array of events to wait on
/// \param event is the event that represents this operation
pi_result cuda_piextUSMEnqueueMemcpy2D(pi_queue queue, pi_bool blocking,
                                       void *dst_ptr, size_t dst_pitch,
                                       const void *src_ptr, size_t src_pitch,
                                       size_t width, size_t height,
                                       pi_uint32 num_events_in_wait_list,
                                       const pi_event *event_wait_list,
                                       pi_event *event) {

  assert(queue != nullptr);

  pi_result result = PI_SUCCESS;

  try {
    ScopedContext active(queue->get_context());
    CUstream cuStream = queue->get_next_transfer_stream();
    result = enqueueEventsWait(queue, cuStream, num_events_in_wait_list,
                               event_wait_list);
    if (event) {
      (*event) = _pi_event::make_native(PI_COMMAND_TYPE_MEM_BUFFER_COPY_RECT,
                                        queue, cuStream);
      (*event)->start();
    }

    // Determine the direction of copy using cuPointerGetAttribute
    // for both the src_ptr and dst_ptr
    CUDA_MEMCPY2D cpyDesc = {0};

    getUSMHostOrDevicePtr(src_ptr, &cpyDesc.srcMemoryType, &cpyDesc.srcDevice,
                          &cpyDesc.srcHost);
    getUSMHostOrDevicePtr(dst_ptr, &cpyDesc.dstMemoryType, &cpyDesc.dstDevice,
                          &cpyDesc.dstHost);

    cpyDesc.dstPitch = dst_pitch;
    cpyDesc.srcPitch = src_pitch;
    cpyDesc.WidthInBytes = width;
    cpyDesc.Height = height;

    result = PI_CHECK_ERROR(cuMemcpy2DAsync(&cpyDesc, cuStream));

    if (event) {
      (*event)->record();
    }
    if (blocking) {
      result = PI_CHECK_ERROR(cuStreamSynchronize(cuStream));
    }
  } catch (pi_result err) {
    result = err;
  }
  return result;
}

/// API to query information about USM allocated pointers
/// Valid Queries:
///   PI_MEM_ALLOC_TYPE returns host/device/shared pi_host_usm value
///   PI_MEM_ALLOC_BASE_PTR returns the base ptr of an allocation if
///                         the queried pointer fell inside an allocation.
///                         Result must fit in void *
///   PI_MEM_ALLOC_SIZE returns how big the queried pointer's
///                     allocation is in bytes. Result is a size_t.
///   PI_MEM_ALLOC_DEVICE returns the pi_device this was allocated against
///
/// \param context is the pi_context
/// \param ptr is the pointer to query
/// \param param_name is the type of query to perform
/// \param param_value_size is the size of the result in bytes
/// \param param_value is the result
/// \param param_value_size_ret is how many bytes were written
pi_result cuda_piextUSMGetMemAllocInfo(pi_context context, const void *ptr,
                                       pi_mem_alloc_info param_name,
                                       size_t param_value_size,
                                       void *param_value,
                                       size_t *param_value_size_ret) {
  assert(context != nullptr);
  assert(ptr != nullptr);
  pi_result result = PI_SUCCESS;

  try {
    ScopedContext active(context);
    switch (param_name) {
    case PI_MEM_ALLOC_TYPE: {
      unsigned int value;
      // do not throw if cuPointerGetAttribute returns CUDA_ERROR_INVALID_VALUE
      CUresult ret = cuPointerGetAttribute(
          &value, CU_POINTER_ATTRIBUTE_IS_MANAGED, (CUdeviceptr)ptr);
      if (ret == CUDA_ERROR_INVALID_VALUE) {
        // pointer not known to the CUDA subsystem
        return getInfo(param_value_size, param_value, param_value_size_ret,
                       PI_MEM_TYPE_UNKNOWN);
      }
      result = check_error(ret, __func__, __LINE__ - 5, __FILE__);
      if (value) {
        // pointer to managed memory
        return getInfo(param_value_size, param_value, param_value_size_ret,
                       PI_MEM_TYPE_SHARED);
      }
      result = PI_CHECK_ERROR(cuPointerGetAttribute(
          &value, CU_POINTER_ATTRIBUTE_MEMORY_TYPE, (CUdeviceptr)ptr));
      assert(value == CU_MEMORYTYPE_DEVICE || value == CU_MEMORYTYPE_HOST);
      if (value == CU_MEMORYTYPE_DEVICE) {
        // pointer to device memory
        return getInfo(param_value_size, param_value, param_value_size_ret,
                       PI_MEM_TYPE_DEVICE);
      }
      if (value == CU_MEMORYTYPE_HOST) {
        // pointer to host memory
        return getInfo(param_value_size, param_value, param_value_size_ret,
                       PI_MEM_TYPE_HOST);
      }
      // should never get here
#ifdef _MSC_VER
      __assume(0);
#else
      __builtin_unreachable();
#endif
      return getInfo(param_value_size, param_value, param_value_size_ret,
                     PI_MEM_TYPE_UNKNOWN);
    }
    case PI_MEM_ALLOC_BASE_PTR: {
#if CUDA_VERSION >= 10020
      // CU_POINTER_ATTRIBUTE_RANGE_START_ADDR was introduced in CUDA 10.2
      unsigned int value;
      result = PI_CHECK_ERROR(cuPointerGetAttribute(
          &value, CU_POINTER_ATTRIBUTE_RANGE_START_ADDR, (CUdeviceptr)ptr));
      return getInfo(param_value_size, param_value, param_value_size_ret,
                     value);
#else
      return PI_ERROR_INVALID_VALUE;
#endif
    }
    case PI_MEM_ALLOC_SIZE: {
#if CUDA_VERSION >= 10020
      // CU_POINTER_ATTRIBUTE_RANGE_SIZE was introduced in CUDA 10.2
      unsigned int value;
      result = PI_CHECK_ERROR(cuPointerGetAttribute(
          &value, CU_POINTER_ATTRIBUTE_RANGE_SIZE, (CUdeviceptr)ptr));
      return getInfo(param_value_size, param_value, param_value_size_ret,
                     value);
#else
      return PI_ERROR_INVALID_VALUE;
#endif
    }
    case PI_MEM_ALLOC_DEVICE: {
      // get device index associated with this pointer
      unsigned int device_idx;
      result = PI_CHECK_ERROR(cuPointerGetAttribute(
          &device_idx, CU_POINTER_ATTRIBUTE_DEVICE_ORDINAL, (CUdeviceptr)ptr));

      // currently each device is in its own platform, so find the platform at
      // the same index
      std::vector<pi_platform> platforms;
      platforms.resize(device_idx + 1);
      result = pi2ur::piPlatformsGet(device_idx + 1, platforms.data(), nullptr);

      // get the device from the platform
      // TODO(ur): Remove cast when this entry point is moved to UR
      pi_device device =
          reinterpret_cast<pi_device>(platforms[device_idx]->devices_[0].get());
      return getInfo(param_value_size, param_value, param_value_size_ret,
                     device);
    }
    }
  } catch (pi_result error) {
    result = error;
  }
  return result;
}

pi_result cuda_piextEnqueueDeviceGlobalVariableWrite(
    pi_queue queue, pi_program program, const char *name,
    pi_bool blocking_write, size_t count, size_t offset, const void *src,
    pi_uint32 num_events_in_wait_list, const pi_event *event_wait_list,
    pi_event *event) {
  assert(queue != nullptr);
  assert(program != nullptr);

  if (name == nullptr || src == nullptr)
    return PI_ERROR_INVALID_VALUE;

  // Since CUDA requires a the global variable to be referenced by name, we use
  // metadata to find the correct name to access it by.
  auto device_global_name_it = program->globalIDMD_.find(name);
  if (device_global_name_it == program->globalIDMD_.end())
    return PI_ERROR_INVALID_VALUE;
  std::string device_global_name = device_global_name_it->second;

  pi_result result = PI_SUCCESS;
  try {
    CUdeviceptr device_global = 0;
    size_t device_global_size = 0;
    result = PI_CHECK_ERROR(
        cuModuleGetGlobal(&device_global, &device_global_size, program->get(),
                          device_global_name.c_str()));

    if (offset + count > device_global_size)
      return PI_ERROR_INVALID_VALUE;

    return cuda_piextUSMEnqueueMemcpy(
        queue, blocking_write, reinterpret_cast<void *>(device_global + offset),
        src, count, num_events_in_wait_list, event_wait_list, event);
  } catch (pi_result error) {
    result = error;
  }
  return result;
}

pi_result cuda_piextEnqueueDeviceGlobalVariableRead(
    pi_queue queue, pi_program program, const char *name, pi_bool blocking_read,
    size_t count, size_t offset, void *dst, pi_uint32 num_events_in_wait_list,
    const pi_event *event_wait_list, pi_event *event) {
  assert(queue != nullptr);
  assert(program != nullptr);

  if (name == nullptr || dst == nullptr)
    return PI_ERROR_INVALID_VALUE;

  // Since CUDA requires a the global variable to be referenced by name, we use
  // metadata to find the correct name to access it by.
  auto device_global_name_it = program->globalIDMD_.find(name);
  if (device_global_name_it == program->globalIDMD_.end())
    return PI_ERROR_INVALID_VALUE;
  std::string device_global_name = device_global_name_it->second;

  pi_result result = PI_SUCCESS;
  try {
    CUdeviceptr device_global = 0;
    size_t device_global_size = 0;
    result = PI_CHECK_ERROR(
        cuModuleGetGlobal(&device_global, &device_global_size, program->get(),
                          device_global_name.c_str()));

    if (offset + count > device_global_size)
      return PI_ERROR_INVALID_VALUE;

    return cuda_piextUSMEnqueueMemcpy(
        queue, blocking_read, dst,
        reinterpret_cast<const void *>(device_global + offset), count,
        num_events_in_wait_list, event_wait_list, event);
  } catch (pi_result error) {
    result = error;
  }
  return result;
}

/// Host Pipes
pi_result cuda_piextEnqueueReadHostPipe(
    pi_queue queue, pi_program program, const char *pipe_symbol,
    pi_bool blocking, void *ptr, size_t size, pi_uint32 num_events_in_waitlist,
    const pi_event *events_waitlist, pi_event *event) {
  (void)queue;
  (void)program;
  (void)pipe_symbol;
  (void)blocking;
  (void)ptr;
  (void)size;
  (void)num_events_in_waitlist;
  (void)events_waitlist;
  (void)event;

  sycl::detail::pi::die("cuda_piextEnqueueReadHostPipe not implemented");
  return {};
}

pi_result cuda_piextEnqueueWriteHostPipe(
    pi_queue queue, pi_program program, const char *pipe_symbol,
    pi_bool blocking, void *ptr, size_t size, pi_uint32 num_events_in_waitlist,
    const pi_event *events_waitlist, pi_event *event) {
  (void)queue;
  (void)program;
  (void)pipe_symbol;
  (void)blocking;
  (void)ptr;
  (void)size;
  (void)num_events_in_waitlist;
  (void)events_waitlist;
  (void)event;

  sycl::detail::pi::die("cuda_piextEnqueueWriteHostPipe not implemented");
  return {};
}

const char SupportedVersion[] = _PI_CUDA_PLUGIN_VERSION_STRING;

pi_result piPluginInit(pi_plugin *PluginInit) {
  // Check that the major version matches in PiVersion and SupportedVersion
  _PI_PLUGIN_VERSION_CHECK(PluginInit->PiVersion, SupportedVersion);

  // PI interface supports higher version or the same version.
  size_t PluginVersionSize = sizeof(PluginInit->PluginVersion);
  if (strlen(SupportedVersion) >= PluginVersionSize)
    return PI_ERROR_INVALID_VALUE;
  strncpy(PluginInit->PluginVersion, SupportedVersion, PluginVersionSize);

  // Set whole function table to zero to make it easier to detect if
  // functions are not set up below.
  std::memset(&(PluginInit->PiFunctionTable), 0,
              sizeof(PluginInit->PiFunctionTable));

  enableCUDATracing();

// Forward calls to CUDA RT.
#define _PI_CL(pi_api, cuda_api)                                               \
  (PluginInit->PiFunctionTable).pi_api = (decltype(&::pi_api))(&cuda_api);

  // Platform
  _PI_CL(piPlatformsGet, pi2ur::piPlatformsGet)
  _PI_CL(piPlatformGetInfo, pi2ur::piPlatformGetInfo)
  // Device
  _PI_CL(piDevicesGet, pi2ur::piDevicesGet)
  _PI_CL(piDeviceGetInfo, pi2ur::piDeviceGetInfo)
  _PI_CL(piDevicePartition, pi2ur::piDevicePartition)
  _PI_CL(piDeviceRetain, pi2ur::piDeviceRetain)
  _PI_CL(piDeviceRelease, pi2ur::piDeviceRelease)
  _PI_CL(piextDeviceSelectBinary, cuda_piextDeviceSelectBinary)
  _PI_CL(piextGetDeviceFunctionPointer, cuda_piextGetDeviceFunctionPointer)
  _PI_CL(piextDeviceGetNativeHandle, pi2ur::piextDeviceGetNativeHandle)
  _PI_CL(piextDeviceCreateWithNativeHandle,
         pi2ur::piextDeviceCreateWithNativeHandle)
  // Context
  _PI_CL(piextContextSetExtendedDeleter, pi2ur::piextContextSetExtendedDeleter)
  _PI_CL(piContextCreate, pi2ur::piContextCreate)
  _PI_CL(piContextGetInfo, pi2ur::piContextGetInfo)
  _PI_CL(piContextRetain, pi2ur::piContextRetain)
  _PI_CL(piContextRelease, pi2ur::piContextRelease)
  _PI_CL(piextContextGetNativeHandle, pi2ur::piextContextGetNativeHandle)
  _PI_CL(piextContextCreateWithNativeHandle,
         pi2ur::piextContextCreateWithNativeHandle)
  // Queue
  _PI_CL(piQueueCreate, pi2ur::piQueueCreate)
  _PI_CL(piextQueueCreate, pi2ur::piextQueueCreate)
  _PI_CL(piextQueueCreate2, pi2ur::piextQueueCreate2)
  _PI_CL(piQueueGetInfo, pi2ur::piQueueGetInfo)
  _PI_CL(piQueueFinish, pi2ur::piQueueFinish)
  _PI_CL(piQueueFlush, pi2ur::piQueueFlush)
  _PI_CL(piQueueRetain, pi2ur::piQueueRetain)
  _PI_CL(piQueueRelease, pi2ur::piQueueRelease)
  _PI_CL(piextQueueGetNativeHandle, pi2ur::piextQueueGetNativeHandle)
  _PI_CL(piextQueueGetNativeHandle2, pi2ur::piextQueueGetNativeHandle2)
  _PI_CL(piextQueueCreateWithNativeHandle,
         pi2ur::piextQueueCreateWithNativeHandle)
  _PI_CL(piextQueueCreateWithNativeHandle2,
         pi2ur::piextQueueCreateWithNativeHandle2)
  // Memory
  _PI_CL(piMemBufferCreate, cuda_piMemBufferCreate)
  _PI_CL(piMemImageCreate, cuda_piMemImageCreate)
  _PI_CL(piMemGetInfo, cuda_piMemGetInfo)
  _PI_CL(piMemImageGetInfo, cuda_piMemImageGetInfo)
  _PI_CL(piMemRetain, cuda_piMemRetain)
  _PI_CL(piMemRelease, cuda_piMemRelease)
  _PI_CL(piMemBufferPartition, cuda_piMemBufferPartition)
  _PI_CL(piextMemGetNativeHandle, cuda_piextMemGetNativeHandle)
  _PI_CL(piextMemCreateWithNativeHandle, cuda_piextMemCreateWithNativeHandle)
  // Program
  _PI_CL(piProgramCreate, pi2ur::piProgramCreate)
  _PI_CL(piclProgramCreateWithSource, pi2ur::piclProgramCreateWithSource)
  _PI_CL(piProgramCreateWithBinary, pi2ur::piProgramCreateWithBinary)
  _PI_CL(piProgramGetInfo, pi2ur::piProgramGetInfo)
  _PI_CL(piProgramCompile, pi2ur::piProgramCompile)
  _PI_CL(piProgramBuild, pi2ur::piProgramBuild)
  _PI_CL(piProgramLink, pi2ur::piProgramLink)
  _PI_CL(piProgramGetBuildInfo, pi2ur::piProgramGetBuildInfo)
  _PI_CL(piProgramRetain, pi2ur::piProgramRetain)
  _PI_CL(piProgramRelease, pi2ur::piProgramRelease)
  _PI_CL(piextProgramGetNativeHandle, pi2ur::piextProgramGetNativeHandle)
  _PI_CL(piextProgramCreateWithNativeHandle,
         pi2ur::piextProgramCreateWithNativeHandle)
  _PI_CL(piextProgramSetSpecializationConstant,
         pi2ur::piextProgramSetSpecializationConstant)
  // Kernel
  _PI_CL(piKernelCreate, pi2ur::piKernelCreate)
  _PI_CL(piKernelSetArg, pi2ur::piKernelSetArg)
  _PI_CL(piKernelGetInfo, pi2ur::piKernelGetInfo)
  _PI_CL(piKernelGetGroupInfo, pi2ur::piKernelGetGroupInfo)
  _PI_CL(piKernelGetSubGroupInfo, pi2ur::piKernelGetSubGroupInfo)
  _PI_CL(piKernelRetain, pi2ur::piKernelRetain)
  _PI_CL(piKernelRelease, pi2ur::piKernelRelease)
  _PI_CL(piextKernelGetNativeHandle, pi2ur::piKernelGetNativeHandle)
  _PI_CL(piKernelSetExecInfo, pi2ur::piKernelSetExecInfo)
  _PI_CL(piextKernelSetArgPointer, pi2ur::piKernelSetArgPointer)
  _PI_CL(piextKernelCreateWithNativeHandle,
         pi2ur::piextKernelCreateWithNativeHandle)

  // Event
  _PI_CL(piEventCreate, pi2ur::piEventCreate)
  _PI_CL(piEventGetInfo, pi2ur::piEventGetInfo)
  _PI_CL(piEventGetProfilingInfo, pi2ur::piEventGetProfilingInfo)
  _PI_CL(piEventsWait, pi2ur::piEventsWait)
  _PI_CL(piEventSetCallback, pi2ur::piEventSetCallback)
  _PI_CL(piEventSetStatus, pi2ur::piEventSetStatus)
  _PI_CL(piEventRetain, pi2ur::piEventRetain)
  _PI_CL(piEventRelease, pi2ur::piEventRelease)
  _PI_CL(piextEventGetNativeHandle, pi2ur::piextEventGetNativeHandle)
  _PI_CL(piextEventCreateWithNativeHandle,
         pi2ur::piextEventCreateWithNativeHandle)
  // Sampler
  _PI_CL(piSamplerCreate, cuda_piSamplerCreate)
  _PI_CL(piSamplerGetInfo, cuda_piSamplerGetInfo)
  _PI_CL(piSamplerRetain, cuda_piSamplerRetain)
  _PI_CL(piSamplerRelease, cuda_piSamplerRelease)
  // Queue commands
  _PI_CL(piEnqueueKernelLaunch, cuda_piEnqueueKernelLaunch)
  _PI_CL(piEnqueueNativeKernel, cuda_piEnqueueNativeKernel)
  _PI_CL(piEnqueueEventsWait, cuda_piEnqueueEventsWait)
  _PI_CL(piEnqueueEventsWaitWithBarrier, cuda_piEnqueueEventsWaitWithBarrier)
  _PI_CL(piEnqueueMemBufferRead, cuda_piEnqueueMemBufferRead)
  _PI_CL(piEnqueueMemBufferReadRect, cuda_piEnqueueMemBufferReadRect)
  _PI_CL(piEnqueueMemBufferWrite, cuda_piEnqueueMemBufferWrite)
  _PI_CL(piEnqueueMemBufferWriteRect, cuda_piEnqueueMemBufferWriteRect)
  _PI_CL(piEnqueueMemBufferCopy, cuda_piEnqueueMemBufferCopy)
  _PI_CL(piEnqueueMemBufferCopyRect, cuda_piEnqueueMemBufferCopyRect)
  _PI_CL(piEnqueueMemBufferFill, cuda_piEnqueueMemBufferFill)
  _PI_CL(piEnqueueMemImageRead, cuda_piEnqueueMemImageRead)
  _PI_CL(piEnqueueMemImageWrite, cuda_piEnqueueMemImageWrite)
  _PI_CL(piEnqueueMemImageCopy, cuda_piEnqueueMemImageCopy)
  _PI_CL(piEnqueueMemImageFill, cuda_piEnqueueMemImageFill)
  _PI_CL(piEnqueueMemBufferMap, cuda_piEnqueueMemBufferMap)
  _PI_CL(piEnqueueMemUnmap, cuda_piEnqueueMemUnmap)
  // USM
  _PI_CL(piextUSMHostAlloc, cuda_piextUSMHostAlloc)
  _PI_CL(piextUSMDeviceAlloc, cuda_piextUSMDeviceAlloc)
  _PI_CL(piextUSMSharedAlloc, cuda_piextUSMSharedAlloc)
  _PI_CL(piextUSMFree, cuda_piextUSMFree)
  _PI_CL(piextUSMEnqueueMemset, cuda_piextUSMEnqueueMemset)
  _PI_CL(piextUSMEnqueueMemcpy, cuda_piextUSMEnqueueMemcpy)
  _PI_CL(piextUSMEnqueuePrefetch, cuda_piextUSMEnqueuePrefetch)
  _PI_CL(piextUSMEnqueueMemAdvise, cuda_piextUSMEnqueueMemAdvise)
  _PI_CL(piextUSMEnqueueFill2D, cuda_piextUSMEnqueueFill2D)
  _PI_CL(piextUSMEnqueueMemset2D, cuda_piextUSMEnqueueMemset2D)
  _PI_CL(piextUSMEnqueueMemcpy2D, cuda_piextUSMEnqueueMemcpy2D)
  _PI_CL(piextUSMGetMemAllocInfo, cuda_piextUSMGetMemAllocInfo)
  // Device global variable
  _PI_CL(piextEnqueueDeviceGlobalVariableWrite,
         cuda_piextEnqueueDeviceGlobalVariableWrite)
  _PI_CL(piextEnqueueDeviceGlobalVariableRead,
         cuda_piextEnqueueDeviceGlobalVariableRead)

  // Host Pipe
  _PI_CL(piextEnqueueReadHostPipe, cuda_piextEnqueueReadHostPipe)
  _PI_CL(piextEnqueueWriteHostPipe, cuda_piextEnqueueWriteHostPipe)

  _PI_CL(piextKernelSetArgMemObj, cuda_piextKernelSetArgMemObj)
  _PI_CL(piextKernelSetArgSampler, cuda_piextKernelSetArgSampler)
  _PI_CL(piPluginGetLastError, pi2ur::piPluginGetLastError)
  _PI_CL(piTearDown, pi2ur::piTearDown)
  _PI_CL(piGetDeviceAndHostTimer, pi2ur::piGetDeviceAndHostTimer)
  _PI_CL(piPluginGetBackendOption, cuda_piPluginGetBackendOption)

#undef _PI_CL

  return PI_SUCCESS;
}

#ifdef _WIN32
#define __SYCL_PLUGIN_DLL_NAME "pi_cuda.dll"
#include "../common_win_pi_trace/common_win_pi_trace.hpp"
#undef __SYCL_PLUGIN_DLL_NAME
#endif

} // extern "C"
