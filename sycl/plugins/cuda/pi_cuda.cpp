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

pi_mem_type map_ur_mem_type(ur_mem_type_t mem_type) {
  switch (mem_type) {
  case UR_MEM_TYPE_BUFFER:
  default:
    return PI_MEM_TYPE_BUFFER;
  case UR_MEM_TYPE_IMAGE2D:
    return PI_MEM_TYPE_IMAGE2D;
  case UR_MEM_TYPE_IMAGE3D:
    return PI_MEM_TYPE_IMAGE3D;
  case UR_MEM_TYPE_IMAGE2D_ARRAY:
    return PI_MEM_TYPE_IMAGE2D_ARRAY;
  case UR_MEM_TYPE_IMAGE1D:
    return PI_MEM_TYPE_IMAGE1D;
  case UR_MEM_TYPE_IMAGE1D_ARRAY:
    return PI_MEM_TYPE_IMAGE1D_ARRAY;
  case UR_MEM_TYPE_IMAGE1D_BUFFER:
    return PI_MEM_TYPE_IMAGE1D_BUFFER;
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
/// \endcond

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

/// \TODO Not implemented
pi_result cuda_piEnqueueNativeKernel(pi_queue, void (*)(void *), void *, size_t,
                                     pi_uint32, const pi_mem *, const void **,
                                     pi_uint32, const pi_event *, pi_event *) {
  sycl::detail::pi::die("Not implemented in CUDA backend");
  return {};
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

    pi_mem_type imgType =
        map_ur_mem_type(image->mem_.surface_mem_.get_image_type());
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

    pi_mem_type imgType =
        map_ur_mem_type(image->mem_.surface_mem_.get_image_type());
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

    pi_mem_type imgType =
        map_ur_mem_type(src_image->mem_.surface_mem_.get_image_type());
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
  // TODO(ur): Remove conversion when this is ported to UR.
  ur_map_flags_t map_flags_ur;
  pi2urMapFlags(map_flags, &map_flags_ur);
  auto hostPtr = buffer->mem_.buffer_mem_.map_to_ptr(offset, map_flags_ur);
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
      ret_err = pi2ur::piEnqueueEventsWait(
          command_queue, num_events_in_wait_list, event_wait_list, nullptr);
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
      ret_err = pi2ur::piEnqueueEventsWait(
          command_queue, num_events_in_wait_list, event_wait_list, nullptr);
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

    return pi2ur::piextUSMEnqueueMemcpy(
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

    return pi2ur::piextUSMEnqueueMemcpy(
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
  _PI_CL(piMemBufferCreate, pi2ur::piMemBufferCreate)
  _PI_CL(piMemImageCreate, pi2ur::piMemImageCreate)
  _PI_CL(piMemGetInfo, pi2ur::piMemGetInfo)
  _PI_CL(piMemImageGetInfo, pi2ur::piMemImageGetInfo)
  _PI_CL(piMemRetain, pi2ur::piMemRetain)
  _PI_CL(piMemRelease, pi2ur::piMemRelease)
  _PI_CL(piMemBufferPartition, pi2ur::piMemBufferPartition)
  _PI_CL(piextMemGetNativeHandle, pi2ur::piextMemGetNativeHandle)
  _PI_CL(piextMemCreateWithNativeHandle, pi2ur::piextMemCreateWithNativeHandle)

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
  _PI_CL(piSamplerCreate, pi2ur::piSamplerCreate)
  _PI_CL(piSamplerGetInfo, pi2ur::piSamplerGetInfo)
  _PI_CL(piSamplerRetain, pi2ur::piSamplerRetain)
  _PI_CL(piSamplerRelease, pi2ur::piSamplerRelease)
  // Queue commands
  _PI_CL(piEnqueueKernelLaunch, pi2ur::piEnqueueKernelLaunch)
  _PI_CL(piEnqueueNativeKernel, cuda_piEnqueueNativeKernel)
  _PI_CL(piEnqueueEventsWait, pi2ur::piEnqueueEventsWait)
  _PI_CL(piEnqueueEventsWaitWithBarrier, pi2ur::piEnqueueEventsWaitWithBarrier)
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
  _PI_CL(piextUSMHostAlloc, pi2ur::piextUSMHostAlloc)
  _PI_CL(piextUSMDeviceAlloc, pi2ur::piextUSMDeviceAlloc)
  _PI_CL(piextUSMSharedAlloc, pi2ur::piextUSMSharedAlloc)
  _PI_CL(piextUSMFree, pi2ur::piextUSMFree)
  _PI_CL(piextUSMEnqueueMemset, pi2ur::piextUSMEnqueueMemset)
  _PI_CL(piextUSMEnqueueMemcpy, pi2ur::piextUSMEnqueueMemcpy)
  _PI_CL(piextUSMEnqueuePrefetch, pi2ur::piextUSMEnqueuePrefetch)
  _PI_CL(piextUSMEnqueueMemAdvise, pi2ur::piextUSMEnqueueMemAdvise)
  _PI_CL(piextUSMEnqueueFill2D, pi2ur::piextUSMEnqueueFill2D)
  _PI_CL(piextUSMEnqueueMemset2D, pi2ur::piextUSMEnqueueMemset2D)
  _PI_CL(piextUSMEnqueueMemcpy2D, pi2ur::piextUSMEnqueueMemcpy2D)
  _PI_CL(piextUSMGetMemAllocInfo, pi2ur::piextUSMGetMemAllocInfo)
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
