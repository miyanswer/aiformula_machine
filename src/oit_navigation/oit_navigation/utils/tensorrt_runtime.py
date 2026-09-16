"""
tensorrt_runtime.py - Minimal TensorRT engine runner for YOLOP inference on real
hardware (Jetson). Only imported when yolop_lane_detector's `use_tensorrt`
parameter is enabled; this module's own imports (tensorrt, pycuda) are simply
absent on machines without the NVIDIA TensorRT SDK - e.g. this project's
default Mac/CPU dev container - so importing it there raises ImportError,
which yolop_lane_detector catches to fall back to plain PyTorch.

Engines are built with export_tensorrt.py's YOLOPExportWrapper, which always
declares exactly one input ("input") and two outputs ("raw_detections",
"ll_seg") - the same two tensors yolop_lane_detector's PyTorch path already
unpacks from the model's forward() before running NMS/lane decoding. This
wrapper's job is only to reproduce those same two tensors from an engine
instead of a live nn.Module.

Supports both of TensorRT's binding APIs, auto-detected from the installed
`tensorrt` package version:
- TensorRT < 10 (most JetPack 5/6 releases, i.e. real hardware): legacy
  binding-index API (num_bindings / get_binding_* / execute_async_v2).
- TensorRT >= 10 (e.g. a desktop dev GPU with a recent pip `tensorrt-cu12`):
  tensor-name API (num_io_tensors / get_tensor_* / execute_async_v3).
"""

import numpy as np
import torch


class TensorRTYOLOPRunner:
    """Loads a serialized TensorRT engine and runs inference, returning the
    same (raw_detections, ll_seg) tensors the PyTorch path produces so
    yolop_lane_detector's NMS/lane-decode code is identical either way."""

    def __init__(self, engine_path: str):
        import tensorrt as trt
        import pycuda.driver as cuda
        import pycuda.autoinit  # noqa: F401  (creates/binds the CUDA context)

        self._trt = trt
        self._cuda = cuda
        self._is_trt10 = int(trt.__version__.split(".")[0]) >= 10

        # pycuda.autoinit's context is only "current" on the thread that created
        # it (here: whatever thread constructs this runner, typically the ROS
        # node's main thread during __init__). yolop_lane_detector.py actually
        # calls infer() from a separate dedicated inference thread, so without
        # explicitly pushing this context onto that thread too, every CUDA call
        # below fails with "invalid resource handle" (looks fine at import/init
        # time - it only breaks once a real inference is attempted).
        self._cuda_context = cuda.Context.get_current()

        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine: {engine_path}")
        self.context = self.engine.create_execution_context()

        self._input_name, self._output_names = self._discover_bindings()
        self._allocate_buffers()
        self.stream = cuda.Stream()

    def _discover_bindings(self):
        trt = self._trt
        input_name = None
        output_names = []
        if self._is_trt10:
            for i in range(self.engine.num_io_tensors):
                name = self.engine.get_tensor_name(i)
                if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                    input_name = name
                else:
                    output_names.append(name)
        else:
            for i in range(self.engine.num_bindings):
                name = self.engine.get_binding_name(i)
                if self.engine.binding_is_input(i):
                    input_name = name
                else:
                    output_names.append(name)
        if input_name is None or len(output_names) != 2:
            raise RuntimeError(
                "Unexpected TensorRT engine bindings (expected 1 input + 2 outputs: "
                "raw_detections, ll_seg) - re-export with export_tensorrt.py"
            )
        # Keep a stable order matching YOLOPExportWrapper's declared output_names.
        output_names.sort(key=lambda n: 0 if n == "raw_detections" else 1)
        return input_name, output_names

    def _allocate_buffers(self):
        trt = self._trt
        self.buffers = {}
        if self._is_trt10:
            names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
            for name in names:
                shape = tuple(self.engine.get_tensor_shape(name))
                dtype = trt.nptype(self.engine.get_tensor_dtype(name))
                host_mem = self._cuda.pagelocked_empty(int(np.prod(shape)), dtype)
                device_mem = self._cuda.mem_alloc(host_mem.nbytes)
                self.buffers[name] = {"host": host_mem, "device": device_mem, "shape": shape}
                # Addresses are stable for the life of these buffers, so this only
                # needs to be set once here rather than before every infer() call.
                self.context.set_tensor_address(name, int(device_mem))
        else:
            for i in range(self.engine.num_bindings):
                name = self.engine.get_binding_name(i)
                shape = tuple(self.engine.get_binding_shape(i))
                dtype = trt.nptype(self.engine.get_binding_dtype(i))
                host_mem = self._cuda.pagelocked_empty(int(np.prod(shape)), dtype)
                device_mem = self._cuda.mem_alloc(host_mem.nbytes)
                self.buffers[name] = {"host": host_mem, "device": device_mem, "shape": shape}

    def infer(self, input_tensor: torch.Tensor):
        """`input_tensor`: (1,3,H,W) float32 torch tensor, already letterboxed+normalized.
        Returns (raw_detections, ll_seg) as torch tensors, matching the PyTorch path."""
        # Make this runner's CUDA context current on whichever thread calls infer()
        # (see the note in __init__ - it's created on the constructing thread but
        # actually used from yolop_lane_detector's dedicated inference thread).
        self._cuda_context.push()
        try:
            input_np = np.ascontiguousarray(input_tensor.detach().cpu().numpy().astype(np.float32))

            input_buf = self.buffers[self._input_name]
            np.copyto(input_buf["host"], input_np.ravel())
            self._cuda.memcpy_htod_async(input_buf["device"], input_buf["host"], self.stream)

            if self._is_trt10:
                self.context.execute_async_v3(stream_handle=self.stream.handle)
            else:
                binding_order = [self._input_name] + self._output_names
                bindings = [int(self.buffers[name]["device"]) for name in binding_order]
                self.context.execute_async_v2(bindings=bindings, stream_handle=self.stream.handle)

            for name in self._output_names:
                buf = self.buffers[name]
                self._cuda.memcpy_dtoh_async(buf["host"], buf["device"], self.stream)
            self.stream.synchronize()

            outputs = {}
            for name in self._output_names:
                buf = self.buffers[name]
                outputs[name] = torch.from_numpy(buf["host"].reshape(buf["shape"]).copy())

            return outputs["raw_detections"], outputs["ll_seg"]
        finally:
            self._cuda_context.pop()
