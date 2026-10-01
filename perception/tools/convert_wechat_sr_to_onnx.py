"""Export the bundled, hash-checked WeChat SR network for OpenCV 5.

Development-only: requires OpenCV 4, onnx, onnxruntime and numpy. The service
loads the exported sr.onnx directly and does not need the conversion packages.
Run from the repository root: python perception/tools/convert_wechat_sr_to_onnx.py
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import cv2
import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import onnxruntime as ort


def convert(directory: Path) -> Path:
    if int(cv2.__version__.split(".")[0]) >= 5:
        raise RuntimeError("Conversion needs OpenCV 4 to read the original Caffe weights")
    # This exporter describes this exact network, not arbitrary Caffe models.
    source_hashes = {
        "sr.prototxt": "69db99927a70df953b471daaba03fbef",
        "sr.caffemodel": "cbfcd60361a73beb8c583eea7e8e6664",
    }
    buffers = []
    for name, digest in source_hashes.items():
        data = (directory / name).read_bytes()
        if hashlib.md5(data).hexdigest() != digest:
            raise ValueError(f"Unexpected {name}; this exporter only supports the bundled WeChat SR model")
        buffers.append(np.frombuffer(data, dtype=np.uint8))
    caffe = cv2.dnn.readNetFromCaffe(*buffers)
    nodes, weights = [], []

    def conv(name, source, *, pad=0, stride=1, group=1, transpose=False, activate=True):
        blobs = caffe.getLayer(caffe.getLayerId(name)).blobs
        inputs = [source]
        for index, blob in enumerate(blobs):
            tensor_name = f"{name}.{'weight' if index == 0 else 'bias'}"
            weights.append(numpy_helper.from_array(blob if index == 0 else blob.flatten(), tensor_name))
            inputs.append(tensor_name)
        nodes.append(helper.make_node(
            "ConvTranspose" if transpose else "Conv", inputs, [name], name=name,
            kernel_shape=list(blobs[0].shape[-2:]), strides=[stride, stride],
            pads=[pad] * 4, group=group,
        ))
        if activate:
            output = name + ".activated"
            nodes.append(helper.make_node("LeakyRelu", [name], [output], alpha=0.05000000074505806))
            return output
        return name

    current = conv("conv0", "data", pad=1)
    for block in ("db1", "db2"):
        reduced = conv(f"{block}/reduce", current)
        depthwise = conv(f"{block}/3x3", reduced, pad=1, group=8)
        expanded = conv(f"{block}/1x1", depthwise)
        output = f"{block}/concat"
        nodes.append(helper.make_node("Concat", [current, expanded], [output], axis=1))
        current = output
    current = conv("upsample/reduce", current)
    current = conv("upsample/deconv", current, pad=1, stride=2, group=32, transpose=True)
    residual = conv("upsample/rec", current, activate=False)
    nearest = conv("nearest", "data", stride=2, transpose=True, activate=False)
    # Caffe Crop1 takes the top-left (2H-1)x(2W-1) of the 2Hx2W nearest
    # branch. Asymmetric ConvTranspose padding performs the same crop without
    # shape-dependent Slice nodes, including for non-square dynamic inputs.
    for attribute in nodes[-1].attribute:
        if attribute.name == "pads":
            attribute.ints[:] = [0, 0, 1, 1]
    nodes.append(helper.make_node("Add", [nearest, residual], ["fc"]))
    graph = helper.make_graph(
        nodes, "WeChatSR", [helper.make_tensor_value_info("data", TensorProto.FLOAT, [1, 1, "height", "width"])],
        [helper.make_tensor_value_info("fc", TensorProto.FLOAT, [1, 1, "out_height", "out_width"])], weights,
    )
    model = helper.make_model(graph, producer_name="CDF_warehouse WeChat SR converter",
                              opset_imports=[helper.make_opsetid("", 11)], ir_version=7)
    helper.set_model_props(model, {f"source_md5/{name}": digest for name, digest in source_hashes.items()})
    onnx.checker.check_model(model)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    runtime = ort.InferenceSession(model.SerializeToString(), sess_options=options, providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(0)
    max_error = 0.0
    for height, width in ((17, 29), (48, 96), (64, 111)):
        sample = rng.random((1, 1, height, width), dtype=np.float32)
        caffe.setInput(sample)
        expected = caffe.forward()
        actual = runtime.run(None, {"data": sample})[0]
        np.testing.assert_allclose(actual, expected, rtol=1e-4, atol=1e-5)
        max_error = max(max_error, float(np.max(np.abs(actual - expected))))
    target = directory / "sr.onnx"
    target.write_bytes(model.SerializeToString())
    print(f"Validated Caffe/ONNX outputs: max_abs_error={max_error:.9g}")
    print(f"Wrote {target.name}: {target.stat().st_size} bytes, sha256={hashlib.sha256(target.read_bytes()).hexdigest()}")
    return target


if __name__ == "__main__":
    convert(Path(__file__).resolve().parents[1] / "opencv_3rdparty-wechat_qrcode")
