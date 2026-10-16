"""torch -> policy.tflite via litert_torch, proven with the ai_edge_litert
interpreter the Pi uses. Same toolchain and pins as camera_setup."""
import numpy as np
import torch

from model import Deployed


def export_tflite(model, path, check_frames, tol=1e-4):
    """Write path; return the worst |tflite - torch| over check_frames (rad).
    Exits if it exceeds tol, so a bad export never reaches the car."""
    import litert_torch
    from ai_edge_litert.interpreter import Interpreter
    deployed = Deployed(model).eval()
    # Keyword trace names the tflite input tensor img_in (positional = args_0).
    litert_torch.convert(deployed, sample_kwargs={'img_in': torch.zeros(1, 120, 160, 3)}).export(path)
    it = Interpreter(model_path=path)
    it.allocate_tensors()
    inp, out = it.get_input_details()[0], it.get_output_details()[0]
    with torch.no_grad():
        ref = deployed(torch.from_numpy(check_frames).float()).numpy()
    got = []
    for f in check_frames:
        it.set_tensor(inp['index'], f[None].astype(np.float32))
        it.invoke()
        got.append(float(it.get_tensor(out['index']).ravel()[0]))
    worst = float(np.abs(np.array(got) - ref).max())
    if worst > tol:
        raise SystemExit(f'tflite disagrees with torch by {worst:.2e} rad')
    return worst
