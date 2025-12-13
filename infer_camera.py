# infer_camera.py — realtime на DA3METRIC-LARGE
import sys
from pathlib import Path

# добавляем папку src в sys.path
sys.path.append(str(Path(__file__).resolve().parent / "src"))

import os
import cv2
import torch
import numpy as np
from depth_anything_3.api import DepthAnything3

LOCAL_MODEL_DIR = os.path.join("src", "depth_anything_3", "weights", "DA3METRIC-LARGE")

def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("Device:", device)

    model = DepthAnything3.from_pretrained(LOCAL_MODEL_DIR)
    model.to(device)
    model.eval()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Camera not opened (index 0).")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # frame is BGR from OpenCV; convert to RGB
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # model.inference expects a list
        prediction = model.inference([img_rgb])
        depth = prediction.depth[0]

        depth_norm = depth / (np.max(depth) + 1e-8)
        depth_vis = (depth_norm * 255.0).astype(np.uint8)

        cv2.imshow("RGB", frame)
        cv2.imshow("Depth", depth_vis)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
