# infer_image.py — запуск inference для одного изображения (DA3METRIC-LARGE)
import sys
from pathlib import Path

# добавляем папку src в sys.path
sys.path.append(str(Path(__file__).resolve().parent / "src"))

import os
import cv2
import torch
import numpy as np
from depth_anything_3.api import DepthAnything3

# Путь к локальной папке с моделью (от корня проекта)
LOCAL_MODEL_DIR = os.path.join("src", "depth_anything_3", "weights", "DA3METRIC-LARGE")

# Путь к входному изображению
INPUT_IMAGE = "assets/images/inputs/test_2.jpg"

# Папка для сохранения результата
OUTPUT_DIR = "assets/images/outputs"
BASE_NAME = "depth_output"
EXT = ".png"

# Находим следующий свободный номер
i = 1
while os.path.exists(os.path.join(OUTPUT_DIR, f"{BASE_NAME}_{i}{EXT}")):
    i += 1

OUT_PATH = os.path.join(OUTPUT_DIR, f"{BASE_NAME}_{i}{EXT}")

def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("Device:", device)

    # Загружаем модель из локальной папки через from_pretrained(local_path)
    print("Loading model from:", LOCAL_MODEL_DIR)
    model = DepthAnything3.from_pretrained(LOCAL_MODEL_DIR)  # HuggingFace-style local load
    model.to(device)
    model.eval()

    # Чтение изображения
    img_bgr = cv2.imread(INPUT_IMAGE)
    if img_bgr is None:
        raise FileNotFoundError(f"Input image not found: {INPUT_IMAGE}. Помести test.jpg в корень проекта или укажи корректный путь.")
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    # inference: метод API принимает список изображений
    print("Running inference...")
    prediction = model.inference([img_rgb])

    # prediction.depth — скорее всего numpy array (N, H, W)
    depth = prediction.depth[0]   # numpy array float32

    # Нормализация для сохранения
    depth_norm = depth / (np.max(depth) + 1e-8)
    depth_vis = (depth_norm * 255.0).astype(np.uint8)
    cv2.imwrite(OUT_PATH, depth_vis)
    print("Saved:", OUT_PATH)

if __name__ == "__main__":
    main()
