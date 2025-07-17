import os
import json
import random
import shutil
import numpy as np
from PIL import Image
from tqdm import tqdm

# --- CONFIGURATION ---
SOURCE_DIRS = ["left", "right"]
OUTPUT_DIR = "yolo_dataset"
TRAIN_RATIO = 0.8

# --- COLLECT ALL CLASS LABELS ---
print("🔍 Scanning label files for all unique classes...")
all_class_map = {}

for source in SOURCE_DIRS:
    label_dir = os.path.join(source, "bounding_box_2d_tight")
    label_files = [f for f in os.listdir(label_dir) if f.startswith("bounding_box_2d_tight_labels_") and f.endswith(".json")]
    for label_file in label_files:
        label_path = os.path.join(label_dir, label_file)
        try:
            with open(label_path, 'r') as f:
                data = json.load(f)
            if isinstance(data, dict):
                for k, v in data.items():
                    cls_id = int(k)
                    cls_name = v["class"]
                    if cls_id not in all_class_map:
                        all_class_map[cls_id] = cls_name
        except Exception as e:
            print(f"❌ Error reading {label_file}: {e}")

# Sort class map into consistent YOLO order
sorted_class_ids = sorted(all_class_map.keys())
id_map = {orig_id: i for i, orig_id in enumerate(sorted_class_ids)}  # remap if class IDs are non-contiguous
class_names = [all_class_map[cid] for cid in sorted_class_ids]

# --- COLLECT ALL IMAGES ---
all_image_info = []
image_index = 0

for source in SOURCE_DIRS:
    rgb_dir = os.path.join(source, "rgb")
    n_images = len([f for f in os.listdir(rgb_dir) if f.endswith(".png")])
    for i in range(n_images):
        all_image_info.append({
            "source": source,
            "index": i,
            "image_id": image_index,
            "file_name": f"{source}_rgb_{i:04d}.png"
        })
        image_index += 1

# --- SPLIT INTO TRAIN AND VAL ---
random.shuffle(all_image_info)
split_idx = int(len(all_image_info) * TRAIN_RATIO)
train_images = all_image_info[:split_idx]
val_images = all_image_info[split_idx:]

# --- PROCESS FUNCTION ---
def process_image(info, split_name):
    source = info["source"]
    idx = info["index"]
    image_name = f"{source}_rgb_{idx:04d}.png"

    rgb_path = os.path.join(source, "rgb", f"rgb_{idx:04d}.png")
    npy_path = os.path.join(source, "bounding_box_2d_tight", f"bounding_box_2d_tight_{idx:04d}.npy")

    if not os.path.exists(rgb_path) or not os.path.exists(npy_path):
        return

    # Load image for dimensions
    img = Image.open(rgb_path)
    width, height = img.size

    # Load bounding boxes
    bboxes = np.load(npy_path)

    label_lines = []
    for box in bboxes:
        class_id, x_min, y_min, x_max, y_max, _ = box  # ignore confidence
        class_id = id_map[int(class_id)]  # remap if needed

        # Convert to YOLO format
        bbox_width = x_max - x_min
        bbox_height = y_max - y_min
        x_center = x_min + bbox_width / 2.0
        y_center = y_min + bbox_height / 2.0

        # Normalize
        x_center /= width
        y_center /= height
        bbox_width /= width
        bbox_height /= height

        label_lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {bbox_width:.6f} {bbox_height:.6f}")

    # Write label file
    label_dir = os.path.join(OUTPUT_DIR, "labels", split_name)
    os.makedirs(label_dir, exist_ok=True)
    label_path = os.path.join(label_dir, image_name.replace(".png", ".txt"))
    with open(label_path, 'w') as f:
        f.write("\n".join(label_lines))

    # Copy image
    img_dest_dir = os.path.join(OUTPUT_DIR, "images", split_name)
    os.makedirs(img_dest_dir, exist_ok=True)
    shutil.copy(rgb_path, os.path.join(img_dest_dir, image_name))

# --- PROCESS TRAIN AND VAL SPLITS ---
print("📦 Processing training images...")
for info in tqdm(train_images):
    process_image(info, "train")

print("📦 Processing validation images...")
for info in tqdm(val_images):
    process_image(info, "val")

# --- WRITE DATASET.YAML FOR YOLO ---
yaml_path = os.path.join(OUTPUT_DIR, "dataset.yaml")
with open(yaml_path, 'w') as f:
    f.write(f"path: {OUTPUT_DIR}\n")
    f.write("train: images/train\n")
    f.write("val: images/val\n\n")
    f.write(f"nc: {len(class_names)}\n")
    f.write("names: [")
    f.write(", ".join([f"'{name}'" for name in class_names]))
    f.write("]\n")

print("✅ YOLO dataset ready at:", OUTPUT_DIR)
