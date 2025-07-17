import os
import json
import random
import shutil
import numpy as np
from PIL import Image
from tqdm import tqdm

# --- CONFIGURATION ---
SOURCE_DIRS = ["left", "right"]
OUTPUT_DIR = "coco_dataset"
TRAIN_RATIO = 0.8

# --- BUILD MASTER CLASS LIST FROM ALL LABEL FILES ---
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
            else:
                print(f"⚠️ Skipping {label_file} (not a dict, found {type(data)})")

        except Exception as e:
            print(f"❌ Error reading {label_file}: {e}")


# Sort and map to COCO-style categories
categories = []
for cid in sorted(all_class_map.keys()):
    categories.append({"id": cid, "name": all_class_map[cid]})

# --- COLLECT ALL IMAGES ---
all_image_info = []
image_index = 0  # unique global ID for COCO

for source in SOURCE_DIRS:
    rgb_dir = os.path.join(source, "rgb")
    n_images = len([f for f in os.listdir(rgb_dir) if f.endswith(".png")])
    for i in range(n_images):
        all_image_info.append({
            "source": source,
            "index": i,
            "image_id": image_index,
            "file_name": f"{source}_rgb_{i:04d}.png"  # namespacing to avoid file name collision
        })
        image_index += 1

# --- SHUFFLE AND SPLIT ---
random.shuffle(all_image_info)
split_idx = int(len(all_image_info) * TRAIN_RATIO)
train_images = all_image_info[:split_idx]
val_images = all_image_info[split_idx:]

# --- INIT COCO STRUCTURE ---
def coco_template():
    return {
        "images": [],
        "annotations": [],
        "categories": categories
    }

train_json = coco_template()
val_json = coco_template()
ann_id = 1  # global annotation ID

# --- HELPER TO PROCESS IMAGES ---
def process_image(info, split_json, split_name):
    global ann_id

    source = info["source"]
    idx = info["index"]
    image_id = info["image_id"]
    image_name = f"{source}_rgb_{idx:04d}.png"

    rgb_path = os.path.join(source, "rgb", f"rgb_{idx:04d}.png")
    npy_path = os.path.join(source, "bounding_box_2d_tight", f"bounding_box_2d_tight_{idx:04d}.npy")

    if not os.path.exists(rgb_path) or not os.path.exists(npy_path):
        return

    img = Image.open(rgb_path)
    width, height = img.size

    split_json["images"].append({
        "id": image_id,
        "file_name": image_name,
        "width": width,
        "height": height
    })

    bboxes = np.load(npy_path)

    for box in bboxes:
        class_id, x_min, y_min, x_max, y_max, conf = box
        class_id = int(class_id)
        bbox_width = x_max - x_min
        bbox_height = y_max - y_min
        area = bbox_width * bbox_height

        annotation = {
            "id": ann_id,
            "image_id": image_id,
            "category_id": class_id,
            "bbox": [float(x_min), float(y_min), float(bbox_width), float(bbox_height)],
            "area": float(area),
            "iscrowd": 0
        }
        split_json["annotations"].append(annotation)
        ann_id += 1

    # Copy image to split folder
    dest_dir = os.path.join(OUTPUT_DIR, "images", split_name)
    os.makedirs(dest_dir, exist_ok=True)
    shutil.copy(rgb_path, os.path.join(dest_dir, image_name))

# --- PROCESS TRAINING IMAGES ---
print("📦 Processing training images...")
for info in tqdm(train_images):
    process_image(info, train_json, "train")

# --- PROCESS VALIDATION IMAGES ---
print("📦 Processing validation images...")
for info in tqdm(val_images):
    process_image(info, val_json, "val")

# --- SAVE ANNOTATIONS ---
os.makedirs(os.path.join(OUTPUT_DIR, "annotations"), exist_ok=True)

with open(os.path.join(OUTPUT_DIR, "annotations", "instances_train.json"), 'w') as f:
    json.dump(train_json, f)

with open(os.path.join(OUTPUT_DIR, "annotations", "instances_val.json"), 'w') as f:
    json.dump(val_json, f)

# --- WRITE YOLO dataset.yaml ---
yaml_path = os.path.join(OUTPUT_DIR, "dataset.yaml")
with open(yaml_path, 'w') as f:
    f.write(f"path: {OUTPUT_DIR}\n")
    f.write("train: images/train\n")
    f.write("val: images/val\n\n")
    f.write(f"nc: {len(categories)}\n")
    f.write("names: [")
    f.write(", ".join([f"'{cat['name']}'" for cat in categories]))
    f.write("]\n")

print("✅ Combined COCO dataset and YOLO config saved to:", OUTPUT_DIR)
