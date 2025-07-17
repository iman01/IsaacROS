#Generates some images with bounding boxes based on COCO dataset

import os
import json
import random
from PIL import Image, ImageDraw, ImageFont
from tqdm import tqdm

# --- CONFIG ---
DATASET_DIR = "coco_dataset"
ANNOTATION_FILE = "annotations/instances_val.json"  # Change to instances_train.json if needed
IMG_FOLDER = os.path.join(DATASET_DIR, "images", "val")  # or "train"
OUTPUT_FOLDER = os.path.join(DATASET_DIR, "vis")
NUM_IMAGES = 10  # How many to visualize

COLORS = { #blue is default
    "maize" : "green",
    "weed" : "red"
}

os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# --- LOAD ANNOTATIONS ---
with open(os.path.join(DATASET_DIR, ANNOTATION_FILE), "r") as f:
    coco = json.load(f)

images = {img["id"]: img for img in coco["images"]}
annotations = coco["annotations"]
categories = {cat["id"]: cat["name"] for cat in coco["categories"]}

# --- GROUP ANNOTATIONS BY IMAGE ---
ann_by_image = {}
for ann in annotations:
    img_id = ann["image_id"]
    if img_id not in ann_by_image:
        ann_by_image[img_id] = []
    ann_by_image[img_id].append(ann)

# --- VISUALIZE ---
print(f"🔍 Visualizing {NUM_IMAGES} random images...")

for img_id in random.sample(list(images.keys()), min(NUM_IMAGES, len(images))):
    img_info = images[img_id]
    img_path = os.path.join(IMG_FOLDER, img_info["file_name"])
    image = Image.open(img_path).convert("RGB")
    draw = ImageDraw.Draw(image)

    # Optional: use font (might not work in headless environments)
    try:
        font = ImageFont.truetype("arial.ttf", 14)
    except:
        font = ImageFont.load_default()

    for ann in ann_by_image.get(img_id, []):
        x, y, w, h = ann["bbox"]
        class_id = ann["category_id"]
        class_name = categories[class_id]
        
        frame_color = ""
        if class_name in COLORS:
            frame_color = COLORS[class_name]
        else:
            frame_color = "blue"



        # Draw box
        draw.rectangle([(x, y), (x + w, y + h)], outline=frame_color, width=2)
        draw.text((x, y - 10), class_name, fill=frame_color, font=font)

    # Save result
    out_path = os.path.join(OUTPUT_FOLDER, f"vis_{img_info['file_name']}")
    image.save(out_path)

print(f"✅ Done. Check {OUTPUT_FOLDER} for visualized images.")
