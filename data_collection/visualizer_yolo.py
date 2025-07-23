#Generates some images with bounding boxes based on YOLO dataset
import os
import random
from PIL import Image, ImageDraw, ImageFont
from tqdm import tqdm
import yaml

# --- CONFIG ---
DATASET_DIR = "yolo_dataset"
SPLIT = "val"  # or "train"
IMG_FOLDER = os.path.join(DATASET_DIR, "images", SPLIT)
LABEL_FOLDER = os.path.join(DATASET_DIR, "labels", SPLIT)
YAML_PATH = os.path.join(DATASET_DIR, "dataset.yaml")
OUTPUT_FOLDER = os.path.join(DATASET_DIR, "vis")
NUM_IMAGES = 10

COLORS = {
    "maize": "green",
    "weed": "red"
}

os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# --- LOAD CLASS NAMES ---
with open(YAML_PATH, "r") as f:
    yml = yaml.safe_load(f)

class_names = yml["names"]
nc = yml["nc"]

# --- GET IMAGE FILES ---
image_files = [f for f in os.listdir(IMG_FOLDER) if f.lower().endswith(".png")]
image_files = random.sample(image_files, min(NUM_IMAGES, len(image_files)))

# --- VISUALIZE ---
print(f"🔍 Visualizing {len(image_files)} images from {SPLIT} set...")

for image_file in tqdm(image_files):
    img_path = os.path.join(IMG_FOLDER, image_file)
    label_path = os.path.join(LABEL_FOLDER, image_file.replace(".png", ".txt"))

    image = Image.open(img_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    width, height = image.size

    try:
        font = ImageFont.truetype("arial.ttf", 14)
    except:
        font = ImageFont.load_default()

    if os.path.exists(label_path):
        with open(label_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 5:
                    continue
                class_id, x_center, y_center, box_w, box_h = map(float, parts)
                class_id = int(class_id)
                class_name = class_names[class_id]

                # Convert back to absolute coordinates
                xc = x_center * width
                yc = y_center * height
                bw = box_w * width
                bh = box_h * height
                x0 = xc - bw / 2
                y0 = yc - bh / 2
                x1 = xc + bw / 2
                y1 = yc + bh / 2

                color = COLORS.get(class_name, "blue")
                draw.rectangle([(x0, y0), (x1, y1)], outline=color, width=2)
                draw.text((x0, y0 - 10), class_name, fill=color, font=font)

    # Save output image
    out_path = os.path.join(OUTPUT_FOLDER, f"vis_{image_file}")
    image.save(out_path)

print(f"✅ Done. Check {OUTPUT_FOLDER} for visualized images.")
