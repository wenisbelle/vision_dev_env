#from ultralytics.data.converter import convert_dota_to_yolo_obb
from ultralytics.utils import TQDM
from pathlib import Path
import cv2

def convert_to_yolo_obb(dota_root_path: str):
    """
    Converts dataset annotations to YOLO OBB (Oriented Bounding Box) format.

    The function processes images in the 'train' and 'val' folders of the dataset. For each image, it reads the
    associated label from the original labels directory and writes new labels in YOLO OBB format to a new directory.

    Notes:
        The directory structure assumed for the dataset:

            - dataset
                ├─ images
                │   ├─ train
                │   └─ val
                └─ labels
                    ├─ train_original
                    └─ val_original

        After execution, the function will organize the labels into:

            - dataset
                └─ labels
                    ├─ train
                    └─ val
    """
    dota_root_path = Path(dota_root_path)

    # Class names to indices mapping
    class_mapping = {
        "ammunition": 0,
    }

    def convert_label(image_name, image_width, image_height, orig_label_dir, save_dir):
        """Converts a single image's DOTA annotation to YOLO OBB format and saves it to a specified directory."""
        orig_label_path = orig_label_dir / f"{image_name}.txt"
        save_path = save_dir / f"{image_name}.txt"

        with orig_label_path.open("r") as f, save_path.open("w") as g:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split()
                if len(parts) < 9:
                    continue
                class_name = parts[8]
                class_idx = class_mapping[class_name]
                coords = [float(p) for p in parts[:8]]
                normalized_coords = [
                    coords[i] / image_width if i % 2 == 0 else coords[i] / image_height for i in range(8)
                ]
                formatted_coords = ["{:.6g}".format(coord) for coord in normalized_coords]
                g.write(f"{class_idx} {' '.join(formatted_coords)}\n")

    for phase in ["train", "val"]:
        image_dir = dota_root_path / "images" / phase
        orig_label_dir = dota_root_path / "labels" / f"{phase}_original"
        save_dir = dota_root_path / "labels" / phase

        save_dir.mkdir(parents=True, exist_ok=True)

        image_paths = list(image_dir.iterdir())
        for image_path in TQDM(image_paths, desc=f"Processing {phase} images"):
            if image_path.suffix != ".png":
                continue
            image_name_without_ext = image_path.stem
            img = cv2.imread(str(image_path))
            h, w = img.shape[:2]
            convert_label(image_name_without_ext, w, h, orig_label_dir, save_dir)


if __name__ == '__main__':

    convert_to_yolo_obb("./datasets/mortar_ammunition/")