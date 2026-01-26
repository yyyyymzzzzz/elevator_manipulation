import os
import shutil
import random
import yaml
from pathlib import Path

def find_image_file(image_root_dirs, base_filename):
    """Recursively find an image file matching the base_filename in a list of directories."""
    for image_root_dir in image_root_dirs:
        for root, _, files in os.walk(image_root_dir):
            for file in files:
                if Path(file).stem == base_filename:
                    # Check for common image extensions
                    if file.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.webp')):
                        return os.path.join(root, file)
    return None

def prepare_yolo_dataset(base_dir, label_dirs, image_dirs, val_split=0.2):
    """
    Prepares a YOLO dataset by matching labels and images, splitting them
    into training and validation sets, and creating a data.yaml file.
    Handles merging new data into an existing dataset.
    """
    # Define paths
    output_dir = os.path.join(base_dir, 'xiazhi_dataset_v3')   # !!!请修改为实际需要导出的目录

    # Create output directories
    paths = {
        'train_images': os.path.join(output_dir, 'images', 'train'),
        'val_images': os.path.join(output_dir, 'images', 'val'),
        'train_labels': os.path.join(output_dir, 'labels', 'train'),
        'val_labels': os.path.join(output_dir, 'labels', 'val')
    }
    for path in paths.values():
        os.makedirs(path, exist_ok=True)

    # --- Class Name Merging ---
    all_class_names = []
    existing_yaml_path = os.path.join(output_dir, 'elevator.yaml')
    if os.path.exists(existing_yaml_path):
        with open(existing_yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            if 'names' in data:
                all_class_names.extend(data['names'])

    for label_dir in label_dirs:
        classes_file_path = os.path.join(label_dir, 'classes.txt')
        if os.path.isfile(classes_file_path):
            with open(classes_file_path, 'r') as f:
                new_names = [line.strip() for line in f.readlines() if line.strip()]
                for name in new_names:
                    if name not in all_class_names:
                        all_class_names.append(name)
        else:
            print(f"Warning: classes.txt not found in '{label_dir}'")

    if not all_class_names:
        print("Error: No class names found in any of the provided label directories.")
        return

    # --- Identify New Label Files ---
    existing_labels = set()
    for split in ['train', 'val']:
        label_path = paths[f'{split}_labels']
        if os.path.isdir(label_path):
            for f in os.listdir(label_path):
                existing_labels.add(Path(f).stem)

    new_label_files = []
    for label_dir in label_dirs:
        if not os.path.isdir(label_dir):
            print(f"Warning: Label source directory not found at '{label_dir}'")
            continue
        for f in os.listdir(label_dir):
            if f.endswith('.txt') and f != 'classes.txt':
                if Path(f).stem not in existing_labels:
                    new_label_files.append((label_dir, f))

    if not new_label_files:
        print("No new labels to process.")
    else:
        random.shuffle(new_label_files)
        # Split files
        split_index = int(len(new_label_files) * (1 - val_split))
        train_files = new_label_files[:split_index]
        val_files = new_label_files[split_index:]

        print(f"Found {len(new_label_files)} new labels. Splitting into {len(train_files)} train and {len(val_files)} val.")

        # Process files
        for split_name, file_list in [('train', train_files), ('val', val_files)]:
            image_dest_dir = paths[f'{split_name}_images']
            label_dest_dir = paths[f'{split_name}_labels']
            
            for label_source_dir, label_filename in file_list:
                base_filename = Path(label_filename).stem
                
                # Find corresponding image file
                image_path = find_image_file(image_dirs, base_filename)
                label_path = os.path.join(label_source_dir, label_filename)

                if image_path:
                    # Copy image and label files
                    shutil.copy(image_path, os.path.join(image_dest_dir, os.path.basename(image_path)))
                    shutil.copy(label_path, os.path.join(label_dest_dir, label_filename))
                else:
                    print(f"Warning: No matching image found for label '{label_filename}'")

    # Create/Update data.yaml file
    data_yaml = {
        'train': os.path.relpath(paths['train_images'], output_dir),
        'val': os.path.relpath(paths['val_images'], output_dir),
        'nc': len(all_class_names),
        'names': all_class_names
    }

    with open(existing_yaml_path, 'w') as f:
        yaml.dump(data_yaml, f, sort_keys=False)

    print(f"\nDataset preparation complete. Output saved to '{output_dir}'")
    print(f"YOLO config file updated at '{existing_yaml_path}'")


if __name__ == '__main__':
    # The script is expected to be in /home/ymz/Workspace/data/
    current_script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # --- Specify all label and image directories here ---
    # Add new label directories to this list to merge them.
    label_directories = [
        # os.path.join(current_script_dir, 'elevator_label'),
        os.path.join(current_script_dir, 'xiazhi_v2')   # !!!请修改为实际标注label的目录
    ]
    
    # Add corresponding image directories.
    image_directories = [
        os.path.join(current_script_dir, '20260124_150653')   # !!!请修改为实际对应image的目录
    ]

    prepare_yolo_dataset(
        base_dir=current_script_dir,
        label_dirs=label_directories,
        image_dirs=image_directories
    )