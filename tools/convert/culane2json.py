#!/usr/bin/env python3
"""
Convert CULane .lines.txt labels into per-frame JSON understood by
Ransac-Line-Detector.

Usage examples
--------------
# 1) полный CULane (в корне есть list/train_gt.txt)
python culane2json.py --culane_root /path/to/culane --out /tmp/json

# 2) mini-CULane без списка
python culane2json.py --culane_root . --out ../culane/train

# 3) свой список файлов
find driver_* -name '*.lines.txt' > my_list.txt
python culane2json.py --culane_root . --list_file my_list.txt --out ../culane/train
"""
import argparse, json, cv2, os, sys
from pathlib import Path
from tqdm import tqdm

def load_list(root: Path, list_file: str | None):
    if list_file:
        txt = Path(list_file)
    else:
        txt = root / "list/train_gt.txt"
    if txt.exists():
        with txt.open() as f:
            return [line.strip() for line in f]
    # fallback: рекурсивный поиск .lines.txt
    return [str(p.relative_to(root)) for p in root.rglob("*.lines.txt")]

def convert_one(root: Path, rel_path: str, out_frames: Path, out_labels: Path):
    lbl_path = root / rel_path
    # путь к изображению: заменяем segments→driver и .lines.txt→.jpg
    img_path = root / rel_path.replace("laneseg_label_w16", "driver").replace(".lines.txt", ".jpg")
    if not img_path.exists():
        return
    lanes = []
    with lbl_path.open() as f:
        for line in f:
            nums = [round(float(v)) for v in line.split()]
            xs, ys = nums[::2], nums[1::2]
            pts = [[x, y] for x, y in zip(xs, ys) if x >= 0]
            if len(pts) >= 2:
                lanes.append({"points": pts, "isSolid": False})
    stem = img_path.stem
    (out_labels / f"{stem}.json").write_text(json.dumps({"lanes": lanes}))
    dst_img = out_frames / f"{stem}.jpg"
    if not dst_img.exists():
        cv2.imwrite(str(dst_img), cv2.imread(str(img_path)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--culane_root", required=True)
    ap.add_argument("--out",        required=True)
    ap.add_argument("--list_file",  help="optional txt file with list of *.lines.txt paths")
    args = ap.parse_args()

    root = Path(args.culane_root).resolve()
    out_frames = Path(args.out) / "frames"
    out_labels = Path(args.out) / "labels"
    out_frames.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    rel_list = load_list(root, args.list_file)
    if not rel_list:
        print("No .lines.txt found!", file=sys.stderr); sys.exit(1)

    for rel in tqdm(rel_list, desc="Converting"):
        convert_one(root, rel, out_frames, out_labels)

if __name__ == "__main__":
    main()
