from pathlib import Path
import shutil

DATASET_ROOT = Path("/home/noh/dataset")

IMAGE_TRAIN_DIR = DATASET_ROOT / "images" / "train"
IMAGE_VAL_DIR   = DATASET_ROOT / "images" / "val"

LABEL_TRAIN_DIR = DATASET_ROOT / "labels" / "train"
LABEL_VAL_DIR   = DATASET_ROOT / "labels" / "val"

VAL_RATIO = 0.15
DRY_RUN = False   # True로 먼저 테스트, 실제 이동하려면 False

IMAGE_EXT = ".jpg"

# YOLO txt 라벨이면 .txt만 쓰면 됨
# labelImg에서 PascalVOC xml로 저장했다면 ".xml"도 추가 가능
LABEL_EXTS = [".txt"]


RANGES = [
    (3785, 4086), (4856, 4984), (3705, 3724), (4691, 4705),
    (4198, 4290), (4767, 4808), (3725, 3744), (4706, 4710),
    (4087, 4197), (4722, 4766), (3770, 3784), (4717, 4721),
    (4291, 4431), (4809, 4855), (3745, 3769), (4711, 4716),
    (3443, 3704), (4511, 4618), (3270, 3442), (4619, 4690),
    (2643, 2791), (2792, 2864), (2545, 2642), (2865, 2906),
    (3122, 3269), (2949, 3011), (3012, 3121), (2907, 2948),
    (2085, 2544), (1670, 1832), (1833, 2084), (1562, 1669),
    (655, 1287),  (1288, 1561), (236, 654),   (41, 235),
]


def pick_evenly_spaced_numbers(start: int, end: int, ratio: float):
    """
    start~end 번호 중 ratio 비율만큼 균등 간격으로 선택.
    랜덤 X.
    """
    numbers = list(range(start, end + 1))
    total = len(numbers)

    n_val = max(1, round(total * ratio))

    if n_val >= total:
        return numbers

    selected = []

    # 구간을 n_val개로 나누고 각 구간 중앙 근처 번호를 선택
    for i in range(n_val):
        idx = round((i + 0.5) * total / n_val - 0.5)
        idx = max(0, min(idx, total - 1))
        selected.append(numbers[idx])

    return selected


def make_image_filename(number: int):
    """
    number=1    -> img_0001.jpg
    number=41   -> img_0041.jpg
    number=655  -> img_0655.jpg
    number=3785 -> img_3785.jpg
    """
    return f"img_{number:04d}{IMAGE_EXT}"


def find_image_by_number(number: int):
    """
    img_0001.jpg 형식의 파일을 정확히 찾음.
    """
    filename = make_image_filename(number)
    image_path = IMAGE_TRAIN_DIR / filename

    if image_path.exists():
        return image_path

    return None


def move_file(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)

    if DRY_RUN:
        print(f"[DRY-RUN] {src}  ->  {dst}")
    else:
        shutil.move(str(src), str(dst))


def main():
    IMAGE_VAL_DIR.mkdir(parents=True, exist_ok=True)
    LABEL_VAL_DIR.mkdir(parents=True, exist_ok=True)

    total_selected = 0
    total_moved_images = 0
    total_missing_images = 0
    total_moved_labels = 0

    selected_all = []

    print("========== 구간별 val 선택 ==========")

    for start, end in RANGES:
        selected = pick_evenly_spaced_numbers(start, end, VAL_RATIO)
        selected_all.extend(selected)

        print(
            f"{start}-{end}: 전체 {end - start + 1}장 중 "
            f"{len(selected)}장 val 선택"
        )

    print("\n========== 이동 시작 ==========")

    # 혹시 겹치는 번호가 있을 경우 중복 제거
    selected_all = sorted(set(selected_all))
    total_selected = len(selected_all)

    for number in selected_all:
        image_path = find_image_by_number(number)

        if image_path is None:
            print(f"[이미지 없음] number={number}, expected={make_image_filename(number)}")
            total_missing_images += 1
            continue

        dst_image_path = IMAGE_VAL_DIR / image_path.name
        move_file(image_path, dst_image_path)
        total_moved_images += 1

        # 라벨 파일도 있으면 같이 이동
        for label_ext in LABEL_EXTS:
            label_path = LABEL_TRAIN_DIR / f"{image_path.stem}{label_ext}"

            if label_path.exists():
                dst_label_path = LABEL_VAL_DIR / label_path.name
                move_file(label_path, dst_label_path)
                total_moved_labels += 1

    print("\n========== 결과 ==========")
    print(f"선택된 val 번호 수: {total_selected}")
    print(f"이동 대상 이미지 수: {total_moved_images}")
    print(f"없는 이미지 수: {total_missing_images}")
    print(f"같이 이동한 라벨 수: {total_moved_labels}")

    if DRY_RUN:
        print("\n현재는 DRY_RUN=True라 실제 이동하지 않았습니다.")
        print("출력 확인 후 DRY_RUN=False로 바꾸고 다시 실행하세요.")


if __name__ == "__main__":
    main()