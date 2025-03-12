import sys
import os
import tensorflow as tf
import numpy as np
import cv2

# ANSI escape codes untuk warna di terminal
RESET = "\033[0m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"

# Daftar label COCO (singkat, tambahkan lebih banyak jika perlu)
coco_labels = {
    1: "Person", 2: "Bicycle", 3: "Car", 4: "Motorcycle", 5: "Airplane",
    6: "Bus", 7: "Train", 8: "Truck", 9: "Boat", 10: "Traffic Light",
    11: "Fire Hydrant", 13: "Stop Sign", 14: "Parking Meter", 15: "Bench",
    16: "Bird", 17: "Cat", 18: "Dog", 19: "Horse", 20: "Sheep", 
    21: "Cow", 22: "Elephant", 23: "Bear", 24: "Zebra", 25: "Giraffe", 
    27: "Backpack", 28: "Umbrella", 31: "Handbag", 32: "Tie", 33: "Suitcase",
    34: "Frisbee", 35: "Skis", 36: "Snowboard", 37: "Sports Ball", 38: "Kite",
    39: "Baseball Bat", 40: "Baseball Glove", 41: "Skateboard", 42: "Surfboard",
    43: "Tennis Racket", 44: "Bottle", 46: "Wine Glass", 47: "Cup", 48: "Fork",
    49: "Knife", 50: "Spoon", 51: "Bowl", 52: "Banana", 53: "Apple", 
    54: "Sandwich", 55: "Orange", 56: "Broccoli", 57: "Carrot", 58: "Hot Dog",
    59: "Pizza", 60: "Donut", 61: "Cake", 62: "Chair", 63: "Couch", 
    64: "Potted Plant", 65: "Bed", 67: "Dining Table", 70: "Toilet", 
    72: "TV", 73: "Laptop", 74: "Mouse", 75: "Remote", 76: "Keyboard", 
    77: "Cell Phone", 78: "Microwave", 79: "Oven", 80: "Toaster", 
    81: "Sink", 82: "Refrigerator", 84: "Book", 85: "Clock", 86: "Vase", 
    87: "Scissors", 88: "Teddy Bear", 89: "Hair Drier", 90: "Toothbrush"
}

# Fungsi untuk menentukan warna tetap berdasarkan kelas
def get_color_for_class(class_id):
    np.random.seed(class_id)  # Tetapkan seed agar warna tetap
    return tuple(np.random.randint(0, 255, 3).tolist())  # Warna RGB acak tapi tetap

# Load model
model_path = "mobilenet_ssd/saved_model"
model = tf.saved_model.load(model_path)
infer = model.signatures["serving_default"]

# Folder untuk menyimpan frame hasil deteksi
output_folder = "output_detection"
os.makedirs(output_folder, exist_ok=True)

# Fungsi untuk deteksi objek
def detect_objects(video_path):
    cap = cv2.VideoCapture(video_path)
    frame_count = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break  # Hentikan jika tidak ada frame lagi
            
        h, w, _ = frame.shape
        img_resized = cv2.resize(frame, (320, 320))

        input_tensor = tf.convert_to_tensor([img_resized], dtype=tf.uint8)
        detections = infer(input_tensor)

        # Ambil hasil deteksi
        boxes = detections['detection_boxes'][0].numpy()
        scores = detections['detection_scores'][0].numpy()
        classes = detections['detection_classes'][0].numpy().astype(int)

        for i in range(len(scores)):
            if scores[i] > 0.3:  # Tampilkan hanya deteksi dengan skor > 30%
                class_id = classes[i]
                class_name = coco_labels.get(class_id, f"Unknown ({class_id})")
                y1, x1, y2, x2 = boxes[i]
                y1, x1, y2, x2 = int(y1 * h), int(x1 * w), int(y2 * h), int(x2 * w)

                # Pilih warna untuk tiap kelas
                color = get_color_for_class(class_id)

                # Gambar bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # Tambahkan teks label di atas kotak
                label = f"{class_name} ({scores[i]:.2f})"
                cv2.putText(frame, label, (x1, y1 - 10 if y1 - 10 > 10 else y1 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Cetak output di terminal dengan warna
                print(
                    f"{CYAN}Deteksi: {RESET}"
                    f"{GREEN}Kelas={class_name}{RESET}, "
                    f"{YELLOW}Skor={scores[i]:.2f}{RESET}, "
                    f"{MAGENTA}Kotak={[x1, y1, x2, y2]}{RESET}"
                )

        # Simpan hasil deteksi
        output_frame_path = os.path.join(output_folder, f"frame_{frame_count:04d}.jpg")
        cv2.imwrite(output_frame_path, frame)
        frame_count += 1

        print(f"{BLUE}Frame {frame_count} diproses dan disimpan sebagai {output_frame_path}{RESET}")

    cap.release()
    print("\n🔹 Semua frame telah diproses dan disimpan di folder output_frames")

# Jalankan deteksi pada video
detect_objects("input_video.mp4")
