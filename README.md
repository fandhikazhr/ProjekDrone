# 🚀 Autonomous Drone System

Selamat datang di **Autonomous Drone System**, repositori ini berisi berbagai proyek terkait drone otonom, deteksi objek, dan ground control station (GCS). Tujuan utama proyek ini adalah mengembangkan ekosistem drone yang dapat beroperasi secara mandiri dengan kemampuan navigasi cerdas dan deteksi visual.

## ✈️ Fitur Utama

### 1️⃣ **Autonomous Navigation**
- Implementasi sistem navigasi otonom untuk drone menggunakan ROS dan MAVROS.
- Simulasi penerbangan di Gazebo dengan kontrol penuh menggunakan Mavlink.
- Pengujian obstacle avoidance menggunakan kamera RealSense D435.

### 2️⃣ **Object Detection & Tracking**
- Model deteksi berbasis **YOLOv5** dan **MobileNet SSD** untuk identifikasi objek.
- Penggunaan model yang dioptimalkan agar berjalan lancar di **Raspberry Pi**.
- Implementasi tracking untuk mengikuti atau menghindari objek tertentu.

### 3️⃣ **Ground Control Station (GCS) Web-based**
- Pengembangan GCS berbasis web untuk memonitor dan mengontrol drone secara real-time.
- Komunikasi langsung dengan simulasi drone menggunakan WebSockets atau MAVSDK.
- Visualisasi peta dan jalur penerbangan secara interaktif.

## 🛠️ Teknologi yang Digunakan
- **ROS & MAVROS** → Komunikasi dan kontrol drone.
- **Gazebo** → Simulasi lingkungan penerbangan.
- **YOLOv5 & MobileNet SSD** → Deteksi dan pelacakan objek.
- **RealSense D435** → Sensor kedalaman untuk obstacle avoidance.
- **Web-based GCS** → Pengembangan dashboard untuk pemantauan drone.

## 🔥 Cara Menggunakan
1. Clone repositori ini:
   ```bash
   git clone https://github.com/username/autonomous-drone.git
   cd autonomous-drone
   ```
2. Jalankan simulasi di Gazebo:
   ```bash
   roslaunch autonomous_drone simulation.launch
   ```
3. Aktifkan deteksi objek:
   ```bash
   python object_detection.py
   ```
4. Buka GCS Web di browser untuk mengontrol drone secara interaktif.

## 🎯 Roadmap Pengembangan
✅ Simulasi drone otonom di Gazebo  
✅ Implementasi YOLOv5 untuk deteksi objek  
✅ Optimasi model untuk Raspberry Pi  
⏳ Pengembangan obstacle avoidance dengan RealSense D435  
⏳ Pembuatan UI GCS berbasis web  

## 🤝 Kontribusi
Terbuka untuk kontribusi! Jika ingin menambahkan fitur atau meningkatkan proyek ini, silakan buat **pull request** atau diskusi di **issues**.

## 📜 Lisensi
Proyek ini menggunakan lisensi **MIT**, silakan gunakan dan kembangkan lebih lanjut.

🚀 **Let's build the future of autonomous drones together!**
