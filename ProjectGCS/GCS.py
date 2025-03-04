import sys
import numpy as np
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout
from PyQt5.QtGui import QPainter, QPen, QColor, QFont
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from dronekit import connect, VehicleMode

class DroneHorizon(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ground Control System v1")
        self.resize(800, 600)
        
        self.pitch = 0
        self.roll = 0
        self.heading = 0
        self.mode = "DISARMED"
        self.gps_status = "GPS Tidak Siap"
        
        print("Connecting to drone...")
        self.vehicle = connect("udp:127.0.0.1:14550", wait_ready=True)
        print("Drone connected!")
        
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.status_label = QLabel("")
        self.layout.addWidget(self.status_label)

        self.button_layout = QHBoxLayout()
        self.arm_button = QPushButton("Arm")
        self.disarm_button = QPushButton("Disarm")
        self.rtl_button = QPushButton("RTL")
        self.land_button = QPushButton("LAND")

        self.button_layout.addWidget(self.arm_button)
        self.button_layout.addWidget(self.disarm_button)
        self.button_layout.addWidget(self.rtl_button)
        self.button_layout.addWidget(self.land_button)
        self.layout.addLayout(self.button_layout)

        self.arm_button.clicked.connect(self.arm_drone)
        self.disarm_button.clicked.connect(self.disarm_drone)
        self.rtl_button.clicked.connect(self.return_to_launch)
        self.land_button.clicked.connect(self.land_drone)
        
        self.data_thread = DroneDataThread(self.vehicle)
        self.data_thread.data_updated.connect(self.update_data)
        self.data_thread.start()

    def arm_drone(self):
        self.status_label.setText("Status: Arming...")  # Tampilkan status awal
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        #self.status_label.setText("Status: Arming...")
        # Gunakan QTimer untuk mengecek apakah drone benar-benar armed
        QTimer.singleShot(2000, self.check_arming_status)

    def check_arming_status(self):
        if self.vehicle.armed:
            self.status_label.setText("Status: Armed")
        else:
            self.status_label.setText("Status: Failed to Arm")
            
    def disarm_drone(self):
        self.status_label.setText("Status: Disarming...")  # Tampilkan status awal
        self.vehicle.armed = False
        #self.status_label.setText("Status: Disarming...")
        # Gunakan QTimer untuk mengecek apakah drone benar-benar disarmed
        QTimer.singleShot(2000, self.check_disarming_status)

    def check_disarming_status(self):
        if not self.vehicle.armed:
            self.status_label.setText("Status: Disarmed")
        else:
            self.status_label.setText("Status: Failed to Disarm")
            
    def return_to_launch(self):
        self.vehicle.mode = VehicleMode("RTL")
        #self.status_label.setText("Status: Returning to Launch...")

    def land_drone(self):
        self.vehicle.mode = VehicleMode("LAND")
        #self.status_label.setText("Status: Landing...")
    
    def update_data(self, pitch, roll, altitude, heading, mode, gps_ready):
        self.pitch = pitch
        self.roll = roll
        self.altitude = altitude
        self.heading = heading
        self.mode = mode
        
        if gps_ready:
            self.gps_status = "GPS Siap"
        else:
            self.gps_status = "GPS Tidak Siap"
        
        if self.vehicle.armed:
            self.status_label.setText("Status : Propeller Arming")
            self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 11px;")
        if not self.vehicle.armed:
            self.status_label.setText("Status : Propeller Disarmed ")
            self.status_label.setStyleSheet("color: green; font-weight: bold; font-size: 11px;")
        # Jika drone sudah terbang (altitude > 1 meter), hapus status arming/disarming
        if self.altitude > 1:
            self.status_label.setText("")  # Kosongkan teks status
            
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()
        center_x = width // 2
        center_y = height // 2

        # Latar belakang horizon
        painter.setBrush(QColor(135, 206, 250))
        painter.drawRect(0, 0, width, center_y)
        
        painter.setBrush(QColor(139, 69, 19))
        painter.drawRect(0, center_y, width, center_y)

        # Garis horizon dipengaruhi roll
        roll_angle = np.radians(self.roll)
        horizon_shift = int(np.tan(-roll_angle) * (width // 4))
        
        pen = QPen(Qt.white, 3)
        painter.setPen(pen)
        painter.drawLine(center_x - (width // 2), center_y + horizon_shift, 
                         center_x + (width // 2), center_y - horizon_shift)

        # **Indikator Pitch**
        pitch_offset = int(self.pitch * 5)  # Sensitivitas pitch
        
        # Garis netral hijau
        pen = QPen(Qt.green, 3)
        painter.setPen(pen)
        painter.drawLine(center_x - 50, center_y, center_x + 50, center_y)
        
        # Garis merah menunjukkan pitch
        pen = QPen(Qt.red, 3)
        painter.setPen(pen)
        # Titik tengah garis runcing
        apex_x = center_x  # Titik puncak tetap di tengah
        apex_y = center_y - pitch_offset - 10  # Puncak lebih tinggi dari garis utama

        # Titik kiri dan kanan bawah
        left_x = center_x - 50
        right_x = center_x + 50
        base_y = center_y - pitch_offset  # Sejajar dengan netral

        # Gambar garis meruncing ke atas
        painter.drawLine(left_x, base_y, apex_x, apex_y)   # Garis dari kiri ke puncak
        painter.drawLine(right_x, base_y, apex_x, apex_y)  # Garis dari kanan ke puncak

        #painter.drawLine(center_x - 30, center_y - pitch_offset, center_x + 30, center_y - pitch_offset)
        
        # **Gambar indikator arah mata angin dengan warna dan posisi yang lebih baik**
        compass_x = width - 100
        compass_y = 100
        compass_radius = 50

        painter.setBrush(QColor(50, 50, 50))  # Warna abu-abu untuk background kompas
        painter.setPen(QPen(Qt.white, 2))
        painter.drawEllipse(compass_x - compass_radius, compass_y - compass_radius, compass_radius * 2, compass_radius * 2)

        font = QFont("Arial", 12, QFont.Bold)
        painter.setFont(font)
        painter.setPen(Qt.white)
        directions = [("N", 0, -compass_radius - 10), ("E", compass_radius + 10, 5),
                      ("S", 0, compass_radius + 20), ("W", -compass_radius - 20, 5)]

        for dir_label, dx, dy in directions:
            painter.drawText(compass_x + dx - 5, compass_y + dy, dir_label)

        heading_rad = np.radians(self.heading)
        arrow_x = compass_x + int(compass_radius * np.sin(heading_rad))
        arrow_y = compass_y - int(compass_radius * np.cos(heading_rad))
        painter.setPen(QPen(Qt.red, 3))
        painter.drawLine(compass_x, compass_y, arrow_x, arrow_y)
        
        # **Gambar heading (indikator arah)**
        font = QFont("Arial", 12, QFont.Bold)
        painter.setFont(font)
        painter.setPen(Qt.black)
        painter.drawText(self.width() - 140, 20, f"Heading: {self.heading}°")
        painter.drawText(10, 40, f"Mode: {self.mode}")
        painter.drawText(10, 60, f"Altitude: {self.altitude:.2f} m")
        
        if self.gps_status == "GPS Siap":
            painter.setPen(QColor(0, 128, 0))  # Hijau
            painter.drawText(10, 20, f"GPS: {self.gps_status}")
        else:
            painter.setPen(QColor(255, 0, 0))  # Merah
            painter.drawText(10, 20, f"GPS: {self.gps_status}")
    
    def closeEvent(self, event):
        self.data_thread.stop()
        self.vehicle.close()
        event.accept()

class DroneDataThread(QThread):
    data_updated = pyqtSignal(float, float, float, int, str, bool)

    def __init__(self, vehicle):
        super().__init__()
        self.vehicle = vehicle
        self.running = True

    def run(self):
        while self.running:
            try:
                pitch = self.vehicle.attitude.pitch * (180 / np.pi)
                roll = self.vehicle.attitude.roll * (180 / np.pi)
                altitude = self.vehicle.location.global_relative_frame.alt
                heading = self.vehicle.heading
                mode = self.vehicle.mode.name
                gps_ready = self.vehicle.gps_0.fix_type >= 3  # Fix type >= 3 menandakan GPS siap
                self.data_updated.emit(pitch, roll, altitude, heading, mode, gps_ready)
            except Exception as e:
                print(f"Error fetching drone data: {e}")
            self.msleep(50)

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DroneHorizon()
    window.show()
    sys.exit(app.exec_())

