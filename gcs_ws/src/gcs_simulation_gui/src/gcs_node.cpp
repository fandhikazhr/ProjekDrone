#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QPainter>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <cmath>

class GCSGUI : public QWidget {
    QLabel *altitudeLabel;
    QLabel *modeLabel;
    QLabel *rollPitchLabel;
    double roll, pitch, yaw, altitude;
    std::string flight_mode;

public:
    GCSGUI(QWidget *parent = nullptr) : QWidget(parent), roll(0), pitch(0), yaw(0), altitude(0) {
        QVBoxLayout *layout = new QVBoxLayout(this);
        altitudeLabel = new QLabel("Altitude: 0.0 m", this);
        modeLabel = new QLabel("Mode: Unknown", this);
        rollPitchLabel = new QLabel("Roll: 0.0 | Pitch: 0.0", this);

        layout->addWidget(modeLabel);
        layout->addWidget(altitudeLabel);
        layout->addWidget(rollPitchLabel);
    }

    void updateAltitude(double alt) {
        altitude = alt;
        altitudeLabel->setText(QString("Altitude: %1 m").arg(altitude));
    }

    void updateMode(const std::string &mode) {
        flight_mode = mode;
        modeLabel->setText(QString("Mode: %1").arg(QString::fromStdString(flight_mode)));
    }

    void updateRollPitchYaw(double r, double p, double y) {
        roll = r;
        pitch = p;
        yaw = y;
        rollPitchLabel->setText(QString("Roll: %1 | Pitch: %2 | Yaw: %3").arg(roll).arg(pitch).arg(yaw));
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // Background langit dan ground
        painter.fillRect(0, 0, width(), height() / 2, Qt::blue);
        painter.fillRect(0, height() / 2, width(), height() / 2, Qt::darkGreen);
        
        int centerX = width() / 2;
        int centerY = height() / 2;
        int lineLength = 80;

        // Garis netral hijau
        painter.setPen(QPen(Qt::green, 3));
        painter.drawLine(centerX - lineLength, centerY, centerX + lineLength, centerY);
        
        // Garis indikator roll (putih melintang dengan sudut sesuai roll)
        int rollOffset = static_cast<int>(roll * 10);
        int rollYShift = static_cast<int>(roll * 5);
        painter.setPen(QPen(Qt::white, 3));
        painter.drawLine(centerX - lineLength + rollOffset, centerY - rollYShift, centerX + lineLength + rollOffset, centerY + rollYShift);
        
        // Garis indikator pitch (segitiga lancip)
        int pitchOffset = static_cast<int>(pitch * 5);
        painter.setPen(QPen(Qt::red, 3));
        painter.drawLine(centerX - 20, centerY + pitchOffset, centerX, centerY + pitchOffset - 10);
        painter.drawLine(centerX, centerY + pitchOffset - 10, centerX + 20, centerY + pitchOffset);

        // Gambar kompas
        int compassRadius = 50;
        int compassX = width() - 100;
        int compassY = 50;
        painter.setPen(QPen(Qt::white, 2));
        painter.drawEllipse(QPoint(compassX, compassY), compassRadius, compassRadius);
        
        // Label arah mata angin
        painter.drawText(compassX - 5, compassY - compassRadius - 5, "N");
        painter.drawText(compassX - compassRadius - 15, compassY + 5, "W");
        painter.drawText(compassX + compassRadius + 5, compassY + 5, "E");
        painter.drawText(compassX - 5, compassY + compassRadius + 15, "S");
        
        // Jarum merah yang berputar sesuai yaw
        painter.setPen(QPen(Qt::red, 3));
        double angle = -yaw * M_PI / 180.0; // Konversi ke radian
        int needleX = compassX + static_cast<int>(compassRadius * sin(angle));
        int needleY = compassY - static_cast<int>(compassRadius * cos(angle));
        painter.drawLine(compassX, compassY, needleX, needleY);
    }
};

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg, GCSGUI *gui) {
    double roll = atan2(2.0 * (msg->orientation.w * msg->orientation.x + msg->orientation.y * msg->orientation.z),
                        1.0 - 2.0 * (msg->orientation.x * msg->orientation.x + msg->orientation.y * msg->orientation.y));
    double pitch = asin(2.0 * (msg->orientation.w * msg->orientation.y - msg->orientation.z * msg->orientation.x));
    double yaw = atan2(2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y),
                       1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z));
    gui->updateRollPitchYaw(roll * 57.2958, pitch * 57.2958, yaw * 57.2958);
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg, GCSGUI *gui) {
    gui->updateMode(msg->mode);
}

void altitudeCallback(const std_msgs::Float64::ConstPtr &msg, GCSGUI *gui) {
    gui->updateAltitude(msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gcs_gui");
    ros::NodeHandle nh;

    QApplication app(argc, argv);
    GCSGUI gui;
    gui.resize(400, 400);
    gui.show();

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, boost::bind(&imuCallback, _1, &gui));
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&stateCallback, _1, &gui));
    ros::Subscriber alt_sub = nh.subscribe<std_msgs::Float64>("/mavros/altitude", 10, boost::bind(&altitudeCallback, _1, &gui));

    ros::AsyncSpinner spinner(2);
    spinner.start();
    return app.exec();
}

