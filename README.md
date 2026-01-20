# 🐭 Micromouse Maze-Solving Robot

A Micromouse bot built using the **STM32F303RE microcontroller** for autonomous maze solving. The bot integrates **MPU6050 (gyro+accelerometer)**, **ultrasonic and IR proximity sensors**, and **hall-effect sensor-based geared motors** for precise motion and environment sensing.

## 🧠 Features

* ✅ Maze-solving with **Flood Fill Algorithm**
* 📀 Accurate turning using **MPU6050 gyroscope**
* 🧱 Obstacle detection via **IR and ultrasonic sensors**
* ⚙️ Dead-reckoning with **hall-effect motor encoders**
* ⚡ Real-time debugging over UART

---

## 🔧 Hardware Components

| Component                | Purpose                              |
| ------------------------ | ------------------------------------ |
| STM32F303RE Nucleo       | Main microcontroller (ARM Cortex-M4) |
| MPU6050                  | Gyroscope and accelerometer          |
| 940nm IR Sensors         | Wall and line detection              |
| HC-SR04 Ultrasonic       | Obstacle distance measurement        |
| Geared Motors + Encoders | Precision motion and step counting   |
| L298N / TB6612FNG        | Motor driver                         |
| LiPo Battery + Regulator | Power supply                         |

---

## ⚙️ Firmware Architecture

### Project Structure

```
/Micromouse/
├── Core/
│   ├── Src/
│   ├── Inc/
├── Drivers/
├── FloodFill/
├── MotionControl/
├── Sensors/
└── Utilities/
```

---

## 🚗 Motion Control

Implemented using PID-controlled motor driver signals and encoder feedback.

```c
// PID motor speed control logic
void Motor_PID_Update(int target_rpm, int current_rpm) {
    // PID logic here
}
```

### Hall Effect Step Counting

```c
void EXTI15_10_IRQHandler(void) {
    // Increment encoder tick count here
}
```

---

## 📀 Gyroscope Integration (MPU6050)

Used for orientation and precise angular turns.

```c
float get_gyro_angle_z() {
    // Read raw gyro data and integrate over time
}
```

---

## 📱 Sensor Integration

### IR Sensor Reading

```c
uint16_t read_ir_sensor(uint8_t channel) {
    // ADC reading for IR sensor
}
```

### Ultrasonic Distance Measurement

```c
uint32_t get_distance_cm() {
    // Echo pulse time to distance conversion
}
```

---

## 🧩 Maze Solving Algorithm

Implemented using **Flood Fill** with wall memory and visited cell tracking.

```c
void flood_fill_update(uint8_t x, uint8_t y) {
    // Update cost matrix based on current cell
}
```

---

## 📤 Debugging and Telemetry

UART/USB serial output for live data monitoring.

```c
printf("X: %d Y: %d Heading: %.2f\r\n", pos_x, pos_y, heading);
```

---

## 🔋 Power Management

* Main motor supply: 7.4V LiPo
* Logic supply: 5V LDO regulator
* Brown-out and undervoltage protection

---

## 💪 Development Environment

* **IDE**: STM32CubeIDE / Keil uVision / VS Code + PlatformIO
* **Toolchain**: ARM GCC
* **Debugging**: ST-Link V2 / UART Serial Monitor

---

## 📽️ Demo

*(Add images or video links of the bot running in a maze)*

---

## 📌 Future Work

* Implement **SLAM** with improved path memory
* Add **Bluetooth interface** for remote commands
* Optimize motion profile for faster traversal

---

## 📚 References

* [STM32F3 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00043574.pdf)
* [MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
* [Micromouse Algorithms Wiki](https://micromouseonline.com/)

---

## 🤝 Contributions

Feel free to fork and contribute! Open issues for suggestions or bugs.

---

## Event pics


https://github.com/user-attachments/assets/90825e2e-136e-47be-9c7b-b385be89e601


![mouse1](https://github.com/user-attachments/assets/65936c3d-db50-4105-8471-9a234fa2bcf7)
![mouse2](https://github.com/user-attachments/assets/5c6084e9-ac87-45ec-9658-fc84ff0b02a5)
![mouse3](https://github.com/user-attachments/assets/73a68ee4-6d4a-4825-87f1-2d6b7d71806d)



## 📜 License

This project is licensed under the MIT License.
