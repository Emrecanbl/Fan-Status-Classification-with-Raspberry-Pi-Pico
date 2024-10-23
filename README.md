🌀 Fan Status Classification Using Edge Impulse with Raspberry Pi Pico
This project utilizes the Raspberry Pi Pico, INA219 current sensor, and MPU6050 IMU sensor to analyze and classify the operational status of a fan. The system leverages Edge Impulse to detect anomalies in the fan’s performance using voltage, current, and motion data.


Image: The setup of the project, including Raspberry Pi Pico and sensors

📋 Project Overview
In this project, a model is trained using Edge Impulse to classify the fan's status by analyzing:

Voltage and Current from the INA219 sensor
Motion and Vibration data from the MPU6050 sensor
The Raspberry Pi Pico acts as the central microcontroller, collecting sensor data and feeding it into the Edge Impulse model for real-time classification.

🛠️ Components
Raspberry Pi Pico: The main MCU for reading sensor data and processing.
INA219: A current sensor to measure voltage and current, used to monitor the fan’s power consumption.
MPU6050: An IMU sensor that captures motion and vibration data to detect irregularities in the fan's operation.
Edge Impulse: A platform used for training and deploying the machine learning model.
🔄 System Workflow
Data Collection: Data from the INA219 and MPU6050 sensors is gathered to monitor the fan's electrical and motion parameters.
Model Training: The collected data is used to train a classification model on Edge Impulse to detect normal operation or anomalies.
Model Deployment: The trained model is deployed on the Raspberry Pi Pico for real-time classification.
Fan Status Classification: The system continuously classifies the fan's status based on live sensor data.
🚀 Future Enhancements
Integrating LoRa communication to transmit data wirelessly to a central hub.
Developing a Qt-based interface on a Raspberry Pi to visualize the real-time sensor data.
📦 Getting Started
Prerequisites
Raspberry Pi Pico SDK
Edge Impulse CLI
INA219 and MPU6050 libraries for Raspberry Pi Pico
Installation
Clone the repository:
bash
Kodu kopyala
git clone https://github.com/yourusername/fan-status-classification
Install the required libraries for INA219 and MPU6050.
Set up an Edge Impulse project and deploy the trained model to the Raspberry Pi Pico.
Usage
Connect the INA219 and MPU6050 sensors to the Raspberry Pi Pico.
Flash the firmware with the deployed Edge Impulse model.
Start monitoring the fan status in real-time.
🖼️ Adding Visuals
To enhance the explanation, you can add diagrams or photos of the setup.
Place your image files in the repository and link them like this:

markdown
Kodu kopyala
![Fan Classification Setup](path-to-your-image.png)
For example, include a diagram of your circuit or screenshots from Edge Impulse showing model accuracy.
