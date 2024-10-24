#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <PicoSoftwareSerial.h>
#include <LoRa.h>
#include <Wire.h>
#include <Rp2040_inferencing.h>


#define N_SENSORS     9

uint8_t poll_acc(void);
uint8_t poll_gyr(void);
uint8_t poll_INA(void);

/* Private variables ------------------------------------------------------- */
static const bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static float data[N_SENSORS];
static int8_t fusion_sensors[N_SENSORS];
static int fusion_ix = 9;

/* Forward declarations ------------------------------------------------------- */
float ei_get_sign(float number);
static bool ei_connect_fusion_list(const char *input_list);

//LoRa
const long frequency = 433E6;  // LoRa Frequency
char message[100];
int Systems_Status;
const int csPin = 20;          // LoRa radio chip select
const int resetPin = 21;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin
/*
MOSI: 19/MISO: 16/SCK: 18/SS: 17
*/
// Initialize MPU6050 & INA219
Adafruit_MPU6050 mpu;
Adafruit_INA219 ina219;

// MPU6050 I2C address
const int MPU = 0x68;
int counter ;
// Variables to hold accelerometer and gyroscope data
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AngleRoll, AnglePitch, yaw;

// Variables to INA219 
#define DATA_INPUT_USER 16
#define AXIS_NUMBER 9
float shuntvoltage,busvoltage,current_mA,power_mW ;
float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // Buffer of input values
int i;
//Functions Prototype
void Data_Collect_Mode(void);
void MPU_GetData(void);
void Get_INA219_Data(void);
void LoRa_rxMode(void);
void LoRa_txMode(void);
void LoRa_sendMessage(String message);
void onReceive(int packetSize);
void onTxDone(void);
void LoRa_Send(ei_impulse_result_t result);
boolean runEvery(unsigned long interval);
uint8_t poll_acc_gyr(void);
void print_inference_result(ei_impulse_result_t result);
void Ai_Detection(void);
/** Struct to link sensor axis name to sensor value function */
typedef struct{
    const char *name;
    float *value;
    uint8_t (*poll_sensor)(void);
    int8_t used;
    int8_t status;  // -1 not used 0 used(unitialized) 1 used(initalized) 2 data sampled
} eiSensors;

ei_impulse_result_t result = {0} ;

#define NOT_USED -1

eiSensors sensors[] =
{
    "accX", &data[0], &poll_acc_gyr, 1 , NOT_USED,
    "accY", &data[1], &poll_acc_gyr, 1, NOT_USED,
    "accZ", &data[2], &poll_acc_gyr, 1 , NOT_USED,
    "gyrX", &data[3], &poll_acc_gyr, 1, NOT_USED,
    "gyrY", &data[4], &poll_acc_gyr, 1, NOT_USED,
    "gyrZ", &data[5], &poll_acc_gyr, 1 , NOT_USED,
    "busvoltage", &data[6], &poll_acc_gyr, 1, NOT_USED,
    "current_mA", &data[7], &poll_acc_gyr, 1, NOT_USED,
    "power_mW", &data[8], &poll_acc_gyr, 1 , NOT_USED,
};

void setup() {
  Serial.begin(115200);
  LoRa.setPins(csPin, resetPin, irqPin);
  
  /* Connect used sensors */
  if(ei_connect_fusion_list(EI_CLASSIFIER_FUSION_AXES_STRING) == false) {  
    while (1) {
      delay(1000);
      Serial.println("ERR: Errors in sensor list detected\r\n");
    }
    }
  if (!LoRa.begin(frequency)) {
    while (1) {
      delay(1000);
    Serial.println("Starting LoRa failed!");
    }
  }
  if (!ina219.begin()) {
    while (1) {
      delay(1000);
      Serial.print("Starting INA219 failed!\n"); }
  }
  Serial.print("INA219_started \n"); 
  // Initialize MPU6050
  if (!mpu.begin()) {
    delay(10);
    while (1) {
      delay(1000);
      Serial.print("Starting MPU6050 failed!\n"); }
  }
  Serial.print("MPU_started\n"); 

  // Set MPU6050 ranges and bandwidth
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);  
  
  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Node");
  Serial.println("Only receive messages from gateways");
  Serial.println("Tx: invertIQ disable");
  Serial.println("Rx: invertIQ enable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  delay(5000);
  Serial.println("AccX,AccY,AccZ,GyroX,GyroY,GyroZ,busvoltage,current_mA,power_mW");
  Ai_Detection();
}
void loop() {
    LoRa_Send(result);
}
void Ai_Detection(){
    ei_printf("\nStarting inferencing in 1 seconds...\r\n");
    delay(1000);
    // Turn the raw buffer in a signal which we can the classify
    poll_acc_gyr();
    float buffer[9] = {data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]};
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("ERR:(%d)\r\n", err);
        return;
    }
    // Run the classifier
    result = { 0 };
    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR:(%d)\r\n", err);
        return;
    }
  print_inference_result(result);
}
void LoRa_Send(ei_impulse_result_t result){
  Get_INA219_Data();
  Ai_Detection();
  if (runEvery(1000)) { // repeat every 1000 millis
  sprintf(message,"System Status = Unknown /classification Score = Unknown/V = %.4f/mA = %.4f/mW = %.4f",busvoltage,current_mA,power_mW);
  for(int i = 0 ;i<EI_CLASSIFIER_LABEL_COUNT;i++){
    if(result.classification[i].value>=0.80){
      Systems_Status = i ;
    //ei_classifier_inferencing_categories[i];
    //result.classification[i].value;
      sprintf(message,"System Status = %s/classification Score = %.3f/V = %.4f/mA = %.4f/mW = %.4f",ei_classifier_inferencing_categories[Systems_Status],result.classification[Systems_Status].value,busvoltage,current_mA,power_mW);
    }
  }
  LoRa_sendMessage(message); // send a message
  Serial.println(message);
  Serial.println("Send Message!");
  }
}
// Function to get data from MPU6050
void MPU_GetData(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  AccX = a.acceleration.x; // X-axis value
  AccY = a.acceleration.y ;// Y-axis value
  AccZ = a.acceleration.z ;// Z-axis value
  GyroX=(float)g.gyro.x;
  GyroY=(float)g.gyro.y;
  GyroZ=(float)g.gyro.z;
	float tanx,tany,tanz;
	tanx = AccX/(sqrt((AccZ)*(AccZ))+(AccY*AccY));
	tany = AccY/(sqrt((AccX)*(AccX))+(AccZ*AccZ));
	tanz = AccZ/(sqrt((AccY)*(AccY))+(AccX*AccX));
	tanx = atan(tanx);
	tany = atan(tany);
	tanz = atan(tanz);
	AngleRoll = tanx*(1/(3.14/180.0));
	AnglePitch = tany*(1/(3.14/180.0));
	yaw = tanz*(1/(3.14/180.0));
}
void Get_INA219_Data() {
  // Read voltage and current from INA219.
 shuntvoltage = ina219.getShuntVoltage_mV();
 busvoltage = ina219.getBusVoltage_V();
 current_mA = ina219.getCurrent_mA();

  // Compute load voltage, power, and milliamp-hours.
  float loadvoltage = busvoltage + (shuntvoltage / 1000);
 power_mW = loadvoltage * current_mA;
  (void)power_mW;
}

void Data_Collect_Mode(){
    MPU_GetData();
    Get_INA219_Data();
    delay(100);
		Serial.print(AccX);
    Serial.print(",");
		Serial.print(AccY);
    Serial.print(",");
		Serial.print(AccZ);
    Serial.print(",");
    Serial.print(GyroX);
    Serial.print(",");
		Serial.print(GyroY);
    Serial.print(",");
		Serial.print(GyroZ);
    Serial.print(",");
    Serial.print(busvoltage);
    Serial.print(",");
		Serial.print(current_mA);
    Serial.print(",");
		Serial.println(power_mW);
}


void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  Serial.print("Node Receive: ");
  Serial.println(message);
}

void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

static int8_t ei_find_axis(char *axis_name)
{
    int ix;
    for(ix = 0; ix < N_SENSORS; ix++) {
        if(strstr(axis_name, sensors[ix].name)) {
            return ix;
        }
    }
    return -1;
}

static bool ei_connect_fusion_list(const char *input_list)
{
    char *buff;
    bool is_fusion = false;

    /* Copy const string in heap mem */
    char *input_string = (char *)ei_malloc(strlen(input_list) + 1);
    if (input_string == NULL) {
        return false;
    }
    memset(input_string, 0, strlen(input_list) + 1);
    strncpy(input_string, input_list, strlen(input_list));

    /* Clear fusion sensor list */
    memset(fusion_sensors, 0, N_SENSORS);
    fusion_ix = 0;

    buff = strtok(input_string, "+");

    while (buff != NULL) { /* Run through buffer */
        int8_t found_axis = 0;

        is_fusion = false;
        found_axis = ei_find_axis(buff);

        if(found_axis >= 0) {
            if(fusion_ix < N_SENSORS) {
                fusion_sensors[fusion_ix++] = found_axis;
                sensors[found_axis].status = -1;
            }
            is_fusion = true;
        }

        buff = strtok(NULL, "+ ");
    }

    ei_free(input_string);

    return is_fusion;
}

float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

uint8_t poll_acc_gyr(void) {
    MPU_GetData();
    Get_INA219_Data();
    data[0] = AccX;
    data[1] = AccY;
    data[2] = AccZ;
    data[3] = GyroX;
    data[4] = GyroY;
    data[5] = GyroZ;
    data[6] = busvoltage;
    data[7] = current_mA;
    data[8] = power_mW;
    return 0;
}

void print_inference_result(ei_impulse_result_t result) {

    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
}
