#include <ESP32_Gesture_inferencing.h> // change the header file to your EdgeImpulse Library name
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define FREQUENCY_HZ        60
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

Adafruit_MPU6050 mpu;

static unsigned long last_interval_ms = 0;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Classificador de gestos com TinyML");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}

void loop() {

  sensors_event_t a, g, temp;

  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();

    mpu.getEvent(&a, &g, &temp);
// printing imu values uncomment the below lines to debug sensor
    //Serial.print(a.acceleration.x);
    //Serial.print(",");
    //Serial.print(a.acceleration.y);
    //Serial.print(",");
    //Serial.println(a.acceleration.z);

  }

  static unsigned long last_interval_ms = 0;

if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();

        features[feature_ix++] = a.acceleration.x;
        features[feature_ix++] = a.acceleration.y;
        features[feature_ix++] = a.acceleration.z;

if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            ei_impulse_result_t result;

            // create signal from features frame
            signal_t signal;
            numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

            // run classifier
            EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
            //ei_printf("run_classifier returned: %d\n", res);
            if (res != 0) return;

            // print predictions
            /*ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);   */

            // print the predictions
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                Serial.println("Edge Impulse Classification:");
                ei_printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
                Serial.println("                                                                               ");
            }  
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf("anomaly:\t%.3f\n", result.anomaly);            
            }
        #endif
            // reset features frame
            feature_ix = 0;
        }
}}   
