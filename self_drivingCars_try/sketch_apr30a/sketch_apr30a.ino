// =========================================================
// ESP32 — Data Acquisition for Kalman Filter
// Sensors : HC-SR04 Ultrasonic + MPU6050 IMU
// Output  : Serial data in format "distance,acceleration\n"
//           Example : "1.523,0.312"
//
// PINOUT :
//   HC-SR04 :
//     TRIG -> GPIO 4
//     ECHO -> GPIO 6
//     VCC  -> 3.3V or 5V
//     GND  -> GND
//
//   MPU6050 :
//     SDA  -> GPIO 8
//     SCL  -> GPIO 9
//     VCC  -> 3.3V
//     GND  -> GND
//
// MATLAB expects :
//   - Baud rate : 115200
//   - Format    : "float,float\n"
//   - Rate      : one line every 50ms (dt = 0.05s)
// =========================================================

#include <Wire.h>
#include <MPU6050.h>

// =========================================================
// SECTION 1 : PIN DEFINITIONS
// =========================================================

#define TRIG_PIN  4    // Ultrasonic trigger pin
#define ECHO_PIN  6    // Ultrasonic echo pin

#define SDA_PIN   8    // MPU6050 I2C data pin
#define SCL_PIN   9    // MPU6050 I2C clock pin

// =========================================================
// SECTION 2 : CONSTANTS
// =========================================================

// Speed of sound in air at room temperature (m/s)
// For more precision : v_sound = 331.0 + 0.6 * temperature_celsius
#define SOUND_SPEED       343.0

// Sampling period in milliseconds — must match MATLAB dt
// dt = 0.05s -> SAMPLE_PERIOD_MS = 50
#define SAMPLE_PERIOD_MS  50

// Ultrasonic valid range limits
#define MIN_DISTANCE_M    0.02    // 2 cm minimum (HC-SR04 blind zone)
#define MAX_DISTANCE_M    4.00    // 4 m maximum range

// MPU6050 acceleration scale factor
// At +/-2g range : 16384 LSB per g
// 1g = 9.81 m/s²
#define ACCEL_SCALE       16384.0
#define GRAVITY           9.81

// =========================================================
// SECTION 3 : GLOBAL VARIABLES
// =========================================================

MPU6050 mpu;   // MPU6050 object

// Calibration offsets (computed during setup)
float accel_offset_x = 0.0;   // Mean bias on X axis at rest

// Previous valid measurements (for spike rejection)
float last_valid_distance = 1.0;   // Start assuming 1 meter
float last_valid_accel    = 0.0;

// Sample counter for console debug output
unsigned long sample_count = 0;

// =========================================================
// SECTION 4 : FUNCTION DECLARATIONS
// =========================================================

float measure_distance();
float measure_acceleration();
void  calibrate_mpu();
bool  is_valid_distance(float d);
bool  is_valid_acceleration(float a);

// =========================================================
// SECTION 5 : SETUP
// =========================================================

void setup() {

    // Start serial communication with MATLAB
    // Must match COM_PORT and BAUD_RATE in MATLAB code
    Serial.begin(115200);
    while (!Serial) { delay(10); }   // Wait for serial to be ready

    // ---- Initialize I2C on custom pins ----
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);   // 400kHz fast mode for MPU6050

    // ---- Initialize MPU6050 ----
    Serial.println("# Initializing MPU6050...");
    mpu.initialize();

    // Check MPU6050 connection
    if (!mpu.testConnection()) {
        Serial.println("# ERROR : MPU6050 not found. Check wiring.");
        Serial.println("# SDA -> GPIO 8 | SCL -> GPIO 9");
        // Blink LED forever to signal error
        while (true) {
            delay(500);
        }
    }
    Serial.println("# MPU6050 connected successfully.");

    // Set accelerometer range to +/-2g (most sensitive)
    // Options : MPU6050_ACCEL_FS_2  (±2g,  16384 LSB/g)
    //           MPU6050_ACCEL_FS_4  (±4g,   8192 LSB/g)
    //           MPU6050_ACCEL_FS_8  (±8g,   4096 LSB/g)
    //           MPU6050_ACCEL_FS_16 (±16g,  2048 LSB/g)
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    // Set digital low pass filter to reduce vibration noise
    // MPU6050_DLPF_BW_5   : 5  Hz bandwidth (very smooth)
    // MPU6050_DLPF_BW_10  : 10 Hz bandwidth
    // MPU6050_DLPF_BW_20  : 20 Hz bandwidth (recommended)
    // MPU6050_DLPF_BW_42  : 42 Hz bandwidth
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);

    // ---- Initialize Ultrasonic Sensor ----
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    Serial.println("# Ultrasonic sensor initialized.");

    // ---- Calibrate MPU6050 at rest ----
    // Vehicle must be stationary during this phase
    Serial.println("# Calibrating MPU6050 — keep vehicle stationary...");
    calibrate_mpu();
    Serial.println("# Calibration complete.");

    // ---- Signal MATLAB that ESP32 is ready ----
    // MATLAB will ignore lines starting with '#'
    Serial.println("# ESP32 ready. Sending data...");
    Serial.println("# Format : distance_m,acceleration_ms2");

    delay(500);   // Small delay before starting acquisition
}

// =========================================================
// SECTION 6 : MAIN LOOP
// =========================================================

void loop() {

    // Record start time of this iteration
    unsigned long t_start = millis();

    // ---- Read ultrasonic distance ----
    float distance = measure_distance();

    // ---- Read IMU acceleration (X axis = longitudinal) ----
    float acceleration = measure_acceleration();

    // ---- Validate and apply spike rejection ----
    // If reading is invalid, use last valid value
    // This prevents garbage from corrupting the KF
    if (!is_valid_distance(distance)) {
        distance = last_valid_distance;
    } else {
        last_valid_distance = distance;
    }

    if (!is_valid_acceleration(acceleration)) {
        acceleration = last_valid_accel;
    } else {
        last_valid_accel = acceleration;
    }

    // ---- Send data to MATLAB via Serial ----
    // Format : "value1,value2\n"
    // MATLAB reads this as : y = [distance; acceleration]
    Serial.print(distance, 4);       // 4 decimal places
    Serial.print(",");
    Serial.println(acceleration, 4); // println adds \n at end

    // ---- Debug output every 20 samples (~1 second) ----
    sample_count++;
    if (sample_count % 20 == 0) {
        // These lines start with '#' so MATLAB ignores them
        Serial.print("# Sample ");
        Serial.print(sample_count);
        Serial.print(" | d=");
        Serial.print(distance, 3);
        Serial.print(" m | a=");
        Serial.print(acceleration, 3);
        Serial.println(" m/s²");
    }

    // ---- Wait to maintain exact sampling period ----
    // This ensures dt = 0.05s as expected by MATLAB KF
    unsigned long elapsed = millis() - t_start;
    if (elapsed < SAMPLE_PERIOD_MS) {
        delay(SAMPLE_PERIOD_MS - elapsed);
    }
    // If measurement took longer than period, no delay needed
}

// =========================================================
// SECTION 7 : ULTRASONIC MEASUREMENT FUNCTION
// =========================================================

float measure_distance() {
    // HC-SR04 protocol :
    //   1. Send 10µs HIGH pulse on TRIG
    //   2. Measure duration of HIGH pulse on ECHO
    //   3. Distance = (duration * sound_speed) / 2
    //      Division by 2 because sound travels to obstacle AND back

    // Ensure TRIG is LOW before starting
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Send 10µs trigger pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo duration in microseconds
    // Timeout = 30000µs = 30ms corresponding to ~5m max range
    long duration_us = pulseIn(ECHO_PIN, HIGH, 30000);

    // If pulseIn returns 0, no echo received (out of range)
    if (duration_us == 0) {
        return -1.0;   // Signal invalid reading
    }

    // Convert duration to distance in meters
    // duration is in microseconds, sound_speed in m/s
    // distance = (duration_us * 1e-6 * SOUND_SPEED) / 2
    float distance_m = (duration_us * SOUND_SPEED) / 2000000.0;

    return distance_m;
}

// =========================================================
// SECTION 8 : MPU6050 ACCELERATION MEASUREMENT FUNCTION
// =========================================================

float measure_acceleration() {
    // Read raw 16-bit acceleration values from MPU6050
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;   // Gyro not used but required by function

    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw,
                   &gx_raw, &gy_raw, &gz_raw);

    // Convert raw value to m/s²
    // Raw value / scale_factor = value in g
    // Value in g * 9.81 = value in m/s²
    float ax_ms2 = (ax_raw / ACCEL_SCALE) * GRAVITY;

    // Subtract calibration offset (bias measured at rest)
    ax_ms2 -= accel_offset_x;

    // ax_ms2 is the longitudinal acceleration of the vehicle
    // Positive : vehicle accelerates forward (distance decreases)
    // Negative : vehicle decelerates / brakes

    return ax_ms2;
}

// =========================================================
// SECTION 9 : MPU6050 CALIBRATION FUNCTION
// =========================================================

void calibrate_mpu() {
    // Collect N samples at rest and compute mean bias
    // This offset is subtracted from all future readings

    const int N_CAL = 200;       // Number of calibration samples
    float sum_ax    = 0.0;

    Serial.print("# Collecting ");
    Serial.print(N_CAL);
    Serial.println(" calibration samples...");

    for (int i = 0; i < N_CAL; i++) {
        int16_t ax_raw, ay_raw, az_raw;
        int16_t gx_raw, gy_raw, gz_raw;

        mpu.getMotion6(&ax_raw, &ay_raw, &az_raw,
                       &gx_raw, &gy_raw, &gz_raw);

        // Accumulate raw X acceleration
        sum_ax += (ax_raw / ACCEL_SCALE) * GRAVITY;

        delay(5);   // 5ms between samples during calibration
    }

    // Compute mean bias
    accel_offset_x = sum_ax / N_CAL;

    Serial.print("# Calibration done. Accel X offset = ");
    Serial.print(accel_offset_x, 4);
    Serial.println(" m/s²");
}

// =========================================================
// SECTION 10 : VALIDATION FUNCTIONS
// =========================================================

bool is_valid_distance(float d) {
    // Reject readings outside physical sensor range
    // HC-SR04 : 2cm to 400cm
    return (d >= MIN_DISTANCE_M && d <= MAX_DISTANCE_M);
}

bool is_valid_acceleration(float a) {
    // Reject extreme values that indicate sensor fault
    // A car prototype should not exceed ±5g in normal operation
    return (abs(a) <= 5.0 * GRAVITY);
}