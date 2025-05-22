#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

// ===== SHAKE DETECTION SENSITIVITY ADJUSTMENTS =====
const float SHAKE_THRESHOLD = 5.0;// Aggressive shake threshold (in m/s²)
const int STABILITY_TIME = 500;// How long to wait before resetting shake (ms)
const int MIN_SHAKE_DURATION = 50;// How long shake must persist (ms)
const int MOVING_AVG_WINDOW = 5;// Smoothing over 5 readings

// ===== SPEAKER PIN =====
const int SPEAKER_PIN = 9;// Speaker connected directly to Pin 9

// ===== SHAKE COUNTING VARIABLES =====
const int MIN_SHAKES = 20;// Minimum number of shakes for the random target
const int MAX_SHAKES = 30;// Maximum number of shakes for the random target
int targetShakes = 0;// Random target number of shakes
int shakeCount = 0;// Shake Counter
int doubleThreshold = 0;// Point for Double Beeps

// ===== VARIABLES =====
unsigned long lastShakeTime = 0;
unsigned long lastMovementTime = 0;
bool isShaking = false;
float lastAccelMagnitude = 0.0;
unsigned long currentTime;

// Smoothing buffers
float accelXBuffer[MOVING_AVG_WINDOW];
float accelYBuffer[MOVING_AVG_WINDOW];
float accelZBuffer[MOVING_AVG_WINDOW];
int bufferIndex = 0;

void setup() {
  Serial.begin(9600);

  if (!bno.begin()) {
    Serial.println("Couldn't find BNO055 sensor!");
    while (1);
  }

  bno.setExtCrystalUse(true);
  pinMode(SPEAKER_PIN, OUTPUT);

  // Initialize buffers
  for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
    accelXBuffer[i] = 0;
    accelYBuffer[i] = 0;
    accelZBuffer[i] = 0;
  }

  // Set a random target shake count
  randomSeed(analogRead(0));  // Use an analog pin to seed the random number generator
  targetShakes = random(MIN_SHAKES, MAX_SHAKES + 1);
  doubleThreshold = int(targetShakes * 0.75);

  
  Serial.print("Shake Target: ");
  Serial.println(targetShakes);
}

void loop() {
  // ===== READ ACCELEROMETER =====
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float accelX = accel.x();
  float accelY = accel.y();
  float accelZ = accel.z();

  // ===== ACCELERATION CALCULATIONS =====
  accelXBuffer[bufferIndex] = accelX;
  accelYBuffer[bufferIndex] = accelY;
  accelZBuffer[bufferIndex] = accelZ;
  bufferIndex = (bufferIndex + 1) % MOVING_AVG_WINDOW;

  float smoothedX = 0, smoothedY = 0, smoothedZ = 0;
  for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
    smoothedX += accelXBuffer[i];
    smoothedY += accelYBuffer[i];
    smoothedZ += accelZBuffer[i];
  }
  smoothedX /= MOVING_AVG_WINDOW;
  smoothedY /= MOVING_AVG_WINDOW;
  smoothedZ /= MOVING_AVG_WINDOW;

  // ===== ACCELERATION MAGNITUDE | 3D PYTHAG =====
  float accelMagnitude = sqrt(smoothedX * smoothedX +
                              smoothedY * smoothedY +
                              smoothedZ * smoothedZ);

  Serial.print("Accel Magnitude: ");
  Serial.println(accelMagnitude);

  Serial.println(targetShakes);
  Serial.println(doubleThreshold);

  currentTime = millis();
  float deltaAccel = abs(accelMagnitude - lastAccelMagnitude);

  // ===== SHAKE DETECTION =====
  if (deltaAccel > SHAKE_THRESHOLD) {
    if (!isShaking && (currentTime - lastMovementTime > MIN_SHAKE_DURATION)) {
      Serial.println("Shake Detected!");
      isShaking = true;
      lastShakeTime = currentTime;
      lastMovementTime = currentTime;
      shakeCount++;  // Increment the shake counter
      Serial.print("Shake Count: ");
      Serial.println(shakeCount);
      tone(SPEAKER_PIN, 1000, 200);

      if(shakeCount >= doubleThreshold) // Double beep when getting closer to targer
      {
        delay(100);
        tone(SPEAKER_PIN, 1500, 200);
      } 


      // If the shake count reaches the random target
      if (shakeCount >= targetShakes) {
        Serial.println("Target Shakes Reached! Beeping...");
        for (int i = 0; i < 5; i++) {
          tone(SPEAKER_PIN, 1000, 200);  // Play 1000 Hz tone for 200 ms
          delay(300);  // Wait for 300ms between beeps
        }
        // Reset the shake count and set a new random target
        shakeCount = 0;
        targetShakes = random(MIN_SHAKES, MAX_SHAKES + 1);
        Serial.print("New Shake Target: ");
        Serial.println(targetShakes);
      }
    }
  }

  if (isShaking && (currentTime - lastShakeTime > STABILITY_TIME)) {
    isShaking = false;  // Reset shake status after period of stability
  }

  lastAccelMagnitude = accelMagnitude;
  delay(50);  // Short delay to stabilize reading loop
}
