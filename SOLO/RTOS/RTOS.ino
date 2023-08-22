//#include <FreeRTOS.h>



void capdata() {
  // Simulate capturing data
  Serial.println("Captured data from sensor.");
  // Replace with your actual sensor reading code later.
}


void bluetoothSendTask(void *parameter) {
  for (;;) {
    Serial.println("Sending data via Bluetooth.");
    Serial.printf("Bluetooth Send Task running on core %d\n", xPortGetCoreID());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void captureDataTask(void *parameter) {
  for (;;) {
    capdata();
    Serial.printf("Capture Data Task running on core %d\n", xPortGetCoreID());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}



void setup() {
  Serial.begin(115200);

  // Initialize your sensors and Bluetooth here.

  // Create the data capturing task on core 0
  xTaskCreatePinnedToCore(
    captureDataTask,  // Task function
    "CaptureData",    // Name of the task
    2000,             // Stack size of the task
    NULL,             // Parameter of the task
      1,                // Priority of the task
    NULL,             // Task handle to keep track of created task
    0);               // Core where the task should run

  // Create the Bluetooth sending task on core 1
  xTaskCreatePinnedToCore(
    bluetoothSendTask,  // Task function
    "BluetoothSend",    // Name of the task
    2000,               // Stack size of the task
    NULL,               // Parameter of the task
    1,                  // Priority of the task
    NULL,               // Task handle to keep track of created task
    1);                 // Core where the task should run
}

void loop() {
  // Not needed, as the tasks manage the operations.
  vTaskDelay(1000 / portTICK_PERIOD_MS);  // Prevent watchdog timeouts, if any.
}
