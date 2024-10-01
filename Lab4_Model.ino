#include <stdarg.h>

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include "BuiltinColourLED.h"
#include "LSM6DSOXFIFOWrapper.h"
#include "model.h" // Include the model header file generated from the TensorFlow Lite model

#define UART_CLOCK_RATE 921600 // Does not matter here since RP2040 is using USB Serial Port. (Virtual UART)
#define IIC_BUS_SPEED 400e3    // I2C bus speed in Hz. Options are: 100 kHz, 400 kHz, and 1.0 Mhz.
#define PRINT_BUFFER_SIZE 128  // Increase this number if you see the output gets truncated

const size_t num_features = 6;  // There are 6 features for each sample. (aX, aY, aZ, gX, gY, and gZ)
const size_t num_samples = 120; // Total number of samples
static size_t samples_read = 0; // How many samples has been read since last inference

// Tensor Arena size
const size_t tensor_arena_size = 4 * model_parameters;
// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
uint8_t tensor_arena[tensor_arena_size];

// Global variables used for TensorFlow Lite (Micro)

// TensorFlow Lite setup
const tflite::Model *tflModel = nullptr;

tflite::MicroErrorReporter tflMicroErrorReporter; // Not used
// Pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, it would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver;
tflite::MicroInterpreter *tflInterpreter = nullptr;
TfLiteTensor *tflInputTensor = nullptr;
TfLiteTensor *tflOutputTensor = nullptr;

static LSM6DSOXFIFO IMU = LSM6DSOXFIFO(Wire, LSM6DSOX_I2C_ADD_L); // IMU on the I2C bus
static BuiltinColourLED ColourLED;                                // Arduino Nano RP2040 RGB LED

// Write formatted log message to Serial
static int log(const char *format, ...);

static int log(const char *format, ...)
{
    static char print_buffer[PRINT_BUFFER_SIZE] = {'\0'};
    int ret_val;
    va_list va;
    va_start(va, format);
    ret_val = vsnprintf(print_buffer, PRINT_BUFFER_SIZE, format, va);
    va_end(va);
    Serial.write(print_buffer);
    return ret_val;
}

// Rotate an array to the left by `amount`.
// Refer to: https://godbolt.org/z/z8TnsMzjc
void leftRotate(float *array, int array_size, int amount);

void leftRotate(float *array, int array_size, int amount)
{

    if (array_size == 0)
        return;

    // Get the effective number of rotations:
    amount = amount % array_size;

    // Step 1: Reverse the first `amount` elements
    std::reverse(array, array + amount);

    // Step 2: Reverse the last (`array_size`-`amount`) elements
    std::reverse(array + amount, array + array_size);

    // Step 3: Reverse the entire array (The order of steps doesn't matter)
    std::reverse(array, array + array_size);
}

static int IMULoggingCB(const char *str)
{
    return log("%s", str);
}

static void IMUDataReadyCB([[maybe_unused]] LSM6DSOXFIFO::imu_data_t *data)
{
    static float last_sample_millis = 0;
    const float delta_millis = (1000.0f / IMU_SAMPLING_RATE);

    log("[IMU] [%11d ms], ", int(last_sample_millis));
    if (data->acceleration_data_ready)
        log("Acc: [%6.3f, %6.3f, %6.3f] G, ", data->acceleration_data.X / 1000.0f, data->acceleration_data.Y / 1000.0f, data->acceleration_data.Z / 1000.0f); // Acceleration
    if (data->rotation_data_ready)
        log("Gyro: [%8.2f, %8.2f, %8.2f] DPS", data->rotation_data.X / 1000.0f, data->rotation_data.Y / 1000.0f, data->rotation_data.Z / 1000.0f); // Angular Velocity
    log("%s", "\n");

    last_sample_millis += delta_millis;

    // Populate input, divided by 1000 since the training data is also divided by 1000
    uint32_t index = samples_read * num_features;
    tflInputTensor->data.f[index++] = data->acceleration_data.X / 1000.0f;
    tflInputTensor->data.f[index++] = data->acceleration_data.Y / 1000.0f;
    tflInputTensor->data.f[index++] = data->acceleration_data.Z / 1000.0f;
    tflInputTensor->data.f[index++] = data->rotation_data.X / 1000.0f;
    tflInputTensor->data.f[index++] = data->rotation_data.Y / 1000.0f;
    tflInputTensor->data.f[index++] = data->rotation_data.Z / 1000.0f;
    samples_read++;
}

void setup()
{
    ColourLED.enable();
    ColourLED.setRGB(0, 0, 0);

    Serial.begin(UART_CLOCK_RATE);

    // Comment out this section to skip waiting for serial:
    // while (!Serial)
    // {
    //     static uint32_t last_millis = millis();
    //     const static uint16_t delta = 600;
    //     static bool state = true;
    //     if (millis() - last_millis >= delta)
    //     {
    //         ColourLED.setRGB(0, state ? 100 : 0, 0);
    //         state = !state;
    //         last_millis += delta;
    //     }
    // }

    ColourLED.setRGB(0, 0, 100);

    // Get the TFL representation of the model byte array
    tflModel = tflite::GetModel(model_data);
    if (tflModel == nullptr)
    {
        log("Failed to load model\n");
        while (1)
            ; // Halt execution
    }
    log("tflModel = %p\n", (void *)tflModel);

    int32_t model_version = tflModel->version();
    if (model_version != TFLITE_SCHEMA_VERSION)
    {
        log("Model schema mismatch!\n");
        while (1)
            ;
    }

    // Create an interpreter to run the model
    tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensor_arena, tensor_arena_size, nullptr, nullptr);

    // Allocate memory for the model's input and output tensors
    tflInterpreter->AllocateTensors();

    // Get pointers for the model's input and output tensors
    tflInputTensor = tflInterpreter->input(0);
    tflOutputTensor = tflInterpreter->output(0);

    // Initialize input tensors to NAN.
    for (size_t i = 0; i < (num_samples * num_features); i++)
        tflInputTensor->data.f[i] = NAN;

    log("Model initialization successful.\n");

    // I2C, fast mode
    Wire.begin();
    Wire.setClock(IIC_BUS_SPEED);

    // Initialize sensors
    IMU.registerLoggingCallback(IMULoggingCB);
    IMU.registerDataReadyCallback(IMUDataReadyCB);
    if (!IMU.initialize())
    {
        log("Failed to initialize IMU\n");
        while (1)
            ; // Halt execution
    }

    ColourLED.setRGB(100, 100, 100);
    log("Starting...\n");
}

void loop()
{
    // Read IMU data from FIFO
    IMU.update();

    // After here the `samples_read` will contain the number of NEW samples available

    // Arrange buffer so newer data are located to the right-most side
    leftRotate(tflInputTensor->data.f, num_samples * num_features, samples_read * num_features);
    samples_read = 0;

    // Run inference
    TfLiteStatus invokeStatus = tflInterpreter->Invoke();
    if (invokeStatus != kTfLiteOk)
    {
        log("Invoke failed!");
        while (1)
            ;
        return;
    }

    // Get the highest score of gesture index
    size_t max_index = 0;
    float max_value = tflOutputTensor->data.f[0];
    for (size_t i = 0; i < gesture_len; i++)
        if (tflOutputTensor->data.f[i] > max_value)
        {
            max_value = tflOutputTensor->data.f[i];
            max_index = i;
        }

    // Log the inference result
    log("[Res] [%11d ms] |", millis());
    for (size_t i = 0; i < gesture_len; i++)
        log(" [%6s: %4.2f]", gestures[i], tflOutputTensor->data.f[i]);
    log(" | [%6s: %4.2f]\n", gestures[max_index], max_value);

    // Set LED colour based on the inference result
    switch (max_index)
    {
    case 0: // Refer to `gestures[]` in `model.h` for the full name definition
        ColourLED.setRGB(0, 0, 0);
        break;
    case 1:
        ColourLED.setRGB(0, 255, 0);
        break;
    case 2:
        ColourLED.setRGB(0, 0, 255);
        break;
    case 3:
        ColourLED.setRGB(255, 0, 0);
        break;

        // Add more cases if needed

    default: // Unhandled case
        ColourLED.setRGB(0, 0, 0);
        log("Gesture id %d unhandled", max_index);
        break;
    }
}