
#include <stdarg.h>

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include "LSM6DSOXFIFOWrapper.h"
#include "model.h"

const int numSamples = 120;
static int samplesRead = 0;

// Global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// Pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model *tflModel = nullptr;
tflite::MicroInterpreter *tflInterpreter = nullptr;
TfLiteTensor *tflInputTensor = nullptr;
TfLiteTensor *tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
uint8_t tensorArena[tensorArenaSize];

// array to map gesture index to a name
const char *GESTURES[] = {
    "idle",
    "left",
    "right"};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))
#define UART_CLOCK_RATE 921600 // Does not matter here since RP2040 is using USB Serial Port. (Virtual UART)
#define IIC_BUS_SPEED 400e3    // I2C bus speed in Hz. Options are: 100 kHz, 400 kHz, and 1.0 Mhz.
#define PRINT_BUFFER_SIZE 128  // Increase this number if you see the output gets truncated

static LSM6DSOXFIFO IMU = LSM6DSOXFIFO(Wire, LSM6DSOX_I2C_ADD_L); // IMU on the I2C bus

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

static int IMULoggingCB(const char *str)
{
    return log("%s", str);
}

static void IMUDataReadyCB([[maybe_unused]] LSM6DSOXFIFO::imu_data_t *data)
{
    IMU.print();

    tflInputTensor->data.f[samplesRead * 6 + 0] = data->acceleration_data.X;
    tflInputTensor->data.f[samplesRead * 6 + 1] = data->acceleration_data.Y;
    tflInputTensor->data.f[samplesRead * 6 + 2] = data->acceleration_data.Z;
    tflInputTensor->data.f[samplesRead * 6 + 3] = data->rotation_data.X;
    tflInputTensor->data.f[samplesRead * 6 + 4] = data->rotation_data.Y;
    tflInputTensor->data.f[samplesRead * 6 + 5] = data->rotation_data.Z;

    samplesRead++;
}

void setup()
{
    Serial.begin(UART_CLOCK_RATE);

    // Comment out this line to skip waiting for serial:
    while (!Serial)
        delay(10);

    // I2C, fast mode
    Wire.begin();
    Wire.setClock(IIC_BUS_SPEED);

    // Initialize sensors
    if (!IMU.initialize())
    {
        log("Failed to initialize IMU\n");
        while (1)
            ; // Halt execution
    }

    IMU.registerLoggingCallback(IMULoggingCB);
    IMU.registerDataReadyCallback(IMUDataReadyCB);

    // Get the TFL representation of the model byte array
    tflModel = tflite::GetModel(model);
    if (tflModel->version() != TFLITE_SCHEMA_VERSION)
    {
        Serial.println("Model schema mismatch!");
        while (1)
            ;
    }

    // Create an interpreter to run the model
    tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, nullptr, nullptr);

    // Allocate memory for the model's input and output tensors
    tflInterpreter->AllocateTensors();

    // Get pointers for the model's input and output tensors
    tflInputTensor = tflInterpreter->input(0);
    tflOutputTensor = tflInterpreter->output(0);

    log("Starting...\n");
}

void loop()
{
    // put your main code here, to run repeatedly:
    IMU.update();
    if (samplesRead == numSamples)
    {
        samplesRead = 0;

        // Run inferencing
        TfLiteStatus invokeStatus = tflInterpreter->Invoke();
        if (invokeStatus != kTfLiteOk)
        {
            Serial.println("Invoke failed!");
            while (1)
                ;
            return;
        }

        // Loop through the output tensor values from the model
        for (uint32_t i = 0; i < NUM_GESTURES; i++)
        {
            Serial.print(GESTURES[i]);
            Serial.print(": ");
            Serial.println(tflOutputTensor->data.f[i], 6);
        }
        Serial.println();
    }
}
