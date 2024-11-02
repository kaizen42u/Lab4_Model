#include "LSM6DSOXFIFOWrapper.h" // Include the header file for LSM6DSOX FIFO wrapper

LSM6DSOXFIFO::LSM6DSOXFIFO(TwoWire &wire, uint8_t address)
    : lsm6dsoxSensor(&wire, address) // Constructor initializes the sensor with the I2C wire and address
{
    logCallback = nullptr;       // Initialize the log callback as nullptr
    dataReadyCallback = nullptr; // Initialize the data ready callback as nullptr
}

int LSM6DSOXFIFO::initialize(void)
{
    // Begin communication with the sensor
    lsm6dsoxSensor.begin();

    // Enable gyroscope and accelerometer
    if (lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_X() != LSM6DSOX_OK)
    {
        this->sendLog("Error in enabling accelerometer and gyroscope\n");
        return false; // Return failure
    }
    this->sendLog("Success in enabling accelerometer and gyroscope\n");

    // Read and check device ID to ensure correct sensor is connected
    uint8_t device_id;
    lsm6dsoxSensor.ReadID(&device_id);
    if (device_id != LSM6DSOX_ID)
    {
        this->sendLog("Wrong ID (Read:%#02x Expect:%#02x) for LSM6DSOX sensor. Check device is plugged\n", device_id, LSM6DSOX_ID);
        return false; // Return failure
    }
    this->sendLog("Success checking ID for LSM6DSOX sensor\n");

    // Set accelerometer scale. Available values are: 2, 4, 8, 16 G
    lsm6dsoxSensor.Set_X_FS(IMU_ACCELEROMETER_SCALE);
    // Set gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 dps
    lsm6dsoxSensor.Set_G_FS(IMU_GYROSCOPE_SCALE);

    // Set accelerometer Output Data Rate. Available values are: 1.6, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_X_ODR(IMU_SAMPLING_RATE);
    // Set gyroscope Output Data Rate. Available values are 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_G_ODR(IMU_SAMPLING_RATE);

    // Set FIFO Batch Data Rate for accelerometer and gyroscope. Available values are: 0, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_FIFO_X_BDR(IMU_SAMPLING_RATE);
    lsm6dsoxSensor.Set_FIFO_G_BDR(IMU_SAMPLING_RATE);

    /** Set FIFO operation mode. Available values are:
     * LSM6DSOX_BYPASS_MODE: FIFO is not used, the buffer content is cleared
     * LSM6DSOX_FIFO_MODE: buffer continues filling until it becomes full. Then it stops collecting data.
     * LSM6DSOX_STREAM_MODE: continuous mode. Older data are replaced by the new data.
     * LSM6DSOX_STREAM_TO_FIFO_MODE: FIFO buffer starts operating in Continuous mode and switches to FIFO mode when an event condition occurs.
     * LSM6DSOX_BYPASS_TO_STREAM_MODE: FIFO buffer starts operating in Bypass mode and switches to Continuous mode when an event condition occurs.
     * */
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // Flush any previous value in FIFO before start
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // Start batching in continuous mode

    // Set FIFO watermark level to trigger when the defined threshold is reached
    lsm6dsoxSensor.Set_FIFO_Watermark_Level(IMU_FIFO_WATERMARK_LEVEL);

    return true; // Return success
}

void LSM6DSOXFIFO::update(void)
{
    // Check if the FIFO threshold level has been reached
    uint8_t fifo_watermark_threshold_reached = 0; // FIFO watermark status
    lsm6dsoxSensor.Get_FIFO_Watermark_Status(&fifo_watermark_threshold_reached);

    // Process data while the FIFO threshold level is reached
    while (fifo_watermark_threshold_reached)
    {
        // Fetch data from FIFO
        for (uint16_t i = 0; i < IMU_FIFO_WATERMARK_LEVEL; i++)
            readFIFObuffer();                                                        // Read data from FIFO
        lsm6dsoxSensor.Get_FIFO_Watermark_Status(&fifo_watermark_threshold_reached); // Update FIFO watermark status
    }

    // Check if FIFO is full.
    uint8_t fifo_full = 0; // FIFO full status
    lsm6dsoxSensor.Get_FIFO_Full_Status(&fifo_full);

    if (fifo_full)
    {
        this->sendLog("-- FIFO is full! Consider reducing Watermark Level or Buffer Data Rate.\n");
        this->sendLog("Flushing data from FIFO.\n");
        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // Flush FIFO data
        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // Continue batching
    }
}

int LSM6DSOXFIFO::readFIFObuffer(void)
{
    int ret_val = 0;
    uint8_t fifo_tag;                       // FIFO data sensor identifier
    lsm6dsoxSensor.Get_FIFO_Tag(&fifo_tag); // Get data identifier

    switch (fifo_tag)
    {
    [[likely]] case IMU_FIFO_TAG_GYROSCOPE:
        // Get gyroscope data
        if (data.rotation_data_ready)
            this->sendLog("Overwriting rotation data for a more recent ones.\n");
        lsm6dsoxSensor.Get_FIFO_G_Axes(vector3intSerialize(&data.rotation_data));
        data.rotation_data_ready = true;
        ret_val = IMU_FIFO_TAG_GYROSCOPE;
        break;
    [[likely]] case IMU_FIFO_TAG_ACCELEROMETER:
        // Get accelerometer data
        if (data.acceleration_data_ready)
            this->sendLog("Overwriting acceleration data for a more recent ones.\n");
        lsm6dsoxSensor.Get_FIFO_X_Axes(vector3intSerialize(&data.acceleration_data));
        data.acceleration_data_ready = true;
        ret_val = IMU_FIFO_TAG_ACCELEROMETER;
        break;
    [[unlikely]] default:
        // Ignore everything else.
        this->sendLog("Discarding FIFO data TAG ID %02d.\n", fifo_tag);
        ret_val = 0;
        break;
    }

    // If data is ready and callback is set, call the callback function
    if (isDataReady() && dataReadyCallback)
    {
        dataReadyCallback(&data);
        data.flags = false;
    }

    return ret_val; // Return the tag value
}

void LSM6DSOXFIFO::print(imu_data_t *data) const
{
    if (data == NULL) // Return if data is null
        return;

    this->sendLog("[IMU] [%11ld ms], ", millis()); // Log timestamp
    if (data->acceleration_data_ready)
        this->sendLog("Acc: [%6.3f, %6.3f, %6.3f] G, ", data->acceleration_data.X / 1000.0f, data->acceleration_data.Y / 1000.0f, data->acceleration_data.Z / 1000.0f); // Log acceleration data
    if (data->rotation_data_ready)
        this->sendLog("Gyro: [%8.2f, %8.2f, %8.2f] DPS", data->rotation_data.X / 1000.0f, data->rotation_data.Y / 1000.0f, data->rotation_data.Z / 1000.0f); // Log gyroscope data
    this->sendLog("%s", "\n");                                                                                                                               // New line in log
}

void LSM6DSOXFIFO::registerLoggingCallback(const log_callback_t callback)
{
    logCallback = callback; // Register logging callback function
}

void LSM6DSOXFIFO::registerDataReadyCallback(const data_ready_callback_t callback)
{
    dataReadyCallback = callback; // Register data ready callback function
}

int32_t *LSM6DSOXFIFO::vector3intSerialize(vector3int_t *vector) const
{
    return reinterpret_cast<int32_t *>(vector); // Serialize vector data
}

int LSM6DSOXFIFO::sendLog(const char *format, ...) const
{
    if (!logCallback)
        return 0; // Return if log callback is not set

    char buffer[128]; // Buffer for formatted log message
    int ret_val;
    va_list va;
    va_start(va, format);
    ret_val = vsnprintf(buffer, sizeof(buffer), format, va); // Format log message
    va_end(va);
    logCallback(buffer); // Call the log callback with the formatted message
    return ret_val;      // Return the formatted message length
}

bool LSM6DSOXFIFO::isDataReady(void)
{
    // Check if both acceleration and rotation data are ready
    return (data.acceleration_data_ready && data.rotation_data_ready);
}
