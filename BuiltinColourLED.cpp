#include "BuiltinColourLED.h" // Include the header file for BuiltinColourLED class

#include <WiFiNINA.h> // Include the library for WiFiNINA functionality on Arduino

void BuiltinColourLED::enable() const
{
    // Set the LED pins to output mode
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    // Manually set the LED pins to LOW to ensure they're off
    // This is a workaround for a bug in the WiFiNINA library's analogWrite function
    // Refer to: https://forum.arduino.cc/t/rp2040-connect-rgb-led-still-glows-after-analogwrite-ledr-255/868632
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
}

void BuiltinColourLED::setRGB(const rgb_t &rgb) const
{
    // analogWrite(LEDR, ~gammaCorrection(rgb.r, LED_GAMMA_VALUE));
    // analogWrite(LEDG, ~gammaCorrection(rgb.g, LED_GAMMA_VALUE));
    // analogWrite(LEDB, ~gammaCorrection(rgb.b, LED_GAMMA_VALUE));

    // Software fix for BUG in WiFiNINA analogWrite code not fully switching off the LED
    // Writing 255 to the LED pin *should* turn off the LED but not in this case,
    // so we will need to set the pin to digital mode and turn it off that way.
    // This is a workaround for a bug in the WiFiNINA library's analogWrite function
    // Refer to: https://forum.arduino.cc/t/rp2040-connect-rgb-led-still-glows-after-analogwrite-ledr-255/868632

    // Apply gamma correction to each color component
    uint8_t r = gammaCorrection(rgb.r, LED_GAMMA_VALUE);
    uint8_t g = gammaCorrection(rgb.g, LED_GAMMA_VALUE);
    uint8_t b = gammaCorrection(rgb.b, LED_GAMMA_VALUE);

    // Use ternary operators to either write the corrected value with analogWrite
    // or set the pin mode to OUTPUT if the value is 0 (to turn off the LED)
    (r) ? analogWrite(LEDR, ~r) : pinMode(LEDR, OUTPUT);
    (g) ? analogWrite(LEDG, ~g) : pinMode(LEDG, OUTPUT);
    (b) ? analogWrite(LEDB, ~b) : pinMode(LEDB, OUTPUT);
}

void BuiltinColourLED::setRGB(uint8_t r, uint8_t g, uint8_t b) const
{
    // Overloaded method to set RGB values directly
    setRGB(rgb(r, g, b));
}

void BuiltinColourLED::setHSV(const hsv_t &hsv) const
{
    // Convert HSV values to RGB and set the LED colors
    rgb_t rgb;
    hsv2rgb(hsv, rgb);
    setRGB(rgb);
}

void BuiltinColourLED::setHSV(uint16_t h, uint8_t s, uint8_t v) const
{
    // Overloaded method to set HSV values directly
    setHSV(hsv(h, s, v));
}

void BuiltinColourLED::hsv2rgb(const hsv_t &hsv, rgb_t &rgb) const
{
    // Convert HSV color model to RGB color model
    float h = hsv.h / 360.0f;           // Normalize hue to [0, 1]
    float s = hsv.s / 100.0f;           // Normalize saturation to [0, 1]
    float v = hsv.v / 100.0f;           // Normalize value (brightness) to [0, 1]
    int8_t i = static_cast<int>(h * 6); // Sector 0 to 5

    // Fractional part of h
    float f = h * 6 - i;
    // Calculate intermediate values
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    // Assign RGB values based on the sector of the hue
    switch (i % 6)
    {
    case 0:
        rgb.r = v * LED_PWM_RESOLUTION;
        rgb.g = t * LED_PWM_RESOLUTION;
        rgb.b = p * LED_PWM_RESOLUTION;
        break;
    case 1:
        rgb.r = q * LED_PWM_RESOLUTION;
        rgb.g = v * LED_PWM_RESOLUTION;
        rgb.b = p * LED_PWM_RESOLUTION;
        break;
    case 2:
        rgb.r = p * LED_PWM_RESOLUTION;
        rgb.g = v * LED_PWM_RESOLUTION;
        rgb.b = t * LED_PWM_RESOLUTION;
        break;
    case 3:
        rgb.r = p * LED_PWM_RESOLUTION;
        rgb.g = q * LED_PWM_RESOLUTION;
        rgb.b = v * LED_PWM_RESOLUTION;
        break;
    case 4:
        rgb.r = t * LED_PWM_RESOLUTION;
        rgb.g = p * LED_PWM_RESOLUTION;
        rgb.b = v * LED_PWM_RESOLUTION;
        break;
    case 5:
        rgb.r = v * LED_PWM_RESOLUTION;
        rgb.g = p * LED_PWM_RESOLUTION;
        rgb.b = q * LED_PWM_RESOLUTION;
        break;
    }
}

uint8_t BuiltinColourLED::gammaCorrection(const uint8_t component, const float gamma) const
{
    // Apply gamma correction to the color component
    return static_cast<uint8_t>(powf(component / LED_PWM_RESOLUTION, gamma) * LED_PWM_RESOLUTION + 0.5f);
}
