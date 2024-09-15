#include "BuiltinColourLED.h"

#include <WiFiNINA.h>

void BuiltinColourLED::enable() const
{
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
}

void BuiltinColourLED::setRGB(const rgb_t &rgb) const
{
    analogWrite(LEDR, ~gammaCorrection(rgb.r, LED_GAMMA_VALUE));
    analogWrite(LEDG, ~gammaCorrection(rgb.g, LED_GAMMA_VALUE));
    analogWrite(LEDB, ~gammaCorrection(rgb.b, LED_GAMMA_VALUE));
}

void BuiltinColourLED::setRGB(uint8_t r, uint8_t g, uint8_t b) const
{
    setRGB(rgb(r, g, b));
}

void BuiltinColourLED::setHSV(const hsv_t &hsv) const
{
    rgb_t rgb;
    hsv2rgb(hsv, rgb);
    setRGB(rgb);
}

void BuiltinColourLED::setHSV(uint16_t h, uint8_t s, uint8_t v) const
{
    setHSV(hsv(h, s, v));
}

void BuiltinColourLED::hsv2rgb(const hsv_t &hsv, rgb_t &rgb) const
{
    float h = hsv.h / 360.0f;
    float s = hsv.s / 100.0f;
    float v = hsv.v / 100.0f;
    int8_t i = static_cast<int>(h * 6);

    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

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
    return static_cast<uint8_t>(powf(component / LED_PWM_RESOLUTION, 1 / gamma) * LED_PWM_RESOLUTION);
}
