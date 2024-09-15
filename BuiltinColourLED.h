#pragma once

#include <Arduino.h>

#define LED_PWM_RESOLUTION 255.0f
#define LED_GAMMA_VALUE 1.8f

class BuiltinColourLED
{
public:
    typedef struct rgb
    {
        uint8_t r; // Colour component red: 0 - 255
        uint8_t g; // Colour component green: 0 - 255
        uint8_t b; // Colour component blue: 0 - 255
        rgb() = default;
        rgb(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) {}
    } rgb_t;

    typedef struct hsv
    {
        uint16_t h; // Hue: 0 - 360
        uint8_t s;  // Saturation:  0 - 100
        uint8_t v;  // Value: 0 - 100
        hsv() = default;
        hsv(uint16_t _h, uint8_t _s, uint8_t _v) : h(_h), s(_s), v(_v) {}
    } __packed hsv_t;

    void enable() const;
    void setRGB(const rgb_t &rgb) const;
    void setRGB(uint8_t r, uint8_t g, uint8_t b) const;
    void setHSV(const hsv_t &hsv) const;
    void setHSV(uint16_t h, uint8_t s, uint8_t v) const;
    void hsv2rgb(const hsv_t &hsv, rgb_t &rgb) const;

private:
    uint8_t gammaCorrection(const uint8_t component, const float gamma) const;
};
