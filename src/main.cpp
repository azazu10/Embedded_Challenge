#include <Arduino.h>
// #define FFT_SPEED_OVER_PRECISION
// #define FFT_SQRT_APPROXIMATION
// #define USE_AVR_PROGMEM
#include <arduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

#define SAMPLES 128           // Must be a power of 2 for FFT
#define SAMPLING_FREQUENCY 48 // Hz, /2 for nyquest i.e. 24 Hz

float samples = SAMPLES;

float vReal[SAMPLES];
float vImag[SAMPLES];

ArduinoFFT<float> fft(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

void setup()
{
    Serial.begin(9600);
    CircuitPlayground.begin();

    // CircuitPlayground.setAccelRange(LIS3DH_RANGE_16_G);
}

void loop()
{
    for (int i = 0; i < SAMPLES; i++)
    {
        float x = CircuitPlayground.motionX();
        float y = CircuitPlayground.motionY();
        float z = CircuitPlayground.motionZ();

        // Serial.print("x: ");
        // Serial.println(x);
        // Serial.print("y: ");
        // Serial.println(y);
        // Serial.print("z: ");
        // Serial.println(z);
        // Serial.println("----");

        // vReal[i] = abs(sqrt(x * x + y * y + z * z) - 9.8);
        vReal[i] = sqrt(x * x + y * y + z * z);
        vImag[i] = 0;

        // Serial.print("sample number: ");
        // Serial.println(i);
        // Serial.println(vReal[i]);

        // Serial.println("-----");

        delay(1000 / SAMPLING_FREQUENCY);
    }

    fft.dcRemoval();

    // Serial.print("vReal after dc removal: ");

    // for (int i = 0; i < SAMPLES; i++)
    // {
    //     Serial.print(vReal[i]);
    //     Serial.print(" ");
    // }

    // Serial.println("-----");

    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);

    // Serial.print("vReal after windowing: ");

    // for (int i = 0; i < SAMPLES; i++)
    // {
    //     Serial.print(vReal[i]);
    //     Serial.print(" ");
    // }

    // Serial.println("-----");

    fft.compute(FFTDirection::Forward);

    // Serial.print("vReal after compute: ");

    // for (int i = 0; i < SAMPLES; i++)
    // {
    //     Serial.print(vReal[i]);
    //     Serial.print(" ");
    // }

    // Serial.println("-----");

    fft.complexToMagnitude();

    // Serial.print("vReal after complex to magnitude: ");

    // for (int i = 0; i < SAMPLES; i++)
    // {
    //     Serial.print(vReal[i]);
    //     Serial.print(" ");
    // }

    // Serial.println("-----");

    // float freq = fft.majorPeak();

    float freq = fft.majorPeakParabola();

    Serial.print("Frequency peak:");
    Serial.println(freq);
    // Serial.println("-----");
}
