#include <Arduino.h>
#define FFT_SPEED_OVER_PRECISION
#define FFT_SQRT_APPROXIMATION
#define USE_AVR_PROGMEM
#include <arduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

#define SAMPLES 128           // Must be a power of 2 for FFT
#define SAMPLING_FREQUENCY 32 // Hz, /2 for nyquest i.e. 24 Hz
#define MAX_COUNT 10

float samples = SAMPLES;

float vReal[SAMPLES];
float vImag[SAMPLES];

ArduinoFFT<float> fft(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

bool isTremorDetected = false;
int tremorCount = 0;
int total = 0;

void setup()
{
    Serial.begin(115200);
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
    Serial.print(freq);
    // Serial.println("-----");
    total++;

    if (freq >= 3 && freq <= 6)
    {
        tremorCount++;
        CircuitPlayground.setPixelColor(tremorCount - 1, 255, 0, 0);
        if (tremorCount >= MAX_COUNT)
        {
            if (!isTremorDetected)
            {
                isTremorDetected = true;

                for (int i = 0; i <= 10; i++)
                {
                    CircuitPlayground.playTone(500, 100);
                    delay(1000);
                }

                CircuitPlayground.clearPixels();
                isTremorDetected = false;
                total = 0;
                tremorCount = 0;
            }
        }
    }

    else
    {
        if ((float(tremorCount) / float(total) < 0.75))
        {
            tremorCount = 0;
            total = 0;
            isTremorDetected = false;
            CircuitPlayground.clearPixels();
        }
    }
    Serial.print(" tremorcount: ");
    Serial.print(tremorCount);
    Serial.print(" total: ");
    Serial.println(total);
}
