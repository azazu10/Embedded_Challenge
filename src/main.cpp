/* 
Embedded Challenge 2024
Mearaj Ahmed
Azaz-ur-Rehman Nasir
Kaddy Koroma

The accelerometer is used to detect tremor-like activity
This device is expected to be worn
We used the FFT to find the peak frequency over a period of time
4 seconds used to gather data from accelerometer and stored into array
FFT analysis of this array to find the peak frequency in those 4 seconds
If that frequency is between of 3-6 Hz "tremor-like frequency"
then the count for tremor-like frequencies is increased, and the total is increased regardless of what is detected
The tremor-like counter is reset if the value of tremor-like/total is less than 75% (arbitrarily picked but higher % was preferred)
Neopixels used to visualize counter in real-time
If the tremor-like counter is above 10, then the device signals a tremor may be occurring, a flag is set,
making a beeping sound until a button is used to reset

Notes:
We could have centerered our 3-6 Hz range so that Hann/Hamming windowing would be optimized,
But through testing this did not work practically with the Playground Classic.
*/

#include <Arduino.h> //Arduino Library
#define FFT_SPEED_OVER_PRECISION //Speed over precision for FFT
#define FFT_SQRT_APPROXIMATION //Using sqrt approximation for speed in FFT
#define USE_AVR_PROGMEM //Stores FFT factors in flash
#include <arduinoFFT.h> //ArduinoFFT Library
#include <Adafruit_CircuitPlayground.h> //Adafruit Circuit Playground Library for NeoPixels & Accelerometer

#define SAMPLES 128.0           // Must be a power of 2 for FFT; More samples means higher freq. resolution
#define SAMPLING_FREQUENCY 32 // Hz, /2 for nyquest i.e. 16 Hz (some overhead to allow larger than 16 Hz frequencies to fall into higher bins)
#define MAX_COUNT 10          // Maximum number of tremor-like frequencies detected (works in conjunction with ratio)
#define FREQUENCY_RATIO 0.75  // Ratio of tremor-like frequencies detected / Total frequencies observed

float vReal[SAMPLES]; // Real vector of signal
float vImag[SAMPLES]; // Imaginary vector of signal (empty)

ArduinoFFT<float> fft(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY); // FFT object

bool isTremorDetected = false; // Flag for tremor like behavior (number of tremor-like freq. >= FREQUENCY_RATIO)
int tremorCount = 0; // Counter for tremor-like freq. detected
int total = 0; // Counter for total freq. observed

void setup()
{
    //Serial.begin(115200); // Used for serial monitor during testing
    CircuitPlayground.begin(); // Initializes CircuitPlayground for Accelerometer & Neopixel usage

    // CircuitPlayground.setAccelRange(LIS3DH_RANGE_16_G); //Used during testing to see how a more wide/narrow G value affected results
}

void loop()
{
    // Collect samples based on # of samples specified earlier
    for (int i = 0; i < SAMPLES; i++)
    {
        // Collect and store x,y,z acceleration into variables
        float x = CircuitPlayground.motionX();
        float y = CircuitPlayground.motionY();
        float z = CircuitPlayground.motionZ();

        // Used for debugging/monitoring during testing
        // Serial.print("x: ");
        // Serial.println(x);
        // Serial.print("y: ");
        // Serial.println(y);
        // Serial.print("z: ");
        // Serial.println(z);
        // Serial.println("----");

        // Was tested as a way to perform DC removal (removed in favor of ArduinoFFT dcRemoval method)
        // vReal[i] = abs(sqrt(x * x + y * y + z * z) - 9.8);

        // Calculate magnitude of acceleration vector b/c we do not care about the direction
        vReal[i] = sqrt(x * x + y * y + z * z);
        vImag[i] = 0;

        // Used for debugging/monitoring during testing
        // Serial.print("sample number: ");
        // Serial.println(i);
        // Serial.println(vReal[i]);
        // Serial.println("-----");

        // Delay the next reading based on the sampling frequency
        delay(1000 / SAMPLING_FREQUENCY);
    }

    // Center the signal around 0
    fft.dcRemoval();

   
    // Used for debugging/monitoring during testing 
    // Serial.print("vReal after dc removal: ");
    // for (int i = 0; i < SAMPLES; i++)
    // {
    //     Serial.print(vReal[i]);
    //     Serial.print(" ");
    // }
    // Serial.println("-----");

    // Hann or Hamming were good options for windowing as we didn't care for frequencies near edges
    // fft.windowing(FFTWindow::Hann, FFTDirection::Forward);
    
    // Hamming was selected after testing
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);

    // Used for debugging/monitoring during testing 
    // Serial.print("vReal after windowing: ");
    // for (int i = 0; i < SAMPLES; i++)
    // {
    //     Serial.print(vReal[i]);
    //     Serial.print(" ");
    // }
    // Serial.println("-----");

    // Computes the FFT of the signal (signal/time -> frequency bins)
    fft.compute(FFTDirection::Forward);

    // Used for debugging/monitoring during testing 
    // Serial.print("vReal after compute: ");
    // for (int i = 0; i < SAMPLES; i++)
    // {
    //     Serial.print(vReal[i]);
    //     Serial.print(" ");
    // }
    // Serial.println("-----");

    // Overwrites first half of complex array; Turns data in array into magnitudes at each frequency bin
    fft.complexToMagnitude();

    // Used for debugging/monitoring during testing 
    // Serial.print("vReal after complex to magnitude: ");
    // for (int i = 0; i < SAMPLES; i++)
    // {
    //     Serial.print(vReal[i]);
    //     Serial.print(" ");
    // }
    // Serial.println("-----");

    // Was an option but majorPeakParabola was picked instead
    // float freq = fft.majorPeak();

    // Stores the interpolated frequency value of the largest frequency,
    // was picked as our focus is on data near the middle and not the ends
    float freq = fft.majorPeakParabola();

    // Used for debugging/monitoring during testing 
    // Serial.print("Frequency peak:");
    // Serial.print(freq);
    // Serial.println("-----");

    // Increases counter for total peak frequencies observed
    total++;

    if (freq >= 3 && freq <= 6)
    {
        // Tremor-like counter Increased
        tremorCount++;

        // Neopixels colored red for # of tremor-likes detected
        CircuitPlayground.setPixelColor(tremorCount - 1, 255, 0, 0);

        if (tremorCount >= MAX_COUNT)
        {
            // Block for tremor-like frequency detected
            if (!isTremorDetected)
            {
                // Tremor-like behavior flag is set
                isTremorDetected = true;

                // Continuous beeping until the left button is pressed
                while (!CircuitPlayground.leftButton()){
                  CircuitPlayground.playTone(500,100);
                  delay(100);
                }

                /*  Reset */
                
                CircuitPlayground.clearPixels();
                isTremorDetected = false;
                Serial.println("\nReset Tremor Detection");
                total = 0;
                tremorCount = 0;
            }
        }
    }

    else
    {
        // Block for tremor-like frequency not detected
        if ((float(tremorCount) / float(total) < FREQUENCY_RATIO))
        {

            /* Reset */
            
            tremorCount = 0;
            total = 0;
            isTremorDetected = false;
            CircuitPlayground.clearPixels();
        }
    }
    
    // Used for debugging/monitoring during testing 
    // Serial.print(" tremorcount: ");
    // Serial.print(tremorCount);
    // Serial.print(" total: ");
    // Serial.println(total);
    
}
