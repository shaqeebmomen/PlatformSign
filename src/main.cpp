#include <Arduino.h>
#include "Adafruit_ZeroFFT.h"
#include <Adafruit_NeoPixel.h>
#include <FIR.h>

// Conditional Compilation defines
//#define DEBUG_FFT // Examine FFT output
// #define DEBUG_BUCKET // Examine bucket consolidation
// #define DEBUG_SAMPLE_TIME // Examine how long code cycles can be

#define EN_FIR

// Software defines
#define SAMPLE_SIZE  2048 // keep to a power of 2
#define ZERO_OFFSET 20.0
#define MAX_SCALE 100.0 // Max value expected out of any FFT bucket (after combination) and removing zero offset
/**
 * Bucket Ranges
 * Bucket LOW : (FREQ_MIN, FREQ_LOW)
 * Bucket MED: (FREQ_LOW, FREQ_HIGH)
 * Bucket HIGH: (FREQ_HIGH, FREQ_MAX)
 */
#define FREQ_MIN 100.0 // Include only freqencies above this bucket
#define FREQ_LOW 400.0
#define FREQ_HIGH 1000.0
#define FREQ_MAX 2000.0 // Include only frequencies below this bucket

#define ANIM_COUNT 5
// LED Count defines
#define LED_COUNT 25

// Colours
#define CYAN_RGB 0, 255, 255
#define GREEN_RGB 0, 255, 0
#define PURPLE_RGB 106, 13, 173

// Filter defs
#define TAPS 20

#define POT_READ_MS 300 // ms between reads of the potentiometer

// Pin defines
#define PIN_STRIP 4 // LED strip pin
#define PIN_MIC 8   // Pin for microphone
#define PIN_BTN 5   // User button to change anim
#define PIN_ADJ 3   // Pot to adjust brightness

enum stripName : uint8_t
{
  STRIP_LOW = 0,
  STRIP_MED,
  STRIP_HIGH
};

// LED objects
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, PIN_STRIP, NEO_GRB + NEO_KHZ800);
float gain[3] = {0.6, 3, 6.5};
float constants[3][TAPS] = {
    {0.02, 0.15, 0.1, 0.05, 0.1, 0.15, 0.1, 0.05, 0.02, 0.15, 0.1, 0.05, 0.1, 0.15, 0.1, 0.05, 0.02, 0.15, 0.1, 0.05},
    {0.2, 0.2, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.2, 0.2, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
FIR<TAPS> mavg[3] = {FIR<TAPS>(constants[0]), FIR<TAPS>(constants[1]), FIR<TAPS>(constants[2])};

// Data objects
int16_t micData[SAMPLE_SIZE]; // mic signal
float bucketAvg[3];           // averages for each of the 3 zones of frequences
float filteredBuckets[3];     // averages after applying filtering

uint32_t lastSampleTS;  // timestamp of last sample capture
uint32_t sampleTime;    // time taken for last sample buffer
uint32_t potReadMarker; // timestamp marker for reads of potentiometer

uint8_t animIndex = 0;
bool animEntry = true;   // Flag indicating that the animation is queued to change
bool sampleAudio = true; // Flag to enable or disable audio sample for power

void initLEDs()
{
  // Init LED strips
  strip.begin();
  strip.setBrightness(50);
}

void takeData(int16_t *arr)
{
  uint32_t lastSampleTS = micros();
  int32_t avg = 0;
  for (uint16_t i = 0; i < SAMPLE_SIZE; i++)
  {
    int16_t val = analogRead(PIN_MIC);
    avg += val;
    arr[i] = val;
  }
  sampleTime = micros() - lastSampleTS;
#ifdef DEBUG_SAMPLE_TIME
  Serial.print("Sample Size: ");
  Serial.print(SAMPLE_SIZE);
  Serial.print("\t");
  Serial.print("Capture time: ");
  Serial.println(sampleTime);
  Serial.print("Recommended Freq: ");
  Serial.println(1000000.0 / ((float)sampleTime / (float)SAMPLE_SIZE));

#endif

  // remove DC offset and add gain
  avg = avg / SAMPLE_SIZE;
  for (int i = 0; i < SAMPLE_SIZE; i++)
    arr[i] = (arr[i] - avg) * 32; // 128

  // Run the FFT
  ZeroFFT(arr, SAMPLE_SIZE);

#ifdef DEBUG_FFT
  for (int i = 0; i < SAMPLE_SIZE / 2; i++)
  {
    // print the frequency
    Serial.print(" Hz: ");
    Serial.print(FFT_BIN(i, FREQ_SAMPLE, SAMPLE_SIZE));
    Serial.print(",");

    // print the corresponding FFT output
    Serial.print(arr[i]);
    Serial.println();
  }
  delay(2000);
#endif
}

void compactData(int16_t *data, float *bucket)
{
  uint16_t count[3] = {0, 0, 0};
  // Reset bucket
  for (uint8_t i = 0; i < 3; i++)
  {
    bucket[i] = 0;
  }
  for (uint16_t i = 0; i < SAMPLE_SIZE / 2; i++)
  {
    float currentFreq = FFT_BIN(i, 1000000.0 / ((float)sampleTime / (float)SAMPLE_SIZE), SAMPLE_SIZE);

    if (currentFreq < FREQ_LOW && currentFreq > FREQ_MIN)
    {
      bucket[0] += (float)data[i];
      count[0]++;
    }
    if (currentFreq > FREQ_LOW && currentFreq < FREQ_HIGH)
    {
      bucket[1] += (float)data[i];
      count[1]++;
    }
    if (currentFreq > FREQ_HIGH && currentFreq < FREQ_MAX)
    {
      bucket[2] += (float)data[i]; // adding a slight gain for higher frequencies
      count[2]++;
    }
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    bucket[i] = bucket[i] / (float)count[i] * gain[i]; // Take the average of the buckets
    mavg[i].getOutput(bucket[i]);
    filteredBuckets[i] = mavg[i].getOutput();
  }

#ifdef DEBUG_BUCKET
  Serial.print("/*");
  for (uint8_t i = 0; i < 3; i++)
  {
#ifdef EN_FIR
    Serial.print(mavg[i].getOutput());
#else
    Serial.println(bucketAvg[i]);
#endif
    if (i < 2)
    {
      Serial.print(",");
    }
  }
  Serial.print("*/");
#endif
}

// Calculate last LED to drive with animation, if the # of LEDs driven is based on amplitude of the bucket
uint8_t calcCap(float bucketAvg, uint8_t max)
{
  uint8_t cap;
  if (bucketAvg - ZERO_OFFSET < MAX_SCALE)
  {
    cap = max * (bucketAvg - ZERO_OFFSET) / MAX_SCALE;
  }
  else
  {
    cap = max;
  }
  return cap;
}

// Calculate a brightness cap based on the passed bucket level
uint8_t calcBrightCap(float bucketAvg)
{
  return constrain(255 * (bucketAvg - ZERO_OFFSET) / MAX_SCALE, 0, 255);
}

/*******************ANIMATION HELPER FUNCTIONS**********************/

/**
 * @brief Drives both "psuedo strips" in a symmetrical matter
 *
 * @param index index to change
 * @param color color to set pixel
 */
void writeSymmetricalLED(uint8_t index, uint32_t color)
{
  // For an odd strip count, make two pixels on one of the longer strips match
  strip.setPixelColor(index, color);
  strip.setPixelColor(strip.numPixels() - index - 1, color);
#if LED_COUNT % 2 != 0
  if (index == strip.numPixels() / 2 - 1)
  {
    strip.setPixelColor(strip.numPixels() / 2, color);
  }
#endif
}

/**
 * @brief drives the leds in a rainbow relative to the whole strip, but individual levels stack atop each other
 *
 * @param levels an array holding the # of leds to light per level
 * @param level_count the number of elements in the array of levels
 */
void rainbowLevel(float *levels, uint8_t level_count)
{
  sampleAudio = true;
  strip.clear();
  float levelSum = 0;
  for (uint8_t level = 0; level < level_count; level++)
  {
    levelSum += levels[level];
  }
  levelSum /= 3;

  uint8_t ledIndex = calcCap(levelSum, LED_COUNT / 2);
  for (uint8_t led = 0; led < ledIndex; led++)
  {
    writeSymmetricalLED(led, strip.gamma32(strip.ColorHSV((led)*65536L / (LED_COUNT / 2))));
  }
  strip.show();
}

/**
 * @brief breathing animation of a color
 *
 * @param r red value
 * @param g green value
 * @param b blue value
 */
void breathing(uint8_t r, uint8_t g, uint8_t b)
{
#define BREATHE_TIME_MS 5000.0f
#define OFF_TIME_MS 1000.0f
  sampleAudio = false;
  static uint32_t timeMarker = 0; // Last time a cycle was begun
  if (animEntry)
  {
    timeMarker = millis();
  }

  float cyclePortion = float(millis() - timeMarker) / BREATHE_TIME_MS; // How far into the current cycle
  if (cyclePortion < 0.5)
  {
    // First Cycle half
    strip.fill(strip.Color(2 * r * cyclePortion, 2 * g * cyclePortion, 2 * b * cyclePortion));
  }
  else if (cyclePortion >= 0.5 && cyclePortion <= 1.0)
  {
    // Second Cycle half
    strip.fill(strip.Color(2 * r * (1.0 - cyclePortion), 2 * g * (1.0 - cyclePortion), 2 * b * (1.0 - cyclePortion)));
  }
  else if (cyclePortion > 1.0 && cyclePortion <= 1.0 + (OFF_TIME_MS / BREATHE_TIME_MS))
  {
    strip.clear();
  }
  else
  {
    timeMarker = millis();
  }
  strip.show();
}

/*******************ANIMATION FUNCTIONS**********************/
void soundPulse()
{
  sampleAudio = true;
  uint8_t colors[3][3] = {{CYAN_RGB}, {GREEN_RGB}, {PURPLE_RGB}};

  for (uint8_t ledIndex = 0; ledIndex < LED_COUNT; ledIndex++)
  {
    uint8_t r = constrain((calcBrightCap(filteredBuckets[ledIndex % 3]) * colors[ledIndex % 3][0]) / 255, 0, 255);
    uint8_t g = constrain((calcBrightCap(filteredBuckets[ledIndex % 3]) * colors[ledIndex % 3][1]) / 255, 0, 255);
    uint8_t b = constrain((calcBrightCap(filteredBuckets[ledIndex % 3]) * colors[ledIndex % 3][2]) / 255, 0, 255);

    strip.setPixelColor(ledIndex, strip.Color(r, g, b));
  }
  strip.show();
}

void rainbowLevel()
{
#ifdef EN_FIR
  rainbowLevel(filteredBuckets, 3);
#else
  rainbowLevel(bucketAvg, 3);
#endif
}

void breathingCyan()
{
  breathing(CYAN_RGB);
}
void breathingGreen()
{
  breathing(GREEN_RGB);
}
void breathingPurple()
{
  breathing(PURPLE_RGB);
}
/**
 * @brief Does a rainbow going around in the ring
 *
 */
void rainbowCycle()
{
  sampleAudio = false;
  static long firstPixelHue = 0;
  if (animEntry)
  {
    firstPixelHue = 0; // On rentry of this animation, reset the wheel cycling
  }
  for (int i = 0; i < strip.numPixels(); i++)
  {
    int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
  }
  firstPixelHue += 256;
  strip.show();
  delay(10);
}

void (*anims[ANIM_COUNT])() = {
    rainbowLevel,
    breathingCyan,
    breathingGreen,
    breathingPurple,
    rainbowCycle};

/*****************MAIN FUNCTIONS***********/

void setup()
{
  // Serial.begin(9600);
  pinMode(PIN_BTN, INPUT_PULLUP);
  analogReadResolution(12);
  initLEDs();
}

void loop()
{

  // Brightness adjust cycle
  /*
  if (millis() - potReadMarker > POT_READ_MS)
  {
    strip.setBrightness(analogRead(PIN_ADJ) >> 4);
    potReadMarker = millis();
  }
  */

  // Only sample audio if animation function requires it
  if (sampleAudio)
  {
    // Sample audio
    takeData(micData);
    compactData(micData, bucketAvg);
  }

  // Run animation
  anims[animIndex]();
  animEntry = false;

  // Active Low button read
  if (!digitalRead(PIN_BTN))
  {
    delay(80);
    if (!digitalRead(PIN_BTN))
    {
      animIndex = (animIndex + 1) % ANIM_COUNT;
      strip.clear();
      animEntry = true;
      while (!digitalRead(PIN_BTN))
        ; // Halt untill button is raised
    }
  }
}