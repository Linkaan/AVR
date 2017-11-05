#include <Arduino.h>
#include <serializer.h>
#include <events.h>

/*
 * Hardware specifications used in voltage divider (see designator R10)
 */
#define R2 57.6f
#define Vin 3.3f
/* Reference: R-T Table Temperature Sensor 5K+10K
 * Coefficients for Steinhart–Hart used to calculate the temperature with DS18B20 thermistor.
 * They have been cut down to ten millionths
 */
#define A 0.0027713f
#define B 0.0002516f
#define C 0.0000003f

float lastTemperature;

/*
 * In order to calculate the median we need a function to sort an array.
 */
void sort_array(float *ar, int n) {
  if (n < 2)
    return;
  float p = ar[n >> 1];
  float *l = ar;
  float *r = ar + n - 1;
  while (l <= r) {
    if (*l < p) {
      l++;
    }
    else if (*r > p) {
      r--;
    }
    else {
      float t = *l;
      *l = *r;
      *r = t;
      l++;
      r--;
    }
  }
  sort_array(ar, r - ar + 1);
  sort_array(l, ar + n - l);
}

/*
 * We use Steinhart–Hart equation to calculate the temperature.
 */
float calculate_temperature(float read_value) {
    /*
     * In our voltage divider we have another resistor where the resistance is known,
     * so we can go back and calculcate the resistance of the sensor.
     */
    float logR1 = log((R2 * read_value)/(1023.0f - read_value));

    return 1.0f / (A + B * logR1 + C * (logR1*logR1*logR1)) - 273.15f; // Use the Steinhart–Hart equation to calculate the temperature.
}

/*
 * We fetch five samples and calculate the median to eleminate noise from sensor readings.
 */
float measure_median_temperature() {
  static const int samples = 5;
  float temperatures[samples];

  /*
   * Obtain five samples from the thermistor with 50 ms inbetween.
   */
  for (int i = 0; i < 5; i++) {
    temperatures[i] = analogRead(0); // Fetch the analog read value in the range of 0-1023
    delay(50);
  }

  /*
   * Sort the array with quicksort algorithm.
   */
  sort_array(temperatures, samples);
  return calculate_temperature(temperatures[2]);
}

int readFgeventFromSerial (struct fgevent *fgev)
{
    size_t serial_size, n;
    byte c;
    unsigned char header_buf[FGEVENT_HEADER_SIZE], *serial_buf;
    struct fgevent header;

    do
      {
        n = Serial.readBytes (&c, 1);
      }
    while (n > 0 && c != 0x02);

    if (n == 0) return 0;

    for (int i = 0; i < FGEVENT_HEADER_SIZE; i++)
      {
        n = Serial.readBytes (&c, 1);
        if (n < 0) break;

        header_buf[i] = (unsigned char) c;
      }

    if (n == 0) return 0;

    deserialize_fgevent_header (header_buf, &header);

    if (header.length > 0)
      {

        serial_size = FGEVENT_HEADER_SIZE + header.length;

        serial_buf = (unsigned char *) malloc (serial_size);
        if (serial_buf == NULL)
          {
            for (size_t i = 0; i < header.length + 1; i++)
              {
                n = Serial.readBytes (&c, 1);
                if (n < 0) break;
              }
            return -1;
          }

        for (size_t i = 0; i < serial_size; i++)
          {
            if (i < FGEVENT_HEADER_SIZE)
              {
                serial_buf[i] = header_buf[i];
                continue;
              }

            n = Serial.readBytes (&c, 1);
            if (n < 0) break;

            serial_buf[i] = (unsigned char) c;
          }

        if (n == 0)
          {
            free (serial_buf);
            return 0;
          }

        do
          {
            n = Serial.readBytes (&c, 1);
          }
        while (n > 0 && c != 0x03);

        deserialize_fgevent (serial_buf, fgev);
      }
    else
      {
        memcpy (fgev, &header, FGEVENT_HEADER_SIZE);
      }

    free (serial_buf);
    return 1;
}

void setup ()
{
    Serial.begin (9600);
}

void loop()
{
  if (Serial.available () > 0)
    {
      struct fgevent fgev;

      if (readFgeventFromSerial (&fgev) <= 0) return;

      if (fgev.id == FG_RETRIEVE_TEMP)
        {
          lastTemperature = measure_median_temperature (); 
      
          if (fgev.writeback)
            {
              struct fgevent ansev;    
              unsigned char buffer[FGEVENT_HEADER_SIZE + 6];
              size_t nbytes;
              int32_t temperatureX10;

              temperatureX10 = (int32_t)(lastTemperature * 10);

              ansev.id = FG_TEMP_RESULT;
              ansev.receiver = fgev.sender;
              ansev.writeback = 0;
              ansev.length = 1;
              ansev.payload = &temperatureX10;

              nbytes = 2;
              nbytes += FGEVENT_HEADER_SIZE;
              nbytes += sizeof (temperatureX10);

              buffer[0] = 0x02;
              serialize_fgevent (buffer+1, &ansev);
              buffer[nbytes-1] = 0x03;

              Serial.write (buffer, nbytes);
            }
        }

      if (fgev.length > 0) free (fgev.payload);
    }
}
