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

#define BUF_LEN 16


char buffer[BUF_LEN];
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
/*
void deserializeStringEvent (String serialized, struct fgevent *fgev)
{
    char buffer[serialized.length () + 1];
    serialized.toCharArray (buffer, sizeof (buffer));

    for (int i = 0; i < serialized.length (); i++) {
        Serial.print (String((int)buffer[i], HEX) + " ");
    }
    Serial.println();

    deserialize_fgevent (buffer, fgev);
}
*/
void parseEvent (char *buffer, struct fgevent *fgev)
{
    char *ptr;

    ptr = buffer;
    while ((size_t)(ptr - buffer) < BUF_LEN && ptr[0] != 0x02) // STX
            ptr++;

    // check if buffer is empty
    if ((size_t)(ptr - buffer) >= BUF_LEN)
        return;

    deserialize_fgevent (++ptr, fgev);
    /*
    int c;
    while (Serial.available () > 0 && (c = Serial.read ()) != 0x02)
        Serial.print (String((int)c, HEX) + " ");
    Serial.println();

    if (Serial.available () <= 0) return;

    Serial.println("Deserializing event\n");
    deserializeStringEvent (Serial.readStringUntil (0x03), fgev);*/
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
      //String serialized;
  
      /*serialized = Serial.readStringUntil (0x03);
      for (int i = 0; i < serialized.length (); i++) {
          Serial.print (String((int)serialized.charAt (i), HEX) + " ");
      }*/
      Serial.readBytesUntil (0x03, buffer, BUF_LEN);
      parseEvent (buffer, &fgev);
      //Serial.print("Well, I received something: ");
      //Serial.println(fgev.id);
      if (fgev.id == RETRIEVE_TEMP)
        {
          lastTemperature = measure_median_temperature (); 
      
          if (fgev.writeback)
            {
              struct fgevent ansev;    
              unsigned char buffer[sizeof (struct fgevent) +
                                   sizeof (lastTemperature) + 2];
              size_t nbytes;
              int32_t temperatureX10;

              temperatureX10 = (int32_t)(lastTemperature * 10);

              ansev.id = TEMP;
              ansev.writeback = 0;
              ansev.length = 1;
              ansev.payload = &temperatureX10;

              nbytes = 2;
              nbytes += sizeof (ansev.id);
              nbytes += sizeof (ansev.writeback);
              nbytes += sizeof (ansev.length);
              nbytes += sizeof (lastTemperature);

              buffer[0] = 0x02;
              serialize_fgevent (buffer+1, &ansev);
              buffer[nbytes-1] = 0x03;

              Serial.write (buffer, nbytes);
            }
        }

      if (fgev.length > 0) free (fgev.payload);
    }
}
