/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "kcordova"
#define IO_KEY       "aio_pXwh41j0ej8Gn40jcAR1x4L5l8ls"
/******************************* WIFI **************************************/
#define WIFI_SSID "Karen "
#define WIFI_PASS "pdcq3570"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"


AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

