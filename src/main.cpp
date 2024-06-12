#include <Arduino.h>
#include <TFT_eSPI.h>
#include <HttpClient.h>
#include <WiFi.h>
#include <inttypes.h>
#include <stdio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>


char ssid[50]; // your network SSID (name)
char pass[50]; // your network password 

// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30 * 1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;

#define PH_PIN 33
#define TdsSensorPin 32
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT 20         // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;
float calibration_value = 34.8;
float ph_val = 0;
int buffer_arr[10],temp;
unsigned long int avgval; 

TFT_eSPI tft = TFT_eSPI();
int ounces = 0;
bool pressed = false;

void nvs_access() {
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
    err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  
  // Open
  Serial.printf("\n");
  Serial.printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    Serial.printf("Done\n");
    Serial.printf("Retrieving SSID/PASSWD\n");
    size_t ssid_len;
    size_t pass_len;
    err = nvs_get_str(my_handle, "ssid", ssid, &ssid_len);
    err |= nvs_get_str(my_handle, "pass", pass, &pass_len);
    switch (err) {
      case ESP_OK:
        Serial.printf("Done\n");
        //Serial.printf("SSID = %s\n", ssid);
        //Serial.printf("PASSWD = %s\n", pass);
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        Serial.printf("The value is not initialized yet!\n");
        break;
      default:
        Serial.printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
  }

// Close
nvs_close(my_handle);
}

void display_stats(float TDS, float ph) {
  //display stats on lcd screen
  tft.setCursor(0, 0, 1); 
  tft.setTextColor(TFT_YELLOW,TFT_BLACK);
  tft.setTextSize(3);
  tft.println("Hydro");
  tft.println("Helper");
  tft.setTextSize(2);
  tft.println(" ");
  tft.println("TDS:");
  tft.println(TDS);
  tft.println(" ");
  tft.println("pH:");
  tft.println(ph);
  tft.println(" ");
  tft.setTextSize(1);
  tft.println("Daily Oz:");
  tft.println(ounces);
  tft.println("TDS:");
  tft.println(TDS);
  tft.println("pH ");
  tft.println(ph);
  tft.print("Add Oz         Reset");
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

float get_ph() {
  //get 10 ph value samples
  for(int i=0;i<10;i++) {
    buffer_arr[i]=analogRead(PH_PIN);
    delay(30);
  }
  //sort values in ascending order
  for(int i=0;i<9;i++) {
    for(int j=i+1;j<10;j++) {
      if(buffer_arr[i]>buffer_arr[j]) {
        temp=buffer_arr[i];
        buffer_arr[i]=buffer_arr[j];
        buffer_arr[j]=temp;
        }
    }
  }
  avgval=0;
  for(int i=2;i<8;i++) avgval+=buffer_arr[i];
  float volt=(float)avgval*3.0/1024/6;
  float ph_act = -5.70 * volt + calibration_value;
  return ph_act;
}
void setup() {
  Serial.begin(9600);
  tft.init();
  tft.fillScreen(TFT_BLACK);
  pinMode(0,INPUT_PULLUP);
  pinMode(35,INPUT_PULLUP);
  
  delay(1000);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  delay(500);

  // Retrieve SSID/PASSWD from flash before anything else
  nvs_access();
  delay(300);
  // We start by connecting to a WiFi network
   Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());

  pinMode(TdsSensorPin, INPUT);
}

void loop() {
  //left button - add oz  
  if (digitalRead(0) == 0 && pressed == false) {
    ounces++;
    pressed = true;
  } 
  if (digitalRead(0) == 1) {
    pressed = false;
  }
  //right button - reset oz
  if (digitalRead(35) == 0) {
    ounces = 0;
    tft.fillScreen(TFT_BLACK);
  }

  //read and calculate tds value from sensor

  //every 40 milliseconds,read the analog value from the ADC
  for (int i = 0; i< SCOUNT; i++)
  {
    analogBuffer[i] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    delay(15);
  }

    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value

    Serial.print("TDS----Value:");
    Serial.print(tdsValue);
    Serial.println("ppm");
  
  ph_val = get_ph();
  Serial.println("pH Val: ");
  Serial.println(ph_val);

  display_stats(tdsValue,ph_val);
  //convert data to send to server
  std::string query = "/?tds=" + std::to_string(tdsValue) + "&ph=" + std::to_string(ph_val) + "&oz=" + std::to_string(ounces);
  char data[query.length()+1];
  strcpy(data,query.c_str());

  int err = 0;
  WiFiClient c;
  HttpClient http(c);
  //err = http.get(kHostname, kPath);
  err = http.get("18.188.57.94", 5000, data, NULL);
  if (err == 0) {
    Serial.println("startedRequest ok");
    err = http.responseStatusCode();
    if (err >= 0) {
      Serial.print("Got status code: ");
      Serial.println(err);
  // Usually you'd check that the response code is 200 or a
  // similar "success" code (200-299) before carrying on,
  // but we'll print out whatever response we get
      err = http.skipResponseHeaders();
      if (err >= 0) {
      int bodyLen = http.contentLength();
      Serial.print("Content length is: ");
      Serial.println(bodyLen);
      Serial.println();
      Serial.println("Body returned follows:");
      // Now we've got to the body, so we can print it out
      unsigned long timeoutStart = millis();
      char c;
      // Whilst we haven't timed out & haven't reached the end of the body
      while ((http.connected() || http.available()) &&
        ((millis() - timeoutStart) < kNetworkTimeout)) {
        if (http.available()) {
          c = http.read();
          // Print out this character
          Serial.print(c);
          bodyLen--;
          // We read something, reset the timeout counter
          timeoutStart = millis();
        } else {
        // We haven't got any data, so let's pause to allow some to
        // arrive
         delay(kNetworkDelay);
        }
      }
      } else {
        Serial.print("Failed to skip response headers: ");
        Serial.println(err);
      }
    } else {
      Serial.print("Getting response failed: ");
      Serial.println(err);
    }
  } else {
    Serial.print("Connect failed: ");
    Serial.println(err);
  }
  http.stop();

}
