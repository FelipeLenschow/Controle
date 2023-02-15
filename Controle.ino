#include <esp_now.h>
#include <WiFi.h>
#include <PS4Controller.h>
#include <esp_coexist.h>
//#include <Math.h>
//C:\Users\felip\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.6\tools\sdk
//https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/esp32/sdkconfig
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/coexist.html

#define QuarterPi 0.78539816339
#define HalfPi 1.57079632679
#define ThreeQuarterPi 2.35619449019

bool arma_dir = false;
int8_t X, Y;

//uint8_t broadcastAddress[] = {0xC4, 0x5B, 0xBE, 0x61, 0xFA, 0x3A}; // Retransmissor
uint8_t broadcastAddress[] = {0xCC, 0x50, 0xE3, 0x56, 0xAD, 0xF4}; // Robo
//uint8_t broadcastAddress[] = {0x48, 0x55, 0x19, 0x00, 0x4A, 0xAF}; // Node

typedef struct struct_message
{
  signed int motor_e;
  signed int motor_d;
  uint8_t arma_vel;
  byte led[2][3];
} struct_message;

struct_message Controle;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/// Converter valores para direção do motor
void ConverterMotores(signed int X, signed int Y)
{
  if (X != 0)
  {
    double YdX = 1000 * Y / X;
    float Angle = atan(YdX / 1000);
    if (X < 0)
    {
      if (Y > 0)
        Angle += PI;
      else
        Angle -= PI;
    }

    float Module = sqrt(X * X + Y * Y);
    signed int Max_Y = 0;
    signed int Max_X = 0;

    if (abs(X) > abs(Y))
    {
      Max_X = 255;
      Max_Y = 255 * tan(Angle);
    }
    else
    {
      Max_Y = 255;
      Max_X = 255 / tan(Angle);
    }

    int Max_Module = sqrt(Max_X * Max_X + Max_Y * Max_Y);
    int vel = constrain(255 * Module / Max_Module, 0, 255);

    Controle.motor_d = vel * sin(HalfPi * cos(Angle - ThreeQuarterPi));
    Controle.motor_e = vel * sin(HalfPi * cos(Angle - QuarterPi));
  }
}

void setup() {

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  PS4.begin("2C:DC:D7:0A:0D:4C");
  PS4.setFlashRate(1,1);
  Serial.println("PS4 controller ready.");
  
  
}

void loop() {
  if (PS4.isConnected()) {
    ConverterMotores(PS4.LStickX(), PS4.LStickY());

    if (arma_dir)
      Controle.arma_vel = map(PS4.R2Value(), 0, 255, 90, 180);
    else
      Controle.arma_vel = map(PS4.R2Value(), 0, 255, 90, 0);
    //PS4.setLed(r, g, b);

    Serial.print(Controle.motor_e); Serial.print("  ");
    Serial.print(Controle.motor_d); Serial.print("  ");
    Serial.println(Controle.arma_vel);
    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
    esp_now_send(broadcastAddress, (uint8_t *) &Controle, sizeof(Controle));
    esp_now_send(broadcastAddress, (uint8_t *) &Controle, sizeof(Controle));
    esp_now_send(broadcastAddress, (uint8_t *) &Controle, sizeof(Controle));
    esp_now_send(broadcastAddress, (uint8_t *) &Controle, sizeof(Controle));
  }
  delay(2);
}
