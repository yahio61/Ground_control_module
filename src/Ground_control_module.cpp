#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "user_sd.h"
#include "data_struct.h"

#define LCD Serial1
#define LORA Serial2

#define LORA_BAUD 115200
#define LCD_BAUD 9600
#define SERIAL_BAUD 115200

#define LORA_M0 21
#define LORA_M1 22

#define BT1 25
#define BT2 26
#define BT3 27
#define SW1 4

uint8_t broadcastAddress[] = { 0x68, 0x25, 0xDD, 0x23, 0x79, 0xF4 };  //receiver
char data_received[100];
int counter = 0;
char data[100];
char buf[128];

String success;

data_pack_t veriler;
esp_now_peer_info_t peerInfo;

void Lcd_send(char *data);
void printDatasPacked(data_pack_t *veriler);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

void setup() {
  pinMode(BT1, INPUT);
  pinMode(BT2, INPUT);
  pinMode(BT3, INPUT);
  pinMode(SW1, INPUT);
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);

  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);

  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  LORA.begin(LORA_BAUD);
 /* 
  Serial.begin(9600);
  LORA.begin(9600);
  digitalWrite(LORA_M1, HIGH);

  while (1) {
    
    if (LORA.available()) {
      Serial.write(LORA.read());
    }
    if (Serial.available()) {
      LORA.write(Serial.read());
    }
  }
*/
  LORA.setTimeout(100);

  LCD.begin(9600, SERIAL_8N1, 32, 33);  // RX = GPIO32, TX = GPIO33

  Serial.begin(SERIAL_BAUD);
  delay(100);
  sd_init(SD, FILE_PATH);
}

void loop() {
  int length = LORA.readBytes(veriler.arr, 64);

  if (length == 64 && veriler.dataYapi.CR == '\r' && veriler.dataYapi.LF == '\n' && calculateCRC(&veriler) == veriler.dataYapi.checkSum && veriler.dataYapi.basla == 0xFF) {
    printDatasPacked(&veriler);
    esp_now_send(broadcastAddress, (uint8_t *)&veriler, sizeof(veriler));
    sd_write_datas(SD, FILE_PATH, &veriler);

    sprintf(data, "t1.txt=\"%.6f\"", veriler.dataYapi.lat);
    Lcd_send(data);
    sprintf(data, "t2.txt=\"%.6f\"", veriler.dataYapi.lon);
    Lcd_send(data);
    sprintf(data, "n0.val=%.0f", veriler.dataYapi.yukseklik_p);
    Lcd_send(data);
    sprintf(data, "n1.val=%d", veriler.dataYapi.maxAltitude);
    Lcd_send(data);
    sprintf(data, "n2.val=%.0f", veriler.dataYapi.aci);
    Lcd_send(data);
    sprintf(data, "t17.txt=\"%d:%d\"", veriler.dataYapi.zaman >> 2, (veriler.dataYapi.zaman & 0x03) << 4 | veriler.dataYapi.durum >> 4);
    Lcd_send(data);
    sprintf(data, "t16.txt=\"%.1f\"", ((float)(int)veriler.dataYapi.sicaklik) * 5.0 / 10.0);
    Lcd_send(data);
    sprintf(data, "t15.txt=\"%.2f\"", ((float)veriler.dataYapi.voltaj) / 100.0);
    Lcd_send(data);
  }
}

void Lcd_send(char *data) {
  LCD.write(data);
  LCD.write(0xFF);
  LCD.write(0xFF);
  LCD.write(0xFF);
}

void printDatasPacked(data_pack_t *veriler) {
  Serial.write(veriler->arr, sizeof(veriler->arr));
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == 0) {
    success = "Delivery Success :)";
  } else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(data_received, incomingData, len);
}


