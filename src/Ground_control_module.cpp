#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "user_sd.h"
#include "data_struct.h"
#include "lora_e22.h"
#include <Preferences.h>


#define LCD Serial1
#define LORA Serial2

#define LORA_BAUD 115200
#define LCD_BAUD 9600
#define SERIAL_BAUD 115200

#define BT1 25
#define BT2 26
#define BT3 27
#define SW1 4

#define LCD_TX  32
#define LCD_RX  33

uint16_t air_rates[] = {2400, 4800, 9600, 19200, 38400, 62500};
uint8_t esp_now_status = 1;
uint8_t received_lcd[10] = {0};
uint8_t broadcastAddress[] = { 0x68, 0x25, 0xDD, 0x23, 0x79, 0xF4 };  //receiver
char data_received[100];
int counter = 0;
char data[100];
char buf[128];

String success;
e22_conf_struct_t lora_1;
data_pack_t veriler;
esp_now_peer_info_t peerInfo;
Preferences prefs;

void Lcd_send(char *data);
void printDatasPacked(data_pack_t *veriler);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void lora_begin();

void setup() {
  pinMode(BT1, INPUT);
  pinMode(BT2, INPUT);
  pinMode(BT3, INPUT);
  pinMode(SW1, INPUT);
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  lora_e22_transmission_mode();
  
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  prefs.begin("vals", false);

  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  LORA.begin(115200, SERIAL_8N1, LORA_TX, LORA_RX);
  LORA.setTimeout(100);
  lora_begin();

  LCD.begin(9600, SERIAL_8N1, LCD_TX, LCD_RX);
  LCD.setTimeout(100);
  Serial.begin(SERIAL_BAUD);
  

  while(digitalRead(SW1))
  {
    //lora_e22_configure();
    if(LCD.readBytes(received_lcd,5))
    {
      sprintf(data, "n0.val=%d", (uint16_t)prefs.getInt("chan", 0) + 410);
      Lcd_send(data);
      sprintf(data, "n5.val=%d", air_rates[prefs.getInt("rate", 0)]); 
      Lcd_send(data);
      data[0] = '\0';
      if(!memcmp("setc", received_lcd, 4))
      {
        uint8_t lora_chan = received_lcd[4];
        lora_1.channel = lora_chan;
        prefs.putInt("chan", lora_chan);
        if(lora_e22_init(&lora_1))
        {
          lora_chan = 0;
          prefs.putInt("chan", 0);
        }
        sprintf(data, "n0.val=%d", (uint16_t)prefs.getInt("chan", 0) + 410);
      }
      else if(!memcmp("setr", received_lcd, 4))
      {
        uint8_t lora_air_rate = received_lcd[4];
        lora_1.air_rate = lora_air_rate + 2;
        prefs.putInt("rate", lora_air_rate);
        if(lora_e22_init(&lora_1))
        {
          lora_air_rate = 0;
          prefs.putInt("rate", 0);
        }
        sprintf(data, "n5.val=%d", air_rates[prefs.getInt("rate", 0)]); 
      }
      else if(!memcmp("espb", received_lcd, 4))
      {
        esp_now_status = received_lcd[4];
        if(esp_now_status)
        {
          sprintf(data, "t4.txt=\"%s\"", "on");
        }
        else
        {
          sprintf(data, "t4.txt=\"%s\"", "off");
        }
      }
      else if(!memcmp("page1", received_lcd, 5))
      {
        //nothing rigth now.
      }
      Lcd_send(data);
    }
  }
  delay(100);
  sd_init(SD, FILE_PATH);
}

void loop() {
  

  int length = LORA.readBytes(veriler.arr, 64);

  if (length == 64 && veriler.dataYapi.CR == '\r' && veriler.dataYapi.LF == '\n' && calculateCRC(&veriler) == veriler.dataYapi.checkSum && veriler.dataYapi.basla == 0xFF) {
    printDatasPacked(&veriler);
    if(esp_now_status)
    {
      esp_now_send(broadcastAddress, (uint8_t *)&veriler, sizeof(veriler));
    }
    sd_write_datas(SD, FILE_PATH, &veriler);

    sprintf(data, "t10.txt=\"%.6f\"", veriler.dataYapi.lat);
    Lcd_send(data);
    sprintf(data, "t11.txt=\"%.6f\"", veriler.dataYapi.lon);
    Lcd_send(data);
    sprintf(data, "n1.val=%.0f", veriler.dataYapi.yukseklik_p);
    Lcd_send(data);
    sprintf(data, "n2.val=%d", veriler.dataYapi.maxAltitude);
    Lcd_send(data);
    sprintf(data, "n0.val=%.0f", veriler.dataYapi.aci);
    Lcd_send(data);
    sprintf(data, "t1.txt=\"%d:%d\"", veriler.dataYapi.zaman >> 2, (veriler.dataYapi.zaman & 0x03) << 4 | veriler.dataYapi.durum >> 4);
    Lcd_send(data);
    sprintf(data, "t12.txt=\"%.1f\"", ((float)(int)veriler.dataYapi.sicaklik) * 5.0 / 10.0);
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


void lora_begin()
{
  lora_1.baud_rate 		  = E22_BAUD_RATE_115200;
	lora_1.parity_bit		  =	E22_PARITY_8N1;
	lora_1.air_rate			  =	prefs.getInt("rate", 0) + 2;
	lora_1.packet_size	  =	E22_PACKET_SIZE_64;
	lora_1.rssi_noise		  =	E22_RSSI_NOISE_DISABLE;
	lora_1.power			    =	E22_TRANSMITTING_POWER_30dBm;
	lora_1.rssi_enable	  =	E22_ENABLE_RSSI_DISABLE;
	lora_1.mode				    = E22_TRANSMISSION_MODE_TRANSPARENT;
	lora_1.repeater_func  =	E22_REPEATER_FUNC_DISABLE;
	lora_1.lbt				    =	E22_LBT_DISABLE;
	lora_1.wor				    =	E22_WOR_RECEIVER;
	lora_1.wor_cycle		  =	E22_WOR_CYCLE_1000;
	lora_1.channel			  =	prefs.getInt("chan", 0);

	lora_e22_init(&lora_1);
}