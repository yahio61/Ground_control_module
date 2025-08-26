#include <stdint.h>
#include "esp32-hal-gpio.h"
#include <cstring>
#include <sys/_stdint.h>
#include "esp32-hal.h"
#include "HardwareSerial.h"
#include "lora_e22.h"

int lora_e22_init(e22_conf_struct_t *lora_conf_struct)
{
  uint8_t lora_configuration_packet[LORA_PACKET_SIZE];
  lora_e22_configuration_mode();
  LORA.end();
  LORA.begin(9600, SERIAL_8N1, LORA_TX, LORA_RX);
  LORA.setTimeout(100);

	lora_configuration_packet[0] = 0xC0;	//Set register command.
	lora_configuration_packet[1] = 0x03;	//Starting from byte 0x03
	lora_configuration_packet[2] = 0x04;	//4 bytes will be configured.
  //Wireless air data rate（bps）, Serial parity bit, UART Serial port rate（bps).
	lora_configuration_packet[3] = lora_conf_struct->air_rate | lora_conf_struct->parity_bit << 3 | lora_conf_struct->baud_rate << 5;
  //Transmitting power, RSSI anbient noise enable, Sub packet settings.
	lora_configuration_packet[4] = lora_conf_struct->power | lora_conf_struct->rssi_noise << 5 | lora_conf_struct->packet_size << 6;
  //channel 0-83 (410.125 + CH *1M)
  if(lora_conf_struct->channel > 83)
  {
    lora_conf_struct->channel = 83;
  }
  else if(lora_conf_struct->channel < 0)
  {
    lora_conf_struct->channel = 0;
  }
	lora_configuration_packet[5] = lora_conf_struct->channel;
  //WOR cycle time, WOR transceiver control, LBT enable, Repeater function, Transmission mode, Enable RSSI.
  lora_configuration_packet[6] = lora_conf_struct->wor_cycle | lora_conf_struct->wor << 3 | lora_conf_struct->lbt << 4 | lora_conf_struct->repeater_func << 5 | lora_conf_struct->mode << 6 | lora_conf_struct->rssi_enable << 7;

  uint8_t response[LORA_PACKET_SIZE] = {0};
  for(uint8_t i = 0; i < 10; i++)
  {
    LORA.write(lora_configuration_packet, LORA_PACKET_SIZE);
    LORA.readBytes(response, LORA_PACKET_SIZE);
    if(memcmp(&response[1], &lora_configuration_packet[1], LORA_PACKET_SIZE - 1) == 0)
    {
      //Serial.printf("config: %x %x %x %x %x %x %x\n\r", lora_configuration_packet[0], lora_configuration_packet[1], lora_configuration_packet[2], lora_configuration_packet[3], lora_configuration_packet[4], lora_configuration_packet[5], lora_configuration_packet[6]);
      //Serial.printf("response: %x %x %x %x %x %x %x\n\r", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
      lora_e22_transmission_mode();
      delay(50);
      LORA.end();
      LORA.begin(115200, SERIAL_8N1, LORA_TX, LORA_RX);
      delay(100);
      return 0;
    }
  }
  lora_e22_transmission_mode();
  delay(50);
  LORA.end();
  LORA.begin(115200, SERIAL_8N1, LORA_TX, LORA_RX);
  delay(100);
  return 1;
}

void lora_e22_transmission_mode(void)
{
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(20);
}

void lora_e22_configuration_mode(void)
{
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, HIGH);
  delay(100);
}

void lora_e22_configure(void)
{
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, HIGH);
  LORA.end();
  Serial.begin(9600);
  LORA.begin(9600, SERIAL_8N1, LORA_TX, LORA_RX);
  delay(20);
  while(1)
  {
    if(LORA.available())
    {
      Serial.write(LORA.read());
    }
    if(Serial.available())
    {
      LORA.write(Serial.read());
    }
  }
}