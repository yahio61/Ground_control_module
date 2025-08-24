#include "user_sd.h"

static uint8_t is_sd_available;

void sd_init(fs::FS &fs, const char *path)
{
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  if(!SD.begin(CS_PIN))
  {
    Serial.println("Sd kart baslatılamadı");
    is_sd_available = 0;
    return;
  }
  else
  {
    is_sd_available = 1;
  }

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file");
    return;
  }
  
  if(file.print("\r\n\r\nDATA LOGGER STARTED\r\nmin,sec,stat,volt,mWatt/s,temp,hum,alt,altGps,lat,lon,angle,pitch,roll,yaw,sat,velocity,ivmeX,ivmeY,ivmeZ,CRC\r\n"))
  {
    Serial.println("Init succeed");
  }
  else
  {
    Serial.println("Init failed");
  }

  file.close();
}

void sd_write_datas(fs::FS &fs, const char *path, data_pack_t *veriler_)
{
  if(is_sd_available == 1)
  {
    char sd_buffer[SD_BUFFER_SIZE];
    memset(sd_buffer, 0, SD_BUFFER_SIZE);
    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
      Serial.println("Failed to open file for appending");
      return;
    }
    snprintf(sd_buffer, SD_BUFFER_SIZE, "%d,%d,%d,%.2f,%d,%.1f,%d,%.1f,%.1f,%f,%f,%.1f,%d,%d,%d,%d,%.1f,%.3f,%.3f,%.3f,%d\r\n",
          (veriler_->dataYapi.zaman >> 2), ((veriler_->dataYapi.zaman & 3) << 4) | (veriler_->dataYapi.durum >> 4),
          veriler_->dataYapi.durum & 0x0F, ((float)veriler_->dataYapi.voltaj) / 100 ,
          veriler_->dataYapi.akim , (float)veriler_->dataYapi.sicaklik / 5, veriler_->dataYapi.nem,
          veriler_->dataYapi.yukseklik_p, veriler_->dataYapi.yukseklik_gps, veriler_->dataYapi.lat,
          veriler_->dataYapi.lon, veriler_->dataYapi.aci, (veriler_->dataYapi.pitch * ((veriler_->dataYapi.uyduSayisi & 0x04)?-1:1)),
          (veriler_->dataYapi.roll * (((veriler_->dataYapi.uyduSayisi & 0x02)?-1:1))),
          (veriler_->dataYapi.yaw * ((veriler_->dataYapi.uyduSayisi & 0x01)?-1:1)),
          veriler_->dataYapi.uyduSayisi >> 3, (float)veriler_->dataYapi.hiz / 10,
          veriler_->dataYapi.accX, veriler_->dataYapi.accY,
          veriler_->dataYapi.accZ, veriler_->dataYapi.checkSum);
    if (file.print(sd_buffer))
    {
      Serial.println("Message appended");
    } else {
      Serial.println("Append failed");
    }
    file.close();
  }
}

void sd_read_all(fs::FS &fs, const char *path)
{
  if(is_sd_available == 1)
  {
    File file = fs.open(path, FILE_READ);
    if (!file)
    {
      Serial.println("Failed to open file for reading");
      return;
    }

    Serial.print("\r\nRead from file:\r\n");
    while(file.available())
    {
      String line = file.readStringUntil('\n');
      Serial.println(line);
    }
    file.close();
  }
}