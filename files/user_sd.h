#ifndef USER_SD_H
#define USER_SD_H

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "data_struct.h"
#include <stdint.h>
#include <cstring>

#define FILE_PATH "/data_log.txt"

#define SD_BUFFER_SIZE 200

#define CS_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

void sd_init(fs::FS &fs, const char *path);
void sd_write_datas(fs::FS &fs, const char *path, data_pack_t *veriler_);
void sd_read_all(fs::FS &fs, const char *path);

#endif