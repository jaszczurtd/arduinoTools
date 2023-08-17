//
//  utils.c
//  Index
//
//  Created by Marcin Kielesiński on 07/12/2019.
//

#include "tools.h"

static bool loggerInitialized = false;
static bool crashLoggerInitialized = false;
static bool SDStarted = false;
static File loggerFile;
static File crashFile;
static SPISettings settingsA(1000000, MSBFIRST, SPI_MODE1);
static String logBuffer = "";

int getSDLoggerNumber(void) {
#ifdef SD_LOGGER
  return readAT24Int(EEPROM_LOGGER_ADDR);
#else
  return -1;
#endif
}

#ifndef AT24C256
static bool eepromStarted = false;
#endif
void startRaspberryEEPROM(int size) {
  #ifndef AT24C256
  if(!eepromStarted) {
    EEPROM.begin(size);
    eepromStarted = true;
  }
  #endif
}

//cs: chip select/SD addres
bool initSDLogger(int cs) {
#ifdef SD_LOGGER

  char buf[128] = {0};

  #ifndef AT24C256
  startRaspberryEEPROM(RASPBERRY_EEPROM_SIZE);
  #endif

  int logNumber = getSDLoggerNumber();
  snprintf(buf, sizeof(buf) - 1, "log%d.txt", logNumber);
  logNumber++;
  writeAT24Int(EEPROM_LOGGER_ADDR, logNumber);

  SPI.beginTransaction(settingsA);
  if(!SDStarted) {
    SDStarted = loggerInitialized = SD.begin(cs);
  } else {
    loggerInitialized = SDStarted;
  }
  if(loggerInitialized) {
    loggerFile = SD.open(buf, FILE_WRITE);
    if(!loggerFile) {
      loggerInitialized = false;
    }
  }
  SPI.endTransaction();

#endif
  return loggerInitialized;
}

bool isSDLoggerInitialized(void) {
  return loggerInitialized;
}

static unsigned long lastWriteTime = 0; 
void updateSD(String data) {
  if(isSDLoggerInitialized()) {  
    logBuffer += data;
    logBuffer += "\n";
    unsigned long currentTime = millis();
    if (currentTime - lastWriteTime >= WRITE_INTERVAL) {
      lastWriteTime = currentTime;

      SPI.beginTransaction(settingsA);
      loggerFile.println(logBuffer);
      loggerFile.flush();
      SPI.endTransaction();
      logBuffer = "";
    }
  }
}

void saveLoggerAndClose(void) {
  if(isSDLoggerInitialized()) {  
    loggerInitialized = false;
    SPI.beginTransaction(settingsA);
    loggerFile.println(logBuffer);
    loggerFile.flush();
    loggerFile.close();
    SPI.endTransaction();
    logBuffer = "";
  }
}

int getSDCrashNumber(void) {
#ifdef SD_LOGGER
  return readAT24Int(EEPROM_CRASH_ADDR);
#else
  return -1;
#endif
}

//cs: chip select/SD addres
bool initCrashLogger(const char *addToName, int cs) {
#ifdef SD_LOGGER

  char buf[128] = {0};

  #ifndef AT24C256
  startRaspberryEEPROM(RASPBERRY_EEPROM_SIZE);
  #endif

  int crashNumber = getSDCrashNumber();
  if(addToName != NULL && strlen(addToName) > 0) {
    snprintf(buf, sizeof(buf) - 1, "watchdog%d(%s).txt", crashNumber, addToName);
  } else {
    snprintf(buf, sizeof(buf) - 1, "watchdog%d.txt", crashNumber);
  }
  crashNumber++;

  writeAT24Int(EEPROM_CRASH_ADDR, crashNumber);

  SPI.beginTransaction(settingsA);

  if(!SDStarted) {
    SDStarted = crashLoggerInitialized = SD.begin(cs);
  } else {
    crashLoggerInitialized = SDStarted;
  }

  if(crashLoggerInitialized) {
    crashFile = SD.open(buf, FILE_WRITE);
    if(!crashFile) {
      crashLoggerInitialized = false;
    }
  }
  SPI.endTransaction();

  if(crashLoggerInitialized) {
    snprintf(buf, sizeof(buf) - 1, "corresponded log file: log%d.txt", 
      getSDLoggerNumber() - 1);

    updateCrashReport(buf);
  }

#endif
  return crashLoggerInitialized;
}

bool isCrashLoggerInitialized(void) {
  return crashLoggerInitialized;
}

void updateCrashReport(String data) {
#ifdef SD_LOGGER
  if(isCrashLoggerInitialized()) {  
    SPI.beginTransaction(settingsA);
    crashFile.println(data + "\n");
    crashFile.flush();
    SPI.endTransaction();
  }
#endif
}

void saveCrashLoggerAndClose(void) {
#ifdef SD_LOGGER
  if(isCrashLoggerInitialized()) {  
    crashLoggerInitialized = false;
    SPI.beginTransaction(settingsA);
    crashFile.flush();
    crashFile.close();
    SPI.endTransaction();
  }
#endif
}

void crashReport(const char *format, ...) {
#ifdef SD_LOGGER
  if(isCrashLoggerInitialized()) {  
    va_list valist;
    va_start(valist, format);

    char buffer[128];
    memset (buffer, 0, sizeof(buffer));
    vsnprintf(buffer, sizeof(buffer) - 1, format, valist);

    updateCrashReport(buffer);

    va_end(valist);
  }
#endif
}

NOINIT static char deb_buffer[128];
void deb(const char *format, ...) {

  rp2040.idleOtherCore();

  va_list valist;
  va_start(valist, format);

  memset (deb_buffer, 0, sizeof(deb_buffer));
  vsnprintf(deb_buffer, sizeof(deb_buffer) - 1, format, valist);
  Serial.println(deb_buffer);

#ifdef SD_LOGGER
  updateSD(deb_buffer);
#endif

  va_end(valist);

  rp2040.resumeOtherCore();
}

NOINIT static char derr_buffer[128];
void derr(const char *format, ...) {

  rp2040.idleOtherCore();

  va_list valist;
  va_start(valist, format);

  memset (derr_buffer, 0, sizeof(derr_buffer));

  const char *error = "ERROR! ";
  int len = strlen(error);

  strcpy(derr_buffer, error);

  vsnprintf(derr_buffer + len, sizeof(derr_buffer) - 1 - len, format, valist);
  Serial.println(derr_buffer);

#ifdef SD_LOGGER
  updateSD(derr_buffer);
#endif

  va_end(valist);

  rp2040.resumeOtherCore();
}

void floatToDec(float val, int *hi, int *lo) {
	int t1 = (int)val;
	if(t1 > -128) {
		if(hi != NULL) {
			*hi = t1;
		}
		int t2 = (int) (((float)val - t1) * 10);
		if(lo != NULL) {
			if(t2 >= 0) {
				*lo = t2;
			} else {
				*lo = 0;
			}
		}
	}
}

float adcToVolt(int adc, float r1, float r2) {
  const float V_REF = 3.3;
  const float V_DIVIDER_SCALE = (r1 + r2) / r2;

  return adc * (V_REF / pow(2, ADC_BITS)) * V_DIVIDER_SCALE;
}

float getAverageValueFrom(int tpin) {

    uint8_t i;
    float average = 0;

    // take N samples in a row, with a slight delay
    for (i = 0; i < NUMSAMPLES; i++) {
        average += analogRead(tpin);
        delay(1);
    }
    average /= NUMSAMPLES;

    return average;
}

float getAverageForTable(int *idx, int *overall, float val, float *table) {

  table[(*idx)++] = val;
  if(*idx > TEMPERATURE_TABLES_SIZE - 1) {
    *idx = 0;
  }
  (*overall)++;
  if(*overall >= TEMPERATURE_TABLES_SIZE - 1) {
    *overall = TEMPERATURE_TABLES_SIZE - 1;
  }

  float average = 0;
  for (int i = 0; i < *overall; i++) {
      average += table[i];
  }
  average /= *overall;
  return average;
}

float ntcToTemp(int tpin, int thermistor, int r) {

    float average = getAverageValueFrom(tpin);

    // convert the value to resistance
    average = 4095.0 / average - 1;
    average = r / average;

    float steinhart;
    steinhart = average / thermistor;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert absolute temp to C    

    return steinhart;
}

int percentToGivenVal(float percent, int givenVal) {
    return int(((percent / 100.0) * givenVal));
}

#ifdef I2C_SCANNER

unsigned int loopCounter = 0;
static bool t = false;
void i2cScanner(void) {
  byte error, address;
  int nDevices;
 
  while(true) {
    Serial.print("Scanning ");
    Serial.print(loopCounter++);
    Serial.print("...\n");
  
    nDevices = 0;
    for(address = 1; address < 127; address++ ) {
      watchdog_update();
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
  
      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        Serial.println("  !");
  
        nDevices++;
      }
      else if (error == 4) {
        Serial.print("Unknown error at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.println(address, HEX);
      }    
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");
  
    delay(500);           // wait 500 mseconds for next scan
  }

}
#endif

unsigned long getSeconds(void) {
  return ((millis() + 500) / 1000);
}

byte MSB(unsigned short value){
  return (byte)(value >> 8) & 0xFF;
}

byte LSB(unsigned short value){
  return (byte)(value & 0x00FF);
}

unsigned short byteArrayToWord(unsigned char* bytes) {
  unsigned short word = ((unsigned short)bytes[0] << 8) | bytes[1];
  return word;
}

void wordToByteArray(unsigned short word, unsigned char* bytes) {
  bytes[0] = MSB(word);
  bytes[1] = LSB(word);
}

bool isWireBusy(unsigned int dataAddress) {
  Wire.beginTransmission(dataAddress);
  return Wire.endTransmission();
}

void resetEEPROM(void) {
#ifndef AT24C256
  if(!eepromStarted) {
    startRaspberryEEPROM(RASPBERRY_EEPROM_SIZE);
    for(int a = 0; a < RASPBERRY_EEPROM_SIZE; a++) {
      EEPROM.write(a, 0);
    }
    EEPROM.commit();
  }
#else
  for(int a = 0; a < AT24C256_EEPROM_SIZE; a++) {
    writeAT24(a, 0);
    watchdog_update();
  }
#endif
}

void writeAT24(unsigned int dataAddress, byte dataVal) {
  #ifdef AT24C256
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write(dataAddress >> 8);
  Wire.write(dataAddress & 0xff);
  Wire.write(dataVal);
  Wire.endTransmission();
  while(isWireBusy(EEPROM_I2C_ADDRESS)){   
    delayMicroseconds(100);
    watchdog_update();
  }
  delay(5);
  #else
  EEPROM.write(dataAddress, dataVal);
  EEPROM.commit();
  #endif
}

// Function to read from EEPROM
byte readAT24(unsigned int dataAddress) {
  byte readData = 0;
  #ifdef AT24C256
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write(dataAddress >> 8);
  Wire.write(dataAddress & 0xff);
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDRESS, 1);
  if (Wire.available()) {
    readData = Wire.read();
  }
  #else
  readData = EEPROM.read(dataAddress);
  #endif
  return readData;
}

void writeAT24Int(unsigned int dataAddress, int dataVal) {
  writeAT24(dataAddress + 0, (dataVal >> 24) & 0xFF);
  writeAT24(dataAddress + 1, (dataVal >> 16) & 0xFF);
  writeAT24(dataAddress + 2, (dataVal >> 8) & 0xFF);
  writeAT24(dataAddress + 3, (dataVal & 0xFF));
}

int readAT24Int(unsigned int dataAddress) {
  return  (readAT24(dataAddress + 0) << 24) |
          (readAT24(dataAddress + 1) << 16) |
          (readAT24(dataAddress + 2) << 8) |
          (readAT24(dataAddress + 3));
}

float rroundf(float val) {
  return (int(val * 10) / 10.0);
}

bool isValidString(const char *s, int maxBufSize) {
  if (*s == '\0') {
      return false; 
  }

  for (int a = 0; a < maxBufSize; a++) {
      if (s[a] == '\0') {
          return true; 
      }
      
      bool p = (
          isdigit(s[a]) || 
          isalpha(s[a]) || 
          isspace(s[a]) || 
          isgraph(s[a]) );
      if (!p) {
          return false; 
      }
  }
  return true; 
}

unsigned short rgbToRgb565(unsigned char r, unsigned char g, unsigned char b) {
    unsigned short r5 = (r >> 3) & 0x1F;
    unsigned short g6 = (g >> 2) & 0x3F;
    unsigned short b5 = (b >> 3) & 0x1F;
    
    return (r5 << 11) | (g6 << 5) | b5;
}
