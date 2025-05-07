/*
 * ESP32_BQ76952.ino
 * 
 * Arduino sketch for ESP32 to communicate with BQ76952 Battery Management IC
 * This code is adapted from the STM32 implementation
 * 
 * Connection description: 
 * - ESP32 GPIO21 (SDA) to BQ76952EVM SDA
 * - ESP32 GPIO22 (SCL) to BQ76952EVM SCL
 * - ESP32 GND to BQ76952EVM GND
 * - Optional: ESP32 GPIO23 to BQ76952EVM ALERT pin
 * - Optional: ESP32 GPIO19 to BQ76952EVM DFETOFF pin 
 * - Optional: ESP32 GPIO18 to BQ76952EVM RST_SHUT pin
 */

#include <Wire.h>

// BQ76952 definitions
#define DEV_ADDR 0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode 0     // 0 for disabled, 1 for enabled
#define MAX_BUFFER_SIZE 32

// Pin Definitions
#define DFETOFF_PIN 19  // DFETOFF pin (BOTHOFF) 
#define RST_SHUT_PIN 18 // RST_SHUT pin
#define ALERT_PIN 23    // ALERT pin
#define LED_PIN 2       // Built-in LED on most ESP32 dev boards

// Direct Command Registers
#define ControlStatus 0x00
#define SafetyStatusA 0x03
#define SafetyStatusB 0x05
#define SafetyStatusC 0x07
#define PFStatusA 0x0B
#define PFStatusB 0x0D
#define PFStatusC 0x0F
#define PFStatusD 0x11
#define Cell1Voltage 0x14
#define Cell2Voltage 0x16
#define Cell3Voltage 0x18
#define Cell4Voltage 0x1A
#define Cell5Voltage 0x1C
#define Cell6Voltage 0x1E
#define Cell7Voltage 0x20
#define Cell8Voltage 0x22
#define Cell9Voltage 0x24
#define Cell10Voltage 0x26
#define Cell11Voltage 0x28
#define Cell12Voltage 0x2A
#define Cell13Voltage 0x2C
#define Cell14Voltage 0x2E
#define Cell15Voltage 0x30
#define Cell16Voltage 0x32
#define StackVoltage 0x34
#define PACKPinVoltage 0x36
#define LDPinVoltage 0x38
#define CC2Current 0x3A
#define AlarmStatus 0x62
#define AlarmRawStatus 0x64
#define AlarmEnable 0x66
#define TS1Temperature 0x70
#define TS3Temperature 0x74
#define FETStatus 0x7F

// Subcommands
#define FW_VERSION 0x0002
#define SET_CFGUPDATE 0x0090
#define EXIT_CFGUPDATE 0x0092
#define FET_ENABLE 0x0022
#define SLEEP_DISABLE 0x009A
#define BQ769x2_RESET 0x0012

// RAM Registers
#define PowerConfig 0x9234
#define REG0Config 0x9237
#define REG12Config 0x9236
#define DFETOFFPinConfig 0x92FB
#define ALERTPinConfig 0x92FC
#define TS1Config 0x92FD
#define TS3Config 0x92FF
#define HDQPinConfig 0x9300
#define VCellMode 0x9304
#define EnabledProtectionsA 0x9261
#define EnabledProtectionsB 0x9262
#define DefaultAlarmMask 0x926D
#define BalancingConfiguration 0x9335
#define CUVThreshold 0x9275
#define COVThreshold 0x9278
#define OCCThreshold 0x9280
#define OCD1Threshold 0x9282
#define SCDThreshold 0x9286
#define SCDDelay 0x9287
#define SCDLLatchLimit 0x9295

#define BATTERY_STATUS 0x12

// Global Variables
uint8_t RX_data[2] = {0x00, 0x00}; // Used for receiving data from BQ769x2
uint8_t RX_32Byte[32] = {0}; // For larger responses

uint16_t CellVoltage[16] = {0};
float Temperature[3] = {0, 0, 0};
uint16_t Stack_Voltage = 0;
uint16_t Pack_Voltage = 0;
uint16_t LD_Voltage = 0;
uint16_t Pack_Current = 0;

uint16_t AlarmBits = 0x00;
uint8_t value_SafetyStatusA = 0;
uint8_t value_SafetyStatusB = 0;
uint8_t value_SafetyStatusC = 0;
uint8_t value_PFStatusA = 0;
uint8_t value_PFStatusB = 0;
uint8_t value_PFStatusC = 0;
uint8_t FET_Status = 0;

uint8_t UV_Fault = 0;
uint8_t OV_Fault = 0;
uint8_t SCD_Fault = 0;
uint8_t OCD_Fault = 0;
uint8_t ProtectionsTriggered = 0;

uint8_t DSG = 0;
uint8_t CHG = 0;
uint8_t PCHG = 0;
uint8_t PDSG = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 BQ76952 Battery Management System");
  
  // Initialize GPIO pins
  pinMode(DFETOFF_PIN, OUTPUT);
  pinMode(RST_SHUT_PIN, OUTPUT);
  pinMode(ALERT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(DFETOFF_PIN, LOW);  // DFETOFF pin (BOTHOFF) set low
  digitalWrite(RST_SHUT_PIN, LOW);  // RST_SHUT pin set low
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C at 400kHz
  Wire.begin();
  Wire.setClock(400000);
  
  delay(1000);
  
  // Reset the BQ769x2 registers
  CommandSubcommands(BQ769x2_RESET);
  delay(60);
  
  // Configure the BQ769x2 register settings
  BQ769x2_Init();
  delay(10);
  
  // Enable the CHG and DSG FETs
  CommandSubcommands(FET_ENABLE);
  delay(10);
  
  // Disable Sleep mode (enabled by default)
  CommandSubcommands(SLEEP_DISABLE);
  
  delay(250);
  
  Serial.println("Initialization complete");
}

void loop() {
  // Read Alarm Status to check if protections triggered or new measurements available
  Serial.println("\n==================== Loop Start ====================");
  Serial.println("Battery Status at loop start:");
  BQ769x2_ReadBatteryStatus();

  BQ769x2_ReadVCellMode(); // Read VCellMode register to check if in VCell mode
  
  AlarmBits = BQ769x2_ReadAlarmStatus();
  Serial.print("AlarmBits : ");
  Serial.println(AlarmBits, BIN);
  
  if (AlarmBits & 0x80 || true) {  // Check if FULLSCAN is complete (new measurements available)
    BQ769x2_ReadAllVoltages();
    Pack_Current = BQ769x2_ReadCurrent();
    Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
    Temperature[1] = BQ769x2_ReadTemperature(TS3Temperature);
    
    // Print some cell voltages to Serial monitor
    Serial.println("-------------Battery Data-------------");
    Serial.print("Cell 1 Voltage: "); Serial.print(CellVoltage[8]); Serial.println(" mV");
    Serial.print("Cell 2 Voltage: "); Serial.print(CellVoltage[9]); Serial.println(" mV");
    Serial.print("Cell 3 Voltage: "); Serial.print(CellVoltage[13]); Serial.println(" mV");
    Serial.print("Cell 4 Voltage: "); Serial.print(CellVoltage[15]); Serial.println(" mV");
    Serial.print("Stack Voltage: "); Serial.print(Stack_Voltage); Serial.println(" mV");
    Serial.print("Pack Current: "); Serial.print(Pack_Current); Serial.println(" mA");
    Serial.print("Temperature 1: "); Serial.print(Temperature[0]); Serial.println(" °C");
    Serial.print("Temperature 2: "); Serial.print(Temperature[1]); Serial.println(" °C");
    
    // Check for voltage issues (over/under voltage)
    BQ769x2_ReadVoltageStatus();
    
    // Clear the FULLSCAN bit
    DirectCommands(AlarmStatus, 0x0080, 1);
  }
  
  if (AlarmBits & 0xC000 || true) {  // Safety Status bits are showing in AlarmStatus register
    BQ769x2_ReadSafetyStatus(); // Read Safety Status registers
    
    if (ProtectionsTriggered) {
      digitalWrite(LED_PIN, HIGH); // Turn on LED to indicate Protection triggered
      Serial.println("PROTECTION TRIGGERED!");
      
      // Print which protections triggered
      if (UV_Fault) Serial.println("Under-voltage fault");
      if (OV_Fault) Serial.println("Over-voltage fault");
      if (SCD_Fault) Serial.println("Short-circuit fault");
      if (OCD_Fault) Serial.println("Over-current fault");
      
      // Clear the Safety Status Alarm bits
      DirectCommands(AlarmStatus, 0xF800, 1);
    }
  } else {
    if (ProtectionsTriggered) {
      BQ769x2_ReadSafetyStatus();
      if (!ProtectionsTriggered) {
        digitalWrite(LED_PIN, LOW); // Turn off LED if Safety Status has cleared
        Serial.println("Protection condition cleared");
      }
    }
  }
  
  // Optional: Read FET status
  BQ769x2_ReadFETStatus();
  if (DSG) Serial.println("Discharge FET: ON");
  else Serial.println("Discharge FET: OFF");
  if (CHG) Serial.println("Charge FET: ON");
  else Serial.println("Charge FET: OFF");
  
  // Read battery status register
  BQ769x2_ReadBatteryStatus();
  
  delay(1000); // Repeat loop every 1 second
}

// CRC8 calculation function
unsigned char CRC8(unsigned char *ptr, unsigned char len) {
  unsigned char i;
  unsigned char crc = 0;
  while (len-- != 0) {
    for (i = 0x80; i != 0; i /= 2) {
      if ((crc & 0x80) != 0) {
        crc *= 2;
        crc ^= 0x107;
      } else {
        crc *= 2;
      }
      
      if ((*ptr & i) != 0)
        crc ^= 0x107;
    }
    ptr++;
  }
  return(crc);
}

// Checksum calculation function
unsigned char Checksum(unsigned char *ptr, unsigned char len) {
  unsigned char i;
  unsigned char checksum = 0;
  
  for (i = 0; i < len; i++)
    checksum += ptr[i];
  
  checksum = 0xff & ~checksum;
  
  return(checksum);
}

// Copy array function
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count) {
  uint8_t copyIndex = 0;
  for (copyIndex = 0; copyIndex < count; copyIndex++) {
    dest[copyIndex] = source[copyIndex];
  }
}

// I2C Write Register function
void I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
  uint8_t TX_Buffer[MAX_BUFFER_SIZE] = {0};
  
#if CRC_Mode
  uint8_t crc_count = 0;
  crc_count = count * 2;
  uint8_t crc1stByteBuffer[3] = {0x10, reg_addr, reg_data[0]};
  unsigned int j;
  unsigned int i;
  uint8_t temp_crc_buffer[3];
  
  TX_Buffer[0] = reg_data[0];
  TX_Buffer[1] = CRC8(crc1stByteBuffer, 3);
  
  j = 2;
  for (i = 1; i < count; i++) {
    TX_Buffer[j] = reg_data[i];
    j = j + 1;
    temp_crc_buffer[0] = reg_data[i];
    TX_Buffer[j] = CRC8(temp_crc_buffer, 1);
    j = j + 1;
  }
  
  Wire.beginTransmission(DEV_ADDR >> 1); // Convert 8-bit address to 7-bit
  Wire.write(reg_addr);
  Wire.write(TX_Buffer, crc_count);
  Wire.endTransmission();
#else
  Wire.beginTransmission(DEV_ADDR >> 1);
  Wire.write(reg_addr);
  Wire.write(reg_data, count);
  Wire.endTransmission();
#endif
}

// I2C Read Register function
int I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
  unsigned int RX_CRC_Fail = 0;
  uint8_t RX_Buffer[MAX_BUFFER_SIZE] = {0};
  
#if CRC_Mode
  uint8_t crc_count = 0;
  uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
  crc_count = count * 2;
  unsigned int j;
  unsigned int i;
  unsigned char CRCc = 0;
  uint8_t temp_crc_buffer[3];
  
  Wire.beginTransmission(DEV_ADDR >> 1);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(DEV_ADDR >> 1, crc_count);
  
  for (i = 0; i < crc_count; i++) {
    if (Wire.available()) {
      ReceiveBuffer[i] = Wire.read();
    }
  }
  
  uint8_t crc1stByteBuffer[4] = {0x10, reg_addr, 0x11, ReceiveBuffer[0]};
  CRCc = CRC8(crc1stByteBuffer, 4);
  if (CRCc != ReceiveBuffer[1]) {
    RX_CRC_Fail += 1;
  }
  RX_Buffer[0] = ReceiveBuffer[0];
  
  j = 2;
  for (i = 1; i < count; i++) {
    RX_Buffer[i] = ReceiveBuffer[j];
    temp_crc_buffer[0] = ReceiveBuffer[j];
    j = j + 1;
    CRCc = CRC8(temp_crc_buffer, 1);
    if (CRCc != ReceiveBuffer[j])
      RX_CRC_Fail += 1;
    j = j + 1;
  }
  CopyArray(RX_Buffer, reg_data, count);
#else
  Wire.beginTransmission(DEV_ADDR >> 1);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(DEV_ADDR >> 1, count);
  
  for (int i = 0; i < count; i++) {
    if (Wire.available()) {
      reg_data[i] = Wire.read();
    }
  }
#endif
  return RX_CRC_Fail;
}

// BQ769x2 Set Register function
void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen) {
  uint8_t TX_Buffer[2] = {0x00, 0x00};
  uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  // TX_RegData in little endian format
  TX_RegData[0] = reg_addr & 0xff; 
  TX_RegData[1] = (reg_addr >> 8) & 0xff;
  TX_RegData[2] = reg_data & 0xff; // 1st byte of data
  
  switch (datalen) {
    case 1: // 1 byte datalength
      I2C_WriteReg(0x3E, TX_RegData, 3);
      delay(2);
      TX_Buffer[0] = Checksum(TX_RegData, 3); 
      TX_Buffer[1] = 0x05; // combined length of register address and data
      I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      delay(2);
      break;
    case 2: // 2 byte datalength
      TX_RegData[3] = (reg_data >> 8) & 0xff;
      I2C_WriteReg(0x3E, TX_RegData, 4);
      delay(2);
      TX_Buffer[0] = Checksum(TX_RegData, 4); 
      TX_Buffer[1] = 0x06; // combined length of register address and data
      I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      delay(2);
      break;
    case 4: // 4 byte datalength, Only used for CCGain and Capacity Gain
      TX_RegData[3] = (reg_data >> 8) & 0xff;
      TX_RegData[4] = (reg_data >> 16) & 0xff;
      TX_RegData[5] = (reg_data >> 24) & 0xff;
      I2C_WriteReg(0x3E, TX_RegData, 6);
      delay(2);
      TX_Buffer[0] = Checksum(TX_RegData, 6); 
      TX_Buffer[1] = 0x08; // combined length of register address and data
      I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      delay(2);
      break;
  }
}

// Command-only Subcommands
void CommandSubcommands(uint16_t command) {
  uint8_t TX_Reg[2] = {0x00, 0x00};
  
  // TX_Reg in little endian format
  TX_Reg[0] = command & 0xff;
  TX_Reg[1] = (command >> 8) & 0xff;
  
  I2C_WriteReg(0x3E, TX_Reg, 2);
  delay(2);
}

// Subcommands with data
void Subcommands(uint16_t command, uint16_t data, uint8_t type) {
  uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t TX_Buffer[2] = {0x00, 0x00};
  
  // TX_Reg in little endian format
  TX_Reg[0] = command & 0xff;
  TX_Reg[1] = (command >> 8) & 0xff;
  
  if (type == 0) { // Read
    I2C_WriteReg(0x3E, TX_Reg, 2);
    delay(2);
    I2C_ReadReg(0x40, RX_32Byte, 32);
  } else if (type == 1) { // Write
    TX_Reg[2] = data & 0xff;
    I2C_WriteReg(0x3E, TX_Reg, 3);
    delay(1);
    TX_Buffer[0] = Checksum(TX_Reg, 3);
    TX_Buffer[1] = 0x05; // combined length of registers address and data
    I2C_WriteReg(0x60, TX_Buffer, 2);
    delay(1);
  } else if (type == 2) { // Write data with 2 bytes
    TX_Reg[2] = data & 0xff;
    TX_Reg[3] = (data >> 8) & 0xff;
    I2C_WriteReg(0x3E, TX_Reg, 4);
    delay(1);
    TX_Buffer[0] = Checksum(TX_Reg, 4);
    TX_Buffer[1] = 0x06; // combined length of registers address and data
    I2C_WriteReg(0x60, TX_Buffer, 2);
    delay(1);
  }
}

// Direct Commands
void DirectCommands(uint8_t command, uint16_t data, uint8_t type) {
  uint8_t TX_data[2] = {0x00, 0x00};
  
  // Little endian format
  TX_data[0] = data & 0xff;
  TX_data[1] = (data >> 8) & 0xff;
  
  if (type == 0) { // Read
    I2C_ReadReg(command, RX_data, 2);
    delay(2);
  } else if (type == 1) { // Write
    I2C_WriteReg(command, TX_data, 2);
    delay(2);
  }
}

// BQ769x2 Initialization function
void BQ769x2_Init() {
  // Enter CONFIGUPDATE mode
  BQ769x2_ReadBatteryStatus();
  CommandSubcommands(SET_CFGUPDATE);
  delay(20);
  
  // Print battery status after entering config mode
  Serial.println("Battery Status after entering CONFIG mode:");
  BQ769x2_ReadBatteryStatus();
  
  // Set Power Config - Enable DSL_PDO and set wake speed bits
  BQ769x2_SetRegister(PowerConfig, 0x2D80, 2);
  
  // Enable REG0 pre-regulator
  BQ769x2_SetRegister(REG0Config, 0x01, 1);
  
  // Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
  BQ769x2_SetRegister(REG12Config, 0x0D, 1);
  
  // Set DFETOFF pin to control BOTH CHG and DSG FET
  BQ769x2_SetRegister(DFETOFFPinConfig, 0x42, 1);
  
  // Set up ALERT Pin
  BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);
  
  // Set TS1 to measure Cell Temperature
  // BQ769x2_SetRegister(TS1Config, 0x07, 1);
  
  // Set TS3 to measure FET Temperature
  // BQ769x2_SetRegister(TS3Config, 0x0F, 1);
  
  // Disable HDQ thermistor (no thermistor on EVM HDQ pin)
  BQ769x2_SetRegister(HDQPinConfig, 0x00, 1);
  
  // Enable 16 cells (default setting)
  BQ769x2_SetRegister(VCellMode, 0x0000, 2);
  
  // Enable protections in 'Enabled Protections A'
  // Enables SCD, OCD1, OCC, COV, CUV
  BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1);
  
  // Enable all protections in 'Enabled Protections B'
  // Enables OTF, OTINT, OTD, OTC, UTINT, UTD, UTC
  BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);
  
  // Set default alarm mask
  BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);
  
  // Set up Cell Balancing Configuration - Automated balancing in Relax or Charge modes
  BQ769x2_SetRegister(BalancingConfiguration, 0x03, 1);
  
  // Set up CUV (under-voltage) Threshold - 0x31 (2479 mV)
  BQ769x2_SetRegister(CUVThreshold, 0x31, 1);
  
  // Set up COV (over-voltage) Threshold - 0x55 (4301 mV)
  BQ769x2_SetRegister(COVThreshold, 0x55, 1);
  
  // Set up OCC (over-current in charge) Threshold - 0x05 (10 mV = 10A with 1mOhm)
  BQ769x2_SetRegister(OCCThreshold, 0x05, 1);
  
  // Set up OCD1 Threshold - 0x0A (20 mV = 20A with 1mOhm)
  BQ769x2_SetRegister(OCD1Threshold, 0x0A, 1);
  
  // Set up SCD Threshold - 0x05 (100 mV = 100A with 1mOhm)
  BQ769x2_SetRegister(SCDThreshold, 0x05, 1);
  
  // Set up SCD Delay - 0x03 (30 us)
  BQ769x2_SetRegister(SCDDelay, 0x03, 1);
  
  // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal
  BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1);
  
  // Exit CONFIGUPDATE mode
  CommandSubcommands(EXIT_CFGUPDATE);
  delay(2);
}

// BQ769x2 read voltage function
uint16_t BQ769x2_ReadVoltage(uint8_t command) {
  DirectCommands(command, 0x00, 0);
  if (command >= Cell1Voltage && command <= Cell16Voltage) {
    return (RX_data[1] * 256 + RX_data[0]); // Voltage is reported in mV
  } else {
    return 10 * (RX_data[1] * 256 + RX_data[0]); // Voltage is reported in 0.01V units
  }
}

// Read all voltages
void BQ769x2_ReadAllVoltages() {
  int cellvoltageholder = Cell1Voltage;
  for (int x = 0; x < 16; x++) {
    CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
    cellvoltageholder = cellvoltageholder + 2;
  }
  Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
  Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
  LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}

// Read current
uint16_t BQ769x2_ReadCurrent() {
  DirectCommands(CC2Current, 0x00, 0);
  return (RX_data[1] * 256 + RX_data[0]); // Current is reported in mA
}

// Read temperature
float BQ769x2_ReadTemperature(uint8_t command) {
  DirectCommands(command, 0x00, 0);
  return (0.1 * (float)(RX_data[1] * 256 + RX_data[0])) - 273.15; // Converts from 0.1K to Celsius
}

// Read alarm status
uint16_t BQ769x2_ReadAlarmStatus() {
  DirectCommands(AlarmStatus, 0x00, 0);
  return (RX_data[1] * 256 + RX_data[0]);
}

// Read safety status
void BQ769x2_ReadSafetyStatus() {
  DirectCommands(SafetyStatusA, 0x00, 0);
  value_SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);
  UV_Fault = ((0x4 & RX_data[0]) >> 2);
  OV_Fault = ((0x8 & RX_data[0]) >> 3);
  SCD_Fault = ((0x8 & RX_data[1]) >> 3);
  OCD_Fault = ((0x2 & RX_data[1]) >> 1);
  
  DirectCommands(SafetyStatusB, 0x00, 0);
  value_SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);
  
  DirectCommands(SafetyStatusC, 0x00, 0);
  value_SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);
  
  Serial.print("value_SafetyStatusA: ");
  Serial.println(value_SafetyStatusA, BIN);
  Serial.print("value_SafetyStatusB: ");
  Serial.println(value_SafetyStatusB, BIN);
  Serial.print("value_SafetyStatusC: ");
  Serial.println(value_SafetyStatusC, BIN);
  
  if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 1) {
    ProtectionsTriggered = 1;
  } else {
    ProtectionsTriggered = 0;
  }
}

// Read FET status
void BQ769x2_ReadFETStatus() {
  DirectCommands(FETStatus, 0x00, 0);
  FET_Status = (RX_data[1] * 256 + RX_data[0]);
  DSG = ((0x4 & RX_data[0]) >> 2); // Discharge FET state
  CHG = (0x1 & RX_data[0]); // Charge FET state
  PCHG = ((0x2 & RX_data[0]) >> 1); // Pre-charge FET state
  PDSG = ((0x8 & RX_data[0]) >> 3); // Pre-discharge FET state
}

// Read battery status register
void BQ769x2_ReadBatteryStatus() {
  DirectCommands(BATTERY_STATUS, 0x00, 0);
  uint16_t batteryStatus = (RX_data[1] * 256 + RX_data[0]);
  Serial.print("Battery Status Register: 0x");
  Serial.println(batteryStatus, BIN);
  
  // Decode important bits
  // Serial.print("DSG allowed: ");
  // Serial.println((batteryStatus & 0x01) ? "Yes" : "No");
  // Serial.print("CHG allowed: ");
  // Serial.println((batteryStatus & 0x02) ? "Yes" : "No");
  // Serial.print("PCHG allowed: ");
  // Serial.println((batteryStatus & 0x04) ? "Yes" : "No");
  // Serial.print("DCHG mode: ");
  // Serial.println((batteryStatus & 0x08) ? "Yes" : "No");
  // Serial.print("SLEEP mode: ");
  // Serial.println((batteryStatus & 0x10) ? "Yes" : "No");
  // Serial.print("SHUTDOWN mode: ");
  // Serial.println((batteryStatus & 0x20) ? "Yes" : "No");
  // Serial.print("SEC0 mode: ");
  // Serial.println((batteryStatus & 0x40) ? "Yes" : "No");
  // Serial.print("SEC1 mode: ");
  // Serial.println((batteryStatus & 0x80) ? "Yes" : "No");
  // Serial.print("FUSE blown: ");
  // Serial.println((batteryStatus & 0x100) ? "Yes" : "No");
  // Serial.print("SS present: ");
  // Serial.println((batteryStatus & 0x200) ? "Yes" : "No");
  // Serial.print("Chemical fuse blown: ");
  // Serial.println((batteryStatus & 0x400) ? "Yes" : "No");
  // Serial.print("Full access sealed: ");
  // Serial.println((batteryStatus & 0x800) ? "Yes" : "No");
  // Serial.println("----------------------------------");
}

// DFETOFF (BOTHOFF) control functions
void BQ769x2_BOTHOFF() {
  digitalWrite(DFETOFF_PIN, HIGH);
}

void BQ769x2_RESET_BOTHOFF() {
  digitalWrite(DFETOFF_PIN, LOW);
}

// Shutdown control functions
void BQ769x2_ShutdownPin() {
  digitalWrite(RST_SHUT_PIN, HIGH);
}

void BQ769x2_ReleaseShutdownPin() {
  digitalWrite(RST_SHUT_PIN, LOW);
}

// Read VCellMode register to determine active cells
void BQ769x2_ReadVCellMode() {
  uint8_t RX_Buffer[4] = {0};
  uint8_t TX_RegData[2] = {0x04, 0x93}; // VCellMode register address 0x9304 in little-endian
  uint16_t vcellMode = 0;
  uint8_t numCellsEnabled = 0;
  
  // Read VCellMode register (0x9304)
  I2C_WriteReg(0x3E, TX_RegData, 2);
  delay(2);
  I2C_ReadReg(0x40, RX_Buffer, 2);
  vcellMode = (RX_Buffer[1] << 8) | RX_Buffer[0];
  
  Serial.println("\n----------VCell Configuration----------");
  Serial.print("VCellMode Register Value: ");
  Serial.println(vcellMode, BIN);

  Serial.println("--------------------------------------");
}

// Read undervoltage and overvoltage status for all cells
void BQ769x2_ReadVoltageStatus() {
  uint8_t RX_Buffer[2] = {0};
  uint16_t safetyStatusA = 0;
  
  // Read SafetyStatusA register
  DirectCommands(SafetyStatusA, 0x00, 0);
  safetyStatusA = (RX_data[1] << 8) | RX_data[0];
  
  Serial.println("\n----------Voltage Protection Status----------");
  
  // Check for cell undervoltage (CUV) - bit 2 in SafetyStatusA
  if (safetyStatusA & 0x0004) {
    Serial.println("FAULT: Cell Undervoltage Detected!");
    
    // Get snapshot of which cells have undervoltage
    uint8_t TX_Reg[2] = {0x80, 0x00}; // CUV_SNAPSHOT subcommand 0x0080
    I2C_WriteReg(0x3E, TX_Reg, 2);
    delay(2);
    I2C_ReadReg(0x40, RX_Buffer, 2);
    uint16_t cuvSnapshot = (RX_Buffer[1] << 8) | RX_Buffer[0];
    
    Serial.println("Cells with undervoltage:");
    for (int i = 0; i < 16; i++) {
      if (cuvSnapshot & (1 << i)) {
        Serial.print("Cell ");
        Serial.print(i + 1);
        Serial.print(": UNDERVOLTAGE (");
        Serial.print(CellVoltage[i]);
        Serial.println(" mV)");
      }
    }
  } else {
    Serial.println("All cells above minimum voltage threshold");
  }
  
  // Check for cell overvoltage (COV) - bit 3 in SafetyStatusA
  if (safetyStatusA & 0x0008) {
    Serial.println("FAULT: Cell Overvoltage Detected!");
    
    // Get snapshot of which cells have overvoltage
    uint8_t TX_Reg[2] = {0x81, 0x00}; // COV_SNAPSHOT subcommand 0x0081
    I2C_WriteReg(0x3E, TX_Reg, 2);
    delay(2);
    I2C_ReadReg(0x40, RX_Buffer, 2);
    uint16_t covSnapshot = (RX_Buffer[1] << 8) | RX_Buffer[0];
    
    Serial.println("Cells with overvoltage:");
    for (int i = 0; i < 16; i++) {
      if (covSnapshot & (1 << i)) {
        Serial.print("Cell ");
        Serial.print(i + 1);
        Serial.print(": OVERVOLTAGE (");
        Serial.print(CellVoltage[i]);
        Serial.println(" mV)");
      }
    }
  } else {
    Serial.println("All cells below maximum voltage threshold");
  }
  
  // Check for overall voltage status
  Serial.println("\nOverall Pack Voltage Status:");
  
  // Read current thresholds
  uint8_t RX_ThresholdData[4] = {0};
  uint8_t TX_RegAddr[2] = {0x75, 0x92}; // CUVThreshold address 0x9275
  
  // Enter CFGUPDATE mode to read the register
  CommandSubcommands(SET_CFGUPDATE);
  delay(10);
  
  // Read CUV Threshold
  I2C_WriteReg(0x3E, TX_RegAddr, 2);
  delay(2);
  I2C_ReadReg(0x40, RX_ThresholdData, 1);
  uint16_t cuvThreshold = RX_ThresholdData[0] * 50.6; // Convert to mV
  
  // Read COV Threshold
  TX_RegAddr[0] = 0x78;
  TX_RegAddr[1] = 0x92; // COVThreshold address 0x9278
  I2C_WriteReg(0x3E, TX_RegAddr, 2);
  delay(2);
  I2C_ReadReg(0x40, RX_ThresholdData, 1);
  uint16_t covThreshold = RX_ThresholdData[0] * 50.6; // Convert to mV
  
  // Exit CFGUPDATE mode
  CommandSubcommands(EXIT_CFGUPDATE);
  delay(2);
  
  // Print voltage thresholds
  Serial.print("Cell Undervoltage Threshold: ");
  Serial.print(cuvThreshold);
  Serial.println(" mV");
  
  Serial.print("Cell Overvoltage Threshold: ");
  Serial.print(covThreshold);
  Serial.println(" mV");
  
  // Find min and max cell voltage
  uint16_t minCellVoltage = 65535;
  uint16_t maxCellVoltage = 0;
  uint8_t minCellIndex = 0;
  uint8_t maxCellIndex = 0;
  
  for (int i = 0; i < 16; i++) {
    if (CellVoltage[i] > 0) { // Only check active cells
      if (CellVoltage[i] < minCellVoltage) {
        minCellVoltage = CellVoltage[i];
        minCellIndex = i;
      }
      if (CellVoltage[i] > maxCellVoltage) {
        maxCellVoltage = CellVoltage[i];
        maxCellIndex = i;
      }
    }
  }
  
  Serial.print("Minimum Cell Voltage: Cell ");
  Serial.print(minCellIndex + 1);
  Serial.print(" (");
  Serial.print(minCellVoltage);
  Serial.println(" mV)");
  
  Serial.print("Maximum Cell Voltage: Cell ");
  Serial.print(maxCellIndex + 1);
  Serial.print(" (");
  Serial.print(maxCellVoltage);
  Serial.println(" mV)");
  
  Serial.print("Cell Imbalance: ");
  Serial.print(maxCellVoltage - minCellVoltage);
  Serial.println(" mV");
  
  Serial.print("Pack Voltage: ");
  Serial.print(Stack_Voltage);
  Serial.println(" mV");
  Serial.println("------------------------------------------");
}