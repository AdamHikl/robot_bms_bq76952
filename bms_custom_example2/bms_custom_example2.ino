#include <Wire.h>

// BQ76952 I2C address (7-bit)
#define BQ76952_I2C_ADDR 0x08

// Command addresses
#define BATTERY_STATUS 0x12
#define CELL1_VOLTAGE_L 0x14
#define CELL1_VOLTAGE_H 0x15
#define FET_STATUS 0x7F
#define COMMAND_REG 0x3E          // Direct commands register
#define SUBCMD_FET_ENABLE 0x0022  // FET_ENABLE subcommand

// Variables to track communication status
bool commEstablished = false;
int retryCount = 0;
const int maxRetries = 5;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 400000);  // Initialize I2C with default pins (SDA=21, SCL=22)

  // Try to establish communication
  while (!commEstablished && retryCount < maxRetries) {
    if (checkCommunication()) {
      commEstablished = true;
      Serial.println("Communication with BQ76952 established successfully!");
    } else {
      retryCount++;
      Serial.print("Communication failed, retry ");
      Serial.print(retryCount);
      Serial.println("/5");
      delay(1000);
    }
  }

  if (!commEstablished) {
    Serial.println("Failed to establish communication with BQ76952");
    while (1)
      ;  // Halt if communication fails
  }
  delay(2000);
  // softReset();
  // delay(2000);

  // for (int i = 0; i < 500; i++) {
  //   uint16_t batteryStatusa = readWord(0x12);
  //   Serial.print("batteryStatusa (0x12): ");
  //   Serial.println(batteryStatusa, BIN);

  //   // Extract SEC1:0 = bits 9 and 8
  //   uint8_t secState = (batteryStatusa >> 8) & 0b11;

  //   if (secState != 0) {
  //     Serial.print("Initialized (SEC1:0 = ");
  //     Serial.print(secState);
  //     Serial.println(")");
  //     break;
  //   }

  //   delay(100);
  // }

  // uint16_t deviceType = readWord(0x01);
  // Serial.print("Device Type (0x01): 0x");
  // Serial.println(deviceType, HEX);

  // uint16_t firmwareVersion = readWord(0x02);
  // Serial.print("Firmware Version (0x02): 0x");
  // Serial.println(firmwareVersion, HEX);

  // uint16_t hardwareVersion = readWord(0x03);
  // Serial.print("Hardware Version (0x03): 0x");
  // Serial.println(hardwareVersion, HEX);

  // Read register 0x00 (subcommand register) to see if the command was processed
  writeSubcommand(0x0011);     // RESET subcommand to apply

  uint16_t subcmdStatus = readWord(0x00);  // Read the value of the subcommand register
  Serial.print("Subcommand Status (0x00): ");
  Serial.println(subcmdStatus, BIN);  // Should show the subcommand that was sent, e.g., 0x0011

  uint16_t batteryStatus = readWord(0x12);  // 1111110010001100
  Serial.print("batteryStatus (0x12): ");
  Serial.println(batteryStatus, BIN);

  uint16_t manufacturingStatus = readWord(0x9343);  // 1111110010001100
  Serial.print("manufacturingStatus (0x934): ");
  Serial.println(manufacturingStatus, BIN);

  uint16_t opStatus = readWord(0x57);
  Serial.print("OperationStatusAAA: ");
  Serial.println(opStatus, BIN);  // Now should be non-zero

  uint16_t errorStatus = readWord(0x64);  // Error flags register 1101010011101011
  Serial.print("Error Status: ");
  Serial.println(errorStatus, BIN);  // Check for error flags

  uint16_t fetStatus = readWord(0x0097);  // FET status register  // 1011011100001111   0x0098 
  Serial.print("FET Status: ");
  Serial.println(fetStatus, BIN);  // Check FET status bits

  uint16_t LDOStatus = readWord(0x0098);  // LDO status register   
  Serial.print("LDO Status: ");
  Serial.println(LDOStatus, BIN);  // Check FET status bits

  uint16_t sealedStatus = readWord(0x54);
  Serial.print("Sealed Status Register (0x54): ");
  Serial.println(sealedStatus, BIN);

  uint16_t ctrl0 = readWord(0x03);  // Read CTRL0 (Load Detect settings)
  Serial.print("CTRL0: ");
  Serial.println(ctrl0, BIN);

  uint16_t faultStatus = readWord(0x64);
  Serial.print("Before clearing, Fault Status: 0x");  // 1101010011101011
  Serial.println(faultStatus, HEX);

  clearFaults();

  // writeSubcommand(0x0011);     // RESET subcommand to apply
  // writeSubcommand(0x0020);  // SEAL subcommand
  writeSubcommand(SUBCMD_FET_ENABLE);  // SEAL subcommand

  delay(500);  // Give time for reset  0x0012 

  faultStatus = readWord(0x64);
  Serial.print("After clearing, Fault Status: 0x");  //
  Serial.println(faultStatus, HEX);

  enterConfigUpdateMode();

  setCellConfiguration();
}

void loop() {
  // Read and print cell voltages (example for cells 1-4)
  for (int cell = 1; cell <= 16; cell++) {
    float voltage = readCellVoltage(cell);
    Serial.print("Cell ");
    Serial.print(cell);
    Serial.print(" voltage: ");
    Serial.print(voltage, 3);
    Serial.println(" V");
  }

  // readCellVoltages();
  // readPackCurrent();
  writeSubcommand(0x000E);

  uint16_t batteryStatus = readWord(0x12);  // 1111110010001100
  Serial.print("batteryStatus (0x12): ");
  Serial.println(batteryStatus, BIN);

  // writeRegister(0x0096, 0x03);  // Enable both CHG and DSG
  writeSubcommand(SUBCMD_FET_ENABLE);  // SEAL subcommand
  delay(500);

  // Read and print FET status
  uint8_t fetStatus = readRegister(0x0C);
  Serial.print("FET Status: ");
  Serial.println(fetStatus, BIN);

  uint16_t protStatusA = readWord(0x92C0);
  uint16_t protStatusB = readWord(0x92C1);
  uint16_t protStatusC = readWord(0x92C2);
  uint16_t protStatusD = readWord(0x92C3);

  // writeRegister(0x001A, 0x00);  // Exit Sleep Mode

  Serial.print("Protection A: ");
  Serial.println(protStatusA, BIN);
  Serial.print("Protection B: ");
  Serial.println(protStatusB, BIN);
  Serial.print("Protection C: ");
  Serial.println(protStatusC, BIN);
  Serial.print("Protection D: ");
  Serial.println(protStatusD, BIN);

  // uint8_t protStatusCA = readRegister(0x9265);
  // uint8_t protStatusCB = readRegister(0x9266);
  // uint8_t protStatusCC = readRegister(0x9267);

  // // writeRegister(0x001A, 0x00);  // Exit Sleep Mode

  // Serial.print("Protection CA: ");
  // Serial.println(protStatusCA, BIN);
  // Serial.print("Protection CB: ");
  // Serial.println(protStatusCB, BIN);
  // Serial.print("Protection CC: ");
  // Serial.println(protStatusCC, BIN);

  uint8_t fetOptions = readExtendedData(0x9234);
  Serial.print("FET Options: ");
  Serial.println(fetOptions, BIN);

  uint16_t uv_raw = readWord(0x9275);  // Corrected register
  uint16_t ov_raw = readWord(0x9278);  // Corrected register

  if (uv_raw != 0xFFFF && ov_raw != 0xFFFF) {
    Serial.print("Undervoltage Threshold: ");
    Serial.print(uv_raw);
    Serial.println(" mV");

    Serial.print("Overvoltage Threshold: ");
    Serial.print(ov_raw);
    Serial.println(" mV");
  } else {
    Serial.println("Failed to read register values.");
  }

  Serial.print("Some random register by chatgpt: ");
  Serial.println(readRegister(0x0C), BIN);  // Should now return 0b00000011

  readCellConfig();

  delay(1000);  // Delay between readings
}

bool checkCommunication() {
  // Try to read the Battery Status register
  Wire.beginTransmission(BQ76952_I2C_ADDR);
  Wire.write(BATTERY_STATUS);
  if (Wire.endTransmission(false) != 0) {  // Send repeated start
    return false;
  }

  // Request 2 bytes (Battery Status is 16-bit)
  if (Wire.requestFrom(BQ76952_I2C_ADDR, 2) != 2) {
    return false;
  }

  // If we got data, communication is working
  uint16_t batteryStatus = Wire.read() | (Wire.read() << 8);
  return true;
}

float readCellVoltage(int cellNumber) {
  // Calculate register addresses for the cell
  uint8_t regLow = CELL1_VOLTAGE_L + (2 * (cellNumber - 1));
  uint8_t regHigh = regLow + 1;

  // Read low and high bytes
  uint8_t lowByte = readRegister(regLow);
  uint8_t highByte = readRegister(regHigh);

  // Combine bytes and convert to voltage (mV to V)
  int16_t voltage_mV = (highByte << 8) | lowByte;
  return voltage_mV / 1000.0;
}

uint16_t readWord(uint8_t reg) {
  Wire.beginTransmission(BQ76952_I2C_ADDR);  // 7-bit address of BQ76952
  Wire.write(reg);                           // Register address
  Wire.endTransmission(false);               // Send repeated start

  delayMicroseconds(100);

  Wire.requestFrom(BQ76952_I2C_ADDR, 2);  // Request 2 bytes
  if (Wire.available() == 2) {
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    return (msb << 8) | lsb;  // Correct byte order
  }
  return 0xFFFF;  // Error
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(BQ76952_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);  // Send repeated start

  delayMicroseconds(100);

  Wire.requestFrom(BQ76952_I2C_ADDR, 1);
  while (Wire.available() < 1)
    ;  // Wait for data
  return Wire.read();
}

// Function to toggle FETs using the FET_ENABLE subcommand
bool toggleFET(bool enableCharging, bool enableDischarging) {
  uint8_t fetConfig = 0;
  if (enableCharging) fetConfig |= 0x01;     // CHG FET enable bit
  if (enableDischarging) fetConfig |= 0x02;  // DSG FET enable bit

  Wire.beginTransmission(BQ76952_I2C_ADDR);
  Wire.write(COMMAND_REG);
  Wire.write(SUBCMD_FET_ENABLE & 0xFF);         // Low byte of subcommand
  Wire.write((SUBCMD_FET_ENABLE >> 8) & 0xFF);  // High byte of subcommand
  Wire.write(fetConfig);                        // Configuration data
  return (Wire.endTransmission() == 0);
}

uint8_t readExtendedData(uint16_t address) {
  // Write address to 0x3E (Data Memory Class/Offset selector)
  writeRegister(0x3E, (address >> 8) & 0xFF);  // Class ID
  writeRegister(0x3F, address & 0xFF);         // Offset

  delay(2);  // Wait for data to be loaded to 0x40

  return readRegister(0x40);  // Data appears at 0x40 after selecting class/offset
}

// void enterConfigUpdate() {
//   Wire.beginTransmission(0x08);
//   Wire.write(0x00);  // Manufacturer Access Register
//   Wire.write(0x2E);  // Subcommand to exit CONFIG_UPDATE mode
//   Wire.write(0x00);
//   Wire.endTransmission();
//   delay(100);

//   uint8_t operationStatus = readRegister(0x0057);

//   Serial.print("OperationStatus: ");
//   Serial.println(operationStatus, HEX);

//   if (operationStatus & 0b00010000) {
//     Serial.println("BQ76952 is in CONFIG_UPDATE mode.");
//   } else {
//     Serial.println("Not in CONFIG_UPDATE mode.");
//   }


//   testLaLaLa();
// }

void enterConfigUpdateMode() {
  writeSubcommand(0x0090);  // ENTER_CFG_UPDATE
  delay(1);

  // Wait for CFGUPDATE bit to be set in BatteryStatus (0x12), bit 4
  bool entered = false;
  // for (int i = 0; i < 50; i++) {
  //   uint16_t batteryStatus = readWord(0x12);
  //   Serial.print("batteryStatus (0x12): ");
  //   Serial.println(batteryStatus, BIN);
  //   if (batteryStatus & (1 << 4)) {
  //     Serial.println("Entered CONFIG_UPDATE mode.");
  //     entered = true;
  //     break;
  //   }
  //   delay(100);
  // }

  uint16_t batteryStatus = readWord(0x12);
  Serial.print("batteryStatus (0x12): ");
  Serial.println(batteryStatus, BIN);
  if (batteryStatus & (1 << 4)) {
    Serial.println("Entered CONFIG_UPDATE mode.");
    entered = true;
    // break;
  }

  writeWord(0x9304, 0x800F);

  Serial.println("Cell configuration set to 5S.");

  if (!entered) {
    Serial.println("Failed to enter CONFIG_UPDATE mode.");
  } else {
    delay(1000);
    writeSubcommand(0x0092);  // EXIT_CFG_UPDATE
    delay(100);
    Serial.println("Exited CONFIG_UPDATE mode.");
    delay(1000);
    uint16_t batteryStatus = readWord(0x12);
    Serial.print("batteryStatus (0x12): ");
    Serial.println(batteryStatus, BIN);
  }
}

// Address 0x9234 = FET Options
void writeFETOptions(uint8_t value) {
  writeExtendedData(0x9234, value);
}

void testLaLaLa() {
  writeFETOptions(0x2D);
  delay(100);
  writeSubcommand(0x0092);  // Exit CONFIG_UPDATE
  delay(100);
  writeRegister(0x0096, 0x03);  // Enable CHG + DSG
  delay(100);
}

void writeSubcommand(uint16_t command) {
  Wire.beginTransmission(0x08);               // Correct I2C address (0x08)
  Wire.write(0x3E);                           // Subcommand register (0x00)
  Wire.write((byte)command & 0x00FF);         // LSB
  Wire.write((byte)(command >> 8) & 0x00FF);  // MSB
  Wire.endTransmission();
  delay(10);  // Short delay after sending
}

void writeWord(uint8_t reg, uint16_t value) {
  // Break the 16-bit value into two 8-bit values
  uint8_t highByte = (value >> 8) & 0xFF;  // Extract the higher 8 bits
  uint8_t lowByte = value & 0xFF;          // Extract the lower 8 bits

  // Write the two bytes to the register
  writeRegister(reg, highByte);     // Write the high byte
  writeRegister(reg + 1, lowByte);  // Write the low byte
}

// Write data to a specific register
void writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(0x08);     // Correct I2C address (0x08)
  Wire.write(reg);                  // Register address
  Wire.write(value & 0xFF);         // LSB
  Wire.write((value >> 8) & 0xFF);  // MSB
  Wire.endTransmission();
  delay(10);  // Short delay after sending
}

void writeExtendedData(uint16_t address, uint8_t data) {
  writeRegister(0x3E, (address >> 8) & 0xFF);  // Class ID
  writeRegister(0x3F, address & 0xFF);         // Offset
  delay(2);
  writeRegister(0x40, data);  // Data
  delay(2);
}

void clearFaults() {
  Wire.beginTransmission(BQ76952_I2C_ADDR);
  Wire.write(0x07);  // System Status Command Register
  Wire.write(0xFF);  // Clear all faults
  Wire.endTransmission();

  Serial.println("Faults cleared.");
}

void softReset() {
  Wire.beginTransmission(BQ76952_I2C_ADDR);
  Wire.write(0x09);  // Reset command register
  Wire.write(0x00);  // Command to execute reset
  Wire.endTransmission();

  Serial.println("Soft reset sent.");
}

int16_t readPackCurrent() {
  uint16_t low = readRegister(0x3A);
  uint16_t high = readRegister(0x3B);
  int16_t current = (high << 8) | low;  // Signed value

  Serial.print("Pack Current: ");
  Serial.print(current * 0.001);  // Convert to Amps
  Serial.println("A");

  return current;
}

void readCellVoltages() {
  Serial.println("Reading cell voltages:");

  for (uint8_t i = 0; i < 16; i++) {  // Adjust for your battery size
    uint8_t reg = 0x14 + (i * 2);
    uint16_t voltage = readRegister(reg) | (readRegister(reg + 1) << 8);

    Serial.print("Cell ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(voltage * 0.001);  // Convert to volts
    Serial.println("V");
  }
}

void setCellConfiguration() {
  writeWord(0x9304, 0x800F);

  Serial.println("Cell configuration set to 5S.");
}

void readCellConfig() {
  uint16_t config = readWord(0x9304);
  Serial.print("Cell Configuration Register (0x9304): 0x");
  Serial.println(config, HEX);
  enterConfigUpdateMode();
}