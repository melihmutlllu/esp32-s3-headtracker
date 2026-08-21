#include <Arduino.h>
#include <Wire.h>

static constexpr uint8_t SDA_PIN = 8;
static constexpr uint8_t SCL_PIN = 9;
static constexpr uint32_t SERIAL_BAUD = 115200;

static constexpr uint8_t MPU_ADDR_0 = 0x68;
static constexpr uint8_t MPU_ADDR_1 = 0x69;

static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
static constexpr uint8_t REG_CONFIG = 0x1A;
static constexpr uint8_t REG_INT_PIN_CFG = 0x37;
static constexpr uint8_t REG_I2C_MST_CTRL = 0x24;
static constexpr uint8_t REG_I2C_SLV0_ADDR = 0x25;
static constexpr uint8_t REG_I2C_SLV0_REG = 0x26;
static constexpr uint8_t REG_I2C_SLV0_CTRL = 0x27;
static constexpr uint8_t REG_EXT_SENS_DATA_00 = 0x49;
static constexpr uint8_t REG_USER_CTRL = 0x6A;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
static constexpr uint8_t REG_WHO_AM_I = 0x75;

static constexpr uint8_t AK8963_ADDR = 0x0C;
static constexpr uint8_t AK8963_REG_WIA = 0x00;
static constexpr uint8_t AK8963_REG_CNTL1 = 0x0A;
static constexpr uint8_t AK8963_REG_CNTL2 = 0x0B;

static uint8_t mpuAddr = MPU_ADDR_0;

static bool addrAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true) == 0;
}

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

static bool readReg(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(addr, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) != 1) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }
  value = static_cast<uint8_t>(Wire.read());
  return true;
}

static void printHexByte(uint8_t value) {
  Serial.print("0x");
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

static void scanBus(const char *label) {
  Serial.print("# scan_start=");
  Serial.println(label);
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission(true);
    if (err == 0) {
      Serial.print("# i2c_device=");
      printHexByte(addr);
      Serial.println();
      ++found;
    }
  }
  Serial.print("# scan_found_count=");
  Serial.println(found);
  Serial.print("# scan_done=");
  Serial.println(label);
}

static void readWhoAmI(uint8_t addr) {
  uint8_t who = 0x00;
  Serial.print("# probe_addr=");
  printHexByte(addr);
  Serial.print(",ack=");
  Serial.println(addrAck(addr) ? "true" : "false");

  if (readReg(addr, REG_WHO_AM_I, who)) {
    Serial.print("# mpu_whoami_addr_");
    printHexByte(addr);
    Serial.print('=');
    printHexByte(who);
    Serial.println();
  } else {
    Serial.print("# mpu_whoami_addr_");
    printHexByte(addr);
    Serial.println("=read_failed");
  }
}

static bool wakeMpu() {
  bool ok = true;
  ok &= writeReg(mpuAddr, REG_PWR_MGMT_1, 0x80);
  delay(100);
  ok &= writeReg(mpuAddr, REG_PWR_MGMT_1, 0x01);
  delay(10);
  ok &= writeReg(mpuAddr, REG_PWR_MGMT_2, 0x00);
  delay(10);
  ok &= writeReg(mpuAddr, REG_CONFIG, 0x03);
  ok &= writeReg(mpuAddr, REG_SMPLRT_DIV, 0x07);
  return ok;
}

static void forceBypass() {
  writeReg(mpuAddr, REG_USER_CTRL, 0x00);
  delay(20);
  writeReg(mpuAddr, REG_INT_PIN_CFG, 0x02);
  delay(100);
}

static void directAk8963Probe() {
  Serial.println("# direct_ak8963_probe_start");

  const bool ack = addrAck(AK8963_ADDR);
  Serial.print("# ak8963_ack=");
  Serial.println(ack ? "true" : "false");

  uint8_t wia = 0x00;
  if (readReg(AK8963_ADDR, AK8963_REG_WIA, wia)) {
    Serial.print("# ak8963_wia=");
    printHexByte(wia);
    Serial.println();
  } else {
    Serial.println("# ak8963_wia=read_failed");
  }

  bool writeOk = true;
  writeOk &= writeReg(AK8963_ADDR, AK8963_REG_CNTL2, 0x01);
  delay(100);
  writeOk &= writeReg(AK8963_ADDR, AK8963_REG_CNTL1, 0x00);
  delay(10);
  writeOk &= writeReg(AK8963_ADDR, AK8963_REG_CNTL1, 0x16);
  delay(10);
  Serial.print("# ak8963_mode_write_ok=");
  Serial.println(writeOk ? "true" : "false");
  Serial.println("# direct_ak8963_probe_done");
}

static void internalMasterAk8963Probe() {
  Serial.println("# internal_master_ak8963_probe_start");

  writeReg(mpuAddr, REG_INT_PIN_CFG, 0x00);
  delay(10);
  writeReg(mpuAddr, REG_USER_CTRL, 0x20);       // I2C_MST_EN
  delay(10);
  writeReg(mpuAddr, REG_I2C_MST_CTRL, 0x0D);    // 400 kHz-ish master clock
  delay(10);
  writeReg(mpuAddr, REG_I2C_SLV0_ADDR, 0x80 | AK8963_ADDR);
  writeReg(mpuAddr, REG_I2C_SLV0_REG, AK8963_REG_WIA);
  writeReg(mpuAddr, REG_I2C_SLV0_CTRL, 0x81);   // enable, read 1 byte
  delay(100);

  uint8_t ext = 0x00;
  if (readReg(mpuAddr, REG_EXT_SENS_DATA_00, ext)) {
    Serial.print("# internal_master_ak8963_wia=");
    printHexByte(ext);
    Serial.println();
  } else {
    Serial.println("# internal_master_ak8963_wia=read_failed");
  }

  writeReg(mpuAddr, REG_USER_CTRL, 0x00);
  delay(10);
  writeReg(mpuAddr, REG_INT_PIN_CFG, 0x02);
  delay(10);
  Serial.println("# internal_master_ak8963_probe_done");
}

static void runProbe() {
  Serial.println();
  Serial.println("# mpu9250_deep_probe");
  Serial.print("# SDA_PIN=");
  Serial.println(SDA_PIN);
  Serial.print("# SCL_PIN=");
  Serial.println(SCL_PIN);

  Wire.setClock(100000);
  delay(50);
  scanBus("initial_100khz");
  readWhoAmI(MPU_ADDR_0);
  readWhoAmI(MPU_ADDR_1);

  if (addrAck(MPU_ADDR_0)) {
    mpuAddr = MPU_ADDR_0;
  } else if (addrAck(MPU_ADDR_1)) {
    mpuAddr = MPU_ADDR_1;
  } else {
    Serial.println("# no_mpu_at_0x68_or_0x69");
    return;
  }

  Serial.print("# selected_mpu_addr=");
  printHexByte(mpuAddr);
  Serial.println();
  Serial.print("# wake_mpu_ok=");
  Serial.println(wakeMpu() ? "true" : "false");

  forceBypass();
  scanBus("bypass_100khz");
  directAk8963Probe();
  internalMasterAk8963Probe();

  Wire.setClock(400000);
  delay(50);
  forceBypass();
  scanBus("bypass_400khz");
  directAk8963Probe();

  Serial.println("# deep_probe_done");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t startMs = millis();
  while (!Serial && millis() - startMs < 3000) {
    delay(10);
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  runProbe();
}

void loop() {
  static bool prompted = false;
  if (!prompted) {
    prompted = true;
    Serial.println("# send r to rerun deep probe");
  }

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == 'r' || c == 'R') {
      runProbe();
    }
  }
  delay(20);
}
