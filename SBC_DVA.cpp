/*
    SBC-DVA.h - Library for SBC-DVA with the TI INA236 chip.
    https://joy-it.net/de/products/SBC-DVA
    Created by L0L0-tack, 24 October, 2023.
    Released for Joy-IT.
    Last modified, 24 October, 2023.
*/

#include <Arduino.h>
#include <SBC_DVA.h>

// Constructor //

SBCDVA::SBCDVA(uint8_t address) {
  Address = address;
  Current_lsb = 0;
  Lsb = _LSB[0];
  Address = _ADDRESS_A[0];
  Mode = _MODE[7];
  Vshct = _VSHCT[4];
  Vbusct = _VBUSCT[4];
  Avg = _AVG[0];
  Adcrange = _ADCRANGE[0];
  byte temp[2];
}

// Private //

// Write the required 16-bit value into the register
void SBCDVA::_write_register(byte reg, int val) {
  Wire.beginTransmission(Address);
  Wire.write(reg);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  Wire.endTransmission();
}

// Read the 16-bit register value that will be returned
int SBCDVA::_read_register(byte reg) {
  Wire.beginTransmission(Address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(Address, 2);
  int value = (Wire.read() << 8) | Wire.read();
  return value;
}

// Public //

// Initialize the INA236 with the user specified parameters
void SBCDVA::init_ina236(byte address, int mode, int vshct, int vbusct, int avg, int adcrange) {
  Mode = _MODE[mode];
  Vshct = _VSHCT[vshct];
  Vbusct = _VBUSCT[vbusct];
  Avg = _AVG[avg];
  Adcrange = _ADCRANGE[adcrange];
  Address = _ADDRESS_A[address];
  Lsb = (Adcrange == ADCRANGE_1) ? _LSB[0] : _LSB[1];
  int config = 0x0000;
  config |= Mode;
  config |= Vshct;
  config |= Vbusct;
  config |= Avg;
  config |= Adcrange;
  _write_register(Config_Reg, config);
}

// Reset the INA236 registers
void SBCDVA::reset_ina236(byte address) {
  Address = address;
  _write_register(Config_Reg, RST);
}

// Calibrate the INA236 for the correct Shunt measurement
void SBCDVA::calibrate_ina236() {
  float current_lsb_min = 5 / pow(2, 15);
  Current_lsb = current_lsb_min + 0.0000074;
  int SHUNT_CAL = 0.00512 / (Current_lsb * 0.016);
  SHUNT_CAL = static_cast<int>(SHUNT_CAL);
  if (Adcrange == ADCRANGE_2) SHUNT_CAL /= 4;
  _write_register(Calibration_Reg, SHUNT_CAL);
}

// Read the current across the Shunt resistor
float SBCDVA::read_current() {
  int CURRENT = _read_register(Current_Reg);
  return (Current_lsb * CURRENT);
}

// Read the power across the Shunt resistor
float SBCDVA::read_power() {
  int POWER = _read_register(Power_Reg);
  return fabs(32 * Current_lsb * POWER);
}

// Read the voltage across the Shunt resistor
float SBCDVA::read_shunt_voltage() {
  int value_raw = _read_register(Shunt_Volt_Reg);
  int value_comp = ~value_raw;
  int value = value_comp + 1;
  return fabs(value * Lsb);
}

// Read the bus voltage
float SBCDVA::read_bus_voltage() {
  int value = _read_register(Bus_Volt_Reg);
  return value * _LSB[2];
}

// Set the alert register
void SBCDVA::mask_enable(byte val = 1) {
  int out = _MASK_ENABLE[val];
  _write_register(Mask_Enable_Reg, out);
}

// Write a value into the alert register as reference
void SBCDVA::write_alert_limit(int val = 0x294) {
  _write_register(Alert_Limit_Reg, val);
}

// Read the value from the alert register
void SBCDVA::read_alert_limit() {
  int raw = _read_register(Alert_Limit_Reg);
  Serial.println("Mask/Alert register value: " + String(raw));
}

// Read the Manufacturer ID
void SBCDVA::manufacturer_ID() {
  if (_read_register(Manufacturer_ID_Reg) == 21577) {
    Serial.println("Manufacturer ID: TI");
  } else {
    Serial.println("Manufacturer ID: nan");
  }
}

// Read the Device ID
void SBCDVA::device_ID() {
  Serial.print("Device ID: ");
  Serial.println(String(_read_register(Device_ID_Reg)));
}