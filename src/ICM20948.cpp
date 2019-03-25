#include "ICM20948.h"

//==============================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer,
//====== and temperature data
//==============================================================================

ICM20948::ICM20948(int8_t cspin /*=NOT_SPI*/) // Uses I2C communication by default
{
  // Hardware SPI communication
  // If used with DP Eng ICM-20948 breakout board
  // https://www.ebay.co.uk/itm/323724746939
  // https://www.amazon.co.uk/DP-Eng-ICM-20948-Breakout-obsolete/dp/B07PDTKK3Y
  // I2C highly recommended.
  if ((cspin > NOT_SPI) && (cspin < NUM_DIGITAL_PINS))
  {
    _csPin = cspin;
    SPI.begin();
    pinMode(_csPin, OUTPUT);
    deselect();
  }
  else
  {
    _csPin = NOT_SPI;
    Wire.begin();
  }
}

void ICM20948::getMres()
{
    mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
}

void ICM20948::getGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

void ICM20948::getAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
}


void ICM20948::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array
  readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = (int16_t)(rawData[0] << 8) | rawData[1];
  destination[1] = (int16_t)(rawData[2] << 8) | rawData[3];
  destination[2] = (int16_t)(rawData[4] << 8) | rawData[5];
}


void ICM20948::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = (int16_t)(rawData[0] << 8) | rawData[1];
  destination[1] = (int16_t)(rawData[2] << 8) | rawData[3];
  destination[2] = (int16_t)(rawData[4] << 8) | rawData[5];
}

void ICM20948::readMagData(int16_t * destination)
{
  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end
  // of data acquisition
  uint8_t rawData[8];
  // Wait for magnetometer data ready bit to be set
  if (readByte(AK09916_ADDRESS, AK09916_ST1) & 0x01)
  {
	  
    // Read the six raw data and ST2 registers sequentially into data array
    readBytes(AK09916_ADDRESS, AK09916_XOUT_L, 8, &rawData[0]);
    uint8_t c = rawData[7]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow set, if not then report data
	  // Remove once finished
	
    if (!(c & 0x08))
    {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
      // Data stored as little Endian
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }
  }
}

int16_t ICM20948::readTempData()
{
  uint8_t rawData[2]; // x/y/z gyro register data stored here
  // Read the two raw data registers sequentially into data array
  readBytes(ICM20948_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
  // Turn the MSB and LSB into a 16-bit value
  return ((int16_t)rawData[0] << 8) | rawData[1];
}

// Calculate the time the last update took for use in the quaternion filters
// TODO: This doesn't really belong in this class.
void ICM20948::updateTime()
{
  Now = micros();

  // Set integration time by time elapsed since last filter update
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
}

void ICM20948::initAK09916()
{
	
	// Write code to initialise magnetometer
	// Bypass I2C master interface and turn on magnetometer
  //writeByte(ICM20948_ADDRESS, INT_PIN_CFG, 0x02); Already set in initICM20948

  // Configure the magnetometer for continuous read and highest resolution.
  // Enable continuous mode data acquisition Mmode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates.

  // Set magnetometer data resolution and sample ODR
  writeByte(AK09916_ADDRESS, AK09916_CNTL2, 0x08);
  delay(10);
}

void ICM20948::initICM20948()
{
    // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  delay(200);
  
  
  
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
	
  // Configure Gyro and Thermometer
  // Disable FSYNC and set gyro bandwidth to 51.2 Hz,
  // respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion
  // update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the ICM20948, it is possible to get gyro sample rates of 32 kHz (!),
  // 8 kHz, or 1 kHz
  // Set gyroscope full scale range to 250 dps
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x19);
  writeByte(ICM20948_ADDRESS, TEMP_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + GYRO_SMPLRT_DIV)
  // Use a 220 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above.
  
  writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x04);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3

  // Set accelerometer full-scale range configuration
  // Get current ACCEL_CONFIG register value
  uint8_t c = readByte(ICM20948_ADDRESS, ACCEL_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x06;  // Clear AFS bits [4:3]
  c = c | Ascale << 1; // Set full scale range for the accelerometer
  c = c | 0x01; // Set enable accel DLPF for the accelerometer
  c = c | 0x18; // and set DLFPFCFG to 50.4 hz
  // Write new ACCEL_CONFIG register value
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, c);

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
  // 1.13 kHz
  writeByte(ICM20948_ADDRESS, ACCEL_SMPLRT_DIV_2, 0x04);
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the GYRO_SMPLRT_DIV setting

  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
  // until interrupt cleared, clear on read of INT_STATUS, and enable
  // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
  // controlled by the Arduino as master.
  writeByte(ICM20948_ADDRESS, INT_PIN_CFG, 0x22);
  // Enable data ready (bit 0) interrupt
  writeByte(ICM20948_ADDRESS, INT_ENABLE_1, 0x01);
}


// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void ICM20948::calibrateICM20948(float * gyroBias, float * accelBias)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	
  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAG);
  delay(200);

  // get stable time source; Auto select clock source to be PLL gyroscope
  // reference if ready else use the internal oscillator, bits 2:0 = 001
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  delay(200);

  // Configure device for bias calculation
  // Disable all interrupts
  writeByte(ICM20948_ADDRESS, INT_ENABLE, 0x00);
  // Disable FIFO
  writeByte(ICM20948_ADDRESS, FIFO_EN_1, 0x00);
  writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x00);
  // Turn on internal clock source
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x00);
  // Disable I2C master
  //writeByte(ICM20948_ADDRESS, I2C_MST_CTRL, 0x00); Already disabled
  // Disable FIFO and I2C master modes
  writeByte(ICM20948_ADDRESS, USER_CTRL, 0x00);
  // Reset FIFO and DMP
  writeByte(ICM20948_ADDRESS, USER_CTRL, 0x08);
  writeByte(ICM20948_ADDRESS, FIFO_RST, 0x1F);
  delay(10);
  writeByte(ICM20948_ADDRESS, FIFO_RST, 0x00);
  delay(15);

  // Set FIFO mode to snapshot
  writeByte(ICM20948_ADDRESS, FIFO_MODE, 0x1F);
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
  // Configure ICM20948 gyro and accelerometer for bias calculation
  // Set low-pass filter to 188 Hz
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x01);
  // Set sample rate to 1 kHz
  writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x00);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x00);
  // Set accelerometer full-scale to 2 g, maximum sensitivity
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x00);

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(ICM20948_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
  // ICM20948)
  writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x1E);
  delay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  // Disable gyro and accelerometer sensors for FIFO
  writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x00);
  // Read FIFO sample count
  readBytes(ICM20948_ADDRESS, FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // Read data for averaging
    readBytes(ICM20948_ADDRESS, FIFO_R_W, 12, &data[0]);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases.
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity;
  }
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup.
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format.
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  // Biases are additive, so change sign on calculated average gyro biases
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

  // Push gyro biases to hardware registers
  writeByte(ICM20948_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(ICM20948_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(ICM20948_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(ICM20948_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(ICM20948_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(ICM20948_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer
  // bias registers. These registers contain factory trim values which must be
  // added to the calculated accelerometer biases; on boot up these registers
  // will hold non-zero values. In addition, bit 0 of the lower byte must be
  // preserved since it is used for temperature compensation calculations.
  // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.
  
  // Switch to user bank 1
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
  // A place to hold the factory accelerometer trim biases
  int32_t accel_bias_reg[3] = {0, 0, 0};
  // Read factory accelerometer trim values
  readBytes(ICM20948_ADDRESS, XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(ICM20948_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(ICM20948_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  // Define mask for temperature compensation bit 0 of lower byte of
  // accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};

  for (ii = 0; ii < 3; ii++)
  {
    // If temperature compensation bit is set, record that fact in mask_bit
    if ((accel_bias_reg[ii] & mask))
    {
      mask_bit[ii] = 0x01;
    }
  }

  // Construct total accelerometer bias, including calculated average
  // accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
  // (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[5] = data[5] | mask_bit[2];

  // Apparently this is not working for the acceleration biases in the ICM-20948
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(ICM20948_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(ICM20948_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(ICM20948_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(ICM20948_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(ICM20948_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(ICM20948_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
void ICM20948::ICM20948SelfTest(float * destination)
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

	
  // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  delay(200);
  
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
  // Set gyro sample rate to 1 kHz
  writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x00);
  // Set gyro sample rate to 1 kHz, DLPF to 119.5 Hz and FSR to 250 dps
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x11);
  // Set accelerometer rate to 1 kHz and bandwidth to 111.4 Hz
  // Set full scale range for the accelerometer to 2 g
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x11);
  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

  // Get average current values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {
Serial.print("BHW::ii = ");
Serial.println(ii);
    // Read the six raw data registers into data array
    readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average current readings
  for (int ii =0; ii < 3; ii++)
  {
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
  
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

  // Configure the accelerometer for self-test
  // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x1C);
  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_2,  0x38);
  delay(25);  // Delay a while to let the device stabilize
  
  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

  // Get average self-test values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {
    // Read the six raw data registers into data array
    readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average self-test readings
  for (int ii =0; ii < 3; ii++)
  {
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }
  
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

  // Configure the gyro and accelerometer for normal operation
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x00);
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_2,  0x00);
  delay(25);  // Delay a while to let the device stabilize
  
  // Switch to user bank 1
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  // X-axis accel self-test results
  selfTest[0] = readByte(ICM20948_ADDRESS, SELF_TEST_X_ACCEL);
  // Y-axis accel self-test results
  selfTest[1] = readByte(ICM20948_ADDRESS, SELF_TEST_Y_ACCEL);
  // Z-axis accel self-test results
  selfTest[2] = readByte(ICM20948_ADDRESS, SELF_TEST_Z_ACCEL);
  // X-axis gyro self-test results
  selfTest[3] = readByte(ICM20948_ADDRESS, SELF_TEST_X_GYRO);
  // Y-axis gyro self-test results
  selfTest[4] = readByte(ICM20948_ADDRESS, SELF_TEST_Y_GYRO);
  // Z-axis gyro self-test results
  selfTest[5] = readByte(ICM20948_ADDRESS, SELF_TEST_Z_GYRO);
  
  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

  // Retrieve factory self-test value from self-test code reads
  // FT[Xa] factory trim calculation
  factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
  // FT[Ya] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
  // FT[Za] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
  // FT[Xg] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
  // FT[Yg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
  // FT[Zg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
  // of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++)
  {
    // Report percent differences
    destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]
      - 100.;
    // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]
      - 100.;
  }
}

// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void ICM20948::magCalICM20948(float * bias_dest, float * scale_dest)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3]  = {0, 0, 0},
          mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3]  = {0x8000, 0x8000, 0x8000},
          mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF},
          mag_temp[3] = {0, 0, 0};

  // Make sure resolution has been calculated
  getMres();

  Serial.println(F("Mag Calibration: Wave device in a figure 8 until done!"));
  Serial.println(
      F("  4 seconds to get ready followed by 15 seconds of sampling)"));
  delay(4000);

  // shoot for ~fifteen seconds of mag data
  // at 8 Hz ODR, new mag data is available every 125 ms
  if (Mmode == M_8HZ)
  {
    sample_count = 128;
  }
  // at 100 Hz ODR, new mag data is available every 10 ms
  if (Mmode == M_100HZ)
  {
    sample_count = 1500;
  }

  for (ii = 0; ii < sample_count; ii++)
  {
    readMagData(mag_temp);  // Read the mag data

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj])
      {
        mag_max[jj] = mag_temp[jj];
      }
      if (mag_temp[jj] < mag_min[jj])
      {
        mag_min[jj] = mag_temp[jj];
      }
    }

    if (Mmode == M_8HZ)
    {
      delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
    }
    if (Mmode == M_100HZ)
    {
      delay(12);  // At 100 Hz ODR, new mag data is available every 10 ms
    }
  }

  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  // Get 'average' x mag bias in counts
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
  // Get 'average' y mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  // Get 'average' z mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // Save mag biases in G for main program
  bias_dest[0] = (float)mag_bias[0] * mRes;// * factoryMagCalibration[0];
  bias_dest[1] = (float)mag_bias[1] * mRes;// * factoryMagCalibration[1];
  bias_dest[2] = (float)mag_bias[2] * mRes;// * factoryMagCalibration[2];

  // Get soft iron correction estimate
  // Get average x axis max chord length in counts
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
  // Get average y axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
  // Get average z axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  scale_dest[0] = avg_rad / ((float)mag_scale[0]);
  scale_dest[1] = avg_rad / ((float)mag_scale[1]);
  scale_dest[2] = avg_rad / ((float)mag_scale[2]);

  Serial.println(F("Mag Calibration done!"));
}

// Wire.h read and write protocols
uint8_t ICM20948::writeByte(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t data)
{
  if (_csPin != NOT_SPI)
  {
    return writeByteSPI(registerAddress, data);
  }
  else
  {
    return writeByteWire(deviceAddress,registerAddress, data);
  }
}

uint8_t ICM20948::writeByteSPI(uint8_t registerAddress, uint8_t writeData)
{
  uint8_t returnVal;

  SPI.beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
  select();

  SPI.transfer(registerAddress);
  returnVal = SPI.transfer(writeData);

  deselect();
  SPI.endTransaction();
#ifdef SERIAL_DEBUG
  Serial.print("ICM20948::writeByteSPI slave returned: 0x");
  Serial.println(returnVal, HEX);
#endif
  return returnVal;
}

uint8_t ICM20948::writeByteWire(uint8_t deviceAddress, uint8_t registerAddress,
                            uint8_t data)
{
  Wire.beginTransmission(deviceAddress);  // Initialize the Tx buffer
  Wire.write(registerAddress);      // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
  // TODO: Fix this to return something meaningful
  return NULL;
}

// Read a byte from given register on device. Calls necessary SPI or I2C
// implementation. This was configured in the constructor.
uint8_t ICM20948::readByte(uint8_t deviceAddress, uint8_t registerAddress)
{
  if (_csPin != NOT_SPI)
  {
    return readByteSPI(registerAddress);
  }
  else
  {
    return readByteWire(deviceAddress, registerAddress);
  }
}

// Read a byte from the given register address from device using I2C
uint8_t ICM20948::readByteWire(uint8_t deviceAddress, uint8_t registerAddress)
{
  uint8_t data; // `data` will store the register data

  // Initialize the Tx buffer
  Wire.beginTransmission(deviceAddress);
  // Put slave register address in Tx buffer
  Wire.write(registerAddress);
  // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(false);
  // Read one byte from slave register address
  Wire.requestFrom(deviceAddress, (uint8_t) 1);
  // Fill Rx buffer with result
  data = Wire.read();
  // Return data read from slave register
  return data;
}

// Read a byte from the given register address using SPI
uint8_t ICM20948::readByteSPI(uint8_t registerAddress)
{
  return writeByteSPI(registerAddress | READ_FLAG, 0xFF /*0xFF is arbitrary*/);
}

// Read 1 or more bytes from given register and device using I2C
uint8_t ICM20948::readBytesWire(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
  // Initialize the Tx buffer
  Wire.beginTransmission(deviceAddress);
  // Put slave register address in Tx buffer
  Wire.write(registerAddress);
  // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(false);

  uint8_t i = 0;
  // Read bytes from slave register address
  Wire.requestFrom(deviceAddress, count);
  while (Wire.available())
  {
    // Put read results in the Rx buffer
    dest[i++] = Wire.read();
  }

  return i; // Return number of bytes written
}

// Select slave IC by asserting CS pin
void ICM20948::select()
{
  digitalWrite(_csPin, LOW);
}

// Select slave IC by deasserting CS pin
void ICM20948::deselect()
{
  digitalWrite(_csPin, HIGH);
}

uint8_t ICM20948::readBytesSPI(uint8_t registerAddress, uint8_t count,
                           uint8_t * dest)
{
  SPI.beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
  select();

  SPI.transfer(registerAddress | READ_FLAG);

  uint8_t i;

  for (i = 0; i < count; i++)
  {
    dest[i] = SPI.transfer(0x00);
#ifdef SERIAL_DEBUG
    Serial.print("readBytesSPI::Read byte: 0x");
    Serial.println(dest[i], HEX);
#endif
  }

  SPI.endTransaction();
  deselect();

  delayMicroseconds(50);

  return i; // Return number of bytes written

  /*
#ifdef SERIAL_DEBUG
  Serial.print("ICM20948::writeByteSPI slave returned: 0x");
  Serial.println(returnVal, HEX);
#endif
  return returnVal;
  */

  /*
  // Set slave address of AK09916 and set AK09916 for read
  writeByteSPI(I2C_SLV0_ADDR, AK09916_ADDRESS | READ_FLAG);

Serial.print("\nBHW::I2C_SLV0_ADDR set to: 0x");
Serial.println(readByte(ICM20948_ADDRESS, I2C_SLV0_ADDR), HEX);

  // Set address to start read from
  writeByteSPI(I2C_SLV0_REG, registerAddress);
  // Read bytes from magnetometer
  //
Serial.print("\nBHW::I2C_SLV0_CTRL gets 0x");
Serial.println(READ_FLAG | count, HEX);

  // Read count bytes from registerAddress via I2C_SLV0
  Serial.print("BHW::readBytesSPI: return value test: ");
  Serial.println(writeByteSPI(I2C_SLV0_CTRL, READ_FLAG | count));
  */
}

uint8_t ICM20948::readBytes(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
  if (_csPin == NOT_SPI)  // Read via I2C
  {
    return readBytesWire(deviceAddress, registerAddress, count, dest);
  }
  else  // Read using SPI
  {
    return readBytesSPI(registerAddress, count, dest);
  }
}

bool ICM20948::magInit()
{
  // Reset registers to defaults, bit auto clears
  writeByteSPI(0x6B, 0x80);
  // Auto select the best available clock source
  writeByteSPI(0x6B, 0x01);
  // Enable X,Y, & Z axes of accel and gyro
  writeByteSPI(0x6C, 0x00);
  // Config disable FSYNC pin, set gyro/temp bandwidth to 184/188 Hz
  writeByteSPI(0x1A, 0x01);
  // Self tests off, gyro set to +/-2000 dps FS
  writeByteSPI(0x1B, 0x18);
  // Self test off, accel set to +/- 8g FS
  writeByteSPI(0x1C, 0x08);
  // Bypass DLPF and set accel bandwidth to 184 Hz
  writeByteSPI(0x1D, 0x09);
  // Configure INT pin (active high / push-pull / latch until read)
  writeByteSPI(0x37, 0x30);
  // Enable I2C master mode
  // TODO Why not do this 11-100 ms after power up?
  writeByteSPI(0x6A, 0x20);
  // Disable multi-master and set I2C master clock to 400 kHz
  //https://developer.mbed.org/users/kylongmu/code/MPU9250_SPI/ calls says
  // enabled multi-master... TODO Find out why
  writeByteSPI(0x24, 0x0D);
  // Set to write to slave address 0x0C
  writeByteSPI(0x25, 0x0C);
  // Point save 0 register at AK09916's control 2 (soft reset) register
  writeByteSPI(0x26, 0x0B);
  // Send 0x01 to AK09916 via slave 0 to trigger a soft restart
  writeByteSPI(0x63, 0x01);
  // Enable simple 1-byte I2C reads from slave 0
  writeByteSPI(0x27, 0x81);
  // Point save 0 register at AK09916's control 1 (mode) register
  writeByteSPI(0x26, 0x0A);
  // 16-bit continuous measurement mode 1
  writeByteSPI(0x63, 0x12);
  // Enable simple 1-byte I2C reads from slave 0
  writeByteSPI(0x27, 0x81);

  // TODO: Remove this code
  uint8_t ret = ak09916WhoAmI_SPI();
#ifdef SERIAL_DEBUG
  Serial.print("ICM20948::magInit to return ");
  Serial.println((ret == 0x48) ? "true" : "false");
#endif
  return ret == 0x48;
}

// Write a null byte w/o CS assertion to get SPI hardware to idle high (mode 3)
void ICM20948::kickHardware()
{
  SPI.beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
  SPI.transfer(0x00); // Send null byte
  SPI.endTransaction();
}

bool ICM20948::begin()
{
  kickHardware();
  return magInit();
}

// Read the WHOAMI (WIA) register of the AK09916
// TODO: This method has side effects
uint8_t ICM20948::ak09916WhoAmI_SPI()
{
  uint8_t response, oldSlaveAddress, oldSlaveRegister, oldSlaveConfig;
  // Save state
  oldSlaveAddress  = readByteSPI(I2C_SLV0_ADDR);
  oldSlaveRegister = readByteSPI(I2C_SLV0_REG);
  oldSlaveConfig   = readByteSPI(I2C_SLV0_CTRL);
#ifdef SERIAL_DEBUG
  Serial.print("Old slave address: 0x");
  Serial.println(oldSlaveAddress, HEX);
  Serial.print("Old slave register: 0x");
  Serial.println(oldSlaveRegister, HEX);
  Serial.print("Old slave config: 0x");
  Serial.println(oldSlaveConfig, HEX);
#endif

  // Set the I2C slave addres of AK09916 and set for read
  response = writeByteSPI(I2C_SLV0_ADDR, AK09916_ADDRESS|READ_FLAG);
  // I2C slave 0 register address from where to begin data transfer
  response = writeByteSPI(I2C_SLV0_REG, 0x00);
  // Enable 1-byte reads on slave 0
  response = writeByteSPI(I2C_SLV0_CTRL, 0x81);
  delayMicroseconds(1);
  // Read WIA register
  response = writeByteSPI(WHO_AM_I_AK09916|READ_FLAG, 0x00);

  // Restore state
  writeByteSPI(I2C_SLV0_ADDR, oldSlaveAddress);
  writeByteSPI(I2C_SLV0_REG, oldSlaveRegister);
  writeByteSPI(I2C_SLV0_CTRL, oldSlaveConfig);

  return response;
}
