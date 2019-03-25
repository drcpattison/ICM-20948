/*
 Note: The ICM-20948 is an I2C sensor and uses the Arduino Wire library.
 */
#ifndef _ICM20948_H_
#define _ICM20948_H_

#include <SPI.h>
#include <Wire.h>

#define SERIAL_DEBUG true

// See also ICM-20948 Datasheet, Register Map and Descriptions, Revision 1.3,
// https://www.invensense.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
// and AK09916 Datasheet and Register Map
// https://www.akm.com/akm/en/file/datasheet/AK09916C.pdf

//Magnetometer Registers
#define AK09916_ADDRESS  0x0C 
#define WHO_AM_I_AK09916 0x01 // (AKA WIA2) should return 0x09
#define AK09916_ST1      0x10  // data ready status bit 0
#define AK09916_XOUT_L   0x11  // data
#define AK09916_XOUT_H   0x12
#define AK09916_YOUT_L   0x13
#define AK09916_YOUT_H   0x14
#define AK09916_ZOUT_L   0x15
#define AK09916_ZOUT_H   0x16
#define AK09916_ST2      0x18  // Data overflow bit 3 and data read error status bit 2
#define AK09916_CNTL     0x30  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK09916_CNTL2    0x31  // Normal (0), Reset (1)

// ICM-20948

// USER BANK 0 REGISTER MAP
#define WHO_AM_I_ICM20948  0x00 // Should return 0xEA
#define USER_CTRL          0x03  // Bit 7 enable DMP, bit 3 reset DMP
#define LP_CONFIG		   0x05 // Not found in MPU-9250
#define PWR_MGMT_1         0x06 // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x07
#define INT_PIN_CFG        0x0F
#define INT_ENABLE         0x10
#define INT_ENABLE_1	   0x11 // Not found in MPU-9250
#define INT_ENABLE_2	   0x12 // Not found in MPU-9250
#define INT_ENABLE_3	   0x13 // Not found in MPU-9250
#define I2C_MST_STATUS     0x17
#define INT_STATUS         0x19
#define INT_STATUS_1	   0x1A // Not found in MPU-9250
#define INT_STATUS_2	   0x1B // Not found in MPU-9250
#define INT_STATUS_3	   0x1C // Not found in MPU-9250
#define DELAY_TIMEH		   0x28	// Not found in MPU-9250
#define DELAY_TIMEL		   0x29	// Not found in MPU-9250
#define ACCEL_XOUT_H       0x2D
#define ACCEL_XOUT_L       0x2E
#define ACCEL_YOUT_H       0x2F
#define ACCEL_YOUT_L       0x30
#define ACCEL_ZOUT_H       0x31
#define ACCEL_ZOUT_L       0x32
#define GYRO_XOUT_H        0x33
#define GYRO_XOUT_L        0x34
#define GYRO_YOUT_H        0x35
#define GYRO_YOUT_L        0x36
#define GYRO_ZOUT_H        0x37
#define GYRO_ZOUT_L        0x38
#define TEMP_OUT_H         0x39
#define TEMP_OUT_L         0x3A
#define EXT_SENS_DATA_00   0x3B
#define EXT_SENS_DATA_01   0x3C
#define EXT_SENS_DATA_02   0x3D
#define EXT_SENS_DATA_03   0x3E
#define EXT_SENS_DATA_04   0x3F
#define EXT_SENS_DATA_05   0x40
#define EXT_SENS_DATA_06   0x41
#define EXT_SENS_DATA_07   0x42
#define EXT_SENS_DATA_08   0x43
#define EXT_SENS_DATA_09   0x44
#define EXT_SENS_DATA_10   0x45
#define EXT_SENS_DATA_11   0x46
#define EXT_SENS_DATA_12   0x47
#define EXT_SENS_DATA_13   0x48
#define EXT_SENS_DATA_14   0x49
#define EXT_SENS_DATA_15   0x4A
#define EXT_SENS_DATA_16   0x4B
#define EXT_SENS_DATA_17   0x4C
#define EXT_SENS_DATA_18   0x4D
#define EXT_SENS_DATA_19   0x4E
#define EXT_SENS_DATA_20   0x4F
#define EXT_SENS_DATA_21   0x50
#define EXT_SENS_DATA_22   0x51
#define EXT_SENS_DATA_23   0x52
#define FIFO_EN_1          0x66
#define FIFO_EN_2          0x67 // Not found in MPU-9250
#define FIFO_RST		   0x68 // Not found in MPU-9250
#define FIFO_MODE		   0x69 // Not found in MPU-9250
#define FIFO_COUNTH        0x70
#define FIFO_COUNTL        0x71
#define FIFO_R_W           0x72
#define DATA_RDY_STATUS	   0x74 // Not found in MPU-9250
#define FIFO_CFG		   0x76 // Not found in MPU-9250
#define REG_BANK_SEL	   0x7F // Not found in MPU-9250

// USER BANK 1 REGISTER MAP
#define SELF_TEST_X_GYRO  			0x02
#define SELF_TEST_Y_GYRO  			0x03
#define SELF_TEST_Z_GYRO  			0x04
#define SELF_TEST_X_ACCEL 			0x0E
#define SELF_TEST_Y_ACCEL 			0x0F
#define SELF_TEST_Z_ACCEL 			0x10
#define XA_OFFSET_H       			0x14
#define XA_OFFSET_L       			0x15
#define YA_OFFSET_H       			0x17
#define YA_OFFSET_L       			0x18
#define ZA_OFFSET_H       			0x1A
#define ZA_OFFSET_L       			0x1B
#define TIMEBASE_CORRECTION_PLL		0x28

// USER BANK 2 REGISTER MAP
#define GYRO_SMPLRT_DIV        	0x00 // Not found in MPU-9250
#define GYRO_CONFIG_1      		0x01 // Not found in MPU-9250
#define GYRO_CONFIG_2      		0x02 // Not found in MPU-9250
#define XG_OFFSET_H       		0x03  // User-defined trim values for gyroscope
#define XG_OFFSET_L       		0x04
#define YG_OFFSET_H       		0x05
#define YG_OFFSET_L       		0x06
#define ZG_OFFSET_H       		0x07
#define ZG_OFFSET_L       		0x08
#define ODR_ALIGN_EN			0x09 // Not found in MPU-9250
#define ACCEL_SMPLRT_DIV_1     	0x10 // Not found in MPU-9250
#define ACCEL_SMPLRT_DIV_2     	0x11 // Not found in MPU-9250
#define ACCEL_INTEL_CTRL		0x12 // Not found in MPU-9250
#define ACCEL_WOM_THR			0x13 // Not found in MPU-9250 (could be WOM_THR)
#define ACCEL_CONFIG      		0x14
#define ACCEL_CONFIG_2     		0x15 // Not found in MPU-9250 (could be ACCEL_CONFIG2)
#define FSYNC_CONFIG			0x52 // Not found in MPU-9250
#define TEMP_CONFIG				0x53 // Not found in MPU-9250
#define MOD_CTRL_USR			0x54 // Not found in MPU-9250

// USER BANK 3 REGISTER MAP
#define I2C_MST_ODR_CONFIG		0x00 // Not found in MPU-9250
#define I2C_MST_CTRL       		0x01
#define I2C_MST_DELAY_CTRL 		0x02
#define I2C_SLV0_ADDR      		0x03
#define I2C_SLV0_REG       		0x04
#define I2C_SLV0_CTRL      		0x05
#define I2C_SLV0_DO        		0x06
#define I2C_SLV1_ADDR      		0x07
#define I2C_SLV1_REG       		0x08
#define I2C_SLV1_CTRL      		0x09
#define I2C_SLV1_DO        		0x0A
#define I2C_SLV2_ADDR      		0x0B
#define I2C_SLV2_REG       		0x0C
#define I2C_SLV2_CTRL      		0x0D
#define I2C_SLV2_DO        		0x0E
#define I2C_SLV3_ADDR      		0x0F
#define I2C_SLV3_REG       		0x10
#define I2C_SLV3_CTRL      		0x11
#define I2C_SLV3_DO        		0x12
#define I2C_SLV4_ADDR      		0x13
#define I2C_SLV4_REG       		0x14
#define I2C_SLV4_CTRL      		0x15
#define I2C_SLV4_DO        		0x16
#define I2C_SLV4_DI        		0x17


// Using the ICM-20948 breakout board, ADO is set to 1
// Seven-bit device address is 1000100 for ADO = 0 and 1000101 for ADO = 1
#define ADO 1
#if ADO
#define ICM20948_ADDRESS 0x69  // Device address when ADO = 1
#else
#define ICM20948_ADDRESS 0x68  // Device address when ADO = 0
#define AK09916_ADDRESS  0x0C   // Address of magnetometer
#endif // AD0

#define READ_FLAG 0x80
#define NOT_SPI -1
#define SPI_DATA_RATE 1000000 // 1MHz is the max speed of the ICM-20948
//#define SPI_DATA_RATE 1000000 // 1MHz is the max speed of the ICM-20948
#define SPI_MODE SPI_MODE3

class ICM20948
{
  protected:
    // Set initial input parameters
    enum Ascale
    {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum Gscale {
      GFS_250DPS = 0,
      GFS_500DPS,
      GFS_1000DPS,
      GFS_2000DPS
    };

    enum Mscale {
      MFS_14BITS = 0, // 0.6 mG per LSB
      MFS_16BITS      // 0.15 mG per LSB
    };

    enum M_MODE {
      M_8HZ = 0x02,  // 8 Hz update
      M_100HZ = 0x06 // 100 Hz continuous magnetometer
    };

    // TODO: Add setter methods for this hard coded stuff
    // Specify sensor full scale
    uint8_t Gscale = GFS_250DPS;
    uint8_t Ascale = AFS_2G;

    // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    uint8_t Mmode = M_100HZ;

    // SPI chip select pin
    int8_t _csPin;

    uint8_t writeByteWire(uint8_t, uint8_t, uint8_t);
    uint8_t writeByteSPI(uint8_t, uint8_t);
    uint8_t readByteSPI(uint8_t subAddress);
    uint8_t readByteWire(uint8_t address, uint8_t subAddress);
    bool magInit();
    void kickHardware();
    void select();
    void deselect();
// TODO: Remove this next line
public:
    uint8_t ak09916WhoAmI_SPI();

  public:
    float pitch, yaw, roll;
    float temperature;   // Stores the real internal chip temperature in Celsius
    int16_t tempCount;   // Temperature raw count output
    uint32_t delt_t = 0; // Used to control display output rate

    uint32_t count = 0, sumCount = 0; // used to control display output rate
    float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
    uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    uint32_t Now = 0;        // used to calculate integration interval

    int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
    // Scale resolutions per LSB for the sensors
    float aRes, gRes, mRes;
    // Variables to hold latest sensor data values
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    // Factory mag calibration and mag bias
    float factoryMagCalibration[3] = {0, 0, 0}, factoryMagBias[3] = {0, 0, 0};
    // Bias corrections for gyro, accelerometer, and magnetometer
    float gyroBias[3]  = {0, 0, 0},
          accelBias[3] = {0, 0, 0},
          magBias[3]   = {0, 0, 0},
          magScale[3]  = {0, 0, 0};
    float selfTest[6];
    // Stores the 16-bit signed accelerometer sensor output
    int16_t accelCount[3];

    // Public method declarations
    ICM20948(int8_t csPin=NOT_SPI);
    void getMres();
    void getGres();
    void getAres();
    void readAccelData(int16_t *);
    void readGyroData(int16_t *);
    void readMagData(int16_t *);
    int16_t readTempData();
    void updateTime();
    void initAK09916();
    void initICM20948();
    void calibrateICM20948(float * gyroBias, float * accelBias);
    void ICM20948SelfTest(float * destination);
    void magCalICM20948(float * dest1, float * dest2);
    uint8_t writeByte(uint8_t, uint8_t, uint8_t);
    uint8_t readByte(uint8_t, uint8_t);
    uint8_t readBytes(uint8_t, uint8_t, uint8_t, uint8_t *);
    // TODO: make SPI/Wire private
    uint8_t readBytesSPI(uint8_t, uint8_t, uint8_t *);
    uint8_t readBytesWire(uint8_t, uint8_t, uint8_t, uint8_t *);
    bool isInI2cMode() { return _csPin == -1; }
    bool begin();
};  // class ICM20948

#endif // _ICM20948_H_
