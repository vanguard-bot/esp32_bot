import machine, ubinascii, time, math
from machine import Pin, I2C
from time import sleep
from kalman import KalmanAngle

# personal touch
from lib.utilities import random_int


class MPU6050(object):
    def __init__(self, i2c, mpu6050_addr=0x68):
        """
        Class for managing one to many MPU6050 Six-Axis (Gyro + Accelerometer)
        :param i2c: is a machine.I2C object
        :param mpu6050_addr: is the address of the MPU6050 sensor standard address for MPU6050 starts at 0x68 or 104
        """
        self.i2c = i2c
        self.mpu6050_addr = mpu6050_addr  # 104
        self.kalmanX = KalmanAngle()
        self.kalmanY = KalmanAngle()
        # Required MPU6050 registers and their addresses
        self.PWR_MGMT_1 = 0x6B  # 107
        self.SMPLRT_DIV = 0x19  # 25
        self.CONFIG = 0x1A  # 26
        self.GYRO_CONFIG = 0x1B  # 27
        self.INT_ENABLE = 0x38  # 56
        self.ACCEL_XOUT_H = 0x3B  # 59
        self.ACCEL_YOUT_H = 0x3D  # 61
        self.ACCEL_ZOUT_H = 0x3F  # 63
        self.GYRO_XOUT_H = 0x43  # 67
        self.GYRO_YOUT_H = 0x45  # 69
        self.GYRO_ZOUT_H = 0x47  # 71
        self.TEMP_OUT_H = 0X41  # 65
        # Globals
        self.last_read_time = 0.0
        # These are the filtered angles
        self.last_x_angle = 0.0
        self.last_y_angle = 0.0
        self.last_z_angle = 0.0
        # Calibrated measurements to offset some bias or error in the readings.
        self.calib_x_accel = 0.0
        self.calib_y_accel = 0.0
        self.calib_z_accel = 0.0
        self.calib_x_gyro = 0.0
        self.calib_y_gyro = 0.0
        self.calib_z_gyro = 0.0
        self.counter = 0
        self.counter_interval = random_int()
        self.calibrate_sensors()

    def init_MPU(self):
        """
        Initializing the MPU6050
        :return:
        """
        # write to sample rate register
        self.i2c.writeto_mem(self.mpu6050_addr, self.SMPLRT_DIV, b'\x07')
        # Write to power management register to wake up mpu6050
        self.i2c.writeto_mem(self.mpu6050_addr, self.PWR_MGMT_1, b'\x00')
        # Write to Configuration register
        self.i2c.writeto_mem(self.mpu6050_addr, self.CONFIG, b'\x00')
        # Write to Gyro configuration register to self test gyro
        self.i2c.writeto_mem(self.mpu6050_addr, self.GYRO_CONFIG, b'\x18')
        # Set interrupt enable register to 0 .. disable interrupts
        self.i2c.writeto_mem(self.mpu6050_addr, self.INT_ENABLE, b'\x00')

    def read_raw_data(self, addr):
        """
        Reads out the raw data values
        :param addr: address of the value point to lookup
        :return: returns the value from the address
        """
        # Accelero and Gyro value are 16-bit
        high = self.i2c.readfrom_mem(self.mpu6050_addr, addr, 1)
        # print(ubinascii.hexlify(high))
        low = self.i2c.readfrom_mem(self.mpu6050_addr, addr + 1, 1)
        # print(ubinascii.hexlify(low))

        # concatenate higher and lower values
        val = high[0] << 8 | low[0]

        # we're expecting a 16 bit signed int (between -32768 to 32768). This step ensures 16 bit unsigned int raw readings are resolved.
        if (val > 32768):
            val = val - 65536
        return val

    def read_mpu6050(self):
        self.init_MPU()
        self.calibrate_sensors()
        try:
            while True:
                t_now = time.ticks_ms()
                dt = (t_now - self.get_last_time()) / 1000.0

                acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self.read_values_helper()

                # Full scale range +/- 250 degree/C as per sensitivity scale factor. The is linear acceleration in each of the 3 directions ins g's
                Ax = acc_x / 16384.0
                Ay = acc_y / 16384.0
                Az = acc_z / 16384.0

                # This is angular velocity in each of the 3 directions
                Gx = (gyro_x - self.calib_x_gyro) / 131.0
                Gy = (gyro_y - self.calib_y_gyro) / 131.0
                Gz = (gyro_z - self.calib_z_gyro) / 131.0

                # tp = temp/ 340 + 36.53

                # Calculate angle of inclination or tilt for the x and y axes with acquired acceleration vectors
                acc_angles = self.acc_angle(Ax, Ay, Az)

                # Calculate angle of inclination or tilt for x,y and z axes with angular rates and dt
                gyr_angles = self.gyr_angle(Gx, Gy, Gz, dt)

                # filtered tilt angle i.e. what we're after
                (c_angle_x, c_angle_y) = self.c_filtered_angle(acc_angles[0], acc_angles[1], gyr_angles[0], gyr_angles[ 1])
                (k_angle_x, k_angle_y) = self.k_filtered_angle(acc_angles[0], acc_angles[1], Gx, Gy, dt)

                self.set_last_read_angles(t_now, c_angle_x, c_angle_y)
                # print ("Gx=%.6f" %Gx, u'\u00b0'+ "/s", "\tGy=%.6f" %Gy, u'\u00b0'+ "/s", "\tGz=%.6f" %Gz, u'\u00b0'+ "/s", "\tAx=%.6f g" %Ax, "\tAy=%.6f g" %Ay, "\tAz=%.6f g" %Az, "\ttemp=%.6f" %tp, u'\u00b0'+ "C")

                print("%.8f," % c_angle_x, "%.8f," % c_angle_y, "%.8f," % k_angle_x, "%.8f" % k_angle_y)

                time.sleep_ms(4)

        except KeyboardInterrupt:
            pass

    def get_position(self):
        """
        Get a single x, y position
        :return:
        """
        if self.counter % self.counter_interval == 0:
            self.init_MPU()
            self.calibrate_sensors()
            self.counter = 1
        try:
            t_now = time.ticks_ms()
            dt = (t_now - self.get_last_time()) / 1000.0

            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self.read_values_helper()

            # Full scale range +/- 250 degree/C as per sensitivity scale factor. The is linear acceleration in each of the 3 directions ins g's
            Ax = acc_x / 16384.0
            Ay = acc_y / 16384.0
            Az = acc_z / 16384.0

            # This is angular velocity in each of the 3 directions
            Gx = (gyro_x - self.calib_x_gyro) / 131.0
            Gy = (gyro_y - self.calib_y_gyro) / 131.0
            Gz = (gyro_z - self.calib_z_gyro) / 131.0

            # tp = temp/ 340 + 36.53

            # Calculate angle of inclination or tilt for the x and y axes with acquired acceleration vectors
            acc_angles = self.acc_angle(Ax, Ay, Az)

            # Calculate angle of inclination or tilt for x,y and z axes with angular rates and dt
            gyr_angles = self.gyr_angle(Gx, Gy, Gz, dt)
            # print(gyr_angles)

            # filtered tilt angle i.e. what we're after
            (c_angle_x, c_angle_y) = self.c_filtered_angle(acc_angles[0], acc_angles[1], gyr_angles[0], gyr_angles[1])
            (k_angle_x, k_angle_y) = self.k_filtered_angle(acc_angles[0], acc_angles[1], Gx, Gy, dt)

            self.set_last_read_angles(t_now, c_angle_x, c_angle_y)
            # print ("Gx=%.6f" %Gx, u'\u00b0'+ "/s", "\tGy=%.6f" %Gy, u'\u00b0'+ "/s", "\tGz=%.6f" %Gz, u'\u00b0'+ "/s", "\tAx=%.6f g" %Ax, "\tAy=%.6f g" %Ay, "\tAz=%.6f g" %Az, "\ttemp=%.6f" %tp, u'\u00b0'+ "C")

            # print("%.8f," %c_angle_x, "%.8f," %c_angle_y, "%.8f," %k_angle_x,"%.8f" %k_angle_y)
            # time.sleep_ms(4)
            self.counter += 1
            return {"x": k_angle_x, "y": k_angle_y}
        except KeyboardInterrupt:
            pass
        except Exception as e:
            return{"Error": e}

    def calibrate_sensors(self):
        self.x_accel = 0
        self.y_accel = 0
        self.z_accel = 0
        self.x_gyro = 0
        self.y_gyro = 0
        self.z_gyro = 0

        # print("Starting Calibration")

        # Discard the first set of values read from the IMU
        self.read_values_helper()

        # Read and average the raw values from the IMU
        for int in range(10):
            values = self.read_values_helper()
            self.x_accel += values[0]
            self.y_accel += values[1]
            self.z_accel += values[2]
            self.x_gyro += values[3]
            self.y_gyro += values[4]
            self.z_gyro += values[5]
            time.sleep_ms(100)

        self.x_accel /= 10
        self.y_accel /= 10
        self.z_accel /= 10
        self.x_gyro /= 10
        self.y_gyro /= 10
        self.z_gyro /= 10

        # Store the raw calibration values globally
        self.calib_x_accel = self.x_accel
        self.calib_y_accel = self.y_accel
        self.calib_z_accel = self.z_accel
        self.calib_x_gyro = self.x_gyro
        self.calib_y_gyro = self.y_gyro
        self.calib_z_gyro = self.z_gyro

        # print("Finishing Calibration")

    def set_last_read_angles(self, time, x, y):
        self.last_read_time = time
        self.last_x_angle = x
        self.last_y_angle = y
        # last_z_angle = z

    # accelerometer data can't be used to calculate 'yaw' angles or rotation around z axis.
    def acc_angle(self, Ax, Ay, Az):
        radToDeg = 180 / 3.14159
        ax_angle = math.atan(Ay / math.sqrt(math.pow(Ax, 2) + math.pow(Az, 2))) * radToDeg
        ay_angle = math.atan((-1 * Ax) / math.sqrt(math.pow(Ay, 2) + math.pow(Az, 2))) * radToDeg
        return (ax_angle, ay_angle)

    def gyr_angle(self, Gx, Gy, Gz, dt):
        gx_angle = Gx * dt + self.get_last_x_angle()
        gy_angle = Gy * dt + self.get_last_y_angle()
        gz_angle = Gz * dt + self.get_last_z_angle()
        return (gx_angle, gy_angle, gz_angle)

    # A complementary filter to determine the change in angle by combining accelerometer and gyro values. Alpha depends on the sampling rate...
    def c_filtered_angle(self, ax_angle, ay_angle, gx_angle, gy_angle):
        alpha = 0.90
        c_angle_x = alpha * gx_angle + (1.0 - alpha) * ax_angle
        c_angle_y = alpha * gy_angle + (1.0 - alpha) * ay_angle
        return (c_angle_x, c_angle_y)

    # Kalman filter to determine the change in angle by combining accelerometer and gyro values.
    def k_filtered_angle(self, ax_angle, ay_angle, Gx, Gy, dt):
        k_angle_x = self.kalmanX.getAngle(ax_angle, Gx, dt)
        k_angle_y = self.kalmanY.getAngle(ay_angle, Gy, dt)
        return (k_angle_x, k_angle_y)

    def read_values_helper(self):
        # Read Accelerometer raw value
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

        # Read Gyroscope raw value
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

        # Read Temp raw value
        temp = self.read_raw_data(self.TEMP_OUT_H)

        return (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

    def get_last_time(self):
        return self.last_read_time

    def get_last_x_angle(self):
        return self.last_x_angle

    def get_last_y_angle(self):
        return self.last_y_angle

    def get_last_z_angle(self):
        return self.last_z_angle