use embassy_stm32::i2c::{I2c, Instance};
use embassy_time::Delay;

use mpu9250_i2c::{Mpu9250, calibration::Calibration};

pub fn get_imu<T: Instance>(i2c: I2c<T>, ) -> Mpu9250<I2c<T>, Delay> {
    let cal = Calibration::default();
    let mut mpu = Mpu9250::new(i2c, Delay, cal).unwrap();
    mpu.init().unwrap();
    return mpu
}
