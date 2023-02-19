use embassy_stm32::{Peripheral, usart::{UartTx, BasicInstance, TxDma}};

use defmt::*;

pub struct CytronSerial<'d, T: BasicInstance, S: TxDma<T>> {
    uart_tx: UartTx<'d, T, S>,
    board_id: Option<i8>
}

impl<'d, T: BasicInstance, S: TxDma<T>> CytronSerial<'d, T, S> {
    pub fn new(uart_tx: UartTx<'d, T, S>, board_id: Option<i8>) -> Self {
        CytronSerial {
            uart_tx,
            board_id
        }
    }

    pub async fn control(&mut self, speed_l: i8, speed_r: i8) {
        let mut command_byte: u8;
        let motor_l: u8;
        let motor_r: u8;

        if let Some(_) = self.board_id {}

        if speed_l > 0 {
            command_byte = 0x0;
            motor_l = map(speed_l.into(), 0, 127, 0, 63) as u8;
        } else {
            command_byte = 0x40;
            motor_l = map(speed_l.into(), 0, -128, 0, 63) as u8;
        }

        command_byte = command_byte | motor_l;
        self.uart_tx.write(&[command_byte]).await.unwrap();

        if speed_r > 0 {
            command_byte = 0xC0;
            motor_r = map(speed_r.into(), 0, 127, 0, 63) as u8;
        } else {
            command_byte = 0x80;
            motor_r = map(speed_r.into(), 0, -128, 0, 63) as u8;
        }

        command_byte = command_byte | motor_r;
        self.uart_tx.write(&[command_byte]).await.unwrap();
    }
}

fn map(x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> i8 {
    let y = ( (x - in_min) * (out_max - out_min) / (in_max - in_min) ) + out_min;
    return y.try_into().unwrap()
}
