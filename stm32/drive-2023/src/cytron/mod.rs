use embassy_stm32::{Peripheral, usart::{UartTx, BasicInstance}};

pub struct CytronSerial<'d, T: BasicInstance, S: Peripheral> {
    uart_tx: UartTx<'d, T, S>,
    board_id: Option<i8>
}

impl<'d, T: BasicInstance, S: Peripheral> CytronSerial<'d, T, S> {
    pub fn new(uart_tx: UartTx<'d, T, S>, board_id: Option<i8>) -> Self {
        CytronSerial {
            uart_tx,
            board_id
        }
    }

    pub fn control(&mut self, speed_l: i8, speed_r: i8) {
        let mut command_byte: u8;
        let motor_l: u8;
        let motor_r: u8;

        if let Some(_) = self.board_id {}

        if speed_l > 0 {
            command_byte = 0x0;
            motor_l = map(speed_l, 0, 100, 0, 63) as u8;
        } else {
            command_byte = 0x40;
            motor_l = map(speed_l, 0, -100, 0, 63) as u8;
        }

        command_byte = command_byte | motor_l;
        self.uart_tx.blocking_write(&[command_byte]).unwrap();

        if speed_r > 0 {
            command_byte = 0xC0;
            motor_r = map(speed_r, 0, 100, 0, 63) as u8;
        } else {
            command_byte = 0x80;
            motor_r = map(speed_r, 0, -100, 0, 63) as u8;
        }

        command_byte = command_byte | motor_r;
        self.uart_tx.blocking_write(&[command_byte]).unwrap();
    }
}

fn map(x: i8, in_min: i8, in_max: i8, out_min: i8, out_max: i8) -> i8 {
    return ( (x - in_min) * (out_max - out_min) / (in_max - in_min) ) + out_min
}
