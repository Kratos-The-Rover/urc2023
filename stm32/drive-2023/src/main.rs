#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_stm32::{peripherals::*, interrupt, usart::{UartTx, UartRx, Uart, self}};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_stm32::gpio::Pin;

use defmt::*;

use serde::{Serialize, Deserialize};
use serde_json_core as serde_json;

use heapless::{Deque, String};

// use {defmt_rtt as _, panic_probe as _};
use defmt_rtt as _;

pub mod cytron;
pub mod imu;
pub mod panic;

use panic::PANIC_CHANNEL;

const UART_BUF_SIZE: usize = 512;
const UART_Q_SIZE: usize = 1024;

static DRIVE_SIGNAL: Signal<ThreadModeRawMutex, DriveData> = Signal::new();
static CAM_SIGNAL: Signal<ThreadModeRawMutex, CameraFeed> = Signal::new();
static UART_SIGNAL: Signal<ThreadModeRawMutex, SomeData> = Signal::new();

#[derive(Serialize, Deserialize, defmt::Format)]
struct SomeData {
    speed_l: i8,
    speed_r: i8,
    reverse: bool,
    camera: CameraFeed
}

struct DriveData {
    speed_l: i8,
    speed_r: i8
}

#[derive(Serialize, Deserialize, defmt::Format)]
#[repr(C)]
enum CameraFeed {
    Chassis,
    Cam0,
    Cam1
}

fn find_first(q: &Deque<u8, UART_Q_SIZE>, u: u8) -> Option<usize> {
    for (i, val) in q.iter().enumerate() {
        if *val == u {
            return Some(i)
        }
    }
    None
}

#[embassy_executor::task]
async fn reader(mut uart_rx: UartRx<'static, USART2, DMA1_CH5>) {
    let mut rxque: Deque<u8, UART_Q_SIZE> = Deque::new();
    let mut rxbuf: [u8; UART_BUF_SIZE];
    let mut jsonbuf: String<512>;
    let mut s: SomeData;
    let mut c: char;
    loop {
        rxbuf = [0; UART_BUF_SIZE];
        uart_rx.read_until_idle(&mut rxbuf).await.unwrap();
        for c in rxbuf.iter() {
            rxque.push_back(*c).unwrap();
        }
        while let Some(i) = find_first(&rxque, 0) {
            jsonbuf = String::new();
            if i == 0 {
                rxque.clear();
                break;
            }
            for _ in 0..i+1 {
                c = rxque.pop_front().unwrap() as char;
                if c == 0 as char {
                    continue;
                }
                jsonbuf.push(c).unwrap();
            }
            s = serde_json::from_str::<SomeData>(jsonbuf.as_str()).unwrap().0;
            UART_SIGNAL.signal(s);
        }
    }
}

#[embassy_executor::task]
async fn processor() {
    let mut uart_data: SomeData;

    loop {
        uart_data = UART_SIGNAL.wait().await;
        DRIVE_SIGNAL.signal(DriveData{
            speed_l: uart_data.speed_l,
            speed_r: uart_data.speed_r
        });
    }
}

#[embassy_executor::task]
async fn motor_control(uart_tx: UartTx<'static, USART1, DMA2_CH7>) {
    let mut cyt = cytron::CytronSerial::new(uart_tx, None);

    cyt.control(0, 0).await;

    let mut d: DriveData;
    let panic_sub = PANIC_CHANNEL.subscriber().unwrap();

    loop {
        d = DRIVE_SIGNAL.wait().await;
        cyt.control(d.speed_l, d.speed_r).await;
        if panic_sub.available() != 0 {
            cyt.control(0, 0).await;
            break;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_stm32::init(Default::default());

    let irq = interrupt::take!(USART2);

    let mut uart2_config =  usart::Config::default();
    uart2_config.baudrate = 1000000;

    let mut uart1_config =  usart::Config::default();
    uart1_config.baudrate = 115200;

    let uart2 = Uart::new(peripherals.USART2, peripherals.PA3, peripherals.PA2, irq, peripherals.DMA1_CH6, peripherals.DMA1_CH5, uart2_config);

    let uart2_tx: UartTx<USART2, DMA1_CH6>;
    let uart2_rx: UartRx<USART2, DMA1_CH5>;

    (uart2_tx, uart2_rx) = uart2.split();

    let uart1_tx = UartTx::new(peripherals.USART1, peripherals.PA9, peripherals.DMA2_CH7, uart1_config);

    let p_wdt = peripherals.IWDG;

    unwrap!(spawner.spawn(reader(uart2_rx)));
    unwrap!(spawner.spawn(processor()));
    unwrap!(spawner.spawn(motor_control(uart1_tx)));
    unwrap!(spawner.spawn(panic::watchdog(p_wdt, peripherals.PC13.degrade())))
}
