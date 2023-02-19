use core::sync::atomic::{Ordering, AtomicBool};
use core::panic::PanicInfo;

use defmt;

use lazy_static::lazy_static;

use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::gpio::{AnyPin, Output, Level, Speed};
use embassy_stm32::peripherals::IWDG;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, pubsub::PubSubChannel, pubsub::Publisher};
use embassy_time::{Timer, Duration};

const WATCHDOG_RESET_TIME_US: u32 = 2_000_000;
static PANICKED: AtomicBool = AtomicBool::new(false);

pub static PANIC_CHANNEL: PubSubChannel<ThreadModeRawMutex, (), 1, 16, 1> = PubSubChannel::new();

lazy_static! {
    static ref PANIC_PUB: Publisher<'static, ThreadModeRawMutex, (), 1, 16, 1> = PANIC_CHANNEL.publisher().unwrap();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {

    // Guard against infinite recursion, just in case.
    if !PANICKED.load(Ordering::Relaxed) {
        PANICKED.store(true, Ordering::Relaxed);

        defmt::error!("{}", defmt::Display2Format(info));

        PANIC_PUB.publish_immediate(());

        defmt::info!("Restarting...");
    }

    loop {};
}

#[embassy_executor::task]
pub async fn watchdog(p_wdt: IWDG, led_pin: AnyPin) {
    PANICKED.store(false, Ordering::Relaxed);
    let mut dbug_led = Output::new(led_pin, Level::High, Speed::Low);

    let mut wdt = IndependentWatchdog::new(p_wdt, WATCHDOG_RESET_TIME_US);

    for _ in 0..49 {
        Timer::after(Duration::from_micros((WATCHDOG_RESET_TIME_US / 50).into())).await;
        dbug_led.toggle();
    }
    unsafe {
        wdt.unleash();
    }

    loop {
        Timer::after(Duration::from_micros((WATCHDOG_RESET_TIME_US / 2).into())).await;
        if !PANICKED.load(Ordering::Relaxed) {
            unsafe {
                wdt.pet();
            }
            dbug_led.toggle();
        }
    }
}

