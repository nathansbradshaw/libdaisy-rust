//! examples/sdram.rs
#![no_main]
#![no_std]

#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true,
)]
mod app {
    // Includes a panic handler and optional logging facilities
    use libdaisy::logger;
    use libdaisy::{gpio::*, system};
    use log::info;
    use micromath::F32Ext;
    use stm32h7xx_hal::{stm32, time::MilliSeconds, timer::Timer};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        seed_led: SeedLed,
        timer2: Timer<stm32::TIM2>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();
        let mut core = ctx.core;
        let device = ctx.device;
        let ccdr = system::System::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let system = libdaisy::system_init!(core, device, ccdr);
        info!("Startup done!");
        let mut timer2 = stm32h7xx_hal::timer::TimerExt::timer(
            device.TIM2,
            MilliSeconds::from_ticks(100).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );
        timer2.listen(stm32h7xx_hal::timer::Event::TimeOut);

        timer2.set_freq(MilliSeconds::from_ticks(500).into_rate());

        let sdram = system.sdram;

        let sdram_size_bytes = libdaisy::sdram::Sdram::bytes();
        let sdram_size = sdram_size_bytes / core::mem::size_of::<u32>();

        info!(
            "SDRAM size: {} bytes, {} words starting at {:?}",
            sdram_size_bytes, sdram_size, &sdram[0] as *const _
        );

        // Make sure that we're not reading memory from a previous test run
        info!("Clear memory...");
        for item in sdram.iter_mut().take(sdram_size) {
            *item = 0.0;
        }

        info!("Write test pattern...");
        let mut data: f32 = 0.0;
        for item in sdram.iter_mut().take(sdram_size) {
            *item = data;
            data = (data + 1.0) % core::f32::MAX;
        }

        info!("Read test pattern...");
        let percent = (sdram_size as f64 / 100.0) as f32;
        data = 0.0;
        for (i, item) in sdram.iter_mut().enumerate().take(sdram_size) {
            assert!((*item - data).abs() < f32::EPSILON);
            data = (data + 1.0) % core::f32::MAX;

            if (i as f32 % (10.0 * percent)) == 0.0 {
                info!("{}% done", i as f32 / percent);
            }
        }
        info!("Test Success!");

        (
            Shared {},
            Local {
                seed_led: system.gpio.led,
                timer2,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = TIM2, local = [timer2, seed_led, led_is_on: bool = true])]
    fn blink(ctx: blink::Context) {
        ctx.local.timer2.clear_irq();

        if *ctx.local.led_is_on {
            ctx.local.seed_led.set_high();
        } else {
            ctx.local.seed_led.set_low();
        }
        *ctx.local.led_is_on = !(*ctx.local.led_is_on);
    }
}
