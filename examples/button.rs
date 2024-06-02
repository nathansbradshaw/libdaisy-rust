#![deny(warnings)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use libdaisy::{gpio, system, hal::gpio::{Edge, ExtiPin}};

    #[shared]
    struct SharedResources {}
    #[local]
    struct LocalResources {
        button: gpio::SeedButton,
        led: gpio::SeedLed,
    }

    use panic_halt as _;

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        let mut core = ctx.core;
        let mut device = ctx.device;
        let ccdr = system::System::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let mut system = libdaisy::system_init!(core, device, ccdr);

        // Button
        system.gpio.button.make_interrupt_source(&mut device.SYSCFG);
        system
            .gpio
            .button
            .trigger_on_edge(&mut device.EXTI, Edge::Rising);
        system.gpio.button.enable_interrupt(&mut device.EXTI);

        (
            SharedResources {},
            LocalResources {
                button: system.gpio.button,
                led: system.gpio.led,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = EXTI3, local = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.local.button.clear_interrupt_pending_bit();
        ctx.local.led.toggle();
    }
}
