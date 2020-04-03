#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_semihosting; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger
use core::{fmt::Write,
           f32::consts::PI,
           // ptr
       };
use cortex_m::{iprintln,};// Peripherals};
use cortex_m_rt::entry;
// use cortex_m_semihosting::{
//                     debug,
// 					hio::{self, hstdout, HStdout},
// 					hprintln,
// 				};
use embedded_graphics::{prelude::*, fonts::{ Font12x16, }};
use l3gd20::{L3gd20, Odr};
use lsm303dlhc::{Lsm303dlhc, AccelOdr, MagOdr};
use ssd1306::{prelude::*,
              mode::graphics::*,
              Builder,
             };	
use stm32f3xx_hal::{prelude::*,
                     // gpio::{gpioe::Parts::pe3},
  			          spi::Spi,
			          i2c::{I2c,
			          		// SclPin,
			          		// SdaPin,
			          	   },	
			          // timer::Timer,
			          delay::Delay,
			          // time::MonoTimer,
			          // serial::Serial,
			          stm32,
			          // usb::{Peripheral, UsbBus},
			       };   
	     

mod led; //::Leds;

#[entry]
fn main() -> ! {

	let mut cp =  cortex_m::Peripherals::take().unwrap();
    let p = stm32::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
  	let mut rcc = p.RCC.constrain();
    let mut syst = cp.SYST;

	let clocks = rcc
                    .cfgr
                    // .sysclk(16.mhz())
                    // .use_hse(8.mhz())
                    // .sysclk(24.mhz())
                    // .pclk1(16.mhz())
//                     .pclk2(24.mhz())
                    .freeze(&mut flash.acr);

    
	let stim = &mut cp.ITM.stim[0];
	let mut delay = Delay::new(syst, clocks);
	// let mut leds = led::Leds::new(gpioe);

	let mut gpioa = p.GPIOA.split(&mut rcc.ahb);
   	let mut gpiob = p.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = p.GPIOE.split(&mut rcc.ahb);

    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = I2c::i2c1(
        p.I2C1,
        (scl, sda),
        400.khz(),
        clocks,
        &mut rcc.apb1,
    );

    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mut nss = gpioe.pe3.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    nss.set_high().unwrap();

    let spi = Spi::spi1(
        p.SPI1,
        (sck, miso, mosi),
        l3gd20::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );
    let mut l3gd20 = L3gd20::new(spi, nss).unwrap();


    let bus = shared_bus::CortexMBusManager::new(i2c);
    let mut disp: GraphicsMode <_> = Builder::new()
                                        .with_size(DisplaySize::Display128x32)
                                        .connect_i2c(bus.acquire())
                                        .into();       
    disp.init().unwrap();

 	let mut buffer: heapless::String<heapless::consts::U64> = heapless::String::new();

                    buffer.clear();
                    // write!(buffer, "{}", yaw).unwrap();
                    write!(buffer, "Hello world").unwrap();
                    disp.draw(
                        Font12x16::render_str(&buffer)
                            .translate(Coord::new(0, 0))
                            .into_iter(),
                    );
                    buffer.clear();
                    disp.flush().unwrap();



    let mut lsm303dlhc = Lsm303dlhc::new(bus.acquire()).unwrap();   
    iprintln!(stim, "lsm303 initialized");



    let half_period = 150_u16;
    let mut count = 0;
    let mut i = 0;
    let mut j = 6;
    let mut k = 7;
    loop {
    	delay.delay_ms(half_period);
    	if i==0 {
    	 	iprintln!(stim, "{}", count);
    	 	count += 1;
    	}
    	
    	// leds[i].on();
    	// leds[k].off();
    	k=j;
    	j=i;
    	i = (i + 1) % 8
    }
}
