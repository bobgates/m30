#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_semihosting; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger


extern crate byteorder;
use aligned::{Aligned, A4};
use byteorder::{ByteOrder, LE};
use cast::{f32, i32};
use core::{fmt::Write,
           f32::consts::PI,
           cell::Cell, 
           // ptr
       };
use cortex_m::{itm, 
			   iprintln,
			   // interrupt,
			   interrupt::Mutex,
			  };// Peripherals};
use cortex_m_rt::{entry,
				  // interrupt,
				 };
// use cortex_m_semihosting::{
//                     debug,
// 					hio::{self, hstdout, HStdout},
// 					hprintln,
// 				};
use embedded_graphics::{prelude::*, fonts::{ Font12x16, }};
use l3gd20::{L3gd20, Odr};
use lsm303dlhc::{Lsm303dlhc, AccelOdr, MagOdr};
use nb::block;
use madgwick::{F32x3, Marg};
use ssd1306::{prelude::*,
            //   mode::graphics::*,
              Builder,
             };	
use stm32f3::stm32f303::interrupt;
use stm32f3xx_hal::{prelude::*,
                     // gpio::{gpioe::Parts::pe3},
  			          spi::Spi,
			          i2c::{I2c,
			          		// SclPin,
			          		// SdaPin,
			          	   },	
			          timer::Timer,
			          delay::Delay,
			          // time::MonoTimer,
			          // serial::Serial,
			          stm32,
			          // usb::{Peripheral, UsbBus},
			       };   
// use f3::Leds;	     

/// Array of all the user LEDs on the board
// Number of samples to use for gyroscope calibration
const NSAMPLES: i32 = 256;

// Magnetometer calibration parameters
// NOTE you need to use the right parameters for *your* magnetometer
// You can use the `log-sensors` example to calibrate your magnetometer. The producer is explained
// in https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
const M_BIAS_X: f32 = -183.;
const M_SCALE_X: f32 = 435.;

const M_BIAS_Y: f32 = -172.;
const M_SCALE_Y: f32 = 507.;

const M_BIAS_Z: f32 = -136.;
const M_SCALE_Z: f32 = 632.;

// Sensitivities of the accelerometer and gyroscope, respectively
const K_G: f32 = 2. / (1 << 15) as f32; // LSB -> g
const K_AR: f32 = 8.75e-3 * PI / 180.; // LSB -> rad/s
// Madgwick filter parameters
const SAMPLE_FREQ: u32 = 220;
const BETA: f32 = 1e-3;

mod led;
use led::Leds;

static COUNTER : Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[interrupt]
fn TIM3(){
	static mut TICK: u32 = 0;
	*TICK = *TICK + 1;
}

#[entry]
fn main() -> ! {

	const DISPLAY : bool = false   ;

	let mut cp =  cortex_m::Peripherals::take().unwrap();
    let mut p = stm32::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
  	let mut rcc = p.RCC.constrain();
    let syst = cp.SYST;
    let exti = p.EXTI;

    cp.NVIC.enable(stm32f3::stm32f303::Interrupt::TIM3);
    
	let clocks = rcc
                    .cfgr
                    .sysclk(64.mhz())
        			.pclk1(32.mhz())
                    // .sysclk(16.mhz())
                    // .use_hse(8.mhz())
                    // .sysclk(24.mhz())
                    // .pclk1(16.mhz())
//                     .pclk2(24.mhz())
                    .freeze(&mut flash.acr);

    
	let stim = &mut cp.ITM.stim[0];
	let mut delay = Delay::new(syst, clocks);


	let mut gpioa = p.GPIOA.split(&mut rcc.ahb);
   	let mut gpiob = p.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = p.GPIOE.split(&mut rcc.ahb); //.split(&mut rcc.ahb);

	// let mut leds = led::Leds::new(gpioe);
	// let mut gpioe = gpioe.split(&mut rcc.ahb);

    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let mut marg = Marg::new(BETA, 1. / f32(SAMPLE_FREQ));
    let mut timer = Timer::tim2(p.TIM2, 380.hz(), clocks, &mut rcc.apb1);
    let mut tick = Timer::tim3(p.TIM3, 1.hz(), clocks, &mut rcc.apb1);

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
    let nss = gpioe.pe3.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let spi = Spi::spi1(
        p.SPI1,
        (sck, miso, mosi),
        l3gd20::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );
    let mut l3gd20 = L3gd20::new(spi, nss).unwrap();
    // iprintln!(stim, "l3gd20 initialized");

    let mut leds = Leds {
        leds: [
			gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
		    gpioe.pe10.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
		    gpioe.pe11.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
		    gpioe.pe12.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
		    gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
		    gpioe.pe14.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
		    gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
		    gpioe.pe8.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
        ],
    };

    let mut buffer: heapless::String<heapless::consts::U64> = heapless::String::new();

	let bus = shared_bus::CortexMBusManager::new(i2c);

    
 	
 	if DISPLAY{
	    let mut disp: GraphicsMode <_> = Builder::new()
	                                        .with_size(DisplaySize::Display128x32)
	                                        .connect_i2c(bus.acquire())
	                                        .into();       
	    disp.init().unwrap();
	    buffer.clear();
        // write!(buffer, "{}", yaw).unwrap();
        write!(buffer, "Hello").unwrap();
        disp.draw(
            Font12x16::render_str(&buffer)
                .translate(Coord::new(0, 0))
                .into_iter(),
        );
        buffer.clear();
        write!(buffer, "world").unwrap();
        disp.draw(
            Font12x16::render_str(&buffer)
                .translate(Coord::new(0, 16))
                .into_iter(),
        );
        buffer.clear();
            disp.flush().unwrap();
    }

    let mut lsm303dlhc = Lsm303dlhc::new(bus.acquire()).unwrap(); 
    lsm303dlhc.accel_odr(AccelOdr::Hz400).unwrap();
    lsm303dlhc.mag_odr(MagOdr::Hz220).unwrap();
      
    // iprintln!(stim, "lsm303 initialized");
    
    // iprintln!(stim, "mx, my, mz, gx, gy, gz, ax, ay, az");
            

    l3gd20.set_odr(Odr::Hz380).unwrap();
    let mut m = lsm303dlhc.mag().unwrap();
    let mut ar = l3gd20.all().unwrap().gyro;
    let mut g = lsm303dlhc.accel().unwrap();

    let period = 20_u16;
    let mut count = 0;
    let mut i = 0;
    let mut j = 6;
    let mut k = 7;

    let mut ar_bias_x = 0;
    let mut ar_bias_y = 0;
    let mut ar_bias_z = 0;    
    for _ in 0..NSAMPLES {
        block!(timer.wait()).unwrap();

        let ar = l3gd20.all().unwrap().gyro;

        ar_bias_x += i32(ar.x);
        ar_bias_y += i32(ar.y);
        ar_bias_z += i32(ar.z);
    }
    let ar_bias_x = (ar_bias_x / NSAMPLES) as i16;
    let ar_bias_y = (ar_bias_y / NSAMPLES) as i16;
    let ar_bias_z = (ar_bias_z / NSAMPLES) as i16;


   	// let mut tx_buf: Aligned<A4, [u8; 18]> = Aligned([0; 18]);
   	let mut tx_buf: Aligned<u32, [u8; 18]> = Aligned([0; 18]);

   	p.TIM2 = timer.release();
	let mut timer = Timer::tim2(p.TIM2, 400.hz(), clocks, &mut rcc.apb1);
    
    loop {
    	block!(timer.wait()).unwrap();

        m = lsm303dlhc.mag().unwrap();
        let m_x = (f32(m.x) - M_BIAS_X) / M_SCALE_X;
        let m_y = (f32(m.y) - M_BIAS_Y) / M_SCALE_Y;
        let m_z = (f32(m.z) - M_BIAS_Z) / M_SCALE_Z;
        let m = F32x3 {
            x: m_x,
            y: m_y,
            z: m_z,
        };

        ar = l3gd20.all().unwrap().gyro;
        let ar_x = f32(ar.x - ar_bias_x) * K_AR;
        let ar_y = f32(ar.y - ar_bias_y) * K_AR;
        let ar_z = f32(ar.z - ar_bias_z) * K_AR;
        let ar = F32x3 {
            x: ar_x,
            y: ar_y,
            z: ar_z,
        };
        g = lsm303dlhc.accel().unwrap();
        let g_x = f32(g.x) * K_G;
        let g_y = f32(g.y) * K_G;
        let g_z = f32(g.z) * K_G;
        let g = F32x3 {
            x: g_y,
            y: -g_x,
            z: g_z,
        };


    	// delay.delay_ms(period);
    	// if i==0 {
    	//  	// iprintln!(stim, "{}", count);
     //        iprintln!(stim, "{},{},{},{},{},{},{},{},{}",
     //                             m.x, m.y, m.z, 
     //                             g.x, g.y, g.z,
     //                             ar.x, ar.y, ar.z);
    	// }

    	let quat = marg.update(m, ar, g);
     // //    // Serialize the quaternion
     //    let mut start = 0;
     //    let mut buf = Aligned([0; 16]);
     //    LE::write_f32(&mut buf[start..start + 4], quat.0);
     //    start += 4;
     //    LE::write_f32(&mut buf[start..start + 4], quat.1);
     //    start += 4;
     //    LE::write_f32(&mut buf[start..start + 4], quat.2);
     //    start += 4;
     //    LE::write_f32(&mut buf[start..start + 4], quat.3);
     //    itm::write_aligned(stim, &buf);
    	
        // cobs::encode(&buf, &mut tx_buf);

        count += 1;
        if (count % 10) == 0 {
	    	leds[i].on();
	        leds[k].off();
	    	k=j;
	    	j=i;
	    	i = (i + 1) % 8
    	}
    	if (count % 1_000) == 0 {
    		// iprintln!(stim, "Seconds: {}", TICK);
    	}
    }
}
