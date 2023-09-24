use core::marker::PhantomData;

use avr_hal_generic::i2c::{Direction, Error};
// use embedded_hal::prelude::{
//     _embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_delay_DelayUs,
// };

use super::hal;

type Speed = hal::clock::MHz1;
pub type I2C = I2c<hal::port::PB0, hal::port::PB2, Speed>;

enum IoPin<Pin> {
    Input(hal::port::Pin<hal::port::mode::Input, Pin>),
    Output(hal::port::Pin<hal::port::mode::Output, Pin>),
    None,
}

impl<Pin> IoPin<Pin>
where
    Pin: hal::port::PinOps,
{
    #[inline]
    pub fn to_floating_input(&mut self) -> &mut hal::port::Pin<hal::port::mode::Input, Pin> {
        *self = match core::mem::replace(self, Self::None) {
            Self::Output(pin) => Self::Input(pin.into_floating_input().forget_imode()),
            s => s,
        };
        match self {
            Self::Input(pin) => pin,
            _ => unreachable!(),
        }
    }

    #[inline]
    pub fn as_pull_up_input(&mut self) -> &mut hal::port::Pin<hal::port::mode::Input, Pin> {
        *self = match core::mem::replace(self, Self::None) {
            Self::Output(pin) => Self::Input(pin.into_pull_up_input().forget_imode()),
            s => s,
        };
        match self {
            Self::Input(pin) => pin,
            _ => unreachable!(),
        }
    }

    #[inline]
    pub fn as_output(&mut self) -> &mut hal::port::Pin<hal::port::mode::Output, Pin> {
        *self = match core::mem::replace(self, Self::None) {
            Self::Input(pin) => Self::Output(pin.into_output()),
            s => s,
        };
        match self {
            Self::Output(pin) => pin,
            _ => unreachable!(),
        }
    }

    #[inline]
    pub fn as_output_high(&mut self) -> &mut hal::port::Pin<hal::port::mode::Output, Pin> {
        *self = match core::mem::replace(self, Self::None) {
            Self::Input(pin) => Self::Output(pin.into_output_high()),
            s => s,
        };
        match self {
            Self::Output(pin) => pin,
            _ => unreachable!(),
        }
    }
}

pub struct I2c<SDAPIN, SCLPIN, CLOCK> {
    p: hal::pac::USI,
    sda: IoPin<SDAPIN>,
    scl: hal::port::Pin<hal::port::mode::Output, SCLPIN>,
    _clock: PhantomData<CLOCK>,
    // delay: hal::delay::Delay::<hal::clock::MHz1>
}

impl<SDAPIN, SCLPIN, CLOCK> I2c<SDAPIN, SCLPIN, CLOCK>
where
    SDAPIN: hal::port::PinOps,
    SCLPIN: hal::port::PinOps,
{
    #[inline]
    fn raw_setup(&mut self) {
        // Preload data register with "released level" data.
        self.p.usidr.write(|w| w.bits(0xff));
        self.p.usicr.write(|w| {
            // Set USI in Two-wire mode. Method is incorrectly named.
            w.usiwm()
                .two_wire_slave()
                // Software stobe as counter clock source
                .usics()
                .ext_pos()
                .usiclk()
                .set_bit()
        });
        self.p.usisr.write(|w| {
            // Clear flags
            w.usisif()
                .set_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                // Reset counter
                .usicnt()
                .bits(0)
        });
    }

    #[inline]
    fn raw_start(&mut self, address: u8, direction: Direction) -> Result<(), Error> {
        // Send start
        let sda = self.sda.as_output_high();
        // self.delay.delay_us(2u16);
        // Release SCL
        self.scl.set_high();
        // FIXME: example code waits for PINx bit to go high, why?
        // Pull SDA low
        sda.set_low();
        // Pull SCL low
        self.scl.set_low();
        // Release SDA
        sda.set_high();

        let dirbit = if direction == avr_hal_generic::i2c::Direction::Read {
            1
        } else {
            0
        };
        let rawaddr = (address << 1) | dirbit;
        self.raw_write_bits(rawaddr, 8);
        if self.raw_read_bits(1) & 0b1 != 0 {
            return Err(Error::AddressNack);
        }

        Ok(())
    }

    #[inline]
    fn raw_write(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for byte in bytes {
            self.raw_write_bits(*byte, 8);
            if self.raw_read_bits(1) & 0b1 != 0 {
                return Err(Error::DataNack);
            }
        }
        Ok(())
    }

    #[inline]
    fn raw_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let last = buffer.len() - 1;
        for (i, byte) in buffer.iter_mut().enumerate() {
            *byte = self.raw_read_bits(8);

            self.raw_write_bits(if i != last { 0x00 } else { 0xFF }, 1);
        }
        Ok(())
    }

    #[inline]
    fn raw_stop(&mut self) -> Result<(), Error> {
        // Pull SDA low
        let sda = self.sda.as_output();
        // Release
        self.scl.set_high();
        // FIXME: wait for scl to go high
        // Release SDA
        sda.set_high();
        Ok(())
    }

    #[inline]
    fn raw_write_bits(&mut self, data: u8, bits: u8) {
        self.sda.as_output_high();
        // Pull SCL low
        self.scl.set_low();
        self.p.usidr.write(|w| w.bits(data));
        self.raw_transfer(bits);
        // Release SDA
        self.p.usidr.write(|w| w.bits(0xFF));
    }

    #[inline]
    fn raw_read_bits(&mut self, bits: u8) -> u8 {
        self.sda.as_pull_up_input();
        self.raw_transfer(bits);
        let value = self.p.usidr.read().bits();
        // Release SDA
        self.p.usidr.write(|w| w.bits(0xFF));
        self.sda.as_output_high();
        value
    }

    #[inline]
    fn raw_transfer(&mut self, bits: u8) -> u8 {
        self.p.usisr.write(|w| {
            w.usisif()
                .set_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(16 - bits * 2)
        });
        while self.p.usisr.read().usioif().bit_is_clear() {
            self.p.usicr.write(|w| {
                // Set USI in Two-wire mode. Method is incorrectly named.
                w.usiwm()
                    .two_wire_slave()
                    // Software stobe as counter clock source
                    .usics()
                    .ext_pos()
                    .usiclk()
                    .set_bit()
                    // Toggle Clock Port
                    .usitc()
                    .set_bit()
            });
        }
        let data = self.p.usidr.read().bits();
        data
    }
}

impl<SDAPIN, SCLPIN, CLOCK> I2c<SDAPIN, SCLPIN, CLOCK>
where
    SDAPIN: hal::port::PinOps,
    SCLPIN: hal::port::PinOps,
{
    pub fn with_external_pullup(
        p: hal::pac::USI,
        sda: hal::port::Pin<hal::port::mode::Input<hal::port::mode::Floating>, SDAPIN>,
        scl: hal::port::Pin<hal::port::mode::Input<hal::port::mode::Floating>, SCLPIN>,
        // delay: hal::delay::Delay::<hal::clock::MHz1>
    ) -> Self {
        let mut i2c = Self {
            p,
            sda: IoPin::Input(sda.forget_imode()),
            scl: scl.into_output_high(),
            _clock: PhantomData,
            // delay
        };
        i2c.raw_setup();

        i2c
    }
}

impl<SDAPIN, SCLPIN, CLOCK> embedded_hal::blocking::i2c::Write for I2c<SDAPIN, SCLPIN, CLOCK>
where
    SDAPIN: hal::port::PinOps,
    SCLPIN: hal::port::PinOps,
{
    type Error = Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.raw_start(address, Direction::Write)?;
        self.raw_write(bytes)?;
        self.raw_stop()?;
        Ok(())
    }
}



impl<SDAPIN, SCLPIN, CLOCK> embedded_hal::blocking::i2c::WriteRead for I2c<SDAPIN, SCLPIN, CLOCK>
where
    SDAPIN: hal::port::PinOps,
    SCLPIN: hal::port::PinOps,
{
    type Error = Error;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.raw_start(address, Direction::Write)?;
        self.raw_write(bytes)?;
        self.raw_start(address, Direction::Read)?;
        self.raw_read(buffer)?;
        self.raw_stop()?;
        Ok(())
    }
}

// TODO Need test
impl<SDAPIN, SCLPIN, CLOCK> embedded_hal::blocking::i2c::Read for I2c<SDAPIN, SCLPIN, CLOCK>
    where
    SDAPIN: hal::port::PinOps,
    SCLPIN: hal::port::PinOps,
{
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.raw_start(address, Direction::Read)?;
        self.raw_read(buffer)?;
        self.raw_stop()?;
        Ok(())
    }
}
