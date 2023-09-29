use attiny_hal as hal;
use avr_hal_generic::i2c::Direction;
use hal::{clock::MHz1, delay::Delay, prelude::*};

#[derive(thiserror::Error, Debug)]
pub enum UsiError {
    #[error("Generated Stop Condition not detected on bus")]
    UsiTwiMissingStopCon,
    #[error("Generated Start Condition not detected on bus")]
    UsiTwiMissingStartCon,
    #[error("The slave did not acknowledge the address")]
    UsiTwiNoAckOnAddress,
    #[error("The slave did not acknowledge all data")]
    UsiTwiNoAckOnData,
}

pub enum IoPin<Pin> {
    Input(hal::port::Pin<hal::port::mode::Input, Pin>),
    Output(hal::port::Pin<hal::port::mode::Output, Pin>),
    None,
}

impl<Pin> IoPin<Pin>
where
    Pin: hal::port::PinOps,
{
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
            Self::Output(pin) => Self::Output(pin.into_output()),
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
            Self::Output(pin) => Self::Output(pin.into_output_high()),
            s => s,
        };
        match self {
            Self::Output(pin) => pin,
            _ => unreachable!(),
        }
    }

    pub fn is_low(&self) -> bool {
        match self {
            Self::Input(v) => v.is_low(),
            Self::Output(v) => v.is_set_low(),
            _ => false,
        }
    }
}

pub enum SlavePollEvent {
    None,

    /// Emitted when the I2C controller addresses us in read mode.
    StartRead,

    /// Emitted when the I2C controller addresses us in write mode.
    StartWrite,
}

#[derive(PartialEq)]
pub enum SlaveWriteResult {
    Stop,
    Continue,
}

pub enum SlaveReadResult {
    Stop(Option<u8>),
    Continue(u8),
}

enum WaitResult {
    Stop,
    Continue,
}

pub struct TwoWire {
    pub address: Option<u8>,
    pub fast_mode: bool,
    pub delay: Delay<MHz1>,
    pub usi: hal::pac::USI,
    pub sda: IoPin<hal::port::PB0>,
    pub scl: IoPin<hal::port::PB2>,
}

impl TwoWire {
    fn delay_t2twi(&mut self) {
        let time = if self.fast_mode { 2u8 } else { 5u8 };
        self.delay.delay_us(time);
    }

    fn delay_t4twi(&mut self) {
        let time = if self.fast_mode { 1u8 } else { 4u8 };
        self.delay.delay_us(time);
    }

    fn usi_twi_master_initialize(&mut self) {
        let usi = &self.usi;
        self.scl.as_output_high();
        self.sda.as_output_high();
        usi.usidr.write(|w| w.bits(0xFF));
        // i2c uses two_wire_slave
        usi.usicr.write(|w| {
            w.usiwm()
                .two_wire_slave()
                .usics()
                .ext_pos()
                .usiclk()
                .set_bit()
        });
        usi.usisr.write(|w| {
            w.usisif()
                .set_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(0)
        });
    }

    fn usi_twi_master_toggle_scl(&self) {
        self.usi.usicr.write(|w| {
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

    fn wait_scl_go_high(&self) {
        while self.scl.is_low() {}
    }

    fn usi_overflow_not_detected(&self) -> bool {
        self.usi.usisr.read().usioif().bit_is_clear()
    }

    fn wait_usi_overflow(&self) -> WaitResult {
        loop {
            let usisr = self.usi.usisr.read();
            if usisr.usipf().bit_is_set() {
                return WaitResult::Stop;
            }
            if usisr.usioif().bit_is_set() {
                break;
            }
        }
        WaitResult::Continue
    }

    fn usi_twi_master_transfer(&mut self, bits: u8) -> u8 {
        self.usi.usisr.write(|w| {
            w.usisif()
                .set_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(16 - 2 * bits)
        });

        while self.usi_overflow_not_detected() {
            self.delay_t2twi();
            self.usi_twi_master_toggle_scl();
            self.wait_scl_go_high();
            self.delay_t4twi();
            self.usi_twi_master_toggle_scl();
        }
        self.delay_t2twi();

        let data = self.usi.usidr.read().bits();
        self.usi.usidr.write(|w| w.bits(0xFF));
        self.sda.as_output_high();
        data
    }

    fn raw_stop(&mut self) -> Result<(), UsiError> {
        self.sda.as_output();
        self.scl.as_output_high();
        self.wait_scl_go_high();
        self.delay_t4twi();
        self.sda.as_output_high();
        self.delay_t2twi();
        if self.usi.usisr.read().usipf().bit_is_clear() {
            return Err(UsiError::UsiTwiMissingStopCon);
        }
        Ok(())
    }

    fn start_condition_not_detected(&self) -> bool {
        self.usi.usisr.read().usisif().bit_is_clear()
    }

    fn raw_start(&mut self, address: u8, direction: Direction) -> Result<(), UsiError> {
        self.scl.as_output_high();
        self.wait_scl_go_high();
        self.delay_t4twi();
        self.sda.as_output();
        self.delay_t4twi();
        self.scl.as_output();
        self.sda.as_output_high();
        if self.start_condition_not_detected() {
            return Err(UsiError::UsiTwiMissingStartCon);
        }
        self.raw_write_bits((address << 1) | direction as u8)
            .map_err(|_| UsiError::UsiTwiNoAckOnAddress)
    }

    fn raw_write_bits(&mut self, data: u8) -> Result<(), UsiError> {
        // self.scl.as_output();
        self.sda.as_output_high();
        self.usi.usidr.write(|w| w.bits(data));
        self.usi_twi_master_transfer(8);
        self.sda.as_pull_up_input();
        if self.usi_twi_master_transfer(1) & 0b1 != 0 {
            Err(UsiError::UsiTwiNoAckOnData)
        } else {
            Ok(())
        }
    }

    fn raw_read_bits(&mut self, last: bool) -> u8 {
        self.sda.as_pull_up_input();
        let data = self.usi_twi_master_transfer(8);
        self.usi
            .usidr
            .write(|w| w.bits(if last { 0xFF } else { 0x00 }));
        self.usi_twi_master_transfer(1);
        data
    }

    pub fn begin(&mut self, address: Option<u8>) {
        self.address = address;
        match address {
            Some(_) => self.usi_twi_slave_initialize(),
            None => self.usi_twi_master_initialize(),
        };
    }

    pub fn master_read(&mut self, buffer: &mut [u8]) -> Result<(), UsiError> {
        let last = buffer.len() - 1;
        for (i, byte) in buffer.iter_mut().enumerate() {
            *byte = self.raw_read_bits(i == last);
        }
        Ok(())
    }

    /// put 0 on first byte of msg
    pub fn master_write(&mut self, bytes: &[u8]) -> Result<(), UsiError> {
        for data in bytes {
            self.raw_write_bits(*data)?;
        }
        Ok(())
    }

    fn usi_twi_slave_initialize(&mut self) {
        self.scl.as_output_high();
        self.sda.as_pull_up_input();
        self.usi.usicr.write(|w| {
            w.usiwm()
                .two_wire_slave()
                .usics()
                .ext_pos()
                .usiclk()
                .clear_bit()
        });
        // Clear all flags and reset overflow counter
        // TODO replace by safe methods
        self.usi.usisr.write(|w| unsafe { w.bits(0xF0) });
    }

    fn slave_end(&mut self) {
        self.scl.as_pull_up_input();
        self.sda.as_pull_up_input();
        self.usi.usicr.write(|w| unsafe { w.bits(0x00) });
        self.usi.usisr.write(|w| unsafe { w.bits(0xF0) })
    }

    // TODO change to basic polling (or maybe async)
    pub fn slave_poll(&mut self) -> SlavePollEvent {
        while self.start_condition_not_detected() {}
        self.usi.usicr.write(|w| {
            w.usiwm()
                .two_wire_master()
                .usics()
                .ext_pos()
                .usiclk()
                .clear_bit()
                .usitc()
                .clear_bit()
        });
        self.usi.usisr.write(|w| {
            w.usisif()
                .set_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(0)
        });
        // check address
        while self.usi_overflow_not_detected() {}
        let data = self.usi.usidr.read().bits();
        if self.address.unwrap() == data >> 1 {
            self.set_usi_to_send_ack();
            while self.usi_overflow_not_detected() {}
            if data & 0x01 == 1 {
                SlavePollEvent::StartWrite
            } else {
                SlavePollEvent::StartRead
            }
        } else {
            self.set_usi_to_twi_start_condition_mode();
            SlavePollEvent::None
        }
    }

    pub fn slave_write(&mut self, data: Option<u8>) -> SlaveWriteResult {
        match data {
            Some(v) => {
                self.usi.usidr.write(|w| w.bits(v));
                self.set_usi_to_send_data();
            }
            None => {
                self.set_usi_to_send_nack();
                return SlaveWriteResult::Stop;
            }
        };
        while self.usi_overflow_not_detected() {}
        self.set_usi_to_read_ack();
        while self.usi_overflow_not_detected() {}
        let data = self.usi.usidr.read().bits();
        if data & 0b1 != 0 {
            // If NACK, the master does not want more data.
            self.set_usi_to_twi_start_condition_mode();
            SlaveWriteResult::Stop
        } else {
            SlaveWriteResult::Continue
        }
    }

    pub fn slave_read(&mut self) -> SlaveReadResult {
        self.set_usi_to_read_data();
        match self.wait_usi_overflow() {
            WaitResult::Stop => return SlaveReadResult::Stop(None),
            WaitResult::Continue => {}
        };
        let data = self.usi.usidr.read().bits();
        self.set_usi_to_send_ack();
        match self.wait_usi_overflow() {
            WaitResult::Stop => SlaveReadResult::Stop(Some(data)),
            WaitResult::Continue => SlaveReadResult::Continue(data),
        }
    }

    fn set_usi_to_send_ack(&mut self) {
        self.usi.usidr.write(|w| w.bits(0));
        self.sda.as_output();
        self.usi.usisr.write(|w| {
            w.usisif()
                .clear_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(0x0E)
        });
    }

    fn set_usi_to_send_nack(&mut self) {
        self.sda.as_pull_up_input();
        self.usi.usisr.write(|w| {
            w.usisif()
                .clear_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(0x0E)
        });
    }

    fn set_usi_to_read_ack(&mut self) {
        self.sda.as_pull_up_input();
        self.usi.usidr.write(|w| w.bits(0));
        self.usi.usisr.write(|w| {
            w.usisif()
                .clear_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(0x0E)
        });
    }

    fn set_usi_to_twi_start_condition_mode(&mut self) {
        self.sda.as_pull_up_input();
        self.usi.usicr.write(|w| {
            w.usiwm()
                .two_wire_slave()
                .usics()
                .ext_pos()
                .usiclk()
                .clear_bit()
        });
        self.usi.usisr.write(|w| {
            w.usisif()
                .clear_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(0)
        });
    }

    fn set_usi_to_send_data(&mut self) {
        self.sda.as_output_high(); // maybe high
        self.usi
            .usisr
            .write(|w| w.usioif().set_bit().usipf().set_bit().usicnt().bits(0));
    }

    fn set_usi_to_read_data(&mut self) {
        self.sda.as_pull_up_input();
        self.usi.usisr.write(|w| {
            w.usisif()
                .clear_bit()
                .usioif()
                .set_bit()
                .usipf()
                .set_bit()
                .usicnt()
                .bits(0)
        });
    }
}

impl embedded_hal::blocking::i2c::Write for TwoWire {
    type Error = UsiError;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.raw_start(address, Direction::Write)?;
        self.master_write(bytes)?;
        self.raw_stop()?;
        Ok(())
    }
}

impl embedded_hal::blocking::i2c::WriteRead for TwoWire {
    type Error = UsiError;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        
        self.raw_start(address, Direction::Write)?;
        self.master_write(bytes)?;
        self.raw_start(address, Direction::Read)?;
        self.master_read(buffer)?;
        self.raw_stop()?;
        Ok(())
    }
}

impl embedded_hal::blocking::i2c::Read for TwoWire {
    type Error = UsiError;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.raw_start(address, Direction::Read)?;
        self.master_read(buffer)?;
        self.raw_stop()?;
        Ok(())
    }
}
