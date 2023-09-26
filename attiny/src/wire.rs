use attiny_hal as hal;
use avr_hal_generic::i2c::Direction;
use core::sync::atomic::AtomicBool;
use hal::{clock::MHz1, delay::Delay, prelude::*};

// 1,2,4,8,16,32,64,128 or 256 bytes are allowed buffer sizes
const TWI_RX_BUFFER_SIZE: usize = 16;
const TWI_RX_BUFFER_MASK: usize = TWI_RX_BUFFER_SIZE - 1;

const TWI_TX_BUFFER_SIZE: usize = 16;
const TWI_TX_BUFFER_MASK: usize = TWI_TX_BUFFER_SIZE - 1;

const TWI_BUFFER_SIZE: usize = TWI_RX_BUFFER_SIZE + TWI_TX_BUFFER_SIZE;

const USI_SLAVE_CHECK_ADDRESS: u16 = 0x00;
const USI_SLAVE_SEND_DATA: u16 = 0x01;
const USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA: u16 = 0x02;
const USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA: u16 = 0x03;
const USI_SLAVE_REQUEST_DATA: u16 = 0x04;
const USI_SLAVE_GET_DATA_AND_SEND_ACK: u16 = 0x05;

const TWI_READ_BIT: u8 = 0; // Bit position for R/W bit in "address byte".
const TWI_ADR_BITS: u8 = 1; // Bit position for LSB of the slave address bits in the init byte.
const TWI_NACK_BIT: u8 = 0; // Bit position for (N)ACK bit.

#[derive(thiserror::Error, Debug)]
enum UsiError {
    #[error("Generated Stop Condition not detected on bus")]
    UsiTwiMissingStopCon,
    #[error("Generated Start Condition not detected on bus")]
    UsiTwiMissingStartCon,
    #[error("The slave did not acknowledge the address")]
    UsiTwiNoAckOnAddress,
    #[error("The slave did not acknowledge all data")]
    UsiTwiNoAckOnData,
}

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

enum SlavePollEvent {
    None,

    /// Emitted when the I2C controller addresses us in read mode.
    StartRead,

    /// Emitted when the I2C controller addresses us in write mode.
    StartWrite,
}

enum SlaveWriteResult {
    Stop,
    Continue,
}

enum SlaveReadResult {
    Stop(u8),
    Continue(u8),
}

pub struct TwoWire {
    address: Option<u8>,
    fast_mode: bool,
    delay: Delay<MHz1>,
    pinb: hal::pac::portb::PINB,
    usi: hal::pac::USI,
    sda: IoPin<hal::port::PB0>,
    scl: IoPin<hal::port::PB2>,
}

impl TwoWire {
    fn delay_t2twi(&mut self) {
        let time = if self.fast_mode { 2u8 } else { 5u8 };
        self.delay.delay_us(time);
    }

    fn delay_t4twi(&mut self) {
        let time = if self.fast_mode { 1u8 } else { 4u8 };
        self.delay.delay_us(4u8);
    }

    fn usi_twi_master_initialize(&mut self) {
        let usi = &self.usi;
        self.scl.as_output_high();
        self.sda.as_output_high();
        usi.usidr.write(|w| w.bits(0xFF));
        // i2c uses two_wire_slave
        usi.usicr.write(|w| {
            w.usiwm()
                .two_wire_master()
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
                .two_wire_master()
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
        while self.pinb.read().pb2().bit_is_clear() {}
    }

    fn usi_overflow_not_detected(&self) -> bool {
        self.usi.usisr.read().usioif().bit_is_clear()
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
        self.sda.as_output();
        data
    }

    fn usi_twi_master_stop(&mut self) -> Result<(), UsiError> {
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

    fn usi_twi_start_transceiver_with_data_stop(
        &mut self,
        msg: &[u8],
        msg_size: usize,
        send_stop: bool,
    ) -> Result<[u8; TWI_BUFFER_SIZE], UsiError> {
        let mut input_msg = [0u8; TWI_BUFFER_SIZE];

        let mut address_mode = true;
        let master_write_data_mode = msg[0] & 0b1 == 0;
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

        // Write address and Read/Write data
        for index in 0..msg_size {
            if address_mode || master_write_data_mode {
                self.scl.as_output();
                self.usi.usidr.write(|w| w.bits(msg[index]));
                self.usi_twi_master_transfer(8);
                self.sda.as_pull_up_input();
                if self.usi_twi_master_transfer(1) & 0b1 != 0 {
                    return Err(match address_mode {
                        true => UsiError::UsiTwiNoAckOnAddress,
                        false => UsiError::UsiTwiNoAckOnData,
                    });
                }
                address_mode = false;
            } else {
                self.sda.as_pull_up_input();
                input_msg[index - 1] = self.usi_twi_master_transfer(8);
                let last = index == msg_size - 1;
                self.usi
                    .usidr
                    .write(|w| w.bits(if last { 0xFF } else { 0x00 }));
                self.usi_twi_master_transfer(1);
            }
        }

        if send_stop {
            self.usi_twi_master_stop()?;
        }

        Ok(input_msg)
    }

    fn usi_twi_slave_initialize(&mut self) {
        self.scl.as_pull_up_input();
        self.sda.as_pull_up_input();
        self.usi.usicr.write(|w| {
            w.usisie()
                .set_bit()
                .usioie()
                .set_bit()
                .usiwm()
                .two_wire_slave()
                .usics()
                .ext_pos()
        });
        // Clear all flags and reset overflow counter
        // TODO replace by safe methods
        self.usi.usisr.write(|w| unsafe { w.bits(0xF0) });
    }

    fn slave_end(&mut self) {
        self.scl.as_pull_up_input();
        self.sda.as_pull_up_input();
        self.usi.usicr.write(|w| w);
        self.usi.usisr.write(|w| unsafe { w.bits(0xF0) })
    }

    pub fn begin(&mut self, address: Option<u8>) {
        self.address = address;
        match address {
            Some(v) => self.usi_twi_slave_initialize(),
            None => self.usi_twi_master_initialize(),
        };
    }

    // TODO replace quantity and TWI_BUFFER_SIZE by generic param const N: usize
    pub fn request_from(
        &mut self,
        address: u8,
        quantity: usize,
        send_stop: Option<bool>,
    ) -> Result<[u8; TWI_BUFFER_SIZE], UsiError> {
        let out_msg = [(address << TWI_ADR_BITS) | (0b1)];
        let in_msg = self.usi_twi_start_transceiver_with_data_stop(
            &out_msg,
            quantity,
            send_stop.unwrap_or(true),
        )?;
        Ok(in_msg)
    }

    /// put 0 on first byte of msg
    pub fn master_write(
        &mut self,
        address: u8,
        msg: &mut [u8],
        send_stop: Option<bool>,
    ) -> Result<(), UsiError> {
        msg[0] = address << TWI_ADR_BITS;
        self.usi_twi_start_transceiver_with_data_stop(msg, msg.len(), send_stop.unwrap_or(true))?;
        Ok(())
    }

    // TODO change to basic polling (or maybe async)
    pub fn slave_poll(&mut self) -> SlavePollEvent {
        while self.start_condition_not_detected() {}
        self.usi_twi_slave_initialize();
        // check address
        while self.usi_overflow_not_detected() {}
        let data = self.usi.usidr.read().bits();
        if self.address.unwrap() == data >> 1 {
            self.set_usi_to_send_ack();
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

    pub fn slave_write(&mut self, data: u8) -> SlaveWriteResult {
        self.usi.usidr.write(|w| w.bits(data));
        self.set_usi_to_send_data();
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
        while self.usi_overflow_not_detected() {}
        let data = self.usi.usidr.read().bits();
        self.set_usi_to_send_ack();
        let usisr = self.usi.usisr.read();
        while usisr.usisif().bit_is_clear()
            || usisr.usipf().bit_is_clear()
            || usisr.usioif().bit_is_clear()
        {}
        if usisr.usipf().bit_is_set() {
            SlaveReadResult::Stop(data)
        } else {
            SlaveReadResult::Continue(data)
        }
    }

    fn set_usi_to_send_ack(&mut self) {
        self.usi.usidr.write(|w| w.bits(0));
        self.sda.as_output();
        self.usi
            .usisr
            .write(|w| w.usioif().set_bit().usipf().set_bit().usicnt().bits(0x0E));
    }

    fn set_usi_to_read_ack(&mut self) {
        self.sda.as_pull_up_input();
        self.usi.usidr.write(|w| w.bits(0));
        self.usi
            .usisr
            .write(|w| w.usioif().set_bit().usipf().set_bit().usicnt().bits(0x0E));
    }

    fn set_usi_to_twi_start_condition_mode(&mut self) {
        self.sda.as_pull_up_input();
        self.usi.usicr.write(|w| {
            w.usisie()
                .set_bit()
                .usioie()
                .set_bit()
                .usiwm()
                .two_wire_slave()
                .usics()
                .ext_pos()
        });
        self.usi
            .usisr
            .write(|w| w.usioif().set_bit().usipf().set_bit().usicnt().bits(0));
    }

    fn set_usi_to_send_data(&mut self) {
        self.sda.as_output();
        self.usi
            .usisr
            .write(|w| w.usioif().set_bit().usipf().set_bit().usicnt().bits(0));
    }

    fn set_usi_to_read_data(&mut self) {
        self.sda.as_pull_up_input();
        self.usi
            .usisr
            .write(|w| w.usioif().set_bit().usipf().set_bit().usicnt().bits(0));
    }
}
