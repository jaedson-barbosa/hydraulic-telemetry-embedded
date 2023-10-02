#![no_std]

use desse::{DesseSized, Desse};

#[derive(Debug, PartialEq, Desse, desse::DesseSized)]
pub struct AttinyResponse {
    pub charger_en: bool,
}

#[derive(Debug, PartialEq, Desse, desse::DesseSized)]
pub enum AttinyRequest {
    UpdateChargerEn(bool),
    // /// send last sent value to attiny to reset value
    // ResetPulses(u16)
}

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
