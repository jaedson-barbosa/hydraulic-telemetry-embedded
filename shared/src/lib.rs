#![no_std]

use desse::{DesseSized, Desse};

#[derive(Debug, PartialEq, Desse, desse::DesseSized)]
pub struct AttinyResponse {
    pub n_pulses: u8,
    pub generator_mv: u16
}

#[derive(Debug, PartialEq, Desse, desse::DesseSized)]
pub enum AttinyRequest {
    None
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
