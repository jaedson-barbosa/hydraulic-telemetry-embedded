#![no_std]

use desse::{DesseSized, Desse};

#[derive(Debug, PartialEq, Desse, desse::DesseSized)]
pub struct CommTest {
    pub a: u8,
    pub b: i16
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
