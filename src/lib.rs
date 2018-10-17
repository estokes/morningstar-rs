extern crate libmodbus_rs;
#[macro_use]
extern crate bitflags;

pub mod prostar_mppt;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
