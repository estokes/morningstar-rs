/// There is a sub module for each hardware family, as well as a common error type.

extern crate libmodbus_rs;
#[macro_use]
extern crate bitflags;
extern crate half;
extern crate uom;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate chrono;
#[macro_use]
extern crate error_chain;

pub mod error;
/// For the Prostar MPPT family (should support all models), tested
/// with a Prostar MPPT 40M.
pub mod prostar_mppt;
