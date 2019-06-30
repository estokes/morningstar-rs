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
/**
Interface with the Prostar MPPT (all models) as documented at
http://support.morningstarcorp.com/wp-content/uploads/2015/12/PSMPPT_public-MODBUS-doc_v04.pdf

# Examples
```
use morningstar::{error as mse, prostar_mppt as ps};
use std::{thread::sleep, time::{Instant, Duration}};

let con = ps::Connection::new("/dev/ttyUSB0", 1).expect("connection failed");
println!("{}", con.stats().expect("failed to get stats"));
// It doesn't like getting commands too quickly
sleep(Duration::from_millis(500));

// Stop charging the battery
con.write_coil(ps::Coil::ChargeDisconnect, true).expect("failed to stop charging");
sleep(Duration::from_millis(500));

// Start Charging again
con.write_coil(ps::Coil::ChargeDisconnect, false).expect("failed to start charging");
```
*/
pub mod prostar_mppt;
