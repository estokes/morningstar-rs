/// There is a sub module for each hardware family, as well as a common error type.

#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate serde_derive;
#[macro_use]
extern crate anyhow;

/**
Interface with the Prostar MPPT (all models) as documented at
http://support.morningstarcorp.com/wp-content/uploads/2015/12/PSMPPT_public-MODBUS-doc_v04.pdf

# Examples
```
use morningstar::prostar_mppt as ps;
use std::{thread::sleep, time::{Instant, Duration}};

let con = ps::Connection::new("/dev/ttyUSB0", 1).await.expect("connection failed");
println!("{}", con.stats().expect("failed to get stats"));

// Stop charging the battery
con.write_coil(ps::Coil::ChargeDisconnect, true).await.expect("failed to stop charging");

// Start Charging again
con.write_coil(ps::Coil::ChargeDisconnect, false).await.expect("failed to start charging");
```
*/
pub mod prostar_mppt;
