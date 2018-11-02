use libmodbus_rs::prelude as mb;

error_chain! {
    foreign_links {
        ModbusError(mb::Error);
    }
}
