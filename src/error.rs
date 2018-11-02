use libmodbus_rs::prelude as mb;

error_chain! {
    errors {
        ModbusError(e: mb::Error) {
            description("modbus error"),
            display("modbus error: {}", e),
        }
    }
}

impl From<mb::Error> for Error {
    fn from(e: mb::Error) -> Error { ErrorKind::ModbusError(e).into() }
}
