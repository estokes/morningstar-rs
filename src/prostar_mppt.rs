use libmodbus_rs::{prelude::Error, Modbus, ModbusRTU};

pub enum ChargeState {
    UnknownState(u16),
    Start,
    NightCheck,
    Disconnect,
    Night,
    Fault,
    BulkMPPT,
    Absorption,
    Float,
    Equalize,
    Slave,
    Fixed
}

impl From<u16> for ChargeState {
    fn from(i: u16) -> Self {
        match i {
            0u16 -> ChargeState::Start,
            1u16 -> ChargeState::NightCheck,
            2u16 -> ChargeState::Disconnect,
            3u16 -> ChargeState::Night,
            4u16 -> ChargeState::Fault,
            5u16 -> ChargeState::BulkMPPT,
            6u16 -> ChargeState::Absorption,
            7u16 -> ChargeState::Float,
            8u16 -> ChargeState::Equalize,
            9u16 -> ChargeState::Slave,
            10u16 -> ChargeState::Fixed,
            i -> ChargeState::UnknownState(i)
        }
    }
}

bitflags! {
    #[Derive(Default)]
    pub struct ArrayFaults: u16 {
        const Overcurrent                = 0x0001;
        const MOSFETShorted              = 0x0002;
        const Software                   = 0x0004;
        const BatteryHVD                 = 0x0008;
        const ArrayHVD                   = 0x0010;
        const CustomSettingsEdit         = 0x0020;
        const RTSShorted                 = 0x0040;
        const RTSNoLongerValid           = 0x0080;
        const LocalTempSensorDamaged     = 0x0100;
        const BatterLowVoltageDisconnect = 0x0200;
        const SlaveTimeout               = 0x0400;
        const DIPSwitchChanged           = 0x0800;
        const Fault13                    = 0x1000;
        const Fault14                    = 0x2000;
        const Fault15                    = 0x4000;
        const Fault16                    = 0x8000;
    }

    #[Derive(Default)]
    pub struct LoadFaults: u16 {
        const ExternalShortCircuit = 0x0001;
        const Overcurrent          = 0x0002;
        const MOSFETShorted        = 0x0004;
        const Software             = 0x0008;
        const LoadHVD              = 0x0010;
        const HighTempDisconnect   = 0x0020;
        const DipSwitchChanged     = 0x0040;
        const CustomSettingsEdit   = 0x0080;
    }

    #[Derive(Default)]
    pub struct Alarms: u32 {
        const RTSOpen                   = 0x00000001;
        const RTSShorted                = 0x00000002;
        const RTSDisconnected           = 0x00000004;
        const HeatsinkTempSensorOpen    = 0x00000008;
        const HeatsinkTempSensorShorted = 0x00000010;
        const HeatsinkTempLimit         = 0x00000020;
        const InductorTempSensorOpen    = 0x00000040;
        const InductorTempSensorShorted = 0x00000080;
        const InductorTempLimit         = 0x00000100;
        const CurrentLimit              = 0x00000200;
        const CurrentMeasurementError   = 0x00000400;
        const BatterySenseOutOfRange    = 0x00000800;
        const BatterySenseDisconnected  = 0x00001000;
        const Uncalibrated              = 0x00002000;
        const TB5V                      = 0x00004000;
        const FP10SupplyOutOfRange      = 0x00008000;
        const UNUSED                    = 0x00010000;
        const MOSFETOpen                = 0x00020000;
        const ArrayCurrentOffset        = 0x00040000;
        const LoadCurrentOffset         = 0x00080000;
        const P3v3SupplyOutOfRange      = 0x00100000;
        const P12vSupplyOutOfRange      = 0x00200000;
        const HighInputVoltageLimit     = 0x00400000;
        const ControllerReset           = 0x00800000;
        const LoadLVD                   = 0x01000000;
        const LogTimeout                = 0x02000000;
        const EEPROMAccessFailure       = 0x04000000;
    }
}

pub enum LoadState {
    Unknown(u16),
    Start,
    Normal,
    LVDWarning,
    LVD,
    Fault,
    Disconnect,
    NormalOff,
    Override,
    NotUsed
}

impl From<u16> for LoadState {
    fn from(i: u16) -> LoadState {
        match i {
            0u16 -> LoadState::Start,
            1u16 -> LoadState::Normal,
            2u16 -> LoadState::LVDWarning,
            3u16 -> LoadState::LVD,
            4u16 -> LoadState::Fault,
            5u16 -> LoadState::Disconnect,
            6u16 -> LoadState::NormalOff,
            7u16 -> LoadState::Override,
            8u16 -> LoadState::NotUsed,
            i -> LoadState::Unknown(i)
        }
    }
}

pub struct Stats {
    battery_terminal_voltage: f32,
    array_voltage: f32,
    load_voltage: f32,
    charge_current: f32,
    load_current: f32,
    battery_current_net: f32,
    battery_sense_voltage: f32,
    meterbus_voltage: f32,
    heatsink_temperature: f32,
    battery_temperature: f32,
    ambient_temperature: f32,
    rts_temperature: f32,
    u_inductor_temperature: f32,
    v_inductor_temperature: f32,
    w_inductor_temperature: f32,
    charge_state: ChargeState,
    array_faults: ArrayFaults,
    battery_voltage_slow: f32,
    target_voltage: f32,
    ah_charge_resettable: f32,
    ah_charge_total: f32,
    kwh_charge_resettable: f32,
    kwh_charge_total: f32,
    load_state: LoadState,
    load_faults: LoadFaults,
    lvd_setpoint_current_compensated: f32,
    ah_load_resettable: f32,
    ah_load_total: f32,
    hourmeter: f32,
    alarms: Alarms,
    array_power: f32,
    array_vmp: f32,
    array_max_power_sweet: f32,
    array_voc: f32,
    battery_v_min_daily: f32,
    battery_v_max_daily: f32,
    ah_charge_daily: f32,
    ah_load_daily: f32,
    array_faults_daily: ArrayFaults,
    load_faults_daily: LoadFaults,
    alarms_daily: AlarmsDaily,
    array_voltage_max_daily: f32,
    array_voltage_fixed: f32,
    array_voc_percent_fixed: f32    
}

pub struct Con(Modbus);

impl Con {
    fn connect(device: &str, slave: u8) -> Result<Con, Error> {
        let con = Modbus::new_rtu(device, 9600, 'N', 8, 2)?;
        con.set_slave(slave)?;
        con.connect()?;
        Con(con)
    }

    fn stats(&self) -> Result<Stats, Error> {
        let base = 0x0008;
        let mut raw = [u16; 72];
        self.0.read_registers(0x0008, 72, &mut raw)?;
        Stats {
            battery_terminal_voltage: raw[0x0012 - base] as f32,
            
        }
    }
}
