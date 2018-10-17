use libmodbus_rs::{prelude::Error, Modbus, ModbusRTU, ModbusClient};
use half::f16;

#[derive(Debug, Clone, Copy)]
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
            0u16 => ChargeState::Start,
            1u16 => ChargeState::NightCheck,
            2u16 => ChargeState::Disconnect,
            3u16 => ChargeState::Night,
            4u16 => ChargeState::Fault,
            5u16 => ChargeState::BulkMPPT,
            6u16 => ChargeState::Absorption,
            7u16 => ChargeState::Float,
            8u16 => ChargeState::Equalize,
            9u16 => ChargeState::Slave,
            10u16 => ChargeState::Fixed,
            i => ChargeState::UnknownState(i)
        }
    }
}

bitflags! {
    #[derive(Default)]
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
}

bitflags! {
    #[derive(Default)]
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
}
    
bitflags! {
    #[derive(Default)]
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

#[derive(Debug, Clone, Copy)]
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
            0u16 => LoadState::Start,
            1u16 => LoadState::Normal,
            2u16 => LoadState::LVDWarning,
            3u16 => LoadState::LVD,
            4u16 => LoadState::Fault,
            5u16 => LoadState::Disconnect,
            6u16 => LoadState::NormalOff,
            7u16 => LoadState::Override,
            8u16 => LoadState::NotUsed,
            i => LoadState::Unknown(i)
        }
    }
}

#[derive(Debug, Clone, Copy)]
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
    ah_charge_resettable: u32,
    ah_charge_total: u32,
    kwh_charge_resettable: f32,
    kwh_charge_total: f32,
    load_state: LoadState,
    load_faults: LoadFaults,
    lvd_setpoint: f32,
    ah_load_resettable: u32,
    ah_load_total: u32,
    hourmeter: u32,
    alarms: Alarms,
    array_power: f32,
    array_vmp: f32,
    array_max_power_sweep: f32,
    array_voc: f32,
    battery_v_min_daily: f32,
    battery_v_max_daily: f32,
    ah_charge_daily: f32,
    ah_load_daily: f32,
    array_faults_daily: ArrayFaults,
    load_faults_daily: LoadFaults,
    alarms_daily: Alarms,
    array_voltage_max_daily: f32,
    array_voltage_fixed: f32,
    array_voc_percent_fixed: f32
}

pub struct Con(Modbus);

impl Con {
    pub fn connect(device: &str, slave: u8) -> Result<Con, Error> {
        let mut con = Modbus::new_rtu(device, 9600, 'N', 8, 2)?;
        con.set_slave(slave)?;
        con.connect()?;
        Ok(Con(con))
    }

    pub fn stats(&self) -> Result<Stats, Error> {
        let mut raw = [0u16; 81];
        self.0.read_registers(0x0, 80, &mut raw)?;
        fn gu32(hi: u16, low: u16) -> u32 { (hi as u32) << 16 | (low as u32) }
        fn gf32(u: u16) -> f32 { f16::from_bits(u).to_f32() }
        Ok(Stats {
            battery_terminal_voltage: gf32(raw[0x0012]),
            array_voltage: gf32(raw[0x0013]),
            load_voltage: gf32(raw[0x0014]),
            charge_current: gf32(raw[0x0010]),
            load_current: gf32(raw[0x0016]),
            battery_current_net: gf32(raw[0x0015]),
            battery_sense_voltage: gf32(raw[0x0017]),
            meterbus_voltage: gf32(raw[0x0008]),
            heatsink_temperature: gf32(raw[0x001A]),
            battery_temperature: gf32(raw[0x001B]),
            ambient_temperature: gf32(raw[0x001C]),
            rts_temperature: gf32(raw[0x001D]),
            u_inductor_temperature: gf32(raw[0x001E]),
            v_inductor_temperature: gf32(raw[0x001F]),
            w_inductor_temperature: gf32(raw[0x0020]),
            charge_state: ChargeState::from(raw[0x0021]),
            array_faults: ArrayFaults::from_bits_truncate(raw[0x0022]),
            battery_voltage_slow: gf32(raw[0x0023]),
            target_voltage: gf32(raw[0x0024]),
            ah_charge_resettable: gu32(raw[0x0026], raw[0x0027]),
            ah_charge_total: gu32(raw[0x0028], raw[0x0029]),
            kwh_charge_resettable: gf32(raw[0x002A]) * 0.1,
            kwh_charge_total: gf32(raw[0x002B]) * 0.1,
            load_state: LoadState::from(raw[0x002E]),
            load_faults: LoadFaults::from_bits_truncate(raw[0x002F]),
            lvd_setpoint: gf32(raw[0x0030]),
            ah_load_resettable: gu32(raw[0x0032], raw[0x0033]),
            ah_load_total: gu32(raw[0x0034], raw[0x0035]),
            hourmeter: (raw[0x0036] as u32) << 16 | raw[0x0037] as u32,
            alarms: Alarms::from_bits_truncate((raw[0x0038] as u32) << 16 | raw[0x0039] as u32),
            array_power: gf32(raw[0x003C]),
            array_vmp: gf32(raw[0x003D]),
            array_max_power_sweep: gf32(raw[0x003E]),
            array_voc: gf32(raw[0x003F]),
            battery_v_min_daily: gf32(raw[0x0041]),
            battery_v_max_daily: gf32(raw[0x0042]),
            ah_charge_daily: gf32(raw[0x0043]),
            ah_load_daily: gf32(raw[0x0044]),
            array_faults_daily: ArrayFaults::from_bits_truncate(raw[0x0045]),
            load_faults_daily: LoadFaults::from_bits_truncate(raw[0x0046]),
            alarms_daily: Alarms::from_bits_truncate((raw[0x0047] as u32) << 16 | raw[0x0048] as u32),
            array_voltage_max_daily: gf32(raw[0x004C]),
            array_voltage_fixed: gf32(raw[0x004F]),
            array_voc_percent_fixed: gf32(raw[0x0050])
        })
    }
}
