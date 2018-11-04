/***
Interface with the Prostar MPPT (all models) as documented at
http://support.morningstarcorp.com/wp-content/uploads/2015/12/PSMPPT_public-MODBUS-doc_v04.pdf
*/
use chrono::prelude::*;
use libmodbus_rs::{Modbus, ModbusRTU, ModbusClient};
use half::f16;
use uom::si::{
    Unit,
    f32::*,
    thermodynamic_temperature::degree_celsius,
    electric_potential::volt,
    electric_current::ampere,
    electric_charge::ampere_hour,
    electrical_resistance::ohm,
    power::watt,
    energy::kilowatt_hour,
    time::{hour, second, day}
};
use std::fmt;
use error::*;

fn gu32(h: u16, l: u16) -> u32 { (h as u32) << 16 | (l as u32) }
fn gf32(u: u16) -> f32 { f16::from_bits(u).to_f32() }
fn v(u: f32) -> ElectricPotential { ElectricPotential::new::<volt>(u) }
fn a(u: f32) -> ElectricCurrent { ElectricCurrent::new::<ampere>(u) }
fn ah(u: f32) -> ElectricCharge { ElectricCharge::new::<ampere_hour>(u) }
fn c(u: f32) -> ThermodynamicTemperature { ThermodynamicTemperature::new::<degree_celsius>(u) }
fn w(u: f32) -> Power { Power::new::<watt>(u) }
fn om(u: f32) -> ElectricalResistance { ElectricalResistance::new::<ohm>(u) }
fn kwh(u: f32) -> Energy { Energy::new::<kilowatt_hour>(u) }
fn hr(u: f32) -> Time { Time::new::<hour>(u) }
fn sec(u: f32) -> Time { Time::new::<second>(u) }
fn dy(u: f32) -> Time { Time::new::<day>(u) }

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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

impl Default for ChargeState {
    fn default() -> Self { ChargeState::UnknownState(0) }
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
    #[derive(Default, Serialize, Deserialize)]
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
    #[derive(Default, Serialize, Deserialize)]
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
    #[derive(Default, Serialize, Deserialize)]
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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

impl Default for LoadState {
    fn default() -> Self { LoadState::Unknown(0) }
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

/** Charge controller statistics */
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Stats {
    pub timestamp: DateTime<Local>,
    pub software_version: u16,
    pub battery_voltage_settings_multiplier: u16,
    pub supply_3v3: ElectricPotential,
    pub supply_12v: ElectricPotential,
    pub supply_5v: ElectricPotential,
    pub gate_drive_voltage: ElectricPotential,
    pub battery_terminal_voltage: ElectricPotential,
    pub array_voltage: ElectricPotential,
    pub load_voltage: ElectricPotential,
    pub charge_current: ElectricCurrent,
    pub array_current: ElectricCurrent,
    pub load_current: ElectricCurrent,
    pub battery_current_net: ElectricCurrent,
    pub battery_sense_voltage: ElectricPotential,
    pub meterbus_voltage: ElectricPotential,
    pub heatsink_temperature: ThermodynamicTemperature,
    pub battery_temperature: ThermodynamicTemperature,
    pub ambient_temperature: ThermodynamicTemperature,
    pub rts_temperature: Option<ThermodynamicTemperature>,
    pub u_inductor_temperature: ThermodynamicTemperature,
    pub v_inductor_temperature: ThermodynamicTemperature,
    pub w_inductor_temperature: ThermodynamicTemperature,
    pub charge_state: ChargeState,
    pub array_faults: ArrayFaults,
    pub battery_voltage_slow: ElectricPotential,
    pub target_voltage: ElectricPotential,
    pub ah_charge_resettable: ElectricCharge,
    pub ah_charge_total: ElectricCharge,
    pub kwh_charge_resettable: Energy,
    pub kwh_charge_total: Energy,
    pub load_state: LoadState,
    pub load_faults: LoadFaults,
    pub lvd_setpoint: ElectricPotential,
    pub ah_load_resettable: ElectricCharge,
    pub ah_load_total: ElectricCharge,
    pub hourmeter: Time,
    pub alarms: Alarms,
    pub array_power: Power,
    pub array_vmp: ElectricPotential,
    pub array_max_power_sweep: ElectricPotential,
    pub array_voc: ElectricPotential,
    pub battery_v_min_daily: ElectricPotential,
    pub battery_v_max_daily: ElectricPotential,
    pub ah_charge_daily: ElectricCharge,
    pub ah_load_daily: ElectricCharge,
    pub array_faults_daily: ArrayFaults,
    pub load_faults_daily: LoadFaults,
    pub alarms_daily: Alarms,
    pub array_voltage_max_daily: ElectricPotential,
    pub array_voltage_fixed: ElectricPotential,
    pub array_voc_percent_fixed: f32,
}

macro_rules! as_unit {
    ($f:ident, $obj:ident, $field:ident, $unit:ident) => {
        write!(
            $f, "    {}: {:.2} {},\n",
            stringify!($field),
            $obj.$field.get::<$unit>(),
            $unit::abbreviation()
        )
    }
}

impl fmt::Display for Stats {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Stats {{\n")?;
        write!(f, "    timestamp: {},\n", self.timestamp)?;
        write!(f, "    software_version: {},\n", self.software_version)?;
        write!(f, "    battery_voltage_settings_multiplier: {},\n",
               self.battery_voltage_settings_multiplier)?;
        as_unit!(f, self, supply_3v3, volt)?;
        as_unit!(f, self, supply_12v, volt)?;
        as_unit!(f, self, supply_5v, volt)?;
        as_unit!(f, self, gate_drive_voltage, volt)?;
        as_unit!(f, self, battery_terminal_voltage, volt)?;
        as_unit!(f, self, array_voltage, volt)?;
        as_unit!(f, self, load_voltage, volt)?;
        as_unit!(f, self, charge_current, ampere)?;
        as_unit!(f, self, array_current, ampere)?;
        as_unit!(f, self, load_current, ampere)?;
        as_unit!(f, self, battery_current_net, ampere)?;
        as_unit!(f, self, battery_sense_voltage, volt)?;
        as_unit!(f, self, meterbus_voltage, volt)?;
        as_unit!(f, self, heatsink_temperature, degree_celsius)?;
        as_unit!(f, self, battery_temperature, degree_celsius)?;
        as_unit!(f, self, ambient_temperature, degree_celsius)?;
        match self.rts_temperature {
            None => write!(f, "    rts_temperature: None,\n")?,
            Some(t) => write!(
                f, "    rts_temperature: {} {:.2},\n", t.get::<degree_celsius>(),
                degree_celsius::abbreviation()
            )?
        }
        as_unit!(f, self, u_inductor_temperature, degree_celsius)?;
        as_unit!(f, self, v_inductor_temperature, degree_celsius)?;
        as_unit!(f, self, w_inductor_temperature, degree_celsius)?;
        write!(f, "    charge_state: {:#?},\n", self.charge_state)?;
        write!(f, "    array_faults: {:#?},\n", self.array_faults)?;
        as_unit!(f, self, battery_voltage_slow, volt)?;
        as_unit!(f, self, target_voltage, volt)?;
        as_unit!(f, self, ah_charge_resettable, ampere_hour)?;
        as_unit!(f, self, ah_charge_total, ampere_hour)?;
        as_unit!(f, self, kwh_charge_resettable, kilowatt_hour)?;
        as_unit!(f, self, kwh_charge_total, kilowatt_hour)?;
        write!(f, "    load_state: {:#?},\n", self.load_state)?;
        write!(f, "    load_faults: {:#?},\n", self.load_faults)?;
        as_unit!(f, self, lvd_setpoint, volt)?;
        as_unit!(f, self, ah_load_resettable, ampere_hour)?;
        as_unit!(f, self, ah_load_total, ampere_hour)?;
        as_unit!(f, self, hourmeter, hour)?;
        write!(f, "    alarms: {:#?},\n", self.alarms)?;
        as_unit!(f, self, array_power, watt)?;
        as_unit!(f, self, array_vmp, volt)?;
        as_unit!(f, self, array_max_power_sweep, volt)?;
        as_unit!(f, self, array_voc, volt)?;
        as_unit!(f, self, battery_v_min_daily, volt)?;
        as_unit!(f, self, battery_v_max_daily, volt)?;
        as_unit!(f, self, ah_charge_daily, ampere_hour)?;
        as_unit!(f, self, ah_load_daily, ampere_hour)?;
        write!(f, "    array_faults_daily: {:#?},\n", self.array_faults_daily)?;
        write!(f, "    load_faults_daily: {:#?},\n", self.load_faults_daily)?;
        write!(f, "    alarms_daily: {:#?},\n", self.alarms_daily)?;
        as_unit!(f, self, array_voltage_max_daily, volt)?;
        as_unit!(f, self, array_voltage_fixed, volt)?;
        write!(f, "    array_voc_percent_fixed: {:.2},\n", self.array_voc_percent_fixed)?;
        write!(f, "}}")?;
        Ok(())
    }
}

/** Device configuration settings */
#[derive(Default, Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Settings {
    pub regulation_voltage: ElectricPotential,
    pub float_voltage: ElectricPotential,
    pub time_before_float: Time,
    pub time_before_float_low_battery: Time,
    pub float_low_battery_voltage_trigger: ElectricPotential,
    pub float_cancel_voltage: ElectricPotential,
    pub exit_float_time: Time,
    pub equalize_voltage: ElectricPotential,
    pub days_between_equalize_cycles: Time,
    pub equalize_time_limit_above_regulation_voltage: Time,
    pub equalize_time_limit_at_regulation_voltage: Time,
    pub alarm_on_setting_change: bool,
    pub reference_charge_voltage_limit: ElectricPotential,
    pub battery_charge_current_limit: ElectricCurrent,
    pub temperature_compensation_coefficent: ElectricPotential,
    pub high_voltage_disconnect: ElectricPotential,
    pub high_voltage_reconnect: ElectricPotential,
    pub maximum_charge_voltage_reference: ElectricPotential,
    pub max_battery_temp_compensation_limit: ThermodynamicTemperature,
    pub min_battery_temp_compensation_limit: ThermodynamicTemperature,
    pub load_low_voltage_disconnect: ElectricPotential,
    pub load_low_voltage_reconnect: ElectricPotential,
    pub load_high_voltage_disconnect: ElectricPotential,
    pub load_high_voltage_reconnect: ElectricPotential,
    pub lvd_load_current_compensation: ElectricalResistance,
    pub lvd_warning_timeout: Time,
    pub led_green_to_green_and_yellow_limit: ElectricPotential,
    pub led_green_and_yellow_to_yellow_limit: ElectricPotential,
    pub led_yellow_to_yellow_and_red_limit: ElectricPotential,
    pub led_yellow_and_red_to_red_flashing_limit: ElectricPotential,
    pub modbus_id: u8,
    pub meterbus_id: u8,
    pub mppt_fixed_vmp: ElectricPotential,
    pub mppt_fixed_vmp_percent: f32,
    pub charge_current_limit: ElectricCurrent
}

macro_rules! validate {
    ($o:ident, $field:ident, $unit:ident, $min:expr, $max:expr) => {
        if $o.$field < $unit($min) || $o.$field > $unit($max) {
            bail!("{} {} <= x <= {}", stringify!($field), $min, $max)
        }
    }
}

impl Settings {
    pub fn validate(&self) -> Result<()> {
        validate!(self, regulation_voltage, v, 0., 17.5);
        validate!(self, float_voltage, v, 0., 17.5);
        validate!(self, time_before_float, sec, 0., 65535.);
        validate!(self, time_before_float_low_battery, sec, 0., 65535.);
        validate!(self, float_low_battery_voltage_trigger, v, 0., 17.5);
        validate!(self, float_cancel_voltage, v, 0., 17.5);
        validate!(self, exit_float_time, sec, 0., 65535.);
        validate!(self, equalize_voltage, v, 0., 17.5);
        validate!(self, days_between_equalize_cycles, dy, 0., 255.);
        validate!(self, equalize_time_limit_above_regulation_voltage, sec, 0., 65535.);
        validate!(self, equalize_time_limit_at_regulation_voltage, sec, 0., 65535.);
        validate!(self, reference_charge_voltage_limit, v, 0., 17.5);
        validate!(self, battery_charge_current_limit, a, 0., 40.);
        validate!(self, temperature_compensation_coefficent, v, 0., 17.5);
        validate!(self, high_voltage_disconnect, v, 0., 17.5);
        validate!(self, high_voltage_reconnect, v, 0., 17.5);
        validate!(self, maximum_charge_voltage_reference, v, 0., 17.5);
        validate!(self, max_battery_temp_compensation_limit, c, -128., 127.);
        validate!(self, min_battery_temp_compensation_limit, c, -128., 127.);
        validate!(self, load_low_voltage_disconnect, v, 0., 17.5);
        validate!(self, load_low_voltage_reconnect, v, 0., 17.5);
        validate!(self, load_high_voltage_disconnect, v, 0., 17.5);
        validate!(self, load_high_voltage_reconnect, v, 0., 17.5);
        validate!(self, lvd_load_current_compensation, om, 0., 10000.);
        validate!(self, lvd_warning_timeout, sec, 0., 65535.);
        validate!(self, led_green_to_green_and_yellow_limit, v, 0., 17.5);
        validate!(self, led_green_and_yellow_to_yellow_limit, v, 0., 17.5);
        validate!(self, led_yellow_to_yellow_and_red_limit, v, 0., 17.5);
        validate!(self, led_yellow_and_red_to_red_flashing_limit, v, 0., 17.5);
        if self.modbus_id < 1 || self.modbus_id > 247 {
            bail!("modbus_id 1 <= x <= 247");
        }
        if self.meterbus_id < 1 || self.meterbus_id > 15 {
            bail!("meterbus_id 1 <= x <= 15");
        }
        validate!(self, mppt_fixed_vmp, v, 0., 120.);
        if self.mppt_fixed_vmp_percent < 0. || self.mppt_fixed_vmp_percent > 1. {
            bail!("mppt_fixed_vmp_percent 0 <= x <= 1")
        }
        validate!(self, charge_current_limit, a, 0., 40.);
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Coil {
    EqualizeTriggered,
    LoadDisconnect,
    ChargeDisconnect,
    ClearAhResettable,
    ClearAhTotal,
    ClearKwhResettable,
    ClearFaults,
    ClearAlarms,
    ForceEEPROMUpdate,
    ClearKwhTotal,
    ClearVbMinMax,
    LightingModeTest,
    FactoryReset,
    ResetControl
}

impl Coil {
    fn address(&self) -> u16 {
        match self {
            Coil::EqualizeTriggered  => 0x0000,
            Coil::LoadDisconnect     => 0x0001,
            Coil::ChargeDisconnect   => 0x0002,
            Coil::ClearAhResettable  => 0x0010,
            Coil::ClearAhTotal       => 0x0011,
            Coil::ClearKwhResettable => 0x0012,
            Coil::ClearFaults        => 0x0014,
            Coil::ClearAlarms        => 0x0015,
            Coil::ForceEEPROMUpdate  => 0x0016,
            Coil::ClearKwhTotal      => 0x0018,
            Coil::ClearVbMinMax      => 0x0019,
            Coil::LightingModeTest   => 0x0020,
            Coil::FactoryReset       => 0x00FE,
            Coil::ResetControl       => 0x00FF
        }
    }
}

/** Device connection. */
pub struct Connection(Modbus);

impl Connection {
    pub fn new(device: &str, modbus_id: u8) -> Result<Connection> {
        let mut con =
            Modbus::new_rtu(device, 9600, 'N', 8, 2).map_err(Error::from)
            .chain_err(|| "failed to create a new rtu object")?;
        con.set_slave(modbus_id).map_err(Error::from)
            .chain_err(|| "failed to set modbus id")?;
        con.connect().map_err(Error::from)
            .chain_err(|| "failed to connect to device")?;
        Ok(Connection(con))
    }

    pub fn read_coil(&self, coil: Coil) -> Result<bool> {
        let mut raw = [0u8; 1];
        self.0.read_bits(coil.address(), 1, &mut raw)?;
        Ok(if raw[0] == 0 { false } else { true })
    }

    pub fn write_coil(&self, coil: Coil, val: bool) -> Result<()> {
        Ok(self.0.write_bit(coil.address(), val)?)
    }

    pub fn stats(&self) -> Result<Stats> {
        let mut raw = [0u16; 81];
        self.0.read_registers(0x0, 80, &mut raw)?;
        Ok(Stats {
            timestamp: Local::now(),
            software_version: raw[0x0000],
            battery_voltage_settings_multiplier: raw[0x0001],
            supply_3v3: v(gf32(raw[0x0004])),
            supply_12v: v(gf32(raw[0x0005])),
            supply_5v: v(gf32(raw[0x0006])),
            gate_drive_voltage: v(gf32(raw[0x0007])),
            battery_terminal_voltage: v(gf32(raw[0x0012])),
            array_voltage: v(gf32(raw[0x0013])),
            load_voltage: v(gf32(raw[0x0014])),
            charge_current: a(gf32(raw[0x0010])),
            array_current: a(gf32(raw[0x0011])),
            load_current: a(gf32(raw[0x0016])),
            battery_current_net: a(gf32(raw[0x0015])),
            battery_sense_voltage: v(gf32(raw[0x0017])),
            meterbus_voltage: v(gf32(raw[0x0008])),
            heatsink_temperature: c(gf32(raw[0x001A])),
            battery_temperature: c(gf32(raw[0x001B])),
            ambient_temperature: c(gf32(raw[0x001C])),
            rts_temperature: {
                let t = gf32(raw[0x001D]);
                if t.is_nan() { None } else { Some(c(t)) }
            },
            u_inductor_temperature: c(gf32(raw[0x001E])),
            v_inductor_temperature: c(gf32(raw[0x001F])),
            w_inductor_temperature: c(gf32(raw[0x0020])),
            charge_state: ChargeState::from(raw[0x0021]),
            array_faults: ArrayFaults::from_bits_truncate(raw[0x0022]),
            battery_voltage_slow: v(gf32(raw[0x0023])),
            target_voltage: v(gf32(raw[0x0024])),
            ah_charge_resettable: ah(gu32(raw[0x0026], raw[0x0027]) as f32 * 0.1),
            ah_charge_total: ah(gu32(raw[0x0028], raw[0x0029]) as f32 * 0.1),
            kwh_charge_resettable: kwh(gf32(raw[0x002A]) * 0.1),
            kwh_charge_total: kwh(gf32(raw[0x002B]) * 0.1),
            load_state: LoadState::from(raw[0x002E]),
            load_faults: LoadFaults::from_bits_truncate(raw[0x002F]),
            lvd_setpoint: v(gf32(raw[0x0030])),
            ah_load_resettable: ah(gu32(raw[0x0032], raw[0x0033]) as f32 * 0.1),
            ah_load_total: ah(gu32(raw[0x0034], raw[0x0035]) as f32 * 0.1),
            hourmeter: hr(gu32(raw[0x0036], raw[0x0037]) as f32),
            alarms: Alarms::from_bits_truncate((raw[0x0038] as u32) << 16 | raw[0x0039] as u32),
            array_power: w(gf32(raw[0x003C])),
            array_vmp: v(gf32(raw[0x003D])),
            array_max_power_sweep: v(gf32(raw[0x003E])),
            array_voc: v(gf32(raw[0x003F])),
            battery_v_min_daily: v(gf32(raw[0x0041])),
            battery_v_max_daily: v(gf32(raw[0x0042])),
            ah_charge_daily: ah(gf32(raw[0x0043])),
            ah_load_daily: ah(gf32(raw[0x0044])),
            array_faults_daily: ArrayFaults::from_bits_truncate(raw[0x0045]),
            load_faults_daily: LoadFaults::from_bits_truncate(raw[0x0046]),
            alarms_daily: Alarms::from_bits_truncate((raw[0x0047] as u32) << 16 | raw[0x0048] as u32),
            array_voltage_max_daily: v(gf32(raw[0x004C])),
            array_voltage_fixed: v(gf32(raw[0x004F])),
            array_voc_percent_fixed: gf32(raw[0x0050])
        })
    }

    pub fn read_settings(&self) -> Result<Settings> {
        const BASE: usize = 0xE000;
        let mut raw = [0u16; 0xE039 - BASE];
        self.0.read_registers(BASE as u16, 0xE038 - BASE as u16, &mut raw)?;
        Ok(Settings {
            regulation_voltage: v(gf32(raw[0xE000 - BASE])),
            float_voltage: v(gf32(raw[0xE001 - BASE])),
            time_before_float: sec(gf32(raw[0xE002 - BASE])),
            time_before_float_low_battery: sec(gf32(raw[0xE003 - BASE])),
            float_low_battery_voltage_trigger: v(gf32(raw[0xE004 - BASE])),
            float_cancel_voltage: v(gf32(raw[0xE005 - BASE])),
            exit_float_time: sec(gf32(raw[0xE006 - BASE])),
            equalize_voltage: v(gf32(raw[0xE007 - BASE])),
            days_between_equalize_cycles: dy(gf32(raw[0xE008 - BASE])),
            equalize_time_limit_above_regulation_voltage: sec(gf32(raw[0xE009 - BASE])),
            equalize_time_limit_at_regulation_voltage: sec(gf32(raw[0xE00A - BASE])),
            alarm_on_setting_change: raw[0xE00D - BASE] == 1,
            reference_charge_voltage_limit: v(gf32(raw[0xE010 - BASE])),
            battery_charge_current_limit: a(gf32(raw[0xE013 - BASE])),
            temperature_compensation_coefficent: v(gf32(raw[0xE01A - BASE])),
            high_voltage_disconnect: v(gf32(raw[0xE01B - BASE])),
            high_voltage_reconnect: v(gf32(raw[0xE01C - BASE])),
            maximum_charge_voltage_reference: v(gf32(raw[0xE01D - BASE])),
            max_battery_temp_compensation_limit: c(gf32(raw[0xE01E - BASE])),
            min_battery_temp_compensation_limit: c(gf32(raw[0xE01F - BASE])),
            load_low_voltage_disconnect: v(gf32(raw[0xE022 - BASE])),
            load_low_voltage_reconnect: v(gf32(raw[0xE023 - BASE])),
            load_high_voltage_disconnect: v(gf32(raw[0xE024 - BASE])),
            load_high_voltage_reconnect: v(gf32(raw[0xE025 - BASE])),
            lvd_load_current_compensation: om(gf32(raw[0xE026 - BASE])),
            lvd_warning_timeout: sec(gf32(raw[0xE027 - BASE])),
            led_green_to_green_and_yellow_limit: v(gf32(raw[0xE030 - BASE])),
            led_green_and_yellow_to_yellow_limit: v(gf32(raw[0xE031 - BASE])),
            led_yellow_to_yellow_and_red_limit: v(gf32(raw[0xE032 - BASE])),
            led_yellow_and_red_to_red_flashing_limit: v(gf32(raw[0xE033 - BASE])),
            modbus_id: raw[0xE034 - BASE] as u8,
            meterbus_id: raw[0xE035 - BASE] as u8,
            mppt_fixed_vmp: v(gf32(raw[0xE036 - BASE])),
            mppt_fixed_vmp_percent: gf32(raw[0xE037 - BASE]),
            charge_current_limit: a(gf32(raw[0xE038 - BASE]))
        })
    }
}
