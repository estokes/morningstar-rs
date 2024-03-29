use anyhow::{Context, Result};
/**
Interface with the Prostar MPPT (all models) as documented at
http://support.morningstarcorp.com/wp-content/uploads/2015/12/PSMPPT_public-MODBUS-doc_v04.pdf

# Examples
```
use morningstar::prostar_mppt as ps;
use std::{thread::sleep, time::{Instant, Duration}};

let con = ps::Connection::new("/dev/ttyUSB0", 1).await.expect("connection failed");
println!("{}", con.stats().await.expect("failed to get stats"));

// Stop charging the battery
con.write_coil(ps::Coil::ChargeDisconnect, true).await.expect("failed to stop charging");

// Start Charging again
con.write_coil(ps::Coil::ChargeDisconnect, false).await.expect("failed to start charging");
```
*/
use chrono::prelude::*;
use half::f16;
use std::{fmt, mem::transmute, thread::sleep, time::Duration};
use tokio_modbus::{client::Context as Modbus, prelude::*};
use tokio_serial::{self, DataBits, FlowControl, Parity, SerialStream, StopBits};
use uom::si::{
    electric_charge::ampere_hour,
    electric_current::ampere,
    electric_potential::volt,
    electrical_resistance::ohm,
    energy::kilowatt_hour,
    f32::*,
    power::watt,
    thermodynamic_temperature::degree_celsius,
    time::{day, hour, minute, second},
    Unit,
};

fn gu32(h: u16, l: u16) -> u32 {
    (h as u32) << 16 | (l as u32)
}
fn gf32(u: u16) -> f32 {
    let v = f16::from_bits(u).to_f32();
    if v.is_nan() {
        0.
    } else {
        v
    }
}
fn to_v(v: ElectricPotential) -> u16 {
    f16::from_f32(v.get::<volt>()).to_bits()
}
fn v(u: f32) -> ElectricPotential {
    ElectricPotential::new::<volt>(u)
}
fn to_a(v: ElectricCurrent) -> u16 {
    f16::from_f32(v.get::<ampere>()).to_bits()
}
fn a(u: f32) -> ElectricCurrent {
    ElectricCurrent::new::<ampere>(u)
}
fn ah(u: f32) -> ElectricCharge {
    ElectricCharge::new::<ampere_hour>(u)
}
fn to_ic(c: ThermodynamicTemperature) -> u16 {
    unsafe { transmute::<i16, u16>(c.get::<degree_celsius>() as i16) }
}
fn c(u: f32) -> ThermodynamicTemperature {
    ThermodynamicTemperature::new::<degree_celsius>(u)
}
fn ic(u: u16) -> ThermodynamicTemperature {
    ThermodynamicTemperature::new::<degree_celsius>(
        unsafe { transmute::<u16, i16>(u) } as f32
    )
}
fn w(u: f32) -> Power {
    Power::new::<watt>(u)
}
fn to_om(r: ElectricalResistance) -> u16 {
    f16::from_f32(r.get::<ohm>()).to_bits()
}
fn om(u: f32) -> ElectricalResistance {
    ElectricalResistance::new::<ohm>(u)
}
fn kwh(u: f32) -> Energy {
    Energy::new::<kilowatt_hour>(u)
}
fn hr(u: f32) -> Time {
    Time::new::<hour>(u)
}
fn to_sec(s: Time) -> u16 {
    s.get::<second>() as u16
}
fn sec(u: f32) -> Time {
    Time::new::<second>(u)
}
fn to_dy(d: Time) -> u16 {
    d.get::<day>() as u16
}
fn dy(u: f32) -> Time {
    Time::new::<day>(u)
}
fn mn(u: f32) -> Time {
    Time::new::<minute>(u)
}
fn to_mn(m: Time) -> u16 {
    m.get::<minute>() as u16
}

const SETTINGS_BASE: usize = 0xE000;
const SETTINGS_END: usize = 0xE038;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
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
    Fixed,
}

impl Default for ChargeState {
    fn default() -> Self {
        ChargeState::UnknownState(0)
    }
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
            i => ChargeState::UnknownState(i),
        }
    }
}

bitflags! {
    #[derive(Default, Serialize, Deserialize)]
    pub struct ArrayFaults: u16 {
        const OVER_CURRENT                   = 0x0001;
        const MOSFET_SHORTED                 = 0x0002;
        const SOFTWARE                       = 0x0004;
        const BATTERY_HVD                    = 0x0008;
        const ARRAY_HVD                      = 0x0010;
        const CUSTOM_SETTINGS_EDIT           = 0x0020;
        const RTS_SHORTED                    = 0x0040;
        const RTS_NO_LONGER_VALID            = 0x0080;
        const LOCAL_TEMP_SENSOR_DAMAGED      = 0x0100;
        const BATTERY_LOW_VOLTAGE_DISCONNECT = 0x0200;
        const SLAVE_TIMEOUT                  = 0x0400;
        const DIP_SWITCH_CHANGED             = 0x0800;
        const FAULT13                        = 0x1000;
        const FAULT14                        = 0x2000;
        const FAULT15                        = 0x4000;
        const FAULT16                        = 0x8000;
    }
}

bitflags! {
    #[derive(Default, Serialize, Deserialize)]
    pub struct LoadFaults: u16 {
        const EXTERNAL_SHORT_CIRCIT = 0x0001;
        const OVERCURRENT           = 0x0002;
        const MOSFET_SHORTED        = 0x0004;
        const SOFTWARE              = 0x0008;
        const LOAD_HVD              = 0x0010;
        const HIGH_TEMP_DISCONNECT  = 0x0020;
        const DIP_SWITCH_CHANGED    = 0x0040;
        const CUSTOM_SETTINGS_EDIT  = 0x0080;
    }
}

bitflags! {
    #[derive(Default, Serialize, Deserialize)]
    pub struct Alarms: u32 {
        const RTS_OPEN                     = 0x00000001;
        const RTS_SHORTED                  = 0x00000002;
        const RTS_DISCONNECTED             = 0x00000004;
        const HEATSINK_TEMP_SENSOR_OPEN    = 0x00000008;
        const HEATSINK_TEMP_SENSOR_SHORTED = 0x00000010;
        const HEATSINK_TEMP_LIMIT          = 0x00000020;
        const INDUCTOR_TEMP_SENSOR_OPEN    = 0x00000040;
        const INDUCTOR_TEMP_SENSOR_SHORTED = 0x00000080;
        const INDUCTOR_TEMP_LIMIT          = 0x00000100;
        const CURRENT_LIMIT                = 0x00000200;
        const CURRENT_MEASUREMENT_ERROR    = 0x00000400;
        const BATTERY_SENSE_OUT_OF_RANGE   = 0x00000800;
        const BATTERY_SENSE_DISCONNECTED   = 0x00001000;
        const UNCALIBRATED                 = 0x00002000;
        const TB5V                         = 0x00004000;
        const FP10SUPPLY_OUT_OF_RANGE      = 0x00008000;
        const UNUSED                       = 0x00010000;
        const MOSFET_OPEN                  = 0x00020000;
        const ARRAY_CURRENT_OFFSET         = 0x00040000;
        const LOAD_CURRENT_OFFSET          = 0x00080000;
        const P3V3_SUPPLY_OUT_OF_RANGE     = 0x00100000;
        const P12V_SUPPLY_OUT_OF_RANGE     = 0x00200000;
        const HIGH_INPUT_VOLTAGE_LIMIT     = 0x00400000;
        const CONTROLLER_RESET             = 0x00800000;
        const LOAD_LVD                     = 0x01000000;
        const LOG_TIMEOUT                  = 0x02000000;
        const EEPROM_ACCESS_FAILURE        = 0x04000000;
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub enum LoadState {
    Unknown(u16),
    Start,
    LVDWarning,
    LVD,
    Fault,
    Disconnect,
    NormalOff,
    Override,
    Normal,
    NotUsed,
}

impl Default for LoadState {
    fn default() -> Self {
        LoadState::Unknown(0)
    }
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
            i => LoadState::Unknown(i),
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
    pub array_max_power_sweep: Power,
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

impl Default for Stats {
    fn default() -> Stats {
        Stats {
            timestamp: Local::now(),
            software_version: 0,
            battery_voltage_settings_multiplier: 0,
            supply_3v3: ElectricPotential::default(),
            supply_12v: ElectricPotential::default(),
            supply_5v: ElectricPotential::default(),
            gate_drive_voltage: ElectricPotential::default(),
            battery_terminal_voltage: ElectricPotential::default(),
            array_voltage: ElectricPotential::default(),
            load_voltage: ElectricPotential::default(),
            charge_current: ElectricCurrent::default(),
            array_current: ElectricCurrent::default(),
            load_current: ElectricCurrent::default(),
            battery_current_net: ElectricCurrent::default(),
            battery_sense_voltage: ElectricPotential::default(),
            meterbus_voltage: ElectricPotential::default(),
            heatsink_temperature: ThermodynamicTemperature::default(),
            battery_temperature: ThermodynamicTemperature::default(),
            ambient_temperature: ThermodynamicTemperature::default(),
            rts_temperature: None,
            u_inductor_temperature: ThermodynamicTemperature::default(),
            v_inductor_temperature: ThermodynamicTemperature::default(),
            w_inductor_temperature: ThermodynamicTemperature::default(),
            charge_state: ChargeState::default(),
            array_faults: ArrayFaults::default(),
            battery_voltage_slow: ElectricPotential::default(),
            target_voltage: ElectricPotential::default(),
            ah_charge_resettable: ElectricCharge::default(),
            ah_charge_total: ElectricCharge::default(),
            kwh_charge_resettable: Energy::default(),
            kwh_charge_total: Energy::default(),
            load_state: LoadState::default(),
            load_faults: LoadFaults::default(),
            lvd_setpoint: ElectricPotential::default(),
            ah_load_resettable: ElectricCharge::default(),
            ah_load_total: ElectricCharge::default(),
            hourmeter: Time::default(),
            alarms: Alarms::default(),
            array_power: Power::default(),
            array_vmp: ElectricPotential::default(),
            array_max_power_sweep: Power::default(),
            array_voc: ElectricPotential::default(),
            battery_v_min_daily: ElectricPotential::default(),
            battery_v_max_daily: ElectricPotential::default(),
            ah_charge_daily: ElectricCharge::default(),
            ah_load_daily: ElectricCharge::default(),
            array_faults_daily: ArrayFaults::default(),
            load_faults_daily: LoadFaults::default(),
            alarms_daily: Alarms::default(),
            array_voltage_max_daily: ElectricPotential::default(),
            array_voltage_fixed: ElectricPotential::default(),
            array_voc_percent_fixed: 0.,
        }
    }
}

macro_rules! as_unit {
    ($f:ident, $obj:ident, $field:ident, $unit:ident) => {
        write!(
            $f,
            "    {}: {:.2} {},\n",
            stringify!($field),
            $obj.$field.get::<$unit>(),
            $unit::abbreviation()
        )
    };
}

impl fmt::Display for Stats {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Stats {{\n")?;
        write!(f, "    timestamp: {},\n", self.timestamp)?;
        write!(f, "    software_version: {},\n", self.software_version)?;
        write!(
            f,
            "    battery_voltage_settings_multiplier: {},\n",
            self.battery_voltage_settings_multiplier
        )?;
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
                f,
                "    rts_temperature: {} {:.2},\n",
                t.get::<degree_celsius>(),
                degree_celsius::abbreviation()
            )?,
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
        as_unit!(f, self, array_max_power_sweep, watt)?;
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
    pub charge_current_limit: ElectricCurrent,
}

macro_rules! validate {
    ($o:ident, $field:ident, $unit:ident, $min:expr, $max:expr) => {
        if $o.$field < $unit($min) || $o.$field > $unit($max) {
            bail!("{} {} <= x <= {}", stringify!($field), $min, $max)
        }
    };
}

impl fmt::Display for Settings {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Settings {{\n")?;
        as_unit!(f, self, regulation_voltage, volt)?;
        as_unit!(f, self, float_voltage, volt)?;
        as_unit!(f, self, time_before_float, second)?;
        as_unit!(f, self, time_before_float_low_battery, second)?;
        as_unit!(f, self, float_low_battery_voltage_trigger, volt)?;
        as_unit!(f, self, float_cancel_voltage, volt)?;
        as_unit!(f, self, exit_float_time, second)?;
        as_unit!(f, self, equalize_voltage, volt)?;
        as_unit!(f, self, days_between_equalize_cycles, day)?;
        as_unit!(f, self, equalize_time_limit_above_regulation_voltage, second)?;
        as_unit!(f, self, equalize_time_limit_at_regulation_voltage, second)?;
        write!(f, "    alarm_on_setting_change: {},\n", self.alarm_on_setting_change)?;
        as_unit!(f, self, reference_charge_voltage_limit, volt)?;
        as_unit!(f, self, battery_charge_current_limit, ampere)?;
        as_unit!(f, self, temperature_compensation_coefficent, volt)?;
        as_unit!(f, self, high_voltage_disconnect, volt)?;
        as_unit!(f, self, high_voltage_reconnect, volt)?;
        as_unit!(f, self, maximum_charge_voltage_reference, volt)?;
        as_unit!(f, self, max_battery_temp_compensation_limit, degree_celsius)?;
        as_unit!(f, self, min_battery_temp_compensation_limit, degree_celsius)?;
        as_unit!(f, self, load_low_voltage_disconnect, volt)?;
        as_unit!(f, self, load_low_voltage_reconnect, volt)?;
        as_unit!(f, self, load_high_voltage_disconnect, volt)?;
        as_unit!(f, self, load_high_voltage_reconnect, volt)?;
        as_unit!(f, self, lvd_load_current_compensation, ohm)?;
        as_unit!(f, self, lvd_warning_timeout, minute)?;
        as_unit!(f, self, led_green_to_green_and_yellow_limit, volt)?;
        as_unit!(f, self, led_green_and_yellow_to_yellow_limit, volt)?;
        as_unit!(f, self, led_yellow_to_yellow_and_red_limit, volt)?;
        as_unit!(f, self, led_yellow_and_red_to_red_flashing_limit, volt)?;
        write!(f, "    modbus_id: {},\n", self.modbus_id)?;
        write!(f, "    meterbus_id: {},\n", self.meterbus_id)?;
        as_unit!(f, self, mppt_fixed_vmp, volt)?;
        write!(f, "    mppt_fixed_vmp_percent: {},\n", self.mppt_fixed_vmp_percent)?;
        as_unit!(f, self, charge_current_limit, ampere)?;
        write!(f, "}}")?;
        Ok(())
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
    ResetControl,
}

impl Coil {
    fn address(&self) -> u16 {
        match self {
            Coil::EqualizeTriggered => 0x0000,
            Coil::LoadDisconnect => 0x0001,
            Coil::ChargeDisconnect => 0x0002,
            Coil::ClearAhResettable => 0x0010,
            Coil::ClearAhTotal => 0x0011,
            Coil::ClearKwhResettable => 0x0012,
            Coil::ClearFaults => 0x0014,
            Coil::ClearAlarms => 0x0015,
            Coil::ForceEEPROMUpdate => 0x0016,
            Coil::ClearKwhTotal => 0x0018,
            Coil::ClearVbMinMax => 0x0019,
            Coil::LightingModeTest => 0x0020,
            Coil::FactoryReset => 0x00FE,
            Coil::ResetControl => 0x00FF,
        }
    }
}

/** Device connection. */
pub struct Connection(Modbus);

impl Connection {
    pub async fn new(device: &str, modbus_id: u8) -> Result<Connection> {
        let port = SerialStream::open(
            &tokio_serial::new(device, 9600)
                .data_bits(DataBits::Eight)
                .flow_control(FlowControl::None)
                .parity(Parity::None)
                .stop_bits(StopBits::Two)
                .timeout(Duration::from_secs(10)),
        )
        .context("failed to connect to serial port")?;
        let con = rtu::connect_slave(port, Slave(modbus_id))
            .await
            .context("failed to build modbus context")?;
        Ok(Connection(con))
    }

    pub async fn read_coil(&mut self, coil: Coil) -> Result<bool> {
        let res =
            self.0.read_coils(coil.address(), 1).await.context("read coil failed")?;
        if res.len() != 1 {
            bail!("wrong number of coils read {} expected 1", res.len())
        }
        Ok(res[0])
    }

    pub async fn write_coil(&mut self, coil: Coil, val: bool) -> Result<()> {
        Ok(self
            .0
            .write_single_coil(coil.address(), val)
            .await
            .context("failed to write coil")?)
    }

    pub async fn stats(&mut self) -> Result<Stats> {
        let raw = self
            .0
            .read_holding_registers(0x0, 81)
            .await
            .context("stats failed to read holding registers")?;
        if raw.len() != 81 {
            bail!("stats wrong number of registers read {} expected 80", raw.len())
        }
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
                if t.is_nan() {
                    None
                } else {
                    Some(c(t))
                }
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
            kwh_charge_resettable: kwh(gf32(raw[0x002A])),
            kwh_charge_total: kwh(gf32(raw[0x002B])),
            load_state: LoadState::from(raw[0x002E]),
            load_faults: LoadFaults::from_bits_truncate(raw[0x002F]),
            lvd_setpoint: v(gf32(raw[0x0030])),
            ah_load_resettable: ah(gu32(raw[0x0032], raw[0x0033]) as f32 * 0.1),
            ah_load_total: ah(gu32(raw[0x0034], raw[0x0035]) as f32 * 0.1),
            hourmeter: hr(gu32(raw[0x0036], raw[0x0037]) as f32),
            alarms: Alarms::from_bits_truncate(
                (raw[0x0038] as u32) << 16 | raw[0x0039] as u32,
            ),
            array_power: w(gf32(raw[0x003C])),
            array_vmp: v(gf32(raw[0x003D])),
            array_max_power_sweep: w(gf32(raw[0x003E])),
            array_voc: v(gf32(raw[0x003F])),
            battery_v_min_daily: v(gf32(raw[0x0041])),
            battery_v_max_daily: v(gf32(raw[0x0042])),
            ah_charge_daily: ah(gf32(raw[0x0043])),
            ah_load_daily: ah(gf32(raw[0x0044])),
            array_faults_daily: ArrayFaults::from_bits_truncate(raw[0x0045]),
            load_faults_daily: LoadFaults::from_bits_truncate(raw[0x0046]),
            alarms_daily: Alarms::from_bits_truncate(
                (raw[0x0047] as u32) << 16 | raw[0x0048] as u32,
            ),
            array_voltage_max_daily: v(gf32(raw[0x004C])),
            array_voltage_fixed: v(gf32(raw[0x004F])),
            array_voc_percent_fixed: gf32(raw[0x0050]),
        })
    }

    pub async fn read_settings(&mut self) -> Result<Settings> {
        let len = ((SETTINGS_END - SETTINGS_BASE) + 1) as u16;
        let raw = self
            .0
            .read_holding_registers(SETTINGS_BASE as u16, len)
            .await
            .context("read_settings failed to read registers")?;
        if raw.len() != len as usize {
            bail!(
                "read_settings read unexpected number of registers {} expected {}",
                raw.len(),
                len
            );
        }
        Ok(Settings {
            regulation_voltage: v(gf32(raw[0xE000 - SETTINGS_BASE])),
            float_voltage: v(gf32(raw[0xE001 - SETTINGS_BASE])),
            time_before_float: sec(raw[0xE002 - SETTINGS_BASE] as f32),
            time_before_float_low_battery: sec(raw[0xE003 - SETTINGS_BASE] as f32),
            float_low_battery_voltage_trigger: v(gf32(raw[0xE004 - SETTINGS_BASE])),
            float_cancel_voltage: v(gf32(raw[0xE005 - SETTINGS_BASE])),
            exit_float_time: sec(raw[0xE006 - SETTINGS_BASE] as f32),
            equalize_voltage: v(gf32(raw[0xE007 - SETTINGS_BASE])),
            days_between_equalize_cycles: dy(raw[0xE008 - SETTINGS_BASE] as f32),
            equalize_time_limit_above_regulation_voltage: sec(
                raw[0xE009 - SETTINGS_BASE] as f32,
            ),
            equalize_time_limit_at_regulation_voltage: sec(
                raw[0xE00A - SETTINGS_BASE] as f32
            ),
            alarm_on_setting_change: raw[0xE00D - SETTINGS_BASE] == 1,
            reference_charge_voltage_limit: v(gf32(raw[0xE010 - SETTINGS_BASE])),
            battery_charge_current_limit: a(gf32(raw[0xE013 - SETTINGS_BASE])),
            temperature_compensation_coefficent: v(gf32(raw[0xE01A - SETTINGS_BASE])),
            high_voltage_disconnect: v(gf32(raw[0xE01B - SETTINGS_BASE])),
            high_voltage_reconnect: v(gf32(raw[0xE01C - SETTINGS_BASE])),
            maximum_charge_voltage_reference: v(gf32(raw[0xE01D - SETTINGS_BASE])),
            max_battery_temp_compensation_limit: ic(raw[0xE01E - SETTINGS_BASE]),
            min_battery_temp_compensation_limit: ic(raw[0xE01F - SETTINGS_BASE]),
            load_low_voltage_disconnect: v(gf32(raw[0xE022 - SETTINGS_BASE])),
            load_low_voltage_reconnect: v(gf32(raw[0xE023 - SETTINGS_BASE])),
            load_high_voltage_disconnect: v(gf32(raw[0xE024 - SETTINGS_BASE])),
            load_high_voltage_reconnect: v(gf32(raw[0xE025 - SETTINGS_BASE])),
            lvd_load_current_compensation: om(gf32(raw[0xE026 - SETTINGS_BASE])),
            lvd_warning_timeout: mn(raw[0xE027 - SETTINGS_BASE] as f32),
            led_green_to_green_and_yellow_limit: v(gf32(raw[0xE030 - SETTINGS_BASE])),
            led_green_and_yellow_to_yellow_limit: v(gf32(raw[0xE031 - SETTINGS_BASE])),
            led_yellow_to_yellow_and_red_limit: v(gf32(raw[0xE032 - SETTINGS_BASE])),
            led_yellow_and_red_to_red_flashing_limit: v(gf32(
                raw[0xE033 - SETTINGS_BASE],
            )),
            modbus_id: raw[0xE034 - SETTINGS_BASE] as u8,
            meterbus_id: raw[0xE035 - SETTINGS_BASE] as u8,
            mppt_fixed_vmp: v(gf32(raw[0xE036 - SETTINGS_BASE])),
            mppt_fixed_vmp_percent: gf32(raw[0xE037 - SETTINGS_BASE]),
            charge_current_limit: a(gf32(raw[0xE038 - SETTINGS_BASE])),
        })
    }

    async fn write_setting(&mut self, addr: usize, cur: &[u16], new: u16) -> Result<()> {
        if cur[addr - SETTINGS_BASE] == new {
            Ok(())
        } else {
            sleep(Duration::from_millis(100));
            Ok(self
                .0
                .write_single_register(addr as u16, new)
                .await
                .context("write_setting failed to write to register")?)
        }
    }

    /// They will not take effect until the controller is reset, and
    /// if alarm_on_setting_change is false the controller will not
    /// work until a reset.
    pub async fn write_settings(&mut self, settings: &Settings) -> Result<()> {
        settings.validate()?;
        let len = (SETTINGS_END - SETTINGS_BASE) as u16;
        let cur = self
            .0
            .read_holding_registers(SETTINGS_BASE as u16, len)
            .await
            .context("write_settings failed to read current settings")?;
        if cur.len() != len as usize {
            bail!(
                "write_settings, read unexpected number of settings {} expected {}",
                cur.len(),
                len
            )
        }
        self.write_setting(0xE000, &cur, to_v(settings.regulation_voltage)).await?;
        self.write_setting(0xE001, &cur, to_v(settings.float_voltage)).await?;
        self.write_setting(0xE002, &cur, to_sec(settings.time_before_float)).await?;
        self.write_setting(0xE003, &cur, to_sec(settings.time_before_float_low_battery))
            .await?;
        self.write_setting(
            0xE004,
            &cur,
            to_v(settings.float_low_battery_voltage_trigger),
        )
        .await?;
        self.write_setting(0xE005, &cur, to_v(settings.float_cancel_voltage)).await?;
        self.write_setting(0xE006, &cur, to_sec(settings.exit_float_time)).await?;
        self.write_setting(0xE007, &cur, to_v(settings.equalize_voltage)).await?;
        self.write_setting(0xE008, &cur, to_dy(settings.days_between_equalize_cycles))
            .await?;
        self.write_setting(
            0xE009,
            &cur,
            to_sec(settings.equalize_time_limit_above_regulation_voltage),
        )
        .await?;
        self.write_setting(
            0xE00A,
            &cur,
            to_sec(settings.equalize_time_limit_at_regulation_voltage),
        )
        .await?;
        self.write_setting(
            0xE00D,
            &cur,
            if settings.alarm_on_setting_change { 1 } else { 0 },
        )
        .await?;
        self.write_setting(0xE010, &cur, to_v(settings.reference_charge_voltage_limit))
            .await?;
        self.write_setting(0xE013, &cur, to_a(settings.battery_charge_current_limit))
            .await?;
        self.write_setting(
            0xE01A,
            &cur,
            to_v(settings.temperature_compensation_coefficent),
        )
        .await?;
        self.write_setting(0xE01B, &cur, to_v(settings.high_voltage_disconnect)).await?;
        self.write_setting(0xE01C, &cur, to_v(settings.high_voltage_reconnect)).await?;
        self.write_setting(0xE01D, &cur, to_v(settings.maximum_charge_voltage_reference))
            .await?;
        self.write_setting(
            0xE01E,
            &cur,
            to_ic(settings.max_battery_temp_compensation_limit),
        )
        .await?;
        self.write_setting(
            0xE01F,
            &cur,
            to_ic(settings.min_battery_temp_compensation_limit),
        )
        .await?;
        self.write_setting(0xE022, &cur, to_v(settings.load_low_voltage_disconnect))
            .await?;
        self.write_setting(0xE023, &cur, to_v(settings.load_low_voltage_reconnect))
            .await?;
        self.write_setting(0xE024, &cur, to_v(settings.load_high_voltage_disconnect))
            .await?;
        self.write_setting(0xE025, &cur, to_v(settings.load_high_voltage_reconnect))
            .await?;
        self.write_setting(0xE026, &cur, to_om(settings.lvd_load_current_compensation))
            .await?;
        self.write_setting(0xE027, &cur, to_mn(settings.lvd_warning_timeout)).await?;
        self.write_setting(
            0xE030,
            &cur,
            to_v(settings.led_green_to_green_and_yellow_limit),
        )
        .await?;
        self.write_setting(
            0xE031,
            &cur,
            to_v(settings.led_green_and_yellow_to_yellow_limit),
        )
        .await?;
        self.write_setting(
            0xE032,
            &cur,
            to_v(settings.led_yellow_to_yellow_and_red_limit),
        )
        .await?;
        self.write_setting(
            0xE033,
            &cur,
            to_v(settings.led_yellow_and_red_to_red_flashing_limit),
        )
        .await?;
        self.write_setting(0xE034, &cur, settings.modbus_id as u16).await?;
        self.write_setting(0xE035, &cur, settings.meterbus_id as u16).await?;
        self.write_setting(0xE036, &cur, to_v(settings.mppt_fixed_vmp)).await?;
        self.write_setting(
            0xE037,
            &cur,
            f16::from_f32(settings.mppt_fixed_vmp_percent).to_bits(),
        )
        .await?;
        self.write_setting(0xE038, &cur, to_a(settings.charge_current_limit)).await?;
        Ok(())
    }
}
