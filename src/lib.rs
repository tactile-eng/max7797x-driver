#![no_std]
#![warn(missing_docs)]

//! An embedded async driver for the MAX77975/MAX77976 19VIN, 3.5/5.5A 1-Cell Li+ Battery Charger with Smart Power
//! Selector and OTG for USBC PD

use embedded_hal_async::i2c::I2c;
use modular_bitfield::specifiers::{B1, B2};
use modular_bitfield::{bitfield, BitfieldSpecifier};

const ADDR: u8 = 0x6b;

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
struct Reg(pub u8);

#[allow(dead_code)]
impl Reg {
    pub const CHIP_ID: Reg = Reg(0x00);
    pub const CHIP_REVISION: Reg = Reg(0x01);
    pub const OTP_REVISION: Reg = Reg(0x02);
    pub const TOP_INTERRUPT: Reg = Reg(0x03);
    pub const TOP_INTERRUPT_MASK: Reg = Reg(0x04);
    pub const TOP_CONTROL: Reg = Reg(0x05);
    pub const SOFTWARE_RESET: Reg = Reg(0x50);
    pub const SHIP_MODE_CONTROL: Reg = Reg(0x51);
    pub const I2C_CONFIG: Reg = Reg(0x40);

    pub const CHARGER_INTERRUPT: Reg = Reg(0x10);
    pub const CHARGER_INTERRUPT_MASK: Reg = Reg(0x11);
    pub const CHARGER_INTERRUPT_STATUS: Reg = Reg(0x12);
    pub const CHARGER_DETAILS_0: Reg = Reg(0x13);
    pub const CHARGER_DETAILS_1: Reg = Reg(0x14);
    pub const CHARGER_DETAILS_2: Reg = Reg(0x15);
    pub const CHARGER_CONFIG_0: Reg = Reg(0x16);
    pub const CHARGER_CONFIG_1: Reg = Reg(0x17);
    pub const CHARGER_CONFIG_2: Reg = Reg(0x18);
    pub const CHARGER_CONFIG_3: Reg = Reg(0x19);
    pub const CHARGER_CONFIG_4: Reg = Reg(0x1a);
    pub const CHARGER_CONFIG_5: Reg = Reg(0x1b);
    pub const CHARGER_CONFIG_6: Reg = Reg(0x1c);
    pub const CHARGER_CONFIG_7: Reg = Reg(0x1d);
    pub const CHARGER_CONFIG_8: Reg = Reg(0x1e);
    pub const CHARGER_CONFIG_9: Reg = Reg(0x1f);
    pub const CHARGER_CONFIG_10: Reg = Reg(0x20);
    pub const CHARGER_CONFIG_11: Reg = Reg(0x21);
    pub const CHARGER_CONFIG_12: Reg = Reg(0x22);
    pub const CHARGER_CONFIG_13: Reg = Reg(0x23);
    pub const STATUS_LED_CONFIG: Reg = Reg(0x24);

    pub const fn new(val: u8) -> Self {
        Reg(val)
    }

    pub const fn to_u8(self) -> u8 {
        self.0
    }
}

/// A MAX77975/MAX77976 battery charger.
pub struct Charger<D> {
    i2c_dev: D,
}

impl<D: I2c> Charger<D> {
    /// Create a new `Charger`
    pub fn new(i2c_dev: D) -> Self {
        Charger { i2c_dev }
    }

    /// Set the current limit for Vsys out.
    ///
    /// If the current limit is exceeded, Vsys will be shut off. If `recycle_en` is false, it will remain
    /// shut off until a valid charger is present. Otherwise, it will attempt to recycle after 150ms.
    pub async fn set_sys_ilim(&mut self, milliamps: u16, recycle_en: bool) -> Result<(), D::Error> {
        let sys_ilim = (milliamps / 500).saturating_sub(5).min(0xf) as u8;
        let recycle_en = if recycle_en { 0x10 } else { 0x00 };
        self.write_protected_reg(Reg::CHARGER_CONFIG_5, recycle_en | sys_ilim)
            .await
    }

    /// Set the current limit for CHGIN.
    pub async fn set_chgin_ilim(&mut self, milliamps: u16) -> Result<(), D::Error> {
        let chgin_ilim = (milliamps / 50).saturating_sub(1).min(0x3f) as u8;
        self.modify_reg(Reg::CHARGER_CONFIG_9, |val| (val & 0xc0) | chgin_ilim)
            .await
    }

    /// Set the current to use during the [`ChargerDetails::ConstantCurrent`] charging phase.
    pub async fn set_fast_charge_current(&mut self, milliamps: u16) -> Result<(), D::Error> {
        let chg_cc = (milliamps / 50).min(0x7f) as u8;
        self.write_protected_reg(Reg::CHARGER_CONFIG_2, chg_cc)
            .await
    }

    /// Set the charger [`Mode`].
    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), D::Error> {
        self.write_reg(Reg::CHARGER_CONFIG_0, mode as u8).await
    }

    /// Enter ship mode.
    ///
    /// All power will be shut down and remain off until a valid charger is present. Ship mode
    /// can not be enetered when a valid charger is present.
    pub async fn enter_ship_mode(&mut self) -> Result<(), D::Error> {
        self.write_reg(Reg::SHIP_MODE_CONTROL, 0x01).await
    }

    /// Enable charger interrupts.
    ///
    /// Fields set to `true` in `irqs` will have their interrupts enabled.
    pub async fn set_charger_irq_mask(&mut self, irqs: ChargerInterrupts) -> Result<(), D::Error> {
        self.write_reg(Reg::CHARGER_INTERRUPT_MASK, !irqs.into_bytes()[0])
            .await
    }

    /// Reads and clears the current charger interrupt flags
    pub async fn charger_irq_flags(&mut self) -> Result<ChargerInterrupts, D::Error> {
        self.read_reg(Reg::CHARGER_INTERRUPT)
            .await
            .map(|x| ChargerInterrupts::from_bytes([x]))
    }

    /// Clears the charger interrupt flags and returns the current status bits
    ///
    /// This method reads from `Reg::CHARGER_INTERRUPT` through `Reg::CHARGER_INTERRUPT_STATUS`
    /// in a single transaction to atomically clear the interrupt flags and return the current
    /// status bits.
    pub async fn charger_status(&mut self) -> Result<ChargerInterrupts, D::Error> {
        let mut buf = [0; 3];
        self.read_buf(Reg::CHARGER_INTERRUPT, &mut buf)
            .await
            .map(|_| ChargerInterrupts::from_bytes([buf[2]]))
    }

    /// Get the detailed status of the charger.
    pub async fn charger_details(&mut self) -> Result<Details, D::Error> {
        let mut buf = [0; 3];
        self.read_buf(Reg::CHARGER_DETAILS_0, &mut buf).await?;
        Ok(Details::from_bytes(buf))
    }

    async fn read_reg(&mut self, reg: Reg) -> Result<u8, D::Error> {
        let mut val = 0u8;
        self.i2c_dev
            .write_read(
                ADDR,
                core::slice::from_ref(&reg.to_u8()),
                core::slice::from_mut(&mut val),
            )
            .await?;
        Ok(val)
    }

    async fn read_buf(&mut self, base: Reg, buf: &mut [u8]) -> Result<(), D::Error> {
        self.i2c_dev
            .write_read(ADDR, core::slice::from_ref(&base.to_u8()), buf)
            .await
    }

    async fn write_reg(&mut self, reg: Reg, val: u8) -> Result<(), D::Error> {
        let buf = [reg.to_u8(), val];
        self.i2c_dev.write(ADDR, &buf).await
    }

    async fn write_protected_reg(&mut self, reg: Reg, val: u8) -> Result<(), D::Error> {
        self.write_reg(Reg::CHARGER_CONFIG_6, 0x0c).await?;
        let res = self.write_reg(reg, val).await;
        self.write_reg(Reg::CHARGER_CONFIG_6, 0x00).await?;
        res
    }

    async fn modify_reg<F: FnOnce(u8) -> u8>(&mut self, reg: Reg, func: F) -> Result<(), D::Error> {
        let val = self.read_reg(reg).await?;
        let val = func(val);
        self.write_reg(reg, val).await
    }
}

#[bitfield(bits = 8)]
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// The charger interrupt flags
pub struct ChargerInterrupts {
    pub bypass_node: bool,
    pub disqbat: bool,
    #[skip]
    __: B1,
    pub battery: bool,
    pub charger: bool,
    pub input_current_limit: bool,
    pub chgin: bool,
    pub adaptive_input_current_loop: bool,
}

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[bits = 4]
/// Charging mode
pub enum Mode {
    #[default]
    /// Charger = off, OTG = off, buck = off, boost = off.
    /// The QBATT switch is on to allow the battery to support the system. BYP may or may not be biased based on the CHGIN availability.
    Off = 0x0,
    /// Charger = off, OTG = off, buck = on, boost = off.
    /// When there is a valid input, the buck converter regulates the system voltage to be the maximum of (Vminsys and VBATT +4%).
    /// VBYP is equal to VCHGIN minus the resistive drops.
    Buck = 0x4,
    /// Charger = on, OTG = off, buck = on, boost = off.
    /// When there is a valid input, the battery is charging. VSYS is the larger of VSYSMIN and ~VBATT + IBATT x RBAT2SYS.
    /// VBYP is equal to VCHGIN minus the resistive drops.
    Charge = 0x5,
    /// Charger = off, OTG = off, buck = off, boost = on.
    /// The QBATT switch is on to allow the battery to support the system, the charger's DC-DC operates as a boost converter.
    /// BYP voltage is regulated to VBYPSET. QCHGIN is off.
    Boost = 0x9,
    /// Charger = off, OTG = on, buck = off, boost = on.
    /// The QBATT switch is on to allow the battery to support the system, the charger's DC-DC operates as a boost converter.
    /// BYP voltage is regulated to VBYPSET. QCHGIN is on allowing it to source current up to ICHGIN.OTG.LIM.
    Otg = 0xa,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[bits = 2]
/// CHGIN status
pub enum ChgIn {
    /// VBUS is invalid. VCHGIN rising: VCHGIN < VCHGIN_UVLO VCHGIN falling: VCHGIN < VCHGIN_REG (AICL)
    Undervoltage,
    /// VBUS is invalid. VCHGIN < VBATT + VCHGIN2SYS and VCHGIN > VCHGIN_UVLO
    BelowBatt,
    /// VBUS is invalid. VCHGIN > VCHGIN_OVLO
    Overvoltage,
    /// VBUS is valid. VCHGIN > VCHGIN_UVLO and VCHGIN > VBATT + VCHGIN2SYS and VCHGIN < VCHGIN_OVLO
    Valid,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[bits = 2]
/// Battery sense status
pub enum BatterySense {
    /// SPSN remote sense line is connected.
    Connected,
    /// SP remote sense line detected as opened.
    PositiveOpen,
    /// SN remote sense line detected as opened.
    NegativeOpen,
    /// SP and SN remote sense lines are both detected as opened.
    BothOpen,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[bits = 1]
/// Temperature regulation status
pub enum TemperatureRegulation {
    /// The junction temperature is less than the threshold set by REGTEMP and the full charge current limit is available.
    BelowThreshold,
    /// The junction temperature is greater than the threshold set by REGTEMP and the charge current limit may be folding back to reduce power dissipation.
    AboveThreshold,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[bits = 3]
/// Battery status
pub enum BatteryDetails {
    /// Battery Removal A valid adpater is present and the battery is detached, detected on THM pin.
    BatteryRemoved,
    /// Battery Prequalification Voltage A valid adapter is present and the battery voltage is low: VBATT < VTRICKLE.
    ///
    /// *Note:* This condition is also reported in the CHG_DTLS as 0x00.
    PrequalificationVoltage,
    /// Battery Timer Fault A valid adapter is present and the battery has taken longer than expected to charge (exceeded tFC). This could be due to high system currents, an old battery, a damaged battery, or something else. Charging has suspended and the charger is in timer-fault mode.
    ///
    /// *Note:* This condition is also reported in the CHG_DTLS as 0x06.
    TimerFault,
    /// Battery Regular Voltage A valid adapter is present and the battery voltage is greater than the minimum system regulation level but lower than overvoltage level: VSYSMIN < VBATT < VBATTREG + VCOV VSYS is approximately equal to VBATT.
    RegularVoltage,
    /// Battery Low Voltage A valid adapter is present and the battery voltage is lower than the minimum system regulation level but higher than prequalification voltage: VTRICKLE < VBATT < VSYSMIN VSYS is regulated at least equal to VSYSMIN.
    LowVoltage,
    /// Battery Overvoltage A valid adapter is present and the battery voltage is greater than the battery- overvoltage threshold (VBATTREG + VCOV) for the last 30ms.
    ///
    /// *Note:* This flag is only generated when there is a valid input.
    Overvoltage,
    #[doc(hidden)]
    Reserved,
    /// Battery Only No valid adapter is present The battery voltage and battery removal monitoring are not available.
    ///
    /// *Note:* In case of deep suspend, it is considered that no valid adapter is present.
    BatteryOnly = 7,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[bits = 4]
/// Charger status
pub enum ChargerDetails {
    /// Charger is in dead-battery prequalification or low-battery prequalification mode. CHG_OK = 1 and VBATT < VPQLB and TJ < TSHDN
    Prequalification,
    /// Charger is in fast-charge constant current mode. CHG_OK = 1 and VBATT < VBATTREG and TJ < TSHDN
    ConstantCurrent,
    /// Charger is in fast-charge constant voltage mode. CHG_OK = 1 and VBATT = VBATTREG and TJ < TSHDN
    ConstantVoltage,
    /// Charger is in top-off mode. CHG_OK = 1 and VBATT = VBATTREG and TJ < TSHDN
    TopOff,
    /// Charger is in done mode. CHG_OK = 0 and VBATT > VBATTREG - VRSTRT and TJ < TSHDN
    Done,
    #[doc(hidden)]
    Reserved05,
    /// Charger is in timer-fault mode. CHG_OK = 0 and if BAT_DTLS=0b001 then VBATT < VPQLB or VBATT < VPQDB and TJ < TSHDN
    TimerFault = 6,
    /// Charger is suspended because QBATT is disabled (DISQBAT = H or DISIBS = 1). CHG_OK = 0
    QBattDisabled,
    /// Charger is off, charger input invalid and/or charger is disabled. CHG_OK = 1
    Off,
    #[doc(hidden)]
    Reserved09,
    /// Charger is off and the junction temperature is > TSHDN. CHG_OK = 0
    HighTemperature = 0x0a,
    /// Charger is off because the watchdog timer expired. CHG_OK = 0
    WatchdogTimer,
    /// Charger is suspended or charge current or voltage is reduced based on JEITA control. This condition is also reported in THM_DTLS. CHG_OK = 0
    Jeita,
    /// Charger is suspended because battery removal is detected on THM pin. This condition is also reported in THM_DTLS. CHG_OK = 0
    ThermistorRemoval,
    /// Charger is suspended because SUSPEND pin is high. CHG_OK = 0
    SuspendPin,
    #[doc(hidden)]
    Reserved0F,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[bits = 3]
/// Thermistor status
pub enum ThermistorDetails {
    /// Low temperature and charging suspended
    Cold,
    /// Low temperature charging
    Cool,
    /// Normal temperature charging
    Normal,
    /// High temperature charging
    Warm,
    /// High temperaure and charging suspended
    Hot,
    /// Battery removal detected on THM pin
    Removed,
    /// Thermistor monitoring is disabled
    Disabled,
    #[doc(hidden)]
    Reserved,
}

#[repr(C, align(1))]
#[bitfield(bits = 4)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// Bypass node status
pub struct BypassNodeDetails {
    pub otg_current_limit: bool,
    pub boost_current_limit: bool,
    pub buck_current_limit: bool,
    pub boost_on: bool,
}

#[repr(C, align(1))]
#[bitfield(bits = 24)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// Detailed status of the charger
pub struct Details {
    #[skip]
    __: B1,
    #[bits = 2]
    pub sense: BatterySense,
    #[skip]
    __: B2,
    #[bits = 2]
    pub chgin: ChgIn,
    #[skip]
    __: B1,
    #[bits = 4]
    pub charger: ChargerDetails,
    #[bits = 3]
    pub battery: BatteryDetails,
    #[bits = 1]
    pub temp: TemperatureRegulation,
    #[bits = 4]
    pub bypass: BypassNodeDetails,
    #[bits = 3]
    pub thermistor: ThermistorDetails,
    #[skip]
    __: B1,
}
