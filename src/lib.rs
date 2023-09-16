//! # TLV230AIC23B Driver
//!
//! This is driver for the Texas Instruments TLV320AIC23B audio CODEC /
//! amplifier.
//!
//! Specifically, this driver is for setting the registers in the TLV230AIC23B
//! over I²C - this driver does not handle the digital audio interface (I²S, or
//! similar).
//!
//! The TLV230AIC23B has the following inputs and outputs:
//!
//! * Stereo analog Line-level Input
//! * Mono analog Microphone Input, with optional +20dB boost
//! * Stereo analog Line-level Output
//! * Stereo analog Amplified Headphone Output
//! * Stereo digital Output
//! * Stereo digital Input
//!
//! The [`Codec`] object must cache the register contents because the
//! TLV320AIC23B only has a *write-only* interface and you cannot read back any
//! register contents.
//!
//! # Example
//!
//! You might setup the Codec like this:
//!
//! ```rust
//! # use embedded_hal::blocking::i2c::Write;
//! # struct I2c;
//! # impl embedded_hal::blocking::i2c::Write for I2c {
//! #     type Error = ();
//! #     fn write(&mut self, address: embedded_hal::blocking::i2c::SevenBitAddress, bytes: &[u8]) -> Result<(), Self::Error> {
//! #         Ok(())
//! #     }
//! # }
//! # let mut i2c = I2c;
//! let mut codec = tlv320aic23::Codec::new(tlv320aic23::BusAddress::CsLow);
//! if let Err(e) = codec.reset(&mut i2c) {
//!     // Codec didn't respond
//! }
//! codec.set_digital_interface_enabled(true);
//! codec.set_dac_selected(true);
//! codec.set_dac_mute(false);
//! codec.set_bypass(false);
//! codec.set_line_input_mute(true, tlv320aic23::Channel::Both);
//! codec.set_powered_on(tlv320aic23::Subsystem::AnalogDigitalConverter, true);
//! codec.set_powered_on(tlv320aic23::Subsystem::MicrophoneInput, true);
//! codec.set_powered_on(tlv320aic23::Subsystem::LineInput, true);
//! codec.set_headphone_output_volume(50, tlv320aic23::Channel::Both);
//! codec.set_sample_rate(
//!     tlv320aic23::CONFIG_USB_44K1,
//!     tlv320aic23::LrSwap::Disabled,
//!     tlv320aic23::LrPhase::RightOnHigh,
//!     tlv320aic23::Mode::Controller,
//!     tlv320aic23::InputBitLength::B16,
//!     tlv320aic23::DataFormat::I2s,
//! );
//! if let Err(e) = codec.sync(&mut i2c) {
//!     // Codec didn't respond
//! }
//! ```

#![no_std]
#![deny(unsafe_code)]
#![deny(missing_docs)]

//
// Public Types
//

/// The TLV230AIC23B has one of two I²C addresses, depending on whether the CS
/// pin is pulled high or low.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum BusAddress {
    /// The address when the CS pin is high
    CsHigh = 0x1B,
    /// The address when the CS pin is low
    CsLow = 0x1A,
}

/// Selects either only the left channel, only the right channel, or both
/// channels together.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Channel {
    /// Just the left channel
    Left,
    /// Just the right channel
    Right,
    /// Both channels
    Both,
}

/// The level of sidetone that is added.
///
/// Sidetone is when audio input is routed directly to audio output. This is
/// useful when using a microphone and headphones as it allows you to hear
/// yourself more clearly.
pub enum Sidetone {
    /// Sidetone enabled at +0dB
    ZeroDb = 0b1001,
    /// Sidetone enabled at -6dB
    Minus6 = 0b0001,
    /// Sidetone enabled at -9dB
    Minus9 = 0b0011,
    /// Sidetone enabled at -12dB
    Minus12 = 0b0101,
    /// Sidetone enabled at -18dB
    Minus18 = 0b0111,
    /// Sidetone disabled
    Disabled = 0b0000,
}

/// Audio input devices available
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AudioInput {
    /// Select the line input
    LineInput = 0,
    /// Select the microphone input
    Microphone = 1,
}

/// Represents parts of the TLV230AIC23B that we can turn on and off
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Subsystem {
    /// Power for the whole device. Defaults to on.
    Device = 1 << 7,
    /// Clock. Defaults to on.
    Clock = 1 << 6,
    /// Oscillator. Defaults to on.
    Oscillator = 1 << 5,
    /// Outputs. Defaults to on.
    Outputs = 1 << 4,
    /// DAC. Defaults to on.
    DigitalAnalogConverter = 1 << 3,
    /// ADC. Defaults to off.
    AnalogDigitalConverter = 1 << 2,
    /// Microphone Input. Defaults to off.
    MicrophoneInput = 1 << 1,
    /// Line Input. Defaults to off.
    LineInput = 1 << 0,
}

/// Whether the TLV230AIC23B generates or receives clock signals.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Mode {
    /// TLV230AIC23B receives the BCLK and LRCLK signals. The documentation uses
    /// an archaic term beginning with S.
    Target = 0,
    /// TLV230AIC23B generates the BCLK and LRCLK signals. The documentation
    /// uses the archaic term beginning with M.
    Controller = 1,
}

/// The size, in bits, of each sample
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum InputBitLength {
    /// 16-bit samples
    B16 = 0b00,
    /// 20-bit samples
    B20 = 0b01,
    /// 24-bit samples
    B24 = 0b10,
    /// 32-bit samples
    B32 = 0b11,
}

/// How the data is sent over the digital bus
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DataFormat {
    /// MSB First, Right Aligned
    Right = 0b00,
    /// MSB First, Left Aligned
    Left = 0b01,
    /// I²S Format (i.e. MSB first, `left-1` aligned)
    I2s = 0b10,
    /// DSP Format (frame sync followed by two data words)
    Dsp = 0b11,
}

/// Swap the left and right DAC outputs
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum LrSwap {
    /// Don't swap left and right
    Disabled = 0,
    /// Swap Left and Right
    Enabled = 1,
}

/// Control whether LRClk is active high or low
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum LrPhase {
    /// Right Channel when LRCIN is high
    RightOnHigh = 0,
    /// Right Channel when LRCIN is low
    RightOnLow = 1,
}

/// Describes a specific configuration in terms of MCLK, ADC Sample Rate, and
/// DAC Sample Rate
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ConfigParams {
    /// True if you want to divive the clock input by 2.
    pub clk_in_div_2: bool,
    /// True if you want to divive the clock output by 2.
    pub clk_out_div_2: bool,
    /// True if you have a 12 MHz "USB" clock input, false if you have a
    /// different clock crystal
    pub is_usb: bool,
    /// Sample Rate Control
    ///
    /// Take the value from the tables in Datasheet section 3.3.2
    pub src: u8,
    /// Base Oversampling Rate
    ///
    /// Take the value from the tables in Datasheet section 3.3.2
    pub bosr: u8,
}

/// Represents the state inside our TLV230AIC23B chip.
pub struct Codec {
    bus_address: u8,
    register_cache: [(bool, u16); NUM_REGISTERS],
}

//
// Private Types
//

/// The set of registers in the TLV230AIC23B
#[derive(Copy, Clone, Debug)]
enum Register {
    LeftLineInputChannelVolumeControl = 0,
    RightLineInputChannelVolumeControl = 1,
    LeftChannelHeadphoneVolumeControl = 2,
    RightChannelHeadphoneVolumeControl = 3,
    AnalogAudioPathControl = 4,
    DigitalAudioPathControl = 5,
    PowerDownControl = 6,
    DigitalAudioInterfaceFormat = 7,
    SampleRateControl = 8,
    DigitalInterfaceActivation = 9,
    Reset = 15,
}

//
// Public Data
//

/// Configuration for USB Mode (12 MHz), ADC @ 96 kHz, DAC @ 96 kHz
///
/// See datasheet section 3.3.2
pub const CONFIG_USB_96K: ConfigParams = ConfigParams {
    clk_in_div_2: false,
    clk_out_div_2: false,
    is_usb: true,
    src: 0b0111,
    bosr: 0,
};

/// Configuration for USB Mode (12 MHz), ADC @ 48 kHz, DAC @ 48 kHz
///
/// See datasheet section 3.3.2
pub const CONFIG_USB_48K: ConfigParams = ConfigParams {
    clk_in_div_2: false,
    clk_out_div_2: false,
    is_usb: true,
    src: 0b0000,
    bosr: 0,
};

/// Configuration for USB Mode (12 MHz), ADC @ 44.1 kHz, DAC @ 44.1 kHz
///
/// See datasheet section 3.3.2
pub const CONFIG_USB_44K1: ConfigParams = ConfigParams {
    clk_in_div_2: false,
    clk_out_div_2: false,
    is_usb: true,
    src: 0b1000,
    bosr: 1,
};

/// Configuration for USB Mode (12 MHz), ADC @ 32 kHz, DAC @ 32 kHz
///
/// See datasheet section 3.3.2
pub const CONFIG_USB_32K: ConfigParams = ConfigParams {
    clk_in_div_2: false,
    clk_out_div_2: false,
    is_usb: true,
    src: 0b0110,
    bosr: 0,
};

/// Configuration for USB Mode (12 MHz), ADC @ 8 kHz, DAC @ 8 kHz
///
/// See datasheet section 3.3.2
pub const CONFIG_USB_8K: ConfigParams = ConfigParams {
    clk_in_div_2: false,
    clk_out_div_2: false,
    is_usb: true,
    src: 0b0010,
    bosr: 0,
};

//
// Private Data
//

const NUM_REGISTERS: usize = 10;

//
// impls on Public Types
//

impl From<BusAddress> for u8 {
    fn from(addr: BusAddress) -> u8 {
        addr as u8
    }
}

impl Codec {
    /// Create a new TLV230AIC23B CODEC proxy object.
    ///
    /// You can call methods on this object to set various parameters on the
    /// TLV230AIC23B. However, they won't take effect until you call the [Codec::sync]
    /// method.
    pub fn new(bus_address: BusAddress) -> Codec {
        Codec {
            bus_address: bus_address.into(),
            register_cache: [(false, 0); NUM_REGISTERS],
        }
    }

    /// Resets internal register cache to TLV230AIC23B defaults, as per the datasheet.
    fn set_register_defaults(&mut self) {
        let mut set = |reg, value| {
            self.register_cache[reg as usize] = (false, value);
        };
        // LeftLineInputChannelVolumeControl - Left Input muted
        set(Register::LeftLineInputChannelVolumeControl, 0b0_1001_0111);
        // RightLineInputChannelVolumeControl - Right Input muted
        set(Register::RightLineInputChannelVolumeControl, 0b0_1001_0111);
        // LeftChannelHeadphoneVolumeControl - Left Output 0dB, zero-cross enabled
        set(Register::LeftChannelHeadphoneVolumeControl, 0b0_1111_1001);
        // RightChannelHeadphoneVolumeControl - Right Output 0dB, zero-cross enabled
        set(Register::RightChannelHeadphoneVolumeControl, 0b0_1111_1001);
        // AnalogAudioPathControl - No sidetone, DAC off, bypass on, line in selected, mic muted
        set(Register::AnalogAudioPathControl, 0b0_0000_1010);
        // DigitalAudioPathControl - DAC soft mute, de-emphasis disabled, ADC high-pass filter on
        set(Register::DigitalAudioPathControl, 0b0_0000_1000);
        // PowerDownControl - Line In, Mic and ADC all off
        set(Register::PowerDownControl, 0b0_0000_0111);
        // DigitalAudioInterfaceFormat - MSB-first/left-aligned, 16-bit, lrc-high=right, lr-swap off, slave mode
        set(Register::DigitalAudioInterfaceFormat, 0b0_0000_0001);
        // SampleRateControl - Normal mode, 256fs, no clock divider, 44.1 kHz in/out
        set(Register::SampleRateControl, 0b0_0010_0000);
        // DigitalInterfaceActivation - Digital interface disabled
        set(Register::DigitalInterfaceActivation, 0b0_0000_0000);
    }

    /// Update one of the internal registers
    fn set_register_bits(&mut self, register: Register, value: u16, mask: u16) {
        // Clear the bits we want to change
        self.register_cache[register as usize].1 &= !mask;
        // Set any bits as necessary, but only in the cleared section
        self.register_cache[register as usize].1 |= value & mask;
        // Mark as dirty
        self.register_cache[register as usize].0 = true;
    }

    /// Read back one of the internal registers
    ///
    /// Reads from the cache because the TLV230AIC23B is write-only.
    fn get_register_bits(&self, register: Register, mask: u16) -> u16 {
        // Get the bits, after selecting only those bits in the mask
        self.register_cache[register as usize].1 & mask
    }

    /// Set whether line-in is muted
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_line_input_mute(&mut self, mute: bool, channel: Channel) {
        const MUTE_BIT: u16 = 1 << 7;
        let value = if mute { MUTE_BIT } else { 0 };
        if channel == Channel::Left || channel == Channel::Both {
            self.set_register_bits(Register::LeftLineInputChannelVolumeControl, value, MUTE_BIT);
        }
        if channel == Channel::Right || channel == Channel::Both {
            self.set_register_bits(
                Register::RightLineInputChannelVolumeControl,
                value,
                MUTE_BIT,
            );
        }
    }

    /// Get whether line-in is muted.
    ///
    /// See [`Codec::set_line_input_mute`].
    pub fn get_line_input_mute(&self) -> (bool, bool) {
        const MUTE_BIT: u16 = 1 << 7;
        let mute_left =
            self.get_register_bits(Register::LeftLineInputChannelVolumeControl, MUTE_BIT);
        let mute_right =
            self.get_register_bits(Register::RightLineInputChannelVolumeControl, MUTE_BIT);
        (mute_left != 0, mute_right != 0)
    }

    /// Set line-in volume
    ///
    /// * A value of 0 is -34.5 db
    /// * A value of 23 is 0 dB
    /// * A value of 31 is +12 db
    ///
    /// There is 1.5 dB per step. Values will be truncated to 5 bits long.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_line_input_volume_steps(&mut self, steps: u8, channel: Channel) {
        const MASK: u16 = 0b11111;
        if channel == Channel::Left || channel == Channel::Both {
            self.set_register_bits(
                Register::LeftLineInputChannelVolumeControl,
                steps as u16,
                MASK,
            );
        }
        if channel == Channel::Right || channel == Channel::Both {
            self.set_register_bits(
                Register::RightLineInputChannelVolumeControl,
                steps as u16,
                MASK,
            );
        }
    }

    /// Get current line-in volume for both channels.
    ///
    /// See [`Codec::set_line_input_volume_steps`].
    pub fn get_line_input_volume_steps(&self) -> (u8, u8) {
        const MASK: u16 = 0b11111;
        let steps_left =
            self.get_register_bits(Register::LeftLineInputChannelVolumeControl, MASK) as u8;
        let steps_right =
            self.get_register_bits(Register::RightLineInputChannelVolumeControl, MASK) as u8;
        (steps_left, steps_right)
    }

    /// Set headphone output volume.
    ///
    /// * A value of 0 is -73 db (muted)
    /// * A value of 73 is 0 dB
    /// * A value of 79 is +6 db
    ///
    /// There is 1 dB per step. Values over 79 are capped at 79.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_headphone_output_volume(&mut self, mut steps: u8, channel: Channel) {
        if steps > 79 {
            steps = 79;
        }
        // 48 is muted, and 127 is max volume
        steps += 48;
        const MASK: u16 = 0b1111111;
        if channel == Channel::Left || channel == Channel::Both {
            self.set_register_bits(
                Register::LeftChannelHeadphoneVolumeControl,
                steps as u16,
                MASK,
            );
        }
        if channel == Channel::Right || channel == Channel::Both {
            self.set_register_bits(
                Register::RightChannelHeadphoneVolumeControl,
                steps as u16,
                MASK,
            );
        }
    }

    /// Get current headphone volume for both channels.
    ///
    /// See [`Self::set_headphone_output_volume`]
    pub fn get_headphone_output_volume(&self) -> (u8, u8) {
        const MASK: u16 = 0b1111111;
        let steps_left =
            self.get_register_bits(Register::LeftChannelHeadphoneVolumeControl, MASK) as u8;
        let steps_right =
            self.get_register_bits(Register::RightChannelHeadphoneVolumeControl, MASK) as u8;
        (
            steps_left.saturating_sub(48),
            steps_right.saturating_sub(48),
        )
    }

    /// Set sidetone level
    ///
    /// Controls whether sidetone is added to the analog audio output. See
    /// [`Sidetone`].
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_sidetone_level(&mut self, sidetone: Sidetone) {
        const MASK: u16 = 0b1111 << 5;
        self.set_register_bits(
            Register::AnalogAudioPathControl,
            (sidetone as u16) << 5,
            MASK,
        );
    }

    /// Get sidetone level
    ///
    /// See [`Codec::set_sidetone_level`]
    pub fn get_sidetone_level(&self) -> Sidetone {
        let bits = self.get_register_bits(Register::AnalogAudioPathControl, 0b1111 << 5) >> 5;
        // Comes back as STA2 STA1 STA0 STE
        if bits & 0b0001 == 0 {
            // STE bit is clear
            Sidetone::Disabled
        } else if bits & 0b1001 == 0b1001 {
            // STE and STA2 bits are set
            Sidetone::ZeroDb
        } else {
            // Look at STA1 and STA0
            match (bits & 0b0110) >> 1 {
                0b00 => Sidetone::Minus6,
                0b01 => Sidetone::Minus9,
                0b10 => Sidetone::Minus12,
                _ => Sidetone::Minus18,
            }
        }
    }

    /// Control whether microphone boost is enabled.
    ///
    /// Boosts microphone input by +20dB when enabled.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_microphone_boost(&mut self, boost_enabled: bool) {
        const MASK: u16 = 1;
        self.set_register_bits(
            Register::AnalogAudioPathControl,
            if boost_enabled { MASK } else { 0 },
            MASK,
        );
    }

    /// Get whether microphone boost is enabled.
    ///
    /// See [`Codec::set_microphone_boost`]
    pub fn get_microphone_boost(&self) -> bool {
        self.get_register_bits(Register::AnalogAudioPathControl, 1) != 0
    }

    /// Set whether the microphone is muted.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_microphone_mute(&mut self, muted: bool) {
        const MASK: u16 = 1 << 1;
        self.set_register_bits(
            Register::AnalogAudioPathControl,
            if muted { MASK } else { 0 },
            MASK,
        );
    }

    /// Get whether the microphone is muted.
    pub fn get_microphone_mute(&mut self) -> bool {
        self.get_register_bits(Register::AnalogAudioPathControl, 1 << 1) != 0
    }

    /// Select which audio input goes to the ADC.
    ///
    /// You can only select one at a time as there is only one ADC.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_audio_input(&mut self, audio_input: AudioInput) {
        const MASK: u16 = 1 << 2;
        self.set_register_bits(
            Register::AnalogAudioPathControl,
            (audio_input as u16) << 2,
            MASK,
        );
    }

    /// Get which audio input goes to the ADC.
    ///
    /// See [`Codec::set_audio_input`]
    pub fn get_audio_input(&self) -> AudioInput {
        if self.get_register_bits(Register::AnalogAudioPathControl, 1 << 2) == 0 {
            AudioInput::LineInput
        } else {
            AudioInput::Microphone
        }
    }

    /// Set bypass mode.
    ///
    /// In bypass mode, the line input is routed to the line output, bypassing
    /// the ADC and DAC.
    ///
    /// If you want to silence the line output, you need bypass to be off.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_bypass(&mut self, bypass_enabled: bool) {
        const MASK: u16 = 1 << 3;
        self.set_register_bits(
            Register::AnalogAudioPathControl,
            if bypass_enabled { MASK } else { 0 },
            MASK,
        );
    }

    /// Get bypass mode
    ///
    /// See [`Codec::set_bypass_mode`]
    pub fn get_bypass(&self) -> bool {
        self.get_register_bits(Register::AnalogAudioPathControl, 1 << 3) != 0
    }

    /// Control whether the DAC is head at the analog output.
    ///
    /// The DAC must be selected (pass `true` here) to hear it on the speakers.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_dac_selected(&mut self, dac_selected: bool) {
        const MASK: u16 = 1 << 4;
        self.set_register_bits(
            Register::AnalogAudioPathControl,
            if dac_selected { MASK } else { 0 },
            MASK,
        );
    }

    /// Get whether the DAC is enabled or disabled.
    ///
    /// See [`Codec::set_dac_selected`]
    pub fn get_dac_selected(&self) -> bool {
        self.get_register_bits(Register::DigitalAudioPathControl, 1 << 4) != 0
    }

    /// Mute or unmute the DAC.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_dac_mute(&mut self, dac_muted: bool) {
        const MASK: u16 = 1 << 3;
        self.set_register_bits(
            Register::DigitalAudioPathControl,
            if dac_muted { MASK } else { 0 },
            MASK,
        );
    }

    /// Get whether the DAC is muted.
    pub fn get_dac_mute(&self) -> bool {
        self.get_register_bits(Register::DigitalAudioPathControl, 1 << 3) != 0
    }

    /// Control which parts of the TLV230AIC23B are powered on.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_powered_on(&mut self, subsystem: Subsystem, enabled: bool) {
        // 0 bit means on, 1 bit means off (it's the power *down* control)
        self.set_register_bits(
            Register::PowerDownControl,
            if enabled { 0 } else { 0xFFFF },
            subsystem as u16,
        );
    }

    /// Configure the TLV230AIC23B for a specific DAC and ADC sample rate, given
    /// a specific MCLK frequency.
    ///
    /// Look up the appropriate settings for your current MCLK crystal frequency
    /// in the datasheet. Some useful consts for [`ConfigParams`] are provided.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_sample_rate(
        &mut self,
        params: ConfigParams,
        lr_swap: LrSwap,
        lr_phase: LrPhase,
        mode: Mode,
        word_length: InputBitLength,
        format: DataFormat,
    ) {
        self.set_register_bits(
            Register::DigitalAudioInterfaceFormat,
            ((mode as u16) << 6)
                | ((lr_swap as u16) << 5)
                | ((lr_phase as u16) << 4)
                | ((word_length as u16) << 2)
                | (format as u16),
            0b0_0111_1111,
        );
        self.set_register_bits(
            Register::SampleRateControl,
            (if params.clk_out_div_2 { 1 << 7 } else { 0 })
                | (if params.clk_in_div_2 { 1 << 6 } else { 0 })
                | (params.src as u16) << 2
                | (params.bosr as u16) << 1
                | if params.is_usb { 1 } else { 0 },
            0b0_1111_1111,
        );
    }

    /// Turn the I²S interface on or off.
    ///
    /// Call [`Codec::sync`] to have this change take effect.
    pub fn set_digital_interface_enabled(&mut self, enabled: bool) {
        // 1 bit means on, 0 bit means off
        self.set_register_bits(
            Register::DigitalInterfaceActivation,
            if enabled { 0xFFFF } else { 0 },
            0b0_0000_0001,
        );
    }

    /// Resets the TLV230AIC23B and puts all the registers back to defaults
    pub fn reset<B>(&mut self, bus: &mut B) -> Result<(), B::Error>
    where
        B: embedded_hal::blocking::i2c::Write,
    {
        let byte1 = (Register::Reset as u8) << 1;
        let byte2 = 0;
        let buffer = [byte1, byte2];
        bus.write(self.bus_address, &buffer)?;
        self.set_register_defaults();
        Ok(())
    }

    /// Transfer all registers from this proxy object to the actual chip, over I²C.
    pub fn sync<B>(&mut self, bus: &mut B) -> Result<(), B::Error>
    where
        B: embedded_hal::blocking::i2c::Write,
    {
        for (address, (dirty, register_value)) in self.register_cache.iter_mut().enumerate() {
            if *dirty {
                let byte1 = (address << 1) as u8 | ((*register_value >> 8) & 1) as u8;
                let byte2 = (*register_value & 0xFF) as u8;
                let buffer = [byte1, byte2];
                #[cfg(feature = "defmt")]
                defmt::debug!(
                    "Setting TLV230AIC23B 0x{:02x} to 0x{:03x}",
                    address,
                    register_value
                );
                bus.write(self.bus_address, &buffer)?;
                *dirty = false;
            }
        }
        Ok(())
    }
}

//
// impls on Private Types
//

// None

//
// End of file
//
