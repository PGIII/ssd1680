pub struct Cmd;
impl Cmd {
    // Init
    pub const SW_RESET: u8 = 0x12;
    pub const DRIVER_CONTROL: u8 = 0x01;
    pub const DATA_ENTRY_MODE: u8 = 0x11;
    pub const TEMP_CONTROL: u8 = 0x18;
    pub const BORDER_WAVEFORM_CONTROL: u8 = 0x3C;
    pub const DISPLAY_UPDATE_CONTROL: u8 = 0x21;
    pub const SET_RAMXPOS: u8 = 0x44;
    pub const SET_RAMYPOS: u8 = 0x45;

    // LUT / waveform
    pub const WRITE_LUT: u8 = 0x32;
    pub const WRITE_LUT_END: u8 = 0x3F;         // byte 153 of waveform setting
    pub const GATE_DRIVING_VOLTAGE: u8 = 0x03;  // byte 154
    pub const SOURCE_DRIVING_VOLTAGE: u8 = 0x04; // bytes 155-157
    pub const WRITE_VCOM: u8 = 0x2C;            // byte 158

    pub const WRITE_DISP_OPT: u8 = 0x37;    // display option / RAM ping-pong enable

    // Update
    pub const SET_RAMX_COUNTER: u8 = 0x4E;
    pub const SET_RAMY_COUNTER: u8 = 0x4F;
    pub const WRITE_BW_DATA: u8 = 0x24;
    pub const WRITE_RED_DATA: u8 = 0x26;
    pub const UPDATE_DISPLAY_CTRL2: u8 = 0x22;
    pub const MASTER_ACTIVATE: u8 = 0x20;
}