pub struct Flag;
impl Flag {
    pub const DATA_ENTRY_INCRY_INCRX: u8 = 0b11;
    pub const INTERNAL_TEMP_SENSOR: u8 = 0x80;
    pub const BORDER_WAVEFORM_FOLLOW_LUT: u8 = 0b0100;
    pub const BORDER_WAVEFORM_LUT1: u8 = 0b0001;
    pub const DISPLAY_MODE_1: u8 = 0xF7;
    pub const DISPLAY_PARTIAL: u8 = 0x0F;  // partial update with custom LUT
    pub const DISPLAY_CLK_ANALOG: u8 = 0xC0; // enable clock + analog (prep before partial write)
    pub const BORDER_WAVEFORM_HIZ: u8 = 0x80; // HiZ border during partial update
    pub const DISPLAY_UPDATE_SOURCE_OUTPUT: u8 = 0x80; // byte 1 of DISPLAY_UPDATE_CONTROL (init)
    pub const DISP_OPT_RAM_PINGPONG: u8 = 0x40; // byte 5 of WRITE_DISP_OPT: enable RAM ping-pong
}
