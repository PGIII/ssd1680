//! Driver for interacting with SSD1680 display driver
pub use display_interface::DisplayError;

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::{delay::DelayNs, spi::SpiDevice};

#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice as AsyncSpiDevice;

use crate::interface::DisplayInterface;

// Waveshare 2.13" V3 partial-update waveform LUT (159 bytes, SSD1680 datasheet §6.7)
// Source: https://github.com/waveshare/e-Paper/blob/master/RaspberryPi_JetsonNano/c/lib/e-Paper/EPD_2in13_V3.c
#[rustfmt::skip]
const PARTIAL_LUT: [u8; 159] = [
    // bytes 0–59: voltage phase data (5 LUT entries × 12 bytes, verbatim from Waveshare)
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // bytes 60–143: timing rows (12 rows × 7 bytes)
    0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // phase 1: 20 frames
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // phase 2: 1 frame
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // phase 3: 1 frame
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // bytes 144–152: LUT group repeat flags
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00,
    // byte 153: LUT end option (reg 0x3F)
    0x22,
    // byte 154: gate driving voltage (reg 0x03)
    0x17,
    // bytes 155–157: source driving voltage (reg 0x04)
    0x41, 0x00, 0x32,
    // byte 158: VCOM (reg 0x2C)
    0x36,
];
#[cfg(feature = "async")]
use crate::interface::DisplayInterfaceAsync;
use crate::{cmd, flag, HEIGHT, WIDTH};

#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(
        feature = "async",
        idents(
            Ssd1680(async = "Ssd1680Async"),
            DisplayInterface(async = "DisplayInterfaceAsync"),
            SpiDevice(async = "AsyncSpiDevice"),
            DelayNs(async = "AsyncDelayNs"),
        )
    )
)]
/// A configured display with a hardware interface.
pub struct Ssd1680<SPI, BSY, DC, RST> {
    interface: DisplayInterface<SPI, BSY, DC, RST>,
}

// maybe_async_cfg handles generating the sync implementation
// so we just need to write the async version and let the macro handle the rest
// Specify keep_self so that none of the types are renamed for the synchronous version
// However for the async version we need to rename some types
#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(
        feature = "async",
        idents(
            Ssd1680(async = "Ssd1680Async"),
            DisplayInterface(async = "DisplayInterfaceAsync"),
            SpiDevice(async = "AsyncSpiDevice"),
            DelayNs(async = "AsyncDelayNs"),
        )
    )
)]
impl<SPI, BSY, DC, RST> Ssd1680<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    RST: OutputPin,
    DC: OutputPin,
    BSY: InputPin,
{
    /// Create and initialize the display driver
    pub async fn new(
        spi: SPI,
        busy: BSY,
        dc: DC,
        rst: RST,
        delay: &mut impl DelayNs,
    ) -> Result<Self, DisplayError>
    where
        Self: Sized,
    {
        let interface = DisplayInterface::new(spi, busy, dc, rst);
        let mut ssd1680 = Ssd1680 { interface };
        ssd1680.init(delay).await?;
        Ok(ssd1680)
    }

    /// Initialise the controller
    pub async fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface.reset(delay).await;
        self.interface.cmd(cmd::Cmd::SW_RESET).await?;
        self.interface.wait_until_idle(delay).await;

        self.interface
            .cmd_with_data(cmd::Cmd::DRIVER_CONTROL, &[HEIGHT - 1, 0x00, 0x00])
            .await?;

        self.interface
            .cmd_with_data(
                cmd::Cmd::DATA_ENTRY_MODE,
                &[flag::Flag::DATA_ENTRY_INCRY_INCRX],
            )
            .await?;

        self.interface
            .cmd_with_data(
                cmd::Cmd::BORDER_WAVEFORM_CONTROL,
                &[flag::Flag::BORDER_WAVEFORM_FOLLOW_LUT | flag::Flag::BORDER_WAVEFORM_LUT1],
            )
            .await?;

        self.interface
            .cmd_with_data(cmd::Cmd::TEMP_CONTROL, &[flag::Flag::INTERNAL_TEMP_SENSOR])
            .await?;

        self.interface
            .cmd_with_data(cmd::Cmd::DISPLAY_UPDATE_CONTROL, &[0x00, 0x80])
            .await?;

        self.use_full_frame().await?;

        self.interface.wait_until_idle(delay).await;
        Ok(())
    }

    /// Full refresh: write buffer to both BW and Red RAMs then trigger a full update.
    /// Call once after init (or to clear ghosting) before using display_frame().
    /// Writing to both RAMs seeds the ping-pong comparison for subsequent partial updates.
    pub async fn full_refresh(
        &mut self,
        buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        self.use_full_frame().await?;
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_BW_DATA, buffer)
            .await?;
        self.use_full_frame().await?;
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_RED_DATA, buffer)
            .await?;
        self.interface
            .cmd_with_data(cmd::Cmd::UPDATE_DISPLAY_CTRL2, &[flag::Flag::DISPLAY_MODE_1])
            .await?;
        self.interface.cmd(cmd::Cmd::MASTER_ACTIVATE).await?;
        self.interface.wait_until_idle(delay).await;
        Ok(())
    }

    /// Partial update: fast refresh using the partial waveform LUT.
    /// Sequence mirrors EPD_2in13_V3_Display_Partial from the Waveshare reference driver.
    pub async fn display_frame(
        &mut self,
        buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        // Brief RST pulse resets the ping-pong counter so BW RAM is always treated
        // as the "new" state for this update (mirrors Waveshare EPD_2in13_V3_Display_Partial)
        self.interface.brief_reset(delay).await;
        self.load_partial_lut().await?;

        // Enable RAM ping-pong (byte 5 = 0x40): controller compares BW RAM vs Red RAM
        // to determine per-pixel transition type for the LUT
        self.interface
            .cmd_with_data(
                cmd::Cmd::WRITE_DISP_OPT,
                &[0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00],
            )
            .await?;

        // Border waveform: HiZ during partial update
        self.interface
            .cmd_with_data(cmd::Cmd::BORDER_WAVEFORM_CONTROL, &[0x80])
            .await?;

        // Enable clock and analog, then wait — prepares controller to accept new image data
        self.interface
            .cmd_with_data(cmd::Cmd::UPDATE_DISPLAY_CTRL2, &[flag::Flag::DISPLAY_CLK_ANALOG])
            .await?;
        self.interface.cmd(cmd::Cmd::MASTER_ACTIVATE).await?;
        self.interface.wait_until_idle(delay).await;

        // Write new image to BW RAM (Red RAM retains the previous frame for comparison)
        self.use_full_frame().await?;
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_BW_DATA, buffer)
            .await?;

        // Trigger partial update
        self.interface
            .cmd_with_data(cmd::Cmd::UPDATE_DISPLAY_CTRL2, &[flag::Flag::DISPLAY_PARTIAL])
            .await?;
        self.interface.cmd(cmd::Cmd::MASTER_ACTIVATE).await?;
        self.interface.wait_until_idle(delay).await;

        Ok(())
    }

    async fn load_partial_lut(&mut self) -> Result<(), DisplayError> {
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_LUT, &PARTIAL_LUT[..153])
            .await?;
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_LUT_END, &PARTIAL_LUT[153..154])
            .await?;
        self.interface
            .cmd_with_data(cmd::Cmd::GATE_DRIVING_VOLTAGE, &PARTIAL_LUT[154..155])
            .await?;
        self.interface
            .cmd_with_data(cmd::Cmd::SOURCE_DRIVING_VOLTAGE, &PARTIAL_LUT[155..158])
            .await?;
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_VCOM, &PARTIAL_LUT[158..159])
            .await
    }

    async fn use_full_frame(&mut self) -> Result<(), DisplayError> {
        self.set_ram_area(0, 0, u32::from(WIDTH) - 1, u32::from(HEIGHT) - 1)
            .await?;
        self.set_ram_counter(0, 0).await
    }

    async fn set_ram_area(
        &mut self,
        start_x: u32,
        start_y: u32,
        end_x: u32,
        end_y: u32,
    ) -> Result<(), DisplayError> {
        assert!(start_x < end_x);
        assert!(start_y < end_y);

        self.interface
            .cmd_with_data(
                cmd::Cmd::SET_RAMXPOS,
                &[(start_x >> 3) as u8, (end_x >> 3) as u8],
            )
            .await?;

        self.interface
            .cmd_with_data(
                cmd::Cmd::SET_RAMYPOS,
                &[
                    start_y as u8,
                    (start_y >> 8) as u8,
                    end_y as u8,
                    (end_y >> 8) as u8,
                ],
            )
            .await
    }

    async fn set_ram_counter(&mut self, x: u32, y: u32) -> Result<(), DisplayError> {
        self.interface
            .cmd_with_data(cmd::Cmd::SET_RAMX_COUNTER, &[(x >> 3) as u8])
            .await?;
        self.interface
            .cmd_with_data(cmd::Cmd::SET_RAMY_COUNTER, &[y as u8, (y >> 8) as u8])
            .await
    }
}
