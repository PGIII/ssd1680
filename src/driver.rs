//! Driver for interacting with SSD1680 display driver
pub use display_interface::DisplayError;

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::{delay::DelayNs, spi::SpiDevice};

#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice as AsyncSpiDevice;

use crate::interface::DisplayInterface;
#[cfg(feature = "async")]
use crate::interface::DisplayInterfaceAsync;
use crate::{cmd, color, flag, HEIGHT, WIDTH};

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

    /// Update the whole BW buffer on the display driver
    pub async fn update_bw_frame(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.use_full_frame().await?;
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_BW_DATA, buffer)
            .await
    }

    /// Update the whole Red buffer on the display driver
    pub async fn update_red_frame(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.use_full_frame().await?;
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_RED_DATA, buffer)
            .await
    }

    /// Start an update of the whole display
    pub async fn display_frame(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface
            .cmd_with_data(
                cmd::Cmd::UPDATE_DISPLAY_CTRL2,
                &[flag::Flag::DISPLAY_MODE_1],
            )
            .await?;
        self.interface.cmd(cmd::Cmd::MASTER_ACTIVATE).await?;
        self.interface.wait_until_idle(delay).await;
        Ok(())
    }

    /// Make the whole black and white frame on the display driver white
    pub async fn clear_bw_frame(&mut self) -> Result<(), DisplayError> {
        self.use_full_frame().await?;
        let color = color::Color::White.get_byte_value();
        self.interface.cmd(cmd::Cmd::WRITE_BW_DATA).await?;
        self.interface
            .data_x_times(color, u32::from(WIDTH) / 8 * u32::from(HEIGHT))
            .await
    }

    /// Make the whole red frame on the display driver white
    pub async fn clear_red_frame(&mut self) -> Result<(), DisplayError> {
        self.use_full_frame().await?;
        let color = color::Color::White.inverse().get_byte_value();
        self.interface.cmd(cmd::Cmd::WRITE_RED_DATA).await?;
        self.interface
            .data_x_times(color, u32::from(WIDTH) / 8 * u32::from(HEIGHT))
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
