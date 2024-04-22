//! Async Driver for interacting with SSD1680 display driver
pub use display_interface::DisplayError;

use display_interface::{AsyncWriteOnlyDataCommand, DataFormat};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::delay::DelayNs;

use crate::{cmd, color, flag, HEIGHT, WIDTH};

const RESET_DELAY_MS: u8 = 10;

/// A configured display with a hardware interface.
pub struct Ssd1680Async<DI, BSY, RST> {
    interface: DI,
    /// Low for busy, Wait until display is ready!
    busy: BSY,
    /// Pin for Reseting
    rst: RST,
}

impl<DI, BSY, RST> Ssd1680Async<DI, BSY, RST>
where
    DI: AsyncWriteOnlyDataCommand,
    RST: OutputPin,
    BSY: InputPin,
{
    /// Create and initialize the display driver
    pub async fn new(
        interface: DI,
        busy: BSY,
        rst: RST,
        delay: &mut impl DelayNs,
    ) -> Result<Self, DisplayError>
    where
        Self: Sized,
    {
        let mut ssd1680 = Self {
            interface,
            busy,
            rst,
        };
        ssd1680.init(delay).await?;
        Ok(ssd1680)
    }

    /// Initialise the controller
    pub async fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.reset(delay).await;
        self.cmd(cmd::Cmd::SW_RESET).await?;
        self.wait_until_idle(delay).await;

        self.cmd_with_data(cmd::Cmd::DRIVER_CONTROL, &[HEIGHT - 1, 0x00, 0x00])
            .await?;

        self.cmd_with_data(
            cmd::Cmd::DATA_ENTRY_MODE,
            &[flag::Flag::DATA_ENTRY_INCRY_INCRX],
        )
        .await?;

        self.cmd_with_data(
            cmd::Cmd::BORDER_WAVEFORM_CONTROL,
            &[flag::Flag::BORDER_WAVEFORM_FOLLOW_LUT | flag::Flag::BORDER_WAVEFORM_LUT1],
        )
        .await?;

        self.cmd_with_data(cmd::Cmd::TEMP_CONTROL, &[flag::Flag::INTERNAL_TEMP_SENSOR])
            .await?;

        self.cmd_with_data(cmd::Cmd::DISPLAY_UPDATE_CONTROL, &[0x00, 0x80])
            .await?;

        self.use_full_frame().await?;

        self.wait_until_idle(delay).await;
        Ok(())
    }

    /// Update the whole BW buffer on the display driver
    pub async fn update_bw_frame(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.use_full_frame().await?;
        self.cmd_with_data(cmd::Cmd::WRITE_BW_DATA, &buffer).await
    }

    /// Update the whole Red buffer on the display driver
    pub async fn update_red_frame(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.use_full_frame().await?;
        self.cmd_with_data(cmd::Cmd::WRITE_RED_DATA, &buffer).await
    }

    /// Start an update of the whole display
    pub async fn display_frame(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.cmd_with_data(
            cmd::Cmd::UPDATE_DISPLAY_CTRL2,
            &[flag::Flag::DISPLAY_MODE_1],
        )
        .await?;
        self.cmd(cmd::Cmd::MASTER_ACTIVATE).await?;

        self.wait_until_idle(delay).await;

        Ok(())
    }

    /// Make the whole black and white frame on the display driver white
    pub async fn clear_bw_frame(&mut self) -> Result<(), DisplayError> {
        self.use_full_frame().await?;

        // TODO: allow non-white background color
        let color = color::Color::White.get_byte_value();

        self.cmd(cmd::Cmd::WRITE_BW_DATA).await?;
        self.data_x_times(color, u32::from(WIDTH) / 8 * u32::from(HEIGHT))
            .await?;
        Ok(())
    }

    /// Make the whole red frame on the display driver white
    pub async fn clear_red_frame(&mut self) -> Result<(), DisplayError> {
        self.use_full_frame().await?;

        // TODO: allow non-white background color
        let color = color::Color::White.inverse().get_byte_value();

        self.cmd(cmd::Cmd::WRITE_RED_DATA).await?;
        self.data_x_times(color, u32::from(WIDTH) / 8 * u32::from(HEIGHT))
            .await?;
        Ok(())
    }

    async fn use_full_frame(&mut self) -> Result<(), DisplayError> {
        // choose full frame/ram
        self.set_ram_area(0, 0, u32::from(WIDTH) - 1, u32::from(HEIGHT) - 1)
            .await?;

        // start from the beginning
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

        self.cmd_with_data(
            cmd::Cmd::SET_RAMXPOS,
            &[(start_x >> 3) as u8, (end_x >> 3) as u8],
        )
        .await?;

        self.cmd_with_data(
            cmd::Cmd::SET_RAMYPOS,
            &[
                start_y as u8,
                (start_y >> 8) as u8,
                end_y as u8,
                (end_y >> 8) as u8,
            ],
        )
        .await?;
        Ok(())
    }

    async fn set_ram_counter(&mut self, x: u32, y: u32) -> Result<(), DisplayError> {
        // x is positioned in bytes, so the last 3 bits which show the position inside a byte in the ram
        // aren't relevant
        self.cmd_with_data(cmd::Cmd::SET_RAMX_COUNTER, &[(x >> 3) as u8])
            .await?;

        // 2 Databytes: A[7:0] & 0..A[8]
        self.cmd_with_data(cmd::Cmd::SET_RAMY_COUNTER, &[y as u8, (y >> 8) as u8])
            .await?;
        Ok(())
    }

    /// Waits until device isn't busy anymore (busy == HIGH)
    async fn wait_until_idle(&mut self, delay: &mut impl DelayNs) {
        while self.busy.is_high().unwrap_or(true) {
            delay.delay_ms(1).await
        }
    }

    /// Resets the device.
    async fn reset(&mut self, delay: &mut impl DelayNs) {
        self.rst.set_low().unwrap();
        delay.delay_ms(RESET_DELAY_MS.into()).await;
        self.rst.set_high().unwrap();
        delay.delay_ms(RESET_DELAY_MS.into()).await;
    }

    /// Wrapper function around display-interface send_commands function
    async fn cmd(&mut self, command: u8) -> Result<(), DisplayError> {
        self.interface
            .send_commands(DataFormat::U8(&[command]))
            .await
    }

    /// Basic function for sending an array of u8-values of data over spi
    /// Wrapper function around display-interface send_data function
    async fn data(&mut self, data: &[u8]) -> Result<(), DisplayError> {
        self.interface.send_data(DataFormat::U8(data)).await
    }

    /// Basic function for sending a command and the data belonging to it.
    async fn cmd_with_data(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError> {
        self.cmd(command).await?;
        self.data(data).await
    }

    async fn data_x_times(&mut self, data: u8, repetitions: u32) -> Result<(), DisplayError> {
        for _ in 0..repetitions {
            self.data(&[data]).await?;
        }
        Ok(())
    }
}
