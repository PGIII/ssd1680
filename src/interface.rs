//! Display interface using SPI
use display_interface::DisplayError;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::{delay::DelayNs, spi::SpiDevice};

#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice as AsyncSpiDevice;

const RESET_DELAY_MS: u32 = 10;

#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(
        feature = "async",
        idents(DisplayInterface(async = "DisplayInterfaceAsync"))
    )
)]
/// The Connection Interface of all (?) Waveshare EPD-Devices
///
pub(crate) struct DisplayInterface<SPI, BSY, DC, RST> {
    /// SPI device
    spi: SPI,
    /// Low for busy, Wait until display is ready!
    busy: BSY,
    /// Data/Command Control Pin (High for data, Low for command)
    dc: DC,
    /// Pin for Reseting
    rst: RST,
}

#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(
        feature = "async",
        idents(DisplayInterface(async = "DisplayInterfaceAsync"))
    )
)]
impl<SPI, BSY, DC, RST> DisplayInterface<SPI, BSY, DC, RST> {
    /// Create and initialize display
    pub fn new(spi: SPI, busy: BSY, dc: DC, rst: RST) -> Self {
        DisplayInterface { spi, busy, dc, rst }
    }
}

#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(
        feature = "async",
        idents(
            DisplayInterface(async = "DisplayInterfaceAsync"),
            SpiDevice(async = "AsyncSpiDevice"),
            DelayNs(async = "AsyncDelayNs"),
        )
    )
)]
impl<SPI, BSY, DC, RST> DisplayInterface<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    RST: OutputPin,
    DC: OutputPin,
    BSY: InputPin,
{
    /// Basic function for sending commands
    pub(crate) async fn cmd(&mut self, command: u8) -> Result<(), DisplayError> {
        self.dc.set_low().map_err(|_| DisplayError::DCError)?;
        self.spi
            .write(&[command])
            .await
            .map_err(|_| DisplayError::BusWriteError)
    }

    /// Basic function for sending an array of u8-values of data over spi
    pub(crate) async fn data(&mut self, data: &[u8]) -> Result<(), DisplayError> {
        self.dc.set_high().map_err(|_| DisplayError::DCError)?;
        self.spi
            .write(data)
            .await
            .map_err(|_| DisplayError::BusWriteError)
    }

    /// Basic function for sending a command and the data belonging to it.
    pub(crate) async fn cmd_with_data(
        &mut self,
        command: u8,
        data: &[u8],
    ) -> Result<(), DisplayError> {
        self.cmd(command).await?;
        self.data(data).await
    }

    /// Basic function for sending the same byte of data (one u8) multiple times over spi
    /// Used for setting one color for the whole frame
    pub(crate) async fn data_x_times(
        &mut self,
        val: u8,
        repetitions: u32,
    ) -> Result<(), DisplayError> {
        self.dc.set_high().map_err(|_| DisplayError::DCError)?;
        for _ in 0..repetitions {
            self.spi
                .write(&[val])
                .await
                .map_err(|_| DisplayError::BusWriteError)?;
        }
        Ok(())
    }

    /// Waits until device isn't busy anymore (busy == HIGH)
    pub(crate) async fn wait_until_idle(&mut self, delay: &mut impl DelayNs) {
        while self.busy.is_high().unwrap_or(true) {
            delay.delay_ms(1).await;
        }
    }

    /// Resets the device.
    pub(crate) async fn reset(&mut self, delay: &mut impl DelayNs) {
        self.rst.set_low().unwrap();
        delay.delay_ms(RESET_DELAY_MS).await;
        self.rst.set_high().unwrap();
        delay.delay_ms(RESET_DELAY_MS).await;
    }

    /// Short RST pulse (1 ms) to reset the ping-pong RAM counter before each partial update,
    /// without disturbing register settings or RAM contents.
    pub(crate) async fn brief_reset(&mut self, delay: &mut impl DelayNs) {
        self.rst.set_low().unwrap();
        delay.delay_ms(1).await;
        self.rst.set_high().unwrap();
    }
}
