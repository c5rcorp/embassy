//! FTDI FT232x USB ↔ UART bridge host driver.
//!
//! FTDI bridges are vendor-class (no class descriptors — discovery is VID/PID
//! based) and transport UART data over one bulk IN + one bulk OUT endpoint.
//! Two quirks distinguish them from CDC-ACM and from the CP210x:
//!   - line configuration uses FTDI's private control requests addressed to the
//!     **device** recipient with `wIndex` = the 1-based port number (1 for the
//!     single-channel FT232R), matching libftdi's `INTERFACE_A` default.
//!   - every bulk-IN packet is prefixed with a 2-byte modem/line status header
//!     that must be stripped; an idle line still emits the 2-byte header every
//!     latency period, so a read can legitimately yield zero UART bytes.
//!
//! A [`FtdiDevice`] owns the device-level control pipe on endpoint 0; a
//! [`FtdiPort`] opened from it owns the bulk pipes for the data interface. The
//! split mirrors [`super::cp210x`].
//!
//! # Example
//!
//! ```rust,ignore
//! use embassy_usb_host::class::vcp::ftdi::{FtdiDevice, LineCoding, id};
//!
//! if enum_info.device_desc.vendor_id != id::VID_FTDI {
//!     continue;
//! }
//! let device = FtdiDevice::new(&bus, &enum_info)?;
//! let mut port = device.port(&config_buf[..config_len], 0)?;
//! port.reset().await?;
//! port.set_line_coding(&LineCoding::default()).await?;   // 8N1 @ 115200
//! port.set_control_line_state(true, true).await?;        // DTR + RTS high
//!
//! let mut buf = [0u8; 64];
//! let n = port.read(&mut buf).await?;
//! port.write(&buf[..n]).await?;
//! ```

use core::marker::PhantomData;

use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::mutex::Mutex;
use embassy_usb_driver::host::{PipeError, SplitInfo, UsbHostAllocator, UsbPipe, pipe};
use embassy_usb_driver::{Direction as UsbDirection, EndpointAddress, EndpointInfo, EndpointType};

use crate::control::{ControlType, Recipient, RequestType, SetupPacket};
use crate::descriptor::ConfigurationDescriptorChain;
use crate::handler::EnumerationInfo;

/// FTDI VID and common PIDs.
pub mod id {
    /// Future Technology Devices International vendor ID.
    pub const VID_FTDI: u16 = 0x0403;
    /// FT232R / FT245R.
    pub const PID_FT232R: u16 = 0x6001;
    /// FT2232C/D/L.
    pub const PID_FT2232: u16 = 0x6010;
    /// FT4232H.
    pub const PID_FT4232H: u16 = 0x6011;
    /// FT232H.
    pub const PID_FT232H: u16 = 0x6014;
    /// FT231X / FT230X.
    pub const PID_FT231X: u16 = 0x6015;
}

// FTDI vendor request codes (libftdi SIO_*).
const SIO_RESET: u8 = 0x00;
const SIO_SET_MODEM_CTRL: u8 = 0x01;
const SIO_SET_FLOW_CTRL: u8 = 0x02;
const SIO_SET_BAUD_RATE: u8 = 0x03;
const SIO_SET_DATA: u8 = 0x04;
const SIO_SET_LATENCY_TIMER: u8 = 0x09;

const VENDOR_CLASS: u8 = 0xFF;

/// Parity setting.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Parity {
    /// No parity bit.
    None = 0,
    /// Odd parity.
    Odd = 1,
    /// Even parity.
    Even = 2,
    /// Always 1.
    Mark = 3,
    /// Always 0.
    Space = 4,
}

/// Number of stop bits.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum StopBits {
    /// 1 stop bit.
    One = 0,
    /// 1.5 stop bits.
    OneAndHalf = 1,
    /// 2 stop bits.
    Two = 2,
}

/// Serial line parameters.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LineCoding {
    /// Baud rate in bits per second.
    pub baud_rate: u32,
    /// Data bits. Legal values are 7 and 8 (FTDI also allows 5/6 on some parts).
    pub data_bits: u8,
    /// Parity setting.
    pub parity: Parity,
    /// Stop bits.
    pub stop_bits: StopBits,
}

impl Default for LineCoding {
    fn default() -> Self {
        Self {
            baud_rate: 115200,
            data_bits: 8,
            parity: Parity::None,
            stop_bits: StopBits::One,
        }
    }
}

/// FTDI host driver error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FtdiError {
    /// Transfer error.
    Transfer(PipeError),
    /// No vendor-class interface at `interface_idx` with a bulk IN/OUT pair.
    NoInterface,
    /// Failed to allocate a pipe.
    NoPipe,
    /// Argument out of range for the FTDI protocol.
    InvalidArgument,
    /// The requested baud rate is not in the verified divisor table.
    UnsupportedBaud,
}

impl From<PipeError> for FtdiError {
    fn from(e: PipeError) -> Self {
        Self::Transfer(e)
    }
}

impl core::fmt::Display for FtdiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Transfer(_) => write!(f, "Transfer error"),
            Self::NoInterface => write!(f, "No FTDI interface found"),
            Self::NoPipe => write!(f, "No free pipe"),
            Self::InvalidArgument => write!(f, "Invalid argument"),
            Self::UnsupportedBaud => write!(f, "Unsupported baud rate"),
        }
    }
}

impl core::error::Error for FtdiError {}

impl embedded_io_async::Error for FtdiError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self {
            Self::Transfer(e) => match e {
                PipeError::Disconnected => embedded_io_async::ErrorKind::NotConnected,
                PipeError::BufferOverflow => embedded_io_async::ErrorKind::OutOfMemory,
                PipeError::Timeout => embedded_io_async::ErrorKind::TimedOut,
                _ => embedded_io_async::ErrorKind::Other,
            },
            Self::NoInterface => embedded_io_async::ErrorKind::NotFound,
            Self::NoPipe => embedded_io_async::ErrorKind::OutOfMemory,
            Self::InvalidArgument | Self::UnsupportedBaud => {
                embedded_io_async::ErrorKind::InvalidInput
            }
        }
    }
}

/// Encode a baud rate to the FT232R/BM divisor against the 3 MHz base clock,
/// returning `(wValue, wIndex)`.
///
/// This is a table of empirically verified rates rather than the full libftdi
/// fractional-divisor algorithm — the values below are confirmed on hardware.
/// Returns `None` for unlisted rates (callers get [`FtdiError::UnsupportedBaud`]).
fn baud_divisor(baud: u32) -> Option<(u16, u16)> {
    Some(match baud {
        9600 => (0x4138, 0x0000),
        19200 => (0x809C, 0x0000),
        38400 => (0xC04E, 0x0000),
        57600 => (0x0034, 0x0000),
        // 115200 == 115384 on the FT232R (3 MHz / 26); libftdi reports 115384.
        115200 | 115384 => (0x001A, 0x0000),
        230400 => (0x000D, 0x0000),
        _ => return None,
    })
}

/// Descriptor-located info for the FTDI data interface.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FtdiInfo {
    /// USB interface number.
    pub interface: u8,
    /// Bulk IN endpoint address.
    pub bulk_in_ep: u8,
    /// Bulk IN max packet size.
    pub bulk_in_mps: u16,
    /// Bulk OUT endpoint address.
    pub bulk_out_ep: u8,
    /// Bulk OUT max packet size.
    pub bulk_out_mps: u16,
}

/// Return the `interface_idx`-th (0-indexed) vendor-class interface in
/// `config_desc` that exposes a bulk IN + bulk OUT endpoint pair. Use
/// `interface_idx = 0` for single-channel parts (FT232R); `0..N` for multi-port
/// parts (FT2232/FT4232).
pub fn find_ftdi(config_desc: &[u8], interface_idx: u8) -> Option<FtdiInfo> {
    let cfg = ConfigurationDescriptorChain::try_from_slice(config_desc).ok()?;

    let mut seen = 0u8;
    for iface in cfg.iter_interface() {
        if iface.interface_class != VENDOR_CLASS || iface.alternate_setting != 0 {
            continue;
        }
        let mut in_ep = None;
        let mut out_ep = None;
        for ep in iface.iter_endpoints() {
            if ep.ep_type() != EndpointType::Bulk {
                continue;
            }
            if ep.is_in() {
                in_ep = Some((ep.endpoint_address, ep.max_packet_size));
            } else {
                out_ep = Some((ep.endpoint_address, ep.max_packet_size));
            }
        }
        if let (Some((in_a, in_m)), Some((out_a, out_m))) = (in_ep, out_ep) {
            if seen == interface_idx {
                return Some(FtdiInfo {
                    interface: iface.interface_number,
                    bulk_in_ep: in_a,
                    bulk_in_mps: in_m,
                    bulk_out_ep: out_a,
                    bulk_out_mps: out_m,
                });
            }
            seen += 1;
        }
    }
    None
}

/// FTDI device — owns the shared control pipe on endpoint 0.
///
/// Open one [`FtdiPort`] per channel via [`FtdiDevice::port`]. Construct with
/// [`FtdiDevice::new`] for the common single-task case (the control-pipe mutex
/// is a [`NoopRawMutex`]); use [`FtdiDevice::new_with_raw_mutex`] with a `Sync`
/// raw mutex to drive multiple channels concurrently.
pub struct FtdiDevice<'d, A, M = NoopRawMutex>
where
    A: UsbHostAllocator<'d>,
    M: RawMutex,
{
    alloc: A,
    ctrl: Mutex<M, A::Pipe<pipe::Control, pipe::InOut>>,
    device_address: u8,
    split: Option<SplitInfo>,
    _phantom: PhantomData<&'d ()>,
}

impl<'d, A> FtdiDevice<'d, A, NoopRawMutex>
where
    A: UsbHostAllocator<'d>,
{
    /// Allocate the device-level control pipe on endpoint 0, using a
    /// [`NoopRawMutex`]. Performs no I/O.
    pub fn new(alloc: &A, enum_info: &EnumerationInfo) -> Result<Self, FtdiError> {
        Self::new_with_raw_mutex(alloc, enum_info)
    }
}

impl<'d, A, M> FtdiDevice<'d, A, M>
where
    A: UsbHostAllocator<'d>,
    M: RawMutex,
{
    /// Allocate the device-level control pipe on endpoint 0 using the
    /// caller-chosen raw mutex `M`. Performs no I/O.
    pub fn new_with_raw_mutex(alloc: &A, enum_info: &EnumerationInfo) -> Result<Self, FtdiError> {
        let ctrl_ep_info = EndpointInfo {
            addr: EndpointAddress::from_parts(0, UsbDirection::In),
            ep_type: EndpointType::Control,
            max_packet_size: enum_info.device_desc.max_packet_size0 as u16,
            interval_ms: 0,
        };
        let device_address = enum_info.device_address;
        let split = enum_info.split();
        let ctrl = alloc
            .alloc_pipe::<pipe::Control, pipe::InOut>(device_address, &ctrl_ep_info, split)
            .map_err(|_| FtdiError::NoPipe)?;
        Ok(Self {
            alloc: alloc.clone(),
            ctrl: Mutex::new(ctrl),
            device_address,
            split,
            _phantom: PhantomData,
        })
    }

    /// Open the `interface_idx`-th UART channel (use `0` for single-channel
    /// parts). Allocates the channel's bulk pipes; performs no I/O. The caller
    /// configures the line (reset / line coding / modem lines) before transfer.
    pub fn port<'dev>(
        &'dev self,
        config_desc: &[u8],
        interface_idx: u8,
    ) -> Result<FtdiPort<'dev, 'd, A, M>, FtdiError> {
        let info = find_ftdi(config_desc, interface_idx).ok_or(FtdiError::NoInterface)?;

        let in_ep_info = EndpointInfo {
            addr: EndpointAddress::from_parts((info.bulk_in_ep & 0x0F) as usize, UsbDirection::In),
            ep_type: EndpointType::Bulk,
            max_packet_size: info.bulk_in_mps,
            interval_ms: 0,
        };
        let out_ep_info = EndpointInfo {
            addr: EndpointAddress::from_parts((info.bulk_out_ep & 0x0F) as usize, UsbDirection::Out),
            ep_type: EndpointType::Bulk,
            max_packet_size: info.bulk_out_mps,
            interval_ms: 0,
        };

        let in_ch = self
            .alloc
            .alloc_pipe::<pipe::Bulk, pipe::In>(self.device_address, &in_ep_info, self.split)
            .map_err(|_| FtdiError::NoPipe)?;
        let out_ch = self
            .alloc
            .alloc_pipe::<pipe::Bulk, pipe::Out>(self.device_address, &out_ep_info, self.split)
            .map_err(|_| FtdiError::NoPipe)?;

        Ok(FtdiPort {
            device: self,
            in_ch,
            out_ch,
            // FTDI addresses control requests to the 1-based port number
            // (libftdi INTERFACE_A == 1 for the first/only channel).
            port_index: interface_idx as u16 + 1,
            in_mps: info.bulk_in_mps,
        })
    }
}

/// A single UART channel on an [`FtdiDevice`].
pub struct FtdiPort<'dev, 'd, A, M = NoopRawMutex>
where
    A: UsbHostAllocator<'d>,
    M: RawMutex,
{
    device: &'dev FtdiDevice<'d, A, M>,
    in_ch: A::Pipe<pipe::Bulk, pipe::In>,
    out_ch: A::Pipe<pipe::Bulk, pipe::Out>,
    port_index: u16,
    in_mps: u16,
}

impl<'dev, 'd, A, M> FtdiPort<'dev, 'd, A, M>
where
    A: UsbHostAllocator<'d>,
    M: RawMutex,
{
    async fn vendor_out(&mut self, request: u8, value: u16, index: u16) -> Result<(), FtdiError> {
        let setup = SetupPacket {
            request_type: RequestType {
                direction: UsbDirection::Out,
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
            },
            request,
            value,
            index,
            length: 0,
        };
        let mut ctrl = self.device.ctrl.lock().await;
        ctrl.control_out(&setup.to_bytes(), &[]).await?;
        Ok(())
    }

    /// Reset the FTDI SIO (purges and resets the chip's serial engine).
    pub async fn reset(&mut self) -> Result<(), FtdiError> {
        self.vendor_out(SIO_RESET, 0, self.port_index).await
    }

    /// Program the UART baud rate (verified-table rates; see [`baud_divisor`]).
    pub async fn set_baud_rate(&mut self, baud: u32) -> Result<(), FtdiError> {
        let (value, index) = baud_divisor(baud).ok_or(FtdiError::UnsupportedBaud)?;
        // The divisor high half (index) ORs with the port for multi-channel
        // parts; for the rates in the table it is zero, leaving just the port.
        self.vendor_out(SIO_SET_BAUD_RATE, value, index | self.port_index)
            .await
    }

    /// Program data bits, parity and stop bits.
    pub async fn set_data(&mut self, coding: &LineCoding) -> Result<(), FtdiError> {
        if !matches!(coding.data_bits, 5..=8) {
            return Err(FtdiError::InvalidArgument);
        }
        let value = (coding.data_bits as u16)
            | ((coding.parity as u16) << 8)
            | ((coding.stop_bits as u16) << 11);
        self.vendor_out(SIO_SET_DATA, value, self.port_index).await
    }

    /// Program data/parity/stop bits and the baud rate.
    pub async fn set_line_coding(&mut self, coding: &LineCoding) -> Result<(), FtdiError> {
        self.set_data(coding).await?;
        self.set_baud_rate(coding.baud_rate).await
    }

    /// Disable all flow control (no RTS/CTS, no XON/XOFF).
    pub async fn set_flow_control_off(&mut self) -> Result<(), FtdiError> {
        // libftdi: wIndex = (flowctrl >> 8) | port; disable (0) leaves the port.
        self.vendor_out(SIO_SET_FLOW_CTRL, 0, self.port_index).await
    }

    /// Drive DTR and RTS to the given levels.
    pub async fn set_control_line_state(&mut self, dtr: bool, rts: bool) -> Result<(), FtdiError> {
        // High byte is the "set this line" mask; low byte is the level.
        let value = (1 << 8) | (1 << 9) | (dtr as u16) | ((rts as u16) << 1);
        self.vendor_out(SIO_SET_MODEM_CTRL, value, self.port_index).await
    }

    /// Set the FTDI read latency timer in milliseconds (1..=255).
    pub async fn set_latency(&mut self, ms: u8) -> Result<(), FtdiError> {
        self.vendor_out(SIO_SET_LATENCY_TIMER, ms as u16, self.port_index)
            .await
    }

    /// Read up to one bulk-IN packet of UART data, stripping FTDI's 2-byte
    /// modem/line status header. Returns 0 when only the status header arrived
    /// (an idle line still emits a 2-byte poll every latency period).
    ///
    /// # Cancellation
    ///
    /// Not cancel-safe: bytes received from the device but not yet copied into
    /// `buf` are lost if the future is dropped.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, FtdiError> {
        let mut packet = [0u8; 64];
        let cap = (self.in_mps as usize).min(packet.len());
        let n = self.in_ch.request_in(&mut packet[..cap]).await?;
        if n <= 2 {
            return Ok(0);
        }
        let data = &packet[2..n];
        let take = data.len().min(buf.len());
        buf[..take].copy_from_slice(&data[..take]);
        Ok(take)
    }

    /// Write bytes to the UART transmit stream.
    ///
    /// # Cancellation
    ///
    /// Not cancel-safe: the remote may observe partial data if the future is
    /// dropped mid-transfer.
    pub async fn write(&mut self, data: &[u8]) -> Result<usize, FtdiError> {
        self.out_ch.request_out(data, false).await?;
        Ok(data.len())
    }
}

impl<'dev, 'd, A, M> embedded_io_async::ErrorType for FtdiPort<'dev, 'd, A, M>
where
    A: UsbHostAllocator<'d>,
    M: RawMutex,
{
    type Error = FtdiError;
}

impl<'dev, 'd, A, M> embedded_io_async::Read for FtdiPort<'dev, 'd, A, M>
where
    A: UsbHostAllocator<'d>,
    M: RawMutex,
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        FtdiPort::read(self, buf).await
    }
}

impl<'dev, 'd, A, M> embedded_io_async::Write for FtdiPort<'dev, 'd, A, M>
where
    A: UsbHostAllocator<'d>,
    M: RawMutex,
{
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        FtdiPort::write(self, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
