//! USB CCID (Chip Card Interface Device) class implementation.
extern crate alloc;
use alloc::boxed::Box;
use alloc::sync::Arc;
use core::convert::{TryFrom, TryInto};
use core::mem::MaybeUninit;
use core::ops::Range;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{select3, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::signal::Signal;
use heapless::Vec;

use crate::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use crate::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use crate::types::InterfaceNumber;
use crate::{Builder, Handler};

/// USB class code for CCID devices.
pub const USB_CLASS_CCID: u8 = 0x0B;
/// USB subclass code for CCID devices.
pub const USB_SUBCLASS_NONE: u8 = 0x00;
/// USB protocol code for CCID devices.
pub const USB_PROTOCOL_NONE: u8 = 0x00;
/// Functional Interface String Descriptor (optional, but some hosts require it for CCID devices).
pub const FUNCTIONAL_INTERFACE_STRING: &str = "CCID/ICCD Interface";

/// Maximum packet size for CCID endpoints (as per USB spec, must be 64 for full-speed devices).
pub const PACKET_SIZE: usize = 64;

/// As per CCID spec, the header of each CCID message is 10 bytes long.
pub const CCID_HEADER_LEN: usize = 10;

/// error codes for slot status and parameters responses, as per CCID spec for FAIL responses
const CCID_CMD_FAIL: u8 = 1 << 6;
/// ISO 7816-4 "no precise diagnosis", sent when the application's response cannot be delivered.
const SW_NO_PRECISE_DIAGNOSIS: [u8; 2] = [0x6F, 0x00];

/// raw packet
pub type RawPacket = heapless::Vec<u8, PACKET_SIZE>;

/// Application-level packet, which may be larger than a single USB packet and may require chaining.
pub type ApplicationPacket = Box<[u8]>;

/// A request from the CCID reader to the application.
///
/// The application answers every request with an [`ApplicationPacket`] holding the card's
/// response APDU (data and status word), which the reader returns as RDR_to_PC_DataBlock.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CcidRequest {
    /// A command APDU from PC_to_RDR_XfrBlock, to be sent to the card unchanged.
    Apdu(ApplicationPacket),
    /// A PIN verification from PC_to_RDR_Secure. The application obtains the PIN itself, builds the
    /// VERIFY command with [`PinVerify::build_apdu`] and sends it to the card.
    VerifyPin(PinVerify),
}

/// extended packet for commands that exceed the size of a single USB packet, e.g. APDUs with chaining
pub type ExtPacket = heapless::Vec<u8, MAX_MSG_LENGTH>;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[allow(dead_code, clippy::enum_variant_names)]
/// State of the CCID reader state machine, as per CCID spec.
enum PipeError {
    CmdAborted = 0xff,
    IccMute = 0xfe,
    XfrParityError = 0xfd,
    //..
    CmdSlotBusy = 0xE0,
    // Offset of the offending field in the command: dwLength.
    BadLength = 0x01,
    // Offset of the offending field in the command: bSlot (slot does not exist).
    BadSlot = 0x05,
    CommandNotSupported = 0x00,
}

/// One half of the CCID ABORT handshake: the slot and sequence number being aborted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct AbortRequest {
    slot: u8,
    seq: u8,
}

/// Signal used by the control handler to pass a class-specific ABORT request to the reader.
pub(crate) type AbortSignal = Signal<CriticalSectionRawMutex, AbortRequest>;

#[derive(Clone, Debug, PartialEq, Eq)]
/// CCID Response type
pub enum ResponseType {
    /// Send to the usb endpoint directly
    Internal(ExtPacket),
    /// Send to the application to handle. Holds a complete APDU, which may be assembled from
    /// several chained CCID messages and so can exceed `MAX_MSG_LENGTH` (bounded by
    /// `Config::max_apdu_size`).
    External(CcidRequest),
    /// No response needed, e.g. for ABORT commands
    None,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
/// Bulk Out request codes (bRequest values for class-specific Bulk Out requests).
pub enum BulkInMessage {
    /// PRDR_to_PC_DataBlock
    DataBlock,
    /// PRDR_to_PC_SlotStatus
    SlotStatus,
    /// PRDR_to_PC_Parameters
    Parameters,
    /// PRDR_to_PC_Escape
    Escape,
    /// PRDR_to_PC_DataRateAndClockFrequency
    DataRateAndClockFrequency,
}

impl core::convert::TryFrom<u8> for BulkInMessage {
    type Error = ();
    fn try_from(request: u8) -> core::result::Result<Self, ()> {
        Ok(match request {
            0x80 => Self::DataBlock,
            0x81 => Self::SlotStatus,
            0x82 => Self::Parameters,
            0x83 => Self::Escape,
            0x84 => Self::DataRateAndClockFrequency,
            _ => return Err(()),
        })
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
/// Class-specific requests (bRequest values for class-specific control requests).
pub enum ClassRequest {
    /// As per CCID spec: "slot in low, seq in high byte"
    Abort = 1,
    /// As per CCID spec: returns a list of supported clock frequencies in kHz as 4-byte little-endian values.
    GetClockFrequencies = 2,
    /// As per CCID spec: returns a list of supported data rates in bps as 4-byte little-endian values.
    GetDataRates = 3,
}

impl core::convert::TryFrom<u8> for ClassRequest {
    type Error = ();
    fn try_from(request: u8) -> core::result::Result<Self, ()> {
        Ok(match request {
            1 => Self::Abort,
            2 => Self::GetClockFrequencies,
            3 => Self::GetDataRates,
            _ => return Err(()),
        })
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
/// dwFeatures
pub enum Features {
    /// 00000000h No special characteristics
    NoSpecialCharacteristics = 0x00000000,
    /// 00000002h Automatic parameter configuration based on ATR data
    AutoParamConfig = 0x00000002,
    /// 00000004h Automatic activation of ICC on inserting
    AutoActivation = 0x00000004,
    /// 00000008h Automatic ICC voltage selection
    AutoVoltage = 0x00000008,
    /// 00000010h Automatic ICC clock frequency change according to active parameters provided by the Host or self determined 3
    AutoClockChange = 0x00000010,
    /// 00000020h 3 Automatic baud rate change according to active parameters provided by the Host or self determined
    AutoBaudRateChange = 0x00000020,
    /// 00000040h 4 Automatic parameters negotiation made by the CCID (use of warm or cold resets or PPS according to a manufacturer proprietary algorithm to select the communication parameters with the ICC)
    AutoParamNegotiation = 0x00000040,
    /// 00000080h 4 Automatic PPS made by the CCID according to the active parameters
    AutoPPS = 0x00000080,
    /// 00000100h CCID can set ICC in clock stop mode
    ClockStop = 0x00000100,
    /// 00000200h NAD value other than 00 accepted (T=1 protocol in use)
    NadAccepted = 0x00000200,
    /// 00000400h Automatic IFSD exchange as first exchange (T=1 protocol in use)
    AutoIFSD = 0x00000400,
    /// 00010000h TPDU level exchanges with CCID
    TPDULevel = 0x00010000,
    /// 00020000h Short APDU level exchange with CCID
    ShortAPDULevel = 0x00020000,
    /// 00040000h Short and Extended APDU level exchange with CCID
    ShortExtendedAPDULevel = 0x00040000,
}

// CCID
/// bLength
pub const CCID_DESC_BLENGTH: u8 = 0x36;
/// bDescriptorType
pub const CCID_DESC_DESCTYPE_CCID: u8 = 0x21;
/// bcdCCID
pub const CCID_DESC_SPEC_1_10: [u8; 2] = [0x10, 0x01];
/// bMaxSlotIndex
pub const CCID_DESC_MAX_SLOT_INDEX: u8 = 0x00;
/// bVoltageSupport (5.0V)
pub const CCID_DESC_VOLTAGE_5V: u8 = 0x01;
/// dwProtocols: APDU level, T=1 only (0 = T=0, 3 = T0+T1)
pub const CCID_DESC_PROTOCOL_T1: [u8; 4] = [0x02, 0x00, 0x00, 0x00];
/// 3580 KHz (as per ICCD spec) = 3.58 MHz
/// (not relevant, fixed fixed for legacy reasons)
pub const CCID_DESC_CLOCK_FREQUENCY_KHZ: [u8; 4] = [0xfc, 0x0d, 0x00, 0x00];
/// bNumClockSupported
pub const CCID_DESC_NUM_CLOCK_SUPPORTED: u8 = 0x00;
/// 9600 bps (as per ICCD spec)
/// (not relevant, fixed fixed for legacy reasons)
pub const CCID_DESC_DATA_RATE_BPS: [u8; 4] = [0x80, 0x25, 0x00, 0x00];
/// bNumDataRatesSupported
pub const CCID_DESC_NUM_DATA_RATES_SUPPORTED: u8 = 0x00;
/// dwMaxIFSD: 254, the maximum T=1 IFSD
pub const CCID_DESC_MAX_IFSD: [u8; 4] = 254u32.to_le_bytes();
/// dwSyncProtocols: none
pub const CCID_DESC_SYNC_PROTOCOLS: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
/// dwMechanical: no special characteristics
pub const CCID_DESC_MECHANICAL: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
/// dwFeatures
pub const CCID_DESC_FEATURES: [u8; 4] = (Features::AutoActivation as u32
    | Features::AutoParamConfig as u32
    | Features::ShortAPDULevel as u32
    | Features::AutoVoltage as u32
    | Features::AutoClockChange as u32
    | Features::AutoBaudRateChange as u32
    | Features::AutoParamNegotiation as u32
    | Features::AutoIFSD as u32)
    .to_le_bytes();
//pub const CCID_DESC_FEATURES: [u8; 4] = [0x40, 0x08, 0x04, 0x00];
/// dwMaxCCIDMsgLen 3072
pub const MAX_MSG_LENGTH: usize = 271;
/// "The value shall be between 261 + 10 and 65544 + 10
pub const CCID_DESC_MAX_MSG_LENGTH_LE: [u8; 4] = (MAX_MSG_LENGTH as u32).to_le_bytes();
/// bClassGetResponse ("echo"), as per ICCD spec
pub const CCID_DESC_CLASS_GET_RESPONSE: u8 = 0xFF;
/// bClassEnvelope ("echo"), as per ICCD spec, gnuk: 0
pub const CCID_DESC_CLASS_ENVELOPE: u8 = 0xFF;
/// wlcdLayout (none)
pub const CCID_DESC_LCD_LAYOUT: [u8; 2] = [0x00, 0x00];
/// bPinSupport (0x0 = none, 0x01 = verification, 0x02 = modification)
///
/// Set bit 0 (here or in a custom `Config::ccid_descriptor`) to accept PIN verification with
/// PC_to_RDR_Secure; the application must then handle [`CcidRequest::VerifyPin`]. While it is
/// clear, PC_to_RDR_Secure is rejected as unsupported.
pub const CCID_DESC_PIN_SUPPORT: u8 = 0x00;
/// Offset of bPinSupport in the CCID class descriptor.
const CCID_DESC_PIN_SUPPORT_OFFSET: usize = 52;
/// bPinSupport bit: PIN verification.
const PIN_SUPPORT_VERIFY: u8 = 0x01;
/// bMaxCCIDBusySlots
pub const CCID_DESC_MAX_BUSY_SLOTS: u8 = 1;

/// Default CCID class-specific descriptor, including bLength and bDescriptorType.
pub const DEFAULT_CCID_DESCRIPTOR: [u8; CCID_DESC_BLENGTH as usize] = [
    // bLength
    CCID_DESC_BLENGTH,
    // bDescriptorType
    CCID_DESC_DESCTYPE_CCID,
    // bcdCCID
    CCID_DESC_SPEC_1_10[0],
    CCID_DESC_SPEC_1_10[1],
    // bMaxSlotIndex
    CCID_DESC_MAX_SLOT_INDEX,
    // bVoltageSupport
    CCID_DESC_VOLTAGE_5V,
    // dwProtocols: APDU level, T=1 only (0 = T=0, 3 = T0+T1)
    CCID_DESC_PROTOCOL_T1[0],
    CCID_DESC_PROTOCOL_T1[1],
    CCID_DESC_PROTOCOL_T1[2],
    CCID_DESC_PROTOCOL_T1[3],
    // dwDefaultClock (3.58 MHz)
    CCID_DESC_CLOCK_FREQUENCY_KHZ[0],
    CCID_DESC_CLOCK_FREQUENCY_KHZ[1],
    CCID_DESC_CLOCK_FREQUENCY_KHZ[2],
    CCID_DESC_CLOCK_FREQUENCY_KHZ[3],
    // dwMaximumClock (same)
    CCID_DESC_CLOCK_FREQUENCY_KHZ[0],
    CCID_DESC_CLOCK_FREQUENCY_KHZ[1],
    CCID_DESC_CLOCK_FREQUENCY_KHZ[2],
    CCID_DESC_CLOCK_FREQUENCY_KHZ[3],
    // bNumClockSupported
    CCID_DESC_NUM_CLOCK_SUPPORTED,
    // dwDataRate (9600 bps)
    CCID_DESC_DATA_RATE_BPS[0],
    CCID_DESC_DATA_RATE_BPS[1],
    CCID_DESC_DATA_RATE_BPS[2],
    CCID_DESC_DATA_RATE_BPS[3],
    // dwMaxDataRate (same)
    CCID_DESC_DATA_RATE_BPS[0],
    CCID_DESC_DATA_RATE_BPS[1],
    CCID_DESC_DATA_RATE_BPS[2],
    CCID_DESC_DATA_RATE_BPS[3],
    // bNumDataRatesSupported
    CCID_DESC_NUM_DATA_RATES_SUPPORTED,
    // dwMaxIFSD (254)
    CCID_DESC_MAX_IFSD[0],
    CCID_DESC_MAX_IFSD[1],
    CCID_DESC_MAX_IFSD[2],
    CCID_DESC_MAX_IFSD[3],
    // dwSyncProtocols: none
    CCID_DESC_SYNC_PROTOCOLS[0],
    CCID_DESC_SYNC_PROTOCOLS[1],
    CCID_DESC_SYNC_PROTOCOLS[2],
    CCID_DESC_SYNC_PROTOCOLS[3],
    // dwMechanical: no special characteristics
    CCID_DESC_MECHANICAL[0],
    CCID_DESC_MECHANICAL[1],
    CCID_DESC_MECHANICAL[2],
    CCID_DESC_MECHANICAL[3],
    // dwFeatures, see following comments
    // Auto configuration based on ATR
    // Auto activation on insert
    // Auto voltage selection
    // Auto clock change
    // Auto baud rate change
    // Auto parameter negotiation made by CCID
    // Short APDU level exchange
    // 0xFE, 0x00, 0x02, 0x00,
    // ICCD: lower word (=0840): only requests valid for USB-ICC
    // upper word: 0000 = char level, 0002 = short APDU, 0004 = short+exteded APDU
    CCID_DESC_FEATURES[0],
    CCID_DESC_FEATURES[1],
    CCID_DESC_FEATURES[2],
    CCID_DESC_FEATURES[3],
    // dwMaxCCIDMsgLen (3072)
    // gnuk: 271
    CCID_DESC_MAX_MSG_LENGTH_LE[0],
    CCID_DESC_MAX_MSG_LENGTH_LE[1],
    CCID_DESC_MAX_MSG_LENGTH_LE[2],
    CCID_DESC_MAX_MSG_LENGTH_LE[3],
    // bClassGetResponse ("echo"), as per ICCD spec
    CCID_DESC_CLASS_GET_RESPONSE,
    // bClassEnvelope ("echo"), as per ICCD spec, gnuk: 0
    CCID_DESC_CLASS_ENVELOPE,
    // wlcdLayout (none)
    CCID_DESC_LCD_LAYOUT[0],
    CCID_DESC_LCD_LAYOUT[1],
    // bPinSupport
    CCID_DESC_PIN_SUPPORT,
    // bMaxCCIDBusySlots
    CCID_DESC_MAX_BUSY_SLOTS,
];

// T=1 protocol parameters reported by GetParameters. These must agree with the ATR built by
// `construct_t1_atr`, which uses the same constants for TC1, TA3 and TB3.

/// bmFindexDindex: Fi=372, Di=1. The ATR has no TA1, so the implicit default 0x11 applies.
pub const DEFAULT_FIDI: u8 = 0x11;
/// bmTCCKST1: LRC checksum (bit 0 = 0), direct convention (bit 1 = 0, TS = 0x3B);
/// bits 7..2 are fixed at 000100b by the CCID spec.
pub const DEFAULT_TCCKST1: u8 = 0x10;
/// bGuardTimeT1: extra guard time from TC1 (none).
pub const DEFAULT_EXTRA_GUARDTIME: u8 = 0x00;
/// bWaitingIntegersT1: from TB3, BWI = 1 (high nibble), CWI = 5 (low nibble).
pub const DEFAULT_WAITING_INTEGERS_T1: u8 = 0x15;
/// bClockStop: clock stop not supported.
pub const DEFAULT_CLOCKSTOP: u8 = 0x00;
/// bIFSC: information field size from TA3 (254).
pub const DEFAULT_IFSC: u8 = 0xFE;
/// bNadValue: NAD not used.
pub const DEFAULT_NAD: u8 = 0x00;

/// Configuration for the CCID reader/writer.
pub struct Config<'d> {
    /// Complete CCID class-specific descriptor (54 bytes, including bLength and
    /// bDescriptorType). It is used both in the configuration descriptor and for
    /// GET_DESCRIPTOR requests. An empty slice selects [`DEFAULT_CCID_DESCRIPTOR`].
    pub ccid_descriptor: &'d [u8],

    /// Max packet size for the bulk IN endpoint.
    pub max_packet_size_in: u16,

    /// Max packet size for the bulk OUT endpoint.
    pub max_packet_size_out: u16,

    /// Configures how frequently the host should poll for reading/writing HID reports.
    ///
    /// A lower value means better throughput & latency, at the expense
    /// of CPU on the device & bandwidth on the bus. A value of 10 is reasonable for
    /// high performance uses, and a value of 255 is good for best-effort usecases.
    pub poll_ms: u8,

    /// Maximum size, in bytes, of an APDU assembled from chained XfrBlock fragments.
    ///
    /// The assembled APDU is heap-allocated, so this bounds how much memory a host can make the
    /// driver allocate. Chains that would exceed it are rejected and the reader is reset.
    pub max_apdu_size: usize,
}

/// Report ID
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReportId {
    /// IN report
    BulkIn(u8),
    /// OUT report
    BulkOut(u8),
    /// Feature report
    Interrupt(u8),
}

/// State for the CCID reader/writer.
pub struct State {
    control: MaybeUninit<Control>,
}

impl<'d> Default for State {
    fn default() -> Self {
        Self::new()
    }
}

impl State {
    /// Creates a new `State` with an uninitialized control handler.
    pub const fn new() -> Self {
        State {
            control: MaybeUninit::uninit(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// T=1 protocol data structure (CCID spec, RDR_to_PC_Parameters), returned in response to the
/// GetParameters command. The host uses it to determine the active protocol parameters.
pub struct ProtocolData {
    bm_findex_dindex: u8,
    bm_tcckst1: u8,
    b_guard_time_t1: u8,
    b_waiting_integers_t1: u8,
    b_clock_stop: u8,
    b_ifsc: u8,
    b_nad: u8,
}

impl Default for ProtocolData {
    fn default() -> Self {
        Self {
            bm_findex_dindex: DEFAULT_FIDI,
            bm_tcckst1: DEFAULT_TCCKST1,
            b_guard_time_t1: DEFAULT_EXTRA_GUARDTIME,
            b_waiting_integers_t1: DEFAULT_WAITING_INTEGERS_T1,
            b_clock_stop: DEFAULT_CLOCKSTOP,
            b_ifsc: DEFAULT_IFSC,
            b_nad: DEFAULT_NAD,
        }
    }
}
/// USB CCID reader/writer.
///
/// `READ_N` and `WRITE_N` are the capacities, in messages, of the application channels
/// (`app_to_ccid` and `ccid_to_app`). They do not limit USB packet or CCID message sizes,
/// which are bounded by [`PACKET_SIZE`] and [`MAX_MSG_LENGTH`].
pub struct CcidReaderWriter<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize> {
    bulk_out: CcidBulkOut<'d, D>,
    bulk_in: CcidBulkIn<'d, D>,
    int_in: CcidIntIn<'d, D>,

    protocol_data: ProtocolData,

    ccid_to_app: &'static mut Sender<'static, CriticalSectionRawMutex, CcidRequest, WRITE_N>,

    slot_status: u8,

    seq: u8,
    slot: u8,
    state: CcidReaderState,
    sent: usize,
    outbox: Option<ApplicationPacket>,
    ext_packet: ExtPacket,
    #[allow(dead_code)]
    packet_len: usize,
    receiving_long: bool,
    long_packet_missing: usize,
    in_chain: usize,
    started_processing: bool,
    // bSeq of the XfrBlock whose APDU is currently with the application. Kept separate from
    // `seq`, which every incoming command overwrites, so the application's response is labelled
    // with the sequence number of the command it answers.
    pending_seq: Option<u8>,
    // Limit on the size of an APDU assembled from chained fragments (Config::max_apdu_size).
    max_apdu_size: usize,
    // Set by the control handler on USB bus reset; the reader then discards all state from the
    // previous session before handling more traffic.
    session_reset: Arc<AtomicBool>,
    atr: Vec<u8, 32>,
    // Bulk-pipe half of a pending ABORT handshake (PC_to_RDR_Abort received).
    bulk_abort: Option<AbortRequest>,
    // Control-pipe half of a pending ABORT handshake (class request ABORT received).
    control_abort: Option<AbortRequest>,
    // Control-pipe ABORT requests, signalled by the control handler.
    abort_signal: Arc<AbortSignal>,
    // The descriptor advertises PIN verification (bPinSupport bit 0), so PC_to_RDR_Secure is accepted.
    pin_verify_supported: bool,
    // An ABORT cancelled a command that was already with the application. Its response is
    // dropped when it arrives, and new XfrBlocks are rejected as busy until then, so it can't be
    // mistaken for the response to a later command.
    discard_app_response: bool,
}

fn build<'d, D: Driver<'d>>(
    builder: &mut Builder<'d, D>,
    state: &'d mut State,
    config: Config<'d>,
) -> (
    D::EndpointOut,
    D::EndpointIn,
    D::EndpointIn,
    Arc<AtomicBool>,
    Arc<AbortSignal>,
    bool,
) {
    let mut func = builder.function(USB_CLASS_CCID, USB_SUBCLASS_NONE, USB_PROTOCOL_NONE);
    let mut iface = func.interface();
    let if_num = iface.interface_number();
    let mut alt = iface.alt_setting(USB_CLASS_CCID, USB_SUBCLASS_NONE, USB_PROTOCOL_NONE, None);

    let ccid_descriptor: [u8; CCID_DESC_BLENGTH as usize] = if config.ccid_descriptor.is_empty() {
        DEFAULT_CCID_DESCRIPTOR
    } else {
        config
            .ccid_descriptor
            .try_into()
            .expect("CCID descriptor must be exactly 54 bytes")
    };
    assert!(
        ccid_descriptor[0] == CCID_DESC_BLENGTH && ccid_descriptor[1] == CCID_DESC_DESCTYPE_CCID,
        "CCID descriptor must start with bLength 0x36 and bDescriptorType 0x21"
    );

    // The builder writes bLength and bDescriptorType itself.
    alt.descriptor(CCID_DESC_DESCTYPE_CCID, &ccid_descriptor[2..]);

    // CCID -> HOST
    let ep_in = alt.endpoint_bulk_in(config.max_packet_size_in);
    // HOST -> CCID
    let ep_out = alt.endpoint_bulk_out(config.max_packet_size_out);
    // CCID -> HOST
    let ep_int_in = alt.endpoint_interrupt_in(config.max_packet_size_in, config.poll_ms);

    drop(func);

    let session_reset = Arc::new(AtomicBool::new(false));
    let abort_signal = Arc::new(AbortSignal::new());
    let control = state.control.write(Control::new(
        if_num,
        ccid_descriptor,
        session_reset.clone(),
        abort_signal.clone(),
    ));

    builder.handler(control);

    // PC_to_RDR_Secure is only accepted when the descriptor advertises PIN verification.
    let pin_verify_supported = ccid_descriptor[CCID_DESC_PIN_SUPPORT_OFFSET] & PIN_SUPPORT_VERIFY != 0;

    (
        ep_out,
        ep_in,
        ep_int_in,
        session_reset,
        abort_signal,
        pin_verify_supported,
    )
}

impl<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize> CcidReaderWriter<'d, D, READ_N, WRITE_N> {
    /// Creates a new `CcidReaderWriter`.
    ///
    /// This will allocate one IN and one OUT endpoints. If you only need writing (sending)
    /// CCID reports, consider using [`CcidWriter::new`] instead, which allocates an IN endpoint only.
    ///
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State,
        config: Config<'d>,
        ccid_to_app: &'static mut Sender<'static, CriticalSectionRawMutex, CcidRequest, WRITE_N>,
    ) -> Self {
        let max_apdu_size = config.max_apdu_size;
        let (ep_out, ep_in, ep_int_in, session_reset, abort_signal, pin_verify_supported) =
            build(builder, state, config);

        Self {
            bulk_out: CcidBulkOut { ep_out },
            bulk_in: CcidBulkIn { ep_in },
            int_in: CcidIntIn { ep_int_in },
            slot_status: 0x00, // An ICC is present and active
            seq: 0,
            slot: 0,
            state: CcidReaderState::Idle,
            sent: 0,
            outbox: None,
            ext_packet: Default::default(),
            packet_len: 0,
            receiving_long: false,
            long_packet_missing: 0,
            in_chain: 0,
            started_processing: false,
            pending_seq: None,
            max_apdu_size,
            session_reset,
            atr: Self::construct_t1_atr(),
            bulk_abort: None,
            control_abort: None,
            abort_signal,
            pin_verify_supported,
            discard_app_response: false,
            ccid_to_app,
            protocol_data: ProtocolData::default(),
        }
    }

    /// Constrcts T1 ATR
    fn construct_t1_atr() -> Vec<u8, 32> {
        /*
        TS: Initial Character (0x3B = direct, 0x3F = inverse convention)
        T0: Format Character: high nibble = Y1 = bitfield indicating which of TA1, TB1, TC1, TD1 are present, low nibble = K = number of historical bytes
        Interface bytes (TA1, TB1, TC1, TD1): optional, presence indicated by Y1 in T0, Each TDn’s lower nibble selects a protocol (e.g., 0x01 means T=1) and upper nibble shows which next interface bytes follow.
        Historical Bytes: Vendor info, ATR text, etc.
        TCK: Check character, present if any protocol other than T=0 is used. T=1 requires TCK

        TS=0x3B (direct convention)
        T0=
        TA1=None
        TB1=0x00 (VPP is not electrically connected)
        TC1=0x00 (no extra guard time)
        TD1=0x81 (TD2 included, T=1 protocol)
        TA2=None
        TB2=None
        TC2=None
        TD2=0x31 (TD3 included, T=1 protocol)
        TA3=0xFE (Information Field Size Integer IFSI 254)
        TB3=0x15 (Block Waiting Integer: 1 - Character Waiting Integer: 5)
        TC3=None
        Historycal bytes="Token"
        TCK=
        */
        let historical_bytes = b"Token"; // 5 bytes
        let k = historical_bytes.len() as u8 + 1; // +1 for 0x59

        let mut atr: Vec<u8, 32> = Vec::new();

        // TS: direct convention
        atr.push(0x3B).ok();

        // T0:
        // Y1 = TB1 | TC1 | TD1 present = 0xE0
        // K = number of historical bytes
        let t0 = 0xE0 | k;
        atr.push(t0).ok();

        // ---- Interface bytes group 1 ----
        atr.push(0x00).ok(); // TB1 (VPP not connected)
        atr.push(DEFAULT_EXTRA_GUARDTIME).ok(); // TC1 (no extra guard time)
        atr.push(0x81).ok(); // TD1 (T=1, TD2 follows)

        // ---- Interface bytes group 2 ----
        atr.push(0x31).ok(); // TD2 (T=1, TA3 and TB3 follow)

        // ---- Interface bytes group 3 ----
        atr.push(DEFAULT_IFSC).ok(); // TA3 (IFSC = 254)
        atr.push(DEFAULT_WAITING_INTEGERS_T1).ok(); // TB3 (BWI=1, CWI=5)

        // ---- Add in 0x59 category byte ----
        atr.push(0x59).ok();

        // Historical bytes
        atr.extend_from_slice(historical_bytes).ok();

        // ---- TCK ----
        // XOR of everything from T0 through last historical byte
        let mut tck: u8 = 0;
        for byte in atr.iter().skip(1) {
            tck ^= *byte;
        }
        atr.push(tck).ok();

        trace!("CCID: Constructed ATR: {=[u8]:x}", &atr);

        atr
    }

    /// Splits into the bulk OUT reader, the bulk IN writer, and the interrupt IN writer
    /// (used for RDR_to_PC_NotifySlotChange notifications).
    pub fn split(self) -> (CcidBulkOut<'d, D>, CcidBulkIn<'d, D>, CcidIntIn<'d, D>) {
        (self.bulk_out, self.bulk_in, self.int_in)
    }

    /// Waits for both IN and OUT endpoints to be enabled.
    pub async fn ready(&mut self) {
        self.bulk_out.ready().await;
        self.bulk_in.ready().await;
        self.int_in.ready().await;
    }

    /// Writes `report` to its Bulk endpoint.
    pub async fn write(&mut self, report: &[u8]) -> Result<(), EndpointError> {
        self.bulk_in.write(report).await
    }

    /// Writes an interrupt report to the host, notifying it of a slot change or other event.
    pub async fn write_interrupt(&mut self, status_code: bool) -> Result<(), EndpointError> {
        let data = self.rdr_to_pc_notify_slot_change(status_code).await;
        trace!("CCID: Sending interrupt report to host: {=[u8]:x}", &data);
        self.int_in.write(&data).await
    }

    /// Reads an output report from the Bulk Out pipe.
    ///
    /// See [`CcidReader::read`].
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<ResponseType, ReadError> {
        match self.bulk_out.read(buf).await {
            Ok(len) => {
                trace!("CCID: Received packet from host: {=[u8]:x}", &buf);
                self.handle_packet(RawPacket::from_slice(&buf[..len]).map_err(|_| ReadError::BufferOverflow)?)
                    .await
            }
            Err(e) => {
                warn!("CCID: Failed to read packet from host: {:?}", e);
                Err(e)
            }
        }
    }

    /// Main loop of the CCID reader/writer, which continuously reads from the host and handles packets, while also listening for packets from the application to send to the host.
    pub async fn run(
        mut self,
        app_to_ccid: &'static mut Receiver<'static, CriticalSectionRawMutex, ApplicationPacket, READ_N>,
    ) -> ! {
        let mut buf = [0u8; PACKET_SIZE];

        self.ready().await;

        if let Err(_e) = self.write_interrupt(true).await {
            error!("CCID: Failed to write initial interrupt report to host: {:?}", _e);
        }

        loop {
            // A bus reset may have happened while we were writing to the host.
            if self.session_reset.swap(false, Ordering::AcqRel) {
                self.reset_session(app_to_ccid);
            }

            let abort_signal = self.abort_signal.clone();
            match select3(app_to_ccid.receive(), self.read(&mut buf), abort_signal.wait()).await {
                Either3::First(raw_packet) => {
                    if self.session_reset.swap(false, Ordering::AcqRel) {
                        // This answers a command from the previous session; the host won't expect it.
                        trace!("CCID: Dropping application response from before USB reset");
                        drop(raw_packet);
                        self.reset_session(app_to_ccid);
                        continue;
                    }
                    if self.discard_app_response {
                        // This answers a command the host has aborted.
                        trace!("CCID: Dropping application response to aborted command");
                        self.discard_app_response = false;
                        continue;
                    }

                    trace!(
                        "CCID: Received packet from application to send to host: {=[u8]:x}",
                        &raw_packet
                    );

                    // Answer the XfrBlock that was dispatched to the application, not whichever
                    // command arrived most recently.
                    let response_seq = match self.pending_seq.take() {
                        Some(seq) => seq,
                        None => {
                            warn!("CCID: Application response without a pending XfrBlock");
                            self.seq
                        }
                    };

                    // The reader advertises short APDU level, where a response is always a single unchained
                    // RDR_to_PC_DataBlock (the bulk endpoint splits it across USB packets). Responses longer than
                    // 256 data bytes + SW are returned by the card as 61XX for the host to fetch with GET RESPONSE,
                    // so anything that does not fit one message is an application error.
                    let response: &[u8] = if raw_packet.len() <= MAX_MSG_LENGTH - CCID_HEADER_LEN {
                        &raw_packet
                    } else {
                        error!(
                            "CCID: Application response of {} bytes exceeds the {} byte maximum",
                            raw_packet.len(),
                            MAX_MSG_LENGTH - CCID_HEADER_LEN
                        );
                        &SW_NO_PRECISE_DIAGNOSIS
                    };

                    // Wrap in a PRDR_to_PC_DataBlock response and send to host
                    let data: ExtPacket = DataBlock::new(response_seq, Chain::BeginsAndEnds, response).into();
                    trace!("CCID: Sending response packet to host: {=[u8]:x}", &data);

                    if let Err(_e) = self.write(&data).await {
                        warn!("CCID: Failed to write response packet: {:?}", _e);
                    }
                    self.state = CcidReaderState::Idle;
                }
                Either3::Second(Ok(response_type)) => match response_type {
                    ResponseType::Internal(packet) => {
                        trace!(
                            "CCID: Received packet from host to send back to host: {=[u8]:x}",
                            &packet
                        );

                        if let Err(_e) = self.write(&packet).await {
                            warn!("CCID: Failed to write response packet: {:?}", _e);
                        }
                    }
                    ResponseType::External(request) => {
                        match &request {
                            CcidRequest::Apdu(apdu) => {
                                trace!(
                                    "CCID: Received packet from host to send to application: {=[u8]:x}",
                                    apdu
                                )
                            }
                            CcidRequest::VerifyPin(_) => {
                                trace!("CCID: Sending PIN verification request to application")
                            }
                        }
                        self.ccid_to_app.send(request).await;
                        trace!("CCID: Sent packet to application");
                    }
                    ResponseType::None => {
                        trace!("CCID: Nothing to handle");
                    }
                },
                Either3::Second(Err(e)) => {
                    warn!("CCID: Failed to read packet from host: {:?}", e);
                    // Disabled means the endpoint went away (bus reset or deconfiguration), which
                    // ends the session even if no reset was signalled.
                    if e == ReadError::Disabled || self.session_reset.swap(false, Ordering::AcqRel) {
                        self.reset_session(app_to_ccid);
                    }
                }
                Either3::Third(request) => {
                    trace!("CCID: Control ABORT for slot {} seq {}", request.slot, request.seq);
                    self.control_abort = Some(request);
                    // The bulk half may already have arrived and be waiting for this one.
                    if self.bulk_abort == Some(request) {
                        let packet = self.abort(request);
                        if let Err(_e) = self.write(&packet).await {
                            warn!("CCID: Failed to write ABORT response: {:?}", _e);
                        }
                    }
                }
            }
        }
    }

    /// Handles a received USB packet, updating the internal state of the CCID reader accordingly.
    pub async fn handle_packet(&mut self, packet: RawPacket) -> Result<ResponseType, ReadError> {
        // SHOULD CLEAN THIS UP!
        // The situation is as follows: full 64B USB packet received.
        // CCID packet signals no command chaining, but data length > 64 - 10.
        // Then we can expect to receive more USB packets containing only data.
        // The concatenation of all these is then a valid Command APDU.
        // (which itself may have command chaining on a higher level, e.g.
        // when certificates are transmitted, because PIV somehow uses short APDUs
        // only (can we fix this), so 255B is the maximum)
        if !self.receiving_long {
            if packet.len() < CCID_HEADER_LEN {
                error!("CCID: unexpected short packet");
                self.reset_state();
                return Ok(ResponseType::None);
            }
            self.ext_packet.clear();
            self.ext_packet
                .extend_from_slice(&packet)
                .expect("Raw packets are not larger than ext packets");

            // Compare the bytes actually received against the header's dwLength; the first
            // packet may be shorter than PACKET_SIZE (e.g. a smaller endpoint max packet size).
            let pl = packet.data_len();
            let message_len = CCID_HEADER_LEN.saturating_add(pl);
            if message_len > self.ext_packet.capacity() {
                error!(
                    "CCID: Message length {} exceeds the maximum of {}",
                    message_len,
                    self.ext_packet.capacity()
                );
                self.reset_state();
                return Ok(ResponseType::None);
            }
            if packet.len() < message_len {
                self.receiving_long = true;
                self.in_chain = 1;
                self.long_packet_missing = message_len - packet.len();
                self.packet_len = pl;
                trace!(
                    "CCID: Received first packet of long message, pl {}, missing {}, in_chain {}",
                    pl,
                    self.long_packet_missing,
                    self.in_chain
                );
                return Ok(ResponseType::None);
            }
        } else {
            if self.ext_packet.extend_from_slice(&packet).is_err() {
                error!(
                    "CCID: Extended packet got larger than maximum size ({}), wants {}",
                    self.ext_packet.capacity(),
                    self.ext_packet.len() + packet.len(),
                );
                self.reset_state();
                return Ok(ResponseType::None);
            }
            self.in_chain += 1;
            if packet.len() > self.long_packet_missing {
                error!("CCID: Got larger packet than expected");
                self.long_packet_missing = 0;
            } else {
                self.long_packet_missing -= packet.len();
            }
            if self.long_packet_missing != 0 {
                return Ok(ResponseType::None);
            }

            trace!(
                "CCID: pl {}, p {}, missing {}, in_chain {}",
                self.packet_len,
                packet.len(),
                self.long_packet_missing,
                self.in_chain
            );

            self.receiving_long = false;
        }

        trace!("CCID: Received full packet: {=[u8]:x}", &self.ext_packet);
        match Command::try_from(self.ext_packet.clone()) {
            Ok(command) => {
                self.seq = command.seq();
                self.slot = command.slot();

                // ABORT is a two-part handshake (control request + PC_to_RDR_Abort) that may arrive
                // in either order. Only complete it when both halves name the same slot and seq.
                if matches!(command, Command::Abort(_)) {
                    let request = AbortRequest {
                        slot: self.slot,
                        seq: self.seq,
                    };
                    self.bulk_abort = Some(request);
                    if self.control_abort == Some(request) {
                        return Ok(ResponseType::Internal(self.abort(request)));
                    }
                    trace!(
                        "CCID: Bulk ABORT for slot {} seq {}, waiting for control ABORT",
                        request.slot,
                        request.seq
                    );
                    return Ok(ResponseType::None);
                }

                // Once the control pipe has announced an ABORT, reject all other commands until
                // the matching bulk ABORT arrives.
                if let Some(_control_abort) = self.control_abort {
                    trace!(
                        "CCID: Received command while waiting for bulk abort with seq {}, rejecting",
                        _control_abort.seq
                    );
                    let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN);
                    packet[0] = 0x81; // bMessageType: RDR_to_PC_SlotStatus
                    packet[6] = self.seq;
                    packet[7] = CCID_CMD_FAIL;
                    packet[8] = PipeError::CmdAborted as u8;
                    return Ok(ResponseType::Internal(packet));
                }

                self.bulk_abort = None;

                match command {
                    Command::IccPowerOn(_command) => {
                        self.slot_status = 0x00; // An ICC is present and active
                        trace!("CCID: IccPowerOn command received, sending ATR response");
                        let atr = self.atr.clone();
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_data_block(&atr, Chain::BeginsAndEnds).await,
                        ))
                    }
                    Command::IccPowerOff(_command) => {
                        trace!("CCID: IccPowerOff command received");
                        self.slot_status = 0x01; // An ICC is present and inactive
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_slot_status(self.slot_status, 0x00).await,
                        ))
                    }
                    Command::GetSlotStatus(_command) => {
                        trace!("CCID: GetSlotStatus command received");
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_slot_status(self.slot_status, 0x00).await,
                        ))
                    }
                    Command::XfrBlock(command) => {
                        trace!("CCID: XfrBlock command received, data: {=[u8]:x}", command.data());
                        self.handle_xfer(command).await
                    }
                    // Handled above, before the abort-pending check.
                    Command::Abort(_command) => Ok(ResponseType::None),
                    Command::GetParameters(_command) => {
                        trace!("CCID: GetParameters command received");
                        Ok(ResponseType::Internal(self.rdr_to_pc_parameters(0x0, 0x0).await))
                    }
                    Command::ResetParameters(_reset_parameters) => {
                        trace!(
                            "CCID: ResetParameters command received, data: {=[u8]:x}",
                            _reset_parameters
                        );
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_parameters(CCID_CMD_FAIL, PipeError::CommandNotSupported as u8)
                                .await,
                        ))
                    }
                    Command::SetParameters(_set_parameters) => {
                        trace!("CCID: SetParameters command received, data: {=[u8]:x}", _set_parameters);
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_parameters(CCID_CMD_FAIL, PipeError::CommandNotSupported as u8)
                                .await,
                        ))
                    }
                    Command::Escape(_escape) => {
                        trace!("CCID: Escape command received, data: {=[u8]:x}", _escape);
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_escape(CCID_CMD_FAIL, PipeError::CommandNotSupported as u8, &[0x00; 4])
                                .await,
                        ))
                    }
                    Command::IccClock(_icc_clock) => {
                        trace!("CCID: IccClock command received, data: {=[u8]:x}", _icc_clock);
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_slot_status(CCID_CMD_FAIL, PipeError::CommandNotSupported as u8)
                                .await,
                        ))
                    }
                    Command::T0APDU(_t0_apdu) => {
                        trace!("CCID: T0APDU command received, data: {=[u8]:x}", _t0_apdu);
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_slot_status(CCID_CMD_FAIL, PipeError::CommandNotSupported as u8)
                                .await,
                        ))
                    }
                    Command::Secure(command) => {
                        trace!("CCID: Secure command received, data: {=[u8]:x}", command.data());
                        Ok(self.handle_secure(command))
                    }
                    Command::Mechanical(_mechanical) => {
                        trace!("CCID: Mechanical command received, data: {=[u8]:x}", _mechanical);
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_slot_status(CCID_CMD_FAIL, PipeError::CommandNotSupported as u8)
                                .await,
                        ))
                    }
                    Command::SetDataRateAndClockFrequency(_set_data_rate_and_clock_frequency) => {
                        trace!(
                            "CCID: SetDataRateAndClockFrequency command received, data: {=[u8]:x}",
                            _set_data_rate_and_clock_frequency
                        );
                        Ok(ResponseType::Internal(
                            self.rdr_to_pc_data_rate_and_clock_frequency(
                                CCID_CMD_FAIL,
                                PipeError::CommandNotSupported as u8,
                            )
                            .await,
                        ))
                    }
                }
            }
            Err(PacketError::ShortPacket) => {
                error!("CCID: Unexpectedly short packet");
                self.reset_state();
                let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN);
                packet[0] = 0x81; // bMessageType: RDR_to_PC_SlotStatus
                packet[6] = self.seq;
                packet[7] = CCID_CMD_FAIL;
                packet[8] = PipeError::CommandNotSupported as u8;
                Ok(ResponseType::Internal(packet))
            }
            Err(PacketError::UnknownCommand(_p)) => {
                info!("CCID: Unknown command {:?}", &_p);
                self.seq = self.ext_packet[6];
                let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN);
                packet[0] = 0x81; // bMessageType: RDR_to_PC_SlotStatus
                packet[6] = self.seq;
                packet[7] = CCID_CMD_FAIL;
                packet[8] = PipeError::CommandNotSupported as u8;
                Ok(ResponseType::Internal(packet))
            }
            Err(PacketError::TruncatedPayload) => {
                error!("CCID: dwLength exceeds the received payload, rejecting command");
                let seq = self.ext_packet[6];
                let slot = self.ext_packet[5];
                self.reset_state();
                let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN);
                packet[0] = 0x81; // bMessageType: RDR_to_PC_SlotStatus
                packet[5] = slot;
                packet[6] = seq;
                packet[7] = CCID_CMD_FAIL;
                packet[8] = PipeError::BadLength as u8;
                Ok(ResponseType::Internal(packet))
            }
            Err(PacketError::WrongSlot(slot)) => {
                warn!("CCID: Command for nonexistent slot {}", slot);
                let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN);
                packet[0] = 0x81; // bMessageType: RDR_to_PC_SlotStatus
                packet[5] = slot;
                packet[6] = self.ext_packet[6];
                // bmCommandStatus = failed, bmICCStatus = no ICC present
                packet[7] = CCID_CMD_FAIL | 0x02;
                packet[8] = PipeError::BadSlot as u8;
                Ok(ResponseType::Internal(packet))
            }
        }
    }

    /// Ends the current USB session: clears message assembly, command chaining and the pending
    /// response, and drops application responses already queued for the previous session.
    ///
    /// A response the application sends later for a pre-reset command cannot be told apart from a
    /// new-session response, so the application must stop answering on reset (halo does, via
    /// `USB_CCID_ROUTING_ENDED`).
    fn reset_session(&mut self, app_to_ccid: &Receiver<'static, CriticalSectionRawMutex, ApplicationPacket, READ_N>) {
        self.reset_state();
        self.discard_app_response = false;
        self.abort_signal.reset();
        let mut _dropped = 0;
        while app_to_ccid.try_receive().is_ok() {
            _dropped += 1;
        }
        trace!(
            "CCID: USB session reset, _dropped {} queued application response(s)",
            _dropped
        );
    }

    /// Reset the state of the CCID driver
    ///
    /// This is done on unexpected input instead of panicking
    pub fn reset_state(&mut self) {
        self.seq = 0;
        self.state = CcidReaderState::Idle;
        self.sent = 0;
        self.outbox = None;
        self.packet_len = 0;
        self.receiving_long = false;
        self.long_packet_missing = 0;
        self.in_chain = 0;
        self.started_processing = false;
        self.pending_seq = None;
        self.bulk_abort = None;
        self.control_abort = None;
    }

    // Completes an ABORT. Only call this once matching ABORT requests have arrived on both the
    // control pipe and the bulk endpoint.
    fn abort(&mut self, request: AbortRequest) -> ExtPacket {
        trace!("CCID: Aborting slot {} seq {}", request.slot, request.seq);
        // A command that is already with the application can't be recalled; drop its response
        // when it arrives instead of sending it.
        if self.pending_seq.is_some() {
            self.discard_app_response = true;
        }
        // Cancels message assembly, command chaining, a partly sent response and both abort halves.
        self.reset_state();

        // RDR_to_PC_SlotStatus for the bulk ABORT, reporting success
        let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN);
        packet[0] = 0x81;
        packet[5] = request.slot;
        packet[6] = request.seq;
        packet[7] = self.slot_status;
        packet
    }

    /// Appends a command-chain fragment to the APDU being assembled in `outbox`.
    ///
    /// Checks `max_apdu_size` and reserves memory fallibly before growing the buffer, so a host
    /// cannot exhaust the heap with an unbounded chain. Returns `false` if the fragment was
    /// rejected.
    fn append_chain_fragment(&mut self, fragment: &[u8]) -> bool {
        let mut apdu = self.outbox.take().map(|apdu| apdu.into_vec()).unwrap_or_default();
        let fits = apdu
            .len()
            .checked_add(fragment.len())
            .is_some_and(|len| len <= self.max_apdu_size);
        if !fits {
            error!(
                "CCID: Chained APDU exceeds the maximum of {} bytes ({} + {})",
                self.max_apdu_size,
                apdu.len(),
                fragment.len()
            );
            return false;
        }
        if apdu.try_reserve_exact(fragment.len()).is_err() {
            error!(
                "CCID: Out of memory assembling a {} byte chained APDU",
                apdu.len() + fragment.len()
            );
            return false;
        }
        apdu.extend_from_slice(fragment);
        self.outbox = Some(apdu.into_boxed_slice());
        true
    }

    /// Handles PC_to_RDR_Secure. Only PIN verification is supported: the request is passed to the
    /// application, which supplies the PIN, and its response is returned like an XfrBlock's.
    fn handle_secure(&mut self, command: Secure) -> ResponseType {
        if !self.pin_verify_supported {
            trace!("CCID: Secure command received but PIN verification is not advertised, rejecting");
            return ResponseType::Internal(self.failed_data_block(PipeError::CommandNotSupported));
        }
        if self.discard_app_response || self.state != CcidReaderState::Idle {
            warn!("CCID: Secure command while another command is in progress, rejecting");
            return ResponseType::Internal(self.failed_data_block(PipeError::CmdSlotBusy));
        }
        // wLevelParameter: PIN requests split over several messages are not supported.
        if command[8..10] != [0, 0] {
            warn!("CCID: Chained Secure command not supported");
            return ResponseType::Internal(self.failed_data_block(PipeError::CommandNotSupported));
        }
        match PinVerify::parse(command.data()) {
            Ok(verify) => {
                self.state = CcidReaderState::Processing;
                self.pending_seq = Some(self.seq);
                ResponseType::External(CcidRequest::VerifyPin(verify))
            }
            Err(_e) => {
                warn!("CCID: Unsupported PIN request: {:?}", _e);
                ResponseType::Internal(self.failed_data_block(PipeError::CommandNotSupported))
            }
        }
    }

    /// Builds a failed RDR_to_PC_DataBlock for the current XfrBlock.
    fn failed_data_block(&self, error: PipeError) -> ExtPacket {
        let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN);
        packet[0] = 0x80; // bMessageType: RDR_to_PC_DataBlock
        packet[5] = self.slot;
        packet[6] = self.seq;
        packet[7] = CCID_CMD_FAIL;
        packet[8] = error as u8;
        packet
    }

    /// Resets the reader after a rejected command chain and builds a failed
    /// RDR_to_PC_DataBlock for the current XfrBlock.
    fn reject_chain(&mut self) -> ResponseType {
        let packet = self.failed_data_block(PipeError::BadLength);
        self.reset_state();
        ResponseType::Internal(packet)
    }

    async fn handle_xfer(&mut self, command: XfrBlock) -> Result<ResponseType, ReadError> {
        if self.discard_app_response {
            // The application is still finishing an aborted command; its response must be
            // drained before a new APDU is dispatched.
            warn!("CCID: XfrBlock while an aborted command is still with the application, rejecting");
            return Ok(ResponseType::Internal(self.failed_data_block(PipeError::CmdSlotBusy)));
        }

        // Decode once; an unsupported wLevelParameter is Err and must reach the reset paths below.
        let chain = command.chain();
        trace!("Current state: {:?}, command chain: {:?}", self.state, chain.ok());

        match self.state {
            CcidReaderState::Idle => match chain {
                Ok(Chain::BeginsAndEnds) => {
                    trace!("CCID: Received XfrBlock with no chaining, processing immediately");
                    self.state = CcidReaderState::Processing;
                    self.pending_seq = Some(self.seq);

                    Ok(ResponseType::External(CcidRequest::Apdu(
                        command.data().to_vec().into_boxed_slice(),
                    )))
                }
                Ok(Chain::Begins) => {
                    trace!("CCID: Received XfrBlock with chaining, waiting for more packets");

                    // The fragment can carry up to MAX_MSG_LENGTH - CCID_HEADER_LEN bytes, so it is
                    // copied straight into the boxed buffer rather than through a USB-packet-sized RawPacket.
                    if !self.append_chain_fragment(command.data()) {
                        return Ok(self.reject_chain());
                    }

                    self.state = CcidReaderState::Receiving;
                    Ok(ResponseType::Internal(
                        self.rdr_to_pc_data_block(&[], Chain::ExpectingMore).await,
                    ))
                }
                Err(_) => {
                    error!("Unknown chain");
                    self.reset_state();
                    Ok(ResponseType::None)
                }
                _ => {
                    error!("unexpectedly in idle state");
                    self.reset_state();
                    Ok(ResponseType::None)
                }
            },
            CcidReaderState::Receiving => match chain {
                Ok(Chain::Continues) => {
                    trace!("CCID: Received XfrBlock with chaining, waiting for more packets");

                    if self.outbox.is_none() {
                        error!("Received chained packet but outbox is None");
                        self.reset_state();
                        return Ok(ResponseType::None);
                    }
                    if !self.append_chain_fragment(command.data()) {
                        return Ok(self.reject_chain());
                    }

                    Ok(ResponseType::Internal(
                        self.rdr_to_pc_data_block(&[], Chain::ExpectingMore).await,
                    ))
                }
                Ok(Chain::Ends) => {
                    trace!("CCID: Received last XfrBlock in chain, processing full message");

                    if self.outbox.is_some() {
                        if !self.append_chain_fragment(command.data()) {
                            return Ok(self.reject_chain());
                        }

                        let full_message = self.outbox.take().unwrap_or_default();
                        self.state = CcidReaderState::Processing;
                        self.pending_seq = Some(self.seq);

                        Ok(ResponseType::External(CcidRequest::Apdu(full_message)))
                    } else {
                        error!("Received chained packet but outbox is None");
                        self.reset_state();
                        Ok(ResponseType::None)
                    }
                }
                Err(_) => {
                    error!("Unknown chain");
                    self.reset_state();
                    Ok(ResponseType::None)
                }
                _ => {
                    error!("unexpectedly in idle state");
                    self.reset_state();
                    Ok(ResponseType::None)
                }
            },
            CcidReaderState::Processing | CcidReaderState::ReadyToSend => {
                error!("Received XfrBlock while already processing another, rejecting");
                self.reset_state();
                Ok(ResponseType::None)
            }
            CcidReaderState::Sending => match chain {
                Ok(Chain::ExpectingMore) => {
                    trace!(
                        "CCID: Received XfrBlock while sending, expecting more packets: Sent {} bytes so far",
                        self.sent
                    );
                    // get next block f outbox and prime it for sending
                    if let Some(outbox) = &self.outbox {
                        trace!(
                            "CCID: Outbox has {} bytes, sent {}, remaining {}",
                            outbox.len(),
                            self.sent,
                            outbox.len() - self.sent
                        );
                        let chunk_size = core::cmp::min(PACKET_SIZE - CCID_HEADER_LEN, outbox.len() - self.sent);
                        let chunk = &outbox[self.sent..][..chunk_size];
                        self.sent += chunk_size;
                        let has_more = self.sent < outbox.len();
                        let chain = if has_more { Chain::Continues } else { Chain::Ends };
                        let data = self.rdr_to_pc_data_block(&chunk.to_vec(), chain).await;
                        trace!("CCID: Sending chained response packet to host: {=[u8]:x}", &data);
                        if chain == Chain::Ends {
                            self.reset_state();
                        }
                        Ok(ResponseType::Internal(
                            ExtPacket::from_slice(&data).map_err(|_| ReadError::BufferOverflow)?,
                        ))
                    } else {
                        error!("Received chained packet but outbox is None");
                        self.reset_state();
                        Ok(ResponseType::None)
                    }
                }
                _chain => {
                    error!("unexpectedly in receiving state and got chain");
                    self.reset_state();
                    Ok(ResponseType::None)
                }
            },
        }
    }

    /// builds the RDR_to_PC_NotifySlotChange interrupt endpoint message
    async fn rdr_to_pc_notify_slot_change(&mut self, init: bool) -> ExtPacket {
        // Per CCID spec, 2 bits per slot: LSB = present, MSB = changed
        // Hardcode: slot 0 present+changed, slot 1 present+changed, rest 0
        // Bits: 0=slot0 present, 1=slot0 changed, 2=slot1 present, 3=slot1 changed
        let mut packet = ExtPacket::zeroed_until(2); // Only need 2 bytes for up to 4 slots
        packet[0] = 0x50; // bMessageType
        if init {
            match self.slot {
                0 => packet[1] = 0b11,      // slot 0 present+changed
                1 => packet[1] = 0b11 << 2, // slot 1 present+changed
                2 => packet[1] = 0b11 << 4, // slot 2 present+changed
                _ => packet[1] = 0,         // no slots
            }
        } else {
            match self.slot {
                0 => packet[1] = 0b01,      // slot 0 present+changed
                1 => packet[1] = 0b01 << 2, // slot 1 present+changed
                2 => packet[1] = 0b01 << 4, // slot 2 present+changed
                _ => packet[1] = 0,         // no slots
            }
        }
        packet
    }

    /// builds a DataRateAndClockFrequency response with the given status and error codes.
    async fn rdr_to_pc_data_rate_and_clock_frequency(&mut self, status_code: u8, error_code: u8) -> ExtPacket {
        // Header plus dwClockFrequency and dwDataRate
        let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN + 8);
        // bMessageType
        packet[0] = 0x84;
        // dwLength of data = 8 bytes
        packet[1] = 8;
        // bSlot
        packet[5] = self.slot;
        // bSeq
        packet[6] = self.seq;
        // bStatus
        packet[7] = status_code;
        // bError
        packet[8] = error_code;
        // dwClockFrequency (fixed for legacy reasons)
        packet[10..14].copy_from_slice(&CCID_DESC_CLOCK_FREQUENCY_KHZ);
        // dwDataRate (fixed for legacy reasons)
        packet[14..18].copy_from_slice(&CCID_DESC_DATA_RATE_BPS);
        packet
    }

    /// builds an Escape response with the given status and error codes, and the first 4 bytes of data.
    async fn rdr_to_pc_escape(&mut self, status_code: u8, error_code: u8, data: &[u8; 4]) -> ExtPacket {
        // Header plus the 4 abData bytes declared in dwLength
        let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN + 4);
        // bMessageType
        packet[0] = 0x83;
        // dwLength
        packet[1] = 4;
        // bSlot
        packet[5] = self.slot;
        // bSeq
        packet[6] = self.seq;
        // bStatus: ICC present and active
        packet[7] = status_code;
        // bError
        packet[8] = error_code;
        // bRFU
        packet[9] = 0x00;
        // abData
        packet[10..14].copy_from_slice(&data[..4]);
        packet
    }

    /// builds a DataBlock response with the given data and chain status.
    async fn rdr_to_pc_data_block(&mut self, data: &[u8], chain: Chain) -> ExtPacket {
        let packet = DataBlock::new(self.seq, chain, &data);
        packet.into()
    }

    /// builds a SlotStatus response with the given status and error codes.
    async fn rdr_to_pc_slot_status(&mut self, status_code: u8, error_code: u8) -> ExtPacket {
        let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN);
        // bMessageType
        packet[0] = 0x81;
        // dwLength
        packet[1] = 0;
        // bSlot
        packet[5] = self.slot;
        // bSeq
        packet[6] = self.seq;
        // bStatus: ICC present and active
        packet[7] = status_code;
        // bError
        packet[8] = error_code;
        // bClockStatus, Clock running
        packet[9] = 0x00;
        packet
    }

    /// builds a Parameters response with the current protocol parameters.
    async fn rdr_to_pc_parameters(&mut self, status_code: u8, error_code: u8) -> ExtPacket {
        let mut packet = ExtPacket::zeroed_until(17);
        // bMessageType
        packet[0] = 0x82;
        // dwLength of data = 7 bytes
        packet[1] = 7;
        // bSlot
        packet[5] = self.slot;
        // bSeq
        packet[6] = self.seq;
        // bStatus
        packet[7] = status_code;
        // bError
        packet[8] = error_code;
        // bProtocolNum, T=1 only
        packet[9] = 1;
        // bmFindexDindex
        packet[10] = self.protocol_data.bm_findex_dindex;
        // bmTCCKST1
        packet[11] = self.protocol_data.bm_tcckst1;
        // bGuardTimeT1
        packet[12] = self.protocol_data.b_guard_time_t1;
        // bmWaitingIntegersT1
        packet[13] = self.protocol_data.b_waiting_integers_t1;
        // bClockStop
        packet[14] = self.protocol_data.b_clock_stop;
        // bIFSC
        packet[15] = self.protocol_data.b_ifsc;
        // bNadValue
        packet[16] = self.protocol_data.b_nad;
        packet
    }
}

/// USB CCID writer.
pub struct CcidBulkIn<'d, D: Driver<'d>> {
    ep_in: D::EndpointIn,
}

/// USB CCID reader.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CcidReaderState {
    /// The reader is idle, waiting for a command from the host.
    Idle,
    /// The reader is in the process of receiving a command from the host.
    Receiving,
    /// The reader is processing the received command.
    Processing,
    /// The reader is ready to send a response to the host.
    ReadyToSend,
    /// The reader is sending a response to the host.
    Sending,
}

/// Bulk Out endpoint for receiving CCID commands from the host.
pub struct CcidBulkOut<'d, D: Driver<'d>> {
    ep_out: D::EndpointOut,
}

/// USB CCID interrupt endpoint.
pub struct CcidIntIn<'d, D: Driver<'d>> {
    ep_int_in: D::EndpointIn,
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Errors that can occur when reading the endpoint.
pub enum ReadError {
    /// The given buffer was too small to read the received report.
    BufferOverflow,
    /// The endpoint is disabled.
    Disabled,
    /// The report was only partially read. See [`HidReader::read`] for details.
    Sync(Range<usize>),
}

impl From<EndpointError> for ReadError {
    fn from(val: EndpointError) -> Self {
        use EndpointError::{BufferOverflow, Disabled};
        match val {
            BufferOverflow => ReadError::BufferOverflow,
            Disabled => ReadError::Disabled,
        }
    }
}

impl<'d, D: Driver<'d>> CcidIntIn<'d, D> {
    /// Waits for the interrupt in endpoint to be enabled.
    pub async fn ready(&mut self) {
        self.ep_int_in.wait_enabled().await;
    }

    /// Writes `report` to its interrupt endpoint.
    pub async fn write(&mut self, report: &[u8]) -> Result<(), EndpointError> {
        assert!(report.len() <= PACKET_SIZE);

        let max_packet_size = usize::from(self.ep_int_in.info().max_packet_size);
        let zlp_needed = report.len() < PACKET_SIZE && (report.len() % max_packet_size == 0);
        for chunk in report.chunks(max_packet_size) {
            trace!(
                "CCID: Writing interrupt chunk to host: {=[u8]:x}, {}",
                chunk,
                chunk.len()
            );
            self.ep_int_in.write(chunk).await?;
        }

        if zlp_needed {
            trace!("CCID: Writing ZLP to host");
            self.ep_int_in.write(&[]).await?;
        }

        Ok(())
    }
}

impl<'d, D: Driver<'d>> CcidBulkIn<'d, D> {
    /// Waits for the interrupt in endpoint to be enabled.
    pub async fn ready(&mut self) {
        self.ep_in.wait_enabled().await;
    }

    /// Writes `report` to its interrupt endpoint.
    pub async fn write(&mut self, report: &[u8]) -> Result<(), EndpointError> {
        trace!("CCID: Writing report to host: {=[u8]:x}, {}", report, report.len());
        assert!(report.len() <= MAX_MSG_LENGTH);

        let max_packet_size = usize::from(self.ep_in.info().max_packet_size);
        trace!("CCID: Endpoint max packet size: {}", max_packet_size);
        // A transfer that ends on a full packet must be terminated with a ZLP, otherwise a host
        // read sized for a longer message (up to dwMaxCCIDMessageLength) keeps waiting for data.
        let zlp_needed = !report.is_empty() && report.len() % max_packet_size == 0;
        for chunk in report.chunks(max_packet_size) {
            trace!("CCID: Writing chunk to host: {=[u8]:x}, {}", chunk, chunk.len());
            self.ep_in.write(chunk).await?;
        }
        trace!("CCID: Finished writing report to host");
        if zlp_needed {
            trace!("CCID: Writing ZLP to host");
            self.ep_in.write(&[]).await?;
        }

        Ok(())
    }
}

impl<'d, D: Driver<'d>> CcidBulkOut<'d, D> {
    /// Waits for the interrupt out endpoint to be enabled.
    pub async fn ready(&mut self) {
        self.ep_out.wait_enabled().await;
    }

    /// Reads a single USB packet from the Bulk OUT pipe into `buf`.
    ///
    /// CCID messages longer than one packet are reassembled by the caller (see
    /// [`CcidReaderWriter::handle_packet`]). Reading one packet at a time keeps this
    /// future cancel-safe: dropping it never loses data already taken from the endpoint.
    /// Zero-length packets are skipped. `buf` must hold at least one max-size packet.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ReadError> {
        let max_packet_size = usize::from(self.ep_out.info().max_packet_size);
        assert!(buf.len() >= max_packet_size);

        loop {
            match self.ep_out.read(&mut buf[..max_packet_size]).await {
                Ok(0) => continue,
                Ok(size) => {
                    trace!("CCID Read packet of size {} from host", size);
                    trace!("CCID Read packet: {=[u8]:x}", &buf[..size]);
                    return Ok(size);
                }
                Err(err) => {
                    let read_error: ReadError = err.into();
                    match read_error {
                        ReadError::BufferOverflow => {
                            error!(
                                "Host sent a packet larger than the endpoint max packet size ({})",
                                max_packet_size
                            );
                        }
                        ReadError::Disabled => {
                            warn!("Endpoint was disabled while reading");
                            self.ready().await;
                        }
                        ReadError::Sync(_) => unreachable!(),
                    }
                    return Err(read_error);
                }
            }
        }
    }
}

/// USB CCID control handler.
pub struct Control {
    if_num: InterfaceNumber,
    ccid_descriptor: [u8; CCID_DESC_BLENGTH as usize],
    // Shared with the reader; set on USB bus reset.
    session_reset: Arc<AtomicBool>,
    // Shared with the reader; carries class-specific ABORT requests.
    abort_signal: Arc<AbortSignal>,
}

impl Control {
    /// Creates a new Control handler that reports `ccid_descriptor` for GET_DESCRIPTOR requests.
    ///
    /// Crate-private: the handler shares reset and ABORT state with the reader built alongside it.
    pub(crate) fn new(
        if_num: InterfaceNumber,
        ccid_descriptor: [u8; CCID_DESC_BLENGTH as usize],
        session_reset: Arc<AtomicBool>,
        abort_signal: Arc<AbortSignal>,
    ) -> Self {
        Control {
            if_num,
            ccid_descriptor,
            session_reset,
            abort_signal,
        }
    }
}

impl Handler for Control {
    fn reset(&mut self) {
        trace!("CCID reset");
        self.session_reset.store(true, Ordering::Release);
    }

    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient, req.index)
            != (RequestType::Class, Recipient::Interface, self.if_num.0 as u16)
        {
            return None;
        }

        // This uses a defmt-specific formatter that causes use of the `log`
        // feature to fail to build, so leave it defmt-specific for now.
        trace!("CCID control_out {:?} {=[u8]:x}", req, _data);
        match ClassRequest::try_from(req.request) {
            Ok(request) => match request {
                ClassRequest::Abort => {
                    // spec: "slot in low, seq in high byte"
                    let [slot, seq] = req.value.to_le_bytes();
                    if slot > CCID_DESC_MAX_SLOT_INDEX {
                        return Some(OutResponse::Rejected);
                    }
                    // The reader completes the abort once the matching bulk ABORT arrives.
                    self.abort_signal.signal(AbortRequest { slot, seq });
                    Some(OutResponse::Accepted)
                }
                _ => Some(OutResponse::Rejected),
            },
            Err(_) => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        trace!("CCID control_in {:?} {=[u8]:x}", req, _buf);
        match (req.request_type, req.recipient) {
            (RequestType::Standard, Recipient::Interface) => match req.request {
                Request::GET_DESCRIPTOR => match (req.value >> 8) as u8 {
                    CCID_DESC_DESCTYPE_CCID => {
                        trace!("CCID GET_DESCRIPTOR wValue={:#06x}", req.value);
                        Some(InResponse::Accepted(&self.ccid_descriptor))
                    }
                    _ => Some(InResponse::Rejected),
                },

                _ => Some(InResponse::Rejected),
            },
            (RequestType::Class, Recipient::Interface) => {
                trace!("CCID control_in {:?}", req);
                match ClassRequest::try_from(req.request) {
                    Ok(request) => match request {
                        ClassRequest::GetClockFrequencies => {
                            trace!("CCID GetClockFrequencies");
                            Some(InResponse::Accepted(&CCID_DESC_CLOCK_FREQUENCY_KHZ))
                        }
                        ClassRequest::GetDataRates => {
                            trace!("CCID GetDataRates");
                            Some(InResponse::Accepted(&CCID_DESC_DATA_RATE_BPS))
                        }
                        _ => Some(InResponse::Rejected),
                    },
                    Err(_) => Some(InResponse::Rejected),
                }
            }
            _ => None,
        }
    }
}

// PIN VERIFICATION (PC_to_RDR_Secure)

/// bPINOperation: PIN verification.
const PIN_OPERATION_VERIFY: u8 = 0x00;
/// Offset of abPINApdu in the PIN verification data structure that follows bPINOperation.
const PIN_VERIFY_APDU_OFFSET: usize = 14;
/// ISO 7816-4 VERIFY. The PIN is only ever inserted into this command, so a host cannot have the
/// reader place the PIN into a command that would write or reveal it.
const INS_VERIFY: u8 = 0x20;
/// CLA, INS, P1, P2, Lc.
const APDU_HEADER_LEN: usize = 5;
/// bmFormatString PIN type: BCD.
const PIN_TYPE_BCD: u8 = 0b01;
/// bmFormatString PIN type: ASCII.
const PIN_TYPE_ASCII: u8 = 0b10;

/// Why a PIN verification request or PIN could not be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PinVerifyError {
    /// Not a PIN verification, or a PIN format this reader does not support.
    Unsupported,
    /// Inconsistent lengths, or a PIN or length field that lies outside the APDU data.
    Malformed,
    /// The APDU template is not an ISO 7816-4 VERIFY command.
    NotVerifyCommand,
    /// The PIN is empty, contains non-digits, is outside the requested length range, or does not
    /// fit the PIN block.
    InvalidPin,
}

/// A PIN verification requested by the host with PC_to_RDR_Secure.
///
/// Instead of the host prompting for the PIN, the reader obtains it and inserts it into the host's
/// VERIFY template as described by the CCID PIN verification data structure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PinVerify {
    /// bTimeOut: seconds to wait for PIN entry; 0 means the reader's default.
    pub timeout_s: u8,
    format_string: u8,
    pin_block_string: u8,
    pin_length_format: u8,
    min_pin_len: u8,
    max_pin_len: u8,
    apdu_template: Box<[u8]>,
}

impl PinVerify {
    /// Parses abData of PC_to_RDR_Secure, starting at bPINOperation.
    fn parse(data: &[u8]) -> Result<Self, PinVerifyError> {
        let (&operation, structure) = data.split_first().ok_or(PinVerifyError::Malformed)?;
        if operation != PIN_OPERATION_VERIFY {
            return Err(PinVerifyError::Unsupported);
        }
        if structure.len() < PIN_VERIFY_APDU_OFFSET + APDU_HEADER_LEN {
            return Err(PinVerifyError::Malformed);
        }
        // Remaining fields (bEntryValidationCondition, bNumberMessage, wLangId, bMsgIndex,
        // bTeoPrologue) concern PIN-pad prompts and TPDU readers and are not needed here.
        let verify = PinVerify {
            timeout_s: structure[0],
            format_string: structure[1],
            pin_block_string: structure[2],
            pin_length_format: structure[3],
            // wPINMaxExtraDigit = 0xXXYY (little-endian): XX = minimum, YY = maximum digits.
            max_pin_len: structure[4],
            min_pin_len: structure[5],
            apdu_template: structure[PIN_VERIFY_APDU_OFFSET..].into(),
        };
        verify.validate()?;
        Ok(verify)
    }

    fn validate(&self) -> Result<(), PinVerifyError> {
        if self.apdu_template[1] != INS_VERIFY {
            return Err(PinVerifyError::NotVerifyCommand);
        }
        let data_bits = self.data_len()? * 8;
        if !matches!(self.pin_type(), PIN_TYPE_BCD | PIN_TYPE_ASCII) {
            return Err(PinVerifyError::Unsupported);
        }
        if self.pin_offset_bits() + self.pin_block_bits().unwrap_or(0) > data_bits {
            return Err(PinVerifyError::Malformed);
        }
        let length_bits = self.length_field_bits();
        if length_bits > 8 || (length_bits > 0 && self.length_offset_bits() + length_bits > data_bits) {
            return Err(PinVerifyError::Unsupported);
        }
        Ok(())
    }

    /// Builds the VERIFY command APDU with `pin` (ASCII digits) inserted into the host's template.
    pub fn build_apdu(&self, pin: &[u8]) -> Result<ApplicationPacket, PinVerifyError> {
        let digits = pin.len();
        if digits == 0
            || !pin.iter().all(u8::is_ascii_digit)
            || (self.min_pin_len != 0 && digits < usize::from(self.min_pin_len))
            || (self.max_pin_len != 0 && digits > usize::from(self.max_pin_len))
        {
            return Err(PinVerifyError::InvalidPin);
        }

        let digit_bits = if self.pin_type() == PIN_TYPE_ASCII { 8 } else { 4 };
        let pin_bits = digits * digit_bits;
        let data_bits = self.data_len()? * 8;
        let block_bits = self.pin_block_bits().unwrap_or(pin_bits);
        let length_bits = self.length_field_bits();
        if pin_bits > block_bits
            || self.pin_offset_bits() + block_bits > data_bits
            || (length_bits > 0 && digits >= 1 << length_bits)
        {
            return Err(PinVerifyError::InvalidPin);
        }

        let mut apdu = self.apdu_template.to_vec();
        let data = &mut apdu[APDU_HEADER_LEN..];
        // Bit 2 of bmFormatString: right justification within the PIN block.
        let right_justified = self.format_string & 0x04 != 0;
        let start = self.pin_offset_bits() + if right_justified { block_bits - pin_bits } else { 0 };
        for (i, digit) in pin.iter().enumerate() {
            let value = if self.pin_type() == PIN_TYPE_ASCII {
                *digit
            } else {
                digit - b'0'
            };
            write_bits(data, start + i * digit_bits, digit_bits, value);
        }
        if length_bits > 0 {
            write_bits(data, self.length_offset_bits(), length_bits, digits as u8);
        }
        Ok(apdu.into_boxed_slice())
    }

    /// Lc of the template, checked against the template length.
    fn data_len(&self) -> Result<usize, PinVerifyError> {
        let lc = usize::from(self.apdu_template[4]);
        if self.apdu_template.len() < APDU_HEADER_LEN + lc {
            return Err(PinVerifyError::Malformed);
        }
        Ok(lc)
    }

    fn pin_type(&self) -> u8 {
        self.format_string & 0x03
    }

    /// bmFormatString bits 6..3: PIN position in the APDU data, in bytes (bit 7 set) or bits.
    fn pin_offset_bits(&self) -> usize {
        let position = usize::from((self.format_string >> 3) & 0x0F);
        if self.format_string & 0x80 != 0 {
            position * 8
        } else {
            position
        }
    }

    /// bmPINBlockString bits 3..0: PIN block size in bytes; 0 means just the PIN digits.
    fn pin_block_bits(&self) -> Option<usize> {
        match usize::from(self.pin_block_string & 0x0F) {
            0 => None,
            bytes => Some(bytes * 8),
        }
    }

    /// bmPINBlockString bits 7..4: size of the PIN length field in bits; 0 means no length field.
    fn length_field_bits(&self) -> usize {
        usize::from(self.pin_block_string >> 4)
    }

    /// bmPINLengthFormat bits 3..0: length field position in the APDU data, in bytes (bit 4 set) or bits.
    fn length_offset_bits(&self) -> usize {
        let position = usize::from(self.pin_length_format & 0x0F);
        if self.pin_length_format & 0x10 != 0 {
            position * 8
        } else {
            position
        }
    }
}

/// Writes the low `bits` bits of `value` into `buf`, most significant bit first, starting at
/// `bit_offset` from the start of `buf`.
fn write_bits(buf: &mut [u8], bit_offset: usize, bits: usize, value: u8) {
    for i in 0..bits {
        let bit = (value >> (bits - 1 - i)) & 1;
        let position = bit_offset + i;
        let shift = 7 - position % 8;
        buf[position / 8] = (buf[position / 8] & !(1 << shift)) | (bit << shift);
    }
}

// PACKET

/// RawPacket is a wrapper around a heapless::Vec<u8, N> that represents a CCID packet. It provides methods for parsing the CCID header and data.
pub trait RawPacketExt {
    /// Returns the length of the data in the packet, as indicated by the header.
    fn data_len(&self) -> usize;
    /// Returns a new RawPacket with the same capacity as Self, but with all bytes set to 0.
    fn zeroed() -> Self;
    /// Returns a new RawPacket with the same capacity as Self, but with the first `len` bytes set to 0 and the rest unchanged.
    fn zeroed_until(len: usize) -> Self;
}

impl RawPacketExt for RawPacket {
    fn data_len(&self) -> usize {
        u32::from_le_bytes(self[1..5].try_into().unwrap()) as usize
    }

    fn zeroed() -> Self {
        let mut res = Self::new();
        let cap = res.capacity();
        res.resize_default(cap).unwrap();
        res
    }

    fn zeroed_until(len: usize) -> Self {
        let mut res = Self::new();
        let cap = res.capacity();
        res.resize_default(len.min(cap)).unwrap();
        res
    }
}

impl RawPacketExt for ExtPacket {
    fn data_len(&self) -> usize {
        u32::from_le_bytes(self[1..5].try_into().unwrap()) as usize
    }

    fn zeroed() -> Self {
        let mut res = Self::new();
        let cap = res.capacity();
        res.resize_default(cap).unwrap();
        res
    }

    fn zeroed_until(len: usize) -> Self {
        let mut res = Self::new();
        let cap = res.capacity();
        res.resize_default(len.min(cap)).unwrap();
        res
    }
}

/// Errors that can occur when parsing a packet.
pub enum PacketError {
    /// The packet is too short to contain a valid header.
    ShortPacket,
    /// The packet contains an unknown command in the header.
    UnknownCommand(u8),
    /// The packet addresses a slot above bMaxSlotIndex.
    WrongSlot(u8),
    /// dwLength declares more payload bytes than the packet contains.
    TruncatedPayload,
}

/// A trait for parsing a CCID packet from a raw byte slice. This is implemented for the different command types (e.g. XfrBlock) and provides methods for accessing the header fields and data.
pub trait Packet: core::ops::Deref<Target = ExtPacket> {
    #[inline]
    /// Returns the slot number from the packet header. As per CCID spec, this is in byte 5 of the header. This implementation assumes only one slot (slot 0) and asserts that the slot number is 0.
    fn slot(&self) -> u8 {
        // Command::try_from rejects slots above bMaxSlotIndex, so this is always a valid slot.
        self[5]
    }

    #[inline]
    /// Returns the sequence number from the packet header. As per CCID spec, this is in byte 6 of the header.
    fn seq(&self) -> u8 {
        self[6]
    }
}

/// A trait for packets that contain data. This provides a method for accessing the data portion of the packet, which is the bytes after the 10-byte header. The length of the data is determined by the length field in the header (bytes 1-4), but this implementation also ensures that we don't return more than `MAX_MSG_LENGTH - CCID_HEADER_LEN` bytes to avoid overflowing our buffers.
pub trait PacketWithData: Packet {
    #[inline]
    /// Returns the data portion of the packet as a byte slice, as declared by the length field in
    /// the header (bytes 1-4). `Command::try_from` rejects packets whose dwLength exceeds the
    /// received payload, so the declared length always fits.
    fn data(&self) -> &[u8] {
        let declared_len = u32::from_le_bytes(self[1..5].try_into().unwrap()) as usize;
        &self[CCID_HEADER_LEN..][..declared_len]
    }
}

#[derive(Debug, Default, Clone, Copy)]
/// The chaining status of a packet, as indicated by the chain parameter in the header (byte 9). This is used to determine how to handle packets that are split across multiple USB transfers.
pub struct UnknownChaining;

/// Trait to get the chain
pub trait ChainedPacket: Packet {
    #[inline(always)]
    /// Returns the chaining status of the packet, as indicated by the chain parameter in the header (byte 9). This is used to determine how to handle packets that are split across multiple USB transfers. The possible values are:
    fn chain(&self) -> Result<Chain, UnknownChaining> {
        let level_parameter = u16::from_le_bytes(self[8..10].try_into().unwrap());
        match level_parameter {
            0 => Ok(Chain::BeginsAndEnds),
            1 => Ok(Chain::Begins),
            2 => Ok(Chain::Ends),
            3 => Ok(Chain::Continues),
            0x10 => Ok(Chain::ExpectingMore),
            _ => Err(UnknownChaining),
        }
    }
}

impl ChainedPacket for XfrBlock {}

/// A data block to be sent to the host, which can be converted into a RawPacket. This is used for sending responses that may be split across multiple USB transfers, and allows us to keep track of the sequence number and chaining status of the response.
pub struct DataBlock<'a> {
    seq: u8,
    chain: Chain,
    data: &'a [u8],
}

impl<'a> DataBlock<'a> {
    /// Creates a new DataBlock with the given sequence number, chaining status, and data. The length of the data must be less than or equal to `MAX_MSG_LENGTH - CCID_HEADER_LEN` to ensure that it can fit in a single packet without overflowing our buffers.
    pub fn new(seq: u8, chain: Chain, data: &'a [u8]) -> Self {
        //assert!(data.len() + CCID_HEADER_LEN <= PACKET_SIZE);
        Self { seq, chain, data }
    }
}

impl core::fmt::Debug for DataBlock<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut debug_struct = f.debug_struct("DataBlock");

        debug_struct.field("seq", &self.seq);

        let l = core::cmp::min(self.data.len(), 16);
        let escaped_bytes: heapless::Vec<u8, 64> = self
            .data
            .iter()
            .take(l)
            .flat_map(|byte| core::ascii::escape_default(*byte))
            .collect();
        let data_as_str = &core::str::from_utf8(&escaped_bytes).unwrap();

        debug_struct
            .field("chain", &self.chain)
            .field("len", &self.data.len())
            .field("data", &format_args!("b'{data_as_str}'"))
            .finish()
    }
}

impl From<DataBlock<'_>> for RawPacket {
    fn from(block: DataBlock<'_>) -> RawPacket {
        let len = block.data.len();
        let mut packet = RawPacket::zeroed_until(CCID_HEADER_LEN + len);
        packet[0] = 0x80;
        packet[1..][..4].copy_from_slice(
            &u32::try_from(len)
                .expect("Packets should not be more than 4GiB")
                .to_le_bytes(),
        );
        packet[5] = 0;
        packet[6] = block.seq;

        // status
        packet[7] = 0;
        // error
        packet[8] = 0;
        // chain parameter
        packet[9] = block.chain as u8;
        packet[CCID_HEADER_LEN..][..len].copy_from_slice(block.data);

        packet
    }
}

impl From<DataBlock<'_>> for ExtPacket {
    fn from(block: DataBlock<'_>) -> ExtPacket {
        let len = block.data.len();
        let mut packet = ExtPacket::zeroed_until(CCID_HEADER_LEN + len);
        packet[0] = 0x80;
        packet[1..][..4].copy_from_slice(
            &u32::try_from(len)
                .expect("Packets should not be more than 4GiB")
                .to_le_bytes(),
        );
        packet[5] = 0;
        packet[6] = block.seq;

        // status
        packet[7] = 0;
        // error
        packet[8] = 0;
        // chain parameter
        packet[9] = block.chain as u8;
        packet[CCID_HEADER_LEN..][..len].copy_from_slice(block.data);

        packet
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
/// Bulk Out request codes (bRequest values for class-specific Bulk Out requests).
pub enum CommandType {
    /// PC_to_RDR_IccPowerOn
    IccPowerOn = 0x62,
    /// PC_to_RDR_IccPowerOff
    IccPowerOff = 0x63,
    /// PC_to_RDR_GetSlotStatus
    GetSlotStatus = 0x65,
    /// PC_to_RDR_XfrBlock
    XfrBlock = 0x6F,
    /// PC_to_RDR_GetParameters
    GetParameters = 0x6C,
    /// PC_to_RDR_ResetParameters
    ResetParameters = 0x6D,
    /// PC_to_RDR_SetParameters
    SetParameters = 0x61,
    /// PC_to_RDR_Escape
    Escape = 0x6B,
    /// PC_to_RDR_IccClock
    IccClock = 0x6E,
    /// PC_to_RDR_T0APDU
    T0APDU = 0x6A,
    /// PC_to_RDR_Secure
    Secure = 0x69,
    /// PC_to_RDR_Mechanical
    Mechanical = 0x71,
    /// PC_to_RDR_Abort
    Abort = 0x72,
    /// PC_to_RDR_SetDataRateAndClockFrequency
    SetDataRateAndClockFrequency = 0x73,
}

macro_rules! command_message {

    ($($Name:ident: $code:expr,)*) => {
        $(
            #[derive(Debug)]
            /// A struct representing a specific command type, which contains the raw packet data. This allows us to implement methods for parsing the header fields and data for each command type, while still keeping the raw packet data available for any additional parsing that may be needed.
            pub struct $Name {
                // use reference? pulls in lifetimes though...
                ext_raw: ExtPacket,
            }

            impl core::ops::Deref for $Name {
                type Target = ExtPacket;

                #[inline]
                fn deref(&self) -> &Self::Target {
                    &self.ext_raw
                }
            }

            impl core::ops::DerefMut for $Name {

                #[inline]
                fn deref_mut(&mut self) -> &mut Self::Target {
                    &mut self.ext_raw
                }
            }

            impl Packet for $Name {}
        )*

        /// An enum representing all possible command types, which can be parsed from a raw packet. This allows us to easily match on the command type and access the specific fields and data for each command type, while still keeping the raw packet data available for any additional parsing that may be needed.
        pub enum Command {
            $(
                /// A variant for the $Name command type, which contains the raw packet data for that command. This allows us to implement methods for parsing the header fields and data for each command type, while still keeping the raw packet data available for any additional parsing that may be needed.
                $Name($Name),
            )*
        }

        /// Methods for accessing the common header fields (slot, sequence number, and command type) for any command type. These methods match on the command type and delegate to the specific implementation for each command type, which allows us to keep the logic for parsing the header fields in one place while still allowing for any command-specific parsing that may be needed.
        impl Command {
            /// Returns the sequence number from the packet header. As per CCID spec, this is in byte 6 of the header.
            pub fn seq(&self) -> u8 {
                match self {
                    $(
                        Command::$Name(packet) => packet.seq(),
                    )*
                }
            }

            /// Returns the slot number from the packet header. As per CCID spec, this is in byte 5 of the header. This implementation assumes only one slot (slot 0) and asserts that the slot number is 0.
            pub fn slot(&self) -> u8 {
                match self {
                    $(
                        Command::$Name(packet) => packet.slot(),
                    )*
                }
            }

            /// Returns the command type for this command, which is determined by the command byte in the header (byte 0). This allows us to easily match on the command type and access the specific fields and data for each command type, while still keeping the raw packet data available for any additional parsing that may be needed.
            pub fn command_type(&self) -> CommandType {
                match self {
                    $(
                        Command::$Name(_) => CommandType::$Name,
                    )*
                }
            }
        }

        impl core::convert::TryFrom<ExtPacket> for Command {
            type Error = PacketError;

            #[inline]
            fn try_from(packet: ExtPacket)
                -> core::result::Result<Self, Self::Error>
            {
                if packet.len() < CCID_HEADER_LEN {
                    return Err(PacketError::ShortPacket);
                }
                if packet[5] > CCID_DESC_MAX_SLOT_INDEX {
                    return Err(PacketError::WrongSlot(packet[5]));
                }
                // PacketWithData::data() relies on dwLength fitting the received payload.
                if packet.data_len() > packet.len() - CCID_HEADER_LEN {
                    return Err(PacketError::TruncatedPayload);
                }
                let command_byte = packet[0];
                Ok(match command_byte {
                    $(
                        $code => Command::$Name($Name { ext_raw: packet } ),
                    )*
                    _ => return Err(PacketError::UnknownCommand(command_byte)),
                })
            }
        }

        impl core::ops::Deref for Command {
            type Target = ExtPacket;

            #[inline]
            fn deref(&self) -> &Self::Target {
                match self {
                    $(
                        Command::$Name(packet) => &packet,
                    )*
                }
            }
        }
    }
}

command_message!(
    IccPowerOn: 0x62,
    IccPowerOff: 0x63,
    GetSlotStatus: 0x65,
    GetParameters: 0x6c,
    XfrBlock: 0x6f,
    ResetParameters: 0x6d,
    SetParameters: 0x61,
    Escape: 0x6b,
    IccClock: 0x6e,
    T0APDU: 0x6a,
    Secure: 0x69,
    Mechanical: 0x71,
    Abort: 0x72,
    SetDataRateAndClockFrequency: 0x73,
);

impl PacketWithData for XfrBlock {}
impl PacketWithData for Secure {}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// The chaining status of a packet, as indicated by the chain parameter in the header (byte 9). This is used to determine how to handle packets that are split across multiple USB transfers.
pub enum Chain {
    /// This packet is not part of a chain and contains the complete message.
    BeginsAndEnds = 0,
    /// This packet is the beginning of a chain of packets that together contain a complete message.
    Begins = 1,
    /// This packet is the end of a chain of packets that together contain a complete message.
    Ends = 2,
    /// This packet is in the middle of a chain of packets that together contain a complete message. It is neither the beginning nor the end of the chain.
    Continues = 3,
    /// This packet is part of a chain of packets that together contain a complete message, and the host is indicating that it expects more packets to follow in the chain. This is used when the host is sending a message that is larger than the maximum packet size and needs to be split across multiple USB transfers.
    ExpectingMore = 0x10,
}

impl Chain {
    /// Returns true if the packet is part of an ongoing transfer, which includes packets that are the beginning, middle, or end of a chain, as well as packets that indicate that more packets are expected to follow in the chain. This is used to determine whether we should expect more packets to be part of the same message and handle them accordingly.
    pub fn transfer_ongoing(&self) -> bool {
        matches!(self, Chain::BeginsAndEnds | Chain::Ends | Chain::ExpectingMore)
    }
}

impl core::fmt::Debug for Command {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut debug_struct = f.debug_struct("Command");

        debug_struct
            .field("cmd", &self.command_type())
            .field("seq", &self.seq());

        if let Command::XfrBlock(block) = self {
            let l = core::cmp::min(self.len(), 8);
            let escaped_bytes: heapless::Vec<u8, 64> = block
                .data()
                .iter()
                .take(l)
                .flat_map(|byte| core::ascii::escape_default(*byte))
                .collect();
            let data_as_str = &core::str::from_utf8(&escaped_bytes).unwrap();

            debug_struct
                .field("chain", &block.chain())
                .field("len", &block.data().len());

            if l < self.len() {
                debug_struct.field("data[..8]", &format_args!("b'{data_as_str}'"))
            } else {
                debug_struct.field("data", &format_args!("b'{data_as_str}'"))
            };
        }

        debug_struct.finish()
    }
}
