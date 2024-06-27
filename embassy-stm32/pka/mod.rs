//! PKA Hardware accelerator
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, pac, peripherals, rcc};

static PKA_WAKER: AtomicWaker = AtomicWaker::new();

// List of RAM addresses for data. Since embassy implementation of RAM access already moves base address to
// 0x0400 and multiplies offset by 4 for 32-bit access, we have to take address from reference manual, divide it by 4
// and subtract 0x0400 to get the correct offset.
const OPERAND_LENGTH_ADDR: usize = 0x1; // 0x404
const MODULUS_ADDR: usize = 0x257; // 0xD5C
const MONT_PARAM_R2_MOD_N: usize = 0x65; // 0x594
const OPERAND_A_ADDR: usize = 0x12D; // 0x8B4
const OPERAND_B_ADDR: usize = 0x191; // 0xA44

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// PKA Operation Mode
pub enum PkaOperationMode {
    /// Montgomery Parameter Computation then Modular Exponentiation
    MontgomeryParameterComputationThenModularExponentiation = 0b000000,
    /// Montgomery Parameter Computation Only
    MontgomeryParameterComputationOnly = 0b000001,
    /// Modular Exponentiation Only. (Montgomery parameter must be loaded first)
    ModularExponentiationOnly = 0b000010,
    /// Montgomery Parameter Computation then ECC Scalar Multiplication
    MontgomeryParameterComputationThenEccScalarMultiplication = 0b100000,
    /// ECC Scalar Multiplication Only. (Montgomery parameter must be loaded first)
    EccScalarMultiplicationOnly = 0b100010,
    /// EDCSA Sign
    EcdsaSign = 0b100100,
    /// EDCSA Verification
    EcdsaVerification = 0b100110,
    /// Point On Elliptic Curve FP Check
    PointOnEllipticCurveFpCheck = 0b101000,
    /// RSA CRT Exponentiation
    RsaCrtExponentiation = 0b000111,
    /// Modular Inversion
    ModularInversion = 0b001000,
    /// Arithmetic Addition
    ArithmeticAddition = 0b001001,
    /// Arithmetic Subtraction
    ArithmeticSubtraction = 0b001010,
    /// Arithmetic Multiplication
    ArithmeticMultiplication = 0b001011,
    /// Arithmetic Comparison
    ArithmeticComparison = 0b001100,
    /// Modular Reduction
    ModularReduction = 0b001101,
    /// Modular Addition
    ModularAddition = 0b001110,
    /// Modular Subtraction
    ModularSubtraction = 0b001111,
    /// Montgomery Multiplication
    MontgomeryMultiplication = 0b010000,
}

/// PKA SealedInstance trait.
///
/// Allows access to PAC exclusively for `PKA` module.
trait SealedInstance {
    fn regs() -> pac::pka::Pka;
}

/// PKA interrupt
pub struct InterruptHandler<T: Instance> {
    __phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        info!("PKA interrupt");
        let procendf = T::regs().sr().read().procendf();
        if procendf {
            T::regs().cr().modify(|w| w.set_procendie(false));
            PKA_WAKER.wake();
            info!("PKA interrupt: procendf")
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// RSA structure for PKA accelerator
pub struct Rsa<'c, 'd, T: Instance> {
    /// Modulus
    pub modulus: u32,
    /// Prime number p
    pub prime_p: u32,
    /// Prime number q
    pub prime_q: u32,
    /// Public key
    pub public_key: [u32; 8],
    /// Private key
    pub private_key: [u32; 8],
    /// PKA accelerator
    pka: &'c mut Pka<'d, T>,
}

impl<'c, 'd, T: Instance> Rsa<'c, 'd, T> {
    /// Create a new RSA structure
    pub fn new(pka: &'c mut Pka<'d, T>) -> Self {
        let prime_p: u32 = 7993; // TODO: Generate
        let prime_q: u32 = 6763; // TODO: Generate

        Self {
            modulus: 0,
            prime_p,
            prime_q,
            public_key: [0; 8],
            private_key: [0; 8],
            pka,
        }
    }

    /// Generate RSA key pair
    pub async fn generate_key_pair(&mut self) {
        // Generate public and private keys

        // Generate modulus
        // FIXME: Currently modulus is hardcoded to 32 bits
        self.modulus = self
            .pka
            .arithmetic_multiply(&self.prime_p, &self.prime_q)
            .await
            .unwrap()[0];

        // Compute Eueler's totient function
        let phi = self
            .pka
            .arithmetic_multiply(&(self.prime_p - 1), &(self.prime_q - 1))
            .await
            .unwrap()[0];

        // Generate public key
        self.public_key = [65537, 0, 0, 0, 0, 0, 0, 0];

        // Generate private key
        self.private_key[0] = self.pka.mod_inverse(&self.public_key[0], &[phi]).await.unwrap()[0];
    }

    /// Print key pair
    pub fn print_key_pair(&self) {
        info!("Public key: {:?}", self.public_key);
        info!("Private key: {:?}", self.private_key);
    }
}

/// PKA accelerator
pub struct Pka<'d, T> {
    _peripheral: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Pka<'d, T> {
    /// Create a new PKA accelerator
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        rcc::enable_and_reset::<T>();
        into_ref!(peri);
        let instance = Self { _peripheral: peri };

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        instance
    }

    /// Disable PKA accelerator
    fn disable(&mut self) {
        T::regs().cr().modify(|w| w.set_en(false));
    }

    /// Enable PKA accelerator
    fn enable(&mut self) {
        T::regs().cr().modify(|w| w.set_en(true));
    }

    /// Start PKA operation
    async fn start(&mut self) {
        T::regs().cr().modify(|w| w.set_start(true));
        // Wait for results
        poll_fn(|ctx| {
            if T::regs().sr().read().procendf() {
                T::regs().clrfr().write(|w| w.set_procendfc(true));
                return Poll::Ready(());
            }

            PKA_WAKER.register(ctx.waker());
            T::regs().cr().modify(|w| w.set_procendie(true));

            if T::regs().sr().read().procendf() {
                T::regs().clrfr().write(|w| w.set_procendfc(true));
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    /// Set PKA working mode
    fn set_mode(&mut self, mode: u8) {
        T::regs().cr().modify(|w| w.set_mode(mode));
    }

    /// Write to PKA RAM with offset
    fn write_to_ram(&mut self, offset: usize, data: &[u32]) {
        for (i, byte) in data.iter().enumerate() {
            T::regs().ram(offset + i).write(|w| {
                *w = *byte;
                info!("write_to_ram: {:?}", *byte);
                *w
            });
        }
    }

    /// Read from PKA RAM with offset
    fn read_from_ram(&mut self, offset: usize, data: &mut [u32]) {
        // Read from PKA RAM with offset. RAM is little endian, so we have to convert it
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = T::regs().ram(offset + i).read();
            info!("read_from_ram: {:?}", *byte);
        }
    }

    /// Calculate the Montgomery parameter
    pub async fn montgomery_param(&mut self, mod_value: &[u32]) -> Result<[u32; 1], ()> {
        let mut modulus_size: [u32; 1] = [mod_value.len() as u32];

        // Get the size of a value in modulus in bits
        // FIXME: Currently modulus size is hardcoded to 32 bits.
        // It has to be calculated from the modulus value, which is an array of u32

        // Modulus can only be odd number
        if mod_value[0] % 2 == 0 {
            return Err(());
        }

        // Get the number of leading zeros in the modulus value
        modulus_size[0] = 32 - mod_value[0].leading_zeros();
        self.write_to_ram(OPERAND_LENGTH_ADDR, &modulus_size);
        self.write_to_ram(MODULUS_ADDR, mod_value);

        let mode = PkaOperationMode::MontgomeryParameterComputationOnly as u8 | T::regs().cr().read().mode();
        self.set_mode(mode);

        self.start().await;

        let buf: &mut [u32; 1] = &mut [0; 1];
        self.read_from_ram(MONT_PARAM_R2_MOD_N, buf);
        // Get only "modulus_size" bits from this result
        let shift = 32 - modulus_size[0];
        let mask = (0xFFFFFFFF >> shift) << shift;
        let result = [(buf[0] & mask) >> shift];
        Ok(result)
    }

    /// Multiply two numbers using Montgomery multiplication
    pub async fn modular_multiply(&mut self, a: &u32, b: &u32, modulus: &[u32]) -> Result<[u32; 1], ()> {
        let mut result: [u32; 1] = [0];
        let a_value: [u32; 1] = [*a];
        let b_value: [u32; 1] = [*b];

        //FIXME: Currently modulus size is hardcoded to 32 bits. Same for the result

        // Calculate the maximum size of operands based on leading zeros
        let operand_size: [u32; 1] = [32 - a.leading_zeros().max(32 - b.leading_zeros())];

        self.write_to_ram(OPERAND_LENGTH_ADDR, &operand_size);
        self.write_to_ram(MODULUS_ADDR, modulus);

        self.write_to_ram(OPERAND_A_ADDR, &a_value);
        self.write_to_ram(OPERAND_B_ADDR, &b_value);

        let mode = PkaOperationMode::MontgomeryMultiplication as u8 | T::regs().cr().read().mode();
        self.set_mode(mode);

        self.start().await;

        let buf: &mut [u32; 1] = &mut [0; 1];
        self.read_from_ram(MONT_PARAM_R2_MOD_N, buf);
        result[0] = buf[0];
        Ok(result)
    }

    /// Calculate A * B
    pub async fn arithmetic_multiply(&mut self, a: &u32, b: &u32) -> Result<[u32; 1], ()> {
        let mut result: [u32; 1] = [0];
        let a_value: [u32; 1] = [*a];
        let b_value: [u32; 1] = [*b];

        // Calculate the operand size
        let operand_size: [u32; 1] = [32 - a.leading_zeros().max(32 - b.leading_zeros())];

        self.write_to_ram(OPERAND_LENGTH_ADDR, &operand_size);
        self.write_to_ram(OPERAND_A_ADDR, &a_value);
        self.write_to_ram(OPERAND_B_ADDR, &b_value);

        let mode = PkaOperationMode::ArithmeticMultiplication as u8 | T::regs().cr().read().mode();
        self.set_mode(mode);

        self.start().await;

        let buf: &mut [u32; 1] = &mut [0; 1];
        self.read_from_ram(MONT_PARAM_R2_MOD_N, buf);
        result[0] = buf[0];
        Ok(result)
    }

    /// Calculate the modular exponentiation
    pub async fn mod_inverse(&mut self, a: &u32, modulus: &[u32]) -> Result<[u32; 1], ()> {
        let mut result: [u32; 1] = [0];
        let a_value: [u32; 1] = [*a];

        // Calculate the operand size
        let operand_size: [u32; 1] = [32 - a.leading_zeros()];

        self.write_to_ram(OPERAND_LENGTH_ADDR, &operand_size);
        self.write_to_ram(OPERAND_A_ADDR, &a_value);
        // This is not an error. Once in a while modulus is written to different address
        self.write_to_ram(OPERAND_B_ADDR, modulus);

        let mode = PkaOperationMode::ModularInversion as u8 | T::regs().cr().read().mode();
        self.set_mode(mode);

        self.start().await;

        let buf: &mut [u32; 1] = &mut [0; 1];
        self.read_from_ram(MONT_PARAM_R2_MOD_N, buf);
        result[0] = buf[0];
        Ok(result)
    }
}

/// PKA instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Peripheral<P = Self> + crate::rcc::RccPeripheral + 'static + Send {
    /// Interrupt for this PKA instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_interrupt!(
    ($inst:ident, pka, PKA, GLOBAL, $irq:ident) => {
        impl Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }

        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::pka::Pka {
                crate::pac::$inst
            }
        }
    };
);
