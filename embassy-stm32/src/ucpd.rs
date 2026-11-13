//! USB Type-C/USB Power Delivery Interface (UCPD)

// Implementation Notes
//
// As of Feb. 2024 the UCPD peripheral is available on: G0, G4, H5, L5, U5
//
// Cube HAL LL Driver (g0):
// https://github.com/STMicroelectronics/stm32g0xx_hal_driver/blob/v1.4.6/Inc/stm32g0xx_ll_ucpd.h
// https://github.com/STMicroelectronics/stm32g0xx_hal_driver/blob/v1.4.6/Src/stm32g0xx_ll_ucpd.c
// Except for a the `LL_UCPD_RxAnalogFilterEnable/Disable()` functions the Cube HAL implementation of
// all families is the same.
//
// Dead battery pull-down resistors functionality is enabled by default on startup and must
// be disabled by setting a bit in PWR/SYSCFG registers. The exact name and location for that
// bit is different for each familily.

use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use critical_section::Mutex;
use embassy_hal_internal::PeripheralType;
use embassy_hal_internal::drop::OnDrop;
use embassy_sync::waitqueue::AtomicWaker;

use crate::atomic::{AtomicClear, AtomicModify};
use crate::dma::{ChannelAndRequest, ReadableRingBuffer, TransferOptions};
use crate::interrupt::typelevel::Interrupt;
use crate::pac::ucpd::vals::{Anamode, Ccenable, PscUsbpdclk, Txmode};
pub use crate::pac::ucpd::vals::{Phyccsel as CcSel, Rxordset, TypecVstateCc as CcVState};
use crate::rcc::{self, RccPeripheral};
use crate::{Peri, interrupt};

pub(crate) fn init(
    _cs: critical_section::CriticalSection,
    #[cfg(all(peri_ucpd1, not(stm32n6)))] ucpd1_db_enable: bool,
    #[cfg(peri_ucpd2)] ucpd2_db_enable: bool,
) {
    #[cfg(stm32g0x1)]
    {
        // according to RM0444 (STM32G0x1) section 8.1.1:
        // when UCPD is disabled setting the strobe will disable dead battery
        // (which is enabled after reset) but if UCPD is enabled, setting the
        // strobe will apply the CC pin configuration from the control register
        // (which is why we need to be careful about when we call this)
        crate::pac::SYSCFG.cfgr1().modify(|w| {
            w.set_ucpd1_strobe(!ucpd1_db_enable);
            w.set_ucpd2_strobe(!ucpd2_db_enable);
        });
    }

    #[cfg(any(stm32g4, stm32l5))]
    {
        crate::pac::PWR.cr3().modify(|w| {
            #[cfg(stm32g4)]
            w.set_ucpd1_dbdis(!ucpd1_db_enable);
            #[cfg(stm32l5)]
            w.set_ucpd_dbdis(!ucpd1_db_enable);
        })
    }

    #[cfg(any(stm32h5, stm32u5, stm32h7rs))]
    {
        crate::pac::PWR.ucpdr().modify(|w| {
            w.set_ucpd_dbdis(!ucpd1_db_enable);
        })
    }
}

/// Pull-up or Pull-down resistor state of both CC lines.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CcPull {
    /// Analog PHY for CC pin disabled.
    Disabled,

    /// Rd=5.1k pull-down resistor.
    Sink,

    /// Rp=56k pull-up resistor to indicate default USB power.
    SourceDefaultUsb,

    /// Rp=22k pull-up resistor to indicate support for up to 1.5A.
    Source1_5A,

    /// Rp=10k pull-up resistor to indicate support for up to 3.0A.
    Source3_0A,
}

/// UCPD configuration
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// Receive SOP packets
    pub sop: bool,
    /// Receive SOP' packets
    pub sop_prime: bool,
    /// Receive SOP'' packets
    pub sop_double_prime: bool,
    /// Receive SOP'_Debug packets
    pub sop_prime_debug: bool,
    /// Receive SOP''_Debug packets
    pub sop_double_prime_debug: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sop: true,
            sop_prime: false,
            sop_double_prime: false,
            sop_prime_debug: false,
            sop_double_prime_debug: false,
        }
    }
}

/// UCPD driver.
pub struct Ucpd<'d, T: Instance> {
    cc_phy: CcPhy<'d, T>,
}

impl<'d, T: Instance> Ucpd<'d, T> {
    /// Creates a new UCPD driver instance.
    pub fn new(
        _peri: Peri<'d, T>,
        cc1: Peri<'d, impl Cc1Pin<T>>,
        cc2: Peri<'d, impl Cc2Pin<T>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        cc1.pin_number().set_as_analog();
        cc2.pin_number().set_as_analog();

        rcc::enable_and_reset::<T>();
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        let r = T::REGS;

        #[cfg(stm32h5)]
        r.cfgr2().write(|w| {
            // Only takes effect, when UCPDEN=0.
            w.set_rxafilten(true);
        });

        r.cfgr1().write(|w| {
            // "The receiver is designed to work in the clock frequency range from 6 to 18 MHz.
            // However, the optimum performance is ensured in the range from 6 to 12 MHz"
            // UCPD is driven by HSI16 (16MHz internal oscillator), which we need to divide by 2.
            w.set_psc_usbpdclk(PscUsbpdclk::Div2);

            // Prescaler to produce a target half-bit frequency of 600kHz which is required
            // to produce transmit with a nominal nominal bit rate of 300Kbps+-10% using
            // biphase mark coding (BMC, aka differential manchester coding).
            // A divider of 13 gives the target frequency closest to spec (~615kHz, 1.625us).
            w.set_hbitclkdiv(13 - 1);

            // Time window for detecting non-idle (12-20us).
            // 1.75us * 8 = 14us.
            w.set_transwin(8 - 1);

            // Time from the end of last bit of a Frame until the start of the first bit of the
            // next Preamble (min 25us).
            // 1.75us * 17 = ~30us
            w.set_ifrgap(17 - 1);

            // UNDOCUMENTED: This register can only be written while UCPDEN=0 (found by testing).
            let rxordset = (config.sop as u16)
                | (config.sop_prime as u16) << 1
                | (config.sop_double_prime as u16) << 2
                // Hard reset
                | 0x1 << 3
                | (config.sop_prime_debug as u16) << 4
                | (config.sop_double_prime_debug as u16) << 5;
            w.set_rxordseten(rxordset);

            // Enable DMA
            w.set_txdmaen(true);
            w.set_rxdmaen(true);

            w.set_ucpden(true);
        });

        // Software trim according to RM0481, p. 2650/2668
        #[cfg(stm32h5)]
        {
            let trim_rd_cc1 = unsafe { *(0x4002_242C as *const u32) & 0xF };
            let trim_rd_cc2 = unsafe { ((*(0x4002_242C as *const u32)) >> 8) & 0xF };

            r.cfgr3().write(|w| {
                w.set_trim_cc1_rd(trim_rd_cc1 as u8);
                w.set_trim_cc2_rd(trim_rd_cc2 as u8);
            });
        }

        // Software trim according to RM0456, p. 3480/3462
        #[cfg(stm32u5)]
        {
            let trim_rd_cc1 = unsafe { *(0x0BFA_0544 as *const u8) & 0xF };
            let trim_rd_cc2 = unsafe { *(0x0BFA_0546 as *const u8) & 0xF };

            r.cfgr3().write(|w| {
                w.set_trim_cc1_rd(trim_rd_cc1);
                w.set_trim_cc2_rd(trim_rd_cc2);
            });
        }

        Self {
            cc_phy: CcPhy { _lifetime: PhantomData },
        }
    }

    /// Returns the TypeC CC PHY.
    pub fn cc_phy(&mut self) -> &mut CcPhy<'d, T> {
        &mut self.cc_phy
    }

    /// Splits the UCPD driver into a TypeC PHY to control and monitor CC voltage
    /// and a Power Delivery (PD) PHY with receiver and transmitter.
    ///
    /// PD messages are received continuously into `rx_buf`, a ring buffer, until the
    /// [`PdPhy`] is dropped. It must hold at least one message of the largest size (264 bytes);
    /// a few times that leaves room for messages that wait for [`PdPhy::receive`].
    pub fn split_pd_phy<RX, TX>(
        self,
        rx_dma: Peri<'d, RX>,
        tx_dma: Peri<'d, TX>,
        irqs: impl interrupt::typelevel::Binding<RX::Interrupt, crate::dma::InterruptHandler<RX>>
        + interrupt::typelevel::Binding<TX::Interrupt, crate::dma::InterruptHandler<TX>>
        + 'd,
        cc_sel: CcSel,
        rx_buf: &'d mut [u8],
    ) -> (CcPhy<'d, T>, PdPhy<'d, T>)
    where
        RX: RxDma<T>,
        TX: TxDma<T>,
    {
        let (cc_phy, rx_dma, tx_dma, part) = self.split_dma(rx_dma, tx_dma, irqs);
        let mut pd_phy = PdPhy::new(rx_dma, tx_dma, rx_buf, Some(part));
        pd_phy.start_reception(cc_sel);
        (cc_phy, pd_phy)
    }

    /// Splits the UCPD driver like [`split_pd_phy`](Self::split_pd_phy), but with PD
    /// communication disabled.
    ///
    /// Both parts can be kept for as long as the driver is needed. On attach, enable PD on the
    /// connected CC line with [`IdlePdPhy::activate`], and drop the returned [`ActivePdPhy`] on
    /// detach. Neither affects the CC terminations set with [`CcPhy::set_pull`].
    pub fn split_idle_pd_phy<RX, TX>(
        self,
        rx_dma: Peri<'d, RX>,
        tx_dma: Peri<'d, TX>,
        irqs: impl interrupt::typelevel::Binding<RX::Interrupt, crate::dma::InterruptHandler<RX>>
        + interrupt::typelevel::Binding<TX::Interrupt, crate::dma::InterruptHandler<TX>>
        + 'd,
    ) -> (CcPhy<'d, T>, IdlePdPhy<'d, T>)
    where
        RX: RxDma<T>,
        TX: TxDma<T>,
    {
        let (cc_phy, rx_dma, tx_dma, part) = self.split_dma(rx_dma, tx_dma, irqs);
        (
            cc_phy,
            IdlePdPhy {
                rx_dma,
                tx_dma,
                _part: part,
            },
        )
    }

    fn split_dma<RX, TX>(
        self,
        rx_dma: Peri<'d, RX>,
        tx_dma: Peri<'d, TX>,
        irqs: impl interrupt::typelevel::Binding<RX::Interrupt, crate::dma::InterruptHandler<RX>>
        + interrupt::typelevel::Binding<TX::Interrupt, crate::dma::InterruptHandler<TX>>
        + 'd,
    ) -> (CcPhy<'d, T>, ChannelAndRequest<'d>, ChannelAndRequest<'d>, PdPart<T>)
    where
        RX: RxDma<T>,
        TX: TxDma<T>,
    {
        // Both parts must be dropped before the peripheral can be disabled.
        T::state().drop_not_ready.store(true, Ordering::Relaxed);

        let rx_dma = new_dma_nonopt!(rx_dma, irqs);
        let tx_dma = new_dma_nonopt!(tx_dma, irqs);
        (self.cc_phy, rx_dma, tx_dma, PdPart(PhantomData))
    }
}

/// The PD part of the driver, the counterpart of [`CcPhy`].
///
/// Dropping it disables PD reception, and the whole peripheral if the [`CcPhy`] is gone already.
struct PdPart<T: Instance>(PhantomData<T>);

impl<T: Instance> Drop for PdPart<T> {
    fn drop(&mut self) {
        let r = T::REGS;
        modify_cr::<T>(|w| w.set_phyrxen(false));
        // Check if the CcPhy part was dropped already.
        if !T::state().drop_not_ready.clear() {
            r.cfgr1().write(|w| w.set_ucpden(false));
            rcc::disable::<T>();
            T::Interrupt::disable();
        }
    }
}

/// Enables PD communication on the given CC line.
fn enable_pd<T: Instance>(cc_sel: CcSel) {
    let r = T::REGS;

    // Enable the receiver on one of the two CC lines.
    modify_cr::<T>(|w| w.set_phyccsel(cc_sel));

    // Enable hard reset receive interrupt.
    r.imr().set_bits(|w| w.set_rxhrstdetie(true));

    // Enable PD packet reception
    modify_cr::<T>(|w| w.set_phyrxen(true));
}

/// Control and monitoring of TypeC CC pin functionality.
pub struct CcPhy<'d, T: Instance> {
    _lifetime: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Drop for CcPhy<'d, T> {
    fn drop(&mut self) {
        let r = T::REGS;
        modify_cr::<T>(|w| {
            w.set_cc1tcdis(true);
            w.set_cc2tcdis(true);
            w.set_ccenable(Ccenable::Disabled);
        });

        // Check if the PdPhy part was dropped already.
        if !T::state().drop_not_ready.clear() {
            r.cfgr1().write(|w| w.set_ucpden(false));
            rcc::disable::<T>();
            T::Interrupt::disable();
        }
    }
}

impl<'d, T: Instance> CcPhy<'d, T> {
    /// Sets the pull-up/pull-down resistor values exposed on the CC pins.
    pub fn set_pull(&mut self, cc_pull: CcPull) {
        modify_cr::<T>(|w| {
            w.set_anamode(if cc_pull == CcPull::Sink {
                Anamode::Sink
            } else {
                Anamode::Source
            });
            w.set_anasubmode(match cc_pull {
                CcPull::SourceDefaultUsb => 1,
                CcPull::Source1_5A => 2,
                CcPull::Source3_0A => 3,
                _ => 0,
            });
            w.set_ccenable(if cc_pull == CcPull::Disabled {
                Ccenable::Disabled
            } else {
                Ccenable::Both
            });
        });

        // Software trim according to RM0481, p. 2650/2668
        #[cfg(stm32h5)]
        T::REGS.cfgr3().modify(|w| match cc_pull {
            CcPull::Source1_5A => {
                let trim_1a5_cc1 = unsafe { *(0x08FF_F844 as *const u32) & 0xF };
                let trim_1a5_cc2 = unsafe { ((*(0x08FF_F844 as *const u32)) >> 16) & 0xF };

                w.set_trim_cc1_rp(trim_1a5_cc1 as u8);
                w.set_trim_cc2_rp(trim_1a5_cc2 as u8);
            }
            _ => {
                let trim_3a0_cc1 = unsafe { (*(0x4002_242C as *const u32) >> 4) & 0xF };
                let trim_3a0_cc2 = unsafe { ((*(0x4002_242C as *const u32)) >> 12) & 0xF };

                w.set_trim_cc1_rp(trim_3a0_cc1 as u8);
                w.set_trim_cc2_rp(trim_3a0_cc2 as u8);
            }
        });

        // Software trim according to RM0456, p. 3480/3462
        #[cfg(stm32u5)]
        T::REGS.cfgr3().modify(|w| match cc_pull {
            CcPull::Source1_5A => {
                let trim_1a5_cc1 = unsafe { *(0x0BFA_07A7 as *const u8) & 0xF };
                let trim_1a5_cc2 = unsafe { *(0x0BFA_07A8 as *const u8) & 0xF };

                w.set_trim_cc1_rp(trim_1a5_cc1);
                w.set_trim_cc2_rp(trim_1a5_cc2);
            }
            _ => {
                let trim_3a0_cc1 = unsafe { *(0x0BFA_0545 as *const u8) & 0xF };
                let trim_3a0_cc2 = unsafe { *(0x0BFA_0547 as *const u8) & 0xF };

                w.set_trim_cc1_rp(trim_3a0_cc1);
                w.set_trim_cc2_rp(trim_3a0_cc2);
            }
        });

        // Disable dead-battery pull-down resistors which are enabled by default on boot.
        critical_section::with(|cs| {
            init(
                cs,
                #[cfg(not(stm32n6))]
                false,
                #[cfg(peri_ucpd2)]
                false,
            );
        });
    }

    /// Returns the current voltage level of CC1 and CC2 pin as tuple.
    ///
    /// Interpretation of the voltage levels depends on the configured CC line
    /// pull-up/pull-down resistance.
    ///
    /// This also marks the current state as seen: [`wait_for_vstate_change`](Self::wait_for_vstate_change)
    /// returns once the state changes after this call.
    pub fn vstate(&self) -> (CcVState, CcVState) {
        // Clear the event flags before reading the state, so that a change after the read sets
        // them again instead of being lost.
        T::REGS.icr().write(|w| {
            w.set_typecevt1cf(true);
            w.set_typecevt2cf(true);
        });
        let sr = T::REGS.sr().read();
        (sr.typec_vstate_cc1(), sr.typec_vstate_cc2())
    }

    /// Waits for a change in voltage state on either CC line.
    ///
    /// Returns the current voltage state once the hardware has reported a change on CC1 or CC2
    /// since the state was last read, by [`vstate`](Self::vstate) or by this function. Changes
    /// are latched: one that happened before this call makes it return immediately, so a loop like
    ///
    /// ```ignore
    /// let (mut cc1, _) = cc_phy.vstate();
    /// while cc1 != wanted {
    ///     (cc1, _) = cc_phy.wait_for_vstate_change().await;
    /// }
    /// ```
    ///
    /// can't miss a change. The returned state can be equal to the one seen before, for example
    /// if the line changed and changed back.
    pub async fn wait_for_vstate_change(&mut self) -> (CcVState, CcVState) {
        let _on_drop = OnDrop::new(|| self.enable_cc_interrupts(false));
        poll_fn(|cx| {
            let sr = T::REGS.sr().read();
            if sr.typecevt1() || sr.typecevt2() {
                // Clears the flags, then reads the state.
                Poll::Ready(self.vstate())
            } else {
                T::state().cc_waker.register(cx.waker());
                self.enable_cc_interrupts(true);
                Poll::Pending
            }
        })
        .await
    }

    fn enable_cc_interrupts(&self, enable: bool) {
        if enable {
            T::REGS.imr().set_bits(|w| {
                w.set_typecevt1ie(true);
                w.set_typecevt2ie(true);
            });
        } else {
            T::REGS.imr().clear_bits(|w| {
                w.set_typecevt1ie(false);
                w.set_typecevt2ie(false);
            });
        }
    }
}

/// Start of Packet.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sop {
    /// SOP
    Sop,
    /// SOP'
    SopPrime,
    /// SOP''
    SopDoublePrime,
    /// SOP'_Debug
    SopPrimeDebug,
    /// SOP''_Debug
    SopDoublePrimeDebug,
}

impl Sop {
    fn value(&self) -> u32 {
        match self {
            Sop::Sop => 0b10001_11000_11000_11000,
            Sop::SopPrime => 0b00110_00110_11000_11000,
            Sop::SopDoublePrime => 0b00110_11000_00110_11000,
            Sop::SopPrimeDebug => 0b00110_11001_11001_11000,
            Sop::SopDoublePrimeDebug => 0b10001_00110_11001_11000,
        }
    }
}

/// Receive Error.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxError {
    /// Incorrect CRC or truncated message (a line becoming static before EOP is met).
    Crc,

    /// Provided buffer was too small for the received message.
    Overrun,

    /// Hard Reset received before or during reception.
    HardReset,
}

/// Transmit Error.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxError {
    /// Concurrent receive in progress or excessive noise on the line.
    Discarded,

    /// Hard Reset received before or during transmission.
    HardReset,
}

/// Power Delivery (PD) PHY.
pub struct PdPhy<'d, T: Instance> {
    /// Only set for the PD part from [`Ucpd::split_pd_phy`]. Declared first, so that reception
    /// is switched off before the RX DMA is stopped.
    _part: Option<PdPart<T>>,
    rx_ring: ReadableRingBuffer<'d, u8>,
    tx_dma: ChannelAndRequest<'d>,
}

impl<'d, T: Instance> PdPhy<'d, T> {
    fn new(
        rx_dma: ChannelAndRequest<'d>,
        tx_dma: ChannelAndRequest<'d>,
        rx_buf: &'d mut [u8],
        part: Option<PdPart<T>>,
    ) -> Self {
        assert!(rx_buf.len() >= MAX_MESSAGE_LEN && rx_buf.len() <= 0xFFFF);

        let rx_ring = unsafe {
            ReadableRingBuffer::new(
                rx_dma.channel,
                rx_dma.request,
                T::REGS.rxdr().as_ptr() as *mut u8,
                rx_buf,
                TransferOptions::default(),
            )
        };

        Self {
            _part: part,
            rx_ring,
            tx_dma,
        }
    }

    /// Starts receiving on the given CC line.
    fn start_reception(&mut self, cc_sel: CcSel) {
        let r = T::REGS;

        // The RX DMA runs for as long as reception is on: RXDR holds a single byte, and a byte
        // arriving while no DMA is running is lost, and leaves a DMA request pending that the
        // next DMA would serve with a stale byte. Such a byte is discarded by `flush_rx`.
        self.rx_ring.start();
        r.icr().write(|w| w.set_rxhrstdetcf(true));
        self.flush_rx();
        r.imr().set_bits(|w| {
            w.set_rxorddetie(true);
            w.set_rxmsgendie(true);
        });

        enable_pd::<T>(cc_sel);
    }

    /// Stops receiving.
    fn stop_reception(&mut self) {
        let r = T::REGS;

        modify_cr::<T>(|w| w.set_phyrxen(false));
        // Let the DMA, which is still running, take a byte that may still be in RXDR, so that no
        // DMA request is left pending.
        while r.sr().read().rxne() {}
        r.imr().clear_bits(|w| {
            w.set_rxorddetie(false);
            w.set_rxmsgendie(false);
            w.set_rxhrstdetie(false);
        });
        self.rx_ring.stop();
    }

    /// Discards everything received so far and restarts reception.
    ///
    /// Used when the message bookkeeping no longer matches the ring buffer: after a Hard Reset,
    /// an overflow of the ring buffer or the message queue, or a late interrupt. Reception is
    /// switched off meanwhile, so that no message can be half in, half out.
    fn resync(&mut self) {
        modify_cr::<T>(|w| w.set_phyrxen(false));
        self.flush_rx();
        modify_cr::<T>(|w| w.set_phyrxen(true));
    }

    /// [`Self::resync`] for the case that messages were lost.
    fn overrun(&mut self) -> RxError {
        self.resync();
        RxError::Overrun
    }

    /// Empties the ring buffer and the message bookkeeping, while the RX DMA is running and no
    /// message is being received.
    fn flush_rx(&mut self) {
        let r = T::REGS;

        // Let the DMA take a byte that may still be in RXDR.
        while r.sr().read().rxne() {}
        self.rx_ring.clear();

        critical_section::with(|cs| T::state().rx.borrow_ref_mut(cs).reset());
        r.icr().write(|w| {
            w.set_rxmsgendcf(true);
            w.set_rxovrcf(true);
            w.set_rxorddetcf(true);
        });
    }

    /// Receives a PD message into the provided buffer.
    ///
    /// Returns the number of received bytes or an error.
    pub async fn receive(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.receive_with_sop(buf).await.map(|(_sop, size)| size)
    }

    /// Receives SOP and a PD message into the provided buffer.
    ///
    /// Messages are received continuously into the ring buffer given when PD was enabled, so
    /// messages that arrive between two calls are not lost. Returns the start of packet type and
    /// number of received bytes of the oldest received message, or an error:
    ///
    /// - [`RxError::Crc`] if the message was received with an error.
    /// - [`RxError::Overrun`] if `buf` is too small for the message, or if received messages had
    ///   to be discarded: the ring buffer or the message queue overflowed, or the UCPD interrupt
    ///   was served too late (more than ~300 us after the end of a message) to tell messages
    ///   apart.
    /// - [`RxError::HardReset`] if a Hard Reset was received. Messages received before it are
    ///   discarded.
    pub async fn receive_with_sop(&mut self, buf: &mut [u8]) -> Result<(Sop, usize), RxError> {
        let r = T::REGS;

        poll_fn(|cx| {
            if r.sr().read().rxhrstdet() {
                // Clean and re-enable hard reset receive interrupt.
                r.icr().write(|w| w.set_rxhrstdetcf(true));
                r.imr().set_bits(|w| w.set_rxhrstdetie(true));
                self.resync();
                return Poll::Ready(Err(RxError::HardReset));
            }

            T::state().rx_waker.register(cx.waker());
            match critical_section::with(|cs| T::state().rx.borrow_ref_mut(cs).pop()) {
                Err(Desync) => Poll::Ready(Err(self.overrun())),
                Ok(None) => Poll::Pending,
                Ok(Some((msg, queued))) => Poll::Ready(self.read_message(msg, queued, buf)),
            }
        })
        .await
    }

    /// Takes the bytes of `msg` out of the ring buffer.
    ///
    /// `queued` is the number of bytes of the messages queued after it.
    fn read_message(&mut self, msg: RxMessage, queued: usize, buf: &mut [u8]) -> Result<(Sop, usize), RxError> {
        let len = msg.len as usize;

        // The ring buffer holds this message, the queued ones, and at most one message that is
        // still being received. Anything else means bytes that belong to no recorded message.
        let plausible = self
            .rx_ring
            .len()
            .is_ok_and(|available| available >= len && available <= len + queued + MAX_MESSAGE_LEN);
        if !plausible {
            return Err(self.overrun());
        }

        if let Some(error) = msg.error {
            self.discard(len)?;
            return Err(error);
        }
        if len > buf.len() {
            self.discard(len)?;
            return Err(RxError::Overrun);
        }
        self.read_exact(&mut buf[..len])?;

        let sop = match msg.sop {
            Rxordset::Sop => Sop::Sop,
            Rxordset::SopPrime => Sop::SopPrime,
            Rxordset::SopDoublePrime => Sop::SopDoublePrime,
            Rxordset::SopPrimeDebug => Sop::SopPrimeDebug,
            Rxordset::SopDoublePrimeDebug => Sop::SopDoublePrimeDebug,
            Rxordset::CableReset => return Err(RxError::HardReset),
            // Extension headers are not supported
            _ => unreachable!(),
        };

        Ok((sop, len))
    }

    /// Fills `buf` from the ring buffer.
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), RxError> {
        let mut read = 0;
        while read < buf.len() {
            match self.rx_ring.read(&mut buf[read..]) {
                Ok((n, _)) if n > 0 => read += n,
                // Fewer bytes than expected, or the ring buffer overflowed.
                _ => return Err(self.overrun()),
            }
        }
        Ok(())
    }

    /// Takes `len` bytes out of the ring buffer and drops them.
    fn discard(&mut self, mut len: usize) -> Result<(), RxError> {
        let mut scratch = [0u8; 32];
        while len > 0 {
            let chunk = len.min(scratch.len());
            self.read_exact(&mut scratch[..chunk])?;
            len -= chunk;
        }
        Ok(())
    }

    /// Transmits an SOP PD message.
    pub async fn transmit(&mut self, buf: &[u8]) -> Result<(), TxError> {
        self.transmit_with_sop(Sop::Sop, buf).await
    }

    /// Transmits a PD message with a given SOP.
    pub async fn transmit_with_sop(&mut self, sop: Sop, buf: &[u8]) -> Result<(), TxError> {
        let r = T::REGS;

        // When a previous transmission was dropped before it had finished it
        // might still be running because there is no way to abort an ongoing
        // message transmission. Wait for it to finish but ignore errors.
        if r.cr().read().txsend()
            && let Err(TxError::HardReset) = Self::wait_tx_done().await
        {
            return Err(TxError::HardReset);
        }

        // Clear the TX interrupt flags.
        T::REGS.icr().write(|w| {
            w.set_txmsgdisccf(true);
            w.set_txmsgsentcf(true);
        });

        r.tx_ordsetr().write(|w| w.set_txordset(sop.value()));

        // Start the DMA and let it do its thing in the background.
        let _dma = unsafe {
            self.tx_dma
                .write(buf, r.txdr().as_ptr() as *mut u8, TransferOptions::default())
        };

        // Configure and start the transmission.
        r.tx_payszr().write(|w| w.set_txpaysz(buf.len() as _));
        modify_cr::<T>(|w| {
            w.set_txmode(Txmode::Packet);
            w.set_txsend(true);
        });

        Self::wait_tx_done().await
    }

    async fn wait_tx_done() -> Result<(), TxError> {
        let _on_drop = OnDrop::new(|| Self::enable_tx_interrupts(false));
        poll_fn(|cx| {
            let r = T::REGS;
            let sr = r.sr().read();
            if sr.rxhrstdet() {
                // Clean and re-enable hard reset receive interrupt.
                r.icr().write(|w| w.set_rxhrstdetcf(true));
                r.imr().set_bits(|w| w.set_rxhrstdetie(true));
                Poll::Ready(Err(TxError::HardReset))
            } else if sr.txmsgdisc() {
                Poll::Ready(Err(TxError::Discarded))
            } else if sr.txmsgsent() {
                Poll::Ready(Ok(()))
            } else {
                T::state().tx_waker.register(cx.waker());
                Self::enable_tx_interrupts(true);
                Poll::Pending
            }
        })
        .await
    }

    fn enable_tx_interrupts(enable: bool) {
        if enable {
            T::REGS.imr().set_bits(|w| {
                w.set_txmsgdiscie(true);
                w.set_txmsgsentie(true);
            });
        } else {
            T::REGS.imr().clear_bits(|w| {
                w.set_txmsgdiscie(false);
                w.set_txmsgsentie(false);
            });
        }
    }

    /// Transmit a hard reset.
    pub async fn transmit_hardreset(&mut self) -> Result<(), TxError> {
        // Clear the hardreset interrupt flags.
        T::REGS.icr().write(|w| {
            w.set_hrstdisccf(true);
            w.set_hrstsentcf(true);
        });

        // Trigger hard reset transmission.
        modify_cr::<T>(|w| {
            w.set_txhrst(true);
        });

        let _on_drop = OnDrop::new(|| self.enable_hardreset_interrupts(false));
        poll_fn(|cx| {
            let r = T::REGS;
            let sr = r.sr().read();
            if sr.rxhrstdet() {
                // Clean and re-enable hard reset receive interrupt.
                r.icr().write(|w| w.set_rxhrstdetcf(true));
                r.imr().set_bits(|w| w.set_rxhrstdetie(true));
                Poll::Ready(Err(TxError::HardReset))
            } else if sr.hrstdisc() {
                Poll::Ready(Err(TxError::Discarded))
            } else if sr.hrstsent() {
                Poll::Ready(Ok(()))
            } else {
                T::state().tx_waker.register(cx.waker());
                self.enable_hardreset_interrupts(true);
                Poll::Pending
            }
        })
        .await
    }

    fn enable_hardreset_interrupts(&self, enable: bool) {
        if enable {
            T::REGS.imr().set_bits(|w| {
                w.set_hrstdiscie(true);
                w.set_hrstsentie(true);
            });
        } else {
            T::REGS.imr().clear_bits(|w| {
                w.set_hrstdiscie(false);
                w.set_hrstsentie(false);
            });
        }
    }
}

/// Power Delivery (PD) PHY with PD communication disabled, created by
/// [`Ucpd::split_idle_pd_phy`].
pub struct IdlePdPhy<'d, T: Instance> {
    rx_dma: ChannelAndRequest<'d>,
    tx_dma: ChannelAndRequest<'d>,
    _part: PdPart<T>,
}

impl<'d, T: Instance> IdlePdPhy<'d, T> {
    /// Enables PD communication on the given CC line.
    ///
    /// PD messages are received continuously into `rx_buf`, a ring buffer, until the returned
    /// [`ActivePdPhy`] is dropped, which disables PD reception again. It must hold at least one
    /// message of the largest size (264 bytes); a few times that leaves room for messages that
    /// wait for [`PdPhy::receive`].
    pub fn activate<'a>(&'a mut self, cc_sel: CcSel, rx_buf: &'a mut [u8]) -> ActivePdPhy<'a, T> {
        let rx_dma = ChannelAndRequest {
            channel: self.rx_dma.channel.reborrow(),
            request: self.rx_dma.request,
        };
        let tx_dma = ChannelAndRequest {
            channel: self.tx_dma.channel.reborrow(),
            request: self.tx_dma.request,
        };

        let mut pd_phy = PdPhy::new(rx_dma, tx_dma, rx_buf, None);
        pd_phy.start_reception(cc_sel);
        ActivePdPhy { pd_phy }
    }
}

/// Power Delivery (PD) PHY with PD communication enabled, created by [`IdlePdPhy::activate`].
///
/// Derefs to [`PdPhy`] for receiving and transmitting messages. Dropping it disables PD
/// reception.
pub struct ActivePdPhy<'a, T: Instance> {
    pd_phy: PdPhy<'a, T>,
}

impl<'a, T: Instance> Drop for ActivePdPhy<'a, T> {
    fn drop(&mut self) {
        self.pd_phy.stop_reception();
    }
}

impl<'a, T: Instance> Deref for ActivePdPhy<'a, T> {
    type Target = PdPhy<'a, T>;

    fn deref(&self) -> &Self::Target {
        &self.pd_phy
    }
}

impl<'a, T: Instance> DerefMut for ActivePdPhy<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.pd_phy
    }
}

/// Modifies CR.
///
/// CR is written by both the CC part and the PD part, which may run at different priorities, so
/// the read-modify-write runs in a critical section to keep them from overwriting each other.
///
/// TXSEND, TXHRST and FRSTX are commands that the hardware clears once it acts on them. A value
/// read while one of them was still set must not be written back, as that would issue the command
/// again (e.g. send a message twice), so they are written as 0 unless `f` sets them.
fn modify_cr<T: Instance>(f: impl FnOnce(&mut crate::pac::ucpd::regs::Cr)) {
    critical_section::with(|_| {
        T::REGS.cr().modify(|w| {
            w.set_txsend(false);
            w.set_txhrst(false);
            w.set_frstx(false);
            f(w);
        });
    });
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _marker: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::REGS;
        let state = T::state();
        let sr = r.sr().read();

        if sr.typecevt1() || sr.typecevt2() {
            // Leave the event flags set, `wait_for_vstate_change` clears them. This way a change is
            // not lost if the state has changed back by the time the task runs.
            r.imr().clear_bits(|w| {
                w.set_typecevt1ie(false);
                w.set_typecevt2ie(false);
            });
            state.cc_waker.wake();
        }

        if sr.rxhrstdet() {
            r.imr().clear_bits(|w| w.set_rxhrstdetie(false));
            // A received Hard Reset ends both a pending reception and a pending transmission.
            state.rx_waker.wake();
            state.tx_waker.wake();
        }

        if sr.rxorddet() || sr.rxmsgend() {
            critical_section::with(|cs| state.rx.borrow_ref_mut(cs).on_interrupt(r, sr));
            state.rx_waker.wake();
        }

        if sr.txmsgdisc() || sr.txmsgsent() {
            r.imr().clear_bits(|w| {
                w.set_txmsgdiscie(false);
                w.set_txmsgsentie(false);
            });
            state.tx_waker.wake();
        }

        if sr.hrstdisc() || sr.hrstsent() {
            r.imr().clear_bits(|w| {
                w.set_hrstdiscie(false);
                w.set_hrstsentie(false);
            });
            state.tx_waker.wake();
        }

        // Wake the tasks to clear and re-enabled interrupts.
    }
}

/// Size of the largest message: header, extended header and 260 data bytes of an unchunked
/// extended message.
const MAX_MESSAGE_LEN: usize = 264;

/// Number of received messages that can wait for [`PdPhy::receive`].
const RX_QUEUE_LEN: usize = 8;

/// The message bookkeeping no longer matches the RX ring buffer, see [`PdPhy::resync`].
struct Desync;

/// A message received into the RX ring buffer, recorded at its end.
#[derive(Clone, Copy)]
struct RxMessage {
    /// Number of bytes the message occupies in the ring buffer (RXPAYSZ).
    len: u16,
    sop: Rxordset,
    error: Option<RxError>,
}

/// Bookkeeping of the messages in the RX ring buffer, shared by the interrupt and the receiver.
///
/// The RX DMA runs continuously into the ring buffer, so the bytes of all messages follow each
/// other there. Their boundaries are only known from RXPAYSZ, RX_ORDSETR and the error flags,
/// which the next message overwrites, so the interrupt records them for every message.
struct RxState {
    queue: heapless::Deque<RxMessage, RX_QUEUE_LEN>,
    /// SOP of the message currently being received, recorded at its start.
    sop: Option<Rxordset>,
    /// The bookkeeping no longer matches the ring buffer.
    desync: bool,
}

impl RxState {
    const fn new() -> Self {
        Self {
            queue: heapless::Deque::new(),
            sop: None,
            desync: false,
        }
    }

    fn reset(&mut self) {
        self.queue.clear();
        self.sop = None;
        self.desync = false;
    }

    /// Takes the oldest message, along with the number of bytes of the messages queued after it.
    fn pop(&mut self) -> Result<Option<(RxMessage, usize)>, Desync> {
        if self.desync {
            return Err(Desync);
        }
        let msg = self.queue.pop_front();
        let queued = self.queue.iter().map(|m| m.len as usize).sum();
        Ok(msg.map(|msg| (msg, queued)))
    }

    /// Records the start (RXORDDET) and end (RXMSGEND) of messages.
    ///
    /// RX_ORDSETR is valid from the start of a message until the start of the next one, and
    /// RXPAYSZ and the error flags from its end until the next one starts, at least ~300 us
    /// (inter-frame gap, preamble and SOP). If the interrupt is served later than that, the
    /// values can't be attributed reliably, and the receiver resynchronizes instead.
    fn on_interrupt(&mut self, r: crate::pac::ucpd::Ucpd, sr: crate::pac::ucpd::regs::Sr) {
        if sr.rxorddet() && sr.rxmsgend() {
            // The start and the end of a message, or the end of one and the start of the next,
            // were both pending: served too late to tell which values belong to which message.
            r.icr().write(|w| {
                w.set_rxorddetcf(true);
                w.set_rxmsgendcf(true);
                w.set_rxovrcf(true);
            });
            self.sop = None;
            self.desync = true;
        } else if sr.rxorddet() {
            r.icr().write(|w| w.set_rxorddetcf(true));
            if self.sop.is_some() {
                // The end of the previous message was missed.
                self.desync = true;
            }
            self.sop = Some(r.rx_ordsetr().read().rxordset());
        } else if sr.rxmsgend() {
            let len = r.rx_payszr().read().rxpaysz();
            let error = if sr.rxovr() {
                Some(RxError::Overrun)
            } else if sr.rxerr() {
                Some(RxError::Crc)
            } else {
                None
            };
            r.icr().write(|w| {
                w.set_rxmsgendcf(true);
                w.set_rxovrcf(true);
            });

            let sop = self.sop.take();
            // If the next message has already started, the values read above may belong to it.
            let next_started = r.sr().read().rxorddet();
            match sop {
                Some(sop) if !next_started => {
                    if self.queue.push_back(RxMessage { len, sop, error }).is_err() {
                        self.desync = true;
                    }
                }
                _ => self.desync = true,
            }
        }
    }
}

struct State {
    // CcPhy and PdPhy can be used from different tasks at the same time, and an AtomicWaker only
    // holds one waker, so each wait has its own.
    cc_waker: AtomicWaker,
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
    // Inverted logic for a default state of 0 so that the data goes into the .bss section.
    drop_not_ready: AtomicBool,
    rx: Mutex<RefCell<RxState>>,
}

impl State {
    pub const fn new() -> Self {
        Self {
            cc_waker: AtomicWaker::new(),
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            drop_not_ready: AtomicBool::new(false),
            rx: Mutex::new(RefCell::new(RxState::new())),
        }
    }
}

trait SealedInstance {
    const REGS: crate::pac::ucpd::Ucpd;
    fn state() -> &'static State;
}

/// UCPD instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + RccPeripheral {
    /// Interrupt for this instance.
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

foreach_interrupt!(
    ($inst:ident, ucpd, UCPD, GLOBAL, $irq:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            const REGS: crate::pac::ucpd::Ucpd = crate::pac::$inst;

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
);

pin_trait!(Cc1Pin, Instance);
pin_trait!(Cc2Pin, Instance);

dma_trait!(TxDma, Instance);
dma_trait!(RxDma, Instance);
