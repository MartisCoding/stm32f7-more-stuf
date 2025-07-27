#![no_std]
#![no_main]

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

mod fmt;
use cortex_m_rt::entry;
use defmt::{error, info, unwrap};
use embassy_executor::{task, Executor, InterruptExecutor};
use embassy_futures::select::{select, Either};
use embassy_stm32::adc::{Adc, Resolution, RingBufferedAdc, SampleTime, Sequence};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull::Down;
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{ADC1, UART4};
use embassy_stm32::usart::{Uart, InterruptHandler, Config};
use embassy_sync::{channel::Channel, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::Timer;
use static_cell::StaticCell;

static ADC_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, (u16, u16), 10>> = StaticCell::new();
static ENC_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, u16, 10>> = StaticCell::new();

static DMA_BUFFER: StaticCell<[u16; DMA_BUFF_SIZE]> = StaticCell::new();
const DMA_BUFF_SIZE: usize = 128;

bind_interrupts!(struct Irqs {
    UART4 => InterruptHandler<UART4>;
});

static EXECUTOR_ADC: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_ENC: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_UART: StaticCell<Executor> = StaticCell::new();


#[entry]
fn main() -> ! {
    let mut p = embassy_stm32::init(Default::default());
    //ADC Channel from ADC_TASK to UART_TASK
    let adc_ch = ADC_CHANNEL.init(Channel::new());
    
    let adc_ch_rec = adc_ch.receiver();
    let adc_ch_send = adc_ch.sender();
    
    //ENCODER Channel from ENC_TASK to UART_TASK
    let enc_ch = ENC_CHANNEL.init(Channel::new());
    
    let enc_ch_rec = enc_ch.receiver();
    let enc_ch_send = enc_ch.sender();

    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(Resolution::BITS8);
    let dma_buffer = DMA_BUFFER.init([0; DMA_BUFF_SIZE]);
    let mut adc_ring = adc.into_ring_buffered(p.DMA2_CH0, dma_buffer);
    adc_ring.set_sample_sequence(Sequence::One, &mut p.PA5, SampleTime::CYCLES480);
    adc_ring.set_sample_sequence(Sequence::Two, &mut p.PA6, SampleTime::CYCLES480);
    
    interrupt::UART5.set_priority(Priority::P6);
    let send_sp1 = EXECUTOR_ADC.start(interrupt::UART5);
    unwrap!(send_sp1.spawn(adc_task(adc_ring, adc_ch_send)));

    let exti1 = ExtiInput::new(p.PC8, p.EXTI8, Down);
    let exti2 = ExtiInput::new(p.PC9, p.EXTI9, Down);
    
    interrupt::USART1.set_priority(Priority::P7);
    let send_sp2 = EXECUTOR_ENC.start(interrupt::USART1);
    unwrap!(send_sp2.spawn(enc_task(exti1, exti2, enc_ch_send)));
    
    let uart = Uart::new(
        p.UART4,
        p.PC11,
        p.PC10,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH2,
        Config::default(),
    ).unwrap();
    uart.set_baudrate(115200).unwrap();
    let executor = EXECUTOR_UART.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(uart_ttl_task(uart, enc_ch_rec, adc_ch_rec)));
    })
}

#[interrupt]
unsafe fn UART5() {
    EXECUTOR_ADC.on_interrupt();
}

#[interrupt]
unsafe fn USART1() {
    EXECUTOR_ENC.on_interrupt();
}


#[task]
async fn adc_task(
    mut adc_ring: RingBufferedAdc<'static, ADC1>,
    adc_ch_ref: Sender<'static, CriticalSectionRawMutex, (u16, u16), 10>
) {
    let mut measures = [0u16; DMA_BUFF_SIZE / 2];
    info!("Starting knob task");
    loop {
        match adc_ring.read(&mut measures).await {
            Ok(_) => {
                adc_ring.teardown_adc();
                adc_ch_ref.send((measures[0], measures[1])).await
            },
            Err(e) => error!("Adc driver failed read with Error: {}", e),
        }
        Timer::after_millis(10).await;
    }
}
#[task]
async fn enc_task(
    mut exti1: ExtiInput<'static>, 
    mut exti2: ExtiInput<'static>, 
    enc_ch_ref: Sender<'static, CriticalSectionRawMutex, u16, 10>
)  {
    info!("Startin encoder task");
    let mut counter = 0i16;
    loop {
        info!("LOOP HEARTBEAT");
        let res = select(
            exti1.wait_for_falling_edge(),
            exti2.wait_for_falling_edge(),
        ).await;
        match res {
            Either::First(_) => {
                exti2.wait_for_low().await;
                counter += 1;
            }
            Either::Second(_) => {
                exti1.wait_for_low().await;
                counter -= 1;
            }
        }
        info!("Encoder task internal counter: {}", counter as i16);
        enc_ch_ref.send(counter as u16).await;
        Timer::after_millis(10).await;
    }
}
#[task]
async fn uart_ttl_task(
    mut uart: Uart<'static, Async>,
    enc_ch_ref: Receiver<'static, CriticalSectionRawMutex, u16, 10>,
    adc_ch_ref: Receiver<'static, CriticalSectionRawMutex, (u16, u16), 10>,
) {
    
    let mut buff = [0u8; 64];
    let mut i3 = 0;
    loop {
        //info!("Waiting inputs.");
        let (i1, i2) = adc_ch_ref.receive().await;
        if !enc_ch_ref.is_empty() {
            i3 = enc_ch_ref.receive().await;
        }
        //info!("Preparing UART transfer.");
        let s = format_no_std::show(
            &mut buff,
            format_args!("Knob: ({}, {}), Encoder: {}", i1, i2, i3 as isize),
        ).unwrap().as_bytes();
        let res = uart.write(s).await;
        match res {
            Ok(_) => {
                //info!("uart driver written. Knob: ({}, {}), Encoder: {}", i1, i2, i3 as isize);
            }
            Err(e) => error!("uart driver failed write with Error: {}", e),
        }
        //info!("Finished UART transfer.");
        Timer::after_millis(20).await;
    }
}