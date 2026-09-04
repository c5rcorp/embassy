extern crate std;

use super::*;
use core::future::{Future, pending};
use core::task::{Context, Poll, Waker};
use embassy_usb_driver::host::TimeoutConfig;
use std::boxed::Box;

#[derive(Clone)]
struct Allocator {
    fail: bool,
}
struct MockPipe<T, D>(PhantomData<(T, D)>);

impl<'d> UsbHostAllocator<'d> for Allocator {
    type Pipe<T: pipe::Type, D: pipe::Direction> = MockPipe<T, D>;
    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        _: u8,
        _: &EndpointInfo,
        _: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        if self.fail {
            Err(HostError::RequestFailed)
        } else {
            Ok(MockPipe(PhantomData))
        }
    }
}

impl<T: pipe::Type, D: pipe::Direction> UsbPipe<T, D> for MockPipe<T, D> {
    async fn control_in(&mut self, _: &[u8; 8], _: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        pending().await
    }
    async fn control_out(&mut self, _: &[u8; 8], _: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        pending().await
    }
    async fn request_in(&mut self, _: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        pending().await
    }
    async fn request_out(&mut self, _: &[u8], _: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        pending().await
    }
    fn set_timeout(&mut self, _: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
    }
    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
    }
}

#[test]
fn cancelled_enumeration_releases_reserved_address() {
    let state = BusState::new();
    let bus = BusHandle {
        alloc: Allocator { fail: false },
        state: &state,
    };
    let mut config = [0; 256];
    let mut future = Box::pin(bus.enumerate(BusRoute::Direct(Speed::Full), &mut config));
    assert!(
        future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
            .is_pending()
    );
    drop(future);
    assert_eq!(
        state.alloc_address(),
        Some(1),
        "cancelling enumeration leaked address 1"
    );
}

#[test]
fn repeated_timeouts_do_not_exhaust_device_addresses() {
    let state = BusState::new();
    let bus = BusHandle {
        alloc: Allocator { fail: false },
        state: &state,
    };
    let mut config = [0; 256];
    for attempt in 1..=128 {
        let mut future = Box::pin(bus.enumerate(BusRoute::Direct(Speed::Full), &mut config));
        let result = future.as_mut().poll(&mut Context::from_waker(Waker::noop()));
        assert!(matches!(result, Poll::Pending), "attempt {attempt}: {result:?}");
        drop(future);
    }
}

#[test]
fn failed_enumeration_releases_reserved_address() {
    let state = BusState::new();
    let bus = BusHandle {
        alloc: Allocator { fail: true },
        state: &state,
    };
    let mut config = [0; 256];
    let mut future = Box::pin(bus.enumerate(BusRoute::Direct(Speed::Full), &mut config));
    assert!(matches!(
        future.as_mut().poll(&mut Context::from_waker(Waker::noop())),
        Poll::Ready(Err(EnumerationError::NoPipe))
    ));
    drop(future);
    assert_eq!(state.alloc_address(), Some(1));
}
