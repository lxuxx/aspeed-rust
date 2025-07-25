// Licensed under the Apache-2.0 license

pub struct DummyDelay;

impl embedded_hal::delay::DelayNs for DummyDelay {
    fn delay_ns(&mut self, ns: u32) {
        for _ in 0..(ns / 100) {
            cortex_m::asm::nop();
        }
    }
}

#[repr(align(32))]
pub struct DmaBuffer<const N: usize> {
    pub buf: [u8; N],
}

impl<const N: usize> Default for DmaBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> DmaBuffer<N> {
    #[must_use]
    pub const fn new() -> Self {
        Self { buf: [0; N] }
    }

    #[must_use]
    pub fn as_ptr(&self) -> *const u8 {
        self.buf.as_ptr()
    }

    #[must_use]
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.buf.as_mut_ptr()
    }

    #[must_use]
    pub fn len(&self) -> usize {
        N
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        N == 0
    }

    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.buf
    }

    pub fn as_mut_slice(&mut self, start: usize, end: usize) -> &mut [u8] {
        &mut self.buf[start..end]
    }
}

use core::ops::{Index, IndexMut};

impl<const N: usize> Index<usize> for DmaBuffer<N> {
    type Output = u8;
    fn index(&self, idx: usize) -> &Self::Output {
        &self.buf[idx]
    }
}

impl<const N: usize> IndexMut<usize> for DmaBuffer<N> {
    fn index_mut(&mut self, idx: usize) -> &mut Self::Output {
        &mut self.buf[idx]
    }
}
