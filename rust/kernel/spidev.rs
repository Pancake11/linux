use crate::{
    bindings,
    error::{code::*, from_kernel_result, Error, Result},
    str::CString,
};
use alloc::boxed::Box;
use core::{fmt, marker::PhantomPinned, pin::Pin};
use macros::vtable;

/// This is needed to created the SpiMethods trait
#[derive(Clone, Copy)]
pub struct SpiDevice(*mut bindings::spi_device);

impl SpiDevice {
    /// # Safety
    /// Yes this is unsafe
    pub unsafe fn from_ptr(ptr: *mut bindings::spi_device) -> Self {
        SpiDevice(ptr)
    }

    /// Converts a SpiDevice to a raw pointer to C's spi_device
    pub fn to_ptr(&mut self) -> *mut bindings::spi_device {
        self.0
    }

    pub fn set_max_speed(&mut self, max_speed: u32) {
        let mut speed = unsafe { (*(self.to_ptr())).max_speed_hz };
        if speed > max_speed {
            speed = max_speed;
        }
    }
}

/// Methods declared in the spi_device struct in C
#[vtable]
pub trait SpiMethods {
    /// similar to the C function
    fn probe(mut _spi_dev: SpiDevice) -> Result {
        Err(EINVAL)
    }

    /// similar to the C function
    fn remove(mut _spi_dev: SpiDevice) -> Result {
        Err(EINVAL)
    }

    /// similar to the C function
    fn shutdown(mut _spi_dev: SpiDevice) -> Result {
        Err(EINVAL)
    }
}

/// Represents the creation and registration of a spi Module
pub struct Registration {
    this_module: &'static crate::ThisModule,
    registered: bool,
    spi_drv: bindings::spi_driver,
    name: Option<CString>,
    _pin: PhantomPinned,
}

impl Registration {
    /// Generates a new registration for spi device it is neither pinned or registered
    pub fn new(this_module: &'static crate::ThisModule) -> Self {
        Self {
            this_module,
            registered: false,
            spi_drv: bindings::spi_driver::default(),
            name: None,
            _pin: PhantomPinned,
        }
    }

    /// Generated a pinned and registered spi device.
    pub fn new_pinned<T: SpiMethods>(
        this_module: &'static crate::ThisModule,
        name: fmt::Arguments<'_>,
    ) -> Result<Pin<Box<Self>>> {
        let mut r = Pin::from(Box::try_new(Self::new(this_module))?);

        r.as_mut().register::<T>(name)?;

        Ok(r)
    }

    /// Callback used for the probe spi method
    unsafe extern "C" fn probe_callback<T: SpiMethods>(
        spi_dev: *mut bindings::spi_device,
    ) -> core::ffi::c_int {
        // SAFETY: yes
        from_kernel_result!(
            unsafe { T::probe(SpiDevice::from_ptr(spi_dev))? };
            Ok(0)
        )
    }

    /// Callback used for the remove spi method
    unsafe extern "C" fn remove_callback<T: SpiMethods>(spi_dev: *mut bindings::spi_device) {
        // SAFETY: yes
        unsafe { T::remove(SpiDevice::from_ptr(spi_dev)).unwrap() };
    }

    /// Callback used for the shutdown spi method
    unsafe extern "C" fn shutdown_callback<T: SpiMethods>(spi_dev: *mut bindings::spi_device) {
        // SAFETY: yes
        unsafe { T::shutdown(SpiDevice::from_ptr(spi_dev)).unwrap() };
    }

    /// Registers a spi device.
    pub fn register<T: SpiMethods>(self: Pin<&mut Self>, name: fmt::Arguments<'_>) -> Result {
        let this = unsafe { self.get_unchecked_mut() };
        if this.registered {
            return Err(EINVAL);
        }
        let name = CString::try_from_fmt(name)?;

        this.spi_drv.driver.name = name.as_char_ptr();
        this.spi_drv.probe = Some(Self::probe_callback::<T>);
        this.spi_drv.remove = Some(Self::remove_callback::<T>);
        this.spi_drv.shutdown = Some(Self::shutdown_callback::<T>);

        this.registered = true;

        let ret = unsafe { bindings::__spi_register_driver(this.this_module.0, &mut this.spi_drv) };
        if ret < 0 {
            this.registered = false;
            return Err(Error::from_kernel_errno(ret));
        }

        this.name = Some(name);
        Ok(())
    }
}

pub struct Spi;

impl Spi {
    fn write_then_read_wrapper(dev: &mut SpiDevice, tx: &[u8], rx: &mut [u8]) -> Result {
        let res = unsafe {
            bindings::spi_write_then_read(
                dev.to_ptr(),
                tx.as_ptr() as *const core::ffi::c_void,
                tx.len() as core::ffi::c_uint,
                rx.as_ptr() as *mut core::ffi::c_void,
                rx.len() as core::ffi::c_uint,
            )
        };

        if res != 0 {
            return Err(Error::from_kernel_errno(res));
        }
        Ok(())
    }

    pub fn write(dev: &mut SpiDevice, tx: &[u8]) -> Result {
        Spi::write_then_read_wrapper(dev, tx, &mut [0u8; 0])
    }

    pub fn read(dev: &mut SpiDevice, rx: &mut [u8]) -> Result {
        Spi::write_then_read_wrapper(dev, &[0u8, 0], rx)
    }
}
