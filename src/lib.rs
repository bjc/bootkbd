//! Simple USB host-side driver for boot protocol keyboards.

#![no_std]

use log::{debug, error};
use usb_host::{
    ConfigurationDescriptor, DescriptorType, DeviceDescriptor, Direction, Driver, DriverError,
    Endpoint, RequestCode, RequestDirection, RequestKind, RequestRecipient, RequestType,
    TransferError, TransferType, USBHost, WValue,
};

use core::mem::{self, MaybeUninit};
use core::ptr;

// How long to wait before talking to the device again after setting
// its address. cf §9.2.6.3 of USB 2.0
const SETTLE_DELAY: usize = 2;

// How many total devices this driver can support.
const MAX_DEVICES: usize = 1;

// And how many endpoints we can support per-device.
const MAX_ENDPOINTS: usize = 2;

pub struct BootKeyboard<F> {
    devices: [Option<Device>; MAX_DEVICES],
    callback: F,
}
impl<F> core::fmt::Debug for BootKeyboard<F> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "BootKeyboard")
    }
}

impl<F> BootKeyboard<F>
where
    F: FnMut(u8, &[u8]),
{
    pub fn new(callback: F) -> Self {
        Self {
            devices: [None; MAX_DEVICES],
            callback: callback,
        }
    }
}

impl<F> Driver for BootKeyboard<F>
where
    F: FnMut(u8, &[u8]),
{
    fn want_device(&self, _device: &DeviceDescriptor) -> bool {
        true
    }

    fn add_device(&mut self, _device: DeviceDescriptor, address: u8) -> Result<(), DriverError> {
        for i in 0..self.devices.len() {
            if self.devices[i].is_none() {
                self.devices[i] = Some(Device::new(address));
                return Ok(());
            }
        }
        Err(DriverError::Permanent(address, "out of devices"))
    }

    fn remove_device(&mut self, address: u8) {
        for i in 0..self.devices.len() {
            if let Some(ref dev) = self.devices[i] {
                if dev.addr == address {
                    self.devices[i] = None;
                    return;
                }
            }
        }
    }

    fn tick(&mut self, millis: usize, host: &mut dyn USBHost) -> Result<(), DriverError> {
        for d in &mut self.devices[..] {
            if let Some(ref mut dev) = d {
                if let Err(TransferError::Permanent(e)) = dev.fsm(millis, host, &mut self.callback)
                {
                    return Err(DriverError::Permanent(dev.addr, e));
                }
            }
        }
        Ok(())
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum DeviceState {
    Addressed,
    WaitForSettle(usize),
    GetConfig,
    SetConfig(u8),
    SetIdle,
    SetReport,
    Running,
}

struct Device {
    addr: u8,
    ep0: EP,
    endpoints: [Option<EP>; MAX_ENDPOINTS],
    state: DeviceState,
}

impl Device {
    fn new(addr: u8) -> Self {
        let endpoints: [Option<EP>; MAX_ENDPOINTS] = {
            let mut eps: [MaybeUninit<Option<EP>>; MAX_ENDPOINTS] =
                unsafe { mem::MaybeUninit::uninit().assume_init() };
            for ep in &mut eps[..] {
                unsafe { ptr::write(ep.as_mut_ptr(), None) }
            }
            unsafe { mem::transmute(eps) }
        };

        Self {
            addr: addr,
            ep0: EP::new(addr, 0, TransferType::Control, Direction::In),
            endpoints: endpoints,
            state: DeviceState::Addressed,
        }
    }

    fn fsm(
        &mut self,
        millis: usize,
        host: &mut dyn USBHost,
        callback: &mut dyn FnMut(u8, &[u8]),
    ) -> Result<(), TransferError> {
        // TODO: either we need another `control_transfer` that
        // doesn't take data, or this `none` value needs to be put in
        // the usb-host layer. None of these options are good.
        let none: Option<&mut [u8]> = None;
        unsafe {
            static mut LAST_STATE: DeviceState = DeviceState::Addressed;
            if LAST_STATE != self.state {
                debug!("{:?} -> {:?}", LAST_STATE, self.state);
                LAST_STATE = self.state;
            }
        }

        match self.state {
            DeviceState::Addressed => {
                self.state = DeviceState::WaitForSettle(millis + SETTLE_DELAY)
            }

            DeviceState::WaitForSettle(until) => {
                if millis > until {
                    let mut dev_desc: MaybeUninit<DeviceDescriptor> = MaybeUninit::uninit();
                    let buf = unsafe { to_slice_mut(&mut dev_desc) };
                    let len = host.control_transfer(
                        &mut self.ep0,
                        RequestType::from((
                            RequestDirection::DeviceToHost,
                            RequestKind::Standard,
                            RequestRecipient::Device,
                        )),
                        RequestCode::GetDescriptor,
                        WValue::from((0, DescriptorType::Device as u8)),
                        0,
                        Some(buf),
                    )?;
                    assert!(len == mem::size_of::<DeviceDescriptor>());
                    self.state = DeviceState::GetConfig
                }
            }

            DeviceState::GetConfig => {
                let mut conf_desc: MaybeUninit<ConfigurationDescriptor> = MaybeUninit::uninit();
                let buf = unsafe { to_slice_mut(&mut conf_desc) };
                let len = host.control_transfer(
                    &mut self.ep0,
                    RequestType::from((
                        RequestDirection::DeviceToHost,
                        RequestKind::Standard,
                        RequestRecipient::Device,
                    )),
                    RequestCode::GetDescriptor,
                    WValue::from((0, DescriptorType::Configuration as u8)),
                    0,
                    Some(buf),
                )?;
                assert!(len == mem::size_of::<ConfigurationDescriptor>());
                let conf_desc = unsafe { conf_desc.assume_init() };

                // TODO: do a real allocation later.
                assert!(conf_desc.w_total_length < 64);
                let mut buf: [u8; 64] = [0; 64];
                let mut tmp = &mut buf[..conf_desc.w_total_length as usize];
                let len = host.control_transfer(
                    &mut self.ep0,
                    RequestType::from((
                        RequestDirection::DeviceToHost,
                        RequestKind::Standard,
                        RequestRecipient::Device,
                    )),
                    RequestCode::GetDescriptor,
                    WValue::from((0, DescriptorType::Configuration as u8)),
                    0,
                    Some(&mut tmp),
                )?;
                assert!(len == conf_desc.w_total_length as usize);

                // TODO: browse configs and pick the "best" one. But
                // this should always be ok, at least.
                self.state = DeviceState::SetConfig(1)
            }

            DeviceState::SetConfig(config_index) => {
                host.control_transfer(
                    &mut self.ep0,
                    RequestType::from((
                        RequestDirection::HostToDevice,
                        RequestKind::Standard,
                        RequestRecipient::Device,
                    )),
                    RequestCode::SetConfiguration,
                    WValue::from((config_index, 0)),
                    0,
                    none,
                )?;

                self.endpoints[0] = Some(EP::new(
                    self.addr,
                    1,
                    TransferType::Interrupt,
                    Direction::In,
                ));

                self.state = DeviceState::SetIdle
            }

            DeviceState::SetIdle => {
                host.control_transfer(
                    &mut self.ep0,
                    RequestType::from((
                        RequestDirection::HostToDevice,
                        RequestKind::Class,
                        RequestRecipient::Interface,
                    )),
                    RequestCode::GetInterface,
                    WValue::from((0, 0)),
                    0,
                    none,
                )?;
                self.state = DeviceState::SetReport
            }

            DeviceState::SetReport => {
                let mut report: [u8; 1] = [0];
                host.control_transfer(
                    &mut self.ep0,
                    RequestType::from((
                        RequestDirection::HostToDevice,
                        RequestKind::Class,
                        RequestRecipient::Interface,
                    )),
                    RequestCode::SetConfiguration,
                    WValue::from((0, 2)),
                    0,
                    Some(&mut report),
                )?;

                self.state = DeviceState::Running
            }

            DeviceState::Running => {
                let mut buf: [u8; 8] = [0; 8];
                if let Some(ref mut ep) = self.endpoints[0] {
                    match host.in_transfer(ep, &mut buf) {
                        Err(TransferError::Permanent(msg)) => error!("reading report: {}", msg),
                        Err(TransferError::Retry(_)) => return Ok(()),
                        Ok(_) => {
                            callback(self.addr, &buf);
                        }
                    }
                }
            }
        }

        Ok(())
    }
}

unsafe fn to_slice_mut<T>(v: &mut T) -> &mut [u8] {
    let ptr = v as *mut T as *mut u8;
    let len = mem::size_of::<T>();
    core::slice::from_raw_parts_mut(ptr, len)
}

struct EP {
    addr: u8,
    num: u8,
    transfer_type: TransferType,
    direction: Direction,
    in_toggle: bool,
    out_toggle: bool,
}

impl EP {
    fn new(addr: u8, num: u8, transfer_type: TransferType, direction: Direction) -> Self {
        Self {
            addr: addr,
            num: num,
            transfer_type: transfer_type,
            direction: direction,
            in_toggle: false,
            out_toggle: false,
        }
    }
}

impl Endpoint for EP {
    fn address(&self) -> u8 {
        self.addr
    }

    fn endpoint_num(&self) -> u8 {
        self.num
    }

    fn transfer_type(&self) -> TransferType {
        self.transfer_type
    }

    fn direction(&self) -> Direction {
        self.direction
    }

    fn max_packet_size(&self) -> u16 {
        8
    }

    fn in_toggle(&self) -> bool {
        self.in_toggle
    }

    fn set_in_toggle(&mut self, toggle: bool) {
        self.in_toggle = toggle
    }

    fn out_toggle(&self) -> bool {
        self.out_toggle
    }

    fn set_out_toggle(&mut self, toggle: bool) {
        self.out_toggle = toggle
    }
}
