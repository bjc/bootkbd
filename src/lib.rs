//! Simple USB host-side driver for boot protocol keyboards.

#![no_std]

use log::{debug, error, info};
use usb_host::{
    ConfigurationDescriptor, DescriptorType, DeviceDescriptor, Direction, Driver, DriverError,
    Endpoint, EndpointDescriptor, InterfaceDescriptor, RequestCode, RequestDirection, RequestKind,
    RequestRecipient, RequestType, TransferError, TransferType, USBHost, WValue,
};

use core::convert::TryFrom;
use core::mem::{self, MaybeUninit};
use core::ptr;

// How long to wait before talking to the device again after setting
// its address. cf §9.2.6.3 of USB 2.0
const SETTLE_DELAY: usize = 2;

// How many total devices this driver can support.
const MAX_DEVICES: usize = 1;

// And how many endpoints we can support per-device.
const MAX_ENDPOINTS: usize = 2;

// The maximum size configuration descriptor we can handle.
const CONFIG_BUFFER_LEN: usize = 128;

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

                if (conf_desc.w_total_length as usize) < CONFIG_BUFFER_LEN {
                    return Err(TransferError::Permanent("config descriptor too large"));
                }

                // TODO: do a real allocation later. For now, keep a
                // large-ish static buffer and take an appropriately
                // sized slice into it for the transfer.
                let mut buf: [u8; CONFIG_BUFFER_LEN] = [0; CONFIG_BUFFER_LEN];
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
                let ep = match ep_for_bootkbd(&tmp) {
                    Ok(Some(n)) => n & 0x7f,
                    Ok(None) => Err(TransferError::Permanent("no boot keyboard found"))?,
                    Err(e) => Err(TransferError::Permanent(e))?,
                };
                info!("Boot keyboard found on endpoint {}", ep);

                self.endpoints[0] = Some(EP::new(
                    self.addr,
                    ep,
                    TransferType::Interrupt,
                    Direction::In,
                ));

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

enum Descriptor<'a> {
    Configuration(&'a ConfigurationDescriptor),
    Interface(&'a InterfaceDescriptor),
    Endpoint(&'a EndpointDescriptor),
    Other(&'a [u8]),
}

// TODO: Iter impl.
struct DescriptorParser<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> From<&'a [u8]> for DescriptorParser<'a> {
    fn from(buf: &'a [u8]) -> Self {
        Self { buf: buf, pos: 0 }
    }
}

impl DescriptorParser<'_> {
    fn next(&mut self) -> Option<Descriptor> {
        if self.pos == self.buf.len() {
            return None;
        }

        assert!(self.pos < (i32::max_value() as usize));
        assert!(self.pos <= self.buf.len() + 2);

        let end = self.pos + self.buf[self.pos] as usize;
        assert!(end <= self.buf.len());

        // TODO: this is basically guaranteed to have unaligned
        // access, isn't it? That's not good. RIP zero-copy?
        let res = match DescriptorType::try_from(self.buf[self.pos + 1]) {
            Ok(DescriptorType::Configuration) => {
                let desc: &ConfigurationDescriptor = unsafe {
                    let ptr = self.buf.as_ptr().offset(self.pos as isize);
                    &*(ptr as *const _)
                };
                Some(Descriptor::Configuration(desc))
            }

            Ok(DescriptorType::Interface) => {
                let desc: &InterfaceDescriptor = unsafe {
                    let ptr = self.buf.as_ptr().offset(self.pos as isize);
                    &*(ptr as *const _)
                };
                Some(Descriptor::Interface(desc))
            }

            Ok(DescriptorType::Endpoint) => {
                let desc: &EndpointDescriptor = unsafe {
                    let ptr = self.buf.as_ptr().offset(self.pos as isize);
                    &*(ptr as *const _)
                };
                Some(Descriptor::Endpoint(desc))
            }

            // Return a raw byte slice if we don't know how to parse
            // the descriptor naturally, so callers can figure it out.
            Err(_) => Some(Descriptor::Other(&self.buf[self.pos..end])),
            _ => Some(Descriptor::Other(&self.buf[self.pos..end])),
        };

        self.pos = end;
        res
    }
}

fn ep_for_bootkbd(buf: &[u8]) -> Result<Option<u8>, &'static str> {
    let mut parser = DescriptorParser::from(buf);
    let mut interface_found = false;
    while let Some(desc) = parser.next() {
        if let Descriptor::Interface(idesc) = desc {
            interface_found = idesc.b_interface_class == 0x03
                && idesc.b_interface_sub_class == 0x01
                && idesc.b_interface_protocol == 0x01;
        } else if let Descriptor::Endpoint(edesc) = desc {
            if interface_found {
                return Ok(Some(edesc.b_endpoint_address));
            }
        }
    }
    Ok(None)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn parse_logitech_g105_config() {
        // Config, Interface (0.0), HID, Endpoint, Interface (1.0), HID, Endpoint
        let raw: &[u8] = &[
            0x09, 0x02, 0x3b, 0x00, 0x02, 0x01, 0x04, 0xa0, 0x64, 0x09, 0x04, 0x00, 0x00, 0x01,
            0x03, 0x01, 0x01, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x41, 0x00, 0x07,
            0x05, 0x81, 0x03, 0x08, 0x00, 0x0a, 0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x00, 0x00,
            0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x85, 0x00, 0x07, 0x05, 0x82, 0x03,
            0x08, 0x00, 0x0a,
        ];
        let mut parser = DescriptorParser::from(raw);

        let config_desc = ConfigurationDescriptor {
            b_length: 9,
            b_descriptor_type: DescriptorType::Configuration,
            w_total_length: 59,
            b_num_interfaces: 2,
            b_configuration_value: 1,
            i_configuration: 4,
            bm_attributes: 0xa0,
            b_max_power: 100,
        };
        let desc = parser.next().expect("Parsing configuration");
        if let Descriptor::Configuration(cdesc) = desc {
            assert_eq!(*cdesc, config_desc, "Configuration descriptor mismatch.");
        } else {
            panic!("Wrong descriptor type.");
        }

        let interface_desc1 = InterfaceDescriptor {
            b_length: 9,
            b_descriptor_type: DescriptorType::Interface,
            b_interface_number: 0,
            b_alternate_setting: 0,
            b_num_endpoints: 1,
            b_interface_class: 0x03,     // HID
            b_interface_sub_class: 0x01, // Boot Interface,
            b_interface_protocol: 0x01,  // Keyboard
            i_interface: 0,
        };
        let desc = parser.next().expect("Parsing configuration");
        if let Descriptor::Interface(cdesc) = desc {
            assert_eq!(*cdesc, interface_desc1, "Interface descriptor mismatch.");
        } else {
            panic!("Wrong descriptor type.");
        }

        // Unknown descriptor just yields a byte slice.
        let hid_desc1: &[u8] = &[0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x41, 0x00];
        let desc = parser.next().expect("Parsing configuration");
        if let Descriptor::Other(cdesc) = desc {
            assert_eq!(cdesc, hid_desc1, "HID descriptor mismatch.");
        } else {
            panic!("Wrong descriptor type.");
        }

        let endpoint_desc1 = EndpointDescriptor {
            b_length: 7,
            b_descriptor_type: DescriptorType::Endpoint,
            b_endpoint_address: 0x81,
            bm_attributes: 0x03,
            w_max_packet_size: 0x08,
            b_interval: 0x0a,
        };
        let desc = parser.next().expect("Parsing configuration");
        if let Descriptor::Endpoint(cdesc) = desc {
            assert_eq!(*cdesc, endpoint_desc1, "Endpoint descriptor mismatch.");
        } else {
            panic!("Wrong descriptor type.");
        }

        let interface_desc2 = InterfaceDescriptor {
            b_length: 9,
            b_descriptor_type: DescriptorType::Interface,
            b_interface_number: 1,
            b_alternate_setting: 0,
            b_num_endpoints: 1,
            b_interface_class: 0x03,     // HID
            b_interface_sub_class: 0x00, // No subclass
            b_interface_protocol: 0x00,  // No protocol
            i_interface: 0,
        };
        let desc = parser.next().expect("Parsing configuration");
        if let Descriptor::Interface(cdesc) = desc {
            assert_eq!(*cdesc, interface_desc2, "Interface descriptor mismatch.");
        } else {
            panic!("Wrong descriptor type.");
        }

        // Unknown descriptor just yields a byte slice.
        let hid_desc2 = &[0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x85, 0x00];
        let desc = parser.next().expect("Parsing configuration");
        if let Descriptor::Other(cdesc) = desc {
            assert_eq!(cdesc, hid_desc2, "HID descriptor mismatch.");
        } else {
            panic!("Wrong descriptor type.");
        }

        let endpoint_desc2 = EndpointDescriptor {
            b_length: 7,
            b_descriptor_type: DescriptorType::Endpoint,
            b_endpoint_address: 0x82,
            bm_attributes: 0x03,
            w_max_packet_size: 0x08,
            b_interval: 0x0a,
        };
        let desc = parser.next().expect("Parsing configuration");
        if let Descriptor::Endpoint(cdesc) = desc {
            assert_eq!(*cdesc, endpoint_desc2, "Endpoint descriptor mismatch.");
        } else {
            panic!("Wrong descriptor type.");
        }

        assert!(parser.next().is_none(), "Extra descriptors.");
    }

    #[test]
    fn logitech_g105_discovers_ep0() {
        // Config, Interface (0.0), HID, Endpoint, Interface (1.0), HID, Endpoint
        let raw: &[u8] = &[
            0x09, 0x02, 0x3b, 0x00, 0x02, 0x01, 0x04, 0xa0, 0x64, 0x09, 0x04, 0x00, 0x00, 0x01,
            0x03, 0x01, 0x01, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x41, 0x00, 0x07,
            0x05, 0x81, 0x03, 0x08, 0x00, 0x0a, 0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x00, 0x00,
            0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x85, 0x00, 0x07, 0x05, 0x82, 0x03,
            0x08, 0x00, 0x0a,
        ];

        let n = ep_for_bootkbd(raw).expect("Looking for endpoint");
        assert_eq!(n, Some(0x81));
    }
}
