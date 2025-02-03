pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "service_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__service_msgs__msg__ServiceEventInfo() -> *const std::ffi::c_void;
}

#[link(name = "service_msgs__rosidl_generator_c")]
extern "C" {
    fn service_msgs__msg__ServiceEventInfo__init(msg: *mut ServiceEventInfo) -> bool;
    fn service_msgs__msg__ServiceEventInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ServiceEventInfo>, size: usize) -> bool;
    fn service_msgs__msg__ServiceEventInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ServiceEventInfo>);
    fn service_msgs__msg__ServiceEventInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ServiceEventInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<ServiceEventInfo>) -> bool;
}

// Corresponds to service_msgs__msg__ServiceEventInfo
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServiceEventInfo {
    pub event_type: u8,
    pub stamp: builtin_interfaces::msg::rmw::Time,
    pub client_gid: [u8; 16],
    pub sequence_number: i64,
}

impl ServiceEventInfo {
    pub const REQUEST_SENT: u8 = 0;
    pub const REQUEST_RECEIVED: u8 = 1;
    pub const RESPONSE_SENT: u8 = 2;
    pub const RESPONSE_RECEIVED: u8 = 3;
}


impl Default for ServiceEventInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !service_msgs__msg__ServiceEventInfo__init(&mut msg as *mut _) {
        panic!("Call to service_msgs__msg__ServiceEventInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ServiceEventInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { service_msgs__msg__ServiceEventInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { service_msgs__msg__ServiceEventInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { service_msgs__msg__ServiceEventInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ServiceEventInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ServiceEventInfo where Self: Sized {
  const TYPE_NAME: &'static str = "service_msgs/msg/ServiceEventInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__service_msgs__msg__ServiceEventInfo() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServiceEventInfo {
    pub event_type: u8,
    pub stamp: builtin_interfaces::msg::Time,
    pub client_gid: [u8; 16],
    pub sequence_number: i64,
}

impl ServiceEventInfo {
    pub const REQUEST_SENT: u8 = 0;
    pub const REQUEST_RECEIVED: u8 = 1;
    pub const RESPONSE_SENT: u8 = 2;
    pub const RESPONSE_RECEIVED: u8 = 3;
}


impl Default for ServiceEventInfo {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::ServiceEventInfo::default())
  }
}

impl rosidl_runtime_rs::Message for ServiceEventInfo {
  type RmwMsg = crate::msg::rmw::ServiceEventInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        event_type: msg.event_type,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        client_gid: msg.client_gid,
        sequence_number: msg.sequence_number,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      event_type: msg.event_type,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        client_gid: msg.client_gid,
      sequence_number: msg.sequence_number,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      event_type: msg.event_type,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      client_gid: msg.client_gid,
      sequence_number: msg.sequence_number,
    }
  }
}


