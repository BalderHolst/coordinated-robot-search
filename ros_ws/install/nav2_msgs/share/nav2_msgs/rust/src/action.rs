
pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_Goal__init(msg: *mut AssistedTeleop_Goal) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Goal>);
    fn nav2_msgs__action__AssistedTeleop_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Goal {
    pub time_allowance: builtin_interfaces::msg::rmw::Duration,
}



impl Default for AssistedTeleop_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_Result__init(msg: *mut AssistedTeleop_Result) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Result>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Result>);
    fn nav2_msgs__action__AssistedTeleop_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Result {
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl AssistedTeleop_Result {
    /// Error codes
    /// Note: The expected priority order of the error should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 730;
    pub const TIMEOUT: u16 = 731;
    pub const TF_ERROR: u16 = 732;
}


impl Default for AssistedTeleop_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_Feedback__init(msg: *mut AssistedTeleop_Feedback) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Feedback>);
    fn nav2_msgs__action__AssistedTeleop_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Feedback {
    pub current_teleop_duration: builtin_interfaces::msg::rmw::Duration,
}



impl Default for AssistedTeleop_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_FeedbackMessage__init(msg: *mut AssistedTeleop_FeedbackMessage) -> bool;
    fn nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_FeedbackMessage>);
    fn nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::AssistedTeleop_Feedback,
}



impl Default for AssistedTeleop_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_Goal__init(msg: *mut BackUp_Goal) -> bool;
    fn nav2_msgs__action__BackUp_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Goal>);
    fn nav2_msgs__action__BackUp_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Goal {
    pub target: geometry_msgs::msg::rmw::Point,
    pub speed: f32,
    pub time_allowance: builtin_interfaces::msg::rmw::Duration,
    pub disable_collision_checks: bool,
}



impl Default for BackUp_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_Result__init(msg: *mut BackUp_Result) -> bool;
    fn nav2_msgs__action__BackUp_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Result>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Result>);
    fn nav2_msgs__action__BackUp_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Result {
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl BackUp_Result {
    /// Error codes
    /// Note: The expected priority order of the error should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 710;
    pub const TIMEOUT: u16 = 711;
    pub const TF_ERROR: u16 = 712;
    pub const INVALID_INPUT: u16 = 713;
    pub const COLLISION_AHEAD: u16 = 714;
}


impl Default for BackUp_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_Feedback__init(msg: *mut BackUp_Feedback) -> bool;
    fn nav2_msgs__action__BackUp_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Feedback>);
    fn nav2_msgs__action__BackUp_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Feedback {
    pub distance_traveled: f32,
}



impl Default for BackUp_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_FeedbackMessage__init(msg: *mut BackUp_FeedbackMessage) -> bool;
    fn nav2_msgs__action__BackUp_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_FeedbackMessage>);
    fn nav2_msgs__action__BackUp_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::BackUp_Feedback,
}



impl Default for BackUp_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_Goal__init(msg: *mut ComputePathToPose_Goal) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Goal>);
    fn nav2_msgs__action__ComputePathToPose_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Goal {
    pub goal: geometry_msgs::msg::rmw::PoseStamped,
    pub start: geometry_msgs::msg::rmw::PoseStamped,
    pub planner_id: rosidl_runtime_rs::String,
    pub use_start: bool,
}



impl Default for ComputePathToPose_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_Result__init(msg: *mut ComputePathToPose_Result) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Result>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Result>);
    fn nav2_msgs__action__ComputePathToPose_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Result {
    pub path: nav_msgs::msg::rmw::Path,
    pub planning_time: builtin_interfaces::msg::rmw::Duration,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl ComputePathToPose_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 200;
    pub const INVALID_PLANNER: u16 = 201;
    pub const TF_ERROR: u16 = 202;
    pub const START_OUTSIDE_MAP: u16 = 203;
    pub const GOAL_OUTSIDE_MAP: u16 = 204;
    pub const START_OCCUPIED: u16 = 205;
    pub const GOAL_OCCUPIED: u16 = 206;
    pub const TIMEOUT: u16 = 207;
    pub const NO_VALID_PATH: u16 = 208;
}


impl Default for ComputePathToPose_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_Feedback__init(msg: *mut ComputePathToPose_Feedback) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Feedback>);
    fn nav2_msgs__action__ComputePathToPose_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for ComputePathToPose_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_FeedbackMessage__init(msg: *mut ComputePathToPose_FeedbackMessage) -> bool;
    fn nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_FeedbackMessage>);
    fn nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::ComputePathToPose_Feedback,
}



impl Default for ComputePathToPose_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_Goal__init(msg: *mut ComputePathThroughPoses_Goal) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Goal>);
    fn nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Goal {
    pub goals: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,
    pub start: geometry_msgs::msg::rmw::PoseStamped,
    pub planner_id: rosidl_runtime_rs::String,
    pub use_start: bool,
}



impl Default for ComputePathThroughPoses_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_Result__init(msg: *mut ComputePathThroughPoses_Result) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Result>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Result>);
    fn nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Result {
    pub path: nav_msgs::msg::rmw::Path,
    pub planning_time: builtin_interfaces::msg::rmw::Duration,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl ComputePathThroughPoses_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 300;
    pub const INVALID_PLANNER: u16 = 301;
    pub const TF_ERROR: u16 = 302;
    pub const START_OUTSIDE_MAP: u16 = 303;
    pub const GOAL_OUTSIDE_MAP: u16 = 304;
    pub const START_OCCUPIED: u16 = 305;
    pub const GOAL_OCCUPIED: u16 = 306;
    pub const TIMEOUT: u16 = 307;
    pub const NO_VALID_PATH: u16 = 308;
    pub const NO_VIAPOINTS_GIVEN: u16 = 309;
}


impl Default for ComputePathThroughPoses_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_Feedback__init(msg: *mut ComputePathThroughPoses_Feedback) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Feedback>);
    fn nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for ComputePathThroughPoses_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__init(msg: *mut ComputePathThroughPoses_FeedbackMessage) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_FeedbackMessage>);
    fn nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::ComputePathThroughPoses_Feedback,
}



impl Default for ComputePathThroughPoses_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_Goal__init(msg: *mut DriveOnHeading_Goal) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Goal>);
    fn nav2_msgs__action__DriveOnHeading_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Goal {
    pub target: geometry_msgs::msg::rmw::Point,
    pub speed: f32,
    pub time_allowance: builtin_interfaces::msg::rmw::Duration,
    pub disable_collision_checks: bool,
}



impl Default for DriveOnHeading_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_Result__init(msg: *mut DriveOnHeading_Result) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Result>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Result>);
    fn nav2_msgs__action__DriveOnHeading_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Result {
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl DriveOnHeading_Result {
    /// Error codes
    /// Note: The expected priority order of the error should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 720;
    pub const TIMEOUT: u16 = 721;
    pub const TF_ERROR: u16 = 722;
    pub const COLLISION_AHEAD: u16 = 723;
    pub const INVALID_INPUT: u16 = 724;
}


impl Default for DriveOnHeading_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_Feedback__init(msg: *mut DriveOnHeading_Feedback) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Feedback>);
    fn nav2_msgs__action__DriveOnHeading_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Feedback {
    pub distance_traveled: f32,
}



impl Default for DriveOnHeading_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_FeedbackMessage__init(msg: *mut DriveOnHeading_FeedbackMessage) -> bool;
    fn nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_FeedbackMessage>);
    fn nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::DriveOnHeading_Feedback,
}



impl Default for DriveOnHeading_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_Goal__init(msg: *mut SmoothPath_Goal) -> bool;
    fn nav2_msgs__action__SmoothPath_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Goal>);
    fn nav2_msgs__action__SmoothPath_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Goal {
    pub path: nav_msgs::msg::rmw::Path,
    pub smoother_id: rosidl_runtime_rs::String,
    pub max_smoothing_duration: builtin_interfaces::msg::rmw::Duration,
    pub check_for_collisions: bool,
}



impl Default for SmoothPath_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_Result__init(msg: *mut SmoothPath_Result) -> bool;
    fn nav2_msgs__action__SmoothPath_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Result>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Result>);
    fn nav2_msgs__action__SmoothPath_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Result {
    pub path: nav_msgs::msg::rmw::Path,
    pub smoothing_duration: builtin_interfaces::msg::rmw::Duration,
    pub was_completed: bool,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl SmoothPath_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 500;
    pub const INVALID_SMOOTHER: u16 = 501;
    pub const TIMEOUT: u16 = 502;
    pub const SMOOTHED_PATH_IN_COLLISION: u16 = 503;
    pub const FAILED_TO_SMOOTH_PATH: u16 = 504;
    pub const INVALID_PATH: u16 = 505;
}


impl Default for SmoothPath_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_Feedback__init(msg: *mut SmoothPath_Feedback) -> bool;
    fn nav2_msgs__action__SmoothPath_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Feedback>);
    fn nav2_msgs__action__SmoothPath_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for SmoothPath_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_FeedbackMessage__init(msg: *mut SmoothPath_FeedbackMessage) -> bool;
    fn nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_FeedbackMessage>);
    fn nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::SmoothPath_Feedback,
}



impl Default for SmoothPath_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_Goal__init(msg: *mut FollowPath_Goal) -> bool;
    fn nav2_msgs__action__FollowPath_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Goal>);
    fn nav2_msgs__action__FollowPath_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Goal {
    pub path: nav_msgs::msg::rmw::Path,
    pub controller_id: rosidl_runtime_rs::String,
    pub goal_checker_id: rosidl_runtime_rs::String,
    pub progress_checker_id: rosidl_runtime_rs::String,
}



impl Default for FollowPath_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_Result__init(msg: *mut FollowPath_Result) -> bool;
    fn nav2_msgs__action__FollowPath_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Result>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Result>);
    fn nav2_msgs__action__FollowPath_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Result {
    pub result: std_msgs::msg::rmw::Empty,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl FollowPath_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 100;
    pub const INVALID_CONTROLLER: u16 = 101;
    pub const TF_ERROR: u16 = 102;
    pub const INVALID_PATH: u16 = 103;
    pub const PATIENCE_EXCEEDED: u16 = 104;
    pub const FAILED_TO_MAKE_PROGRESS: u16 = 105;
    pub const NO_VALID_CONTROL: u16 = 106;
    pub const CONTROLLER_TIMED_OUT: u16 = 107;
}


impl Default for FollowPath_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_Feedback__init(msg: *mut FollowPath_Feedback) -> bool;
    fn nav2_msgs__action__FollowPath_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Feedback>);
    fn nav2_msgs__action__FollowPath_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Feedback {
    pub distance_to_goal: f32,
    pub speed: f32,
}



impl Default for FollowPath_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_FeedbackMessage__init(msg: *mut FollowPath_FeedbackMessage) -> bool;
    fn nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_FeedbackMessage>);
    fn nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::FollowPath_Feedback,
}



impl Default for FollowPath_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_Goal__init(msg: *mut NavigateToPose_Goal) -> bool;
    fn nav2_msgs__action__NavigateToPose_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Goal>);
    fn nav2_msgs__action__NavigateToPose_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Goal {
    pub pose: geometry_msgs::msg::rmw::PoseStamped,
    pub behavior_tree: rosidl_runtime_rs::String,
}



impl Default for NavigateToPose_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_Result__init(msg: *mut NavigateToPose_Result) -> bool;
    fn nav2_msgs__action__NavigateToPose_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Result>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Result>);
    fn nav2_msgs__action__NavigateToPose_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Result {
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl NavigateToPose_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
}


impl Default for NavigateToPose_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_Feedback__init(msg: *mut NavigateToPose_Feedback) -> bool;
    fn nav2_msgs__action__NavigateToPose_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Feedback>);
    fn nav2_msgs__action__NavigateToPose_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Feedback {
    pub current_pose: geometry_msgs::msg::rmw::PoseStamped,
    pub navigation_time: builtin_interfaces::msg::rmw::Duration,
    pub estimated_time_remaining: builtin_interfaces::msg::rmw::Duration,
    pub number_of_recoveries: i16,
    pub distance_remaining: f32,
}



impl Default for NavigateToPose_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_FeedbackMessage__init(msg: *mut NavigateToPose_FeedbackMessage) -> bool;
    fn nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_FeedbackMessage>);
    fn nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::NavigateToPose_Feedback,
}



impl Default for NavigateToPose_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_Goal__init(msg: *mut NavigateThroughPoses_Goal) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Goal>);
    fn nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Goal {
    pub poses: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,
    pub behavior_tree: rosidl_runtime_rs::String,
}



impl Default for NavigateThroughPoses_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_Result__init(msg: *mut NavigateThroughPoses_Result) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Result>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Result>);
    fn nav2_msgs__action__NavigateThroughPoses_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Result {
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl NavigateThroughPoses_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
}


impl Default for NavigateThroughPoses_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_Feedback__init(msg: *mut NavigateThroughPoses_Feedback) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Feedback>);
    fn nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Feedback {
    pub current_pose: geometry_msgs::msg::rmw::PoseStamped,
    pub navigation_time: builtin_interfaces::msg::rmw::Duration,
    pub estimated_time_remaining: builtin_interfaces::msg::rmw::Duration,
    pub number_of_recoveries: i16,
    pub distance_remaining: f32,
    pub number_of_poses_remaining: i16,
}



impl Default for NavigateThroughPoses_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__init(msg: *mut NavigateThroughPoses_FeedbackMessage) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_FeedbackMessage>);
    fn nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::NavigateThroughPoses_Feedback,
}



impl Default for NavigateThroughPoses_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_Goal__init(msg: *mut Wait_Goal) -> bool;
    fn nav2_msgs__action__Wait_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_Goal>);
    fn nav2_msgs__action__Wait_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Goal {
    pub time: builtin_interfaces::msg::rmw::Duration,
}



impl Default for Wait_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_Result__init(msg: *mut Wait_Result) -> bool;
    fn nav2_msgs__action__Wait_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_Result>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_Result>);
    fn nav2_msgs__action__Wait_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Result {
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}



impl Default for Wait_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_Feedback__init(msg: *mut Wait_Feedback) -> bool;
    fn nav2_msgs__action__Wait_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_Feedback>);
    fn nav2_msgs__action__Wait_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Feedback {
    pub time_left: builtin_interfaces::msg::rmw::Duration,
}



impl Default for Wait_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_FeedbackMessage__init(msg: *mut Wait_FeedbackMessage) -> bool;
    fn nav2_msgs__action__Wait_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_FeedbackMessage>);
    fn nav2_msgs__action__Wait_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::Wait_Feedback,
}



impl Default for Wait_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_Goal__init(msg: *mut Spin_Goal) -> bool;
    fn nav2_msgs__action__Spin_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_Goal>);
    fn nav2_msgs__action__Spin_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Goal {
    pub target_yaw: f32,
    pub time_allowance: builtin_interfaces::msg::rmw::Duration,
    pub disable_collision_checks: bool,
}



impl Default for Spin_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_Result__init(msg: *mut Spin_Result) -> bool;
    fn nav2_msgs__action__Spin_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_Result>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_Result>);
    fn nav2_msgs__action__Spin_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Result {
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl Spin_Result {
    /// Error codes
    /// Note: The expected priority order of the error should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 700;
    pub const TIMEOUT: u16 = 701;
    pub const TF_ERROR: u16 = 702;
    pub const COLLISION_AHEAD: u16 = 703;
}


impl Default for Spin_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_Feedback__init(msg: *mut Spin_Feedback) -> bool;
    fn nav2_msgs__action__Spin_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_Feedback>);
    fn nav2_msgs__action__Spin_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Feedback {
    pub angular_distance_traveled: f32,
}



impl Default for Spin_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_FeedbackMessage__init(msg: *mut Spin_FeedbackMessage) -> bool;
    fn nav2_msgs__action__Spin_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_FeedbackMessage>);
    fn nav2_msgs__action__Spin_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::Spin_Feedback,
}



impl Default for Spin_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_Goal__init(msg: *mut DummyBehavior_Goal) -> bool;
    fn nav2_msgs__action__DummyBehavior_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Goal>);
    fn nav2_msgs__action__DummyBehavior_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Goal {
    pub command: std_msgs::msg::rmw::String,
}



impl Default for DummyBehavior_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_Result__init(msg: *mut DummyBehavior_Result) -> bool;
    fn nav2_msgs__action__DummyBehavior_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Result>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Result>);
    fn nav2_msgs__action__DummyBehavior_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Result {
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}



impl Default for DummyBehavior_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_Feedback__init(msg: *mut DummyBehavior_Feedback) -> bool;
    fn nav2_msgs__action__DummyBehavior_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Feedback>);
    fn nav2_msgs__action__DummyBehavior_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for DummyBehavior_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_FeedbackMessage__init(msg: *mut DummyBehavior_FeedbackMessage) -> bool;
    fn nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_FeedbackMessage>);
    fn nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::DummyBehavior_Feedback,
}



impl Default for DummyBehavior_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_Goal__init(msg: *mut FollowWaypoints_Goal) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Goal>);
    fn nav2_msgs__action__FollowWaypoints_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Goal {
    pub number_of_loops: u32,
    pub goal_index: u32,
    pub poses: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,
}



impl Default for FollowWaypoints_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_Result__init(msg: *mut FollowWaypoints_Result) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Result>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Result>);
    fn nav2_msgs__action__FollowWaypoints_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Result {
    pub missed_waypoints: rosidl_runtime_rs::Sequence<crate::msg::rmw::MissedWaypoint>,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl FollowWaypoints_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 600;
    pub const TASK_EXECUTOR_FAILED: u16 = 601;
}


impl Default for FollowWaypoints_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_Feedback__init(msg: *mut FollowWaypoints_Feedback) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Feedback>);
    fn nav2_msgs__action__FollowWaypoints_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Feedback {
    pub current_waypoint: u32,
}



impl Default for FollowWaypoints_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_FeedbackMessage__init(msg: *mut FollowWaypoints_FeedbackMessage) -> bool;
    fn nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_FeedbackMessage>);
    fn nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::FollowWaypoints_Feedback,
}



impl Default for FollowWaypoints_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowGPSWaypoints_Goal__init(msg: *mut FollowGPSWaypoints_Goal) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Goal>);
    fn nav2_msgs__action__FollowGPSWaypoints_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_Goal {
    pub number_of_loops: u32,
    pub goal_index: u32,
    pub gps_poses: rosidl_runtime_rs::Sequence<geographic_msgs::msg::rmw::GeoPose>,
}



impl Default for FollowGPSWaypoints_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowGPSWaypoints_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowGPSWaypoints_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowGPSWaypoints_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowGPSWaypoints_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowGPSWaypoints_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowGPSWaypoints_Result__init(msg: *mut FollowGPSWaypoints_Result) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Result>, size: usize) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Result>);
    fn nav2_msgs__action__FollowGPSWaypoints_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_Result {
    pub missed_waypoints: rosidl_runtime_rs::Sequence<crate::msg::rmw::MissedWaypoint>,
    pub error_code: i16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl FollowGPSWaypoints_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 600;
    pub const TASK_EXECUTOR_FAILED: u16 = 601;
}


impl Default for FollowGPSWaypoints_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowGPSWaypoints_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowGPSWaypoints_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowGPSWaypoints_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowGPSWaypoints_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowGPSWaypoints_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowGPSWaypoints_Feedback__init(msg: *mut FollowGPSWaypoints_Feedback) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Feedback>);
    fn nav2_msgs__action__FollowGPSWaypoints_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_Feedback {
    pub current_waypoint: u32,
}



impl Default for FollowGPSWaypoints_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowGPSWaypoints_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowGPSWaypoints_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowGPSWaypoints_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowGPSWaypoints_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowGPSWaypoints_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__init(msg: *mut FollowGPSWaypoints_FeedbackMessage) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_FeedbackMessage>);
    fn nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowGPSWaypoints_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::FollowGPSWaypoints_Feedback,
}



impl Default for FollowGPSWaypoints_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowGPSWaypoints_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowGPSWaypoints_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowGPSWaypoints_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DockRobot_Goal__init(msg: *mut DockRobot_Goal) -> bool;
    fn nav2_msgs__action__DockRobot_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__DockRobot_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Goal>);
    fn nav2_msgs__action__DockRobot_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockRobot_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__DockRobot_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_Goal {
    pub use_dock_id: bool,
    pub dock_id: rosidl_runtime_rs::String,
    pub dock_pose: geometry_msgs::msg::rmw::PoseStamped,
    pub dock_type: rosidl_runtime_rs::String,
    pub max_staging_time: f32,
    pub navigate_to_staging_pose: bool,
}



impl Default for DockRobot_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DockRobot_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DockRobot_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockRobot_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockRobot_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockRobot_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DockRobot_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DockRobot_Result__init(msg: *mut DockRobot_Result) -> bool;
    fn nav2_msgs__action__DockRobot_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Result>, size: usize) -> bool;
    fn nav2_msgs__action__DockRobot_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Result>);
    fn nav2_msgs__action__DockRobot_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockRobot_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__DockRobot_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_Result {
    pub success: bool,
    pub error_code: u16,
    pub num_retries: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl DockRobot_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const DOCK_NOT_IN_DB: u16 = 901;
    pub const DOCK_NOT_VALID: u16 = 902;
    pub const FAILED_TO_STAGE: u16 = 903;
    pub const FAILED_TO_DETECT_DOCK: u16 = 904;
    pub const FAILED_TO_CONTROL: u16 = 905;
    pub const FAILED_TO_CHARGE: u16 = 906;
    pub const UNKNOWN: u16 = 999;
}


impl Default for DockRobot_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DockRobot_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DockRobot_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockRobot_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockRobot_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockRobot_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DockRobot_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DockRobot_Feedback__init(msg: *mut DockRobot_Feedback) -> bool;
    fn nav2_msgs__action__DockRobot_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__DockRobot_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Feedback>);
    fn nav2_msgs__action__DockRobot_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockRobot_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<DockRobot_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__DockRobot_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_Feedback {
    pub state: u16,
    pub docking_time: builtin_interfaces::msg::rmw::Duration,
    pub num_retries: u16,
}

impl DockRobot_Feedback {
    pub const NONE: u16 = 0;
    pub const NAV_TO_STAGING_POSE: u16 = 1;
    pub const INITIAL_PERCEPTION: u16 = 2;
    pub const CONTROLLING: u16 = 3;
    pub const WAIT_FOR_CHARGE: u16 = 4;
    pub const RETRY: u16 = 5;
}


impl Default for DockRobot_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DockRobot_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DockRobot_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockRobot_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockRobot_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockRobot_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DockRobot_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DockRobot_FeedbackMessage__init(msg: *mut DockRobot_FeedbackMessage) -> bool;
    fn nav2_msgs__action__DockRobot_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__DockRobot_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_FeedbackMessage>);
    fn nav2_msgs__action__DockRobot_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockRobot_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<DockRobot_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__DockRobot_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::DockRobot_Feedback,
}



impl Default for DockRobot_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DockRobot_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DockRobot_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockRobot_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockRobot_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockRobot_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DockRobot_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__UndockRobot_Goal__init(msg: *mut UndockRobot_Goal) -> bool;
    fn nav2_msgs__action__UndockRobot_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__UndockRobot_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Goal>);
    fn nav2_msgs__action__UndockRobot_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UndockRobot_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__UndockRobot_Goal
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_Goal {
    pub dock_type: rosidl_runtime_rs::String,
    pub max_undocking_time: f32,
}



impl Default for UndockRobot_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__UndockRobot_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__UndockRobot_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UndockRobot_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UndockRobot_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/UndockRobot_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__UndockRobot_Result__init(msg: *mut UndockRobot_Result) -> bool;
    fn nav2_msgs__action__UndockRobot_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Result>, size: usize) -> bool;
    fn nav2_msgs__action__UndockRobot_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Result>);
    fn nav2_msgs__action__UndockRobot_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UndockRobot_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__UndockRobot_Result
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_Result {
    pub success: bool,
    pub error_code: u16,
    pub error_msg: rosidl_runtime_rs::String,
}

impl UndockRobot_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const DOCK_NOT_VALID: u16 = 902;
    pub const FAILED_TO_CONTROL: u16 = 905;
    pub const UNKNOWN: u16 = 999;
}


impl Default for UndockRobot_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__UndockRobot_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__UndockRobot_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UndockRobot_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UndockRobot_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/UndockRobot_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__UndockRobot_Feedback__init(msg: *mut UndockRobot_Feedback) -> bool;
    fn nav2_msgs__action__UndockRobot_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__UndockRobot_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Feedback>);
    fn nav2_msgs__action__UndockRobot_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UndockRobot_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__UndockRobot_Feedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for UndockRobot_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__UndockRobot_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__UndockRobot_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UndockRobot_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UndockRobot_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/UndockRobot_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__UndockRobot_FeedbackMessage__init(msg: *mut UndockRobot_FeedbackMessage) -> bool;
    fn nav2_msgs__action__UndockRobot_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__UndockRobot_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_FeedbackMessage>);
    fn nav2_msgs__action__UndockRobot_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UndockRobot_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__UndockRobot_FeedbackMessage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: crate::action::rmw::UndockRobot_Feedback,
}



impl Default for UndockRobot_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__UndockRobot_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__UndockRobot_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UndockRobot_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UndockRobot_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/UndockRobot_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_FeedbackMessage() }
  }
}



#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Request__init(msg: *mut AssistedTeleop_SendGoal_Request) -> bool;
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Request>);
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::AssistedTeleop_Goal,
}



impl Default for AssistedTeleop_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Response__init(msg: *mut AssistedTeleop_SendGoal_Response) -> bool;
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Response>);
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for AssistedTeleop_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_GetResult_Request__init(msg: *mut AssistedTeleop_GetResult_Request) -> bool;
    fn nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Request>);
    fn nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for AssistedTeleop_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_GetResult_Response__init(msg: *mut AssistedTeleop_GetResult_Response) -> bool;
    fn nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Response>);
    fn nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::AssistedTeleop_Result,
}



impl Default for AssistedTeleop_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_SendGoal_Request__init(msg: *mut BackUp_SendGoal_Request) -> bool;
    fn nav2_msgs__action__BackUp_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Request>);
    fn nav2_msgs__action__BackUp_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::BackUp_Goal,
}



impl Default for BackUp_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_SendGoal_Response__init(msg: *mut BackUp_SendGoal_Response) -> bool;
    fn nav2_msgs__action__BackUp_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Response>);
    fn nav2_msgs__action__BackUp_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for BackUp_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_GetResult_Request__init(msg: *mut BackUp_GetResult_Request) -> bool;
    fn nav2_msgs__action__BackUp_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Request>);
    fn nav2_msgs__action__BackUp_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for BackUp_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_GetResult_Response__init(msg: *mut BackUp_GetResult_Response) -> bool;
    fn nav2_msgs__action__BackUp_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Response>);
    fn nav2_msgs__action__BackUp_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::BackUp_Result,
}



impl Default for BackUp_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Request__init(msg: *mut ComputePathToPose_SendGoal_Request) -> bool;
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Request>);
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::ComputePathToPose_Goal,
}



impl Default for ComputePathToPose_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Response__init(msg: *mut ComputePathToPose_SendGoal_Response) -> bool;
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Response>);
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for ComputePathToPose_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_GetResult_Request__init(msg: *mut ComputePathToPose_GetResult_Request) -> bool;
    fn nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Request>);
    fn nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for ComputePathToPose_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_GetResult_Response__init(msg: *mut ComputePathToPose_GetResult_Response) -> bool;
    fn nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Response>);
    fn nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::ComputePathToPose_Result,
}



impl Default for ComputePathToPose_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__init(msg: *mut ComputePathThroughPoses_SendGoal_Request) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Request>);
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::ComputePathThroughPoses_Goal,
}



impl Default for ComputePathThroughPoses_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__init(msg: *mut ComputePathThroughPoses_SendGoal_Response) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Response>);
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for ComputePathThroughPoses_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__init(msg: *mut ComputePathThroughPoses_GetResult_Request) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Request>);
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for ComputePathThroughPoses_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__init(msg: *mut ComputePathThroughPoses_GetResult_Response) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Response>);
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::ComputePathThroughPoses_Result,
}



impl Default for ComputePathThroughPoses_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Request__init(msg: *mut DriveOnHeading_SendGoal_Request) -> bool;
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Request>);
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::DriveOnHeading_Goal,
}



impl Default for DriveOnHeading_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Response__init(msg: *mut DriveOnHeading_SendGoal_Response) -> bool;
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Response>);
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for DriveOnHeading_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_GetResult_Request__init(msg: *mut DriveOnHeading_GetResult_Request) -> bool;
    fn nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Request>);
    fn nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for DriveOnHeading_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_GetResult_Response__init(msg: *mut DriveOnHeading_GetResult_Response) -> bool;
    fn nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Response>);
    fn nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::DriveOnHeading_Result,
}



impl Default for DriveOnHeading_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_SendGoal_Request__init(msg: *mut SmoothPath_SendGoal_Request) -> bool;
    fn nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Request>);
    fn nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::SmoothPath_Goal,
}



impl Default for SmoothPath_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_SendGoal_Response__init(msg: *mut SmoothPath_SendGoal_Response) -> bool;
    fn nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Response>);
    fn nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for SmoothPath_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_GetResult_Request__init(msg: *mut SmoothPath_GetResult_Request) -> bool;
    fn nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Request>);
    fn nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for SmoothPath_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_GetResult_Response__init(msg: *mut SmoothPath_GetResult_Response) -> bool;
    fn nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Response>);
    fn nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::SmoothPath_Result,
}



impl Default for SmoothPath_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_SendGoal_Request__init(msg: *mut FollowPath_SendGoal_Request) -> bool;
    fn nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Request>);
    fn nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::FollowPath_Goal,
}



impl Default for FollowPath_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_SendGoal_Response__init(msg: *mut FollowPath_SendGoal_Response) -> bool;
    fn nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Response>);
    fn nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for FollowPath_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_GetResult_Request__init(msg: *mut FollowPath_GetResult_Request) -> bool;
    fn nav2_msgs__action__FollowPath_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Request>);
    fn nav2_msgs__action__FollowPath_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for FollowPath_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_GetResult_Response__init(msg: *mut FollowPath_GetResult_Response) -> bool;
    fn nav2_msgs__action__FollowPath_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Response>);
    fn nav2_msgs__action__FollowPath_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::FollowPath_Result,
}



impl Default for FollowPath_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_SendGoal_Request__init(msg: *mut NavigateToPose_SendGoal_Request) -> bool;
    fn nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Request>);
    fn nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::NavigateToPose_Goal,
}



impl Default for NavigateToPose_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_SendGoal_Response__init(msg: *mut NavigateToPose_SendGoal_Response) -> bool;
    fn nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Response>);
    fn nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for NavigateToPose_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_GetResult_Request__init(msg: *mut NavigateToPose_GetResult_Request) -> bool;
    fn nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Request>);
    fn nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for NavigateToPose_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_GetResult_Response__init(msg: *mut NavigateToPose_GetResult_Response) -> bool;
    fn nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Response>);
    fn nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::NavigateToPose_Result,
}



impl Default for NavigateToPose_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__init(msg: *mut NavigateThroughPoses_SendGoal_Request) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Request>);
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::NavigateThroughPoses_Goal,
}



impl Default for NavigateThroughPoses_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__init(msg: *mut NavigateThroughPoses_SendGoal_Response) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Response>);
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for NavigateThroughPoses_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Request__init(msg: *mut NavigateThroughPoses_GetResult_Request) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Request>);
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for NavigateThroughPoses_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Response__init(msg: *mut NavigateThroughPoses_GetResult_Response) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Response>);
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::NavigateThroughPoses_Result,
}



impl Default for NavigateThroughPoses_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_SendGoal_Request__init(msg: *mut Wait_SendGoal_Request) -> bool;
    fn nav2_msgs__action__Wait_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Request>);
    fn nav2_msgs__action__Wait_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::Wait_Goal,
}



impl Default for Wait_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_SendGoal_Response__init(msg: *mut Wait_SendGoal_Response) -> bool;
    fn nav2_msgs__action__Wait_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Response>);
    fn nav2_msgs__action__Wait_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for Wait_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_GetResult_Request__init(msg: *mut Wait_GetResult_Request) -> bool;
    fn nav2_msgs__action__Wait_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Request>);
    fn nav2_msgs__action__Wait_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for Wait_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_GetResult_Response__init(msg: *mut Wait_GetResult_Response) -> bool;
    fn nav2_msgs__action__Wait_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Response>);
    fn nav2_msgs__action__Wait_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::Wait_Result,
}



impl Default for Wait_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_SendGoal_Request__init(msg: *mut Spin_SendGoal_Request) -> bool;
    fn nav2_msgs__action__Spin_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Request>);
    fn nav2_msgs__action__Spin_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::Spin_Goal,
}



impl Default for Spin_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_SendGoal_Response__init(msg: *mut Spin_SendGoal_Response) -> bool;
    fn nav2_msgs__action__Spin_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Response>);
    fn nav2_msgs__action__Spin_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for Spin_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_GetResult_Request__init(msg: *mut Spin_GetResult_Request) -> bool;
    fn nav2_msgs__action__Spin_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Request>);
    fn nav2_msgs__action__Spin_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for Spin_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_GetResult_Response__init(msg: *mut Spin_GetResult_Response) -> bool;
    fn nav2_msgs__action__Spin_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Response>);
    fn nav2_msgs__action__Spin_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::Spin_Result,
}



impl Default for Spin_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_SendGoal_Request__init(msg: *mut DummyBehavior_SendGoal_Request) -> bool;
    fn nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Request>);
    fn nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::DummyBehavior_Goal,
}



impl Default for DummyBehavior_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_SendGoal_Response__init(msg: *mut DummyBehavior_SendGoal_Response) -> bool;
    fn nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Response>);
    fn nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for DummyBehavior_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_GetResult_Request__init(msg: *mut DummyBehavior_GetResult_Request) -> bool;
    fn nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Request>);
    fn nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for DummyBehavior_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_GetResult_Response__init(msg: *mut DummyBehavior_GetResult_Response) -> bool;
    fn nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Response>);
    fn nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::DummyBehavior_Result,
}



impl Default for DummyBehavior_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Request__init(msg: *mut FollowWaypoints_SendGoal_Request) -> bool;
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Request>);
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::FollowWaypoints_Goal,
}



impl Default for FollowWaypoints_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Response__init(msg: *mut FollowWaypoints_SendGoal_Response) -> bool;
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Response>);
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for FollowWaypoints_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_GetResult_Request__init(msg: *mut FollowWaypoints_GetResult_Request) -> bool;
    fn nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Request>);
    fn nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for FollowWaypoints_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_GetResult_Response__init(msg: *mut FollowWaypoints_GetResult_Response) -> bool;
    fn nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Response>);
    fn nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::FollowWaypoints_Result,
}



impl Default for FollowWaypoints_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__init(msg: *mut FollowGPSWaypoints_SendGoal_Request) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_SendGoal_Request>);
    fn nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowGPSWaypoints_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::FollowGPSWaypoints_Goal,
}



impl Default for FollowGPSWaypoints_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowGPSWaypoints_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowGPSWaypoints_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowGPSWaypoints_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__init(msg: *mut FollowGPSWaypoints_SendGoal_Response) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_SendGoal_Response>);
    fn nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowGPSWaypoints_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for FollowGPSWaypoints_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowGPSWaypoints_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowGPSWaypoints_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowGPSWaypoints_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__init(msg: *mut FollowGPSWaypoints_GetResult_Request) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_GetResult_Request>);
    fn nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowGPSWaypoints_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for FollowGPSWaypoints_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowGPSWaypoints_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowGPSWaypoints_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowGPSWaypoints_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__init(msg: *mut FollowGPSWaypoints_GetResult_Response) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_GetResult_Response>);
    fn nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowGPSWaypoints_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowGPSWaypoints_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::FollowGPSWaypoints_Result,
}



impl Default for FollowGPSWaypoints_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowGPSWaypoints_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowGPSWaypoints_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowGPSWaypoints_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowGPSWaypoints_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DockRobot_SendGoal_Request__init(msg: *mut DockRobot_SendGoal_Request) -> bool;
    fn nav2_msgs__action__DockRobot_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DockRobot_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_SendGoal_Request>);
    fn nav2_msgs__action__DockRobot_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockRobot_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DockRobot_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DockRobot_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::DockRobot_Goal,
}



impl Default for DockRobot_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DockRobot_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DockRobot_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockRobot_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockRobot_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockRobot_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DockRobot_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DockRobot_SendGoal_Response__init(msg: *mut DockRobot_SendGoal_Response) -> bool;
    fn nav2_msgs__action__DockRobot_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DockRobot_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_SendGoal_Response>);
    fn nav2_msgs__action__DockRobot_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockRobot_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DockRobot_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DockRobot_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for DockRobot_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DockRobot_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DockRobot_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockRobot_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockRobot_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockRobot_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DockRobot_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DockRobot_GetResult_Request__init(msg: *mut DockRobot_GetResult_Request) -> bool;
    fn nav2_msgs__action__DockRobot_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DockRobot_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_GetResult_Request>);
    fn nav2_msgs__action__DockRobot_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockRobot_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DockRobot_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DockRobot_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for DockRobot_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DockRobot_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DockRobot_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockRobot_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockRobot_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockRobot_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DockRobot_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DockRobot_GetResult_Response__init(msg: *mut DockRobot_GetResult_Response) -> bool;
    fn nav2_msgs__action__DockRobot_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DockRobot_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockRobot_GetResult_Response>);
    fn nav2_msgs__action__DockRobot_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockRobot_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DockRobot_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DockRobot_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::DockRobot_Result,
}



impl Default for DockRobot_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DockRobot_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DockRobot_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockRobot_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DockRobot_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockRobot_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockRobot_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DockRobot_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DockRobot_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__UndockRobot_SendGoal_Request__init(msg: *mut UndockRobot_SendGoal_Request) -> bool;
    fn nav2_msgs__action__UndockRobot_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__UndockRobot_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_SendGoal_Request>);
    fn nav2_msgs__action__UndockRobot_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UndockRobot_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__UndockRobot_SendGoal_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: crate::action::rmw::UndockRobot_Goal,
}



impl Default for UndockRobot_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__UndockRobot_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__UndockRobot_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UndockRobot_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UndockRobot_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/UndockRobot_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__UndockRobot_SendGoal_Response__init(msg: *mut UndockRobot_SendGoal_Response) -> bool;
    fn nav2_msgs__action__UndockRobot_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__UndockRobot_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_SendGoal_Response>);
    fn nav2_msgs__action__UndockRobot_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UndockRobot_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__UndockRobot_SendGoal_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}



impl Default for UndockRobot_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__UndockRobot_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__UndockRobot_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UndockRobot_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UndockRobot_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/UndockRobot_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__UndockRobot_GetResult_Request__init(msg: *mut UndockRobot_GetResult_Request) -> bool;
    fn nav2_msgs__action__UndockRobot_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__UndockRobot_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_GetResult_Request>);
    fn nav2_msgs__action__UndockRobot_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UndockRobot_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__UndockRobot_GetResult_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for UndockRobot_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__UndockRobot_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__UndockRobot_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UndockRobot_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UndockRobot_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/UndockRobot_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__UndockRobot_GetResult_Response__init(msg: *mut UndockRobot_GetResult_Response) -> bool;
    fn nav2_msgs__action__UndockRobot_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__UndockRobot_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_GetResult_Response>);
    fn nav2_msgs__action__UndockRobot_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UndockRobot_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<UndockRobot_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__UndockRobot_GetResult_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_GetResult_Response {
    pub status: i8,
    pub result: crate::action::rmw::UndockRobot_Result,
}



impl Default for UndockRobot_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__UndockRobot_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__UndockRobot_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UndockRobot_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__UndockRobot_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UndockRobot_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/UndockRobot_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__UndockRobot_GetResult_Response() }
  }
}






  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal
  pub struct AssistedTeleop_SendGoal;

  impl rosidl_runtime_rs::Service for AssistedTeleop_SendGoal {
    type Request = crate::action::rmw::AssistedTeleop_SendGoal_Request;
    type Response = crate::action::rmw::AssistedTeleop_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__AssistedTeleop_GetResult
  pub struct AssistedTeleop_GetResult;

  impl rosidl_runtime_rs::Service for AssistedTeleop_GetResult {
    type Request = crate::action::rmw::AssistedTeleop_GetResult_Request;
    type Response = crate::action::rmw::AssistedTeleop_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__BackUp_SendGoal
  pub struct BackUp_SendGoal;

  impl rosidl_runtime_rs::Service for BackUp_SendGoal {
    type Request = crate::action::rmw::BackUp_SendGoal_Request;
    type Response = crate::action::rmw::BackUp_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__BackUp_GetResult
  pub struct BackUp_GetResult;

  impl rosidl_runtime_rs::Service for BackUp_GetResult {
    type Request = crate::action::rmw::BackUp_GetResult_Request;
    type Response = crate::action::rmw::BackUp_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal
  pub struct ComputePathToPose_SendGoal;

  impl rosidl_runtime_rs::Service for ComputePathToPose_SendGoal {
    type Request = crate::action::rmw::ComputePathToPose_SendGoal_Request;
    type Response = crate::action::rmw::ComputePathToPose_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__ComputePathToPose_GetResult
  pub struct ComputePathToPose_GetResult;

  impl rosidl_runtime_rs::Service for ComputePathToPose_GetResult {
    type Request = crate::action::rmw::ComputePathToPose_GetResult_Request;
    type Response = crate::action::rmw::ComputePathToPose_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal
  pub struct ComputePathThroughPoses_SendGoal;

  impl rosidl_runtime_rs::Service for ComputePathThroughPoses_SendGoal {
    type Request = crate::action::rmw::ComputePathThroughPoses_SendGoal_Request;
    type Response = crate::action::rmw::ComputePathThroughPoses_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult
  pub struct ComputePathThroughPoses_GetResult;

  impl rosidl_runtime_rs::Service for ComputePathThroughPoses_GetResult {
    type Request = crate::action::rmw::ComputePathThroughPoses_GetResult_Request;
    type Response = crate::action::rmw::ComputePathThroughPoses_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal
  pub struct DriveOnHeading_SendGoal;

  impl rosidl_runtime_rs::Service for DriveOnHeading_SendGoal {
    type Request = crate::action::rmw::DriveOnHeading_SendGoal_Request;
    type Response = crate::action::rmw::DriveOnHeading_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__DriveOnHeading_GetResult
  pub struct DriveOnHeading_GetResult;

  impl rosidl_runtime_rs::Service for DriveOnHeading_GetResult {
    type Request = crate::action::rmw::DriveOnHeading_GetResult_Request;
    type Response = crate::action::rmw::DriveOnHeading_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__SmoothPath_SendGoal
  pub struct SmoothPath_SendGoal;

  impl rosidl_runtime_rs::Service for SmoothPath_SendGoal {
    type Request = crate::action::rmw::SmoothPath_SendGoal_Request;
    type Response = crate::action::rmw::SmoothPath_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__SmoothPath_GetResult
  pub struct SmoothPath_GetResult;

  impl rosidl_runtime_rs::Service for SmoothPath_GetResult {
    type Request = crate::action::rmw::SmoothPath_GetResult_Request;
    type Response = crate::action::rmw::SmoothPath_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__FollowPath_SendGoal
  pub struct FollowPath_SendGoal;

  impl rosidl_runtime_rs::Service for FollowPath_SendGoal {
    type Request = crate::action::rmw::FollowPath_SendGoal_Request;
    type Response = crate::action::rmw::FollowPath_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__FollowPath_GetResult
  pub struct FollowPath_GetResult;

  impl rosidl_runtime_rs::Service for FollowPath_GetResult {
    type Request = crate::action::rmw::FollowPath_GetResult_Request;
    type Response = crate::action::rmw::FollowPath_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__NavigateToPose_SendGoal
  pub struct NavigateToPose_SendGoal;

  impl rosidl_runtime_rs::Service for NavigateToPose_SendGoal {
    type Request = crate::action::rmw::NavigateToPose_SendGoal_Request;
    type Response = crate::action::rmw::NavigateToPose_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__NavigateToPose_GetResult
  pub struct NavigateToPose_GetResult;

  impl rosidl_runtime_rs::Service for NavigateToPose_GetResult {
    type Request = crate::action::rmw::NavigateToPose_GetResult_Request;
    type Response = crate::action::rmw::NavigateToPose_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal
  pub struct NavigateThroughPoses_SendGoal;

  impl rosidl_runtime_rs::Service for NavigateThroughPoses_SendGoal {
    type Request = crate::action::rmw::NavigateThroughPoses_SendGoal_Request;
    type Response = crate::action::rmw::NavigateThroughPoses_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult
  pub struct NavigateThroughPoses_GetResult;

  impl rosidl_runtime_rs::Service for NavigateThroughPoses_GetResult {
    type Request = crate::action::rmw::NavigateThroughPoses_GetResult_Request;
    type Response = crate::action::rmw::NavigateThroughPoses_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__Wait_SendGoal
  pub struct Wait_SendGoal;

  impl rosidl_runtime_rs::Service for Wait_SendGoal {
    type Request = crate::action::rmw::Wait_SendGoal_Request;
    type Response = crate::action::rmw::Wait_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__Wait_GetResult
  pub struct Wait_GetResult;

  impl rosidl_runtime_rs::Service for Wait_GetResult {
    type Request = crate::action::rmw::Wait_GetResult_Request;
    type Response = crate::action::rmw::Wait_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__Spin_SendGoal
  pub struct Spin_SendGoal;

  impl rosidl_runtime_rs::Service for Spin_SendGoal {
    type Request = crate::action::rmw::Spin_SendGoal_Request;
    type Response = crate::action::rmw::Spin_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__Spin_GetResult
  pub struct Spin_GetResult;

  impl rosidl_runtime_rs::Service for Spin_GetResult {
    type Request = crate::action::rmw::Spin_GetResult_Request;
    type Response = crate::action::rmw::Spin_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__DummyBehavior_SendGoal
  pub struct DummyBehavior_SendGoal;

  impl rosidl_runtime_rs::Service for DummyBehavior_SendGoal {
    type Request = crate::action::rmw::DummyBehavior_SendGoal_Request;
    type Response = crate::action::rmw::DummyBehavior_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__DummyBehavior_GetResult
  pub struct DummyBehavior_GetResult;

  impl rosidl_runtime_rs::Service for DummyBehavior_GetResult {
    type Request = crate::action::rmw::DummyBehavior_GetResult_Request;
    type Response = crate::action::rmw::DummyBehavior_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal
  pub struct FollowWaypoints_SendGoal;

  impl rosidl_runtime_rs::Service for FollowWaypoints_SendGoal {
    type Request = crate::action::rmw::FollowWaypoints_SendGoal_Request;
    type Response = crate::action::rmw::FollowWaypoints_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__FollowWaypoints_GetResult
  pub struct FollowWaypoints_GetResult;

  impl rosidl_runtime_rs::Service for FollowWaypoints_GetResult {
    type Request = crate::action::rmw::FollowWaypoints_GetResult_Request;
    type Response = crate::action::rmw::FollowWaypoints_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__FollowGPSWaypoints_SendGoal
  pub struct FollowGPSWaypoints_SendGoal;

  impl rosidl_runtime_rs::Service for FollowGPSWaypoints_SendGoal {
    type Request = crate::action::rmw::FollowGPSWaypoints_SendGoal_Request;
    type Response = crate::action::rmw::FollowGPSWaypoints_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__FollowGPSWaypoints_GetResult
  pub struct FollowGPSWaypoints_GetResult;

  impl rosidl_runtime_rs::Service for FollowGPSWaypoints_GetResult {
    type Request = crate::action::rmw::FollowGPSWaypoints_GetResult_Request;
    type Response = crate::action::rmw::FollowGPSWaypoints_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DockRobot_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__DockRobot_SendGoal
  pub struct DockRobot_SendGoal;

  impl rosidl_runtime_rs::Service for DockRobot_SendGoal {
    type Request = crate::action::rmw::DockRobot_SendGoal_Request;
    type Response = crate::action::rmw::DockRobot_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DockRobot_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DockRobot_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__DockRobot_GetResult
  pub struct DockRobot_GetResult;

  impl rosidl_runtime_rs::Service for DockRobot_GetResult {
    type Request = crate::action::rmw::DockRobot_GetResult_Request;
    type Response = crate::action::rmw::DockRobot_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DockRobot_GetResult() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__UndockRobot_SendGoal() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__UndockRobot_SendGoal
  pub struct UndockRobot_SendGoal;

  impl rosidl_runtime_rs::Service for UndockRobot_SendGoal {
    type Request = crate::action::rmw::UndockRobot_SendGoal_Request;
    type Response = crate::action::rmw::UndockRobot_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__UndockRobot_SendGoal() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__UndockRobot_GetResult() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__action__UndockRobot_GetResult
  pub struct UndockRobot_GetResult;

  impl rosidl_runtime_rs::Service for UndockRobot_GetResult {
    type Request = crate::action::rmw::UndockRobot_GetResult_Request;
    type Response = crate::action::rmw::UndockRobot_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__UndockRobot_GetResult() }
    }
  }


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Goal {
    pub time_allowance: builtin_interfaces::msg::Duration,
}



impl Default for AssistedTeleop_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::AssistedTeleop_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Goal {
  type RmwMsg = crate::action::rmw::AssistedTeleop_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_allowance)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_allowance)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time_allowance: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_allowance),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Result {
    pub total_elapsed_time: builtin_interfaces::msg::Duration,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl AssistedTeleop_Result {
    /// Error codes
    /// Note: The expected priority order of the error should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 730;
    pub const TIMEOUT: u16 = 731;
    pub const TF_ERROR: u16 = 732;
}


impl Default for AssistedTeleop_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::AssistedTeleop_Result::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Result {
  type RmwMsg = crate::action::rmw::AssistedTeleop_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Feedback {
    pub current_teleop_duration: builtin_interfaces::msg::Duration,
}



impl Default for AssistedTeleop_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::AssistedTeleop_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Feedback {
  type RmwMsg = crate::action::rmw::AssistedTeleop_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_teleop_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.current_teleop_duration)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_teleop_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.current_teleop_duration)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_teleop_duration: builtin_interfaces::msg::Duration::from_rmw_message(msg.current_teleop_duration),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::AssistedTeleop_Feedback,
}



impl Default for AssistedTeleop_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::AssistedTeleop_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_FeedbackMessage {
  type RmwMsg = crate::action::rmw::AssistedTeleop_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::AssistedTeleop_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::AssistedTeleop_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::AssistedTeleop_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Goal {
    pub target: geometry_msgs::msg::Point,
    pub speed: f32,
    pub time_allowance: builtin_interfaces::msg::Duration,
    pub disable_collision_checks: bool,
}



impl Default for BackUp_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::BackUp_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_Goal {
  type RmwMsg = crate::action::rmw::BackUp_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.target)).into_owned(),
        speed: msg.speed,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_allowance)).into_owned(),
        disable_collision_checks: msg.disable_collision_checks,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.target)).into_owned(),
      speed: msg.speed,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_allowance)).into_owned(),
      disable_collision_checks: msg.disable_collision_checks,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target: geometry_msgs::msg::Point::from_rmw_message(msg.target),
      speed: msg.speed,
      time_allowance: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_allowance),
      disable_collision_checks: msg.disable_collision_checks,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Result {
    pub total_elapsed_time: builtin_interfaces::msg::Duration,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl BackUp_Result {
    /// Error codes
    /// Note: The expected priority order of the error should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 710;
    pub const TIMEOUT: u16 = 711;
    pub const TF_ERROR: u16 = 712;
    pub const INVALID_INPUT: u16 = 713;
    pub const COLLISION_AHEAD: u16 = 714;
}


impl Default for BackUp_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::BackUp_Result::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_Result {
  type RmwMsg = crate::action::rmw::BackUp_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Feedback {
    pub distance_traveled: f32,
}



impl Default for BackUp_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::BackUp_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_Feedback {
  type RmwMsg = crate::action::rmw::BackUp_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        distance_traveled: msg.distance_traveled,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      distance_traveled: msg.distance_traveled,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      distance_traveled: msg.distance_traveled,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::BackUp_Feedback,
}



impl Default for BackUp_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::BackUp_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_FeedbackMessage {
  type RmwMsg = crate::action::rmw::BackUp_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::BackUp_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::BackUp_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::BackUp_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Goal {
    pub goal: geometry_msgs::msg::PoseStamped,
    pub start: geometry_msgs::msg::PoseStamped,
    pub planner_id: std::string::String,
    pub use_start: bool,
}



impl Default for ComputePathToPose_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathToPose_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Goal {
  type RmwMsg = crate::action::rmw::ComputePathToPose_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        planner_id: msg.planner_id.as_str().into(),
        use_start: msg.use_start,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        planner_id: msg.planner_id.as_str().into(),
      use_start: msg.use_start,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.goal),
      start: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.start),
      planner_id: msg.planner_id.to_string(),
      use_start: msg.use_start,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Result {
    pub path: nav_msgs::msg::Path,
    pub planning_time: builtin_interfaces::msg::Duration,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl ComputePathToPose_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 200;
    pub const INVALID_PLANNER: u16 = 201;
    pub const TF_ERROR: u16 = 202;
    pub const START_OUTSIDE_MAP: u16 = 203;
    pub const GOAL_OUTSIDE_MAP: u16 = 204;
    pub const START_OCCUPIED: u16 = 205;
    pub const GOAL_OCCUPIED: u16 = 206;
    pub const TIMEOUT: u16 = 207;
    pub const NO_VALID_PATH: u16 = 208;
}


impl Default for ComputePathToPose_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathToPose_Result::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Result {
  type RmwMsg = crate::action::rmw::ComputePathToPose_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.planning_time)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.planning_time)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      planning_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.planning_time),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for ComputePathToPose_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathToPose_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Feedback {
  type RmwMsg = crate::action::rmw::ComputePathToPose_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::ComputePathToPose_Feedback,
}



impl Default for ComputePathToPose_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathToPose_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_FeedbackMessage {
  type RmwMsg = crate::action::rmw::ComputePathToPose_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::ComputePathToPose_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::ComputePathToPose_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::ComputePathToPose_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Goal {
    pub goals: Vec<geometry_msgs::msg::PoseStamped>,
    pub start: geometry_msgs::msg::PoseStamped,
    pub planner_id: std::string::String,
    pub use_start: bool,
}



impl Default for ComputePathThroughPoses_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathThroughPoses_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Goal {
  type RmwMsg = crate::action::rmw::ComputePathThroughPoses_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goals: msg.goals
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        planner_id: msg.planner_id.as_str().into(),
        use_start: msg.use_start,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goals: msg.goals
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        planner_id: msg.planner_id.as_str().into(),
      use_start: msg.use_start,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goals: msg.goals
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
      start: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.start),
      planner_id: msg.planner_id.to_string(),
      use_start: msg.use_start,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Result {
    pub path: nav_msgs::msg::Path,
    pub planning_time: builtin_interfaces::msg::Duration,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl ComputePathThroughPoses_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 300;
    pub const INVALID_PLANNER: u16 = 301;
    pub const TF_ERROR: u16 = 302;
    pub const START_OUTSIDE_MAP: u16 = 303;
    pub const GOAL_OUTSIDE_MAP: u16 = 304;
    pub const START_OCCUPIED: u16 = 305;
    pub const GOAL_OCCUPIED: u16 = 306;
    pub const TIMEOUT: u16 = 307;
    pub const NO_VALID_PATH: u16 = 308;
    pub const NO_VIAPOINTS_GIVEN: u16 = 309;
}


impl Default for ComputePathThroughPoses_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathThroughPoses_Result::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Result {
  type RmwMsg = crate::action::rmw::ComputePathThroughPoses_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.planning_time)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.planning_time)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      planning_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.planning_time),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for ComputePathThroughPoses_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathThroughPoses_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Feedback {
  type RmwMsg = crate::action::rmw::ComputePathThroughPoses_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::ComputePathThroughPoses_Feedback,
}



impl Default for ComputePathThroughPoses_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathThroughPoses_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_FeedbackMessage {
  type RmwMsg = crate::action::rmw::ComputePathThroughPoses_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::ComputePathThroughPoses_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::ComputePathThroughPoses_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::ComputePathThroughPoses_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Goal {
    pub target: geometry_msgs::msg::Point,
    pub speed: f32,
    pub time_allowance: builtin_interfaces::msg::Duration,
    pub disable_collision_checks: bool,
}



impl Default for DriveOnHeading_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DriveOnHeading_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Goal {
  type RmwMsg = crate::action::rmw::DriveOnHeading_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.target)).into_owned(),
        speed: msg.speed,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_allowance)).into_owned(),
        disable_collision_checks: msg.disable_collision_checks,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.target)).into_owned(),
      speed: msg.speed,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_allowance)).into_owned(),
      disable_collision_checks: msg.disable_collision_checks,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target: geometry_msgs::msg::Point::from_rmw_message(msg.target),
      speed: msg.speed,
      time_allowance: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_allowance),
      disable_collision_checks: msg.disable_collision_checks,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Result {
    pub total_elapsed_time: builtin_interfaces::msg::Duration,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl DriveOnHeading_Result {
    /// Error codes
    /// Note: The expected priority order of the error should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 720;
    pub const TIMEOUT: u16 = 721;
    pub const TF_ERROR: u16 = 722;
    pub const COLLISION_AHEAD: u16 = 723;
    pub const INVALID_INPUT: u16 = 724;
}


impl Default for DriveOnHeading_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DriveOnHeading_Result::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Result {
  type RmwMsg = crate::action::rmw::DriveOnHeading_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Feedback {
    pub distance_traveled: f32,
}



impl Default for DriveOnHeading_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DriveOnHeading_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Feedback {
  type RmwMsg = crate::action::rmw::DriveOnHeading_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        distance_traveled: msg.distance_traveled,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      distance_traveled: msg.distance_traveled,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      distance_traveled: msg.distance_traveled,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::DriveOnHeading_Feedback,
}



impl Default for DriveOnHeading_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DriveOnHeading_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_FeedbackMessage {
  type RmwMsg = crate::action::rmw::DriveOnHeading_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::DriveOnHeading_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::DriveOnHeading_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::DriveOnHeading_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Goal {
    pub path: nav_msgs::msg::Path,
    pub smoother_id: std::string::String,
    pub max_smoothing_duration: builtin_interfaces::msg::Duration,
    pub check_for_collisions: bool,
}



impl Default for SmoothPath_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::SmoothPath_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Goal {
  type RmwMsg = crate::action::rmw::SmoothPath_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        smoother_id: msg.smoother_id.as_str().into(),
        max_smoothing_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.max_smoothing_duration)).into_owned(),
        check_for_collisions: msg.check_for_collisions,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        smoother_id: msg.smoother_id.as_str().into(),
        max_smoothing_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.max_smoothing_duration)).into_owned(),
      check_for_collisions: msg.check_for_collisions,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      smoother_id: msg.smoother_id.to_string(),
      max_smoothing_duration: builtin_interfaces::msg::Duration::from_rmw_message(msg.max_smoothing_duration),
      check_for_collisions: msg.check_for_collisions,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Result {
    pub path: nav_msgs::msg::Path,
    pub smoothing_duration: builtin_interfaces::msg::Duration,
    pub was_completed: bool,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl SmoothPath_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 500;
    pub const INVALID_SMOOTHER: u16 = 501;
    pub const TIMEOUT: u16 = 502;
    pub const SMOOTHED_PATH_IN_COLLISION: u16 = 503;
    pub const FAILED_TO_SMOOTH_PATH: u16 = 504;
    pub const INVALID_PATH: u16 = 505;
}


impl Default for SmoothPath_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::SmoothPath_Result::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Result {
  type RmwMsg = crate::action::rmw::SmoothPath_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        smoothing_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.smoothing_duration)).into_owned(),
        was_completed: msg.was_completed,
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        smoothing_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.smoothing_duration)).into_owned(),
      was_completed: msg.was_completed,
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      smoothing_duration: builtin_interfaces::msg::Duration::from_rmw_message(msg.smoothing_duration),
      was_completed: msg.was_completed,
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for SmoothPath_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::SmoothPath_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Feedback {
  type RmwMsg = crate::action::rmw::SmoothPath_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::SmoothPath_Feedback,
}



impl Default for SmoothPath_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::SmoothPath_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_FeedbackMessage {
  type RmwMsg = crate::action::rmw::SmoothPath_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::SmoothPath_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::SmoothPath_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::SmoothPath_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Goal {
    pub path: nav_msgs::msg::Path,
    pub controller_id: std::string::String,
    pub goal_checker_id: std::string::String,
    pub progress_checker_id: std::string::String,
}



impl Default for FollowPath_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowPath_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Goal {
  type RmwMsg = crate::action::rmw::FollowPath_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        controller_id: msg.controller_id.as_str().into(),
        goal_checker_id: msg.goal_checker_id.as_str().into(),
        progress_checker_id: msg.progress_checker_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        controller_id: msg.controller_id.as_str().into(),
        goal_checker_id: msg.goal_checker_id.as_str().into(),
        progress_checker_id: msg.progress_checker_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      controller_id: msg.controller_id.to_string(),
      goal_checker_id: msg.goal_checker_id.to_string(),
      progress_checker_id: msg.progress_checker_id.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Result {
    pub result: std_msgs::msg::Empty,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl FollowPath_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 100;
    pub const INVALID_CONTROLLER: u16 = 101;
    pub const TF_ERROR: u16 = 102;
    pub const INVALID_PATH: u16 = 103;
    pub const PATIENCE_EXCEEDED: u16 = 104;
    pub const FAILED_TO_MAKE_PROGRESS: u16 = 105;
    pub const NO_VALID_CONTROL: u16 = 106;
    pub const CONTROLLER_TIMED_OUT: u16 = 107;
}


impl Default for FollowPath_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowPath_Result::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Result {
  type RmwMsg = crate::action::rmw::FollowPath_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      result: std_msgs::msg::Empty::from_rmw_message(msg.result),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Feedback {
    pub distance_to_goal: f32,
    pub speed: f32,
}



impl Default for FollowPath_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowPath_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Feedback {
  type RmwMsg = crate::action::rmw::FollowPath_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        distance_to_goal: msg.distance_to_goal,
        speed: msg.speed,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      distance_to_goal: msg.distance_to_goal,
      speed: msg.speed,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      distance_to_goal: msg.distance_to_goal,
      speed: msg.speed,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::FollowPath_Feedback,
}



impl Default for FollowPath_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowPath_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_FeedbackMessage {
  type RmwMsg = crate::action::rmw::FollowPath_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::FollowPath_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::FollowPath_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::FollowPath_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Goal {
    pub pose: geometry_msgs::msg::PoseStamped,
    pub behavior_tree: std::string::String,
}



impl Default for NavigateToPose_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateToPose_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Goal {
  type RmwMsg = crate::action::rmw::NavigateToPose_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        behavior_tree: msg.behavior_tree.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        behavior_tree: msg.behavior_tree.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.pose),
      behavior_tree: msg.behavior_tree.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Result {
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl NavigateToPose_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
}


impl Default for NavigateToPose_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateToPose_Result::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Result {
  type RmwMsg = crate::action::rmw::NavigateToPose_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Feedback {
    pub current_pose: geometry_msgs::msg::PoseStamped,
    pub navigation_time: builtin_interfaces::msg::Duration,
    pub estimated_time_remaining: builtin_interfaces::msg::Duration,
    pub number_of_recoveries: i16,
    pub distance_remaining: f32,
}



impl Default for NavigateToPose_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateToPose_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Feedback {
  type RmwMsg = crate::action::rmw::NavigateToPose_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.current_pose)).into_owned(),
        navigation_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.navigation_time)).into_owned(),
        estimated_time_remaining: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.estimated_time_remaining)).into_owned(),
        number_of_recoveries: msg.number_of_recoveries,
        distance_remaining: msg.distance_remaining,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.current_pose)).into_owned(),
        navigation_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.navigation_time)).into_owned(),
        estimated_time_remaining: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.estimated_time_remaining)).into_owned(),
      number_of_recoveries: msg.number_of_recoveries,
      distance_remaining: msg.distance_remaining,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.current_pose),
      navigation_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.navigation_time),
      estimated_time_remaining: builtin_interfaces::msg::Duration::from_rmw_message(msg.estimated_time_remaining),
      number_of_recoveries: msg.number_of_recoveries,
      distance_remaining: msg.distance_remaining,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::NavigateToPose_Feedback,
}



impl Default for NavigateToPose_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateToPose_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_FeedbackMessage {
  type RmwMsg = crate::action::rmw::NavigateToPose_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::NavigateToPose_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::NavigateToPose_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::NavigateToPose_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Goal {
    pub poses: Vec<geometry_msgs::msg::PoseStamped>,
    pub behavior_tree: std::string::String,
}



impl Default for NavigateThroughPoses_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateThroughPoses_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Goal {
  type RmwMsg = crate::action::rmw::NavigateThroughPoses_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        poses: msg.poses
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        behavior_tree: msg.behavior_tree.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        poses: msg.poses
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        behavior_tree: msg.behavior_tree.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      poses: msg.poses
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
      behavior_tree: msg.behavior_tree.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Result {
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl NavigateThroughPoses_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
}


impl Default for NavigateThroughPoses_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateThroughPoses_Result::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Result {
  type RmwMsg = crate::action::rmw::NavigateThroughPoses_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Feedback {
    pub current_pose: geometry_msgs::msg::PoseStamped,
    pub navigation_time: builtin_interfaces::msg::Duration,
    pub estimated_time_remaining: builtin_interfaces::msg::Duration,
    pub number_of_recoveries: i16,
    pub distance_remaining: f32,
    pub number_of_poses_remaining: i16,
}



impl Default for NavigateThroughPoses_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateThroughPoses_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Feedback {
  type RmwMsg = crate::action::rmw::NavigateThroughPoses_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.current_pose)).into_owned(),
        navigation_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.navigation_time)).into_owned(),
        estimated_time_remaining: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.estimated_time_remaining)).into_owned(),
        number_of_recoveries: msg.number_of_recoveries,
        distance_remaining: msg.distance_remaining,
        number_of_poses_remaining: msg.number_of_poses_remaining,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.current_pose)).into_owned(),
        navigation_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.navigation_time)).into_owned(),
        estimated_time_remaining: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.estimated_time_remaining)).into_owned(),
      number_of_recoveries: msg.number_of_recoveries,
      distance_remaining: msg.distance_remaining,
      number_of_poses_remaining: msg.number_of_poses_remaining,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.current_pose),
      navigation_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.navigation_time),
      estimated_time_remaining: builtin_interfaces::msg::Duration::from_rmw_message(msg.estimated_time_remaining),
      number_of_recoveries: msg.number_of_recoveries,
      distance_remaining: msg.distance_remaining,
      number_of_poses_remaining: msg.number_of_poses_remaining,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::NavigateThroughPoses_Feedback,
}



impl Default for NavigateThroughPoses_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateThroughPoses_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_FeedbackMessage {
  type RmwMsg = crate::action::rmw::NavigateThroughPoses_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::NavigateThroughPoses_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::NavigateThroughPoses_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::NavigateThroughPoses_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Goal {
    pub time: builtin_interfaces::msg::Duration,
}



impl Default for Wait_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Wait_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_Goal {
  type RmwMsg = crate::action::rmw::Wait_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time: builtin_interfaces::msg::Duration::from_rmw_message(msg.time),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Result {
    pub total_elapsed_time: builtin_interfaces::msg::Duration,
    pub error_code: u16,
    pub error_msg: std::string::String,
}



impl Default for Wait_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Wait_Result::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_Result {
  type RmwMsg = crate::action::rmw::Wait_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Feedback {
    pub time_left: builtin_interfaces::msg::Duration,
}



impl Default for Wait_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Wait_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_Feedback {
  type RmwMsg = crate::action::rmw::Wait_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_left: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_left)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_left: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_left)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time_left: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_left),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::Wait_Feedback,
}



impl Default for Wait_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Wait_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_FeedbackMessage {
  type RmwMsg = crate::action::rmw::Wait_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::Wait_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::Wait_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::Wait_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Goal {
    pub target_yaw: f32,
    pub time_allowance: builtin_interfaces::msg::Duration,
    pub disable_collision_checks: bool,
}



impl Default for Spin_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Spin_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_Goal {
  type RmwMsg = crate::action::rmw::Spin_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target_yaw: msg.target_yaw,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_allowance)).into_owned(),
        disable_collision_checks: msg.disable_collision_checks,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      target_yaw: msg.target_yaw,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_allowance)).into_owned(),
      disable_collision_checks: msg.disable_collision_checks,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target_yaw: msg.target_yaw,
      time_allowance: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_allowance),
      disable_collision_checks: msg.disable_collision_checks,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Result {
    pub total_elapsed_time: builtin_interfaces::msg::Duration,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl Spin_Result {
    /// Error codes
    /// Note: The expected priority order of the error should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 700;
    pub const TIMEOUT: u16 = 701;
    pub const TF_ERROR: u16 = 702;
    pub const COLLISION_AHEAD: u16 = 703;
}


impl Default for Spin_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Spin_Result::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_Result {
  type RmwMsg = crate::action::rmw::Spin_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Feedback {
    pub angular_distance_traveled: f32,
}



impl Default for Spin_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Spin_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_Feedback {
  type RmwMsg = crate::action::rmw::Spin_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angular_distance_traveled: msg.angular_distance_traveled,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      angular_distance_traveled: msg.angular_distance_traveled,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      angular_distance_traveled: msg.angular_distance_traveled,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::Spin_Feedback,
}



impl Default for Spin_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Spin_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_FeedbackMessage {
  type RmwMsg = crate::action::rmw::Spin_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::Spin_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::Spin_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::Spin_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Goal {
    pub command: std_msgs::msg::String,
}



impl Default for DummyBehavior_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DummyBehavior_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Goal {
  type RmwMsg = crate::action::rmw::DummyBehavior_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: std_msgs::msg::String::into_rmw_message(std::borrow::Cow::Owned(msg.command)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: std_msgs::msg::String::into_rmw_message(std::borrow::Cow::Borrowed(&msg.command)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      command: std_msgs::msg::String::from_rmw_message(msg.command),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Result {
    pub total_elapsed_time: builtin_interfaces::msg::Duration,
    pub error_code: u16,
    pub error_msg: std::string::String,
}



impl Default for DummyBehavior_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DummyBehavior_Result::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Result {
  type RmwMsg = crate::action::rmw::DummyBehavior_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for DummyBehavior_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DummyBehavior_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Feedback {
  type RmwMsg = crate::action::rmw::DummyBehavior_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::DummyBehavior_Feedback,
}



impl Default for DummyBehavior_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DummyBehavior_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_FeedbackMessage {
  type RmwMsg = crate::action::rmw::DummyBehavior_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::DummyBehavior_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::DummyBehavior_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::DummyBehavior_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Goal {
    pub number_of_loops: u32,
    pub goal_index: u32,
    pub poses: Vec<geometry_msgs::msg::PoseStamped>,
}



impl Default for FollowWaypoints_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowWaypoints_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Goal {
  type RmwMsg = crate::action::rmw::FollowWaypoints_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        number_of_loops: msg.number_of_loops,
        goal_index: msg.goal_index,
        poses: msg.poses
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      number_of_loops: msg.number_of_loops,
      goal_index: msg.goal_index,
        poses: msg.poses
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      number_of_loops: msg.number_of_loops,
      goal_index: msg.goal_index,
      poses: msg.poses
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Result {
    pub missed_waypoints: Vec<crate::msg::MissedWaypoint>,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl FollowWaypoints_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 600;
    pub const TASK_EXECUTOR_FAILED: u16 = 601;
}


impl Default for FollowWaypoints_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowWaypoints_Result::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Result {
  type RmwMsg = crate::action::rmw::FollowWaypoints_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        missed_waypoints: msg.missed_waypoints
          .into_iter()
          .map(|elem| crate::msg::MissedWaypoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        missed_waypoints: msg.missed_waypoints
          .iter()
          .map(|elem| crate::msg::MissedWaypoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      missed_waypoints: msg.missed_waypoints
          .into_iter()
          .map(crate::msg::MissedWaypoint::from_rmw_message)
          .collect(),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Feedback {
    pub current_waypoint: u32,
}



impl Default for FollowWaypoints_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowWaypoints_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Feedback {
  type RmwMsg = crate::action::rmw::FollowWaypoints_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_waypoint: msg.current_waypoint,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      current_waypoint: msg.current_waypoint,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_waypoint: msg.current_waypoint,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::FollowWaypoints_Feedback,
}



impl Default for FollowWaypoints_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowWaypoints_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_FeedbackMessage {
  type RmwMsg = crate::action::rmw::FollowWaypoints_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::FollowWaypoints_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::FollowWaypoints_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::FollowWaypoints_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_Goal {
    pub number_of_loops: u32,
    pub goal_index: u32,
    pub gps_poses: Vec<geographic_msgs::msg::GeoPose>,
}



impl Default for FollowGPSWaypoints_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowGPSWaypoints_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_Goal {
  type RmwMsg = crate::action::rmw::FollowGPSWaypoints_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        number_of_loops: msg.number_of_loops,
        goal_index: msg.goal_index,
        gps_poses: msg.gps_poses
          .into_iter()
          .map(|elem| geographic_msgs::msg::GeoPose::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      number_of_loops: msg.number_of_loops,
      goal_index: msg.goal_index,
        gps_poses: msg.gps_poses
          .iter()
          .map(|elem| geographic_msgs::msg::GeoPose::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      number_of_loops: msg.number_of_loops,
      goal_index: msg.goal_index,
      gps_poses: msg.gps_poses
          .into_iter()
          .map(geographic_msgs::msg::GeoPose::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_Result {
    pub missed_waypoints: Vec<crate::msg::MissedWaypoint>,
    pub error_code: i16,
    pub error_msg: std::string::String,
}

impl FollowGPSWaypoints_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 600;
    pub const TASK_EXECUTOR_FAILED: u16 = 601;
}


impl Default for FollowGPSWaypoints_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowGPSWaypoints_Result::default())
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_Result {
  type RmwMsg = crate::action::rmw::FollowGPSWaypoints_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        missed_waypoints: msg.missed_waypoints
          .into_iter()
          .map(|elem| crate::msg::MissedWaypoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        missed_waypoints: msg.missed_waypoints
          .iter()
          .map(|elem| crate::msg::MissedWaypoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      missed_waypoints: msg.missed_waypoints
          .into_iter()
          .map(crate::msg::MissedWaypoint::from_rmw_message)
          .collect(),
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_Feedback {
    pub current_waypoint: u32,
}



impl Default for FollowGPSWaypoints_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowGPSWaypoints_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_Feedback {
  type RmwMsg = crate::action::rmw::FollowGPSWaypoints_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_waypoint: msg.current_waypoint,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      current_waypoint: msg.current_waypoint,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_waypoint: msg.current_waypoint,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::FollowGPSWaypoints_Feedback,
}



impl Default for FollowGPSWaypoints_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowGPSWaypoints_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_FeedbackMessage {
  type RmwMsg = crate::action::rmw::FollowGPSWaypoints_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::FollowGPSWaypoints_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::FollowGPSWaypoints_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::FollowGPSWaypoints_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_Goal {
    pub use_dock_id: bool,
    pub dock_id: std::string::String,
    pub dock_pose: geometry_msgs::msg::PoseStamped,
    pub dock_type: std::string::String,
    pub max_staging_time: f32,
    pub navigate_to_staging_pose: bool,
}



impl Default for DockRobot_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DockRobot_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for DockRobot_Goal {
  type RmwMsg = crate::action::rmw::DockRobot_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        use_dock_id: msg.use_dock_id,
        dock_id: msg.dock_id.as_str().into(),
        dock_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.dock_pose)).into_owned(),
        dock_type: msg.dock_type.as_str().into(),
        max_staging_time: msg.max_staging_time,
        navigate_to_staging_pose: msg.navigate_to_staging_pose,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      use_dock_id: msg.use_dock_id,
        dock_id: msg.dock_id.as_str().into(),
        dock_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.dock_pose)).into_owned(),
        dock_type: msg.dock_type.as_str().into(),
      max_staging_time: msg.max_staging_time,
      navigate_to_staging_pose: msg.navigate_to_staging_pose,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      use_dock_id: msg.use_dock_id,
      dock_id: msg.dock_id.to_string(),
      dock_pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.dock_pose),
      dock_type: msg.dock_type.to_string(),
      max_staging_time: msg.max_staging_time,
      navigate_to_staging_pose: msg.navigate_to_staging_pose,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_Result {
    pub success: bool,
    pub error_code: u16,
    pub num_retries: u16,
    pub error_msg: std::string::String,
}

impl DockRobot_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const DOCK_NOT_IN_DB: u16 = 901;
    pub const DOCK_NOT_VALID: u16 = 902;
    pub const FAILED_TO_STAGE: u16 = 903;
    pub const FAILED_TO_DETECT_DOCK: u16 = 904;
    pub const FAILED_TO_CONTROL: u16 = 905;
    pub const FAILED_TO_CHARGE: u16 = 906;
    pub const UNKNOWN: u16 = 999;
}


impl Default for DockRobot_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DockRobot_Result::default())
  }
}

impl rosidl_runtime_rs::Message for DockRobot_Result {
  type RmwMsg = crate::action::rmw::DockRobot_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        error_code: msg.error_code,
        num_retries: msg.num_retries,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      error_code: msg.error_code,
      num_retries: msg.num_retries,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      error_code: msg.error_code,
      num_retries: msg.num_retries,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_Feedback {
    pub state: u16,
    pub docking_time: builtin_interfaces::msg::Duration,
    pub num_retries: u16,
}

impl DockRobot_Feedback {
    pub const NONE: u16 = 0;
    pub const NAV_TO_STAGING_POSE: u16 = 1;
    pub const INITIAL_PERCEPTION: u16 = 2;
    pub const CONTROLLING: u16 = 3;
    pub const WAIT_FOR_CHARGE: u16 = 4;
    pub const RETRY: u16 = 5;
}


impl Default for DockRobot_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DockRobot_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for DockRobot_Feedback {
  type RmwMsg = crate::action::rmw::DockRobot_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        state: msg.state,
        docking_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.docking_time)).into_owned(),
        num_retries: msg.num_retries,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      state: msg.state,
        docking_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.docking_time)).into_owned(),
      num_retries: msg.num_retries,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      state: msg.state,
      docking_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.docking_time),
      num_retries: msg.num_retries,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::DockRobot_Feedback,
}



impl Default for DockRobot_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DockRobot_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for DockRobot_FeedbackMessage {
  type RmwMsg = crate::action::rmw::DockRobot_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::DockRobot_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::DockRobot_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::DockRobot_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_Goal {
    pub dock_type: std::string::String,
    pub max_undocking_time: f32,
}



impl Default for UndockRobot_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::UndockRobot_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_Goal {
  type RmwMsg = crate::action::rmw::UndockRobot_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        dock_type: msg.dock_type.as_str().into(),
        max_undocking_time: msg.max_undocking_time,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        dock_type: msg.dock_type.as_str().into(),
      max_undocking_time: msg.max_undocking_time,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      dock_type: msg.dock_type.to_string(),
      max_undocking_time: msg.max_undocking_time,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_Result {
    pub success: bool,
    pub error_code: u16,
    pub error_msg: std::string::String,
}

impl UndockRobot_Result {
    /// Error codes
    /// Note: The expected priority order of the errors should match the message order
    pub const NONE: u16 = 0;
    pub const DOCK_NOT_VALID: u16 = 902;
    pub const FAILED_TO_CONTROL: u16 = 905;
    pub const UNKNOWN: u16 = 999;
}


impl Default for UndockRobot_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::UndockRobot_Result::default())
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_Result {
  type RmwMsg = crate::action::rmw::UndockRobot_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      error_code: msg.error_code,
        error_msg: msg.error_msg.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      error_code: msg.error_code,
      error_msg: msg.error_msg.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_Feedback {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for UndockRobot_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::UndockRobot_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_Feedback {
  type RmwMsg = crate::action::rmw::UndockRobot_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: crate::action::UndockRobot_Feedback,
}



impl Default for UndockRobot_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::UndockRobot_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_FeedbackMessage {
  type RmwMsg = crate::action::rmw::UndockRobot_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: crate::action::UndockRobot_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: crate::action::UndockRobot_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: crate::action::UndockRobot_Feedback::from_rmw_message(msg.feedback),
    }
  }
}





#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::AssistedTeleop_Goal,
}



impl Default for AssistedTeleop_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::AssistedTeleop_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_SendGoal_Request {
  type RmwMsg = crate::action::rmw::AssistedTeleop_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::AssistedTeleop_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::AssistedTeleop_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::AssistedTeleop_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for AssistedTeleop_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::AssistedTeleop_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_SendGoal_Response {
  type RmwMsg = crate::action::rmw::AssistedTeleop_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for AssistedTeleop_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::AssistedTeleop_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_GetResult_Request {
  type RmwMsg = crate::action::rmw::AssistedTeleop_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_GetResult_Response {
    pub status: i8,
    pub result: crate::action::AssistedTeleop_Result,
}



impl Default for AssistedTeleop_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::AssistedTeleop_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_GetResult_Response {
  type RmwMsg = crate::action::rmw::AssistedTeleop_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::AssistedTeleop_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::AssistedTeleop_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::AssistedTeleop_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::BackUp_Goal,
}



impl Default for BackUp_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::BackUp_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_SendGoal_Request {
  type RmwMsg = crate::action::rmw::BackUp_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::BackUp_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::BackUp_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::BackUp_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for BackUp_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::BackUp_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_SendGoal_Response {
  type RmwMsg = crate::action::rmw::BackUp_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for BackUp_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::BackUp_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_GetResult_Request {
  type RmwMsg = crate::action::rmw::BackUp_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_GetResult_Response {
    pub status: i8,
    pub result: crate::action::BackUp_Result,
}



impl Default for BackUp_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::BackUp_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_GetResult_Response {
  type RmwMsg = crate::action::rmw::BackUp_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::BackUp_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::BackUp_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::BackUp_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::ComputePathToPose_Goal,
}



impl Default for ComputePathToPose_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathToPose_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_SendGoal_Request {
  type RmwMsg = crate::action::rmw::ComputePathToPose_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::ComputePathToPose_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::ComputePathToPose_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::ComputePathToPose_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for ComputePathToPose_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathToPose_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_SendGoal_Response {
  type RmwMsg = crate::action::rmw::ComputePathToPose_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for ComputePathToPose_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathToPose_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_GetResult_Request {
  type RmwMsg = crate::action::rmw::ComputePathToPose_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_GetResult_Response {
    pub status: i8,
    pub result: crate::action::ComputePathToPose_Result,
}



impl Default for ComputePathToPose_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathToPose_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_GetResult_Response {
  type RmwMsg = crate::action::rmw::ComputePathToPose_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::ComputePathToPose_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::ComputePathToPose_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::ComputePathToPose_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::ComputePathThroughPoses_Goal,
}



impl Default for ComputePathThroughPoses_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathThroughPoses_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_SendGoal_Request {
  type RmwMsg = crate::action::rmw::ComputePathThroughPoses_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::ComputePathThroughPoses_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::ComputePathThroughPoses_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::ComputePathThroughPoses_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for ComputePathThroughPoses_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathThroughPoses_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_SendGoal_Response {
  type RmwMsg = crate::action::rmw::ComputePathThroughPoses_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for ComputePathThroughPoses_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathThroughPoses_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_GetResult_Request {
  type RmwMsg = crate::action::rmw::ComputePathThroughPoses_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_GetResult_Response {
    pub status: i8,
    pub result: crate::action::ComputePathThroughPoses_Result,
}



impl Default for ComputePathThroughPoses_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ComputePathThroughPoses_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_GetResult_Response {
  type RmwMsg = crate::action::rmw::ComputePathThroughPoses_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::ComputePathThroughPoses_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::ComputePathThroughPoses_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::ComputePathThroughPoses_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::DriveOnHeading_Goal,
}



impl Default for DriveOnHeading_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DriveOnHeading_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_SendGoal_Request {
  type RmwMsg = crate::action::rmw::DriveOnHeading_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::DriveOnHeading_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::DriveOnHeading_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::DriveOnHeading_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for DriveOnHeading_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DriveOnHeading_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_SendGoal_Response {
  type RmwMsg = crate::action::rmw::DriveOnHeading_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for DriveOnHeading_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DriveOnHeading_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_GetResult_Request {
  type RmwMsg = crate::action::rmw::DriveOnHeading_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_GetResult_Response {
    pub status: i8,
    pub result: crate::action::DriveOnHeading_Result,
}



impl Default for DriveOnHeading_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DriveOnHeading_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_GetResult_Response {
  type RmwMsg = crate::action::rmw::DriveOnHeading_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::DriveOnHeading_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::DriveOnHeading_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::DriveOnHeading_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::SmoothPath_Goal,
}



impl Default for SmoothPath_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::SmoothPath_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_SendGoal_Request {
  type RmwMsg = crate::action::rmw::SmoothPath_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::SmoothPath_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::SmoothPath_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::SmoothPath_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for SmoothPath_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::SmoothPath_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_SendGoal_Response {
  type RmwMsg = crate::action::rmw::SmoothPath_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for SmoothPath_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::SmoothPath_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_GetResult_Request {
  type RmwMsg = crate::action::rmw::SmoothPath_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_GetResult_Response {
    pub status: i8,
    pub result: crate::action::SmoothPath_Result,
}



impl Default for SmoothPath_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::SmoothPath_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_GetResult_Response {
  type RmwMsg = crate::action::rmw::SmoothPath_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::SmoothPath_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::SmoothPath_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::SmoothPath_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::FollowPath_Goal,
}



impl Default for FollowPath_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowPath_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_SendGoal_Request {
  type RmwMsg = crate::action::rmw::FollowPath_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::FollowPath_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::FollowPath_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::FollowPath_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for FollowPath_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowPath_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_SendGoal_Response {
  type RmwMsg = crate::action::rmw::FollowPath_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for FollowPath_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowPath_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_GetResult_Request {
  type RmwMsg = crate::action::rmw::FollowPath_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_GetResult_Response {
    pub status: i8,
    pub result: crate::action::FollowPath_Result,
}



impl Default for FollowPath_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowPath_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_GetResult_Response {
  type RmwMsg = crate::action::rmw::FollowPath_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::FollowPath_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::FollowPath_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::FollowPath_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::NavigateToPose_Goal,
}



impl Default for NavigateToPose_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateToPose_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_SendGoal_Request {
  type RmwMsg = crate::action::rmw::NavigateToPose_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::NavigateToPose_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::NavigateToPose_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::NavigateToPose_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for NavigateToPose_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateToPose_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_SendGoal_Response {
  type RmwMsg = crate::action::rmw::NavigateToPose_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for NavigateToPose_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateToPose_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_GetResult_Request {
  type RmwMsg = crate::action::rmw::NavigateToPose_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_GetResult_Response {
    pub status: i8,
    pub result: crate::action::NavigateToPose_Result,
}



impl Default for NavigateToPose_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateToPose_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_GetResult_Response {
  type RmwMsg = crate::action::rmw::NavigateToPose_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::NavigateToPose_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::NavigateToPose_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::NavigateToPose_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::NavigateThroughPoses_Goal,
}



impl Default for NavigateThroughPoses_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateThroughPoses_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_SendGoal_Request {
  type RmwMsg = crate::action::rmw::NavigateThroughPoses_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::NavigateThroughPoses_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::NavigateThroughPoses_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::NavigateThroughPoses_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for NavigateThroughPoses_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateThroughPoses_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_SendGoal_Response {
  type RmwMsg = crate::action::rmw::NavigateThroughPoses_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for NavigateThroughPoses_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateThroughPoses_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_GetResult_Request {
  type RmwMsg = crate::action::rmw::NavigateThroughPoses_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_GetResult_Response {
    pub status: i8,
    pub result: crate::action::NavigateThroughPoses_Result,
}



impl Default for NavigateThroughPoses_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::NavigateThroughPoses_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_GetResult_Response {
  type RmwMsg = crate::action::rmw::NavigateThroughPoses_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::NavigateThroughPoses_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::NavigateThroughPoses_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::NavigateThroughPoses_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::Wait_Goal,
}



impl Default for Wait_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Wait_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_SendGoal_Request {
  type RmwMsg = crate::action::rmw::Wait_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::Wait_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::Wait_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::Wait_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for Wait_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Wait_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_SendGoal_Response {
  type RmwMsg = crate::action::rmw::Wait_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for Wait_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Wait_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_GetResult_Request {
  type RmwMsg = crate::action::rmw::Wait_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_GetResult_Response {
    pub status: i8,
    pub result: crate::action::Wait_Result,
}



impl Default for Wait_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Wait_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_GetResult_Response {
  type RmwMsg = crate::action::rmw::Wait_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::Wait_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::Wait_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::Wait_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::Spin_Goal,
}



impl Default for Spin_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Spin_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_SendGoal_Request {
  type RmwMsg = crate::action::rmw::Spin_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::Spin_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::Spin_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::Spin_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for Spin_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Spin_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_SendGoal_Response {
  type RmwMsg = crate::action::rmw::Spin_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for Spin_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Spin_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_GetResult_Request {
  type RmwMsg = crate::action::rmw::Spin_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_GetResult_Response {
    pub status: i8,
    pub result: crate::action::Spin_Result,
}



impl Default for Spin_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::Spin_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_GetResult_Response {
  type RmwMsg = crate::action::rmw::Spin_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::Spin_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::Spin_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::Spin_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::DummyBehavior_Goal,
}



impl Default for DummyBehavior_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DummyBehavior_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_SendGoal_Request {
  type RmwMsg = crate::action::rmw::DummyBehavior_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::DummyBehavior_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::DummyBehavior_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::DummyBehavior_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for DummyBehavior_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DummyBehavior_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_SendGoal_Response {
  type RmwMsg = crate::action::rmw::DummyBehavior_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for DummyBehavior_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DummyBehavior_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_GetResult_Request {
  type RmwMsg = crate::action::rmw::DummyBehavior_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_GetResult_Response {
    pub status: i8,
    pub result: crate::action::DummyBehavior_Result,
}



impl Default for DummyBehavior_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DummyBehavior_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_GetResult_Response {
  type RmwMsg = crate::action::rmw::DummyBehavior_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::DummyBehavior_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::DummyBehavior_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::DummyBehavior_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::FollowWaypoints_Goal,
}



impl Default for FollowWaypoints_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowWaypoints_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_SendGoal_Request {
  type RmwMsg = crate::action::rmw::FollowWaypoints_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::FollowWaypoints_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::FollowWaypoints_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::FollowWaypoints_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for FollowWaypoints_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowWaypoints_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_SendGoal_Response {
  type RmwMsg = crate::action::rmw::FollowWaypoints_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for FollowWaypoints_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowWaypoints_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_GetResult_Request {
  type RmwMsg = crate::action::rmw::FollowWaypoints_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_GetResult_Response {
    pub status: i8,
    pub result: crate::action::FollowWaypoints_Result,
}



impl Default for FollowWaypoints_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowWaypoints_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_GetResult_Response {
  type RmwMsg = crate::action::rmw::FollowWaypoints_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::FollowWaypoints_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::FollowWaypoints_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::FollowWaypoints_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::FollowGPSWaypoints_Goal,
}



impl Default for FollowGPSWaypoints_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowGPSWaypoints_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_SendGoal_Request {
  type RmwMsg = crate::action::rmw::FollowGPSWaypoints_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::FollowGPSWaypoints_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::FollowGPSWaypoints_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::FollowGPSWaypoints_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for FollowGPSWaypoints_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowGPSWaypoints_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_SendGoal_Response {
  type RmwMsg = crate::action::rmw::FollowGPSWaypoints_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for FollowGPSWaypoints_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowGPSWaypoints_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_GetResult_Request {
  type RmwMsg = crate::action::rmw::FollowGPSWaypoints_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowGPSWaypoints_GetResult_Response {
    pub status: i8,
    pub result: crate::action::FollowGPSWaypoints_Result,
}



impl Default for FollowGPSWaypoints_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::FollowGPSWaypoints_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowGPSWaypoints_GetResult_Response {
  type RmwMsg = crate::action::rmw::FollowGPSWaypoints_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::FollowGPSWaypoints_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::FollowGPSWaypoints_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::FollowGPSWaypoints_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::DockRobot_Goal,
}



impl Default for DockRobot_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DockRobot_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DockRobot_SendGoal_Request {
  type RmwMsg = crate::action::rmw::DockRobot_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::DockRobot_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::DockRobot_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::DockRobot_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for DockRobot_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DockRobot_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DockRobot_SendGoal_Response {
  type RmwMsg = crate::action::rmw::DockRobot_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for DockRobot_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DockRobot_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DockRobot_GetResult_Request {
  type RmwMsg = crate::action::rmw::DockRobot_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockRobot_GetResult_Response {
    pub status: i8,
    pub result: crate::action::DockRobot_Result,
}



impl Default for DockRobot_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::DockRobot_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DockRobot_GetResult_Response {
  type RmwMsg = crate::action::rmw::DockRobot_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::DockRobot_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::DockRobot_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::DockRobot_Result::from_rmw_message(msg.result),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: crate::action::UndockRobot_Goal,
}



impl Default for UndockRobot_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::UndockRobot_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_SendGoal_Request {
  type RmwMsg = crate::action::rmw::UndockRobot_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: crate::action::UndockRobot_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: crate::action::UndockRobot_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: crate::action::UndockRobot_Goal::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::Time,
}



impl Default for UndockRobot_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::UndockRobot_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_SendGoal_Response {
  type RmwMsg = crate::action::rmw::UndockRobot_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}



impl Default for UndockRobot_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::UndockRobot_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_GetResult_Request {
  type RmwMsg = crate::action::rmw::UndockRobot_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UndockRobot_GetResult_Response {
    pub status: i8,
    pub result: crate::action::UndockRobot_Result,
}



impl Default for UndockRobot_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::UndockRobot_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for UndockRobot_GetResult_Response {
  type RmwMsg = crate::action::rmw::UndockRobot_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: crate::action::UndockRobot_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: crate::action::UndockRobot_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: crate::action::UndockRobot_Result::from_rmw_message(msg.result),
    }
  }
}






#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal
pub struct AssistedTeleop_SendGoal;

impl rosidl_runtime_rs::Service for AssistedTeleop_SendGoal {
  type Request = crate::action::AssistedTeleop_SendGoal_Request;
  type Response = crate::action::AssistedTeleop_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_GetResult
pub struct AssistedTeleop_GetResult;

impl rosidl_runtime_rs::Service for AssistedTeleop_GetResult {
  type Request = crate::action::AssistedTeleop_GetResult_Request;
  type Response = crate::action::AssistedTeleop_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__BackUp_SendGoal
pub struct BackUp_SendGoal;

impl rosidl_runtime_rs::Service for BackUp_SendGoal {
  type Request = crate::action::BackUp_SendGoal_Request;
  type Response = crate::action::BackUp_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__BackUp_GetResult
pub struct BackUp_GetResult;

impl rosidl_runtime_rs::Service for BackUp_GetResult {
  type Request = crate::action::BackUp_GetResult_Request;
  type Response = crate::action::BackUp_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal
pub struct ComputePathToPose_SendGoal;

impl rosidl_runtime_rs::Service for ComputePathToPose_SendGoal {
  type Request = crate::action::ComputePathToPose_SendGoal_Request;
  type Response = crate::action::ComputePathToPose_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_GetResult
pub struct ComputePathToPose_GetResult;

impl rosidl_runtime_rs::Service for ComputePathToPose_GetResult {
  type Request = crate::action::ComputePathToPose_GetResult_Request;
  type Response = crate::action::ComputePathToPose_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal
pub struct ComputePathThroughPoses_SendGoal;

impl rosidl_runtime_rs::Service for ComputePathThroughPoses_SendGoal {
  type Request = crate::action::ComputePathThroughPoses_SendGoal_Request;
  type Response = crate::action::ComputePathThroughPoses_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult
pub struct ComputePathThroughPoses_GetResult;

impl rosidl_runtime_rs::Service for ComputePathThroughPoses_GetResult {
  type Request = crate::action::ComputePathThroughPoses_GetResult_Request;
  type Response = crate::action::ComputePathThroughPoses_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal
pub struct DriveOnHeading_SendGoal;

impl rosidl_runtime_rs::Service for DriveOnHeading_SendGoal {
  type Request = crate::action::DriveOnHeading_SendGoal_Request;
  type Response = crate::action::DriveOnHeading_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_GetResult
pub struct DriveOnHeading_GetResult;

impl rosidl_runtime_rs::Service for DriveOnHeading_GetResult {
  type Request = crate::action::DriveOnHeading_GetResult_Request;
  type Response = crate::action::DriveOnHeading_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__SmoothPath_SendGoal
pub struct SmoothPath_SendGoal;

impl rosidl_runtime_rs::Service for SmoothPath_SendGoal {
  type Request = crate::action::SmoothPath_SendGoal_Request;
  type Response = crate::action::SmoothPath_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__SmoothPath_GetResult
pub struct SmoothPath_GetResult;

impl rosidl_runtime_rs::Service for SmoothPath_GetResult {
  type Request = crate::action::SmoothPath_GetResult_Request;
  type Response = crate::action::SmoothPath_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowPath_SendGoal
pub struct FollowPath_SendGoal;

impl rosidl_runtime_rs::Service for FollowPath_SendGoal {
  type Request = crate::action::FollowPath_SendGoal_Request;
  type Response = crate::action::FollowPath_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowPath_GetResult
pub struct FollowPath_GetResult;

impl rosidl_runtime_rs::Service for FollowPath_GetResult {
  type Request = crate::action::FollowPath_GetResult_Request;
  type Response = crate::action::FollowPath_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateToPose_SendGoal
pub struct NavigateToPose_SendGoal;

impl rosidl_runtime_rs::Service for NavigateToPose_SendGoal {
  type Request = crate::action::NavigateToPose_SendGoal_Request;
  type Response = crate::action::NavigateToPose_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateToPose_GetResult
pub struct NavigateToPose_GetResult;

impl rosidl_runtime_rs::Service for NavigateToPose_GetResult {
  type Request = crate::action::NavigateToPose_GetResult_Request;
  type Response = crate::action::NavigateToPose_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal
pub struct NavigateThroughPoses_SendGoal;

impl rosidl_runtime_rs::Service for NavigateThroughPoses_SendGoal {
  type Request = crate::action::NavigateThroughPoses_SendGoal_Request;
  type Response = crate::action::NavigateThroughPoses_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult
pub struct NavigateThroughPoses_GetResult;

impl rosidl_runtime_rs::Service for NavigateThroughPoses_GetResult {
  type Request = crate::action::NavigateThroughPoses_GetResult_Request;
  type Response = crate::action::NavigateThroughPoses_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Wait_SendGoal
pub struct Wait_SendGoal;

impl rosidl_runtime_rs::Service for Wait_SendGoal {
  type Request = crate::action::Wait_SendGoal_Request;
  type Response = crate::action::Wait_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Wait_GetResult
pub struct Wait_GetResult;

impl rosidl_runtime_rs::Service for Wait_GetResult {
  type Request = crate::action::Wait_GetResult_Request;
  type Response = crate::action::Wait_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Spin_SendGoal
pub struct Spin_SendGoal;

impl rosidl_runtime_rs::Service for Spin_SendGoal {
  type Request = crate::action::Spin_SendGoal_Request;
  type Response = crate::action::Spin_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Spin_GetResult
pub struct Spin_GetResult;

impl rosidl_runtime_rs::Service for Spin_GetResult {
  type Request = crate::action::Spin_GetResult_Request;
  type Response = crate::action::Spin_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DummyBehavior_SendGoal
pub struct DummyBehavior_SendGoal;

impl rosidl_runtime_rs::Service for DummyBehavior_SendGoal {
  type Request = crate::action::DummyBehavior_SendGoal_Request;
  type Response = crate::action::DummyBehavior_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DummyBehavior_GetResult
pub struct DummyBehavior_GetResult;

impl rosidl_runtime_rs::Service for DummyBehavior_GetResult {
  type Request = crate::action::DummyBehavior_GetResult_Request;
  type Response = crate::action::DummyBehavior_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal
pub struct FollowWaypoints_SendGoal;

impl rosidl_runtime_rs::Service for FollowWaypoints_SendGoal {
  type Request = crate::action::FollowWaypoints_SendGoal_Request;
  type Response = crate::action::FollowWaypoints_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_GetResult
pub struct FollowWaypoints_GetResult;

impl rosidl_runtime_rs::Service for FollowWaypoints_GetResult {
  type Request = crate::action::FollowWaypoints_GetResult_Request;
  type Response = crate::action::FollowWaypoints_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_SendGoal
pub struct FollowGPSWaypoints_SendGoal;

impl rosidl_runtime_rs::Service for FollowGPSWaypoints_SendGoal {
  type Request = crate::action::FollowGPSWaypoints_SendGoal_Request;
  type Response = crate::action::FollowGPSWaypoints_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints_GetResult
pub struct FollowGPSWaypoints_GetResult;

impl rosidl_runtime_rs::Service for FollowGPSWaypoints_GetResult {
  type Request = crate::action::FollowGPSWaypoints_GetResult_Request;
  type Response = crate::action::FollowGPSWaypoints_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowGPSWaypoints_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DockRobot_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DockRobot_SendGoal
pub struct DockRobot_SendGoal;

impl rosidl_runtime_rs::Service for DockRobot_SendGoal {
  type Request = crate::action::DockRobot_SendGoal_Request;
  type Response = crate::action::DockRobot_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DockRobot_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DockRobot_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DockRobot_GetResult
pub struct DockRobot_GetResult;

impl rosidl_runtime_rs::Service for DockRobot_GetResult {
  type Request = crate::action::DockRobot_GetResult_Request;
  type Response = crate::action::DockRobot_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DockRobot_GetResult() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__UndockRobot_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__UndockRobot_SendGoal
pub struct UndockRobot_SendGoal;

impl rosidl_runtime_rs::Service for UndockRobot_SendGoal {
  type Request = crate::action::UndockRobot_SendGoal_Request;
  type Response = crate::action::UndockRobot_SendGoal_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__UndockRobot_SendGoal() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__UndockRobot_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__UndockRobot_GetResult
pub struct UndockRobot_GetResult;

impl rosidl_runtime_rs::Service for UndockRobot_GetResult {
  type Request = crate::action::UndockRobot_GetResult_Request;
  type Response = crate::action::UndockRobot_GetResult_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__UndockRobot_GetResult() }
  }
}






#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__AssistedTeleop() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__AssistedTeleop
pub struct AssistedTeleop;

impl rosidl_runtime_rs::Action for AssistedTeleop {
  type Goal = crate::action::AssistedTeleop_Goal;
  type Result = crate::action::AssistedTeleop_Result;
  type Feedback = crate::action::AssistedTeleop_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__AssistedTeleop() }
  }
}

impl rosidl_runtime_rs::ActionImpl for AssistedTeleop {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::AssistedTeleop_FeedbackMessage;

  type SendGoalService = crate::action::rmw::AssistedTeleop_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::AssistedTeleop_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::AssistedTeleop_Goal,
  ) -> crate::action::rmw::AssistedTeleop_SendGoal_Request {
    crate::action::rmw::AssistedTeleop_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::AssistedTeleop_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::AssistedTeleop_SendGoal_Response {
    crate::action::rmw::AssistedTeleop_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::AssistedTeleop_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::AssistedTeleop_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::AssistedTeleop_Feedback,
  ) -> crate::action::rmw::AssistedTeleop_FeedbackMessage {
    let mut message = crate::action::rmw::AssistedTeleop_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::AssistedTeleop_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::AssistedTeleop_FeedbackMessage,
  ) -> &crate::action::rmw::AssistedTeleop_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::AssistedTeleop_GetResult_Request {
    crate::action::rmw::AssistedTeleop_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::AssistedTeleop_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::AssistedTeleop_Result,
  ) -> crate::action::rmw::AssistedTeleop_GetResult_Response {
    crate::action::rmw::AssistedTeleop_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::AssistedTeleop_GetResult_Response,
  ) -> &crate::action::rmw::AssistedTeleop_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::AssistedTeleop_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__BackUp() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__BackUp
pub struct BackUp;

impl rosidl_runtime_rs::Action for BackUp {
  type Goal = crate::action::BackUp_Goal;
  type Result = crate::action::BackUp_Result;
  type Feedback = crate::action::BackUp_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__BackUp() }
  }
}

impl rosidl_runtime_rs::ActionImpl for BackUp {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::BackUp_FeedbackMessage;

  type SendGoalService = crate::action::rmw::BackUp_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::BackUp_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::BackUp_Goal,
  ) -> crate::action::rmw::BackUp_SendGoal_Request {
    crate::action::rmw::BackUp_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::BackUp_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::BackUp_SendGoal_Response {
    crate::action::rmw::BackUp_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::BackUp_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::BackUp_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::BackUp_Feedback,
  ) -> crate::action::rmw::BackUp_FeedbackMessage {
    let mut message = crate::action::rmw::BackUp_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::BackUp_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::BackUp_FeedbackMessage,
  ) -> &crate::action::rmw::BackUp_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::BackUp_GetResult_Request {
    crate::action::rmw::BackUp_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::BackUp_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::BackUp_Result,
  ) -> crate::action::rmw::BackUp_GetResult_Response {
    crate::action::rmw::BackUp_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::BackUp_GetResult_Response,
  ) -> &crate::action::rmw::BackUp_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::BackUp_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathToPose() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathToPose
pub struct ComputePathToPose;

impl rosidl_runtime_rs::Action for ComputePathToPose {
  type Goal = crate::action::ComputePathToPose_Goal;
  type Result = crate::action::ComputePathToPose_Result;
  type Feedback = crate::action::ComputePathToPose_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathToPose() }
  }
}

impl rosidl_runtime_rs::ActionImpl for ComputePathToPose {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::ComputePathToPose_FeedbackMessage;

  type SendGoalService = crate::action::rmw::ComputePathToPose_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::ComputePathToPose_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::ComputePathToPose_Goal,
  ) -> crate::action::rmw::ComputePathToPose_SendGoal_Request {
    crate::action::rmw::ComputePathToPose_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::ComputePathToPose_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::ComputePathToPose_SendGoal_Response {
    crate::action::rmw::ComputePathToPose_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::ComputePathToPose_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::ComputePathToPose_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::ComputePathToPose_Feedback,
  ) -> crate::action::rmw::ComputePathToPose_FeedbackMessage {
    let mut message = crate::action::rmw::ComputePathToPose_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::ComputePathToPose_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::ComputePathToPose_FeedbackMessage,
  ) -> &crate::action::rmw::ComputePathToPose_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::ComputePathToPose_GetResult_Request {
    crate::action::rmw::ComputePathToPose_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::ComputePathToPose_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::ComputePathToPose_Result,
  ) -> crate::action::rmw::ComputePathToPose_GetResult_Response {
    crate::action::rmw::ComputePathToPose_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::ComputePathToPose_GetResult_Response,
  ) -> &crate::action::rmw::ComputePathToPose_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::ComputePathToPose_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathThroughPoses() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses
pub struct ComputePathThroughPoses;

impl rosidl_runtime_rs::Action for ComputePathThroughPoses {
  type Goal = crate::action::ComputePathThroughPoses_Goal;
  type Result = crate::action::ComputePathThroughPoses_Result;
  type Feedback = crate::action::ComputePathThroughPoses_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathThroughPoses() }
  }
}

impl rosidl_runtime_rs::ActionImpl for ComputePathThroughPoses {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::ComputePathThroughPoses_FeedbackMessage;

  type SendGoalService = crate::action::rmw::ComputePathThroughPoses_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::ComputePathThroughPoses_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::ComputePathThroughPoses_Goal,
  ) -> crate::action::rmw::ComputePathThroughPoses_SendGoal_Request {
    crate::action::rmw::ComputePathThroughPoses_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::ComputePathThroughPoses_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::ComputePathThroughPoses_SendGoal_Response {
    crate::action::rmw::ComputePathThroughPoses_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::ComputePathThroughPoses_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::ComputePathThroughPoses_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::ComputePathThroughPoses_Feedback,
  ) -> crate::action::rmw::ComputePathThroughPoses_FeedbackMessage {
    let mut message = crate::action::rmw::ComputePathThroughPoses_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::ComputePathThroughPoses_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::ComputePathThroughPoses_FeedbackMessage,
  ) -> &crate::action::rmw::ComputePathThroughPoses_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::ComputePathThroughPoses_GetResult_Request {
    crate::action::rmw::ComputePathThroughPoses_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::ComputePathThroughPoses_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::ComputePathThroughPoses_Result,
  ) -> crate::action::rmw::ComputePathThroughPoses_GetResult_Response {
    crate::action::rmw::ComputePathThroughPoses_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::ComputePathThroughPoses_GetResult_Response,
  ) -> &crate::action::rmw::ComputePathThroughPoses_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::ComputePathThroughPoses_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DriveOnHeading() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DriveOnHeading
pub struct DriveOnHeading;

impl rosidl_runtime_rs::Action for DriveOnHeading {
  type Goal = crate::action::DriveOnHeading_Goal;
  type Result = crate::action::DriveOnHeading_Result;
  type Feedback = crate::action::DriveOnHeading_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DriveOnHeading() }
  }
}

impl rosidl_runtime_rs::ActionImpl for DriveOnHeading {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::DriveOnHeading_FeedbackMessage;

  type SendGoalService = crate::action::rmw::DriveOnHeading_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::DriveOnHeading_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::DriveOnHeading_Goal,
  ) -> crate::action::rmw::DriveOnHeading_SendGoal_Request {
    crate::action::rmw::DriveOnHeading_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::DriveOnHeading_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::DriveOnHeading_SendGoal_Response {
    crate::action::rmw::DriveOnHeading_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::DriveOnHeading_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::DriveOnHeading_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::DriveOnHeading_Feedback,
  ) -> crate::action::rmw::DriveOnHeading_FeedbackMessage {
    let mut message = crate::action::rmw::DriveOnHeading_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::DriveOnHeading_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::DriveOnHeading_FeedbackMessage,
  ) -> &crate::action::rmw::DriveOnHeading_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::DriveOnHeading_GetResult_Request {
    crate::action::rmw::DriveOnHeading_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::DriveOnHeading_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::DriveOnHeading_Result,
  ) -> crate::action::rmw::DriveOnHeading_GetResult_Response {
    crate::action::rmw::DriveOnHeading_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::DriveOnHeading_GetResult_Response,
  ) -> &crate::action::rmw::DriveOnHeading_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::DriveOnHeading_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__SmoothPath() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__SmoothPath
pub struct SmoothPath;

impl rosidl_runtime_rs::Action for SmoothPath {
  type Goal = crate::action::SmoothPath_Goal;
  type Result = crate::action::SmoothPath_Result;
  type Feedback = crate::action::SmoothPath_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__SmoothPath() }
  }
}

impl rosidl_runtime_rs::ActionImpl for SmoothPath {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::SmoothPath_FeedbackMessage;

  type SendGoalService = crate::action::rmw::SmoothPath_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::SmoothPath_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::SmoothPath_Goal,
  ) -> crate::action::rmw::SmoothPath_SendGoal_Request {
    crate::action::rmw::SmoothPath_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::SmoothPath_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::SmoothPath_SendGoal_Response {
    crate::action::rmw::SmoothPath_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::SmoothPath_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::SmoothPath_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::SmoothPath_Feedback,
  ) -> crate::action::rmw::SmoothPath_FeedbackMessage {
    let mut message = crate::action::rmw::SmoothPath_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::SmoothPath_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::SmoothPath_FeedbackMessage,
  ) -> &crate::action::rmw::SmoothPath_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::SmoothPath_GetResult_Request {
    crate::action::rmw::SmoothPath_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::SmoothPath_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::SmoothPath_Result,
  ) -> crate::action::rmw::SmoothPath_GetResult_Response {
    crate::action::rmw::SmoothPath_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::SmoothPath_GetResult_Response,
  ) -> &crate::action::rmw::SmoothPath_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::SmoothPath_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowPath() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowPath
pub struct FollowPath;

impl rosidl_runtime_rs::Action for FollowPath {
  type Goal = crate::action::FollowPath_Goal;
  type Result = crate::action::FollowPath_Result;
  type Feedback = crate::action::FollowPath_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowPath() }
  }
}

impl rosidl_runtime_rs::ActionImpl for FollowPath {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::FollowPath_FeedbackMessage;

  type SendGoalService = crate::action::rmw::FollowPath_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::FollowPath_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::FollowPath_Goal,
  ) -> crate::action::rmw::FollowPath_SendGoal_Request {
    crate::action::rmw::FollowPath_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::FollowPath_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::FollowPath_SendGoal_Response {
    crate::action::rmw::FollowPath_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::FollowPath_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::FollowPath_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::FollowPath_Feedback,
  ) -> crate::action::rmw::FollowPath_FeedbackMessage {
    let mut message = crate::action::rmw::FollowPath_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::FollowPath_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::FollowPath_FeedbackMessage,
  ) -> &crate::action::rmw::FollowPath_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::FollowPath_GetResult_Request {
    crate::action::rmw::FollowPath_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::FollowPath_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::FollowPath_Result,
  ) -> crate::action::rmw::FollowPath_GetResult_Response {
    crate::action::rmw::FollowPath_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::FollowPath_GetResult_Response,
  ) -> &crate::action::rmw::FollowPath_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::FollowPath_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateToPose() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateToPose
pub struct NavigateToPose;

impl rosidl_runtime_rs::Action for NavigateToPose {
  type Goal = crate::action::NavigateToPose_Goal;
  type Result = crate::action::NavigateToPose_Result;
  type Feedback = crate::action::NavigateToPose_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateToPose() }
  }
}

impl rosidl_runtime_rs::ActionImpl for NavigateToPose {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::NavigateToPose_FeedbackMessage;

  type SendGoalService = crate::action::rmw::NavigateToPose_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::NavigateToPose_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::NavigateToPose_Goal,
  ) -> crate::action::rmw::NavigateToPose_SendGoal_Request {
    crate::action::rmw::NavigateToPose_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::NavigateToPose_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::NavigateToPose_SendGoal_Response {
    crate::action::rmw::NavigateToPose_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::NavigateToPose_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::NavigateToPose_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::NavigateToPose_Feedback,
  ) -> crate::action::rmw::NavigateToPose_FeedbackMessage {
    let mut message = crate::action::rmw::NavigateToPose_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::NavigateToPose_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::NavigateToPose_FeedbackMessage,
  ) -> &crate::action::rmw::NavigateToPose_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::NavigateToPose_GetResult_Request {
    crate::action::rmw::NavigateToPose_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::NavigateToPose_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::NavigateToPose_Result,
  ) -> crate::action::rmw::NavigateToPose_GetResult_Response {
    crate::action::rmw::NavigateToPose_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::NavigateToPose_GetResult_Response,
  ) -> &crate::action::rmw::NavigateToPose_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::NavigateToPose_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateThroughPoses() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses
pub struct NavigateThroughPoses;

impl rosidl_runtime_rs::Action for NavigateThroughPoses {
  type Goal = crate::action::NavigateThroughPoses_Goal;
  type Result = crate::action::NavigateThroughPoses_Result;
  type Feedback = crate::action::NavigateThroughPoses_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateThroughPoses() }
  }
}

impl rosidl_runtime_rs::ActionImpl for NavigateThroughPoses {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::NavigateThroughPoses_FeedbackMessage;

  type SendGoalService = crate::action::rmw::NavigateThroughPoses_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::NavigateThroughPoses_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::NavigateThroughPoses_Goal,
  ) -> crate::action::rmw::NavigateThroughPoses_SendGoal_Request {
    crate::action::rmw::NavigateThroughPoses_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::NavigateThroughPoses_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::NavigateThroughPoses_SendGoal_Response {
    crate::action::rmw::NavigateThroughPoses_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::NavigateThroughPoses_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::NavigateThroughPoses_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::NavigateThroughPoses_Feedback,
  ) -> crate::action::rmw::NavigateThroughPoses_FeedbackMessage {
    let mut message = crate::action::rmw::NavigateThroughPoses_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::NavigateThroughPoses_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::NavigateThroughPoses_FeedbackMessage,
  ) -> &crate::action::rmw::NavigateThroughPoses_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::NavigateThroughPoses_GetResult_Request {
    crate::action::rmw::NavigateThroughPoses_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::NavigateThroughPoses_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::NavigateThroughPoses_Result,
  ) -> crate::action::rmw::NavigateThroughPoses_GetResult_Response {
    crate::action::rmw::NavigateThroughPoses_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::NavigateThroughPoses_GetResult_Response,
  ) -> &crate::action::rmw::NavigateThroughPoses_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::NavigateThroughPoses_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Wait() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Wait
pub struct Wait;

impl rosidl_runtime_rs::Action for Wait {
  type Goal = crate::action::Wait_Goal;
  type Result = crate::action::Wait_Result;
  type Feedback = crate::action::Wait_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Wait() }
  }
}

impl rosidl_runtime_rs::ActionImpl for Wait {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::Wait_FeedbackMessage;

  type SendGoalService = crate::action::rmw::Wait_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::Wait_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::Wait_Goal,
  ) -> crate::action::rmw::Wait_SendGoal_Request {
    crate::action::rmw::Wait_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::Wait_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::Wait_SendGoal_Response {
    crate::action::rmw::Wait_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::Wait_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::Wait_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::Wait_Feedback,
  ) -> crate::action::rmw::Wait_FeedbackMessage {
    let mut message = crate::action::rmw::Wait_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::Wait_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::Wait_FeedbackMessage,
  ) -> &crate::action::rmw::Wait_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::Wait_GetResult_Request {
    crate::action::rmw::Wait_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::Wait_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::Wait_Result,
  ) -> crate::action::rmw::Wait_GetResult_Response {
    crate::action::rmw::Wait_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::Wait_GetResult_Response,
  ) -> &crate::action::rmw::Wait_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::Wait_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Spin() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Spin
pub struct Spin;

impl rosidl_runtime_rs::Action for Spin {
  type Goal = crate::action::Spin_Goal;
  type Result = crate::action::Spin_Result;
  type Feedback = crate::action::Spin_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Spin() }
  }
}

impl rosidl_runtime_rs::ActionImpl for Spin {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::Spin_FeedbackMessage;

  type SendGoalService = crate::action::rmw::Spin_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::Spin_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::Spin_Goal,
  ) -> crate::action::rmw::Spin_SendGoal_Request {
    crate::action::rmw::Spin_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::Spin_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::Spin_SendGoal_Response {
    crate::action::rmw::Spin_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::Spin_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::Spin_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::Spin_Feedback,
  ) -> crate::action::rmw::Spin_FeedbackMessage {
    let mut message = crate::action::rmw::Spin_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::Spin_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::Spin_FeedbackMessage,
  ) -> &crate::action::rmw::Spin_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::Spin_GetResult_Request {
    crate::action::rmw::Spin_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::Spin_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::Spin_Result,
  ) -> crate::action::rmw::Spin_GetResult_Response {
    crate::action::rmw::Spin_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::Spin_GetResult_Response,
  ) -> &crate::action::rmw::Spin_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::Spin_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DummyBehavior() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DummyBehavior
pub struct DummyBehavior;

impl rosidl_runtime_rs::Action for DummyBehavior {
  type Goal = crate::action::DummyBehavior_Goal;
  type Result = crate::action::DummyBehavior_Result;
  type Feedback = crate::action::DummyBehavior_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DummyBehavior() }
  }
}

impl rosidl_runtime_rs::ActionImpl for DummyBehavior {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::DummyBehavior_FeedbackMessage;

  type SendGoalService = crate::action::rmw::DummyBehavior_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::DummyBehavior_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::DummyBehavior_Goal,
  ) -> crate::action::rmw::DummyBehavior_SendGoal_Request {
    crate::action::rmw::DummyBehavior_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::DummyBehavior_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::DummyBehavior_SendGoal_Response {
    crate::action::rmw::DummyBehavior_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::DummyBehavior_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::DummyBehavior_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::DummyBehavior_Feedback,
  ) -> crate::action::rmw::DummyBehavior_FeedbackMessage {
    let mut message = crate::action::rmw::DummyBehavior_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::DummyBehavior_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::DummyBehavior_FeedbackMessage,
  ) -> &crate::action::rmw::DummyBehavior_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::DummyBehavior_GetResult_Request {
    crate::action::rmw::DummyBehavior_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::DummyBehavior_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::DummyBehavior_Result,
  ) -> crate::action::rmw::DummyBehavior_GetResult_Response {
    crate::action::rmw::DummyBehavior_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::DummyBehavior_GetResult_Response,
  ) -> &crate::action::rmw::DummyBehavior_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::DummyBehavior_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowWaypoints() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowWaypoints
pub struct FollowWaypoints;

impl rosidl_runtime_rs::Action for FollowWaypoints {
  type Goal = crate::action::FollowWaypoints_Goal;
  type Result = crate::action::FollowWaypoints_Result;
  type Feedback = crate::action::FollowWaypoints_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowWaypoints() }
  }
}

impl rosidl_runtime_rs::ActionImpl for FollowWaypoints {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::FollowWaypoints_FeedbackMessage;

  type SendGoalService = crate::action::rmw::FollowWaypoints_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::FollowWaypoints_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::FollowWaypoints_Goal,
  ) -> crate::action::rmw::FollowWaypoints_SendGoal_Request {
    crate::action::rmw::FollowWaypoints_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::FollowWaypoints_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::FollowWaypoints_SendGoal_Response {
    crate::action::rmw::FollowWaypoints_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::FollowWaypoints_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::FollowWaypoints_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::FollowWaypoints_Feedback,
  ) -> crate::action::rmw::FollowWaypoints_FeedbackMessage {
    let mut message = crate::action::rmw::FollowWaypoints_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::FollowWaypoints_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::FollowWaypoints_FeedbackMessage,
  ) -> &crate::action::rmw::FollowWaypoints_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::FollowWaypoints_GetResult_Request {
    crate::action::rmw::FollowWaypoints_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::FollowWaypoints_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::FollowWaypoints_Result,
  ) -> crate::action::rmw::FollowWaypoints_GetResult_Response {
    crate::action::rmw::FollowWaypoints_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::FollowWaypoints_GetResult_Response,
  ) -> &crate::action::rmw::FollowWaypoints_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::FollowWaypoints_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowGPSWaypoints() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowGPSWaypoints
pub struct FollowGPSWaypoints;

impl rosidl_runtime_rs::Action for FollowGPSWaypoints {
  type Goal = crate::action::FollowGPSWaypoints_Goal;
  type Result = crate::action::FollowGPSWaypoints_Result;
  type Feedback = crate::action::FollowGPSWaypoints_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowGPSWaypoints() }
  }
}

impl rosidl_runtime_rs::ActionImpl for FollowGPSWaypoints {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::FollowGPSWaypoints_FeedbackMessage;

  type SendGoalService = crate::action::rmw::FollowGPSWaypoints_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::FollowGPSWaypoints_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::FollowGPSWaypoints_Goal,
  ) -> crate::action::rmw::FollowGPSWaypoints_SendGoal_Request {
    crate::action::rmw::FollowGPSWaypoints_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::FollowGPSWaypoints_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::FollowGPSWaypoints_SendGoal_Response {
    crate::action::rmw::FollowGPSWaypoints_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::FollowGPSWaypoints_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::FollowGPSWaypoints_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::FollowGPSWaypoints_Feedback,
  ) -> crate::action::rmw::FollowGPSWaypoints_FeedbackMessage {
    let mut message = crate::action::rmw::FollowGPSWaypoints_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::FollowGPSWaypoints_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::FollowGPSWaypoints_FeedbackMessage,
  ) -> &crate::action::rmw::FollowGPSWaypoints_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::FollowGPSWaypoints_GetResult_Request {
    crate::action::rmw::FollowGPSWaypoints_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::FollowGPSWaypoints_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::FollowGPSWaypoints_Result,
  ) -> crate::action::rmw::FollowGPSWaypoints_GetResult_Response {
    crate::action::rmw::FollowGPSWaypoints_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::FollowGPSWaypoints_GetResult_Response,
  ) -> &crate::action::rmw::FollowGPSWaypoints_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::FollowGPSWaypoints_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DockRobot() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DockRobot
pub struct DockRobot;

impl rosidl_runtime_rs::Action for DockRobot {
  type Goal = crate::action::DockRobot_Goal;
  type Result = crate::action::DockRobot_Result;
  type Feedback = crate::action::DockRobot_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DockRobot() }
  }
}

impl rosidl_runtime_rs::ActionImpl for DockRobot {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::DockRobot_FeedbackMessage;

  type SendGoalService = crate::action::rmw::DockRobot_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::DockRobot_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::DockRobot_Goal,
  ) -> crate::action::rmw::DockRobot_SendGoal_Request {
    crate::action::rmw::DockRobot_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::DockRobot_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::DockRobot_SendGoal_Response {
    crate::action::rmw::DockRobot_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::DockRobot_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::DockRobot_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::DockRobot_Feedback,
  ) -> crate::action::rmw::DockRobot_FeedbackMessage {
    let mut message = crate::action::rmw::DockRobot_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::DockRobot_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::DockRobot_FeedbackMessage,
  ) -> &crate::action::rmw::DockRobot_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::DockRobot_GetResult_Request {
    crate::action::rmw::DockRobot_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::DockRobot_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::DockRobot_Result,
  ) -> crate::action::rmw::DockRobot_GetResult_Response {
    crate::action::rmw::DockRobot_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::DockRobot_GetResult_Response,
  ) -> &crate::action::rmw::DockRobot_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::DockRobot_GetResult_Response,
  ) -> i8 {
    response.status
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__UndockRobot() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__UndockRobot
pub struct UndockRobot;

impl rosidl_runtime_rs::Action for UndockRobot {
  type Goal = crate::action::UndockRobot_Goal;
  type Result = crate::action::UndockRobot_Result;
  type Feedback = crate::action::UndockRobot_Feedback;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__UndockRobot() }
  }
}

impl rosidl_runtime_rs::ActionImpl for UndockRobot {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::action::rmw::UndockRobot_FeedbackMessage;

  type SendGoalService = crate::action::rmw::UndockRobot_SendGoal;
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::action::rmw::UndockRobot_GetResult;

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: crate::action::rmw::UndockRobot_Goal,
  ) -> crate::action::rmw::UndockRobot_SendGoal_Request {
    crate::action::rmw::UndockRobot_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn get_goal_request_uuid(
    request: &crate::action::rmw::UndockRobot_SendGoal_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> crate::action::rmw::UndockRobot_SendGoal_Response {
    crate::action::rmw::UndockRobot_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &crate::action::rmw::UndockRobot_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &crate::action::rmw::UndockRobot_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: crate::action::rmw::UndockRobot_Feedback,
  ) -> crate::action::rmw::UndockRobot_FeedbackMessage {
    let mut message = crate::action::rmw::UndockRobot_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn get_feedback_message_uuid(
    feedback: &crate::action::rmw::UndockRobot_FeedbackMessage,
  ) -> &[u8; 16] {
    &feedback.goal_id.uuid
  }

  fn get_feedback_message_feedback(
    feedback: &crate::action::rmw::UndockRobot_FeedbackMessage,
  ) -> &crate::action::rmw::UndockRobot_Feedback {
    &feedback.feedback
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> crate::action::rmw::UndockRobot_GetResult_Request {
    crate::action::rmw::UndockRobot_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &crate::action::rmw::UndockRobot_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: crate::action::rmw::UndockRobot_Result,
  ) -> crate::action::rmw::UndockRobot_GetResult_Response {
    crate::action::rmw::UndockRobot_GetResult_Response {
      status,
      result,
    }
  }

  fn get_result_response_result(
    response: &crate::action::rmw::UndockRobot_GetResult_Response,
  ) -> &crate::action::rmw::UndockRobot_Result {
    &response.result
  }

  fn get_result_response_status(
    response: &crate::action::rmw::UndockRobot_GetResult_Response,
  ) -> i8 {
    response.status
  }
}


