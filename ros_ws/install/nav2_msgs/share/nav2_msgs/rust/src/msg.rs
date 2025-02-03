pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CollisionMonitorState() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__CollisionMonitorState__init(msg: *mut CollisionMonitorState) -> bool;
    fn nav2_msgs__msg__CollisionMonitorState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CollisionMonitorState>, size: usize) -> bool;
    fn nav2_msgs__msg__CollisionMonitorState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CollisionMonitorState>);
    fn nav2_msgs__msg__CollisionMonitorState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CollisionMonitorState>, out_seq: *mut rosidl_runtime_rs::Sequence<CollisionMonitorState>) -> bool;
}

// Corresponds to nav2_msgs__msg__CollisionMonitorState
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CollisionMonitorState {
    pub action_type: u8,
    pub polygon_name: rosidl_runtime_rs::String,
}

impl CollisionMonitorState {
    /// No action
    pub const DO_NOTHING: u8 = 0;
    /// Stop the robot
    pub const STOP: u8 = 1;
    /// Slowdown in percentage from current operating speed
    pub const SLOWDOWN: u8 = 2;
    /// Keep constant time interval before collision
    pub const APPROACH: u8 = 3;
    /// Sets a limit of velocities if pts in range
    pub const LIMIT: u8 = 4;
}


impl Default for CollisionMonitorState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__CollisionMonitorState__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__CollisionMonitorState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CollisionMonitorState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionMonitorState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionMonitorState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionMonitorState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CollisionMonitorState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CollisionMonitorState where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/CollisionMonitorState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CollisionMonitorState() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CollisionDetectorState() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__CollisionDetectorState__init(msg: *mut CollisionDetectorState) -> bool;
    fn nav2_msgs__msg__CollisionDetectorState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CollisionDetectorState>, size: usize) -> bool;
    fn nav2_msgs__msg__CollisionDetectorState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CollisionDetectorState>);
    fn nav2_msgs__msg__CollisionDetectorState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CollisionDetectorState>, out_seq: *mut rosidl_runtime_rs::Sequence<CollisionDetectorState>) -> bool;
}

// Corresponds to nav2_msgs__msg__CollisionDetectorState
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CollisionDetectorState {
    pub polygons: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
    pub detections: rosidl_runtime_rs::Sequence<bool>,
}



impl Default for CollisionDetectorState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__CollisionDetectorState__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__CollisionDetectorState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CollisionDetectorState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionDetectorState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionDetectorState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionDetectorState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CollisionDetectorState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CollisionDetectorState where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/CollisionDetectorState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CollisionDetectorState() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Costmap() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__Costmap__init(msg: *mut Costmap) -> bool;
    fn nav2_msgs__msg__Costmap__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Costmap>, size: usize) -> bool;
    fn nav2_msgs__msg__Costmap__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Costmap>);
    fn nav2_msgs__msg__Costmap__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Costmap>, out_seq: *mut rosidl_runtime_rs::Sequence<Costmap>) -> bool;
}

// Corresponds to nav2_msgs__msg__Costmap
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Costmap {
    pub header: std_msgs::msg::rmw::Header,
    pub metadata: crate::msg::rmw::CostmapMetaData,
    pub data: rosidl_runtime_rs::Sequence<u8>,
}



impl Default for Costmap {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__Costmap__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__Costmap__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Costmap {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Costmap__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Costmap__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Costmap__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Costmap {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Costmap where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/Costmap";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Costmap() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapMetaData() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__CostmapMetaData__init(msg: *mut CostmapMetaData) -> bool;
    fn nav2_msgs__msg__CostmapMetaData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CostmapMetaData>, size: usize) -> bool;
    fn nav2_msgs__msg__CostmapMetaData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CostmapMetaData>);
    fn nav2_msgs__msg__CostmapMetaData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CostmapMetaData>, out_seq: *mut rosidl_runtime_rs::Sequence<CostmapMetaData>) -> bool;
}

// Corresponds to nav2_msgs__msg__CostmapMetaData
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CostmapMetaData {
    pub map_load_time: builtin_interfaces::msg::rmw::Time,
    pub update_time: builtin_interfaces::msg::rmw::Time,
    pub layer: rosidl_runtime_rs::String,
    pub resolution: f32,
    pub size_x: u32,
    pub size_y: u32,
    pub origin: geometry_msgs::msg::rmw::Pose,
}



impl Default for CostmapMetaData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__CostmapMetaData__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__CostmapMetaData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CostmapMetaData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapMetaData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapMetaData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapMetaData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CostmapMetaData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CostmapMetaData where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/CostmapMetaData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapMetaData() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapUpdate() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__CostmapUpdate__init(msg: *mut CostmapUpdate) -> bool;
    fn nav2_msgs__msg__CostmapUpdate__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CostmapUpdate>, size: usize) -> bool;
    fn nav2_msgs__msg__CostmapUpdate__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CostmapUpdate>);
    fn nav2_msgs__msg__CostmapUpdate__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CostmapUpdate>, out_seq: *mut rosidl_runtime_rs::Sequence<CostmapUpdate>) -> bool;
}

// Corresponds to nav2_msgs__msg__CostmapUpdate
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CostmapUpdate {
    pub header: std_msgs::msg::rmw::Header,
    pub x: u32,
    pub y: u32,
    pub size_x: u32,
    pub size_y: u32,
    pub data: rosidl_runtime_rs::Sequence<u8>,
}



impl Default for CostmapUpdate {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__CostmapUpdate__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__CostmapUpdate__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CostmapUpdate {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapUpdate__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapUpdate__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapUpdate__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CostmapUpdate {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CostmapUpdate where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/CostmapUpdate";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapUpdate() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapFilterInfo() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__CostmapFilterInfo__init(msg: *mut CostmapFilterInfo) -> bool;
    fn nav2_msgs__msg__CostmapFilterInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CostmapFilterInfo>, size: usize) -> bool;
    fn nav2_msgs__msg__CostmapFilterInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CostmapFilterInfo>);
    fn nav2_msgs__msg__CostmapFilterInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CostmapFilterInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<CostmapFilterInfo>) -> bool;
}

// Corresponds to nav2_msgs__msg__CostmapFilterInfo
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CostmapFilterInfo {
    pub header: std_msgs::msg::rmw::Header,
    pub type_: u8,
    pub filter_mask_topic: rosidl_runtime_rs::String,
    pub base: f32,
    pub multiplier: f32,
}



impl Default for CostmapFilterInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__CostmapFilterInfo__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__CostmapFilterInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CostmapFilterInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapFilterInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapFilterInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapFilterInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CostmapFilterInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CostmapFilterInfo where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/CostmapFilterInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapFilterInfo() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__SpeedLimit() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__SpeedLimit__init(msg: *mut SpeedLimit) -> bool;
    fn nav2_msgs__msg__SpeedLimit__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SpeedLimit>, size: usize) -> bool;
    fn nav2_msgs__msg__SpeedLimit__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SpeedLimit>);
    fn nav2_msgs__msg__SpeedLimit__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SpeedLimit>, out_seq: *mut rosidl_runtime_rs::Sequence<SpeedLimit>) -> bool;
}

// Corresponds to nav2_msgs__msg__SpeedLimit
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SpeedLimit {
    pub header: std_msgs::msg::rmw::Header,
    pub percentage: bool,
    pub speed_limit: f64,
}



impl Default for SpeedLimit {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__SpeedLimit__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__SpeedLimit__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SpeedLimit {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__SpeedLimit__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__SpeedLimit__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__SpeedLimit__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SpeedLimit {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SpeedLimit where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/SpeedLimit";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__SpeedLimit() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__VoxelGrid() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__VoxelGrid__init(msg: *mut VoxelGrid) -> bool;
    fn nav2_msgs__msg__VoxelGrid__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VoxelGrid>, size: usize) -> bool;
    fn nav2_msgs__msg__VoxelGrid__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VoxelGrid>);
    fn nav2_msgs__msg__VoxelGrid__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VoxelGrid>, out_seq: *mut rosidl_runtime_rs::Sequence<VoxelGrid>) -> bool;
}

// Corresponds to nav2_msgs__msg__VoxelGrid
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VoxelGrid {
    pub header: std_msgs::msg::rmw::Header,
    pub data: rosidl_runtime_rs::Sequence<u32>,
    pub origin: geometry_msgs::msg::rmw::Point32,
    pub resolutions: geometry_msgs::msg::rmw::Vector3,
    pub size_x: u32,
    pub size_y: u32,
    pub size_z: u32,
}



impl Default for VoxelGrid {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__VoxelGrid__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__VoxelGrid__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VoxelGrid {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__VoxelGrid__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__VoxelGrid__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__VoxelGrid__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VoxelGrid {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VoxelGrid where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/VoxelGrid";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__VoxelGrid() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeStatusChange() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__BehaviorTreeStatusChange__init(msg: *mut BehaviorTreeStatusChange) -> bool;
    fn nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeStatusChange>, size: usize) -> bool;
    fn nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeStatusChange>);
    fn nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BehaviorTreeStatusChange>, out_seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeStatusChange>) -> bool;
}

// Corresponds to nav2_msgs__msg__BehaviorTreeStatusChange
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BehaviorTreeStatusChange {
    pub timestamp: builtin_interfaces::msg::rmw::Time,
    pub node_name: rosidl_runtime_rs::String,
    pub uid: u16,
    pub previous_status: rosidl_runtime_rs::String,
    pub current_status: rosidl_runtime_rs::String,
}



impl Default for BehaviorTreeStatusChange {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__BehaviorTreeStatusChange__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__BehaviorTreeStatusChange__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BehaviorTreeStatusChange {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BehaviorTreeStatusChange {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BehaviorTreeStatusChange where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/BehaviorTreeStatusChange";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeStatusChange() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeLog() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__BehaviorTreeLog__init(msg: *mut BehaviorTreeLog) -> bool;
    fn nav2_msgs__msg__BehaviorTreeLog__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeLog>, size: usize) -> bool;
    fn nav2_msgs__msg__BehaviorTreeLog__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeLog>);
    fn nav2_msgs__msg__BehaviorTreeLog__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BehaviorTreeLog>, out_seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeLog>) -> bool;
}

// Corresponds to nav2_msgs__msg__BehaviorTreeLog
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BehaviorTreeLog {
    pub timestamp: builtin_interfaces::msg::rmw::Time,
    pub event_log: rosidl_runtime_rs::Sequence<crate::msg::rmw::BehaviorTreeStatusChange>,
}



impl Default for BehaviorTreeLog {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__BehaviorTreeLog__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__BehaviorTreeLog__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BehaviorTreeLog {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeLog__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeLog__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeLog__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BehaviorTreeLog {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BehaviorTreeLog where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/BehaviorTreeLog";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeLog() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Particle() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__Particle__init(msg: *mut Particle) -> bool;
    fn nav2_msgs__msg__Particle__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Particle>, size: usize) -> bool;
    fn nav2_msgs__msg__Particle__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Particle>);
    fn nav2_msgs__msg__Particle__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Particle>, out_seq: *mut rosidl_runtime_rs::Sequence<Particle>) -> bool;
}

// Corresponds to nav2_msgs__msg__Particle
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Particle {
    pub pose: geometry_msgs::msg::rmw::Pose,
    pub weight: f64,
}



impl Default for Particle {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__Particle__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__Particle__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Particle {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Particle__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Particle__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Particle__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Particle {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Particle where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/Particle";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Particle() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__ParticleCloud() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__ParticleCloud__init(msg: *mut ParticleCloud) -> bool;
    fn nav2_msgs__msg__ParticleCloud__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ParticleCloud>, size: usize) -> bool;
    fn nav2_msgs__msg__ParticleCloud__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ParticleCloud>);
    fn nav2_msgs__msg__ParticleCloud__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ParticleCloud>, out_seq: *mut rosidl_runtime_rs::Sequence<ParticleCloud>) -> bool;
}

// Corresponds to nav2_msgs__msg__ParticleCloud
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParticleCloud {
    pub header: std_msgs::msg::rmw::Header,
    pub particles: rosidl_runtime_rs::Sequence<crate::msg::rmw::Particle>,
}



impl Default for ParticleCloud {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__ParticleCloud__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__ParticleCloud__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ParticleCloud {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__ParticleCloud__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__ParticleCloud__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__ParticleCloud__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ParticleCloud {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ParticleCloud where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/ParticleCloud";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__ParticleCloud() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__MissedWaypoint() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__MissedWaypoint__init(msg: *mut MissedWaypoint) -> bool;
    fn nav2_msgs__msg__MissedWaypoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MissedWaypoint>, size: usize) -> bool;
    fn nav2_msgs__msg__MissedWaypoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MissedWaypoint>);
    fn nav2_msgs__msg__MissedWaypoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MissedWaypoint>, out_seq: *mut rosidl_runtime_rs::Sequence<MissedWaypoint>) -> bool;
}

// Corresponds to nav2_msgs__msg__MissedWaypoint
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MissedWaypoint {
    pub index: u32,
    pub goal: geometry_msgs::msg::rmw::PoseStamped,
    pub error_code: u16,
}



impl Default for MissedWaypoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__MissedWaypoint__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__MissedWaypoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MissedWaypoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__MissedWaypoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__MissedWaypoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__MissedWaypoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MissedWaypoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MissedWaypoint where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/MissedWaypoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__MissedWaypoint() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CollisionMonitorState {
    pub action_type: u8,
    pub polygon_name: std::string::String,
}

impl CollisionMonitorState {
    /// No action
    pub const DO_NOTHING: u8 = 0;
    /// Stop the robot
    pub const STOP: u8 = 1;
    /// Slowdown in percentage from current operating speed
    pub const SLOWDOWN: u8 = 2;
    /// Keep constant time interval before collision
    pub const APPROACH: u8 = 3;
    /// Sets a limit of velocities if pts in range
    pub const LIMIT: u8 = 4;
}


impl Default for CollisionMonitorState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::CollisionMonitorState::default())
  }
}

impl rosidl_runtime_rs::Message for CollisionMonitorState {
  type RmwMsg = crate::msg::rmw::CollisionMonitorState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        action_type: msg.action_type,
        polygon_name: msg.polygon_name.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      action_type: msg.action_type,
        polygon_name: msg.polygon_name.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      action_type: msg.action_type,
      polygon_name: msg.polygon_name.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CollisionDetectorState {
    pub polygons: Vec<std::string::String>,
    pub detections: Vec<bool>,
}



impl Default for CollisionDetectorState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::CollisionDetectorState::default())
  }
}

impl rosidl_runtime_rs::Message for CollisionDetectorState {
  type RmwMsg = crate::msg::rmw::CollisionDetectorState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        polygons: msg.polygons
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        detections: msg.detections.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        polygons: msg.polygons
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        detections: msg.detections.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      polygons: msg.polygons
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      detections: msg.detections
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Costmap {
    pub header: std_msgs::msg::Header,
    pub metadata: crate::msg::CostmapMetaData,
    pub data: Vec<u8>,
}



impl Default for Costmap {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Costmap::default())
  }
}

impl rosidl_runtime_rs::Message for Costmap {
  type RmwMsg = crate::msg::rmw::Costmap;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        metadata: crate::msg::CostmapMetaData::into_rmw_message(std::borrow::Cow::Owned(msg.metadata)).into_owned(),
        data: msg.data.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        metadata: crate::msg::CostmapMetaData::into_rmw_message(std::borrow::Cow::Borrowed(&msg.metadata)).into_owned(),
        data: msg.data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      metadata: crate::msg::CostmapMetaData::from_rmw_message(msg.metadata),
      data: msg.data
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CostmapMetaData {
    pub map_load_time: builtin_interfaces::msg::Time,
    pub update_time: builtin_interfaces::msg::Time,
    pub layer: std::string::String,
    pub resolution: f32,
    pub size_x: u32,
    pub size_y: u32,
    pub origin: geometry_msgs::msg::Pose,
}



impl Default for CostmapMetaData {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::CostmapMetaData::default())
  }
}

impl rosidl_runtime_rs::Message for CostmapMetaData {
  type RmwMsg = crate::msg::rmw::CostmapMetaData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_load_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.map_load_time)).into_owned(),
        update_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.update_time)).into_owned(),
        layer: msg.layer.as_str().into(),
        resolution: msg.resolution,
        size_x: msg.size_x,
        size_y: msg.size_y,
        origin: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.origin)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_load_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map_load_time)).into_owned(),
        update_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.update_time)).into_owned(),
        layer: msg.layer.as_str().into(),
      resolution: msg.resolution,
      size_x: msg.size_x,
      size_y: msg.size_y,
        origin: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.origin)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_load_time: builtin_interfaces::msg::Time::from_rmw_message(msg.map_load_time),
      update_time: builtin_interfaces::msg::Time::from_rmw_message(msg.update_time),
      layer: msg.layer.to_string(),
      resolution: msg.resolution,
      size_x: msg.size_x,
      size_y: msg.size_y,
      origin: geometry_msgs::msg::Pose::from_rmw_message(msg.origin),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CostmapUpdate {
    pub header: std_msgs::msg::Header,
    pub x: u32,
    pub y: u32,
    pub size_x: u32,
    pub size_y: u32,
    pub data: Vec<u8>,
}



impl Default for CostmapUpdate {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::CostmapUpdate::default())
  }
}

impl rosidl_runtime_rs::Message for CostmapUpdate {
  type RmwMsg = crate::msg::rmw::CostmapUpdate;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        x: msg.x,
        y: msg.y,
        size_x: msg.size_x,
        size_y: msg.size_y,
        data: msg.data.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      x: msg.x,
      y: msg.y,
      size_x: msg.size_x,
      size_y: msg.size_y,
        data: msg.data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      x: msg.x,
      y: msg.y,
      size_x: msg.size_x,
      size_y: msg.size_y,
      data: msg.data
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CostmapFilterInfo {
    pub header: std_msgs::msg::Header,
    pub type_: u8,
    pub filter_mask_topic: std::string::String,
    pub base: f32,
    pub multiplier: f32,
}



impl Default for CostmapFilterInfo {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::CostmapFilterInfo::default())
  }
}

impl rosidl_runtime_rs::Message for CostmapFilterInfo {
  type RmwMsg = crate::msg::rmw::CostmapFilterInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        type_: msg.type_,
        filter_mask_topic: msg.filter_mask_topic.as_str().into(),
        base: msg.base,
        multiplier: msg.multiplier,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      type_: msg.type_,
        filter_mask_topic: msg.filter_mask_topic.as_str().into(),
      base: msg.base,
      multiplier: msg.multiplier,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      type_: msg.type_,
      filter_mask_topic: msg.filter_mask_topic.to_string(),
      base: msg.base,
      multiplier: msg.multiplier,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SpeedLimit {
    pub header: std_msgs::msg::Header,
    pub percentage: bool,
    pub speed_limit: f64,
}



impl Default for SpeedLimit {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::SpeedLimit::default())
  }
}

impl rosidl_runtime_rs::Message for SpeedLimit {
  type RmwMsg = crate::msg::rmw::SpeedLimit;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        percentage: msg.percentage,
        speed_limit: msg.speed_limit,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      percentage: msg.percentage,
      speed_limit: msg.speed_limit,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      percentage: msg.percentage,
      speed_limit: msg.speed_limit,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VoxelGrid {
    pub header: std_msgs::msg::Header,
    pub data: Vec<u32>,
    pub origin: geometry_msgs::msg::Point32,
    pub resolutions: geometry_msgs::msg::Vector3,
    pub size_x: u32,
    pub size_y: u32,
    pub size_z: u32,
}



impl Default for VoxelGrid {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::VoxelGrid::default())
  }
}

impl rosidl_runtime_rs::Message for VoxelGrid {
  type RmwMsg = crate::msg::rmw::VoxelGrid;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        data: msg.data.into(),
        origin: geometry_msgs::msg::Point32::into_rmw_message(std::borrow::Cow::Owned(msg.origin)).into_owned(),
        resolutions: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.resolutions)).into_owned(),
        size_x: msg.size_x,
        size_y: msg.size_y,
        size_z: msg.size_z,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        data: msg.data.as_slice().into(),
        origin: geometry_msgs::msg::Point32::into_rmw_message(std::borrow::Cow::Borrowed(&msg.origin)).into_owned(),
        resolutions: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.resolutions)).into_owned(),
      size_x: msg.size_x,
      size_y: msg.size_y,
      size_z: msg.size_z,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      data: msg.data
          .into_iter()
          .collect(),
      origin: geometry_msgs::msg::Point32::from_rmw_message(msg.origin),
      resolutions: geometry_msgs::msg::Vector3::from_rmw_message(msg.resolutions),
      size_x: msg.size_x,
      size_y: msg.size_y,
      size_z: msg.size_z,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BehaviorTreeStatusChange {
    pub timestamp: builtin_interfaces::msg::Time,
    pub node_name: std::string::String,
    pub uid: u16,
    pub previous_status: std::string::String,
    pub current_status: std::string::String,
}



impl Default for BehaviorTreeStatusChange {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::BehaviorTreeStatusChange::default())
  }
}

impl rosidl_runtime_rs::Message for BehaviorTreeStatusChange {
  type RmwMsg = crate::msg::rmw::BehaviorTreeStatusChange;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        timestamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.timestamp)).into_owned(),
        node_name: msg.node_name.as_str().into(),
        uid: msg.uid,
        previous_status: msg.previous_status.as_str().into(),
        current_status: msg.current_status.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        timestamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.timestamp)).into_owned(),
        node_name: msg.node_name.as_str().into(),
      uid: msg.uid,
        previous_status: msg.previous_status.as_str().into(),
        current_status: msg.current_status.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      timestamp: builtin_interfaces::msg::Time::from_rmw_message(msg.timestamp),
      node_name: msg.node_name.to_string(),
      uid: msg.uid,
      previous_status: msg.previous_status.to_string(),
      current_status: msg.current_status.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BehaviorTreeLog {
    pub timestamp: builtin_interfaces::msg::Time,
    pub event_log: Vec<crate::msg::BehaviorTreeStatusChange>,
}



impl Default for BehaviorTreeLog {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::BehaviorTreeLog::default())
  }
}

impl rosidl_runtime_rs::Message for BehaviorTreeLog {
  type RmwMsg = crate::msg::rmw::BehaviorTreeLog;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        timestamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.timestamp)).into_owned(),
        event_log: msg.event_log
          .into_iter()
          .map(|elem| crate::msg::BehaviorTreeStatusChange::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        timestamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.timestamp)).into_owned(),
        event_log: msg.event_log
          .iter()
          .map(|elem| crate::msg::BehaviorTreeStatusChange::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      timestamp: builtin_interfaces::msg::Time::from_rmw_message(msg.timestamp),
      event_log: msg.event_log
          .into_iter()
          .map(crate::msg::BehaviorTreeStatusChange::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Particle {
    pub pose: geometry_msgs::msg::Pose,
    pub weight: f64,
}



impl Default for Particle {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Particle::default())
  }
}

impl rosidl_runtime_rs::Message for Particle {
  type RmwMsg = crate::msg::rmw::Particle;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        weight: msg.weight,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      weight: msg.weight,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      weight: msg.weight,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParticleCloud {
    pub header: std_msgs::msg::Header,
    pub particles: Vec<crate::msg::Particle>,
}



impl Default for ParticleCloud {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::ParticleCloud::default())
  }
}

impl rosidl_runtime_rs::Message for ParticleCloud {
  type RmwMsg = crate::msg::rmw::ParticleCloud;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        particles: msg.particles
          .into_iter()
          .map(|elem| crate::msg::Particle::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        particles: msg.particles
          .iter()
          .map(|elem| crate::msg::Particle::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      particles: msg.particles
          .into_iter()
          .map(crate::msg::Particle::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MissedWaypoint {
    pub index: u32,
    pub goal: geometry_msgs::msg::PoseStamped,
    pub error_code: u16,
}



impl Default for MissedWaypoint {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MissedWaypoint::default())
  }
}

impl rosidl_runtime_rs::Message for MissedWaypoint {
  type RmwMsg = crate::msg::rmw::MissedWaypoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        index: msg.index,
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        error_code: msg.error_code,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      index: msg.index,
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      error_code: msg.error_code,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      index: msg.index,
      goal: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.goal),
      error_code: msg.error_code,
    }
  }
}


