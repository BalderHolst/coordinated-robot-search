pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoint() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__GeoPoint__init(msg: *mut GeoPoint) -> bool;
    fn geographic_msgs__msg__GeoPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GeoPoint>, size: usize) -> bool;
    fn geographic_msgs__msg__GeoPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GeoPoint>);
    fn geographic_msgs__msg__GeoPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GeoPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<GeoPoint>) -> bool;
}

// Corresponds to geographic_msgs__msg__GeoPoint
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GeoPoint {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}



impl Default for GeoPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__GeoPoint__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__GeoPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GeoPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeoPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeoPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeoPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GeoPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GeoPoint where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/GeoPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoint() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__BoundingBox() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__BoundingBox__init(msg: *mut BoundingBox) -> bool;
    fn geographic_msgs__msg__BoundingBox__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BoundingBox>, size: usize) -> bool;
    fn geographic_msgs__msg__BoundingBox__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BoundingBox>);
    fn geographic_msgs__msg__BoundingBox__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BoundingBox>, out_seq: *mut rosidl_runtime_rs::Sequence<BoundingBox>) -> bool;
}

// Corresponds to geographic_msgs__msg__BoundingBox
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BoundingBox {
    pub min_pt: crate::msg::rmw::GeoPoint,
    pub max_pt: crate::msg::rmw::GeoPoint,
}



impl Default for BoundingBox {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__BoundingBox__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__BoundingBox__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BoundingBox {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__BoundingBox__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__BoundingBox__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__BoundingBox__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BoundingBox {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BoundingBox where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/BoundingBox";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__BoundingBox() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMapChanges() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__GeographicMapChanges__init(msg: *mut GeographicMapChanges) -> bool;
    fn geographic_msgs__msg__GeographicMapChanges__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GeographicMapChanges>, size: usize) -> bool;
    fn geographic_msgs__msg__GeographicMapChanges__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GeographicMapChanges>);
    fn geographic_msgs__msg__GeographicMapChanges__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GeographicMapChanges>, out_seq: *mut rosidl_runtime_rs::Sequence<GeographicMapChanges>) -> bool;
}

// Corresponds to geographic_msgs__msg__GeographicMapChanges
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GeographicMapChanges {
    pub header: std_msgs::msg::rmw::Header,
    pub diffs: crate::msg::rmw::GeographicMap,
    pub deletes: rosidl_runtime_rs::Sequence<unique_identifier_msgs::msg::rmw::UUID>,
}



impl Default for GeographicMapChanges {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__GeographicMapChanges__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__GeographicMapChanges__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GeographicMapChanges {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeographicMapChanges__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeographicMapChanges__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeographicMapChanges__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GeographicMapChanges {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GeographicMapChanges where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/GeographicMapChanges";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMapChanges() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMap() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__GeographicMap__init(msg: *mut GeographicMap) -> bool;
    fn geographic_msgs__msg__GeographicMap__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GeographicMap>, size: usize) -> bool;
    fn geographic_msgs__msg__GeographicMap__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GeographicMap>);
    fn geographic_msgs__msg__GeographicMap__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GeographicMap>, out_seq: *mut rosidl_runtime_rs::Sequence<GeographicMap>) -> bool;
}

// Corresponds to geographic_msgs__msg__GeographicMap
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GeographicMap {
    pub header: std_msgs::msg::rmw::Header,
    pub id: unique_identifier_msgs::msg::rmw::UUID,
    pub bounds: crate::msg::rmw::BoundingBox,
    pub points: rosidl_runtime_rs::Sequence<crate::msg::rmw::WayPoint>,
    pub features: rosidl_runtime_rs::Sequence<crate::msg::rmw::MapFeature>,
    pub props: rosidl_runtime_rs::Sequence<crate::msg::rmw::KeyValue>,
}



impl Default for GeographicMap {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__GeographicMap__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__GeographicMap__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GeographicMap {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeographicMap__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeographicMap__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeographicMap__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GeographicMap {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GeographicMap where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/GeographicMap";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMap() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPose() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__GeoPose__init(msg: *mut GeoPose) -> bool;
    fn geographic_msgs__msg__GeoPose__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GeoPose>, size: usize) -> bool;
    fn geographic_msgs__msg__GeoPose__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GeoPose>);
    fn geographic_msgs__msg__GeoPose__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GeoPose>, out_seq: *mut rosidl_runtime_rs::Sequence<GeoPose>) -> bool;
}

// Corresponds to geographic_msgs__msg__GeoPose
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GeoPose {
    pub position: crate::msg::rmw::GeoPoint,
    pub orientation: geometry_msgs::msg::rmw::Quaternion,
}



impl Default for GeoPose {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__GeoPose__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__GeoPose__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GeoPose {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeoPose__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeoPose__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__GeoPose__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GeoPose {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GeoPose where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/GeoPose";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPose() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__KeyValue() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__KeyValue__init(msg: *mut KeyValue) -> bool;
    fn geographic_msgs__msg__KeyValue__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<KeyValue>, size: usize) -> bool;
    fn geographic_msgs__msg__KeyValue__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<KeyValue>);
    fn geographic_msgs__msg__KeyValue__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<KeyValue>, out_seq: *mut rosidl_runtime_rs::Sequence<KeyValue>) -> bool;
}

// Corresponds to geographic_msgs__msg__KeyValue
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyValue {
    pub key: rosidl_runtime_rs::String,
    pub value: rosidl_runtime_rs::String,
}



impl Default for KeyValue {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__KeyValue__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__KeyValue__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for KeyValue {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__KeyValue__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__KeyValue__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__KeyValue__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for KeyValue {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for KeyValue where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/KeyValue";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__KeyValue() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__MapFeature() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__MapFeature__init(msg: *mut MapFeature) -> bool;
    fn geographic_msgs__msg__MapFeature__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MapFeature>, size: usize) -> bool;
    fn geographic_msgs__msg__MapFeature__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MapFeature>);
    fn geographic_msgs__msg__MapFeature__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MapFeature>, out_seq: *mut rosidl_runtime_rs::Sequence<MapFeature>) -> bool;
}

// Corresponds to geographic_msgs__msg__MapFeature
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MapFeature {
    pub id: unique_identifier_msgs::msg::rmw::UUID,
    pub components: rosidl_runtime_rs::Sequence<unique_identifier_msgs::msg::rmw::UUID>,
    pub props: rosidl_runtime_rs::Sequence<crate::msg::rmw::KeyValue>,
}



impl Default for MapFeature {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__MapFeature__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__MapFeature__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MapFeature {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__MapFeature__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__MapFeature__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__MapFeature__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MapFeature {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MapFeature where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/MapFeature";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__MapFeature() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteNetwork() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__RouteNetwork__init(msg: *mut RouteNetwork) -> bool;
    fn geographic_msgs__msg__RouteNetwork__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteNetwork>, size: usize) -> bool;
    fn geographic_msgs__msg__RouteNetwork__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteNetwork>);
    fn geographic_msgs__msg__RouteNetwork__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteNetwork>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteNetwork>) -> bool;
}

// Corresponds to geographic_msgs__msg__RouteNetwork
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteNetwork {
    pub header: std_msgs::msg::rmw::Header,
    pub id: unique_identifier_msgs::msg::rmw::UUID,
    pub bounds: crate::msg::rmw::BoundingBox,
    pub points: rosidl_runtime_rs::Sequence<crate::msg::rmw::WayPoint>,
    pub segments: rosidl_runtime_rs::Sequence<crate::msg::rmw::RouteSegment>,
    pub props: rosidl_runtime_rs::Sequence<crate::msg::rmw::KeyValue>,
}



impl Default for RouteNetwork {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__RouteNetwork__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__RouteNetwork__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteNetwork {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RouteNetwork__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RouteNetwork__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RouteNetwork__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteNetwork {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteNetwork where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/RouteNetwork";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteNetwork() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RoutePath() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__RoutePath__init(msg: *mut RoutePath) -> bool;
    fn geographic_msgs__msg__RoutePath__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RoutePath>, size: usize) -> bool;
    fn geographic_msgs__msg__RoutePath__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RoutePath>);
    fn geographic_msgs__msg__RoutePath__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RoutePath>, out_seq: *mut rosidl_runtime_rs::Sequence<RoutePath>) -> bool;
}

// Corresponds to geographic_msgs__msg__RoutePath
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RoutePath {
    pub header: std_msgs::msg::rmw::Header,
    pub network: unique_identifier_msgs::msg::rmw::UUID,
    pub segments: rosidl_runtime_rs::Sequence<unique_identifier_msgs::msg::rmw::UUID>,
    pub props: rosidl_runtime_rs::Sequence<crate::msg::rmw::KeyValue>,
}



impl Default for RoutePath {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__RoutePath__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__RoutePath__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RoutePath {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RoutePath__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RoutePath__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RoutePath__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RoutePath {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RoutePath where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/RoutePath";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RoutePath() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteSegment() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__RouteSegment__init(msg: *mut RouteSegment) -> bool;
    fn geographic_msgs__msg__RouteSegment__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteSegment>, size: usize) -> bool;
    fn geographic_msgs__msg__RouteSegment__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteSegment>);
    fn geographic_msgs__msg__RouteSegment__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteSegment>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteSegment>) -> bool;
}

// Corresponds to geographic_msgs__msg__RouteSegment
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteSegment {
    pub id: unique_identifier_msgs::msg::rmw::UUID,
    pub start: unique_identifier_msgs::msg::rmw::UUID,
    pub end: unique_identifier_msgs::msg::rmw::UUID,
    pub props: rosidl_runtime_rs::Sequence<crate::msg::rmw::KeyValue>,
}



impl Default for RouteSegment {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__RouteSegment__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__RouteSegment__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteSegment {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RouteSegment__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RouteSegment__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__RouteSegment__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteSegment {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteSegment where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/RouteSegment";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteSegment() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__WayPoint() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__msg__WayPoint__init(msg: *mut WayPoint) -> bool;
    fn geographic_msgs__msg__WayPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<WayPoint>, size: usize) -> bool;
    fn geographic_msgs__msg__WayPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<WayPoint>);
    fn geographic_msgs__msg__WayPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<WayPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<WayPoint>) -> bool;
}

// Corresponds to geographic_msgs__msg__WayPoint
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WayPoint {
    pub id: unique_identifier_msgs::msg::rmw::UUID,
    pub position: crate::msg::rmw::GeoPoint,
    pub props: rosidl_runtime_rs::Sequence<crate::msg::rmw::KeyValue>,
}



impl Default for WayPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__msg__WayPoint__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__msg__WayPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for WayPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__WayPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__WayPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__msg__WayPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for WayPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for WayPoint where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/msg/WayPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__WayPoint() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GeoPoint {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}



impl Default for GeoPoint {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::GeoPoint::default())
  }
}

impl rosidl_runtime_rs::Message for GeoPoint {
  type RmwMsg = crate::msg::rmw::GeoPoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        latitude: msg.latitude,
        longitude: msg.longitude,
        altitude: msg.altitude,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      latitude: msg.latitude,
      longitude: msg.longitude,
      altitude: msg.altitude,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      latitude: msg.latitude,
      longitude: msg.longitude,
      altitude: msg.altitude,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BoundingBox {
    pub min_pt: crate::msg::GeoPoint,
    pub max_pt: crate::msg::GeoPoint,
}



impl Default for BoundingBox {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::BoundingBox::default())
  }
}

impl rosidl_runtime_rs::Message for BoundingBox {
  type RmwMsg = crate::msg::rmw::BoundingBox;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        min_pt: crate::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Owned(msg.min_pt)).into_owned(),
        max_pt: crate::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Owned(msg.max_pt)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        min_pt: crate::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Borrowed(&msg.min_pt)).into_owned(),
        max_pt: crate::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Borrowed(&msg.max_pt)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      min_pt: crate::msg::GeoPoint::from_rmw_message(msg.min_pt),
      max_pt: crate::msg::GeoPoint::from_rmw_message(msg.max_pt),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GeographicMapChanges {
    pub header: std_msgs::msg::Header,
    pub diffs: crate::msg::GeographicMap,
    pub deletes: Vec<unique_identifier_msgs::msg::UUID>,
}



impl Default for GeographicMapChanges {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::GeographicMapChanges::default())
  }
}

impl rosidl_runtime_rs::Message for GeographicMapChanges {
  type RmwMsg = crate::msg::rmw::GeographicMapChanges;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        diffs: crate::msg::GeographicMap::into_rmw_message(std::borrow::Cow::Owned(msg.diffs)).into_owned(),
        deletes: msg.deletes
          .into_iter()
          .map(|elem| unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        diffs: crate::msg::GeographicMap::into_rmw_message(std::borrow::Cow::Borrowed(&msg.diffs)).into_owned(),
        deletes: msg.deletes
          .iter()
          .map(|elem| unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      diffs: crate::msg::GeographicMap::from_rmw_message(msg.diffs),
      deletes: msg.deletes
          .into_iter()
          .map(unique_identifier_msgs::msg::UUID::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GeographicMap {
    pub header: std_msgs::msg::Header,
    pub id: unique_identifier_msgs::msg::UUID,
    pub bounds: crate::msg::BoundingBox,
    pub points: Vec<crate::msg::WayPoint>,
    pub features: Vec<crate::msg::MapFeature>,
    pub props: Vec<crate::msg::KeyValue>,
}



impl Default for GeographicMap {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::GeographicMap::default())
  }
}

impl rosidl_runtime_rs::Message for GeographicMap {
  type RmwMsg = crate::msg::rmw::GeographicMap;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.id)).into_owned(),
        bounds: crate::msg::BoundingBox::into_rmw_message(std::borrow::Cow::Owned(msg.bounds)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| crate::msg::WayPoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        features: msg.features
          .into_iter()
          .map(|elem| crate::msg::MapFeature::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        props: msg.props
          .into_iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.id)).into_owned(),
        bounds: crate::msg::BoundingBox::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bounds)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| crate::msg::WayPoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        features: msg.features
          .iter()
          .map(|elem| crate::msg::MapFeature::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        props: msg.props
          .iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.id),
      bounds: crate::msg::BoundingBox::from_rmw_message(msg.bounds),
      points: msg.points
          .into_iter()
          .map(crate::msg::WayPoint::from_rmw_message)
          .collect(),
      features: msg.features
          .into_iter()
          .map(crate::msg::MapFeature::from_rmw_message)
          .collect(),
      props: msg.props
          .into_iter()
          .map(crate::msg::KeyValue::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GeoPose {
    pub position: crate::msg::GeoPoint,
    pub orientation: geometry_msgs::msg::Quaternion,
}



impl Default for GeoPose {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::GeoPose::default())
  }
}

impl rosidl_runtime_rs::Message for GeoPose {
  type RmwMsg = crate::msg::rmw::GeoPose;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: crate::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Owned(msg.orientation)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: crate::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Borrowed(&msg.orientation)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      position: crate::msg::GeoPoint::from_rmw_message(msg.position),
      orientation: geometry_msgs::msg::Quaternion::from_rmw_message(msg.orientation),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyValue {
    pub key: std::string::String,
    pub value: std::string::String,
}



impl Default for KeyValue {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::KeyValue::default())
  }
}

impl rosidl_runtime_rs::Message for KeyValue {
  type RmwMsg = crate::msg::rmw::KeyValue;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        key: msg.key.as_str().into(),
        value: msg.value.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        key: msg.key.as_str().into(),
        value: msg.value.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      key: msg.key.to_string(),
      value: msg.value.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MapFeature {
    pub id: unique_identifier_msgs::msg::UUID,
    pub components: Vec<unique_identifier_msgs::msg::UUID>,
    pub props: Vec<crate::msg::KeyValue>,
}



impl Default for MapFeature {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MapFeature::default())
  }
}

impl rosidl_runtime_rs::Message for MapFeature {
  type RmwMsg = crate::msg::rmw::MapFeature;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.id)).into_owned(),
        components: msg.components
          .into_iter()
          .map(|elem| unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        props: msg.props
          .into_iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.id)).into_owned(),
        components: msg.components
          .iter()
          .map(|elem| unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        props: msg.props
          .iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.id),
      components: msg.components
          .into_iter()
          .map(unique_identifier_msgs::msg::UUID::from_rmw_message)
          .collect(),
      props: msg.props
          .into_iter()
          .map(crate::msg::KeyValue::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteNetwork {
    pub header: std_msgs::msg::Header,
    pub id: unique_identifier_msgs::msg::UUID,
    pub bounds: crate::msg::BoundingBox,
    pub points: Vec<crate::msg::WayPoint>,
    pub segments: Vec<crate::msg::RouteSegment>,
    pub props: Vec<crate::msg::KeyValue>,
}



impl Default for RouteNetwork {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::RouteNetwork::default())
  }
}

impl rosidl_runtime_rs::Message for RouteNetwork {
  type RmwMsg = crate::msg::rmw::RouteNetwork;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.id)).into_owned(),
        bounds: crate::msg::BoundingBox::into_rmw_message(std::borrow::Cow::Owned(msg.bounds)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| crate::msg::WayPoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        segments: msg.segments
          .into_iter()
          .map(|elem| crate::msg::RouteSegment::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        props: msg.props
          .into_iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.id)).into_owned(),
        bounds: crate::msg::BoundingBox::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bounds)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| crate::msg::WayPoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        segments: msg.segments
          .iter()
          .map(|elem| crate::msg::RouteSegment::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        props: msg.props
          .iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.id),
      bounds: crate::msg::BoundingBox::from_rmw_message(msg.bounds),
      points: msg.points
          .into_iter()
          .map(crate::msg::WayPoint::from_rmw_message)
          .collect(),
      segments: msg.segments
          .into_iter()
          .map(crate::msg::RouteSegment::from_rmw_message)
          .collect(),
      props: msg.props
          .into_iter()
          .map(crate::msg::KeyValue::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RoutePath {
    pub header: std_msgs::msg::Header,
    pub network: unique_identifier_msgs::msg::UUID,
    pub segments: Vec<unique_identifier_msgs::msg::UUID>,
    pub props: Vec<crate::msg::KeyValue>,
}



impl Default for RoutePath {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::RoutePath::default())
  }
}

impl rosidl_runtime_rs::Message for RoutePath {
  type RmwMsg = crate::msg::rmw::RoutePath;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        network: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.network)).into_owned(),
        segments: msg.segments
          .into_iter()
          .map(|elem| unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        props: msg.props
          .into_iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        network: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.network)).into_owned(),
        segments: msg.segments
          .iter()
          .map(|elem| unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        props: msg.props
          .iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      network: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.network),
      segments: msg.segments
          .into_iter()
          .map(unique_identifier_msgs::msg::UUID::from_rmw_message)
          .collect(),
      props: msg.props
          .into_iter()
          .map(crate::msg::KeyValue::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteSegment {
    pub id: unique_identifier_msgs::msg::UUID,
    pub start: unique_identifier_msgs::msg::UUID,
    pub end: unique_identifier_msgs::msg::UUID,
    pub props: Vec<crate::msg::KeyValue>,
}



impl Default for RouteSegment {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::RouteSegment::default())
  }
}

impl rosidl_runtime_rs::Message for RouteSegment {
  type RmwMsg = crate::msg::rmw::RouteSegment;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.id)).into_owned(),
        start: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        end: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.end)).into_owned(),
        props: msg.props
          .into_iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.id)).into_owned(),
        start: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        end: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.end)).into_owned(),
        props: msg.props
          .iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.id),
      start: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.start),
      end: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.end),
      props: msg.props
          .into_iter()
          .map(crate::msg::KeyValue::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WayPoint {
    pub id: unique_identifier_msgs::msg::UUID,
    pub position: crate::msg::GeoPoint,
    pub props: Vec<crate::msg::KeyValue>,
}



impl Default for WayPoint {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::WayPoint::default())
  }
}

impl rosidl_runtime_rs::Message for WayPoint {
  type RmwMsg = crate::msg::rmw::WayPoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.id)).into_owned(),
        position: crate::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        props: msg.props
          .into_iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.id)).into_owned(),
        position: crate::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        props: msg.props
          .iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.id),
      position: crate::msg::GeoPoint::from_rmw_message(msg.position),
      props: msg.props
          .into_iter()
          .map(crate::msg::KeyValue::from_rmw_message)
          .collect(),
    }
  }
}


