

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetGeographicMap_Request {
    pub url: std::string::String,
    pub bounds: crate::msg::BoundingBox,
}



impl Default for GetGeographicMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetGeographicMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetGeographicMap_Request {
  type RmwMsg = crate::srv::rmw::GetGeographicMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        url: msg.url.as_str().into(),
        bounds: crate::msg::BoundingBox::into_rmw_message(std::borrow::Cow::Owned(msg.bounds)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        url: msg.url.as_str().into(),
        bounds: crate::msg::BoundingBox::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bounds)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      url: msg.url.to_string(),
      bounds: crate::msg::BoundingBox::from_rmw_message(msg.bounds),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetGeographicMap_Response {
    pub success: bool,
    pub status: std::string::String,
    pub map: crate::msg::GeographicMap,
}



impl Default for GetGeographicMap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetGeographicMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetGeographicMap_Response {
  type RmwMsg = crate::srv::rmw::GetGeographicMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        status: msg.status.as_str().into(),
        map: crate::msg::GeographicMap::into_rmw_message(std::borrow::Cow::Owned(msg.map)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        status: msg.status.as_str().into(),
        map: crate::msg::GeographicMap::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      status: msg.status.to_string(),
      map: crate::msg::GeographicMap::from_rmw_message(msg.map),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetRoutePlan_Request {
    pub network: unique_identifier_msgs::msg::UUID,
    pub start: unique_identifier_msgs::msg::UUID,
    pub goal: unique_identifier_msgs::msg::UUID,
}



impl Default for GetRoutePlan_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetRoutePlan_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetRoutePlan_Request {
  type RmwMsg = crate::srv::rmw::GetRoutePlan_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        network: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.network)).into_owned(),
        start: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        goal: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        network: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.network)).into_owned(),
        start: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        goal: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      network: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.network),
      start: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.start),
      goal: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetRoutePlan_Response {
    pub success: bool,
    pub status: std::string::String,
    pub plan: crate::msg::RoutePath,
}



impl Default for GetRoutePlan_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetRoutePlan_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetRoutePlan_Response {
  type RmwMsg = crate::srv::rmw::GetRoutePlan_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        status: msg.status.as_str().into(),
        plan: crate::msg::RoutePath::into_rmw_message(std::borrow::Cow::Owned(msg.plan)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        status: msg.status.as_str().into(),
        plan: crate::msg::RoutePath::into_rmw_message(std::borrow::Cow::Borrowed(&msg.plan)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      status: msg.status.to_string(),
      plan: crate::msg::RoutePath::from_rmw_message(msg.plan),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UpdateGeographicMap_Request {
    pub updates: crate::msg::GeographicMapChanges,
}



impl Default for UpdateGeographicMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::UpdateGeographicMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for UpdateGeographicMap_Request {
  type RmwMsg = crate::srv::rmw::UpdateGeographicMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        updates: crate::msg::GeographicMapChanges::into_rmw_message(std::borrow::Cow::Owned(msg.updates)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        updates: crate::msg::GeographicMapChanges::into_rmw_message(std::borrow::Cow::Borrowed(&msg.updates)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      updates: crate::msg::GeographicMapChanges::from_rmw_message(msg.updates),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UpdateGeographicMap_Response {
    pub success: bool,
    pub status: std::string::String,
}



impl Default for UpdateGeographicMap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::UpdateGeographicMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for UpdateGeographicMap_Response {
  type RmwMsg = crate::srv::rmw::UpdateGeographicMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        status: msg.status.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        status: msg.status.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      status: msg.status.to_string(),
    }
  }
}






#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetGeographicMap() -> *const std::ffi::c_void;
}

// Corresponds to geographic_msgs__srv__GetGeographicMap
pub struct GetGeographicMap;

impl rosidl_runtime_rs::Service for GetGeographicMap {
  type Request = crate::srv::GetGeographicMap_Request;
  type Response = crate::srv::GetGeographicMap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetGeographicMap() }
  }
}




#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetRoutePlan() -> *const std::ffi::c_void;
}

// Corresponds to geographic_msgs__srv__GetRoutePlan
pub struct GetRoutePlan;

impl rosidl_runtime_rs::Service for GetRoutePlan {
  type Request = crate::srv::GetRoutePlan_Request;
  type Response = crate::srv::GetRoutePlan_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetRoutePlan() }
  }
}




#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__UpdateGeographicMap() -> *const std::ffi::c_void;
}

// Corresponds to geographic_msgs__srv__UpdateGeographicMap
pub struct UpdateGeographicMap;

impl rosidl_runtime_rs::Service for UpdateGeographicMap {
  type Request = crate::srv::UpdateGeographicMap_Request;
  type Response = crate::srv::UpdateGeographicMap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__UpdateGeographicMap() }
  }
}




pub mod rmw {

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeographicMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__srv__GetGeographicMap_Request__init(msg: *mut GetGeographicMap_Request) -> bool;
    fn geographic_msgs__srv__GetGeographicMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetGeographicMap_Request>, size: usize) -> bool;
    fn geographic_msgs__srv__GetGeographicMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetGeographicMap_Request>);
    fn geographic_msgs__srv__GetGeographicMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetGeographicMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetGeographicMap_Request>) -> bool;
}

// Corresponds to geographic_msgs__srv__GetGeographicMap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetGeographicMap_Request {
    pub url: rosidl_runtime_rs::String,
    pub bounds: crate::msg::rmw::BoundingBox,
}



impl Default for GetGeographicMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__srv__GetGeographicMap_Request__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__srv__GetGeographicMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetGeographicMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetGeographicMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetGeographicMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetGeographicMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetGeographicMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetGeographicMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/srv/GetGeographicMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeographicMap_Request() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeographicMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__srv__GetGeographicMap_Response__init(msg: *mut GetGeographicMap_Response) -> bool;
    fn geographic_msgs__srv__GetGeographicMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetGeographicMap_Response>, size: usize) -> bool;
    fn geographic_msgs__srv__GetGeographicMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetGeographicMap_Response>);
    fn geographic_msgs__srv__GetGeographicMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetGeographicMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetGeographicMap_Response>) -> bool;
}

// Corresponds to geographic_msgs__srv__GetGeographicMap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetGeographicMap_Response {
    pub success: bool,
    pub status: rosidl_runtime_rs::String,
    pub map: crate::msg::rmw::GeographicMap,
}



impl Default for GetGeographicMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__srv__GetGeographicMap_Response__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__srv__GetGeographicMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetGeographicMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetGeographicMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetGeographicMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetGeographicMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetGeographicMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetGeographicMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/srv/GetGeographicMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeographicMap_Response() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Request() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__srv__GetRoutePlan_Request__init(msg: *mut GetRoutePlan_Request) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetRoutePlan_Request>, size: usize) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetRoutePlan_Request>);
    fn geographic_msgs__srv__GetRoutePlan_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetRoutePlan_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetRoutePlan_Request>) -> bool;
}

// Corresponds to geographic_msgs__srv__GetRoutePlan_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetRoutePlan_Request {
    pub network: unique_identifier_msgs::msg::rmw::UUID,
    pub start: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: unique_identifier_msgs::msg::rmw::UUID,
}



impl Default for GetRoutePlan_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__srv__GetRoutePlan_Request__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__srv__GetRoutePlan_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetRoutePlan_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetRoutePlan_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetRoutePlan_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetRoutePlan_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetRoutePlan_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetRoutePlan_Request where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/srv/GetRoutePlan_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Request() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Response() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__srv__GetRoutePlan_Response__init(msg: *mut GetRoutePlan_Response) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetRoutePlan_Response>, size: usize) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetRoutePlan_Response>);
    fn geographic_msgs__srv__GetRoutePlan_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetRoutePlan_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetRoutePlan_Response>) -> bool;
}

// Corresponds to geographic_msgs__srv__GetRoutePlan_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetRoutePlan_Response {
    pub success: bool,
    pub status: rosidl_runtime_rs::String,
    pub plan: crate::msg::rmw::RoutePath,
}



impl Default for GetRoutePlan_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__srv__GetRoutePlan_Response__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__srv__GetRoutePlan_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetRoutePlan_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetRoutePlan_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetRoutePlan_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__GetRoutePlan_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetRoutePlan_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetRoutePlan_Response where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/srv/GetRoutePlan_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Response() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__UpdateGeographicMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__srv__UpdateGeographicMap_Request__init(msg: *mut UpdateGeographicMap_Request) -> bool;
    fn geographic_msgs__srv__UpdateGeographicMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UpdateGeographicMap_Request>, size: usize) -> bool;
    fn geographic_msgs__srv__UpdateGeographicMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UpdateGeographicMap_Request>);
    fn geographic_msgs__srv__UpdateGeographicMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UpdateGeographicMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<UpdateGeographicMap_Request>) -> bool;
}

// Corresponds to geographic_msgs__srv__UpdateGeographicMap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UpdateGeographicMap_Request {
    pub updates: crate::msg::rmw::GeographicMapChanges,
}



impl Default for UpdateGeographicMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__srv__UpdateGeographicMap_Request__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__srv__UpdateGeographicMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UpdateGeographicMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__UpdateGeographicMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__UpdateGeographicMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__UpdateGeographicMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UpdateGeographicMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UpdateGeographicMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/srv/UpdateGeographicMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__UpdateGeographicMap_Request() }
  }
}


#[link(name = "geographic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__UpdateGeographicMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "geographic_msgs__rosidl_generator_c")]
extern "C" {
    fn geographic_msgs__srv__UpdateGeographicMap_Response__init(msg: *mut UpdateGeographicMap_Response) -> bool;
    fn geographic_msgs__srv__UpdateGeographicMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UpdateGeographicMap_Response>, size: usize) -> bool;
    fn geographic_msgs__srv__UpdateGeographicMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UpdateGeographicMap_Response>);
    fn geographic_msgs__srv__UpdateGeographicMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UpdateGeographicMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<UpdateGeographicMap_Response>) -> bool;
}

// Corresponds to geographic_msgs__srv__UpdateGeographicMap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UpdateGeographicMap_Response {
    pub success: bool,
    pub status: rosidl_runtime_rs::String,
}



impl Default for UpdateGeographicMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !geographic_msgs__srv__UpdateGeographicMap_Response__init(&mut msg as *mut _) {
        panic!("Call to geographic_msgs__srv__UpdateGeographicMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UpdateGeographicMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__UpdateGeographicMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__UpdateGeographicMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { geographic_msgs__srv__UpdateGeographicMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UpdateGeographicMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UpdateGeographicMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "geographic_msgs/srv/UpdateGeographicMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__UpdateGeographicMap_Response() }
  }
}






  #[link(name = "geographic_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetGeographicMap() -> *const std::ffi::c_void;
  }

  // Corresponds to geographic_msgs__srv__GetGeographicMap
  pub struct GetGeographicMap;

  impl rosidl_runtime_rs::Service for GetGeographicMap {
    type Request = crate::srv::rmw::GetGeographicMap_Request;
    type Response = crate::srv::rmw::GetGeographicMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetGeographicMap() }
    }
  }




  #[link(name = "geographic_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetRoutePlan() -> *const std::ffi::c_void;
  }

  // Corresponds to geographic_msgs__srv__GetRoutePlan
  pub struct GetRoutePlan;

  impl rosidl_runtime_rs::Service for GetRoutePlan {
    type Request = crate::srv::rmw::GetRoutePlan_Request;
    type Response = crate::srv::rmw::GetRoutePlan_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetRoutePlan() }
    }
  }




  #[link(name = "geographic_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__UpdateGeographicMap() -> *const std::ffi::c_void;
  }

  // Corresponds to geographic_msgs__srv__UpdateGeographicMap
  pub struct UpdateGeographicMap;

  impl rosidl_runtime_rs::Service for UpdateGeographicMap {
    type Request = crate::srv::rmw::UpdateGeographicMap_Request;
    type Response = crate::srv::rmw::UpdateGeographicMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__UpdateGeographicMap() }
    }
  }


}  // mod rmw
