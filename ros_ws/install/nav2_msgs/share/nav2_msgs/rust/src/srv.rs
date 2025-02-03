

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCosts_Request {
    pub use_footprint: bool,
    pub poses: Vec<geometry_msgs::msg::PoseStamped>,
}



impl Default for GetCosts_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetCosts_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetCosts_Request {
  type RmwMsg = crate::srv::rmw::GetCosts_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        use_footprint: msg.use_footprint,
        poses: msg.poses
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      use_footprint: msg.use_footprint,
        poses: msg.poses
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      use_footprint: msg.use_footprint,
      poses: msg.poses
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCosts_Response {
    pub costs: Vec<f32>,
    pub success: bool,
}



impl Default for GetCosts_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetCosts_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetCosts_Response {
  type RmwMsg = crate::srv::rmw::GetCosts_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        costs: msg.costs.into(),
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        costs: msg.costs.as_slice().into(),
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      costs: msg.costs
          .into_iter()
          .collect(),
      success: msg.success,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCostmap_Request {
    pub specs: crate::msg::CostmapMetaData,
}



impl Default for GetCostmap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetCostmap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetCostmap_Request {
  type RmwMsg = crate::srv::rmw::GetCostmap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        specs: crate::msg::CostmapMetaData::into_rmw_message(std::borrow::Cow::Owned(msg.specs)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        specs: crate::msg::CostmapMetaData::into_rmw_message(std::borrow::Cow::Borrowed(&msg.specs)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      specs: crate::msg::CostmapMetaData::from_rmw_message(msg.specs),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCostmap_Response {
    pub map: crate::msg::Costmap,
}



impl Default for GetCostmap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetCostmap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetCostmap_Response {
  type RmwMsg = crate::srv::rmw::GetCostmap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: crate::msg::Costmap::into_rmw_message(std::borrow::Cow::Owned(msg.map)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: crate::msg::Costmap::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map: crate::msg::Costmap::from_rmw_message(msg.map),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsPathValid_Request {
    pub path: nav_msgs::msg::Path,
}



impl Default for IsPathValid_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::IsPathValid_Request::default())
  }
}

impl rosidl_runtime_rs::Message for IsPathValid_Request {
  type RmwMsg = crate::srv::rmw::IsPathValid_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsPathValid_Response {
    pub is_valid: bool,
    pub invalid_pose_indices: Vec<i32>,
}



impl Default for IsPathValid_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::IsPathValid_Response::default())
  }
}

impl rosidl_runtime_rs::Message for IsPathValid_Response {
  type RmwMsg = crate::srv::rmw::IsPathValid_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        is_valid: msg.is_valid,
        invalid_pose_indices: msg.invalid_pose_indices.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      is_valid: msg.is_valid,
        invalid_pose_indices: msg.invalid_pose_indices.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      is_valid: msg.is_valid,
      invalid_pose_indices: msg.invalid_pose_indices
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapExceptRegion_Request {
    pub reset_distance: f32,
}



impl Default for ClearCostmapExceptRegion_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ClearCostmapExceptRegion_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapExceptRegion_Request {
  type RmwMsg = crate::srv::rmw::ClearCostmapExceptRegion_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        reset_distance: msg.reset_distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      reset_distance: msg.reset_distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      reset_distance: msg.reset_distance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapExceptRegion_Response {
    pub response: std_msgs::msg::Empty,
}



impl Default for ClearCostmapExceptRegion_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ClearCostmapExceptRegion_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapExceptRegion_Response {
  type RmwMsg = crate::srv::rmw::ClearCostmapExceptRegion_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.response)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.response)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      response: std_msgs::msg::Empty::from_rmw_message(msg.response),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapAroundRobot_Request {
    pub reset_distance: f32,
}



impl Default for ClearCostmapAroundRobot_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ClearCostmapAroundRobot_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapAroundRobot_Request {
  type RmwMsg = crate::srv::rmw::ClearCostmapAroundRobot_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        reset_distance: msg.reset_distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      reset_distance: msg.reset_distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      reset_distance: msg.reset_distance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapAroundRobot_Response {
    pub response: std_msgs::msg::Empty,
}



impl Default for ClearCostmapAroundRobot_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ClearCostmapAroundRobot_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapAroundRobot_Response {
  type RmwMsg = crate::srv::rmw::ClearCostmapAroundRobot_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.response)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.response)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      response: std_msgs::msg::Empty::from_rmw_message(msg.response),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearEntireCostmap_Request {
    pub request: std_msgs::msg::Empty,
}



impl Default for ClearEntireCostmap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ClearEntireCostmap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ClearEntireCostmap_Request {
  type RmwMsg = crate::srv::rmw::ClearEntireCostmap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.request)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.request)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      request: std_msgs::msg::Empty::from_rmw_message(msg.request),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearEntireCostmap_Response {
    pub response: std_msgs::msg::Empty,
}



impl Default for ClearEntireCostmap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ClearEntireCostmap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ClearEntireCostmap_Response {
  type RmwMsg = crate::srv::rmw::ClearEntireCostmap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.response)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.response)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      response: std_msgs::msg::Empty::from_rmw_message(msg.response),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageLifecycleNodes_Request {
    pub command: u8,
}

impl ManageLifecycleNodes_Request {
    pub const STARTUP: u8 = 0;
    pub const PAUSE: u8 = 1;
    pub const RESUME: u8 = 2;
    pub const RESET: u8 = 3;
    pub const SHUTDOWN: u8 = 4;
    pub const CONFIGURE: u8 = 5;
    pub const CLEANUP: u8 = 6;
}


impl Default for ManageLifecycleNodes_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ManageLifecycleNodes_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ManageLifecycleNodes_Request {
  type RmwMsg = crate::srv::rmw::ManageLifecycleNodes_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: msg.command,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      command: msg.command,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      command: msg.command,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageLifecycleNodes_Response {
    pub success: bool,
}



impl Default for ManageLifecycleNodes_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ManageLifecycleNodes_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ManageLifecycleNodes_Response {
  type RmwMsg = crate::srv::rmw::ManageLifecycleNodes_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Request {
    pub map_url: std::string::String,
}



impl Default for LoadMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::LoadMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Request {
  type RmwMsg = crate::srv::rmw::LoadMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_url: msg.map_url.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_url: msg.map_url.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_url: msg.map_url.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Response {
    pub map: nav_msgs::msg::OccupancyGrid,
    pub result: u8,
}

impl LoadMap_Response {
    pub const RESULT_SUCCESS: u8 = 0;
    pub const RESULT_MAP_DOES_NOT_EXIST: u8 = 1;
    pub const RESULT_INVALID_MAP_DATA: u8 = 2;
    pub const RESULT_INVALID_MAP_METADATA: u8 = 3;
    pub const RESULT_UNDEFINED_FAILURE: u8 = 255;
}


impl Default for LoadMap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::LoadMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Response {
  type RmwMsg = crate::srv::rmw::LoadMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: nav_msgs::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Owned(msg.map)).into_owned(),
        result: msg.result,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: nav_msgs::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map)).into_owned(),
      result: msg.result,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map: nav_msgs::msg::OccupancyGrid::from_rmw_message(msg.map),
      result: msg.result,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Request {
    pub map_topic: std::string::String,
    pub map_url: std::string::String,
    pub image_format: std::string::String,
    pub map_mode: std::string::String,
    pub free_thresh: f32,
    pub occupied_thresh: f32,
}



impl Default for SaveMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SaveMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Request {
  type RmwMsg = crate::srv::rmw::SaveMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_topic: msg.map_topic.as_str().into(),
        map_url: msg.map_url.as_str().into(),
        image_format: msg.image_format.as_str().into(),
        map_mode: msg.map_mode.as_str().into(),
        free_thresh: msg.free_thresh,
        occupied_thresh: msg.occupied_thresh,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_topic: msg.map_topic.as_str().into(),
        map_url: msg.map_url.as_str().into(),
        image_format: msg.image_format.as_str().into(),
        map_mode: msg.map_mode.as_str().into(),
      free_thresh: msg.free_thresh,
      occupied_thresh: msg.occupied_thresh,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_topic: msg.map_topic.to_string(),
      map_url: msg.map_url.to_string(),
      image_format: msg.image_format.to_string(),
      map_mode: msg.map_mode.to_string(),
      free_thresh: msg.free_thresh,
      occupied_thresh: msg.occupied_thresh,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Response {
    pub result: bool,
}



impl Default for SaveMap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SaveMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Response {
  type RmwMsg = crate::srv::rmw::SaveMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: msg.result,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      result: msg.result,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      result: msg.result,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInitialPose_Request {
    pub pose: geometry_msgs::msg::PoseWithCovarianceStamped,
}



impl Default for SetInitialPose_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SetInitialPose_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetInitialPose_Request {
  type RmwMsg = crate::srv::rmw::SetInitialPose_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::PoseWithCovarianceStamped::from_rmw_message(msg.pose),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInitialPose_Response {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for SetInitialPose_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SetInitialPose_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetInitialPose_Response {
  type RmwMsg = crate::srv::rmw::SetInitialPose_Response;

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
pub struct ReloadDockDatabase_Request {
    pub filepath: std::string::String,
}



impl Default for ReloadDockDatabase_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ReloadDockDatabase_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ReloadDockDatabase_Request {
  type RmwMsg = crate::srv::rmw::ReloadDockDatabase_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        filepath: msg.filepath.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        filepath: msg.filepath.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      filepath: msg.filepath.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReloadDockDatabase_Response {
    pub success: bool,
}



impl Default for ReloadDockDatabase_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ReloadDockDatabase_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ReloadDockDatabase_Response {
  type RmwMsg = crate::srv::rmw::ReloadDockDatabase_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}






#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCosts() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__GetCosts
pub struct GetCosts;

impl rosidl_runtime_rs::Service for GetCosts {
  type Request = crate::srv::GetCosts_Request;
  type Response = crate::srv::GetCosts_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCosts() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCostmap() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__GetCostmap
pub struct GetCostmap;

impl rosidl_runtime_rs::Service for GetCostmap {
  type Request = crate::srv::GetCostmap_Request;
  type Response = crate::srv::GetCostmap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCostmap() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__IsPathValid() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__IsPathValid
pub struct IsPathValid;

impl rosidl_runtime_rs::Service for IsPathValid {
  type Request = crate::srv::IsPathValid_Request;
  type Response = crate::srv::IsPathValid_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__IsPathValid() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion
pub struct ClearCostmapExceptRegion;

impl rosidl_runtime_rs::Service for ClearCostmapExceptRegion {
  type Request = crate::srv::ClearCostmapExceptRegion_Request;
  type Response = crate::srv::ClearCostmapExceptRegion_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot
pub struct ClearCostmapAroundRobot;

impl rosidl_runtime_rs::Service for ClearCostmapAroundRobot {
  type Request = crate::srv::ClearCostmapAroundRobot_Request;
  type Response = crate::srv::ClearCostmapAroundRobot_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearEntireCostmap() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ClearEntireCostmap
pub struct ClearEntireCostmap;

impl rosidl_runtime_rs::Service for ClearEntireCostmap {
  type Request = crate::srv::ClearEntireCostmap_Request;
  type Response = crate::srv::ClearEntireCostmap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearEntireCostmap() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ManageLifecycleNodes
pub struct ManageLifecycleNodes;

impl rosidl_runtime_rs::Service for ManageLifecycleNodes {
  type Request = crate::srv::ManageLifecycleNodes_Request;
  type Response = crate::srv::ManageLifecycleNodes_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__LoadMap() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__LoadMap
pub struct LoadMap;

impl rosidl_runtime_rs::Service for LoadMap {
  type Request = crate::srv::LoadMap_Request;
  type Response = crate::srv::LoadMap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__LoadMap() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SaveMap() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__SaveMap
pub struct SaveMap;

impl rosidl_runtime_rs::Service for SaveMap {
  type Request = crate::srv::SaveMap_Request;
  type Response = crate::srv::SaveMap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SaveMap() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SetInitialPose() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__SetInitialPose
pub struct SetInitialPose;

impl rosidl_runtime_rs::Service for SetInitialPose {
  type Request = crate::srv::SetInitialPose_Request;
  type Response = crate::srv::SetInitialPose_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SetInitialPose() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ReloadDockDatabase() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ReloadDockDatabase
pub struct ReloadDockDatabase;

impl rosidl_runtime_rs::Service for ReloadDockDatabase {
  type Request = crate::srv::ReloadDockDatabase_Request;
  type Response = crate::srv::ReloadDockDatabase_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ReloadDockDatabase() }
  }
}




pub mod rmw {

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCosts_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__GetCosts_Request__init(msg: *mut GetCosts_Request) -> bool;
    fn nav2_msgs__srv__GetCosts_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetCosts_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__GetCosts_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetCosts_Request>);
    fn nav2_msgs__srv__GetCosts_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetCosts_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetCosts_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__GetCosts_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCosts_Request {
    pub use_footprint: bool,
    pub poses: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,
}



impl Default for GetCosts_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__GetCosts_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__GetCosts_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetCosts_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCosts_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCosts_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCosts_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetCosts_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetCosts_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/GetCosts_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCosts_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCosts_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__GetCosts_Response__init(msg: *mut GetCosts_Response) -> bool;
    fn nav2_msgs__srv__GetCosts_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetCosts_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__GetCosts_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetCosts_Response>);
    fn nav2_msgs__srv__GetCosts_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetCosts_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetCosts_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__GetCosts_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCosts_Response {
    pub costs: rosidl_runtime_rs::Sequence<f32>,
    pub success: bool,
}



impl Default for GetCosts_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__GetCosts_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__GetCosts_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetCosts_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCosts_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCosts_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCosts_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetCosts_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetCosts_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/GetCosts_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCosts_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__GetCostmap_Request__init(msg: *mut GetCostmap_Request) -> bool;
    fn nav2_msgs__srv__GetCostmap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__GetCostmap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Request>);
    fn nav2_msgs__srv__GetCostmap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetCostmap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__GetCostmap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCostmap_Request {
    pub specs: crate::msg::rmw::CostmapMetaData,
}



impl Default for GetCostmap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__GetCostmap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__GetCostmap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetCostmap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetCostmap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetCostmap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/GetCostmap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__GetCostmap_Response__init(msg: *mut GetCostmap_Response) -> bool;
    fn nav2_msgs__srv__GetCostmap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__GetCostmap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Response>);
    fn nav2_msgs__srv__GetCostmap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetCostmap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__GetCostmap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCostmap_Response {
    pub map: crate::msg::rmw::Costmap,
}



impl Default for GetCostmap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__GetCostmap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__GetCostmap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetCostmap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetCostmap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetCostmap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/GetCostmap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__IsPathValid_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__IsPathValid_Request__init(msg: *mut IsPathValid_Request) -> bool;
    fn nav2_msgs__srv__IsPathValid_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__IsPathValid_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Request>);
    fn nav2_msgs__srv__IsPathValid_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IsPathValid_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__IsPathValid_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsPathValid_Request {
    pub path: nav_msgs::msg::rmw::Path,
}



impl Default for IsPathValid_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__IsPathValid_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__IsPathValid_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IsPathValid_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IsPathValid_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IsPathValid_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/IsPathValid_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__IsPathValid_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__IsPathValid_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__IsPathValid_Response__init(msg: *mut IsPathValid_Response) -> bool;
    fn nav2_msgs__srv__IsPathValid_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__IsPathValid_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Response>);
    fn nav2_msgs__srv__IsPathValid_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IsPathValid_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__IsPathValid_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsPathValid_Response {
    pub is_valid: bool,
    pub invalid_pose_indices: rosidl_runtime_rs::Sequence<i32>,
}



impl Default for IsPathValid_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__IsPathValid_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__IsPathValid_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IsPathValid_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IsPathValid_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IsPathValid_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/IsPathValid_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__IsPathValid_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Request__init(msg: *mut ClearCostmapExceptRegion_Request) -> bool;
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Request>);
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapExceptRegion_Request {
    pub reset_distance: f32,
}



impl Default for ClearCostmapExceptRegion_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearCostmapExceptRegion_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearCostmapExceptRegion_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearCostmapExceptRegion_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapExceptRegion_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearCostmapExceptRegion_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearCostmapExceptRegion_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Response__init(msg: *mut ClearCostmapExceptRegion_Response) -> bool;
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Response>);
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapExceptRegion_Response {
    pub response: std_msgs::msg::rmw::Empty,
}



impl Default for ClearCostmapExceptRegion_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearCostmapExceptRegion_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearCostmapExceptRegion_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearCostmapExceptRegion_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapExceptRegion_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearCostmapExceptRegion_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearCostmapExceptRegion_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Request__init(msg: *mut ClearCostmapAroundRobot_Request) -> bool;
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Request>);
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapAroundRobot_Request {
    pub reset_distance: f32,
}



impl Default for ClearCostmapAroundRobot_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearCostmapAroundRobot_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearCostmapAroundRobot_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearCostmapAroundRobot_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapAroundRobot_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearCostmapAroundRobot_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearCostmapAroundRobot_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Response__init(msg: *mut ClearCostmapAroundRobot_Response) -> bool;
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Response>);
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapAroundRobot_Response {
    pub response: std_msgs::msg::rmw::Empty,
}



impl Default for ClearCostmapAroundRobot_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearCostmapAroundRobot_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearCostmapAroundRobot_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearCostmapAroundRobot_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapAroundRobot_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearCostmapAroundRobot_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearCostmapAroundRobot_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearEntireCostmap_Request__init(msg: *mut ClearEntireCostmap_Request) -> bool;
    fn nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Request>);
    fn nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearEntireCostmap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearEntireCostmap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearEntireCostmap_Request {
    pub request: std_msgs::msg::rmw::Empty,
}



impl Default for ClearEntireCostmap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearEntireCostmap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearEntireCostmap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearEntireCostmap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearEntireCostmap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearEntireCostmap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearEntireCostmap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearEntireCostmap_Response__init(msg: *mut ClearEntireCostmap_Response) -> bool;
    fn nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Response>);
    fn nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearEntireCostmap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearEntireCostmap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearEntireCostmap_Response {
    pub response: std_msgs::msg::rmw::Empty,
}



impl Default for ClearEntireCostmap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearEntireCostmap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearEntireCostmap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearEntireCostmap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearEntireCostmap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearEntireCostmap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearEntireCostmap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ManageLifecycleNodes_Request__init(msg: *mut ManageLifecycleNodes_Request) -> bool;
    fn nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Request>);
    fn nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ManageLifecycleNodes_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageLifecycleNodes_Request {
    pub command: u8,
}

impl ManageLifecycleNodes_Request {
    pub const STARTUP: u8 = 0;
    pub const PAUSE: u8 = 1;
    pub const RESUME: u8 = 2;
    pub const RESET: u8 = 3;
    pub const SHUTDOWN: u8 = 4;
    pub const CONFIGURE: u8 = 5;
    pub const CLEANUP: u8 = 6;
}


impl Default for ManageLifecycleNodes_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ManageLifecycleNodes_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ManageLifecycleNodes_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManageLifecycleNodes_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManageLifecycleNodes_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManageLifecycleNodes_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ManageLifecycleNodes_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ManageLifecycleNodes_Response__init(msg: *mut ManageLifecycleNodes_Response) -> bool;
    fn nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Response>);
    fn nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ManageLifecycleNodes_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageLifecycleNodes_Response {
    pub success: bool,
}



impl Default for ManageLifecycleNodes_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ManageLifecycleNodes_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ManageLifecycleNodes_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManageLifecycleNodes_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManageLifecycleNodes_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManageLifecycleNodes_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ManageLifecycleNodes_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__LoadMap_Request__init(msg: *mut LoadMap_Request) -> bool;
    fn nav2_msgs__srv__LoadMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__LoadMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>);
    fn nav2_msgs__srv__LoadMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__LoadMap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Request {
    pub map_url: rosidl_runtime_rs::String,
}



impl Default for LoadMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__LoadMap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__LoadMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/LoadMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__LoadMap_Response__init(msg: *mut LoadMap_Response) -> bool;
    fn nav2_msgs__srv__LoadMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__LoadMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>);
    fn nav2_msgs__srv__LoadMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__LoadMap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Response {
    pub map: nav_msgs::msg::rmw::OccupancyGrid,
    pub result: u8,
}

impl LoadMap_Response {
    pub const RESULT_SUCCESS: u8 = 0;
    pub const RESULT_MAP_DOES_NOT_EXIST: u8 = 1;
    pub const RESULT_INVALID_MAP_DATA: u8 = 2;
    pub const RESULT_INVALID_MAP_METADATA: u8 = 3;
    pub const RESULT_UNDEFINED_FAILURE: u8 = 255;
}


impl Default for LoadMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__LoadMap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__LoadMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/LoadMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SaveMap_Request__init(msg: *mut SaveMap_Request) -> bool;
    fn nav2_msgs__srv__SaveMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__SaveMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>);
    fn nav2_msgs__srv__SaveMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SaveMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__SaveMap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Request {
    pub map_topic: rosidl_runtime_rs::String,
    pub map_url: rosidl_runtime_rs::String,
    pub image_format: rosidl_runtime_rs::String,
    pub map_mode: rosidl_runtime_rs::String,
    pub free_thresh: f32,
    pub occupied_thresh: f32,
}



impl Default for SaveMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SaveMap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SaveMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SaveMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SaveMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SaveMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SaveMap_Response__init(msg: *mut SaveMap_Response) -> bool;
    fn nav2_msgs__srv__SaveMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__SaveMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>);
    fn nav2_msgs__srv__SaveMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SaveMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__SaveMap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Response {
    pub result: bool,
}



impl Default for SaveMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SaveMap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SaveMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SaveMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SaveMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SaveMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetInitialPose_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SetInitialPose_Request__init(msg: *mut SetInitialPose_Request) -> bool;
    fn nav2_msgs__srv__SetInitialPose_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__SetInitialPose_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Request>);
    fn nav2_msgs__srv__SetInitialPose_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetInitialPose_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__SetInitialPose_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInitialPose_Request {
    pub pose: geometry_msgs::msg::rmw::PoseWithCovarianceStamped,
}



impl Default for SetInitialPose_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SetInitialPose_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SetInitialPose_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetInitialPose_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetInitialPose_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetInitialPose_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SetInitialPose_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetInitialPose_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetInitialPose_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SetInitialPose_Response__init(msg: *mut SetInitialPose_Response) -> bool;
    fn nav2_msgs__srv__SetInitialPose_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__SetInitialPose_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Response>);
    fn nav2_msgs__srv__SetInitialPose_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetInitialPose_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__SetInitialPose_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInitialPose_Response {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for SetInitialPose_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SetInitialPose_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SetInitialPose_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetInitialPose_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetInitialPose_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetInitialPose_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SetInitialPose_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetInitialPose_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ReloadDockDatabase_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ReloadDockDatabase_Request__init(msg: *mut ReloadDockDatabase_Request) -> bool;
    fn nav2_msgs__srv__ReloadDockDatabase_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReloadDockDatabase_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ReloadDockDatabase_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReloadDockDatabase_Request>);
    fn nav2_msgs__srv__ReloadDockDatabase_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReloadDockDatabase_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ReloadDockDatabase_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ReloadDockDatabase_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReloadDockDatabase_Request {
    pub filepath: rosidl_runtime_rs::String,
}



impl Default for ReloadDockDatabase_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ReloadDockDatabase_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ReloadDockDatabase_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReloadDockDatabase_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ReloadDockDatabase_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ReloadDockDatabase_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ReloadDockDatabase_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReloadDockDatabase_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReloadDockDatabase_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ReloadDockDatabase_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ReloadDockDatabase_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ReloadDockDatabase_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ReloadDockDatabase_Response__init(msg: *mut ReloadDockDatabase_Response) -> bool;
    fn nav2_msgs__srv__ReloadDockDatabase_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReloadDockDatabase_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ReloadDockDatabase_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReloadDockDatabase_Response>);
    fn nav2_msgs__srv__ReloadDockDatabase_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReloadDockDatabase_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ReloadDockDatabase_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ReloadDockDatabase_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReloadDockDatabase_Response {
    pub success: bool,
}



impl Default for ReloadDockDatabase_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ReloadDockDatabase_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ReloadDockDatabase_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReloadDockDatabase_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ReloadDockDatabase_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ReloadDockDatabase_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ReloadDockDatabase_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReloadDockDatabase_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReloadDockDatabase_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ReloadDockDatabase_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ReloadDockDatabase_Response() }
  }
}






  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCosts() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__GetCosts
  pub struct GetCosts;

  impl rosidl_runtime_rs::Service for GetCosts {
    type Request = crate::srv::rmw::GetCosts_Request;
    type Response = crate::srv::rmw::GetCosts_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCosts() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCostmap() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__GetCostmap
  pub struct GetCostmap;

  impl rosidl_runtime_rs::Service for GetCostmap {
    type Request = crate::srv::rmw::GetCostmap_Request;
    type Response = crate::srv::rmw::GetCostmap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCostmap() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__IsPathValid() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__IsPathValid
  pub struct IsPathValid;

  impl rosidl_runtime_rs::Service for IsPathValid {
    type Request = crate::srv::rmw::IsPathValid_Request;
    type Response = crate::srv::rmw::IsPathValid_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__IsPathValid() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion
  pub struct ClearCostmapExceptRegion;

  impl rosidl_runtime_rs::Service for ClearCostmapExceptRegion {
    type Request = crate::srv::rmw::ClearCostmapExceptRegion_Request;
    type Response = crate::srv::rmw::ClearCostmapExceptRegion_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot
  pub struct ClearCostmapAroundRobot;

  impl rosidl_runtime_rs::Service for ClearCostmapAroundRobot {
    type Request = crate::srv::rmw::ClearCostmapAroundRobot_Request;
    type Response = crate::srv::rmw::ClearCostmapAroundRobot_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearEntireCostmap() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__ClearEntireCostmap
  pub struct ClearEntireCostmap;

  impl rosidl_runtime_rs::Service for ClearEntireCostmap {
    type Request = crate::srv::rmw::ClearEntireCostmap_Request;
    type Response = crate::srv::rmw::ClearEntireCostmap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearEntireCostmap() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__ManageLifecycleNodes
  pub struct ManageLifecycleNodes;

  impl rosidl_runtime_rs::Service for ManageLifecycleNodes {
    type Request = crate::srv::rmw::ManageLifecycleNodes_Request;
    type Response = crate::srv::rmw::ManageLifecycleNodes_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__LoadMap() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__LoadMap
  pub struct LoadMap;

  impl rosidl_runtime_rs::Service for LoadMap {
    type Request = crate::srv::rmw::LoadMap_Request;
    type Response = crate::srv::rmw::LoadMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__LoadMap() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SaveMap() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__SaveMap
  pub struct SaveMap;

  impl rosidl_runtime_rs::Service for SaveMap {
    type Request = crate::srv::rmw::SaveMap_Request;
    type Response = crate::srv::rmw::SaveMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SaveMap() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SetInitialPose() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__SetInitialPose
  pub struct SetInitialPose;

  impl rosidl_runtime_rs::Service for SetInitialPose {
    type Request = crate::srv::rmw::SetInitialPose_Request;
    type Response = crate::srv::rmw::SetInitialPose_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SetInitialPose() }
    }
  }




  #[link(name = "nav2_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ReloadDockDatabase() -> *const std::ffi::c_void;
  }

  // Corresponds to nav2_msgs__srv__ReloadDockDatabase
  pub struct ReloadDockDatabase;

  impl rosidl_runtime_rs::Service for ReloadDockDatabase {
    type Request = crate::srv::rmw::ReloadDockDatabase_Request;
    type Response = crate::srv::rmw::ReloadDockDatabase_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ReloadDockDatabase() }
    }
  }


}  // mod rmw
