// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from geographic_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice
#include "geographic_msgs/msg/detail/bounding_box__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "geographic_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "geographic_msgs/msg/detail/bounding_box__struct.h"
#include "geographic_msgs/msg/detail/bounding_box__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geographic_msgs/msg/detail/geo_point__functions.h"  // max_pt, min_pt

// forward declare type support functions

bool cdr_serialize_geographic_msgs__msg__GeoPoint(
  const geographic_msgs__msg__GeoPoint * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_geographic_msgs__msg__GeoPoint(
  eprosima::fastcdr::Cdr & cdr,
  geographic_msgs__msg__GeoPoint * ros_message);

size_t get_serialized_size_geographic_msgs__msg__GeoPoint(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_geographic_msgs__msg__GeoPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_geographic_msgs__msg__GeoPoint(
  const geographic_msgs__msg__GeoPoint * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_geographic_msgs__msg__GeoPoint(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_geographic_msgs__msg__GeoPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geographic_msgs, msg, GeoPoint)();


using _BoundingBox__ros_msg_type = geographic_msgs__msg__BoundingBox;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
bool cdr_serialize_geographic_msgs__msg__BoundingBox(
  const geographic_msgs__msg__BoundingBox * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: min_pt
  {
    cdr_serialize_geographic_msgs__msg__GeoPoint(
      &ros_message->min_pt, cdr);
  }

  // Field name: max_pt
  {
    cdr_serialize_geographic_msgs__msg__GeoPoint(
      &ros_message->max_pt, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
bool cdr_deserialize_geographic_msgs__msg__BoundingBox(
  eprosima::fastcdr::Cdr & cdr,
  geographic_msgs__msg__BoundingBox * ros_message)
{
  // Field name: min_pt
  {
    cdr_deserialize_geographic_msgs__msg__GeoPoint(cdr, &ros_message->min_pt);
  }

  // Field name: max_pt
  {
    cdr_deserialize_geographic_msgs__msg__GeoPoint(cdr, &ros_message->max_pt);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
size_t get_serialized_size_geographic_msgs__msg__BoundingBox(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BoundingBox__ros_msg_type * ros_message = static_cast<const _BoundingBox__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: min_pt
  current_alignment += get_serialized_size_geographic_msgs__msg__GeoPoint(
    &(ros_message->min_pt), current_alignment);

  // Field name: max_pt
  current_alignment += get_serialized_size_geographic_msgs__msg__GeoPoint(
    &(ros_message->max_pt), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
size_t max_serialized_size_geographic_msgs__msg__BoundingBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: min_pt
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geographic_msgs__msg__GeoPoint(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: max_pt
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geographic_msgs__msg__GeoPoint(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = geographic_msgs__msg__BoundingBox;
    is_plain =
      (
      offsetof(DataType, max_pt) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
bool cdr_serialize_key_geographic_msgs__msg__BoundingBox(
  const geographic_msgs__msg__BoundingBox * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: min_pt
  {
    cdr_serialize_key_geographic_msgs__msg__GeoPoint(
      &ros_message->min_pt, cdr);
  }

  // Field name: max_pt
  {
    cdr_serialize_key_geographic_msgs__msg__GeoPoint(
      &ros_message->max_pt, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
size_t get_serialized_size_key_geographic_msgs__msg__BoundingBox(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BoundingBox__ros_msg_type * ros_message = static_cast<const _BoundingBox__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: min_pt
  current_alignment += get_serialized_size_key_geographic_msgs__msg__GeoPoint(
    &(ros_message->min_pt), current_alignment);

  // Field name: max_pt
  current_alignment += get_serialized_size_key_geographic_msgs__msg__GeoPoint(
    &(ros_message->max_pt), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
size_t max_serialized_size_key_geographic_msgs__msg__BoundingBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: min_pt
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geographic_msgs__msg__GeoPoint(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: max_pt
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geographic_msgs__msg__GeoPoint(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = geographic_msgs__msg__BoundingBox;
    is_plain =
      (
      offsetof(DataType, max_pt) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _BoundingBox__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const geographic_msgs__msg__BoundingBox * ros_message = static_cast<const geographic_msgs__msg__BoundingBox *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_geographic_msgs__msg__BoundingBox(ros_message, cdr);
}

static bool _BoundingBox__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  geographic_msgs__msg__BoundingBox * ros_message = static_cast<geographic_msgs__msg__BoundingBox *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_geographic_msgs__msg__BoundingBox(cdr, ros_message);
}

static uint32_t _BoundingBox__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_geographic_msgs__msg__BoundingBox(
      untyped_ros_message, 0));
}

static size_t _BoundingBox__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_geographic_msgs__msg__BoundingBox(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BoundingBox = {
  "geographic_msgs::msg",
  "BoundingBox",
  _BoundingBox__cdr_serialize,
  _BoundingBox__cdr_deserialize,
  _BoundingBox__get_serialized_size,
  _BoundingBox__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _BoundingBox__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BoundingBox,
  get_message_typesupport_handle_function,
  &geographic_msgs__msg__BoundingBox__get_type_hash,
  &geographic_msgs__msg__BoundingBox__get_type_description,
  &geographic_msgs__msg__BoundingBox__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geographic_msgs, msg, BoundingBox)() {
  return &_BoundingBox__type_support;
}

#if defined(__cplusplus)
}
#endif
