// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from geographic_msgs:msg/RouteSegment.idl
// generated code does not contain a copyright notice
#include "geographic_msgs/msg/detail/route_segment__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "geographic_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "geographic_msgs/msg/detail/route_segment__struct.h"
#include "geographic_msgs/msg/detail/route_segment__functions.h"
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

#include "geographic_msgs/msg/detail/key_value__functions.h"  // props
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"  // end, id, start

// forward declare type support functions

bool cdr_serialize_geographic_msgs__msg__KeyValue(
  const geographic_msgs__msg__KeyValue * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_geographic_msgs__msg__KeyValue(
  eprosima::fastcdr::Cdr & cdr,
  geographic_msgs__msg__KeyValue * ros_message);

size_t get_serialized_size_geographic_msgs__msg__KeyValue(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_geographic_msgs__msg__KeyValue(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_geographic_msgs__msg__KeyValue(
  const geographic_msgs__msg__KeyValue * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_geographic_msgs__msg__KeyValue(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_geographic_msgs__msg__KeyValue(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geographic_msgs, msg, KeyValue)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_geographic_msgs
bool cdr_serialize_unique_identifier_msgs__msg__UUID(
  const unique_identifier_msgs__msg__UUID * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_geographic_msgs
bool cdr_deserialize_unique_identifier_msgs__msg__UUID(
  eprosima::fastcdr::Cdr & cdr,
  unique_identifier_msgs__msg__UUID * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_geographic_msgs
size_t get_serialized_size_unique_identifier_msgs__msg__UUID(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_geographic_msgs
size_t max_serialized_size_unique_identifier_msgs__msg__UUID(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_geographic_msgs
bool cdr_serialize_key_unique_identifier_msgs__msg__UUID(
  const unique_identifier_msgs__msg__UUID * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_geographic_msgs
size_t get_serialized_size_key_unique_identifier_msgs__msg__UUID(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_geographic_msgs
size_t max_serialized_size_key_unique_identifier_msgs__msg__UUID(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_geographic_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unique_identifier_msgs, msg, UUID)();


using _RouteSegment__ros_msg_type = geographic_msgs__msg__RouteSegment;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
bool cdr_serialize_geographic_msgs__msg__RouteSegment(
  const geographic_msgs__msg__RouteSegment * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: id
  {
    cdr_serialize_unique_identifier_msgs__msg__UUID(
      &ros_message->id, cdr);
  }

  // Field name: start
  {
    cdr_serialize_unique_identifier_msgs__msg__UUID(
      &ros_message->start, cdr);
  }

  // Field name: end
  {
    cdr_serialize_unique_identifier_msgs__msg__UUID(
      &ros_message->end, cdr);
  }

  // Field name: props
  {
    size_t size = ros_message->props.size;
    auto array_ptr = ros_message->props.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_geographic_msgs__msg__KeyValue(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
bool cdr_deserialize_geographic_msgs__msg__RouteSegment(
  eprosima::fastcdr::Cdr & cdr,
  geographic_msgs__msg__RouteSegment * ros_message)
{
  // Field name: id
  {
    cdr_deserialize_unique_identifier_msgs__msg__UUID(cdr, &ros_message->id);
  }

  // Field name: start
  {
    cdr_deserialize_unique_identifier_msgs__msg__UUID(cdr, &ros_message->start);
  }

  // Field name: end
  {
    cdr_deserialize_unique_identifier_msgs__msg__UUID(cdr, &ros_message->end);
  }

  // Field name: props
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->props.data) {
      geographic_msgs__msg__KeyValue__Sequence__fini(&ros_message->props);
    }
    if (!geographic_msgs__msg__KeyValue__Sequence__init(&ros_message->props, size)) {
      fprintf(stderr, "failed to create array for field 'props'");
      return false;
    }
    auto array_ptr = ros_message->props.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_geographic_msgs__msg__KeyValue(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
size_t get_serialized_size_geographic_msgs__msg__RouteSegment(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RouteSegment__ros_msg_type * ros_message = static_cast<const _RouteSegment__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: id
  current_alignment += get_serialized_size_unique_identifier_msgs__msg__UUID(
    &(ros_message->id), current_alignment);

  // Field name: start
  current_alignment += get_serialized_size_unique_identifier_msgs__msg__UUID(
    &(ros_message->start), current_alignment);

  // Field name: end
  current_alignment += get_serialized_size_unique_identifier_msgs__msg__UUID(
    &(ros_message->end), current_alignment);

  // Field name: props
  {
    size_t array_size = ros_message->props.size;
    auto array_ptr = ros_message->props.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_geographic_msgs__msg__KeyValue(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
size_t max_serialized_size_geographic_msgs__msg__RouteSegment(
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

  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: start
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: end
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: props
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geographic_msgs__msg__KeyValue(
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
    using DataType = geographic_msgs__msg__RouteSegment;
    is_plain =
      (
      offsetof(DataType, props) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
bool cdr_serialize_key_geographic_msgs__msg__RouteSegment(
  const geographic_msgs__msg__RouteSegment * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: id
  {
    cdr_serialize_key_unique_identifier_msgs__msg__UUID(
      &ros_message->id, cdr);
  }

  // Field name: start
  {
    cdr_serialize_key_unique_identifier_msgs__msg__UUID(
      &ros_message->start, cdr);
  }

  // Field name: end
  {
    cdr_serialize_key_unique_identifier_msgs__msg__UUID(
      &ros_message->end, cdr);
  }

  // Field name: props
  {
    size_t size = ros_message->props.size;
    auto array_ptr = ros_message->props.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_geographic_msgs__msg__KeyValue(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
size_t get_serialized_size_key_geographic_msgs__msg__RouteSegment(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RouteSegment__ros_msg_type * ros_message = static_cast<const _RouteSegment__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: id
  current_alignment += get_serialized_size_key_unique_identifier_msgs__msg__UUID(
    &(ros_message->id), current_alignment);

  // Field name: start
  current_alignment += get_serialized_size_key_unique_identifier_msgs__msg__UUID(
    &(ros_message->start), current_alignment);

  // Field name: end
  current_alignment += get_serialized_size_key_unique_identifier_msgs__msg__UUID(
    &(ros_message->end), current_alignment);

  // Field name: props
  {
    size_t array_size = ros_message->props.size;
    auto array_ptr = ros_message->props.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_geographic_msgs__msg__KeyValue(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_geographic_msgs
size_t max_serialized_size_key_geographic_msgs__msg__RouteSegment(
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
  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: start
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: end
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: props
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geographic_msgs__msg__KeyValue(
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
    using DataType = geographic_msgs__msg__RouteSegment;
    is_plain =
      (
      offsetof(DataType, props) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _RouteSegment__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const geographic_msgs__msg__RouteSegment * ros_message = static_cast<const geographic_msgs__msg__RouteSegment *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_geographic_msgs__msg__RouteSegment(ros_message, cdr);
}

static bool _RouteSegment__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  geographic_msgs__msg__RouteSegment * ros_message = static_cast<geographic_msgs__msg__RouteSegment *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_geographic_msgs__msg__RouteSegment(cdr, ros_message);
}

static uint32_t _RouteSegment__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_geographic_msgs__msg__RouteSegment(
      untyped_ros_message, 0));
}

static size_t _RouteSegment__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_geographic_msgs__msg__RouteSegment(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RouteSegment = {
  "geographic_msgs::msg",
  "RouteSegment",
  _RouteSegment__cdr_serialize,
  _RouteSegment__cdr_deserialize,
  _RouteSegment__get_serialized_size,
  _RouteSegment__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _RouteSegment__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RouteSegment,
  get_message_typesupport_handle_function,
  &geographic_msgs__msg__RouteSegment__get_type_hash,
  &geographic_msgs__msg__RouteSegment__get_type_description,
  &geographic_msgs__msg__RouteSegment__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geographic_msgs, msg, RouteSegment)() {
  return &_RouteSegment__type_support;
}

#if defined(__cplusplus)
}
#endif
