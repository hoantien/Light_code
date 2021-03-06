/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: mirror_system.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "mirror_system.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__mirror_system__init
                     (Ltpb__MirrorSystem         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__MirrorSystem init_value = LTPB__MIRROR_SYSTEM__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__mirror_system__get_packed_size
                     (const Ltpb__MirrorSystem *message)
{
  assert(message->base.descriptor == &ltpb__mirror_system__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__mirror_system__pack
                     (const Ltpb__MirrorSystem *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__mirror_system__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__mirror_system__pack_to_buffer
                     (const Ltpb__MirrorSystem *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__mirror_system__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__MirrorSystem *
       ltpb__mirror_system__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__MirrorSystem *)
     protobuf_c_message_unpack (&ltpb__mirror_system__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__mirror_system__free_unpacked
                     (Ltpb__MirrorSystem *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__mirror_system__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_FUNC void   ltpb__mirror_actuator_mapping__quadratic_model__init
                     (Ltpb__MirrorActuatorMapping__QuadraticModel         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__MirrorActuatorMapping__QuadraticModel init_value = LTPB__MIRROR_ACTUATOR_MAPPING__QUADRATIC_MODEL__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__mirror_actuator_mapping__actuator_angle_pair__init
                     (Ltpb__MirrorActuatorMapping__ActuatorAnglePair         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__MirrorActuatorMapping__ActuatorAnglePair init_value = LTPB__MIRROR_ACTUATOR_MAPPING__ACTUATOR_ANGLE_PAIR__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__mirror_actuator_mapping__init
                     (Ltpb__MirrorActuatorMapping         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__MirrorActuatorMapping init_value = LTPB__MIRROR_ACTUATOR_MAPPING__INIT;
  *message = init_value;
}

LIGHT_HEADER_FUNC size_t ltpb__mirror_actuator_mapping__get_packed_size
                     (const Ltpb__MirrorActuatorMapping *message)
{
  assert(message->base.descriptor == &ltpb__mirror_actuator_mapping__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__mirror_actuator_mapping__pack
                     (const Ltpb__MirrorActuatorMapping *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__mirror_actuator_mapping__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__mirror_actuator_mapping__pack_to_buffer
                     (const Ltpb__MirrorActuatorMapping *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__mirror_actuator_mapping__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__MirrorActuatorMapping *
       ltpb__mirror_actuator_mapping__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__MirrorActuatorMapping *)
     protobuf_c_message_unpack (&ltpb__mirror_actuator_mapping__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__mirror_actuator_mapping__free_unpacked
                     (Ltpb__MirrorActuatorMapping *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__mirror_actuator_mapping__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__mirror_system__field_descriptors[9] =
{
  {
    "real_camera_location",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorSystem, real_camera_location),
    &ltpb__point3_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "real_camera_orientation",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorSystem, real_camera_orientation),
    &ltpb__matrix3x3_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "rotation_axis",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorSystem, rotation_axis),
    &ltpb__point3_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "point_on_rotation_axis",
    4,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorSystem, point_on_rotation_axis),
    &ltpb__point3_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "distance_mirror_plane_to_point_on_rotation_axis",
    5,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorSystem, distance_mirror_plane_to_point_on_rotation_axis),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mirror_normal_at_zero_degrees",
    6,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorSystem, mirror_normal_at_zero_degrees),
    &ltpb__point3_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "flip_img_around_x",
    7,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_BOOL,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorSystem, flip_img_around_x),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mirror_angle_range",
    8,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorSystem, mirror_angle_range),
    &ltpb__range2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "reprojection_error",
    9,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__MirrorSystem, has_reprojection_error),
    offsetof(Ltpb__MirrorSystem, reprojection_error),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__mirror_system__field_indices_by_name[] = {
  4,   /* field[4] = distance_mirror_plane_to_point_on_rotation_axis */
  6,   /* field[6] = flip_img_around_x */
  7,   /* field[7] = mirror_angle_range */
  5,   /* field[5] = mirror_normal_at_zero_degrees */
  3,   /* field[3] = point_on_rotation_axis */
  0,   /* field[0] = real_camera_location */
  1,   /* field[1] = real_camera_orientation */
  8,   /* field[8] = reprojection_error */
  2,   /* field[2] = rotation_axis */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__mirror_system__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 9 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__mirror_system__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.MirrorSystem",
  "MirrorSystem",
  "Ltpb__MirrorSystem",
  "ltpb",
  sizeof(Ltpb__MirrorSystem),
  9,
  ltpb__mirror_system__field_descriptors,
  ltpb__mirror_system__field_indices_by_name,
  1,  ltpb__mirror_system__number_ranges,
  (ProtobufCMessageInit) ltpb__mirror_system__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__mirror_actuator_mapping__quadratic_model__field_descriptors[4] =
{
  {
    "use_rplus_for_left_segment",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_BOOL,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping__QuadraticModel, use_rplus_for_left_segment),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "use_rplus_for_right_segment",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_BOOL,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping__QuadraticModel, use_rplus_for_right_segment),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "inflection_value",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping__QuadraticModel, inflection_value),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "model_coeffs",
    4,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__MirrorActuatorMapping__QuadraticModel, n_model_coeffs),
    offsetof(Ltpb__MirrorActuatorMapping__QuadraticModel, model_coeffs),
    NULL,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_PACKED,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__mirror_actuator_mapping__quadratic_model__field_indices_by_name[] = {
  2,   /* field[2] = inflection_value */
  3,   /* field[3] = model_coeffs */
  0,   /* field[0] = use_rplus_for_left_segment */
  1,   /* field[1] = use_rplus_for_right_segment */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__mirror_actuator_mapping__quadratic_model__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 4 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__mirror_actuator_mapping__quadratic_model__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.MirrorActuatorMapping.QuadraticModel",
  "QuadraticModel",
  "Ltpb__MirrorActuatorMapping__QuadraticModel",
  "ltpb",
  sizeof(Ltpb__MirrorActuatorMapping__QuadraticModel),
  4,
  ltpb__mirror_actuator_mapping__quadratic_model__field_descriptors,
  ltpb__mirror_actuator_mapping__quadratic_model__field_indices_by_name,
  1,  ltpb__mirror_actuator_mapping__quadratic_model__number_ranges,
  (ProtobufCMessageInit) ltpb__mirror_actuator_mapping__quadratic_model__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__mirror_actuator_mapping__actuator_angle_pair__field_descriptors[2] =
{
  {
    "hall_code",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_INT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping__ActuatorAnglePair, hall_code),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "angle",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping__ActuatorAnglePair, angle),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__mirror_actuator_mapping__actuator_angle_pair__field_indices_by_name[] = {
  1,   /* field[1] = angle */
  0,   /* field[0] = hall_code */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__mirror_actuator_mapping__actuator_angle_pair__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__mirror_actuator_mapping__actuator_angle_pair__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.MirrorActuatorMapping.ActuatorAnglePair",
  "ActuatorAnglePair",
  "Ltpb__MirrorActuatorMapping__ActuatorAnglePair",
  "ltpb",
  sizeof(Ltpb__MirrorActuatorMapping__ActuatorAnglePair),
  2,
  ltpb__mirror_actuator_mapping__actuator_angle_pair__field_descriptors,
  ltpb__mirror_actuator_mapping__actuator_angle_pair__field_indices_by_name,
  1,  ltpb__mirror_actuator_mapping__actuator_angle_pair__number_ranges,
  (ProtobufCMessageInit) ltpb__mirror_actuator_mapping__actuator_angle_pair__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__mirror_actuator_mapping__transformation_type__enum_values_by_number[2] =
{
  { "MEAN_STD_NORMALIZE", "LTPB__MIRROR_ACTUATOR_MAPPING__TRANSFORMATION_TYPE__MEAN_STD_NORMALIZE", 0 },
  { "TAN_HALF_THETA", "LTPB__MIRROR_ACTUATOR_MAPPING__TRANSFORMATION_TYPE__TAN_HALF_THETA", 1 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__mirror_actuator_mapping__transformation_type__value_ranges[] = {
{0, 0},{0, 2}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__mirror_actuator_mapping__transformation_type__enum_values_by_name[2] =
{
  { "MEAN_STD_NORMALIZE", 0 },
  { "TAN_HALF_THETA", 1 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__mirror_actuator_mapping__transformation_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.MirrorActuatorMapping.TransformationType",
  "TransformationType",
  "Ltpb__MirrorActuatorMapping__TransformationType",
  "ltpb",
  2,
  ltpb__mirror_actuator_mapping__transformation_type__enum_values_by_number,
  2,
  ltpb__mirror_actuator_mapping__transformation_type__enum_values_by_name,
  1,
  ltpb__mirror_actuator_mapping__transformation_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__mirror_actuator_mapping__field_descriptors[10] =
{
  {
    "transformation_type",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping, transformation_type),
    &ltpb__mirror_actuator_mapping__transformation_type__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "actuator_length_offset",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping, actuator_length_offset),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "actuator_length_scale",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping, actuator_length_scale),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mirror_angle_offset",
    4,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping, mirror_angle_offset),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mirror_angle_scale",
    5,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping, mirror_angle_scale),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "actuator_angle_pair_vec",
    6,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(Ltpb__MirrorActuatorMapping, n_actuator_angle_pair_vec),
    offsetof(Ltpb__MirrorActuatorMapping, actuator_angle_pair_vec),
    &ltpb__mirror_actuator_mapping__actuator_angle_pair__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "quadratic_model",
    7,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping, quadratic_model),
    &ltpb__mirror_actuator_mapping__quadratic_model__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "angle_to_hall_code_error",
    8,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__MirrorActuatorMapping, has_angle_to_hall_code_error),
    offsetof(Ltpb__MirrorActuatorMapping, angle_to_hall_code_error),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "hall_code_to_angle_error",
    9,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__MirrorActuatorMapping, has_hall_code_to_angle_error),
    offsetof(Ltpb__MirrorActuatorMapping, hall_code_to_angle_error),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "hall_code_range",
    10,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__MirrorActuatorMapping, hall_code_range),
    &ltpb__range2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__mirror_actuator_mapping__field_indices_by_name[] = {
  5,   /* field[5] = actuator_angle_pair_vec */
  1,   /* field[1] = actuator_length_offset */
  2,   /* field[2] = actuator_length_scale */
  7,   /* field[7] = angle_to_hall_code_error */
  9,   /* field[9] = hall_code_range */
  8,   /* field[8] = hall_code_to_angle_error */
  3,   /* field[3] = mirror_angle_offset */
  4,   /* field[4] = mirror_angle_scale */
  6,   /* field[6] = quadratic_model */
  0,   /* field[0] = transformation_type */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__mirror_actuator_mapping__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 10 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__mirror_actuator_mapping__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.MirrorActuatorMapping",
  "MirrorActuatorMapping",
  "Ltpb__MirrorActuatorMapping",
  "ltpb",
  sizeof(Ltpb__MirrorActuatorMapping),
  10,
  ltpb__mirror_actuator_mapping__field_descriptors,
  ltpb__mirror_actuator_mapping__field_indices_by_name,
  1,  ltpb__mirror_actuator_mapping__number_ranges,
  (ProtobufCMessageInit) ltpb__mirror_actuator_mapping__init,
  NULL,NULL,NULL    /* reserved[123] */
};
