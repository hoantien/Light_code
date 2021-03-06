/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: camera_module.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "camera_module.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__camera_module__afinfo__init
                     (Ltpb__CameraModule__AFInfo         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__CameraModule__AFInfo init_value = LTPB__CAMERA_MODULE__AFINFO__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__camera_module__surface__init
                     (Ltpb__CameraModule__Surface         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__CameraModule__Surface init_value = LTPB__CAMERA_MODULE__SURFACE__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__camera_module__init
                     (Ltpb__CameraModule         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__CameraModule init_value = LTPB__CAMERA_MODULE__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__camera_module__get_packed_size
                     (const Ltpb__CameraModule *message)
{
  assert(message->base.descriptor == &ltpb__camera_module__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__camera_module__pack
                     (const Ltpb__CameraModule *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__camera_module__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__camera_module__pack_to_buffer
                     (const Ltpb__CameraModule *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__camera_module__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__CameraModule *
       ltpb__camera_module__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__CameraModule *)
     protobuf_c_message_unpack (&ltpb__camera_module__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__camera_module__free_unpacked
                     (Ltpb__CameraModule *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__camera_module__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__camera_module__afinfo__afmode__enum_values_by_number[1] =
{
  { "AUTO", "LTPB__CAMERA_MODULE__AFINFO__AFMODE__AUTO", 0 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module__afinfo__afmode__value_ranges[] = {
{0, 0},{0, 1}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__camera_module__afinfo__afmode__enum_values_by_name[1] =
{
  { "AUTO", 0 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__camera_module__afinfo__afmode__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.CameraModule.AFInfo.AFMode",
  "AFMode",
  "Ltpb__CameraModule__AFInfo__AFMode",
  "ltpb",
  1,
  ltpb__camera_module__afinfo__afmode__enum_values_by_number,
  1,
  ltpb__camera_module__afinfo__afmode__enum_values_by_name,
  1,
  ltpb__camera_module__afinfo__afmode__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__camera_module__afinfo__field_descriptors[3] =
{
  {
    "mode",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule__AFInfo, mode),
    &ltpb__camera_module__afinfo__afmode__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "roi_center",
    2,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule__AFInfo, roi_center),
    &ltpb__point2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "roi_estimated_disparity",
    3,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__CameraModule__AFInfo, has_roi_estimated_disparity),
    offsetof(Ltpb__CameraModule__AFInfo, roi_estimated_disparity),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__camera_module__afinfo__field_indices_by_name[] = {
  0,   /* field[0] = mode */
  1,   /* field[1] = roi_center */
  2,   /* field[2] = roi_estimated_disparity */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module__afinfo__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 3 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__camera_module__afinfo__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.CameraModule.AFInfo",
  "AFInfo",
  "Ltpb__CameraModule__AFInfo",
  "ltpb",
  sizeof(Ltpb__CameraModule__AFInfo),
  3,
  ltpb__camera_module__afinfo__field_descriptors,
  ltpb__camera_module__afinfo__field_indices_by_name,
  1,  ltpb__camera_module__afinfo__number_ranges,
  (ProtobufCMessageInit) ltpb__camera_module__afinfo__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__camera_module__surface__format_type__enum_values_by_number[10] =
{
  { "RGB_UNPACKED_8BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RGB_UNPACKED_8BPP", 0 },
  { "RGB_UNPACKED_16BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RGB_UNPACKED_16BPP", 1 },
  { "RGB_PACKED_10BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RGB_PACKED_10BPP", 2 },
  { "RGB_PACKED_12BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RGB_PACKED_12BPP", 3 },
  { "RGB_PACKED_14BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RGB_PACKED_14BPP", 4 },
  { "RAW_UNPACKED_8BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RAW_UNPACKED_8BPP", 5 },
  { "RAW_UNPACKED_16BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RAW_UNPACKED_16BPP", 6 },
  { "RAW_PACKED_10BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RAW_PACKED_10BPP", 7 },
  { "RAW_PACKED_12BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RAW_PACKED_12BPP", 8 },
  { "RAW_PACKED_14BPP", "LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RAW_PACKED_14BPP", 9 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module__surface__format_type__value_ranges[] = {
{0, 0},{0, 10}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__camera_module__surface__format_type__enum_values_by_name[10] =
{
  { "RAW_PACKED_10BPP", 7 },
  { "RAW_PACKED_12BPP", 8 },
  { "RAW_PACKED_14BPP", 9 },
  { "RAW_UNPACKED_16BPP", 6 },
  { "RAW_UNPACKED_8BPP", 5 },
  { "RGB_PACKED_10BPP", 2 },
  { "RGB_PACKED_12BPP", 3 },
  { "RGB_PACKED_14BPP", 4 },
  { "RGB_UNPACKED_16BPP", 1 },
  { "RGB_UNPACKED_8BPP", 0 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__camera_module__surface__format_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.CameraModule.Surface.FormatType",
  "FormatType",
  "Ltpb__CameraModule__Surface__FormatType",
  "ltpb",
  10,
  ltpb__camera_module__surface__format_type__enum_values_by_number,
  10,
  ltpb__camera_module__surface__format_type__enum_values_by_name,
  1,
  ltpb__camera_module__surface__format_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__camera_module__surface__field_descriptors[6] =
{
  {
    "start",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule__Surface, start),
    &ltpb__point2_i__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "size",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule__Surface, size),
    &ltpb__point2_i__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "format",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule__Surface, format),
    &ltpb__camera_module__surface__format_type__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "row_stride",
    4,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule__Surface, row_stride),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "data_offset",
    5,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT64,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule__Surface, data_offset),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "data_scale",
    6,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule__Surface, data_scale),
    &ltpb__point2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__camera_module__surface__field_indices_by_name[] = {
  4,   /* field[4] = data_offset */
  5,   /* field[5] = data_scale */
  2,   /* field[2] = format */
  3,   /* field[3] = row_stride */
  1,   /* field[1] = size */
  0,   /* field[0] = start */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module__surface__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 6 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__camera_module__surface__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.CameraModule.Surface",
  "Surface",
  "Ltpb__CameraModule__Surface",
  "ltpb",
  sizeof(Ltpb__CameraModule__Surface),
  6,
  ltpb__camera_module__surface__field_descriptors,
  ltpb__camera_module__surface__field_indices_by_name,
  1,  ltpb__camera_module__surface__number_ranges,
  (ProtobufCMessageInit) ltpb__camera_module__surface__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const int32_t ltpb__camera_module__mirror_position__default_value = 0;
LIGHT_HEADER_CONST static const protobuf_c_boolean ltpb__camera_module__sensor_is_horizontal_flip__default_value = 0;
LIGHT_HEADER_CONST static const protobuf_c_boolean ltpb__camera_module__sensor_is_vertical_flip__default_value = 0;
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__camera_module__field_descriptors[14] =
{
  {
    "af_info",
    1,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, af_info),
    &ltpb__camera_module__afinfo__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "id",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, id),
    &ltpb__camera_id__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "is_enabled",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_BOOL,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, is_enabled),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mirror_position",
    4,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(Ltpb__CameraModule, has_mirror_position),
    offsetof(Ltpb__CameraModule, mirror_position),
    NULL,
    &ltpb__camera_module__mirror_position__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "lens_position",
    5,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_INT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, lens_position),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_total_gain",
    6,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, sensor_total_gain),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_analog_gain",
    7,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, sensor_analog_gain),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_exposure",
    8,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT64,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, sensor_exposure),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_data_surface",
    9,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, sensor_data_surface),
    &ltpb__camera_module__surface__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_temparature",
    10,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_SINT32,
    offsetof(Ltpb__CameraModule, has_sensor_temparature),
    offsetof(Ltpb__CameraModule, sensor_temparature),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_is_horizontal_flip",
    11,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_BOOL,
    offsetof(Ltpb__CameraModule, has_sensor_is_horizontal_flip),
    offsetof(Ltpb__CameraModule, sensor_is_horizontal_flip),
    NULL,
    &ltpb__camera_module__sensor_is_horizontal_flip__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_is_vertical_flip",
    12,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_BOOL,
    offsetof(Ltpb__CameraModule, has_sensor_is_vertical_flip),
    offsetof(Ltpb__CameraModule, sensor_is_vertical_flip),
    NULL,
    &ltpb__camera_module__sensor_is_vertical_flip__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_bayer_red_override",
    13,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModule, sensor_bayer_red_override),
    &ltpb__point2_i__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_digital_gain",
    14,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__CameraModule, has_sensor_digital_gain),
    offsetof(Ltpb__CameraModule, sensor_digital_gain),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__camera_module__field_indices_by_name[] = {
  0,   /* field[0] = af_info */
  1,   /* field[1] = id */
  2,   /* field[2] = is_enabled */
  4,   /* field[4] = lens_position */
  3,   /* field[3] = mirror_position */
  6,   /* field[6] = sensor_analog_gain */
  12,   /* field[12] = sensor_bayer_red_override */
  8,   /* field[8] = sensor_data_surface */
  13,   /* field[13] = sensor_digital_gain */
  7,   /* field[7] = sensor_exposure */
  10,   /* field[10] = sensor_is_horizontal_flip */
  11,   /* field[11] = sensor_is_vertical_flip */
  9,   /* field[9] = sensor_temparature */
  5,   /* field[5] = sensor_total_gain */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 14 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__camera_module__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.CameraModule",
  "CameraModule",
  "Ltpb__CameraModule",
  "ltpb",
  sizeof(Ltpb__CameraModule),
  14,
  ltpb__camera_module__field_descriptors,
  ltpb__camera_module__field_indices_by_name,
  1,  ltpb__camera_module__number_ranges,
  (ProtobufCMessageInit) ltpb__camera_module__init,
  NULL,NULL,NULL    /* reserved[123] */
};
