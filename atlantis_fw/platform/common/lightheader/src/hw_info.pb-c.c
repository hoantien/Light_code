/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: hw_info.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "hw_info.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__camera_module_hw_info__init
                     (Ltpb__CameraModuleHwInfo         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__CameraModuleHwInfo init_value = LTPB__CAMERA_MODULE_HW_INFO__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__camera_module_hw_info__get_packed_size
                     (const Ltpb__CameraModuleHwInfo *message)
{
  assert(message->base.descriptor == &ltpb__camera_module_hw_info__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__camera_module_hw_info__pack
                     (const Ltpb__CameraModuleHwInfo *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__camera_module_hw_info__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__camera_module_hw_info__pack_to_buffer
                     (const Ltpb__CameraModuleHwInfo *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__camera_module_hw_info__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__CameraModuleHwInfo *
       ltpb__camera_module_hw_info__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__CameraModuleHwInfo *)
     protobuf_c_message_unpack (&ltpb__camera_module_hw_info__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__camera_module_hw_info__free_unpacked
                     (Ltpb__CameraModuleHwInfo *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__camera_module_hw_info__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_FUNC void   ltpb__hw_info__init
                     (Ltpb__HwInfo         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__HwInfo init_value = LTPB__HW_INFO__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__hw_info__get_packed_size
                     (const Ltpb__HwInfo *message)
{
  assert(message->base.descriptor == &ltpb__hw_info__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__hw_info__pack
                     (const Ltpb__HwInfo *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__hw_info__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__hw_info__pack_to_buffer
                     (const Ltpb__HwInfo *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__hw_info__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__HwInfo *
       ltpb__hw_info__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__HwInfo *)
     protobuf_c_message_unpack (&ltpb__hw_info__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__hw_info__free_unpacked
                     (Ltpb__HwInfo *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__hw_info__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__camera_module_hw_info__lens_type__enum_values_by_number[4] =
{
  { "LENS_UNKNOWN", "LTPB__CAMERA_MODULE_HW_INFO__LENS_TYPE__LENS_UNKNOWN", 0 },
  { "LENS_SHOWIN", "LTPB__CAMERA_MODULE_HW_INFO__LENS_TYPE__LENS_SHOWIN", 1 },
  { "LENS_LARGAN", "LTPB__CAMERA_MODULE_HW_INFO__LENS_TYPE__LENS_LARGAN", 2 },
  { "LENS_SUNNY", "LTPB__CAMERA_MODULE_HW_INFO__LENS_TYPE__LENS_SUNNY", 3 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module_hw_info__lens_type__value_ranges[] = {
{0, 0},{0, 4}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__camera_module_hw_info__lens_type__enum_values_by_name[4] =
{
  { "LENS_LARGAN", 2 },
  { "LENS_SHOWIN", 1 },
  { "LENS_SUNNY", 3 },
  { "LENS_UNKNOWN", 0 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__camera_module_hw_info__lens_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.CameraModuleHwInfo.LensType",
  "LensType",
  "Ltpb__CameraModuleHwInfo__LensType",
  "ltpb",
  4,
  ltpb__camera_module_hw_info__lens_type__enum_values_by_number,
  4,
  ltpb__camera_module_hw_info__lens_type__enum_values_by_name,
  1,
  ltpb__camera_module_hw_info__lens_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__camera_module_hw_info__lens_actuator_type__enum_values_by_number[3] =
{
  { "LENS_ACTUATOR_UNKNOWN", "LTPB__CAMERA_MODULE_HW_INFO__LENS_ACTUATOR_TYPE__LENS_ACTUATOR_UNKNOWN", 0 },
  { "LENS_ACTUATOR_SHICOH", "LTPB__CAMERA_MODULE_HW_INFO__LENS_ACTUATOR_TYPE__LENS_ACTUATOR_SHICOH", 1 },
  { "LENS_ACTUATOR_PZT", "LTPB__CAMERA_MODULE_HW_INFO__LENS_ACTUATOR_TYPE__LENS_ACTUATOR_PZT", 2 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module_hw_info__lens_actuator_type__value_ranges[] = {
{0, 0},{0, 3}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__camera_module_hw_info__lens_actuator_type__enum_values_by_name[3] =
{
  { "LENS_ACTUATOR_PZT", 2 },
  { "LENS_ACTUATOR_SHICOH", 1 },
  { "LENS_ACTUATOR_UNKNOWN", 0 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__camera_module_hw_info__lens_actuator_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.CameraModuleHwInfo.LensActuatorType",
  "LensActuatorType",
  "Ltpb__CameraModuleHwInfo__LensActuatorType",
  "ltpb",
  3,
  ltpb__camera_module_hw_info__lens_actuator_type__enum_values_by_number,
  3,
  ltpb__camera_module_hw_info__lens_actuator_type__enum_values_by_name,
  1,
  ltpb__camera_module_hw_info__lens_actuator_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__camera_module_hw_info__mirror_actuator_type__enum_values_by_number[2] =
{
  { "MIRROR_ACTUATOR_UNKNOWN", "LTPB__CAMERA_MODULE_HW_INFO__MIRROR_ACTUATOR_TYPE__MIRROR_ACTUATOR_UNKNOWN", 0 },
  { "MIRROR_ACTUATOR_PZT", "LTPB__CAMERA_MODULE_HW_INFO__MIRROR_ACTUATOR_TYPE__MIRROR_ACTUATOR_PZT", 1 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module_hw_info__mirror_actuator_type__value_ranges[] = {
{0, 0},{0, 2}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__camera_module_hw_info__mirror_actuator_type__enum_values_by_name[2] =
{
  { "MIRROR_ACTUATOR_PZT", 1 },
  { "MIRROR_ACTUATOR_UNKNOWN", 0 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__camera_module_hw_info__mirror_actuator_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.CameraModuleHwInfo.MirrorActuatorType",
  "MirrorActuatorType",
  "Ltpb__CameraModuleHwInfo__MirrorActuatorType",
  "ltpb",
  2,
  ltpb__camera_module_hw_info__mirror_actuator_type__enum_values_by_number,
  2,
  ltpb__camera_module_hw_info__mirror_actuator_type__enum_values_by_name,
  1,
  ltpb__camera_module_hw_info__mirror_actuator_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__camera_module_hw_info__mirror_type__enum_values_by_number[3] =
{
  { "MIRROR_UNKNOWN", "LTPB__CAMERA_MODULE_HW_INFO__MIRROR_TYPE__MIRROR_UNKNOWN", 0 },
  { "MIRROR_DIELECTRIC_SNX", "LTPB__CAMERA_MODULE_HW_INFO__MIRROR_TYPE__MIRROR_DIELECTRIC_SNX", 1 },
  { "MIRROR_SILVER_ZUISHO", "LTPB__CAMERA_MODULE_HW_INFO__MIRROR_TYPE__MIRROR_SILVER_ZUISHO", 2 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module_hw_info__mirror_type__value_ranges[] = {
{0, 0},{0, 3}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__camera_module_hw_info__mirror_type__enum_values_by_name[3] =
{
  { "MIRROR_DIELECTRIC_SNX", 1 },
  { "MIRROR_SILVER_ZUISHO", 2 },
  { "MIRROR_UNKNOWN", 0 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__camera_module_hw_info__mirror_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.CameraModuleHwInfo.MirrorType",
  "MirrorType",
  "Ltpb__CameraModuleHwInfo__MirrorType",
  "ltpb",
  3,
  ltpb__camera_module_hw_info__mirror_type__enum_values_by_number,
  3,
  ltpb__camera_module_hw_info__mirror_type__enum_values_by_name,
  1,
  ltpb__camera_module_hw_info__mirror_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const Ltpb__CameraModuleHwInfo__LensType ltpb__camera_module_hw_info__lens__default_value = LTPB__CAMERA_MODULE_HW_INFO__LENS_TYPE__LENS_UNKNOWN;
LIGHT_HEADER_CONST static const Ltpb__CameraModuleHwInfo__MirrorActuatorType ltpb__camera_module_hw_info__mirror_actuator__default_value = LTPB__CAMERA_MODULE_HW_INFO__MIRROR_ACTUATOR_TYPE__MIRROR_ACTUATOR_UNKNOWN;
LIGHT_HEADER_CONST static const Ltpb__CameraModuleHwInfo__MirrorType ltpb__camera_module_hw_info__mirror__default_value = LTPB__CAMERA_MODULE_HW_INFO__MIRROR_TYPE__MIRROR_UNKNOWN;
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__camera_module_hw_info__field_descriptors[5] =
{
  {
    "id",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModuleHwInfo, id),
    &ltpb__camera_id__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(Ltpb__CameraModuleHwInfo, sensor),
    &ltpb__sensor_type__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "lens",
    3,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_ENUM,
    offsetof(Ltpb__CameraModuleHwInfo, has_lens),
    offsetof(Ltpb__CameraModuleHwInfo, lens),
    &ltpb__camera_module_hw_info__lens_type__descriptor,
    &ltpb__camera_module_hw_info__lens__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mirror_actuator",
    4,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_ENUM,
    offsetof(Ltpb__CameraModuleHwInfo, has_mirror_actuator),
    offsetof(Ltpb__CameraModuleHwInfo, mirror_actuator),
    &ltpb__camera_module_hw_info__mirror_actuator_type__descriptor,
    &ltpb__camera_module_hw_info__mirror_actuator__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mirror",
    5,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_ENUM,
    offsetof(Ltpb__CameraModuleHwInfo, has_mirror),
    offsetof(Ltpb__CameraModuleHwInfo, mirror),
    &ltpb__camera_module_hw_info__mirror_type__descriptor,
    &ltpb__camera_module_hw_info__mirror__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__camera_module_hw_info__field_indices_by_name[] = {
  0,   /* field[0] = id */
  2,   /* field[2] = lens */
  4,   /* field[4] = mirror */
  3,   /* field[3] = mirror_actuator */
  1,   /* field[1] = sensor */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__camera_module_hw_info__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 5 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__camera_module_hw_info__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.CameraModuleHwInfo",
  "CameraModuleHwInfo",
  "Ltpb__CameraModuleHwInfo",
  "ltpb",
  sizeof(Ltpb__CameraModuleHwInfo),
  5,
  ltpb__camera_module_hw_info__field_descriptors,
  ltpb__camera_module_hw_info__field_indices_by_name,
  1,  ltpb__camera_module_hw_info__number_ranges,
  (ProtobufCMessageInit) ltpb__camera_module_hw_info__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__hw_info__flash_type__enum_values_by_number[2] =
{
  { "FLASH_UNKNOWN", "LTPB__HW_INFO__FLASH_TYPE__FLASH_UNKNOWN", 0 },
  { "FLASH_OSRAM_CBLPM1", "LTPB__HW_INFO__FLASH_TYPE__FLASH_OSRAM_CBLPM1", 1 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__hw_info__flash_type__value_ranges[] = {
{0, 0},{0, 2}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__hw_info__flash_type__enum_values_by_name[2] =
{
  { "FLASH_OSRAM_CBLPM1", 1 },
  { "FLASH_UNKNOWN", 0 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__hw_info__flash_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.HwInfo.FlashType",
  "FlashType",
  "Ltpb__HwInfo__FlashType",
  "ltpb",
  2,
  ltpb__hw_info__flash_type__enum_values_by_number,
  2,
  ltpb__hw_info__flash_type__enum_values_by_name,
  1,
  ltpb__hw_info__flash_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__hw_info__to_ftype__enum_values_by_number[2] =
{
  { "TOF_UNKNOWN", "LTPB__HW_INFO__TO_FTYPE__TOF_UNKNOWN", 0 },
  { "TOF_STMICRO_VL53L0", "LTPB__HW_INFO__TO_FTYPE__TOF_STMICRO_VL53L0", 1 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__hw_info__to_ftype__value_ranges[] = {
{0, 0},{0, 2}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__hw_info__to_ftype__enum_values_by_name[2] =
{
  { "TOF_STMICRO_VL53L0", 1 },
  { "TOF_UNKNOWN", 0 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__hw_info__to_ftype__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.HwInfo.ToFType",
  "ToFType",
  "Ltpb__HwInfo__ToFType",
  "ltpb",
  2,
  ltpb__hw_info__to_ftype__enum_values_by_number,
  2,
  ltpb__hw_info__to_ftype__enum_values_by_name,
  1,
  ltpb__hw_info__to_ftype__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const ProtobufCEnumValue ltpb__hw_info__model_type__enum_values_by_number[3] =
{
  { "MODEL_P1", "LTPB__HW_INFO__MODEL_TYPE__MODEL_P1", 0 },
  { "MODEL_P1_1", "LTPB__HW_INFO__MODEL_TYPE__MODEL_P1_1", 1 },
  { "MODEL_P2", "LTPB__HW_INFO__MODEL_TYPE__MODEL_P2", 2 },
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__hw_info__model_type__value_ranges[] = {
{0, 0},{0, 3}
};
LIGHT_HEADER_CONST static const ProtobufCEnumValueIndex ltpb__hw_info__model_type__enum_values_by_name[3] =
{
  { "MODEL_P1", 0 },
  { "MODEL_P1_1", 1 },
  { "MODEL_P2", 2 },
};
LIGHT_HEADER_CONST const ProtobufCEnumDescriptor ltpb__hw_info__model_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ltpb.HwInfo.ModelType",
  "ModelType",
  "Ltpb__HwInfo__ModelType",
  "ltpb",
  3,
  ltpb__hw_info__model_type__enum_values_by_number,
  3,
  ltpb__hw_info__model_type__enum_values_by_name,
  1,
  ltpb__hw_info__model_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
LIGHT_HEADER_CONST static const Ltpb__HwInfo__FlashType ltpb__hw_info__flash__default_value = LTPB__HW_INFO__FLASH_TYPE__FLASH_UNKNOWN;
LIGHT_HEADER_CONST static const Ltpb__HwInfo__ToFType ltpb__hw_info__tof__default_value = LTPB__HW_INFO__TO_FTYPE__TOF_UNKNOWN;
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__hw_info__field_descriptors[4] =
{
  {
    "camera",
    1,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(Ltpb__HwInfo, n_camera),
    offsetof(Ltpb__HwInfo, camera),
    &ltpb__camera_module_hw_info__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "flash",
    2,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_ENUM,
    offsetof(Ltpb__HwInfo, has_flash),
    offsetof(Ltpb__HwInfo, flash),
    &ltpb__hw_info__flash_type__descriptor,
    &ltpb__hw_info__flash__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "tof",
    3,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_ENUM,
    offsetof(Ltpb__HwInfo, has_tof),
    offsetof(Ltpb__HwInfo, tof),
    &ltpb__hw_info__to_ftype__descriptor,
    &ltpb__hw_info__tof__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "model",
    4,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_ENUM,
    offsetof(Ltpb__HwInfo, has_model),
    offsetof(Ltpb__HwInfo, model),
    &ltpb__hw_info__model_type__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__hw_info__field_indices_by_name[] = {
  0,   /* field[0] = camera */
  1,   /* field[1] = flash */
  3,   /* field[3] = model */
  2,   /* field[2] = tof */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__hw_info__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 4 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__hw_info__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.HwInfo",
  "HwInfo",
  "Ltpb__HwInfo",
  "ltpb",
  sizeof(Ltpb__HwInfo),
  4,
  ltpb__hw_info__field_descriptors,
  ltpb__hw_info__field_indices_by_name,
  1,  ltpb__hw_info__number_ranges,
  (ProtobufCMessageInit) ltpb__hw_info__init,
  NULL,NULL,NULL    /* reserved[123] */
};
