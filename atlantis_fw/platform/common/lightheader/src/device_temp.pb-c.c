/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: device_temp.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "device_temp.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__device_temp__init
                     (Ltpb__DeviceTemp         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__DeviceTemp init_value = LTPB__DEVICE_TEMP__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__device_temp__get_packed_size
                     (const Ltpb__DeviceTemp *message)
{
  assert(message->base.descriptor == &ltpb__device_temp__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__device_temp__pack
                     (const Ltpb__DeviceTemp *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__device_temp__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__device_temp__pack_to_buffer
                     (const Ltpb__DeviceTemp *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__device_temp__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__DeviceTemp *
       ltpb__device_temp__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__DeviceTemp *)
     protobuf_c_message_unpack (&ltpb__device_temp__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__device_temp__free_unpacked
                     (Ltpb__DeviceTemp *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__device_temp__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__device_temp__field_descriptors[5] =
{
  {
    "sensor_1",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_SINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__DeviceTemp, sensor_1),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_2",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_SINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__DeviceTemp, sensor_2),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_3",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_SINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__DeviceTemp, sensor_3),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_4",
    4,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_SINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__DeviceTemp, sensor_4),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "flex_sensor_1",
    5,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_SINT32,
    offsetof(Ltpb__DeviceTemp, has_flex_sensor_1),
    offsetof(Ltpb__DeviceTemp, flex_sensor_1),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__device_temp__field_indices_by_name[] = {
  4,   /* field[4] = flex_sensor_1 */
  0,   /* field[0] = sensor_1 */
  1,   /* field[1] = sensor_2 */
  2,   /* field[2] = sensor_3 */
  3,   /* field[3] = sensor_4 */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__device_temp__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 5 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__device_temp__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.DeviceTemp",
  "DeviceTemp",
  "Ltpb__DeviceTemp",
  "ltpb",
  sizeof(Ltpb__DeviceTemp),
  5,
  ltpb__device_temp__field_descriptors,
  ltpb__device_temp__field_indices_by_name,
  1,  ltpb__device_temp__number_ranges,
  (ProtobufCMessageInit) ltpb__device_temp__init,
  NULL,NULL,NULL    /* reserved[123] */
};
