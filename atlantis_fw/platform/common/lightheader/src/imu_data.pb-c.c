/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: imu_data.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "../include/imu_data.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__imudata__sample__init
                     (Ltpb__IMUData__Sample         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__IMUData__Sample init_value = LTPB__IMUDATA__SAMPLE__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__imudata__init
                     (Ltpb__IMUData         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__IMUData init_value = LTPB__IMUDATA__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__imudata__get_packed_size
                     (const Ltpb__IMUData *message)
{
  assert(message->base.descriptor == &ltpb__imudata__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__imudata__pack
                     (const Ltpb__IMUData *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__imudata__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__imudata__pack_to_buffer
                     (const Ltpb__IMUData *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__imudata__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__IMUData *
       ltpb__imudata__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__IMUData *)
     protobuf_c_message_unpack (&ltpb__imudata__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__imudata__free_unpacked
                     (Ltpb__IMUData *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__imudata__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__imudata__sample__field_descriptors[2] =
{
  {
    "row_idx",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__IMUData__Sample, row_idx),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "data",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__IMUData__Sample, data),
    &ltpb__point3_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__imudata__sample__field_indices_by_name[] = {
  1,   /* field[1] = data */
  0,   /* field[0] = row_idx */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__imudata__sample__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__imudata__sample__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.IMUData.Sample",
  "Sample",
  "Ltpb__IMUData__Sample",
  "ltpb",
  sizeof(Ltpb__IMUData__Sample),
  2,
  ltpb__imudata__sample__field_descriptors,
  ltpb__imudata__sample__field_indices_by_name,
  1,  ltpb__imudata__sample__number_ranges,
  (ProtobufCMessageInit) ltpb__imudata__sample__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__imudata__field_descriptors[2] =
{
  {
    "accelerometer",
    1,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(Ltpb__IMUData, n_accelerometer),
    offsetof(Ltpb__IMUData, accelerometer),
    &ltpb__imudata__sample__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "gyroscope",
    2,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(Ltpb__IMUData, n_gyroscope),
    offsetof(Ltpb__IMUData, gyroscope),
    &ltpb__imudata__sample__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__imudata__field_indices_by_name[] = {
  0,   /* field[0] = accelerometer */
  1,   /* field[1] = gyroscope */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__imudata__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__imudata__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.IMUData",
  "IMUData",
  "Ltpb__IMUData",
  "ltpb",
  sizeof(Ltpb__IMUData),
  2,
  ltpb__imudata__field_descriptors,
  ltpb__imudata__field_indices_by_name,
  1,  ltpb__imudata__number_ranges,
  (ProtobufCMessageInit) ltpb__imudata__init,
  NULL,NULL,NULL    /* reserved[123] */
};
