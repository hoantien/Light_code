/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: matrix4x4f.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "matrix4x4f.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__matrix4x4_f__init
                     (Ltpb__Matrix4x4F         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__Matrix4x4F init_value = LTPB__MATRIX4X4_F__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__matrix4x4_f__get_packed_size
                     (const Ltpb__Matrix4x4F *message)
{
  assert(message->base.descriptor == &ltpb__matrix4x4_f__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__matrix4x4_f__pack
                     (const Ltpb__Matrix4x4F *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__matrix4x4_f__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__matrix4x4_f__pack_to_buffer
                     (const Ltpb__Matrix4x4F *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__matrix4x4_f__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__Matrix4x4F *
       ltpb__matrix4x4_f__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__Matrix4x4F *)
     protobuf_c_message_unpack (&ltpb__matrix4x4_f__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__matrix4x4_f__free_unpacked
                     (Ltpb__Matrix4x4F *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__matrix4x4_f__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__matrix4x4_f__field_descriptors[16] =
{
  {
    "x00",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x00),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x01",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x01),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x02",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x02),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x03",
    4,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x03),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x10",
    5,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x10),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x11",
    6,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x11),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x12",
    7,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x12),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x13",
    8,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x13),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x20",
    9,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x20),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x21",
    10,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x21),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x22",
    11,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x22),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x23",
    12,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x23),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x30",
    13,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x30),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x31",
    14,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x31),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x32",
    15,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x32),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "x33",
    16,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Matrix4x4F, x33),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__matrix4x4_f__field_indices_by_name[] = {
  0,   /* field[0] = x00 */
  1,   /* field[1] = x01 */
  2,   /* field[2] = x02 */
  3,   /* field[3] = x03 */
  4,   /* field[4] = x10 */
  5,   /* field[5] = x11 */
  6,   /* field[6] = x12 */
  7,   /* field[7] = x13 */
  8,   /* field[8] = x20 */
  9,   /* field[9] = x21 */
  10,   /* field[10] = x22 */
  11,   /* field[11] = x23 */
  12,   /* field[12] = x30 */
  13,   /* field[13] = x31 */
  14,   /* field[14] = x32 */
  15,   /* field[15] = x33 */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__matrix4x4_f__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 16 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__matrix4x4_f__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.Matrix4x4F",
  "Matrix4x4F",
  "Ltpb__Matrix4x4F",
  "ltpb",
  sizeof(Ltpb__Matrix4x4F),
  16,
  ltpb__matrix4x4_f__field_descriptors,
  ltpb__matrix4x4_f__field_indices_by_name,
  1,  ltpb__matrix4x4_f__number_ranges,
  (ProtobufCMessageInit) ltpb__matrix4x4_f__init,
  NULL,NULL,NULL    /* reserved[123] */
};