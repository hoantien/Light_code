/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: point2f.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "point2f.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__point2_f__init
                     (Ltpb__Point2F         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__Point2F init_value = LTPB__POINT2_F__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__point2_f__get_packed_size
                     (const Ltpb__Point2F *message)
{
  assert(message->base.descriptor == &ltpb__point2_f__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__point2_f__pack
                     (const Ltpb__Point2F *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__point2_f__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__point2_f__pack_to_buffer
                     (const Ltpb__Point2F *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__point2_f__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__Point2F *
       ltpb__point2_f__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__Point2F *)
     protobuf_c_message_unpack (&ltpb__point2_f__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__point2_f__free_unpacked
                     (Ltpb__Point2F *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__point2_f__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__point2_f__field_descriptors[2] =
{
  {
    "x",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Point2F, x),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "y",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Point2F, y),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__point2_f__field_indices_by_name[] = {
  0,   /* field[0] = x */
  1,   /* field[1] = y */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__point2_f__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__point2_f__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.Point2F",
  "Point2F",
  "Ltpb__Point2F",
  "ltpb",
  sizeof(Ltpb__Point2F),
  2,
  ltpb__point2_f__field_descriptors,
  ltpb__point2_f__field_indices_by_name,
  1,  ltpb__point2_f__number_ranges,
  (ProtobufCMessageInit) ltpb__point2_f__init,
  NULL,NULL,NULL    /* reserved[123] */
};
