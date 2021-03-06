/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: dead_pixel_map.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "dead_pixel_map.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__dead_pixel_map__init
                     (Ltpb__DeadPixelMap         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__DeadPixelMap init_value = LTPB__DEAD_PIXEL_MAP__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__dead_pixel_map__get_packed_size
                     (const Ltpb__DeadPixelMap *message)
{
  assert(message->base.descriptor == &ltpb__dead_pixel_map__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__dead_pixel_map__pack
                     (const Ltpb__DeadPixelMap *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__dead_pixel_map__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__dead_pixel_map__pack_to_buffer
                     (const Ltpb__DeadPixelMap *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__dead_pixel_map__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__DeadPixelMap *
       ltpb__dead_pixel_map__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__DeadPixelMap *)
     protobuf_c_message_unpack (&ltpb__dead_pixel_map__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__dead_pixel_map__free_unpacked
                     (Ltpb__DeadPixelMap *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__dead_pixel_map__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__dead_pixel_map__field_descriptors[3] =
{
  {
    "data_offset",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT64,
    0,   /* quantifier_offset */
    offsetof(Ltpb__DeadPixelMap, data_offset),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "data_size",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__DeadPixelMap, data_size),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "black_level_threshold",
    3,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__DeadPixelMap, has_black_level_threshold),
    offsetof(Ltpb__DeadPixelMap, black_level_threshold),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__dead_pixel_map__field_indices_by_name[] = {
  2,   /* field[2] = black_level_threshold */
  0,   /* field[0] = data_offset */
  1,   /* field[1] = data_size */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__dead_pixel_map__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 3 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__dead_pixel_map__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.DeadPixelMap",
  "DeadPixelMap",
  "Ltpb__DeadPixelMap",
  "ltpb",
  sizeof(Ltpb__DeadPixelMap),
  3,
  ltpb__dead_pixel_map__field_descriptors,
  ltpb__dead_pixel_map__field_indices_by_name,
  1,  ltpb__dead_pixel_map__number_ranges,
  (ProtobufCMessageInit) ltpb__dead_pixel_map__init,
  NULL,NULL,NULL    /* reserved[123] */
};
