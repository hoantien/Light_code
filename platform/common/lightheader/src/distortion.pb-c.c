/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: distortion.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "distortion.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__distortion__polynomial__init
                     (Ltpb__Distortion__Polynomial         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__Distortion__Polynomial init_value = LTPB__DISTORTION__POLYNOMIAL__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__distortion__cra__init
                     (Ltpb__Distortion__CRA         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__Distortion__CRA init_value = LTPB__DISTORTION__CRA__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__distortion__init
                     (Ltpb__Distortion         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__Distortion init_value = LTPB__DISTORTION__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__distortion__get_packed_size
                     (const Ltpb__Distortion *message)
{
  assert(message->base.descriptor == &ltpb__distortion__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__distortion__pack
                     (const Ltpb__Distortion *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__distortion__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__distortion__pack_to_buffer
                     (const Ltpb__Distortion *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__distortion__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__Distortion *
       ltpb__distortion__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__Distortion *)
     protobuf_c_message_unpack (&ltpb__distortion__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__distortion__free_unpacked
                     (Ltpb__Distortion *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__distortion__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__distortion__polynomial__field_descriptors[4] =
{
  {
    "distortion_center",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Distortion__Polynomial, distortion_center),
    &ltpb__point2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "normalization",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Distortion__Polynomial, normalization),
    &ltpb__point2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "coeffs",
    3,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__Distortion__Polynomial, n_coeffs),
    offsetof(Ltpb__Distortion__Polynomial, coeffs),
    NULL,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_PACKED,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "fit_cost",
    4,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__Distortion__Polynomial, has_fit_cost),
    offsetof(Ltpb__Distortion__Polynomial, fit_cost),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__distortion__polynomial__field_indices_by_name[] = {
  2,   /* field[2] = coeffs */
  0,   /* field[0] = distortion_center */
  3,   /* field[3] = fit_cost */
  1,   /* field[1] = normalization */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__distortion__polynomial__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 4 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__distortion__polynomial__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.Distortion.Polynomial",
  "Polynomial",
  "Ltpb__Distortion__Polynomial",
  "ltpb",
  sizeof(Ltpb__Distortion__Polynomial),
  4,
  ltpb__distortion__polynomial__field_descriptors,
  ltpb__distortion__polynomial__field_indices_by_name,
  1,  ltpb__distortion__polynomial__number_ranges,
  (ProtobufCMessageInit) ltpb__distortion__polynomial__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__distortion__cra__field_descriptors[7] =
{
  {
    "distortion_center",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Distortion__CRA, distortion_center),
    &ltpb__point2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sensor_distance",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Distortion__CRA, sensor_distance),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "exit_pupil_distance",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Distortion__CRA, exit_pupil_distance),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "pixel_size",
    4,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Distortion__CRA, pixel_size),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "cra",
    5,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(Ltpb__Distortion__CRA, n_cra),
    offsetof(Ltpb__Distortion__CRA, cra),
    &ltpb__point2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "coeffs",
    6,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(Ltpb__Distortion__CRA, n_coeffs),
    offsetof(Ltpb__Distortion__CRA, coeffs),
    &ltpb__point2_f__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "fit_cost",
    7,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__Distortion__CRA, has_fit_cost),
    offsetof(Ltpb__Distortion__CRA, fit_cost),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__distortion__cra__field_indices_by_name[] = {
  5,   /* field[5] = coeffs */
  4,   /* field[4] = cra */
  0,   /* field[0] = distortion_center */
  2,   /* field[2] = exit_pupil_distance */
  6,   /* field[6] = fit_cost */
  3,   /* field[3] = pixel_size */
  1,   /* field[1] = sensor_distance */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__distortion__cra__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 7 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__distortion__cra__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.Distortion.CRA",
  "CRA",
  "Ltpb__Distortion__CRA",
  "ltpb",
  sizeof(Ltpb__Distortion__CRA),
  7,
  ltpb__distortion__cra__field_descriptors,
  ltpb__distortion__cra__field_indices_by_name,
  1,  ltpb__distortion__cra__number_ranges,
  (ProtobufCMessageInit) ltpb__distortion__cra__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__distortion__field_descriptors[2] =
{
  {
    "polynomial",
    1,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Distortion, polynomial),
    &ltpb__distortion__polynomial__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "cra",
    2,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__Distortion, cra),
    &ltpb__distortion__cra__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__distortion__field_indices_by_name[] = {
  1,   /* field[1] = cra */
  0,   /* field[0] = polynomial */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__distortion__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__distortion__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.Distortion",
  "Distortion",
  "Ltpb__Distortion",
  "ltpb",
  sizeof(Ltpb__Distortion),
  2,
  ltpb__distortion__field_descriptors,
  ltpb__distortion__field_indices_by_name,
  1,  ltpb__distortion__number_ranges,
  (ProtobufCMessageInit) ltpb__distortion__init,
  NULL,NULL,NULL    /* reserved[123] */
};
