/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: sensor_characterization.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif
#include "sections.h"
#include "sensor_characterization.pb-c.h"
LIGHT_HEADER_FUNC void   ltpb__sensor_characterization__vst_noise_model__vst_model__init
                     (Ltpb__SensorCharacterization__VstNoiseModel__VstModel         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__SensorCharacterization__VstNoiseModel__VstModel init_value = LTPB__SENSOR_CHARACTERIZATION__VST_NOISE_MODEL__VST_MODEL__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__sensor_characterization__vst_noise_model__init
                     (Ltpb__SensorCharacterization__VstNoiseModel         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__SensorCharacterization__VstNoiseModel init_value = LTPB__SENSOR_CHARACTERIZATION__VST_NOISE_MODEL__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC void   ltpb__sensor_characterization__init
                     (Ltpb__SensorCharacterization         *message)
{
  LIGHT_HEADER_STATIC static Ltpb__SensorCharacterization init_value = LTPB__SENSOR_CHARACTERIZATION__INIT;
  *message = init_value;
}
LIGHT_HEADER_FUNC size_t ltpb__sensor_characterization__get_packed_size
                     (const Ltpb__SensorCharacterization *message)
{
  assert(message->base.descriptor == &ltpb__sensor_characterization__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
LIGHT_HEADER_FUNC size_t ltpb__sensor_characterization__pack
                     (const Ltpb__SensorCharacterization *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &ltpb__sensor_characterization__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
LIGHT_HEADER_FUNC size_t ltpb__sensor_characterization__pack_to_buffer
                     (const Ltpb__SensorCharacterization *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &ltpb__sensor_characterization__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LIGHT_HEADER_FUNC Ltpb__SensorCharacterization *
       ltpb__sensor_characterization__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Ltpb__SensorCharacterization *)
     protobuf_c_message_unpack (&ltpb__sensor_characterization__descriptor,
                                allocator, len, data);
}
LIGHT_HEADER_FUNC void   ltpb__sensor_characterization__free_unpacked
                     (Ltpb__SensorCharacterization *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &ltpb__sensor_characterization__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__sensor_characterization__vst_noise_model__vst_model__field_descriptors[2] =
{
  {
    "a",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel__VstModel, a),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "b",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel__VstModel, b),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__sensor_characterization__vst_noise_model__vst_model__field_indices_by_name[] = {
  0,   /* field[0] = a */
  1,   /* field[1] = b */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__sensor_characterization__vst_noise_model__vst_model__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__sensor_characterization__vst_noise_model__vst_model__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.SensorCharacterization.VstNoiseModel.VstModel",
  "VstModel",
  "Ltpb__SensorCharacterization__VstNoiseModel__VstModel",
  "ltpb",
  sizeof(Ltpb__SensorCharacterization__VstNoiseModel__VstModel),
  2,
  ltpb__sensor_characterization__vst_noise_model__vst_model__field_descriptors,
  ltpb__sensor_characterization__vst_noise_model__vst_model__field_indices_by_name,
  1,  ltpb__sensor_characterization__vst_noise_model__vst_model__number_ranges,
  (ProtobufCMessageInit) ltpb__sensor_characterization__vst_noise_model__vst_model__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__sensor_characterization__vst_noise_model__field_descriptors[7] =
{
  {
    "gain",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel, gain),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "threshold",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel, threshold),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "scale",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel, scale),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "red",
    4,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel, red),
    &ltpb__sensor_characterization__vst_noise_model__vst_model__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "green",
    5,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel, green),
    &ltpb__sensor_characterization__vst_noise_model__vst_model__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "blue",
    6,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel, blue),
    &ltpb__sensor_characterization__vst_noise_model__vst_model__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "panchromatic",
    7,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization__VstNoiseModel, panchromatic),
    &ltpb__sensor_characterization__vst_noise_model__vst_model__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__sensor_characterization__vst_noise_model__field_indices_by_name[] = {
  5,   /* field[5] = blue */
  0,   /* field[0] = gain */
  4,   /* field[4] = green */
  6,   /* field[6] = panchromatic */
  3,   /* field[3] = red */
  2,   /* field[2] = scale */
  1,   /* field[1] = threshold */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__sensor_characterization__vst_noise_model__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 7 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__sensor_characterization__vst_noise_model__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.SensorCharacterization.VstNoiseModel",
  "VstNoiseModel",
  "Ltpb__SensorCharacterization__VstNoiseModel",
  "ltpb",
  sizeof(Ltpb__SensorCharacterization__VstNoiseModel),
  7,
  ltpb__sensor_characterization__vst_noise_model__field_descriptors,
  ltpb__sensor_characterization__vst_noise_model__field_indices_by_name,
  1,  ltpb__sensor_characterization__vst_noise_model__number_ranges,
  (ProtobufCMessageInit) ltpb__sensor_characterization__vst_noise_model__init,
  NULL,NULL,NULL    /* reserved[123] */
};
LIGHT_HEADER_CONST static const ProtobufCFieldDescriptor ltpb__sensor_characterization__field_descriptors[4] =
{
  {
    "black_level",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization, black_level),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "white_level",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Ltpb__SensorCharacterization, white_level),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "cliff_slope",
    3,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(Ltpb__SensorCharacterization, has_cliff_slope),
    offsetof(Ltpb__SensorCharacterization, cliff_slope),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "vst_model",
    4,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(Ltpb__SensorCharacterization, n_vst_model),
    offsetof(Ltpb__SensorCharacterization, vst_model),
    &ltpb__sensor_characterization__vst_noise_model__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
LIGHT_HEADER_CONST static const unsigned ltpb__sensor_characterization__field_indices_by_name[] = {
  0,   /* field[0] = black_level */
  2,   /* field[2] = cliff_slope */
  3,   /* field[3] = vst_model */
  1,   /* field[1] = white_level */
};
LIGHT_HEADER_CONST static const ProtobufCIntRange ltpb__sensor_characterization__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 4 }
};
LIGHT_HEADER_CONST const ProtobufCMessageDescriptor ltpb__sensor_characterization__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ltpb.SensorCharacterization",
  "SensorCharacterization",
  "Ltpb__SensorCharacterization",
  "ltpb",
  sizeof(Ltpb__SensorCharacterization),
  4,
  ltpb__sensor_characterization__field_descriptors,
  ltpb__sensor_characterization__field_indices_by_name,
  1,  ltpb__sensor_characterization__number_ranges,
  (ProtobufCMessageInit) ltpb__sensor_characterization__init,
  NULL,NULL,NULL    /* reserved[123] */
};
