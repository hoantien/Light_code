/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: tof_calibration.proto */

#ifndef PROTOBUF_C_tof_5fcalibration_2eproto__INCLUDED
#define PROTOBUF_C_tof_5fcalibration_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1002001 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _Ltpb__ToFCalibration Ltpb__ToFCalibration;


/* --- enums --- */


/* --- messages --- */

struct  _Ltpb__ToFCalibration
{
  ProtobufCMessage base;
  /*
   * distance in mm of the offset chart
   */
  float offset_distance;
  /*
   * measured distance in mm of the offset chart
   */
  float offset_measurement;
  /*
   * distance in mm of the cross talk chart
   */
  float xtalk_distance;
  /*
   * measured distance in mm of the cross talk chart
   */
  float xtalk_measurement;
};
#define LTPB__TO_FCALIBRATION__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ltpb__to_fcalibration__descriptor) \
    , 0, 0, 0, 0 }


/* Ltpb__ToFCalibration methods */
void   ltpb__to_fcalibration__init
                     (Ltpb__ToFCalibration         *message);
size_t ltpb__to_fcalibration__get_packed_size
                     (const Ltpb__ToFCalibration   *message);
size_t ltpb__to_fcalibration__pack
                     (const Ltpb__ToFCalibration   *message,
                      uint8_t             *out);
size_t ltpb__to_fcalibration__pack_to_buffer
                     (const Ltpb__ToFCalibration   *message,
                      ProtobufCBuffer     *buffer);
Ltpb__ToFCalibration *
       ltpb__to_fcalibration__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   ltpb__to_fcalibration__free_unpacked
                     (Ltpb__ToFCalibration *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Ltpb__ToFCalibration_Closure)
                 (const Ltpb__ToFCalibration *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor ltpb__to_fcalibration__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_tof_5fcalibration_2eproto__INCLUDED */
