// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ErrorCodes.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ErrorCodes_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ErrorCodes_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3008000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3008000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ErrorCodes_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ErrorCodes_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ErrorCodes_2eproto;
namespace Cognata {
namespace SDK {
class ErrorMessage;
class ErrorMessageDefaultTypeInternal;
extern ErrorMessageDefaultTypeInternal _ErrorMessage_default_instance_;
}  // namespace SDK
}  // namespace Cognata
PROTOBUF_NAMESPACE_OPEN
template<> ::Cognata::SDK::ErrorMessage* Arena::CreateMaybeMessage<::Cognata::SDK::ErrorMessage>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace Cognata {
namespace SDK {

enum CognataErr : int {
  COGNATA_ERR_OK = 0,
  COGNATA_ERR_LOG_FAILED = -100,
  COGNATA_ERR_NOT_IMPLEMENTED = -101,
  COGNATA_ERR_UNEXPECTED = -102,
  COGNATA_ERR_BAD_PARAMS = -103,
  COGNATA_ERR_CONNECTION_FAILED = -1000,
  COGNATA_ERR_NOT_CONNECTED = -1001,
  COGNATA_ERR_DISCONNECTED = -1002,
  COGNATA_ERR_CONNECTION_TIMEOUT = -1003,
  COGNATA_ERR_CONNECTIONS_MAXED_OUT = -1004,
  COGNATA_ERR_START_FAILED = -2000,
  COGNATA_ERR_SEND_CMD_FAILED = -3000,
  COGNATA_ERR_QUERY = -4000,
  COGNATA_ERR_CAR_NOT_FOUND = -4001,
  COGNATA_ERR_SENSORS = -5000,
  COGNATA_ERR_SENSORS_NOT_FOUND = -5001,
  COGNATA_ERR_REGISTER_SENSOR_FAILED = -5002,
  COGNATA_ERR_BAD_TIMING_SETTINGS = -6000,
  CognataErr_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<::PROTOBUF_NAMESPACE_ID::int32>::min(),
  CognataErr_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<::PROTOBUF_NAMESPACE_ID::int32>::max()
};
bool CognataErr_IsValid(int value);
constexpr CognataErr CognataErr_MIN = COGNATA_ERR_BAD_TIMING_SETTINGS;
constexpr CognataErr CognataErr_MAX = COGNATA_ERR_OK;
constexpr int CognataErr_ARRAYSIZE = CognataErr_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* CognataErr_descriptor();
template<typename T>
inline const std::string& CognataErr_Name(T enum_t_value) {
  static_assert(::std::is_same<T, CognataErr>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function CognataErr_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    CognataErr_descriptor(), enum_t_value);
}
inline bool CognataErr_Parse(
    const std::string& name, CognataErr* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<CognataErr>(
    CognataErr_descriptor(), name, value);
}
// ===================================================================

class ErrorMessage :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:Cognata.SDK.ErrorMessage) */ {
 public:
  ErrorMessage();
  virtual ~ErrorMessage();

  ErrorMessage(const ErrorMessage& from);
  ErrorMessage(ErrorMessage&& from) noexcept
    : ErrorMessage() {
    *this = ::std::move(from);
  }

  inline ErrorMessage& operator=(const ErrorMessage& from) {
    CopyFrom(from);
    return *this;
  }
  inline ErrorMessage& operator=(ErrorMessage&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const ErrorMessage& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ErrorMessage* internal_default_instance() {
    return reinterpret_cast<const ErrorMessage*>(
               &_ErrorMessage_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(ErrorMessage* other);
  friend void swap(ErrorMessage& a, ErrorMessage& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ErrorMessage* New() const final {
    return CreateMaybeMessage<ErrorMessage>(nullptr);
  }

  ErrorMessage* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ErrorMessage>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ErrorMessage& from);
  void MergeFrom(const ErrorMessage& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  #else
  bool MergePartialFromCodedStream(
      ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const final;
  ::PROTOBUF_NAMESPACE_ID::uint8* InternalSerializeWithCachedSizesToArray(
      ::PROTOBUF_NAMESPACE_ID::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ErrorMessage* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "Cognata.SDK.ErrorMessage";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ErrorCodes_2eproto);
    return ::descriptor_table_ErrorCodes_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // .Cognata.SDK.CognataErr errorCode = 1;
  void clear_errorcode();
  static const int kErrorCodeFieldNumber = 1;
  ::Cognata::SDK::CognataErr errorcode() const;
  void set_errorcode(::Cognata::SDK::CognataErr value);

  // @@protoc_insertion_point(class_scope:Cognata.SDK.ErrorMessage)
 private:
  class HasBitSetters;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  int errorcode_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ErrorCodes_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ErrorMessage

// .Cognata.SDK.CognataErr errorCode = 1;
inline void ErrorMessage::clear_errorcode() {
  errorcode_ = 0;
}
inline ::Cognata::SDK::CognataErr ErrorMessage::errorcode() const {
  // @@protoc_insertion_point(field_get:Cognata.SDK.ErrorMessage.errorCode)
  return static_cast< ::Cognata::SDK::CognataErr >(errorcode_);
}
inline void ErrorMessage::set_errorcode(::Cognata::SDK::CognataErr value) {
  
  errorcode_ = value;
  // @@protoc_insertion_point(field_set:Cognata.SDK.ErrorMessage.errorCode)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace SDK
}  // namespace Cognata

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::Cognata::SDK::CognataErr> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::Cognata::SDK::CognataErr>() {
  return ::Cognata::SDK::CognataErr_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ErrorCodes_2eproto