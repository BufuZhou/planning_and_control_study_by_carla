// copyright
#ifndef SRC_COMMON_INCLUDE_COMMON_PROTO_UTIL_HPP_
#define SRC_COMMON_INCLUDE_COMMON_PROTO_UTIL_HPP_
#include <string>
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace common {

bool ReadProtoFromTextFile(const std::string &filename,
                           google::protobuf::Message *proto);

}  // namespace common

#endif  // SRC_COMMON_INCLUDE_COMMON_PROTO_UTIL_HPP_
