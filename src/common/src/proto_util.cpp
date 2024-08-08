// copyright
#include "common/proto_util.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

namespace common {


bool ReadProtoFromTextFile(const std::string &filename,
                           google::protobuf::Message *proto) {
  int fd = open(filename.c_str(), O_RDONLY);
  if (fd < 0) {
    // AERROR << "cannot open file " << filename;
    std::cout << "cannot open file " << filename;
    return false;
  }
  google::protobuf::io::FileInputStream raw_input(fd);

  bool success = google::protobuf::TextFormat::Parse(&raw_input, proto);

  close(fd);
  return success;
}

}  // namespace common
