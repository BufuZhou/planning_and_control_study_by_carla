// copyright

/**
 * @file
 */

#ifndef SRC_COMMON_INCLUDE_LOG_HPP_
#define SRC_COMMON_INCLUDE_LOG_HPP_

#include "glog/logging.h"

#define ADEBUG VLOG(4) << "[DEBUG] "
#define AINFO VLOG(3) << "[INFO] "
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)

#endif  // SRC_COMMON_INCLUDE_LOG_HPP_
