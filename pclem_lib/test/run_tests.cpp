
#include <glog/logging.h>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
