#include "control_module/data_file_logger.h"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace xyber_x1_infer::rl_control_module {
namespace {

std::filesystem::path MakeTempPath(const char* suffix) {
  const auto id = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("data_file_logger_" + std::to_string(id) + suffix);
}

TEST(DataFileLoggerTest, ReportsClosedStreamWritesAsFailures) {
  DataFileLogger logger;

  EXPECT_FALSE(logger.WriteTextLine("not-open"));
  EXPECT_FALSE(logger.WriteRaw("x", 1));
  EXPECT_FALSE(logger.Flush());
  EXPECT_TRUE(logger.Close());
}

TEST(DataFileLoggerTest, WritesAndClosesTextFileSuccessfully) {
  const auto path = MakeTempPath(".csv");
  DataFileLogger logger;

  ASSERT_TRUE(logger.Open(path.string(), false, true));
  EXPECT_TRUE(logger.WriteTextLine("header"));
  EXPECT_TRUE(logger.WriteTextLine("row"));
  EXPECT_TRUE(logger.Flush());
  EXPECT_TRUE(logger.Close());

  std::ifstream input(path);
  const std::string contents((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
  EXPECT_EQ(contents, "header\nrow\n");
  std::filesystem::remove(path);
}

TEST(DataFileLoggerTest, WritesBinaryPayloadWithoutNewline) {
  const auto path = MakeTempPath(".bin");
  const std::vector<float> payload{1.0F, -2.5F, 3.25F};
  DataFileLogger logger;

  ASSERT_TRUE(logger.Open(path.string(), true, false));
  EXPECT_TRUE(logger.WriteRaw(payload.data(), payload.size() * sizeof(float)));
  EXPECT_TRUE(logger.Close());
  EXPECT_EQ(std::filesystem::file_size(path), payload.size() * sizeof(float));
  std::filesystem::remove(path);
}

#ifdef __linux__
TEST(DataFileLoggerTest, ReportsDeviceWriteFailures) {
  if (!std::filesystem::exists("/dev/full")) {
    GTEST_SKIP() << "/dev/full is unavailable";
  }

  DataFileLogger logger;
  if (!logger.Open("/dev/full", true, false)) {
    GTEST_SKIP() << "/dev/full cannot be opened by std::ofstream";
  }

  const char payload[] = "must fail";
  const bool write_ok = logger.WriteRaw(payload, sizeof(payload));
  const bool flush_ok = logger.Flush();
  EXPECT_FALSE(write_ok && flush_ok);
  EXPECT_FALSE(logger.Close());
}
#endif

}  // namespace
}  // namespace xyber_x1_infer::rl_control_module
