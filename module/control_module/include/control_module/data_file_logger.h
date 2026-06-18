#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>

namespace xyber_x1_infer::rl_control_module {

class DataFileLogger {
 public:
  DataFileLogger() = default;
  ~DataFileLogger();

  DataFileLogger(const DataFileLogger&) = delete;
  DataFileLogger& operator=(const DataFileLogger&) = delete;

  bool Open(const std::string& path, bool binary, bool append_newline);
  void Close();
  void Flush() const;
  bool IsOpen() const;

  uint32_t GetLogLevel() const { return 0; }
  void Log(uint32_t lvl,
           uint32_t line,
           uint32_t column,
           const char* file_name,
           const char* function_name,
           const char* log_data,
           size_t log_data_size) const;

  void WriteTextLine(std::string_view line) const;
  void WriteRaw(const void* data, size_t size) const;

 private:
  std::filesystem::path path_;
  bool binary_{false};
  bool append_newline_{false};
  mutable std::mutex mutex_;
  mutable std::ofstream stream_;
};

}  // namespace xyber_x1_infer::rl_control_module
