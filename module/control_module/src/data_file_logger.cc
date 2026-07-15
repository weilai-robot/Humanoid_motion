#include "control_module/data_file_logger.h"

namespace xyber_x1_infer::rl_control_module {

DataFileLogger::~DataFileLogger() {
  (void)Close();
}

bool DataFileLogger::Open(const std::string& path, bool binary, bool append_newline) {
  (void)Close();

  std::lock_guard<std::mutex> lock(mutex_);

  path_ = std::filesystem::path(path);
  binary_ = binary;
  append_newline_ = append_newline;

  std::error_code ec;
  if (path_.has_parent_path()) {
    std::filesystem::create_directories(path_.parent_path(), ec);
    if (ec) {
      return false;
    }
  }

  std::ios::openmode mode = std::ios::out | std::ios::trunc;
  if (binary_) {
    mode |= std::ios::binary;
  }

  stream_.clear();
  stream_.open(path_, mode);
  return stream_.is_open() && stream_.good();
}

bool DataFileLogger::Close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_.is_open()) {
    return true;
  }

  stream_.flush();
  bool success = stream_.good();
  stream_.close();
  success = success && !stream_.fail();
  return success;
}

bool DataFileLogger::Flush() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_.is_open()) {
    return false;
  }

  stream_.flush();
  return stream_.good();
}

bool DataFileLogger::IsOpen() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stream_.is_open();
}

void DataFileLogger::Log(uint32_t lvl,
                         uint32_t line,
                         uint32_t column,
                         const char* file_name,
                         const char* function_name,
                         const char* log_data,
                         size_t log_data_size) const {
  (void)lvl;
  (void)line;
  (void)column;
  (void)file_name;
  (void)function_name;

  (void)Write(log_data, log_data_size);
}

bool DataFileLogger::Write(const char* data, size_t size) const {
  if (data == nullptr || size == 0) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_.is_open()) {
    return false;
  }

  stream_.write(data, static_cast<std::streamsize>(size));
  if (append_newline_) {
    stream_.put('\n');
  }
  return stream_.good();
}

bool DataFileLogger::WriteTextLine(std::string_view line) const {
  return Write(line.data(), line.size());
}

bool DataFileLogger::WriteRaw(const void* data, size_t size) const {
  return Write(static_cast<const char*>(data), size);
}

}  // namespace xyber_x1_infer::rl_control_module
