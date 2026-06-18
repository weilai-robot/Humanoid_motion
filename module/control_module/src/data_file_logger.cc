#include "control_module/data_file_logger.h"

namespace xyber_x1_infer::rl_control_module {

DataFileLogger::~DataFileLogger() {
  Close();
}

bool DataFileLogger::Open(const std::string& path, bool binary, bool append_newline) {
  Close();

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

  stream_.open(path_, mode);
  return stream_.is_open();
}

void DataFileLogger::Close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_.is_open()) {
    stream_.flush();
    stream_.close();
  }
}

void DataFileLogger::Flush() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_.is_open()) {
    stream_.flush();
  }
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

  if (log_data == nullptr || log_data_size == 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_.is_open()) {
    return;
  }

  stream_.write(log_data, static_cast<std::streamsize>(log_data_size));
  if (append_newline_) {
    stream_.put('\n');
  }
}

void DataFileLogger::WriteTextLine(std::string_view line) const {
  Log(0, 0, 0, nullptr, nullptr, line.data(), line.size());
}

void DataFileLogger::WriteRaw(const void* data, size_t size) const {
  Log(0, 0, 0, nullptr, nullptr, static_cast<const char*>(data), size);
}

}  // namespace xyber_x1_infer::rl_control_module
