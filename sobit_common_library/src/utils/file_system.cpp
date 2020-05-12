#include "sobit_common_library/utils/file_system.h"

namespace scl {

std::ofstream openNewFile2Write(std::string file_name) {
  std::ofstream writing_file;
  writing_file.open(file_name, std::ios::out);
  return writing_file;
}

std::ofstream openAddFile2Write(std::string file_name) {
  std::ofstream writing_file;
  writing_file.open(file_name, std::ios::app);
  return writing_file;
}

std::ofstream openFile2OverWrite(std::string file_name) {
  std::ofstream writing_file;
  writing_file.open(file_name, std::ios::app);
  return writing_file;
}

}  // namespace scl