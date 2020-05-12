#ifndef _SOBIT_COMMON_LIBRARY_UTILS_FILE_SYSTEM_H_
#define _SOBIT_COMMON_LIBRARY_UTILS_FILE_SYSTEM_H_

#include <fstream>
#include <iostream>
#include <string>

namespace scl {

std::ofstream openNewFile2Write(std::string file_name);

std::ofstream openAddFile2Write(std::string file_name);

std::ofstream openFile2OverWrite(std::string file_name);

}  // namespace scl

#endif /* _SOBIT_COMMON_LIBRARY_UTILS_FILE_SYSTEM_H_ */