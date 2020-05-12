#ifndef _SOBIT_COMMON_LIBRARY_UTILS_PRINT_H_
#define _SOBIT_COMMON_LIBRARY_UTILS_PRINT_H_
#include <iostream>

namespace scl {

#define printFileName() _printFileName(__FILE__)
#define printFuncName() _printFuncName(__PRETTY_FUNCTION__)
#define printLine() _printLine(__LINE__)
#define printFuncAndLine() _printFuncAndLine(__LINE__, __PRETTY_FUNCTION__)
#define printFileAndLine() _printFileAndLine(__LINE__, __FILE__)

inline void _printFileName(const char* file_name) { printf("%s\n", file_name); }
inline void _printFuncName(const char* func_name) { printf("%s\n", func_name); }
inline void _printLine(int line) { printf("%d\n", line); }
inline void _printFuncAndLine(int line, const char* func_name) { printf("%d line : %s\n", line, func_name); }
inline void _printFileAndLine(int line, const char* file_name) { printf("%d line : %s\n", line, file_name); }

};  // namespace scl

#endif /* _SOBIT_COMMON_LIBRARY_UTILS_PRINT_H_ */