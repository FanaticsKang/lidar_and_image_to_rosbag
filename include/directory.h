#pragma once

#include <string>

// 文件操作
class Directory {
 public:
  // 从绝对路径中提取文件夹
  static std::string GetPrefix(const std::string& path_to_file) {
    size_t i = path_to_file.find_last_of("/");
    return i == std::string::npos ? "" : path_to_file.substr(0, i + 1);
  }

  // 生成绝对路径
  static std::string GetAbsolutePath(const std::string& prefix,
                                     const std::string& path) {
    std::string absolute_path(path);
    if (!absolute_path.empty() && absolute_path[0] != '/') {
      absolute_path = prefix + absolute_path;
    }
    return absolute_path;
  }

  // 判断末尾字符
  static bool EndWith(const std::string& full_string,
                      const std::string& ending) {
    if (full_string.length() >= ending.length()) {
      return (0 ==
              full_string.compare(full_string.length() - ending.length(),
                                  ending.length(), ending));
    }
    return false;
  }

  // 判断是否需要添加一个ending
  static std::string AddEnding(const std::string& full_string,
                               const std::string& ending) {
    if (EndWith(full_string, ending)) {
      return full_string;
    }
    return (full_string + ending);
  }

  // 判断是否需要删除一个ending
  static std::string RemoveEnding(const std::string& full_string,
                                  const std::string& ending) {
    if (EndWith(full_string, ending)) {
      return full_string.substr(0, full_string.size() - ending.size());
    }
    return full_string;
  }
};