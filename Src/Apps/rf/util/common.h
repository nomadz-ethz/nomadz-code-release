#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <iterator>
#include <fstream>
#include <map>
#include <random>
#include <set>
#include <system_error>
#include <unordered_set>
#include <vector>
#include <dirent.h>
#include <errno.h>
#include <libgen.h>
#include <sys/stat.h>

template <template <typename...> class C, typename T> static inline bool has(const C<T>& v, const T& val) {
  return std::find(std::begin(v), std::end(v), val) != std::end(v);
}

template <template <typename...> class C> static inline bool has(const C<std::string>& v, const char* val) {
  return has(v, std::string(val));
}

template <typename K, typename V> static inline bool has(const std::map<K, V>& m, const K& val) {
  return m.count(val) == 1;
}

static std::vector<std::string> split(std::string str, std::string token) {
  std::vector<std::string> result;
  while (str.size()) {
    int index = str.find(token);
    if (index != std::string::npos) {
      result.push_back(str.substr(0, index));
      str = str.substr(index + token.size());
      if (str.size() == 0)
        result.push_back(str);
    } else {
      result.push_back(str);
      str = "";
    }
  }
  return result;
}

static inline bool isFile(const std::string& path) {
  struct stat s;
  return stat(path.c_str(), &s) == 0 && S_ISREG(s.st_mode);
}

static inline bool isDir(const std::string& path) {
  struct stat s;
  return stat(path.c_str(), &s) == 0 && S_ISDIR(s.st_mode);
}

static std::string basename(const std::string& path) {
  std::vector<char> pathCopy(path.c_str(), path.c_str() + path.size() + 1);
  return basename(pathCopy.data());
}

static std::string dirname(const std::string& path) {
  std::vector<char> pathCopy(path.c_str(), path.c_str() + path.size() + 1);
  return dirname(pathCopy.data());
}

static std::string cleanPath(const std::string& path) {
  const std::string dir = dirname(path);
  const std::string base = basename(path);

  if (dir == ".") {
    return base;
  } else {
    if (base == "/") {
      return "/";
    } else {
      return dir + "/" + base;
    }
  }
}

static std::system_error systemError(int ec, const std::string& what) {
  return std::system_error(std::error_code(errno, std::system_category()), what);
}

// assumes dir is the path to an existing and accessible directory
static std::vector<std::string> listFiles(const std::string& dir, bool fullPath = true) {
  DIR* dirp = opendir(dir.c_str());
  if (!dirp) {
    const int err = errno;
    throw systemError(err, dir);
  }

  std::vector<std::string> files;
  struct dirent* dp;
  if (fullPath) {
    while ((dp = readdir(dirp)) != NULL) {
      if (dp->d_type == DT_REG) {
        files.push_back(dir + "/" + dp->d_name);
      }
    }
  } else {
    while ((dp = readdir(dirp)) != NULL) {
      if (dp->d_type == DT_REG) {
        files.push_back(dp->d_name);
      }
    }
  }

  return files;
}

// calls mkdir with permissions 775
static void createDir(const std::string& dir) {
  const int ret = mkdir(dir.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
  if (ret != 0) {
    const int err = errno;
    throw systemError(err, dir);
  }
}

static std::string replaceDir(const std::string& path, const std::string& newDir) {
  return newDir + "/" + basename(path);
}

static void copyFile(const std::string& from, const std::string& to) {
  const std::ifstream src(from, std::ios::binary);
  std::ofstream dst(to, std::ios::binary);
  dst << src.rdbuf();
}

// inspired by
// https://stackoverflow.com/questions/28287138/c-randomly-sample-k-numbers-from-range-0n-1-n-k-without-replacement
template <typename T> static std::unordered_set<T> pickSet(const std::vector<T>& haystack, int k, std::mt19937& gen) {
  std::unordered_set<T> picked;
  const int N = haystack.size();

  k = std::min(k, N);

  for (int r = N - k; r < N; ++r) {
    int v = std::uniform_int_distribution<>(1, r)(gen);

    // there are two cases:
    //  v is not in picked -> add it
    //  v is in picked -> well, r is definitely not, since this is the first
    //   iteration in the loop that we could've picked something that big
    if (!picked.insert(haystack.at(v)).second) {
      picked.insert(haystack.at(r));
    }
  }

  return picked;
}

static void printProgress(int i, int n, int intervals = 1, std::ostream& os = std::cout) {
  if (i > 0 && i % intervals == 0)
    os << i << " / " << n << std::endl;
}
