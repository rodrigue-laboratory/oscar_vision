#pragma once

#include <ctime>
#include <string>

#include <filesystem>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

namespace mimik {
namespace vision {

// https://stackoverflow.com/a/38034148
inline std::tm localtime_xp(std::time_t timer)
{
  std::tm bt{};
#if defined(__unix__)
  localtime_r(&timer, &bt);
#elif defined(_MSC_VER)
  localtime_s(&bt, &timer);
#else
  static std::mutex mtx;
  std::lock_guard<std::mutex> lock(mtx);
  bt = *std::localtime(&timer);
#endif
  return bt;
}

// default = "YYYY-MM-DD HH:MM:SS"
inline std::string time_stamp(const std::string& fmt = "%F %T")
{
  auto bt = localtime_xp(std::time(0));
  char buf[64];
  return { buf, std::strftime(buf, sizeof(buf), fmt.c_str(), &bt) };
}

/// store image + filename
struct ImageDescription
{
  cv::Mat image;
  std::string filename;
};

void loadImagesFromOpenCV(const std::string& directory_path, std::vector<ImageDescription>& image_descriptions)
{
  for (const auto& entry : std::filesystem::directory_iterator(directory_path))
  {
    if (entry.is_directory())
      continue;

    const auto& path = entry.path();

    image_descriptions.emplace_back();

    auto& image_description = image_descriptions.back();
    image_description.filename = path.filename();
    image_description.image = cv::imread(path);
  }
}

}  // namespace vision
}  // namespace mimik
