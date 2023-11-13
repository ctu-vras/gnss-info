// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <cstdlib>
#include <sstream>
#include <string>
#include CXX_FILESYSTEM_INCLUDE

#include <curl/curl.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <gnss_info/cache.h>

namespace fs = CXX_FILESYSTEM_NAMESPACE;

namespace gnss_info
{

std::string getCacheDir()
{
    std::string cacheDir;
    const auto envDir = std::getenv("GNSS_INFO_CACHE_DIR");
    if (envDir != nullptr)
    {
        cacheDir = envDir;
    }
    else
    {
#ifdef _WIN32
        const std::string homeDirEnv {"USERPROFILE"};
#else
        const std::string homeDirEnv{"HOME"};
#endif

        const auto cacheEnv = std::getenv("XDG_CACHE_HOME");
        if (cacheEnv != nullptr)
            cacheDir = cacheEnv;
        else
            cacheDir = std::string(std::getenv(homeDirEnv.c_str())) + "/.cache";
        cacheDir += "/gnss_info";
    }

    if (!fs::is_directory(cacheDir))
        fs::create_directories(cacheDir);

    return cacheDir;
}

cras::expected<std::stringstream, std::string> download(const std::string& url,
    const std::function<void(CURL*)>& curlOptions)
{
    const auto curlCallback = +[](void *contents, size_t size, size_t nmemb, void *userp)
    {
        *static_cast<std::stringstream*>(userp) << std::string(static_cast<char*>(contents), size * nmemb);
        return size * nmemb;
    };

    std::stringstream readBuffer;
    const auto curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1u);
    if (curlOptions)
        curlOptions(curl);  // Set extra config options.
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    const auto res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK)
        return cras::make_unexpected(cras::format("Error downloading %s: %s", url.c_str(), curl_easy_strerror(res)));

    return readBuffer;
}

bool isCacheFileValid(const std::string& file)
{
    return fs::exists(file);
}

bool isCacheFileValid(const std::string& file, const ros::WallDuration& validity)
{
    if (!isCacheFileValid(file))
        return false;

#if CXX_FILESYSTEM_IS_BOOST
    auto fileTime = fs::last_write_time(this->data->cacheFile);
    const auto oldestValidCache = static_cast<time_t>((ros::WallTime::now() - this->data->cacheValidity).sec);
#else
    auto fileTime = fs::last_write_time(file);
    const auto oldestValidCache = decltype(fileTime)::clock::now() - std::chrono::duration<long double>(validity.sec);
#endif
    return oldestValidCache <= fileTime;
}

}
