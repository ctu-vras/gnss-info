// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <cstdlib>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <curl/curl.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <gnss_info/common.h>

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

    if (!boost::filesystem::is_directory(cacheDir))
        boost::filesystem::create_directories(cacheDir.c_str());

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

}