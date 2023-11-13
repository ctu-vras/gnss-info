// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <functional>
#include <sstream>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <ros/duration.h>

typedef void CURL;

namespace gnss_info
{

/**
 * \brief Return path to a directory where cache files should be stored. Also make sure the directory exists.
 * \return The directory path.
 */
std::string getCacheDir();

/**
 * \brief Download the given URL to a stringstream.
 * \param[in] url The URL to download.
 * \param[in] curlOptions Function that provides extra CURL configuration before starting the download.
 *                        URL, WRITEFUNCTION and WRITEDATA options cannot be changed. FOLLOWLOCATION is set to 1
 *                        but can be changed.
 * \return The stream if succeeded, or error message.
 */
cras::expected<std::stringstream, std::string> download(const std::string& url,
    const std::function<void(CURL*)>& curlOptions = {});

/**
 * \brief Return whether the given cache file can be used.
 * \param[in] file Path to the file.
 * \return Whether the file is valid.
 */
bool isCacheFileValid(const std::string& file);

/**
 * \brief Return whether the given cache file can be used.
 * \param[in] file Path to the file.
 * \param[in] validity Duration for which the file is valid (from the time the file was last changed/created).
 * \return Whether the file is valid.
 */
bool isCacheFileValid(const std::string& file, const ros::WallDuration& validity);

}
