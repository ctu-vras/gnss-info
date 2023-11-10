// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <iostream>
#include <limits>
#include <list>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <boost/filesystem.hpp>

#include <curl/curl.h>
#include <gnsstk/FFData.hpp>
#include <gnsstk/SinexStream.hpp>  // Has to be before SinexData include
#include <gnsstk/SinexData.hpp>
#include <gnsstk/SinexTypes.hpp>
#include <gnsstk/SinexBlock.hpp>
#include <gnsstk/YDSTime.hpp>
#include <yaml-cpp/yaml.h>

#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <gnss_info/igs_satellite_metadata.h>
#include <gnss_info_msgs/Enums.h>
#include <gnss_info_msgs/SatelliteInfo.h>
#include <gnsstk_ros/constellations.h>
#include <gnsstk_ros/time.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace gnss_info
{

std::string longSinexTime(const gnsstk::Sinex::Time& t)
{
    if (t.year > 50)
        return "19" + static_cast<std::string>(t);
    return "20" + static_cast<std::string>(t);
}

struct SatelliteIdentifier : gnsstk::Sinex::DataType
{
    static const std::string BLOCK_TITLE;
    static const size_t MIN_LINE_LEN = 40;
    static const size_t MAX_LINE_LEN = 80;

    std::string svn;  //!< SVN
    std::string cosparID;  //!< Cospar ID
    uint32_t satcatID {0u};  //!< Satcat ID
    std::string block;  //!< Block
    std::string comment;  //!< Comment

    SatelliteIdentifier() = default;

    explicit SatelliteIdentifier(const std::string& line, size_t num = 0)
    {
        *this = line;
    }

    void dump(std::ostream& s) const
    {
        s << BLOCK_TITLE << " :" << std::endl;
        s << " svn=" << this->svn << std::endl;
        s << " cosparID=" << this->cosparID << std::endl;
        s << " satcatID=" << this->satcatID << std::endl;
        s << " block=" << this->block << std::endl;
        s << " comment=" << this->comment << std::endl;
    }

    operator std::string() const override
    {
        try
        {
            std::ostringstream ss;
            ss << gnsstk::Sinex::DATA_START;
            ss << gnsstk::Sinex::formatStr(this->svn, 4);
            ss << ' ' << gnsstk::Sinex::formatStr(this->cosparID, 9);
            ss << "  " << gnsstk::Sinex::formatStr(std::to_string(this->satcatID), 5);
            ss << ' ' << gnsstk::Sinex::formatStr(this->block, 15);
            ss << ' ' << gnsstk::Sinex::formatStr(this->comment, 41);
            return ss.str();
        }
        catch (gnsstk::Exception& exc)
        {
            GNSSTK_RETHROW(exc);
        }
    }

    void operator=(const std::string& line) override
    {
        static int FIELD_DIVS[] = {0, 5, 15, 22, 38, -1};
        try
        {
            gnsstk::Sinex::isValidLineStructure(line, MIN_LINE_LEN, MAX_LINE_LEN, FIELD_DIVS);
            this->svn = line.substr(1, 4);
            this->cosparID = line.substr(6, 9);
            this->satcatID = gnsstk::StringUtils::asUnsigned(line.substr(17, 5));
            this->block = line.substr(23, 15);
            while (!this->block.empty() && this->block.back() == ' ')
                cras::stripTrailing(this->block, ' ');
            this->comment = line.substr(39, 41);
        }
        catch (gnsstk::Exception& exc)
        {
            GNSSTK_RETHROW(exc);
        }
    }
};
const std::string SatelliteIdentifier::BLOCK_TITLE {"SATELLITE/IDENTIFIER"};  // NOLINT(runtime/string)

struct SatellitePRN : gnsstk::Sinex::DataType
{
    static const std::string BLOCK_TITLE;
    static constexpr size_t MIN_LINE_LEN = 39;
    static constexpr size_t MAX_LINE_LEN = 80;

    std::string svn;  //!< SVN
    gnsstk::Sinex::Time validFrom;  //!< Time from which the entry is valid
    gnsstk::Sinex::Time validTo;  //!< Time until which the entry is valid (valid up to now if zero)
    std::string prn;  //!< PRN
    std::string comment;  //!< Comment

    SatellitePRN() = default;

    explicit SatellitePRN(const std::string& line, size_t num = 0)
    {
        *this = line;
    }

    void dump(std::ostream& s) const
    {
        s << BLOCK_TITLE << " :" << std::endl;
        s << " svn=" << this->svn << std::endl;
        s << " validFrom=" << static_cast<std::string>(this->validFrom) << std::endl;
        s << " validTo=" << static_cast<std::string>(this->validTo) << std::endl;
        s << " PRN=" << this->prn << std::endl;
        s << " comment=" << this->comment << std::endl;
    }

    operator std::string() const override
    {
        try
        {
            std::ostringstream ss;
            ss << gnsstk::Sinex::DATA_START;
            ss << gnsstk::Sinex::formatStr(this->svn, 4);
            ss << ' ' << longSinexTime(this->validFrom);
            ss << ' ' << longSinexTime(this->validTo);
            ss << ' ' << gnsstk::Sinex::formatStr(this->prn, 3);
            ss << ' ' << gnsstk::Sinex::formatStr(this->comment, 40);
            return ss.str();
        }
        catch (gnsstk::Exception& exc)
        {
            GNSSTK_RETHROW(exc);
        }
    }

    void operator=(const std::string& line) override
    {
        static int FIELD_DIVS[] = {0, 5, 20, 35, -1};
        try
        {
            gnsstk::Sinex::isValidLineStructure(line, MIN_LINE_LEN, MAX_LINE_LEN, FIELD_DIVS);
            this->svn = line.substr(1, 4);
            this->validFrom = line.substr(8, 12);
            this->validTo = line.substr(23, 12);
            this->prn = line.substr(36, 3);
            if (line.length() > 40)
                this->comment = line.substr(40, 40);
        }
        catch (gnsstk::Exception& exc)
        {
            GNSSTK_RETHROW(exc);
        }
    }
};
const std::string SatellitePRN::BLOCK_TITLE {"SATELLITE/PRN"};  // NOLINT(runtime/string)

struct SatelliteFrequencyChannel : gnsstk::Sinex::DataType
{
    static const std::string BLOCK_TITLE;
    static constexpr size_t MIN_LINE_LEN = 39;
    static constexpr size_t MAX_LINE_LEN = 80;

    std::string svn;  //!< SVN
    gnsstk::Sinex::Time validFrom;  //!< Time from which the entry is valid
    gnsstk::Sinex::Time validTo;  //!< Time until which the entry is valid (valid up to now if zero)
    int16_t channel {0};  //!< Channel
    std::string comment;  //!< Comment

    SatelliteFrequencyChannel() = default;

    explicit SatelliteFrequencyChannel(const std::string& line, size_t num = 0)
    {
        *this = line;
    }

    void dump(std::ostream& s) const
    {
        s << BLOCK_TITLE << " :" << std::endl;
        s << " svn=" << this->svn << std::endl;
        s << " validFrom=" << static_cast<std::string>(this->validFrom) << std::endl;
        s << " validTo=" << static_cast<std::string>(this->validTo) << std::endl;
        s << " channel=" << this->channel << std::endl;
        s << " comment=" << this->comment << std::endl;
    }

    operator std::string() const override
    {
        try
        {
            std::ostringstream ss;
            ss << gnsstk::Sinex::DATA_START;
            ss << gnsstk::Sinex::formatStr(this->svn, 4);
            ss << ' ' << longSinexTime(this->validFrom);
            ss << ' ' << longSinexTime(this->validTo);
            ss << ' ' << gnsstk::Sinex::formatInt(this->channel, 3);
            ss << ' ' << gnsstk::Sinex::formatStr(this->comment, 40);
            return ss.str();
        }
        catch (gnsstk::Exception& exc)
        {
            GNSSTK_RETHROW(exc);
        }
    }

    void operator=(const std::string& line) override
    {
        static int FIELD_DIVS[] = {0, 5, 20, 35, 39, -1};
        try
        {
            gnsstk::Sinex::isValidLineStructure(line, MIN_LINE_LEN, MAX_LINE_LEN, FIELD_DIVS);
            this->svn = line.substr(1, 4);
            this->validFrom = line.substr(8, 12);
            this->validTo = line.substr(23, 12);
            this->channel = static_cast<int16_t>(gnsstk::StringUtils::asInt(line.substr(36, 3)));
            this->comment = line.substr(40, 40);
        }
        catch (gnsstk::Exception& exc)
        {
            GNSSTK_RETHROW(exc);
        }
    }
};
const std::string SatelliteFrequencyChannel::BLOCK_TITLE {"SATELLITE/FREQUENCY_CHANNEL"};  // NOLINT(runtime/string)

struct IgnoredBlock : public gnsstk::Sinex::DataType
{
    static const std::string BLOCK_TITLE;

    IgnoredBlock() = default;

    explicit IgnoredBlock(const std::string&, size_t = 0)
    {
    }

    void dump(std::ostream&) const
    {
    }

    operator std::string() const override
    {
        return "";
    }

    void operator=(const std::string&) override
    {
    }
};
const std::string IgnoredBlock::BLOCK_TITLE {"IGNORED/BLOCK"};  // NOLINT(runtime/string)

/**
 * \brief Normal gnsstk::Sinex::Block cannot handle comments after block start. This class adds the support.
 */
template<typename T>
class IgsSinexBlock : public gnsstk::Sinex::Block<T>
{
protected:
    size_t getBlock(gnsstk::Sinex::Stream& s) override
    {
        size_t lineNum {0u};
        char c;
        while (s.good())
        {
            c = s.get();
            if (s.good() )
            {
                if (c == gnsstk::Sinex::DATA_START)
                {
                    std::string line;
                    s.formattedGetLine(line);
                    try
                    {
                        this->dataVec.push_back(T(line.insert(0u, 1u, c), lineNum));
                    }
                    catch (gnsstk::Exception& exc)
                    {
                        gnsstk::FFStreamError err(exc);
                        GNSSTK_THROW(err);
                    }
                    ++lineNum;
                }
                // This branch adds the support for in-block comments
                else if (c == gnsstk::Sinex::COMMENT_START)
                {
                    std::string line;
                    s.formattedGetLine(line);
                    ++lineNum;
                }
                else
                {
                    s.putback(c);
                    break;
                }
            }
        }
        return lineNum;
    }

public:
    static gnsstk::Sinex::BlockBase* create() { return new IgsSinexBlock<T>; }
};

/**
 * \brief Customized Sinex Data reader that can read the almost-SINEX satellite metadata from IGS.
 */
class IgsSinexData : public gnsstk::Sinex::Data
{
public:
    IgsSinexData()
    {
        gnsstk::Sinex::Data::initBlockFactory();
        auto& factory = gnsstk::Sinex::Data::blockFactory;
        factory[gnsstk::Sinex::FileReference::BLOCK_TITLE] = IgsSinexBlock<gnsstk::Sinex::FileReference>::create;
        factory[gnsstk::Sinex::FileComment::BLOCK_TITLE] = IgsSinexBlock<gnsstk::Sinex::FileComment>::create;
        factory[SatelliteIdentifier::BLOCK_TITLE] = IgsSinexBlock<SatelliteIdentifier>::create;
        factory[SatellitePRN::BLOCK_TITLE] = IgsSinexBlock<SatellitePRN>::create;
        factory[SatelliteFrequencyChannel::BLOCK_TITLE] = IgsSinexBlock<SatelliteFrequencyChannel>::create;
        factory["SATELLITE/MASS"] = IgsSinexBlock<IgnoredBlock>::create;
        factory["SATELLITE/COM"] = IgsSinexBlock<IgnoredBlock>::create;
        factory["SATELLITE/ECCENTRICITY"] = IgsSinexBlock<IgnoredBlock>::create;
        factory["SATELLITE/TX_POWER"] = IgsSinexBlock<IgnoredBlock>::create;
    }
};

struct GNSSSignal
{
    std::string name;
    double frequency {std::numeric_limits<double>::quiet_NaN()};
    double channelStep {0.0};
};

struct GNSSBlock
{
    std::string name;
    std::list<GNSSSignal> signals;
};

/**
 * \brief PIMPL data holder.
 */
struct IGSSatelliteMetadataPrivate
{
    std::string url {};  //!< URL to download the metadata from.
    std::string cacheFile {};  //!< Path to the cache file.
    //! Max age of the cache file. If it is older, it is redownloaded.
    ros::WallDuration cacheValidity {ros::WallDuration::DAY * 30};

    std::unordered_map<std::string, uint32_t> svnToSatcat;  //!< Maps SVN to Satcat ID.
    std::unordered_map<uint32_t, std::string> satcatToSvn;  //!< Maps Satcat ID to SVN.
    //! Maps SVN to parsed SATELLITE/IDENTIFIER block entry.
    std::unordered_map<std::string, SatelliteIdentifier> svnToSatInfo;
    //! Maps SVN to all parsed SATELLITE/PRN block entries related to the SVN.
    std::unordered_map<std::string, std::list<SatellitePRN>> svnToSatPRN;
    //! Maps SVN to all parsed SATELLITE/FREQUENCY_CHANNEL block entries related to the SVN (GLONASS only).
    std::unordered_map<std::string, std::list<SatelliteFrequencyChannel>> svnToSatChannel;
    //! Maps GNSS block name to all signals transmitted by satellites from the block.
    std::unordered_map<std::string, GNSSBlock> blockSignals;

    /**
     * \brief Download the IGS satellite catalog file to cacheFile.
     * \return Whether the download succeeded.
     */
    bool downloadMetadata() const;

    /**
     * \brief Load the mappings of available satellite signals from YAML configs into blockSignals.
     * \return Whether the loading succeeded.
     */
    bool loadSignals();
};

bool IGSSatelliteMetadataPrivate::downloadMetadata() const
{
    const auto curlCallback = +[](void *contents, size_t size, size_t nmemb, void *userp)
    {
        *static_cast<std::stringstream*>(userp) << std::string(static_cast<char*>(contents), size * nmemb);
        return size * nmemb;
    };

    std::stringstream readBuffer;
    ROS_INFO("Downloading satellite metadata from %s.", this->url.c_str());
    const auto curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, this->url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1u);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    const auto res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK)
    {
        ROS_ERROR("Error downloading %s: %s\n", this->url.c_str(), curl_easy_strerror(res));
        return false;
    }

    std::ofstream outFile(this->cacheFile.c_str());
    for (std::string line; std::getline(readBuffer, line);)
    {
        // gnsstk::Sinex::SinexData has problems with trailing spaces, so we remove them
        cras::stripTrailing(line, '\r');
        if (line.find_first_not_of(' ') != std::string::npos)
            while (!line.empty() && line.back() == ' ')
                cras::stripTrailing(line, ' ');
        // gnsstk::Sinex::SinexData has problems with UTF-8 characters in comments, so we remove them.
        auto printable {true};
        for (const auto c : line)
        {
            if (!std::isprint(c))
            {
                printable = false;
                break;
            }
        }
        if (printable)
            outFile << line << "\r\n";
    }
    ROS_INFO("Saved satellite metadata to %s.", this->cacheFile.c_str());

    return true;
}

bool IGSSatelliteMetadataPrivate::loadSignals()
{
    if (!this->blockSignals.empty())
        return true;

    const auto defaultDir = ros::package::getPath(ROS_PACKAGE_NAME) + "/data/constellations";
    const auto signalsDirEnv = std::getenv("GNSS_INFO_SIGNALS_PATH");
    const auto dir = (signalsDirEnv == nullptr) ? defaultDir : signalsDirEnv;

    std::unordered_map<std::string, GNSSSignal> signals;

    for (const auto& d : cras::split(dir, ":"))
    {
        for (boost::filesystem::directory_iterator it(d); it != boost::filesystem::directory_iterator(); ++it)
        {
            if (!boost::filesystem::is_regular_file(*it))
                continue;
            const auto path = it->path().string();
            if (!cras::endsWith(path, ".yaml"))
                continue;
            try
            {
                auto yaml = YAML::LoadFile(path);
                if (yaml["signals"] && yaml["signals"].IsMap())
                {
                    for (const auto& constellationAndData : yaml["signals"])
                    {
                        if (!constellationAndData.second.IsMap())
                            continue;
                        for (const auto& s : constellationAndData.second)
                        {
                            if (!s.second.IsMap() || !s.first.IsScalar() || !s.second["frequency"])
                                continue;
                            const auto signalName = s.first.as<std::string>();
                            GNSSSignal signal{signalName, 0.0, 0.0};
                            signal.frequency = s.second["frequency"].as<double>();
                            if (s.second["channel_step"])
                                signal.channelStep = s.second["channel_step"].as<double>();
                            signals[signalName] = signal;
                        }
                    }
                }
                if (yaml["blocks"] && yaml["blocks"].IsMap())
                {
                    for (const auto& constellationAndData : yaml["blocks"])
                    {
                        if (!constellationAndData.second.IsMap())
                            continue;
                        for (const auto& s : constellationAndData.second)
                        {
                            if (!s.second.IsMap() || !s.first.IsScalar() || !s.second["signals"] ||
                                !s.second["signals"].IsSequence())
                            {
                                continue;
                            }
                            const auto blockName = s.first.as<std::string>();
                            GNSSBlock block{blockName, {}};
                            for (const auto& signalName : s.second["signals"])
                            {
                                if (!signalName.IsScalar())
                                    continue;
                                const auto name = signalName.as<std::string>();
                                if (signals.find(name) == signals.end())
                                    continue;
                                block.signals.push_back(signals[name]);
                            }
                            this->blockSignals[blockName] = block;
                        }
                    }
                }
            }
            catch (const YAML::Exception& e)
            {
                ROS_ERROR("Error loading YAML file %s: %s.", path.c_str(), e.what());
            }
        }
    }
    return !signals.empty();
}

IGSSatelliteMetadata::IGSSatelliteMetadata() : data(new IGSSatelliteMetadataPrivate)
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

    this->data->cacheFile = cacheDir + "/igs_satellite_metadata.snx";

    this->data->url = "https://files.igs.org/pub/station/general/igs_satellite_metadata.snx";
    const auto envUrl = std::getenv("GNSS_INFO_IGS_METADATA_URL");
    if (envUrl != nullptr)
        this->data->url = envUrl;
}

IGSSatelliteMetadata::~IGSSatelliteMetadata() = default;

void IGSSatelliteMetadata::setMetadataURL(const std::string& url)
{
    this->data->url = url;
}

void IGSSatelliteMetadata::setCacheFile(const std::string& file)
{
    this->data->cacheFile = file;
}

void IGSSatelliteMetadata::setCacheValidity(const ros::WallDuration& validity)
{
    this->data->cacheValidity = validity;
}

bool IGSSatelliteMetadata::load()
{
    const auto oldestValidCache = static_cast<time_t>((ros::WallTime::now() - this->data->cacheValidity).sec);
    if (!boost::filesystem::exists(this->data->cacheFile) ||
        oldestValidCache > boost::filesystem::last_write_time(this->data->cacheFile))
    {
        if (!this->data->downloadMetadata())
            return false;
    }

    IgsSinexData igsSinexData;
    try
    {
        gnsstk::Sinex::Stream input(this->data->cacheFile.c_str());
        input.exceptions(std::fstream::eofbit | std::fstream::failbit);
        input >> igsSinexData;
    }
    catch (const gnsstk::Exception& e)
    {
        ROS_ERROR("Error loading satellite metadata from %s: %s.", this->data->cacheFile.c_str(), e.what().c_str());
        return false;
    }

    for (const auto& block : igsSinexData.blocks)
    {
        if (dynamic_cast<const gnsstk::Sinex::Block<SatelliteIdentifier>*>(block))
        {
            const auto cb = dynamic_cast<const gnsstk::Sinex::Block<SatelliteIdentifier>*>(block);
            for (const auto& entry : const_cast<gnsstk::Sinex::Block<SatelliteIdentifier>*>(cb)->getData())
            {
                this->data->svnToSatcat[entry.svn] = entry.satcatID;
                this->data->satcatToSvn[entry.satcatID] = entry.svn;
                this->data->svnToSatInfo[entry.svn] = entry;
            }
        }
        else if (dynamic_cast<const gnsstk::Sinex::Block<SatellitePRN>*>(block))
        {
            const auto cb = dynamic_cast<const gnsstk::Sinex::Block<SatellitePRN>*>(block);
            for (const auto& entry : const_cast<gnsstk::Sinex::Block<SatellitePRN>*>(cb)->getData())
            {
                this->data->svnToSatPRN[entry.svn].push_back(entry);
            }
        }
        else if (dynamic_cast<const gnsstk::Sinex::Block<SatelliteFrequencyChannel>*>(block))
        {
            const auto cb = dynamic_cast<const gnsstk::Sinex::Block<SatelliteFrequencyChannel>*>(block);
            for (const auto& entry : const_cast<gnsstk::Sinex::Block<SatelliteFrequencyChannel>*>(cb)->getData())
            {
                this->data->svnToSatChannel[entry.svn].push_back(entry);
            }
        }
    }

    ROS_INFO("Satellite metadata loaded from %s.", this->data->cacheFile.c_str());
    return true;
}

ros::Time sinexTimeToRosTime(const gnsstk::Sinex::Time& t)
{
    if (t.year == 0 && t.doy == 0 && t.sod == 0)  // special meaning of all zeros
        return ros::Time::MAX;
    return gnsstk_ros::convert(t);
}

std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>
IGSSatelliteMetadata::getSatellites(const ros::Time& time, const bool onlyActive,
                                    const std::unordered_set<std::string>& onlyConstellations,
                                    const std::unordered_set<std::string>& onlySignals)
{
    std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> result;

    for (const auto& [svn, satcatID] : this->data->svnToSatcat)
    {
        auto maybeSatellite = this->getSatellite(satcatID, time);
        if (maybeSatellite && (!onlyActive || maybeSatellite->active))
        {
            if (!onlyConstellations.empty())
            {
                if (onlyConstellations.find(maybeSatellite->constellation) == onlyConstellations.cend())
                    continue;
            }
            if (!onlySignals.empty())
            {
                maybeSatellite->signals.erase(std::remove_if(
                    maybeSatellite->signals.begin(), maybeSatellite->signals.end(),
                    [&onlySignals](const gnss_info_msgs::SatelliteSignal& s)
                    {
                        return onlySignals.find(s.name) == onlySignals.cend();
                    }),
                    maybeSatellite->signals.end());
                if (maybeSatellite->signals.empty())
                    continue;
            }
            result[satcatID] = *maybeSatellite;
        }
    }

    return result;
}

cras::optional<gnss_info_msgs::SatelliteInfo> IGSSatelliteMetadata::getSatellite(
    const uint32_t& satcatID, const ros::Time& time)
{
    if (this->data->satcatToSvn.find(satcatID) == this->data->satcatToSvn.cend())
        return cras::nullopt;

    const auto svn = this->data->satcatToSvn[satcatID];
    const auto& satinfo = this->data->svnToSatInfo[svn];

    gnss_info_msgs::SatelliteInfo msg;
    msg.stamp = time;
    msg.active = false;
    msg.satcat_id = satcatID;
    msg.name = cras::split(satinfo.comment, ";", 1).back();
    cras::strip(msg.name);
    const auto maybeConstellation = gnsstk_ros::getRosConstellationFromSVN(svn);
    msg.constellation = maybeConstellation ? *maybeConstellation : "";

    // Find PRN valid for the given time
    for (const auto& prnItem : this->data->svnToSatPRN[svn])
    {
        const auto validFrom = sinexTimeToRosTime(prnItem.validFrom);
        const auto validTo = sinexTimeToRosTime(prnItem.validTo);
        if (validFrom <= time && time <= validTo)
        {
            msg.prn = prnItem.prn;
            msg.active = true;
            break;
        }
    }

    // Fill info about transmitted signals.
    this->data->loadSignals();
    const auto& blockName = this->data->svnToSatInfo[svn].block;
    if (this->data->blockSignals.find(blockName) != this->data->blockSignals.cend())
    {
        int16_t channel{0};
        if (msg.constellation == gnss_info_msgs::Enums::CONSTELLATION_GLONASS)
        {
            for (const auto& satChannelItem : this->data->svnToSatChannel[svn])
            {
                const auto validFrom = sinexTimeToRosTime(satChannelItem.validFrom);
                const auto validTo = sinexTimeToRosTime(satChannelItem.validTo);
                if (validFrom <= time && time <= validTo)
                {
                    channel = satChannelItem.channel;
                    break;
                }
            }
        }
        for (const auto& signal : this->data->blockSignals[blockName].signals)
        {
            gnss_info_msgs::SatelliteSignal sigMsg;
            sigMsg.constellation = msg.constellation;
            sigMsg.name = signal.name;
            sigMsg.frequency = static_cast<float>(signal.frequency + channel * signal.channelStep);
            msg.signals.push_back(sigMsg);
        }
    }

    return msg;
}

cras::optional<gnss_info_msgs::SatelliteInfo> IGSSatelliteMetadata::getSatelliteByPRN(
    const std::string& prn, const ros::Time& time)
{
    for (const auto& [svn, satPRN] : this->data->svnToSatPRN)
    {
        for (const auto& prnItem : this->data->svnToSatPRN[svn])
        {
            if (prnItem.prn != prn)
                continue;
            const auto validFrom = sinexTimeToRosTime(prnItem.validFrom);
            const auto validTo = sinexTimeToRosTime(prnItem.validTo);
            if (validFrom <= time && time <= validTo)
                return this->getSatellite(this->data->svnToSatcat[svn], time);
        }
    }
    return cras::nullopt;
}

cras::optional<gnss_info_msgs::SatelliteInfo> IGSSatelliteMetadata::getSatelliteByPRN(
    const int32_t prn, const std::string& constellation, const ros::Time& time)
{
    const auto prnString = gnsstk_ros::prnIntToString(prn, constellation);
    if (!prnString.has_value() || !prnString->empty())
        return cras::nullopt;

    return this->getSatelliteByPRN(*prnString, time);
}

}
