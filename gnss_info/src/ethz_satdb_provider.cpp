// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <ctime>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <curl/curl.h>
#include <gnsstk/MultiFormatNavDataFactory.hpp>
#include <gnsstk/NavDataFactoryStoreCallback.hpp>
#include <gnsstk/NavDataFactoryWithStoreFile.hpp>
#include <gnsstk/NavLibrary.hpp>
#include <gnsstk/OrbitData.hpp>
#include <gnsstk/Position.hpp>
#include <gnsstk/YDSTime.hpp>
#include <jsoncpp/json/json.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <gnss_info/ethz_satdb_provider.h>
#include <gnss_info/igs_satellite_metadata.h>
#include <gnss_info_msgs/Enums.h>
#include <ros/ros.h>

#include "tle.h"  // NOLINT

namespace gnss_info
{

struct DayIndex
{
    uint16_t year;
    uint8_t month;
    uint8_t day;

    explicit DayIndex(const ros::Time& time)
    {
        const time_t nowSec = time.sec;
        const auto t = *std::gmtime(&nowSec);
        this->year = t.tm_year + 1900;  // struct tm year is relative to year 1900
        this->month = t.tm_mon + 1;
        this->day = t.tm_mday;
    }

    DayIndex(const uint16_t year, const uint8_t month, const uint8_t day) :
        year(year), month(month), day(day)
    {
    }

    explicit operator ros::Time() const
    {
        struct tm start{};
        start.tm_year = this->year - 1900;
        start.tm_mon = this->month - 1;
        start.tm_mday = this->day;
        return ros::Time(std::mktime(&start), 0);
    }

    bool operator==(const DayIndex& other) const
    {
        return this->year == other.year && this->month == other.month && this->day == other.day;
    }
};

}

template <>
struct std::hash<gnss_info::DayIndex>
{
    std::size_t operator()(const gnss_info::DayIndex& k) const noexcept
    {
        return ((std::hash<uint16_t>()(k.year) ^ (std::hash<uint8_t>()(k.month) << 1)) >> 1)
            ^ (std::hash<uint8_t>()(k.day) << 1);
    }
};

namespace gnsstk
{

ros::Time gnssTimeToRosTime(const CommonTime& commonTime)
{
    long days, secs, daysOffset;  // NOLINT(runtime/int)
    double frac;
    YDSTime(1970, 0, 0.0).convertToCommonTime().get(daysOffset, secs, frac);
    commonTime.get(days, secs, frac);
    return ros::Time() +
        ros::Duration::DAY * (days - daysOffset) +
        ros::Duration::SECOND * secs +
        ros::Duration::SECOND * frac;
}

CommonTime rosTimeToGnssTime(const ros::Time& rosTime)
{
    auto commonTime = YDSTime(1970, 0, 0.0).convertToCommonTime();
    commonTime.addDays(rosTime.sec / ros::Duration::DAY.sec);
    commonTime.addSeconds(static_cast<long>(rosTime.sec % ros::Duration::DAY.sec));  // NOLINT(runtime/int)
    commonTime.addSeconds(rosTime.nsec * 1e-9);
    return commonTime;
}

cras::optional<SatID> satelliteInfoToSatID(const gnss_info_msgs::SatelliteInfo& info)
{
    const auto maybeConstellationAndPrn = gnss_info::prnStringToInt(info.prn);
    if (!maybeConstellationAndPrn.has_value())
        return cras::nullopt;
    const auto& [prn, constellationStr] = *maybeConstellationAndPrn;
    SatelliteSystem system;
    if (constellationStr == gnss_info_msgs::Enums::CONSTELLATION_GPS)
        system = SatelliteSystem::GPS;
    else if (constellationStr == gnss_info_msgs::Enums::CONSTELLATION_GALILEO)
        system = SatelliteSystem::Galileo;
    else if (constellationStr == gnss_info_msgs::Enums::CONSTELLATION_GLONASS)
        system = SatelliteSystem::Glonass;
    else if (constellationStr == gnss_info_msgs::Enums::CONSTELLATION_BEIDOU)
        system = SatelliteSystem::BeiDou;
    else if (constellationStr == gnss_info_msgs::Enums::CONSTELLATION_NAVIC)
        system = SatelliteSystem::IRNSS;
    else if (constellationStr == gnss_info_msgs::Enums::CONSTELLATION_QZSS)
        system = SatelliteSystem::QZSS;
    else
        return cras::nullopt;
    return SatID(prn, system);
}

class TLEOrbitData : public OrbitData
{
public:
    explicit TLEOrbitData(const tled_t& tle) : tle(tle)
    {
    }

    bool isSameData(const NavDataPtr& right) const override
    {
        const auto rhs = std::dynamic_pointer_cast<TLEOrbitData>(right);
        if (rhs == nullptr)
            return false;

        return NavData::isSameData(right) &&
            this->tle.name == rhs->tle.name &&
            this->tle.alias == rhs->tle.alias &&
            this->tle.satno == rhs->tle.satno &&
            this->tle.satclass == rhs->tle.satclass &&
            this->tle.desig == rhs->tle.desig &&
            this->tle.epoch.sec == rhs->tle.epoch.sec &&
            this->tle.epoch.time == rhs->tle.epoch.time &&
            this->tle.ndot == rhs->tle.ndot &&
            this->tle.nddot == rhs->tle.nddot &&
            this->tle.bstar == rhs->tle.bstar &&
            this->tle.etype == rhs->tle.etype &&
            this->tle.eleno == rhs->tle.eleno &&
            this->tle.inc == rhs->tle.inc &&
            this->tle.OMG == rhs->tle.OMG &&
            this->tle.ecc == rhs->tle.ecc &&
            this->tle.omg == rhs->tle.omg &&
            this->tle.M == rhs->tle.M &&
            this->tle.n == rhs->tle.n &&
            this->tle.rev == rhs->tle.rev;
    }

    std::list<std::string> compare(const NavDataPtr& right) const override
    {
        auto res = NavData::compare(right);
        const auto rhs = std::dynamic_pointer_cast<TLEOrbitData>(right);
        if (rhs == nullptr)
        {
            res.emplace_back("CLASS");
            return res;
        }

        if (strcmp(this->tle.name, rhs->tle.name) != 0)
            res.emplace_back("name");
        if (strcmp(this->tle.alias, rhs->tle.alias) != 0)
            res.emplace_back("alias");
        if (strcmp(this->tle.satno, rhs->tle.satno) != 0)
            res.emplace_back("satno");
        if (this->tle.satclass != rhs->tle.satclass)
            res.emplace_back("satclass");
        if (strcmp(this->tle.desig, rhs->tle.desig) != 0)
            res.emplace_back("desig");
        if (this->tle.epoch.sec != rhs->tle.epoch.sec || this->tle.epoch.time != rhs->tle.epoch.time)
            res.emplace_back("epoch");
        if (this->tle.ndot != rhs->tle.ndot)
            res.emplace_back("ndot");
        if (this->tle.nddot != rhs->tle.nddot)
            res.emplace_back("nddot");
        if (this->tle.bstar != rhs->tle.bstar)
            res.emplace_back("bstar");
        if (this->tle.etype != rhs->tle.etype)
            res.emplace_back("etype");
        if (this->tle.eleno != rhs->tle.eleno)
            res.emplace_back("eleno");
        if (this->tle.inc != rhs->tle.inc)
            res.emplace_back("inc");
        if (this->tle.OMG != rhs->tle.OMG)
            res.emplace_back("OMG");
        if (this->tle.ecc != rhs->tle.ecc)
            res.emplace_back("ecc");
        if (this->tle.omg != rhs->tle.omg)
            res.emplace_back("omg");
        if (this->tle.M != rhs->tle.M)
            res.emplace_back("M");
        if (this->tle.n != rhs->tle.n)
            res.emplace_back("n");
        if (this->tle.rev != rhs->tle.rev)
            res.emplace_back("rev");

        return res;
    }

    NavDataPtr clone() const override
    {
        return std::make_shared<TLEOrbitData>(this->tle);
    }

    bool validate() const override
    {
        return true;
    }

    bool getXvt(const CommonTime& when, Xvt& xvt, const ObsID& oid) override
    {
        const tle_t tleList{1, 1, &this->tle};

        double rs[6];
        const auto rosWhen = gnssTimeToRosTime(when);
        gtime_t gtime {rosWhen.sec, rosWhen.nsec * 1e-9};
        if (!tle_pos(utc2gpst(gtime), "", this->tle.satno, "", &tleList, nullptr, rs))
        {
            ROS_WARN_THROTTLE(1.0, "Failed to compute ECEF position of satellite %s (%s) at time %s.",
                              this->tle.satno, this->tle.name, cras::to_string(rosWhen).c_str());
            return false;
        }

        Position pos {rs, Position::CoordinateSystem::Cartesian};
        if (pos.radius() < pos.radiusEarth())
        {
            ROS_WARN_THROTTLE(1.0, "Invalid satellite position. Satellite %s (%s) is underground at time %s!",
                              this->tle.satno, this->tle.name, cras::to_string(rosWhen).c_str());
            return false;
        }

        xvt.x = Triple(rs[0], rs[1], rs[2]);
        xvt.v = Triple(rs[3], rs[4], rs[5]);
        xvt.frame = RefFrameRlz::WGS84G0;
        xvt.health = Xvt::Healthy;

        return true;
    }

private:
    tled_t tle;
};

class TLENavDataFactory : public NavDataFactoryWithStoreFile
{
public:
    TLENavDataFactory()
    {
        const ObsID obs {ObservationType::NavMsg, CarrierBand::Any, TrackingCode::Any};
        for (const auto& satSystem : gnsstk::SatelliteSystemIterator())
            this->supportedSignals.insert(NavSignalID(satSystem, obs, NavType::Any));
    }

    static void addSatelliteInfo(const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satelliteInfo)
    {
        TLENavDataFactory::satelliteInfo.insert(satelliteInfo.begin(), satelliteInfo.end());
    }

    bool loadIntoMap(const std::string& filename, NavMessageMap& navMap, NavNearMessageMap& navNearMap,
                     OffsetCvtMap& ofsMap) override
    {
        NavDataFactoryStoreCallback cb(this, navMap, navNearMap, ofsMap);
        return this->process(filename, cb);
    }

    bool process(const std::string& filename, NavDataFactoryCallback& cb) override
    {
        if (!this->procNavTypes.empty() && this->procNavTypes.count(NavMessageType::Almanac) == 0)
            return true;

        if (this->navValidity == NavValidityType::InvalidOnly)
            return true;

        if (this->processedFiles.find(filename) != this->processedFiles.end())
            return true;

        this->processedFiles.insert(filename);

        tle_t tles{};
        if (!tle_read(filename.c_str(), &tles))
        {
            ROS_ERROR("Failed reading TLE file %s.", filename.c_str());
            return false;
        }

        const std::vector<tled_t> tlesVec(tles.data, tles.data + tles.n);
        std::free(tles.data);

        for (const auto& tle : tlesVec)
        {
            std::shared_ptr<TLEOrbitData> alm;
            if (!this->convertToOrbit(tle, alm))
                return false;
            if (alm != nullptr && !cb.process(alm))
                return false;
        }

        return true;
    }

    std::string getFactoryFormats() const override
    {
        if (this->procNavTypes.empty() || this->procNavTypes.count(NavMessageType::Almanac) > 0)
            return "TLE";
        return "";
    }

    bool convertToOrbit(const tled_t& navIn, std::shared_ptr<TLEOrbitData>& navOut)
    {
        try
        {
            // Remove leading zeros from satno, otherwise it would be interpreted as octal.
            auto satno = std::string(navIn.satno);
            while (!satno.empty() && satno[0] == '0')
                satno = satno.substr(1);
            const auto satcatID = cras::parseUInt32(satno);
            if (TLENavDataFactory::satelliteInfo.find(satcatID) == TLENavDataFactory::satelliteInfo.cend())
                return true;

            const auto& info = TLENavDataFactory::satelliteInfo[satcatID];
            const auto satId = satelliteInfoToSatID(info);
            if (!satId.has_value())
                return false;

            navOut = std::make_shared<TLEOrbitData>(navIn);
            navOut->signal.sat = navOut->signal.xmitSat = *satId;
            navOut->signal.system = satId->system;
            navOut->signal.obs = ObsID(ObservationType::NavMsg, CarrierBand::Any, TrackingCode::Any);
            navOut->signal.nav = NavType::Any;
            navOut->signal.messageType = NavMessageType::Almanac;
            ros::Time epoch(navIn.epoch.time, static_cast<uint32_t>(navIn.epoch.sec * 1e9));
            navOut->timeStamp = rosTimeToGnssTime(epoch);

            return true;
        }
        catch (const std::invalid_argument&)
        {
            return false;
        }
    }

protected:
    static std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> satelliteInfo;
    std::unordered_set<std::string> processedFiles;
};

std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> TLENavDataFactory::satelliteInfo;

}

namespace gnss_info
{

struct EthzSatdbProviderPrivate
{
    std::pair<ros::Time, ros::Time> timeRange;
    std::unordered_set<std::string> constellations;
    std::unordered_map<std::string, std::string> nameToSatcat;
    std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> satelliteInfo;
    gnsstk::NavDataFactoryPtr factory;
    gnsstk::NavLibrary navLibrary;
    std::unordered_set<DayIndex> preloadedDays;

    std::string ecefFrameId {"ecef"};
    std::string urlFormat {"https://satdb.ethz.ch/api/satellitedata/?object-name=%s&"
                           "start-datetime=%sT0000&end-datetime=%sT0000&before=3&after=3&"
                           "without-frequency-data=True"};

    size_t maxDownloadDays {1000u};
    boost::filesystem::path cacheDir;

    std::unordered_map<std::string, std::string> satdbConstellations {
        {gnss_info_msgs::Enums::CONSTELLATION_GPS, "NAVSTAR"},
        {gnss_info_msgs::Enums::CONSTELLATION_GALILEO, "GSAT0"},
        {gnss_info_msgs::Enums::CONSTELLATION_BEIDOU, "BEIDOU"},
        {gnss_info_msgs::Enums::CONSTELLATION_GLONASS, "COSMOS"}
    };

    boost::filesystem::path getCacheFile(const DayIndex& day) const;
    cras::expected<bool, std::string> download(const DayIndex& day);
};

boost::filesystem::path EthzSatdbProviderPrivate::getCacheFile(const DayIndex& day) const
{
    const auto filename = cras::format("ethz_satdb_%04u%02u%02u.tle", day.year, day.month, day.day);
    return this->cacheDir / filename;
}

cras::expected<bool, std::string> EthzSatdbProviderPrivate::download(const DayIndex& day)
{
    const auto startDate = cras::format("%04u%02u%02u", day.year, day.month, day.day);
    const DayIndex endDay(static_cast<ros::Time>(day) + ros::Duration::DAY);
    const auto endDate = cras::format("%04u%02u%02u", endDay.year, endDay.month, endDay.day);

    const auto cacheFile = this->getCacheFile(day);
    if (boost::filesystem::exists(cacheFile))
        return true;

    std::stringstream ss;
    size_t numSatellites {0u};
    for (const auto& [constallation, prefix] : this->satdbConstellations)
    {
        const auto curlCallback = +[](void *contents, size_t size, size_t nmemb, void *userp)
        {
            *static_cast<std::stringstream*>(userp) << std::string(static_cast<char*>(contents), size * nmemb);
            return size * nmemb;
        };

        std::stringstream readBuffer;
        const auto url = cras::format(urlFormat, prefix.c_str(), startDate.c_str(), endDate.c_str());
        const auto curl = curl_easy_init();
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1u);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        ROS_INFO("Downloading orbits from %s.", url.c_str());
        const auto res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK)
            return cras::make_unexpected(
                cras::format("Error downloading %s: %s", url.c_str(), curl_easy_strerror(res)));

        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(readBuffer, root))
        {
            return cras::make_unexpected(cras::format("Error parsing data downloaded from URL %s: %s.",
                                                      url.c_str(), reader.getFormattedErrorMessages().c_str()));
        }

        const auto& results = root["results"];
        for (int index = 0; index < results.size(); ++index)
            ss << cras::replace(results[index]["norad_str"].asString(), "\\n", "\n") << std::endl;
        numSatellites++;
    }

    if (numSatellites == 0)
        return cras::make_unexpected("Determining satellite orbits failed.");

    try
    {
        std::ofstream outFile;
        outFile.open(cacheFile.c_str());
        outFile << ss.rdbuf();
    }
    catch (const std::exception& e)
    {
        return cras::make_unexpected(e.what());
    }

    ROS_DEBUG("Saved orbits to %s.", cacheFile.c_str());
    return true;
}

EthzSatdbProvider::EthzSatdbProvider(const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satelliteInfo) :
    data(new EthzSatdbProviderPrivate)
{
    this->data->satelliteInfo = satelliteInfo;
    this->data->timeRange = std::make_pair(static_cast<ros::Time>(DayIndex(2023, 2, 10)), ros::Time::MAX);

    this->data->constellations =
    {
        gnss_info_msgs::Enums::CONSTELLATION_GPS,
        gnss_info_msgs::Enums::CONSTELLATION_GLONASS,
        gnss_info_msgs::Enums::CONSTELLATION_GALILEO,
        gnss_info_msgs::Enums::CONSTELLATION_BEIDOU,
    };

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

    this->data->cacheDir = cacheDir;

    gnsstk::TLENavDataFactory::addSatelliteInfo(satelliteInfo);
    static auto registered {false};
    if (!registered)
    {
        gnsstk::NavDataFactoryPtr factory = std::make_shared<gnsstk::TLENavDataFactory>();
        gnsstk::MultiFormatNavDataFactory::addFactory(factory);
        registered = true;
    }
    this->data->factory = std::make_shared<gnsstk::MultiFormatNavDataFactory>();
    this->data->navLibrary.addFactory(this->data->factory);
}

EthzSatdbProvider::~EthzSatdbProvider()
{
}

std::pair<ros::Time, ros::Time> EthzSatdbProvider::getTimeRange() const
{
    return this->data->timeRange;
}

std::unordered_set<std::string> EthzSatdbProvider::getConstellations() const
{
    return this->data->constellations;
}

bool EthzSatdbProvider::preload(const ros::Time& startTime, const ros::Time& endTime)
{
    auto endTime2 = endTime;
    if (endTime == ros::Time::MAX || (endTime - startTime) < ros::Duration::DAY)
        endTime2 = startTime + ros::Duration::DAY;

    std::list<DayIndex> days;
    for (auto time = startTime; time <= endTime2; time += ros::Duration::DAY)
        days.emplace_back(time);
    days.remove_if([this](const auto& day) {return this->data->preloadedDays.count(day) > 0;});

    if (days.empty())
        return true;

    const auto numDays = days.size();
    if (numDays > 2 * this->data->maxDownloadDays)
    {
        ROS_ERROR("Not loading %zu orbits, it is too much.", numDays);
        return false;
    }

    size_t numToDownload {0u};
    for (const auto& day : days)
    {
        const auto file = this->data->getCacheFile(day);
        if (!boost::filesystem::exists(file))
            numToDownload++;
    }

    if (numToDownload > this->data->maxDownloadDays)
    {
        ROS_ERROR("Not downloading %zu orbits, it is too much.", numToDownload);
        return false;
    }

    if (numToDownload > 0)
    {
        const auto logLevel = (numToDownload < 10 ? ros::console::levels::Info : (
            numToDownload < 365 ? ros::console::levels::Warn : ros::console::levels::Error));
        ROS_LOG(logLevel, ROSCONSOLE_DEFAULT_NAME, "Satellite orbits for %zu day(s) are about to be downloaded.",
                numToDownload);
    }
    else
    {
        const auto logLevel = (numDays < 10 ? ros::console::levels::Info : (
            numDays < 365 ? ros::console::levels::Warn : ros::console::levels::Error));
        ROS_LOG(logLevel, ROSCONSOLE_DEFAULT_NAME, "Satellite orbits for %zu day(s) are about to be loaded.",
                numDays);
    }

    auto hadError {false};
    for (const auto& day : days)
    {
        const auto file = this->data->getCacheFile(day);
        if (!boost::filesystem::exists(file))
        {
            const auto maybeDownloaded = this->data->download(day);
            if (!maybeDownloaded.has_value())
            {
                ROS_ERROR("%s", maybeDownloaded.error().c_str());
                hadError = true;
                continue;
            }
        }

        if (!this->data->factory->addDataSource(file.string()))
        {
            ROS_ERROR("Error processing TLE data file %s.", file.c_str());
            hadError = true;
            continue;
        }

        this->data->preloadedDays.insert(day);
    }
    return hadError;
}

cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>, std::string>
EthzSatdbProvider::getECEFPositions(
    const ros::Time& time, const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satellites)
{
    const auto when = gnsstk::rosTimeToGnssTime(time);

    auto when1 = when, when2 = when;
    when1.addSeconds(-ros::Duration::DAY.sec * 3.0);
    when2.addSeconds(ros::Duration::DAY.sec * 3.0);
    const auto availableSats = this->data->navLibrary.getAvailableSats(when1, when2);
    if (availableSats.empty())
        return cras::make_unexpected("No matching satellite data found.");

    std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition> positions;
    if (satellites.empty())
        return positions;

    std::list<std::string> errors;
    for (const auto& [satcatID, info] : satellites)
    {
        const auto maybeSatID = gnsstk::satelliteInfoToSatID(info);
        if (!maybeSatID.has_value())
        {
            errors.push_back(cras::format("Failed to determine PRN of satellite %u (%s) at time %s.",
                satcatID, info.name.c_str(), cras::to_string(time).c_str()));
            continue;
        }

        const gnsstk::NavSatelliteID navId{*maybeSatID};
        if (availableSats.find(navId) == availableSats.cend())
            continue;

        gnsstk::Xvt xvt;
        const auto success = this->data->navLibrary.getXvt(navId, when, xvt,
            gnsstk::SVHealth::Any, gnsstk::NavValidityType::ValidOnly, gnsstk::NavSearchOrder::Nearest);

        if (!success)
        {
            errors.push_back(cras::format("Failed to compute ECEF position of satellite %u (%s) at time %s.",
                satcatID, info.name.c_str(), cras::to_string(time).c_str()));
            continue;
        }

        gnss_info_msgs::SatellitePosition position;
        position.satcat_id = satcatID;
        position.position.x = xvt.x[0];
        position.position.y = xvt.x[1];
        position.position.z = xvt.x[2];
        position.velocity.x = xvt.v[0];
        position.velocity.y = xvt.v[1];
        position.velocity.z = xvt.v[2];
        positions[satcatID] = position;
    }

    if (!errors.empty() && positions.empty())
        return cras::make_unexpected(cras::join(errors, " "));
    if (positions.empty())
        return cras::make_unexpected("No matching satellite data found.");
    if (!errors.empty())
        ROS_WARN("%s", cras::join(errors, " ").c_str());

    return positions;
}

cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>, std::string>
EthzSatdbProvider::getSkyView(
    const ros::Time& time, const geographic_msgs::GeoPoint& receiverPosition, const double elevationMaskDeg,
    const std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>& satelliteECEFPositions)
{
    const auto& Geodetic = gnsstk::Position::CoordinateSystem::Geodetic;
    const auto& Cartesian = gnsstk::Position::CoordinateSystem::Cartesian;

    gnsstk::Position recPos{receiverPosition.latitude, receiverPosition.longitude, receiverPosition.altitude, Geodetic};
    recPos.transformTo(Cartesian);  // all following computations use Cartesian internally

    std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition> skyView;
    for (const auto& [satcatID, ecefPose] : satelliteECEFPositions)
    {
        gnsstk::Position satPos {ecefPose.position.x, ecefPose.position.y, ecefPose.position.z, Cartesian};
        const auto elDeg = recPos.elevation(satPos);
        if (elDeg < elevationMaskDeg)
            continue;
        const auto azDeg = recPos.azimuth(satPos);
        const auto distance = gnsstk::range(recPos, satPos);
        auto& position = skyView[satcatID];
        position.satcat_id = satcatID;
        position.azimuth_deg = azDeg;
        position.elevation_deg = elDeg;
        position.distance = distance;
    }

    return skyView;
}

bool EthzSatdbProvider::isPrecise() const
{
    return false;
}

bool EthzSatdbProvider::isApproximate() const
{
    return true;
}

std::string EthzSatdbProvider::getName() const
{
    return "ETH Zurich Satellite DB";
}

}
