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
#include CXX_FILESYSTEM_INCLUDE

#include <gnsstk/MultiFormatNavDataFactory.hpp>
#include <gnsstk/NavDataFactoryStoreCallback.hpp>
#include <gnsstk/NavDataFactoryWithStoreFile.hpp>
#include <gnsstk/NavLibrary.hpp>
#include <gnsstk/OrbitData.hpp>
#include <gnsstk/Position.hpp>
#include <jsoncpp/json/json.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <gnss_info/cache.h>
#include <gnss_info/cache_index.h>
#include <gnss_info/ethz_satdb_datasource.h>
#include <gnss_info_msgs/Enums.h>
#include <gnsstk_ros/constellations.h>
#include <gnsstk_ros/time.h>
#include <ros/ros.h>

#include "tle.h"  // NOLINT

namespace fs = CXX_FILESYSTEM_NAMESPACE;

namespace gnsstk
{

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
        const auto rosWhen = gnsstk_ros::convert(when);
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
        for (const auto& satSystem : SatelliteSystemIterator())
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
            const auto satId = gnsstk_ros::satelliteInfoToSatID(info);
            if (!satId.has_value())
                return false;

            navOut = std::make_shared<TLEOrbitData>(navIn);
            navOut->signal.sat = navOut->signal.xmitSat = *satId;
            navOut->signal.system = satId->system;
            navOut->signal.obs = ObsID(ObservationType::NavMsg, CarrierBand::Any, TrackingCode::Any);
            navOut->signal.nav = NavType::Any;
            navOut->signal.messageType = NavMessageType::Almanac;
            ros::Time epoch(navIn.epoch.time, static_cast<uint32_t>(navIn.epoch.sec * 1e9));
            navOut->timeStamp = gnsstk_ros::convert(epoch);

            return true;
        }
        catch (const std::invalid_argument&)
        {
            return false;
        }
    }

protected:
    static std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> satelliteInfo;
};

std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> TLENavDataFactory::satelliteInfo;

}

namespace gnss_info
{

struct EthzSatdbDataSourcePrivate
{
    std::pair<ros::Time, ros::Time> timeRange;
    std::unordered_set<std::string> constellations;
    std::unordered_set<DayIndex> loadedDays;

    std::string urlFormat {"https://satdb.ethz.ch/api/satellitedata/?object-name=%s&"
                           "start-datetime=%sT0000&end-datetime=%sT0000&before=3&after=3&"
                           "without-frequency-data=True"};

    size_t maxDownloadDays {1000u};
    fs::path cacheDir;

    std::unordered_map<std::string, std::string> satdbConstellations {
        {gnss_info_msgs::Enums::CONSTELLATION_GPS, "NAVSTAR"},
        {gnss_info_msgs::Enums::CONSTELLATION_GALILEO, "GSAT0"},
        {gnss_info_msgs::Enums::CONSTELLATION_BEIDOU, "BEIDOU"},
        {gnss_info_msgs::Enums::CONSTELLATION_GLONASS, "COSMOS"}
    };

    fs::path getCacheFile(const DayIndex& day) const;
    cras::expected<bool, std::string> download(const DayIndex& day);
};

fs::path EthzSatdbDataSourcePrivate::getCacheFile(const DayIndex& day) const
{
    const auto filename = cras::format("ethz_satdb_%04u%02u%02u.tle", day.year, day.month, day.day);
    return this->cacheDir / filename;
}

cras::expected<bool, std::string> EthzSatdbDataSourcePrivate::download(const DayIndex& day)
{
    const auto startDate = cras::format("%04u%02u%02u", day.year, day.month, day.day);
    const DayIndex endDay(static_cast<ros::Time>(day) + ros::Duration::DAY);
    const auto endDate = cras::format("%04u%02u%02u", endDay.year, endDay.month, endDay.day);

    const auto cacheFile = this->getCacheFile(day);
    if (isCacheFileValid(cacheFile))
        return true;

    std::stringstream ss;
    size_t numSatellites {0u};
    for (const auto& [constallation, prefix] : this->satdbConstellations)
    {
        const auto url = cras::format(this->urlFormat, prefix.c_str(), startDate.c_str(), endDate.c_str());
        ROS_INFO("Downloading orbits from %s.", url.c_str());
        auto maybeReadBuffer = gnss_info::download(url);

        if (!maybeReadBuffer.has_value())
            return cras::make_unexpected(maybeReadBuffer.error());

        auto& readBuffer = *maybeReadBuffer;
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

EthzSatdbDataSource::EthzSatdbDataSource(
    const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satelliteInfo) :
        data(new EthzSatdbDataSourcePrivate)
{
    this->data->timeRange = std::make_pair(static_cast<ros::Time>(DayIndex(2023, 2, 10)), ros::Time::MAX);

    this->data->constellations =
    {
        gnss_info_msgs::Enums::CONSTELLATION_GPS,
        gnss_info_msgs::Enums::CONSTELLATION_GLONASS,
        gnss_info_msgs::Enums::CONSTELLATION_GALILEO,
        gnss_info_msgs::Enums::CONSTELLATION_BEIDOU,
    };

    this->data->cacheDir = getCacheDir();

    gnsstk::TLENavDataFactory::addSatelliteInfo(satelliteInfo);
    static auto registered {false};
    if (!registered)
    {
        gnsstk::NavDataFactoryPtr factory = std::make_shared<gnsstk::TLENavDataFactory>();
        gnsstk::MultiFormatNavDataFactory::addFactory(factory);
        registered = true;
    }
}

EthzSatdbDataSource::~EthzSatdbDataSource() = default;

std::pair<ros::Time, ros::Time> EthzSatdbDataSource::getTimeRange() const
{
    return this->data->timeRange;
}

std::unordered_set<std::string> EthzSatdbDataSource::getConstellations() const
{
    return this->data->constellations;
}

bool EthzSatdbDataSource::load(const ros::Time& time, const DataSourceLoadCb& cb)
{
    return this->load(time, time + ros::Duration::DAY, cb);
}

bool EthzSatdbDataSource::load(const ros::Time& startTime, const ros::Time& endTime, const DataSourceLoadCb& cb)
{
    auto endTime2 = endTime;
    if (endTime == ros::Time::MAX || (endTime - startTime) < ros::Duration::DAY)
        endTime2 = startTime + ros::Duration::DAY;

    std::list<DayIndex> days;
    for (auto time = startTime; time <= endTime2; time += ros::Duration::DAY)
        days.emplace_back(time);
    days.remove_if([this](const auto& day) {return this->data->loadedDays.count(day) > 0;});

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
        if (!isCacheFileValid(file))
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
    size_t numLoaded {0u};
    for (const auto& day : days)
    {
        const auto file = this->data->getCacheFile(day);
        if (!isCacheFileValid(file))
        {
            const auto maybeDownloaded = this->data->download(day);
            if (!maybeDownloaded.has_value())
            {
                ROS_ERROR("%s", maybeDownloaded.error().c_str());
                hadError = true;
                continue;
            }
        }

        if (!cb(file.string()))
        {
            ROS_ERROR("Error processing TLE data file %s.", file.c_str());
            hadError = true;
            continue;
        }

        numLoaded++;
        this->data->loadedDays.insert(day);
    }

    ROS_INFO("Successfully loaded orbits for %zu days.", numLoaded);
    return !hadError;
}

bool EthzSatdbDataSource::isPrecise() const
{
    return false;
}

bool EthzSatdbDataSource::isApproximate() const
{
    return true;
}

std::string EthzSatdbDataSource::getName() const
{
    return "ETH Zurich Satellite DB";
}

}
