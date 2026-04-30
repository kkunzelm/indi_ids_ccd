#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <peak/peak.hpp>

/**
 * @brief Small helper for IDS exposure-node handling.
 *
 * This class intentionally stays header-only, like IDSNodeCache and
 * IDSPixelFormatManager. It does not own camera state. IDS_CCD still owns the
 * cached node pointer and INDI properties; this helper only centralizes the
 * exposure-specific SDK logic.
 */
class IDSExposureController
{
public:
    struct Limits
    {
        double minimumSeconds {0.0};
        double maximumSeconds {0.0};
        double stepSeconds {0.001};
    };

    using LogCallback = std::function<void(const std::string &)>;

    static std::shared_ptr<peak::core::nodes::FloatNode>
    findExposureNode(const std::shared_ptr<peak::core::NodeMap> &nodeMap,
                     const LogCallback &logDebug = {},
                     const LogCallback &logError = {})
    {
        if (!nodeMap)
            throw std::runtime_error("Remote device node map is not initialized.");

        try
        {
            return nodeMap->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
        }
        catch (const std::exception &e1)
        {
            log(logDebug, std::string("ExposureTime node not available, trying ExposureTimeAbs: ") + e1.what());

            try
            {
                return nodeMap->FindNode<peak::core::nodes::FloatNode>("ExposureTimeAbs");
            }
            catch (const std::exception &e2)
            {
                log(logError, std::string("Neither ExposureTime nor ExposureTimeAbs is available. ExposureTimeAbs error: ") + e2.what());
                throw;
            }
        }
    }

    static bool queryLimits(const std::shared_ptr<peak::core::nodes::FloatNode> &exposureNode,
                            Limits &limits,
                            const LogCallback &logDebug = {},
                            const LogCallback &logError = {})
    {
        try
        {
            if (!exposureNode)
            {
                log(logError, "Exposure node is not available.");
                return false;
            }

            limits.minimumSeconds = exposureNode->Minimum() / microsecondsPerSecond;
            limits.maximumSeconds = exposureNode->Maximum() / microsecondsPerSecond;

            if (exposureNode->HasConstantIncrement())
                limits.stepSeconds = exposureNode->Increment() / microsecondsPerSecond;
            else
                limits.stepSeconds = 0.001;

            std::ostringstream message;
            message << "Exposure limits updated: "
                    << limits.minimumSeconds << " - "
                    << limits.maximumSeconds << " seconds (step: "
                    << limits.stepSeconds << ")";
            log(logDebug, message.str());

            return true;
        }
        catch (const std::exception &e)
        {
            log(logError, std::string("Failed to query exposure limits: ") + e.what());
            return false;
        }
    }

    static bool configure(const std::shared_ptr<peak::core::nodes::FloatNode> &exposureNode,
                          float durationSeconds,
                          const LogCallback &logDebug = {},
                          const LogCallback &logError = {})
    {
        try
        {
            if (!exposureNode)
            {
                log(logError, "ExposureTime node is not initialized or available.");
                return false;
            }

            const double minimumSeconds = exposureNode->Minimum() / microsecondsPerSecond;
            const double maximumSeconds = exposureNode->Maximum() / microsecondsPerSecond;

            if (durationSeconds < minimumSeconds)
            {
                std::ostringstream message;
                message << "Exposure time " << durationSeconds
                        << "s is below minimum " << minimumSeconds << "s";
                log(logError, message.str());
                return false;
            }

            if (durationSeconds > maximumSeconds)
            {
                std::ostringstream message;
                message << "Exposure time " << durationSeconds
                        << "s exceeds maximum " << maximumSeconds << "s";
                log(logError, message.str());
                return false;
            }

            const double exposureMicros = static_cast<double>(durationSeconds) * microsecondsPerSecond;
            exposureNode->SetValue(exposureMicros);

            std::ostringstream message;
            message << "Exposure successfully set to " << durationSeconds
                    << "s (" << exposureMicros << " us)";
            log(logDebug, message.str());

            return true;
        }
        catch (const std::exception &e)
        {
            log(logError, std::string("Failed to configure exposure: ") + e.what());
            return false;
        }
    }

private:
    static constexpr double microsecondsPerSecond = 1'000'000.0;

    static void log(const LogCallback &callback, const std::string &message)
    {
        if (callback)
            callback(message);
    }
};
