#pragma once

#include <memory>
#include <string>
#include <exception>

#include <peak/peak.hpp>

/**
 * @brief Helper for read-only IDS camera temperature monitoring.
 *
 * IDS_CCD still owns the INDI temperature property. This helper owns only the
 * cached GenICam temperature nodes and centralizes the SDK-side logic for
 * discovering and reading the camera temperature sensor.
 */
class IDSTemperatureController
{
public:
    template <typename LogInfo, typename LogWarn, typename LogDebug>
    bool setup(const std::shared_ptr<peak::core::NodeMap> &nodeMap,
               const LogInfo &logInfo,
               const LogWarn &logWarn,
               const LogDebug &logDebug)
    {
        reset();

        if (!nodeMap)
        {
            logWarn("Cannot setup temperature sensor: node map is not available.");
            return false;
        }

        try
        {
            tempSelectorNode =
                nodeMap->FindNode<peak::core::nodes::EnumerationNode>("DeviceTemperatureSelector");

            // Select a stable temperature source if the camera exposes a selector.
            // This is monitoring-only; no cooler or setpoint control is implied.
            if (tempSelectorNode && tempSelectorNode->IsAvailable() && tempSelectorNode->IsWriteable())
            {
                try
                {
                    tempSelectorNode->SetCurrentEntry("Mainboard");
                    logDebug("DeviceTemperatureSelector set to Mainboard.");
                }
                catch (const std::exception &e)
                {
                    logDebug(std::string("Could not select Mainboard temperature source: ") + e.what());
                }
            }

            tempNode = nodeMap->FindNode<peak::core::nodes::FloatNode>("DeviceTemperature");

            const bool available = isAvailable();

            if (available)
                logInfo("Read-only temperature sensor available.");
            else
                logInfo("Temperature sensor not available.");

            return available;
        }
        catch (const std::exception &e)
        {
            reset();
            logInfo(std::string("Temperature sensor not available: ") + e.what());
            return false;
        }
    }

    bool isAvailable() const
    {
        return tempNode && tempNode->IsAvailable() && tempNode->IsReadable();
    }

    template <typename LogDebug>
    double read(double invalidTemperature,
                const LogDebug &logDebug) const
    {
        if (!isAvailable())
            return invalidTemperature;

        try
        {
            return tempNode->Value();
        }
        catch (const std::exception &e)
        {
            logDebug(std::string("Failed to read temperature: ") + e.what());
            return invalidTemperature;
        }
    }

    void reset()
    {
        tempNode.reset();
        tempSelectorNode.reset();
    }

private:
    std::shared_ptr<peak::core::nodes::FloatNode> tempNode;
    std::shared_ptr<peak::core::nodes::EnumerationNode> tempSelectorNode;
};
