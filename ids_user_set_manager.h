#pragma once

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <exception>

#include <peak/peak.hpp>

/**
 * @brief Helper for IDS UserSet discovery, selection and load commands.
 *
 * IDS_CCD still owns the higher-level INDI workflow: preserving ROI, preserving
 * pixel format, and updating exposure limits. This helper centralizes only the
 * IDS/GenICam UserSet node handling and the small UserSet decision policy.
 */
class IDSUserSetManager
{
public:
    void reset()
    {
        currentUserSet = "Default";
        availableUserSets.clear();
        userSetsQueried = false;
        userSetNode.reset();
        userSetLoadNode.reset();
    }

    const std::string &current() const
    {
        return currentUserSet;
    }

    void setCurrent(const std::string &userSet)
    {
        currentUserSet = userSet;
    }

    bool isCurrent(const std::string &userSet) const
    {
        return currentUserSet == userSet;
    }

    template <typename LogDebug, typename LogInfo, typename LogError>
    const std::vector<std::string> &available(const std::shared_ptr<peak::core::NodeMap> &nodeMap,
                                              const LogDebug &logDebug,
                                              const LogInfo &logInfo,
                                              const LogError &logError)
    {
        if (!userSetsQueried)
        {
            availableUserSets = queryAvailable(nodeMap, logDebug, logInfo, logError);
            userSetsQueried = true;
        }

        return availableUserSets;
    }

    template <typename LogDebug, typename LogInfo, typename LogError>
    std::vector<std::string> queryAvailable(const std::shared_ptr<peak::core::NodeMap> &nodeMap,
                                            const LogDebug &logDebug,
                                            const LogInfo &logInfo,
                                            const LogError &logError)
    {
        std::vector<std::string> userSets;

        try
        {
            auto selector = nodeMap->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");
            if (!selector)
            {
                logError("UserSetSelector not available");
                return userSets;
            }

            const auto entries = selector->Entries();
            for (const auto &entry : entries)
            {
                try
                {
                    if ((peak::core::nodes::NodeAccessStatus::NotAvailable != entry->AccessStatus()) &&
                        (peak::core::nodes::NodeAccessStatus::NotImplemented != entry->AccessStatus()))
                    {
                        const std::string entryName = entry->SymbolicValue();
                        userSets.push_back(entryName);
                        logDebug(std::string("Found accessible user set: ") + entryName);
                    }
                    else
                    {
                        std::ostringstream message;
                        message << "Skipping non-accessible user set: "
                                << entry->SymbolicValue()
                                << " (status: " << static_cast<int>(entry->AccessStatus()) << ")";
                        logDebug(message.str());
                    }
                }
                catch (const std::exception &e)
                {
                    logDebug(std::string("User set entry is not accessible: ") + e.what());
                }
            }

            std::ostringstream message;
            message << "Camera has " << userSets.size() << " writable user sets available";
            logInfo(message.str());
        }
        catch (const std::exception &e)
        {
            logError(std::string("Failed to query user sets: ") + e.what());
        }

        return userSets;
    }

    template <typename LogDebug, typename LogInfo, typename LogError>
    std::string determineForDuration(float duration,
                                     double longExposureMin,
                                     const std::shared_ptr<peak::core::NodeMap> &nodeMap,
                                     const LogDebug &logDebug,
                                     const LogInfo &logInfo,
                                     const LogError &logError)
    {
        const auto &sets = available(nodeMap, logDebug, logInfo, logError);

        if (sets.empty())
        {
            logError("No user sets available");
            return "Default";
        }

        {
            std::ostringstream message;
            message << "Determining user set for exposure duration: " << duration << " seconds";
            logDebug(message.str());
        }

        if (sets.size() == 1)
        {
            logDebug(std::string("Only one user set available (") + sets[0] + "), using it");
            return sets[0];
        }

        if (std::find(sets.begin(), sets.end(), "LongExposure") != sets.end())
        {
            if (duration > longExposureMin)
            {
                std::ostringstream message;
                message << "Duration " << duration << "s exceeds threshold "
                        << longExposureMin << "s, selecting LongExposure mode";
                logInfo(message.str());
                return "LongExposure";
            }
        }

        if (std::find(sets.begin(), sets.end(), "Default") != sets.end())
        {
            std::ostringstream message;
            message << "Duration " << duration << "s within threshold "
                    << longExposureMin << "s, selecting Default mode";
            logDebug(message.str());
            return "Default";
        }

        logDebug(std::string("Using first available user set: ") + sets[0]);
        return sets[0];
    }

    template <typename LogDebug, typename LogInfo, typename LogError>
    bool isAvailable(const std::string &userSet,
                     const std::shared_ptr<peak::core::NodeMap> &nodeMap,
                     const LogDebug &logDebug,
                     const LogInfo &logInfo,
                     const LogError &logError)
    {
        const auto &sets = available(nodeMap, logDebug, logInfo, logError);
        return std::find(sets.begin(), sets.end(), userSet) != sets.end();
    }

    template <typename LogError>
    bool load(const std::string &userSet,
              const std::shared_ptr<peak::core::NodeMap> &nodeMap,
              const LogError &logError)
    {
        if (!userSetNode)
            userSetNode = nodeMap->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");

        if (!userSetLoadNode)
            userSetLoadNode = nodeMap->FindNode<peak::core::nodes::CommandNode>("UserSetLoad");

        if (!userSetNode || !userSetLoadNode)
        {
            logError("UserSetSelector or UserSetLoad not available");
            return false;
        }

        if (!userSetNode->IsWriteable())
        {
            logError("UserSetSelector is not Writeable");
            return false;
        }

        userSetNode->SetCurrentEntry(userSet);
        userSetLoadNode->Execute();
        userSetLoadNode->WaitUntilDone();

        currentUserSet = userSet;
        return true;
    }

private:
    std::string currentUserSet {"Default"};
    std::vector<std::string> availableUserSets;
    bool userSetsQueried {false};

    std::shared_ptr<peak::core::nodes::EnumerationNode> userSetNode;
    std::shared_ptr<peak::core::nodes::CommandNode> userSetLoadNode;
};
