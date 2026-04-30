#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <exception>
#include <peak/peak.hpp>

/**
 * @brief Helper for IDS frame geometry operations.
 *
 * This class intentionally does not own camera state. IDS_CCD still owns the
 * INDI CCD chip, stream, and cached nodes. IDSFrameGeometry only centralizes
 * the small, easy-to-test geometry rules used by ROI and binning code.
 */
class IDSFrameGeometry
{
public:
    struct Frame
    {
        int x {0};
        int y {0};
        int w {0};
        int h {0};
    };

    struct NormalizedFrame
    {
        Frame indiFrame;
        int64_t hardwareX {0};
        int64_t hardwareY {0};
        int64_t hardwareW {0};
        int64_t hardwareH {0};
    };

    template <typename LogError>
    static bool isValidFrameRequest(const Frame &frame,
                                    int sensorWidth,
                                    int sensorHeight,
                                    const LogError &logError)
    {
        if (frame.w <= 0 || frame.h <= 0)
        {
            logError("Invalid frame dimensions: w=" + std::to_string(frame.w) +
                     " h=" + std::to_string(frame.h));
            return false;
        }

        if (frame.x < 0 || frame.y < 0)
        {
            logError("Invalid frame origin: x=" + std::to_string(frame.x) +
                     " y=" + std::to_string(frame.y));
            return false;
        }

        if (frame.x + frame.w > sensorWidth || frame.y + frame.h > sensorHeight)
        {
            logError("ROI exceeds sensor boundaries: sensor=" +
                     std::to_string(sensorWidth) + "x" + std::to_string(sensorHeight) +
                     ", requested=(" + std::to_string(frame.x) + "," +
                     std::to_string(frame.y) + ")+(" + std::to_string(frame.w) +
                     "," + std::to_string(frame.h) + ")");
            return false;
        }

        return true;
    }

    static bool isSameFrame(const Frame &lhs, const Frame &rhs)
    {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.w == rhs.w && lhs.h == rhs.h;
    }

    static NormalizedFrame normalizeFrameRequest(
        const Frame &requestedFrame,
        uint32_t binX,
        uint32_t binY,
        const std::shared_ptr<peak::core::nodes::IntegerNode> &offsetXNode,
        const std::shared_ptr<peak::core::nodes::IntegerNode> &offsetYNode,
        const std::shared_ptr<peak::core::nodes::IntegerNode> &widthNode,
        const std::shared_ptr<peak::core::nodes::IntegerNode> &heightNode)
    {
        const int adjUnbinnedW = ((requestedFrame.w + binX - 1) / binX) * binX;
        const int adjUnbinnedH = ((requestedFrame.h + binY - 1) / binY) * binY;
        const int adjUnbinnedX = (requestedFrame.x / binX) * binX;
        const int adjUnbinnedY = (requestedFrame.y / binY) * binY;

        const int64_t binnedX = adjUnbinnedX / binX;
        const int64_t binnedY = adjUnbinnedY / binY;
        const int64_t binnedW = adjUnbinnedW / binX;
        const int64_t binnedH = adjUnbinnedH / binY;

        const int64_t xMin = offsetXNode->Minimum();
        const int64_t yMin = offsetYNode->Minimum();
        const int64_t wMin = widthNode->Minimum();
        const int64_t hMin = heightNode->Minimum();

        const int64_t xInc = std::max<int64_t>(1, offsetXNode->Increment());
        const int64_t yInc = std::max<int64_t>(1, offsetYNode->Increment());
        const int64_t wInc = std::max<int64_t>(1, widthNode->Increment());
        const int64_t hInc = std::max<int64_t>(1, heightNode->Increment());

        auto normalizeValue = [](int64_t value, int64_t minimum, int64_t increment)
        {
            if (value < minimum)
                return minimum;

            return ((value - minimum) / increment) * increment + minimum;
        };

        NormalizedFrame result;
        result.hardwareX = normalizeValue(binnedX, xMin, xInc);
        result.hardwareY = normalizeValue(binnedY, yMin, yInc);
        result.hardwareW = normalizeValue(binnedW, wMin, wInc);
        result.hardwareH = normalizeValue(binnedH, hMin, hInc);

        result.indiFrame.x = static_cast<int>(result.hardwareX * binX);
        result.indiFrame.y = static_cast<int>(result.hardwareY * binY);
        result.indiFrame.w = static_cast<int>(result.hardwareW * binX);
        result.indiFrame.h = static_cast<int>(result.hardwareH * binY);

        return result;
    }

    static void applyHardwareFrame(
        const NormalizedFrame &frame,
        const std::shared_ptr<peak::core::nodes::IntegerNode> &offsetXNode,
        const std::shared_ptr<peak::core::nodes::IntegerNode> &offsetYNode,
        const std::shared_ptr<peak::core::nodes::IntegerNode> &widthNode,
        const std::shared_ptr<peak::core::nodes::IntegerNode> &heightNode)
    {
        const int64_t xMin = offsetXNode->Minimum();
        const int64_t yMin = offsetYNode->Minimum();
        const int64_t wMin = widthNode->Minimum();
        const int64_t hMin = heightNode->Minimum();

        // Many GenICam cameras require offsets to be reduced before width/height changes.
        offsetXNode->SetValue(xMin);
        offsetYNode->SetValue(yMin);

        widthNode->SetValue(wMin);
        heightNode->SetValue(hMin);

        widthNode->SetValue(frame.hardwareW);
        heightNode->SetValue(frame.hardwareH);

        offsetXNode->SetValue(frame.hardwareX);
        offsetYNode->SetValue(frame.hardwareY);
    }

    template <typename LogDebug>
    static void selectFirstUsableBinningRegion(
        const std::shared_ptr<peak::core::nodes::EnumerationNode> &binningSelectorNode,
        const LogDebug &logDebug)
    {
        if (!binningSelectorNode || !binningSelectorNode->IsWriteable())
            return;

        for (const auto &entry : binningSelectorNode->Entries())
        {
            std::string entryName;

            try
            {
                entryName = entry->SymbolicValue();
                binningSelectorNode->SetCurrentEntry(entryName);
                logDebug("Using binning selector entry: " + entryName);
                break;
            }
            catch (const std::exception &e)
            {
                logDebug("Cannot use binning selector entry " + entryName + ": " + e.what());
            }
        }
    }

    template <typename LogInfo, typename LogWarn, typename LogDebug>
    static std::string configureBinningModes(
        const std::shared_ptr<peak::core::nodes::EnumerationNode> &horizontalModeNode,
        const std::shared_ptr<peak::core::nodes::EnumerationNode> &verticalModeNode,
        const LogInfo &logInfo,
        const LogWarn &logWarn,
        const LogDebug &logDebug)
    {
        std::string selectedMode;

        if (horizontalModeNode && horizontalModeNode->IsWriteable())
        {
            selectedMode = selectHorizontalBinningMode(horizontalModeNode, logInfo, logDebug);
        }

        if (verticalModeNode && verticalModeNode->IsWriteable() && !selectedMode.empty())
        {
            try
            {
                verticalModeNode->SetCurrentEntry(selectedMode);
                logInfo("Using vertical binning mode: " + selectedMode);
            }
            catch (const std::exception &e)
            {
                logWarn("Cannot set vertical binning mode " + selectedMode + ": " + e.what());
            }
        }

        return selectedMode;
    }

private:
    template <typename LogInfo, typename LogDebug>
    static std::string selectHorizontalBinningMode(
        const std::shared_ptr<peak::core::nodes::EnumerationNode> &horizontalModeNode,
        const LogInfo &logInfo,
        const LogDebug &logDebug)
    {
        const std::vector<std::string> preferredModes {"Average", "Sum", "Mean"};
        const auto entries = horizontalModeNode->Entries();

        for (const auto &preferredMode : preferredModes)
        {
            for (const auto &entry : entries)
            {
                if (!entry->IsAvailable())
                    continue;

                const std::string modeName = entry->SymbolicValue();

                if (modeName != preferredMode)
                    continue;

                try
                {
                    horizontalModeNode->SetCurrentEntry(modeName);
                    logInfo("Using horizontal binning mode: " + modeName);
                    return modeName;
                }
                catch (const std::exception &e)
                {
                    logDebug("Cannot set binning mode " + modeName + ": " + e.what());
                }
            }
        }

        for (const auto &entry : entries)
        {
            if (!entry->IsAvailable())
                continue;

            const std::string modeName = entry->SymbolicValue();

            try
            {
                horizontalModeNode->SetCurrentEntry(modeName);
                logInfo("Using available horizontal binning mode: " + modeName);
                return modeName;
            }
            catch (const std::exception &e)
            {
                logDebug("Cannot set binning mode " + modeName + ": " + e.what());
            }
        }

        return {};
    }
};
