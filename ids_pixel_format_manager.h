#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <peak/peak.hpp>

/**
 * @brief Maps between IDS/GenICam and INDI pixel format names with conversion metadata.
 */
struct PixelFormatInfo
{
    std::string idsName;
    std::string indiName;
    uint8_t bitsPerPixel = 8;
    bool packed = false;
    bool isDefault = false;
    std::function<void(const uint8_t *, uint8_t *, uint32_t, uint32_t)> expandFunc;

    PixelFormatInfo() = default;

    PixelFormatInfo(std::string ids,
                    std::string indi,
                    uint8_t bpp,
                    bool isPacked,
                    bool defaultFormat,
                    std::function<void(const uint8_t *, uint8_t *, uint32_t, uint32_t)> expand)
        : idsName(std::move(ids)),
          indiName(std::move(indi)),
          bitsPerPixel(bpp),
          packed(isPacked),
          isDefault(defaultFormat),
          expandFunc(std::move(expand))
    {
    }
};

/**
 * @brief Pixel-format discovery and raw-pixel expansion helpers.
 *
 * This class intentionally has no INDI property ownership. IDS_CCD still owns
 * the INDI-facing capture-format property; this helper only builds the format
 * metadata and performs pixel conversion.
 */
class IDSPixelFormatManager
{
public:
    using FormatMap = std::map<std::string, PixelFormatInfo>;
    using BayerPatternMapper = std::function<std::string(const std::string &)>;

    static void queryFormats(const std::shared_ptr<peak::core::nodes::EnumerationNode> &pixelFormatNode,
                             FormatMap &formatMap,
                             std::vector<int> &supportedBitDepths,
                             std::map<int, bool> &compressionSupport,
                             int &maxBitDepth,
                             const BayerPatternMapper &bayerPatternMapper,
                             const std::function<void(const std::string &)> &logInfo,
                             const std::function<void(const std::string &)> &logWarning,
                             const std::function<void(const std::string &)> &logError,
                             const std::function<void(const std::string &)> &logDebug)
    {
        formatMap.clear();
        supportedBitDepths.clear();
        compressionSupport.clear();

        if (!pixelFormatNode)
            throw std::runtime_error("PixelFormat node is not available.");

        const auto allEntries = pixelFormatNode->Entries();
        logInfo("Found " + std::to_string(allEntries.size()) + " pixel format entries");

        bool supports8bit  = false;
        bool supports10bit = false;
        bool supports12bit = false;
        bool supports16bit = false;

        for (const auto &entry : allEntries)
        {
            try
            {
                if (entry->AccessStatus() == peak::core::nodes::NodeAccessStatus::NotAvailable ||
                    entry->AccessStatus() == peak::core::nodes::NodeAccessStatus::NotImplemented)
                {
                    continue;
                }

                if (!entry->IsAvailable())
                    continue;

                const std::string formatName = entry->SymbolicValue();
                const bool isMono = formatName.find("Mono") == 0;
                const bool isBayer = formatName.find("Bayer") != std::string::npos;

                if (!isMono && !isBayer)
                    continue;

                const int sourceBitDepth = detectBitDepth(formatName);
                const bool packed = !formatName.empty() && formatName.back() == 'p';
                const uint8_t outputBpp = static_cast<uint8_t>((sourceBitDepth <= 8) ? 8 : 16);

                if (sourceBitDepth == 8)
                    supports8bit = true;
                else if (sourceBitDepth == 10)
                    supports10bit = true;
                else if (sourceBitDepth == 12)
                    supports12bit = true;
                else if (sourceBitDepth == 16)
                    supports16bit = true;

                if (packed)
                    compressionSupport[sourceBitDepth] = true;

                if (isMono)
                {
                    const std::string label = makeMonoLabel(sourceBitDepth, packed);
                    formatMap[formatName] = PixelFormatInfo(formatName, label, outputBpp, packed, false,
                                                             makeExpandFunc(sourceBitDepth, packed));
                    logInfo("Available Mono format: " + formatName + " -> " + label);
                }
                else if (isBayer)
                {
                    const std::string bayerPattern = bayerPatternMapper(formatName);

                    if (bayerPattern.empty())
                    {
                        logDebug("Skipping Bayer format with unknown pattern: " + formatName);
                        continue;
                    }

                    const std::string label = makeBayerLabel(bayerPattern, sourceBitDepth, packed);
                    formatMap[formatName] = PixelFormatInfo(formatName, label, outputBpp, packed, false,
                                                             makeExpandFunc(sourceBitDepth, packed));
                    logInfo("Available Bayer format: " + formatName + " -> " + label);
                }
            }
            catch (const std::exception &e)
            {
                logDebug(std::string("Skipping pixel format entry because it could not be queried: ") + e.what());
            }
        }

        if (supports8bit)
            supportedBitDepths.push_back(8);
        if (supports10bit)
            supportedBitDepths.push_back(10);
        if (supports12bit)
            supportedBitDepths.push_back(12);
        if (supports16bit)
            supportedBitDepths.push_back(16);

        maxBitDepth = 8;
        if (supports10bit)
            maxBitDepth = 10;
        if (supports12bit)
            maxBitDepth = 12;
        if (supports16bit)
            maxBitDepth = 16;

        if (formatMap.empty())
        {
            logWarning("No supported Mono/Bayer pixel formats found. Falling back to Mono8.");
            formatMap["Mono8"] = PixelFormatInfo("Mono8", "8-bit", 8, false, false, nullptr);
            supportedBitDepths.push_back(8);
            maxBitDepth = 8;
        }
    }

    static void expand10bitTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)
    {
        const uint32_t numPixels = width * height;
        const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src);
        uint16_t *dst16 = reinterpret_cast<uint16_t *>(dst);
        constexpr uint16_t MASK_10_BITS = 0x03FF;

        for (uint32_t i = 0; i < numPixels; ++i)
        {
            const uint16_t value10Bit = src16[i] & MASK_10_BITS;
            dst16[i] = static_cast<uint16_t>(value10Bit << 6);
        }
    }

    static void expand12bitTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)
    {
        const uint32_t numPixels = width * height;
        const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src);
        uint16_t *dst16 = reinterpret_cast<uint16_t *>(dst);

        for (uint32_t i = 0; i < numPixels; ++i)
            dst16[i] = static_cast<uint16_t>((src16[i] & 0x0FFF) << 4);
    }

    static void expand10bitPackedTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)
    {
        const uint32_t numPixels = width * height;
        const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src);
        uint16_t *dst16 = reinterpret_cast<uint16_t *>(dst);

        uint32_t bitPos = 0;
        uint32_t srcIndex = 0;

        for (uint32_t i = 0; i < numPixels; ++i)
        {
            uint32_t value = 0;
            uint32_t bitsNeeded = 10;
            uint32_t bitsCollected = 0;

            while (bitsNeeded > 0)
            {
                const uint32_t availableBits = 16 - (bitPos % 16);
                const uint32_t bitsToTake = std::min(availableBits, bitsNeeded);
                const uint32_t mask = (1U << bitsToTake) - 1U;
                const uint32_t shifted = (src16[srcIndex] >> (bitPos % 16)) & mask;

                value |= (shifted << bitsCollected);
                bitsCollected += bitsToTake;
                bitsNeeded -= bitsToTake;
                bitPos += bitsToTake;

                if (bitPos % 16 == 0)
                    ++srcIndex;
            }

            dst16[i] = static_cast<uint16_t>(value << 6);
        }
    }

    static void expand12bitPackedTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)
    {
        const uint32_t numPixels = width * height;
        uint16_t *dst16 = reinterpret_cast<uint16_t *>(dst);

        for (uint32_t i = 0; i < numPixels; i += 2)
        {
            const uint32_t baseSrc = (i * 3) / 2;
            const uint16_t p1 = (static_cast<uint16_t>(src[baseSrc]) << 4) | (src[baseSrc + 1] >> 4);
            dst16[i] = static_cast<uint16_t>(p1 << 4);

            if (i + 1 < numPixels)
            {
                const uint16_t p2 = (static_cast<uint16_t>(src[baseSrc + 2]) << 4) | (src[baseSrc + 1] & 0x0F);
                dst16[i + 1] = static_cast<uint16_t>(p2 << 4);
            }
        }
    }

private:
    static int detectBitDepth(const std::string &formatName)
    {
        if (formatName.find("16") != std::string::npos)
            return 16;
        if (formatName.find("12") != std::string::npos)
            return 12;
        if (formatName.find("10") != std::string::npos)
            return 10;
        if (formatName.find("8") != std::string::npos)
            return 8;

        return 8;
    }

    static std::string makeMonoLabel(int sourceBitDepth, bool packed)
    {
        if (packed)
            return std::to_string(sourceBitDepth) + "-bit (packed)";

        return std::to_string(sourceBitDepth) + "-bit";
    }

    static std::string makeBayerLabel(const std::string &pattern, int sourceBitDepth, bool packed)
    {
        std::string label = "INDI_BAYER_" + pattern + " " + std::to_string(sourceBitDepth) + "-bit";

        if (packed)
            label += " (packed)";

        return label;
    }

    static std::function<void(const uint8_t *, uint8_t *, uint32_t, uint32_t)> makeExpandFunc(int sourceBitDepth,
                                                                                              bool packed)
    {
        if (sourceBitDepth == 10 && packed)
            return expand10bitPackedTo16bit;
        if (sourceBitDepth == 10 && !packed)
            return expand10bitTo16bit;
        if (sourceBitDepth == 12 && packed)
            return expand12bitPackedTo16bit;
        if (sourceBitDepth == 12 && !packed)
            return expand12bitTo16bit;

        return nullptr;
    }
};
