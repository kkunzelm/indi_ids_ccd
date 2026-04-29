#pragma once  
  
// Core INDI and IDS SDK includes    
#include "indiccd.h"    
#include <peak/peak.hpp>    
#include <peak/converters/peak_buffer_converter_ipl.hpp>    
#include <peak_ipl/peak_ipl.hpp>    
#include <functional>  
#include <map>  
#include <string>  
#include <mutex>  
#include <memory>  
#include <vector>
#include <pthread.h>
#include <sys/time.h>
  
/** * @namespace IDSConstants    
 * @brief Constants specific to IDS camera driver implementation    
 */    
namespace IDSConstants    
{    
    /// Standard polling interval in milliseconds for device status updates    
    constexpr int DEFAULT_POLLING_PERIOD_MS = 500;    
        
    /// Conversion factor: microseconds to seconds (1,000,000 μs = 1 s)    
    constexpr double MICROSEC_PER_SEC = 1000000.0;  
  
    /// Exposure unit conversion (1e-6 for microseconds)  
    constexpr double EXPOSURE_UNIT_SCALE = 1e-6;  
      
    constexpr double INVALID_TEMPERATURE = -273.15;  
}    
    
/** * @struct PixelFormatInfo    
 * @brief Maps between INDI and IDS pixel format names with conversion metadata    
 * * Encapsulated pixel format metadata (must precede class)  
 */    
struct PixelFormatInfo {  
    std::string idsName;  
    std::string indiName;  
    uint8_t bitsPerPixel;  
    bool packed;  
    bool isDefault; // Make sure this exists if you are passing 5 arguments  
    std::function<void(const uint8_t*, uint8_t*, uint32_t, uint32_t)> expandFunc;  
  
    // Constructor to match your push_back calls  
    PixelFormatInfo(std::string ids, std::string indi, uint8_t b, bool p, bool d,  
                    std::function<void(const uint8_t*, uint8_t*, uint32_t, uint32_t)> f)  
        : idsName(ids), indiName(indi), bitsPerPixel(b), packed(p), isDefault(d), expandFunc(f) {}  
      
    // Default constructor for map usage  
    PixelFormatInfo() : bitsPerPixel(8), packed(false), isDefault(false) {}  
};  
  
/** * @class IDS_CCD    
 * @brief INDI driver for IDS Imaging cameras    
 */    
class IDS_CCD : public INDI::CCD    
{    
    // INDI framework friend declarations  
    friend void ::ISGetProperties(const char *dev);    
    friend void ::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);    
    friend void ::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n);    
    friend void ::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);    
    friend void ::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n);    
    friend void ::ISSnoopDevice(XMLEle *root);    
    
public:    
    IDS_CCD();    
    virtual ~IDS_CCD() override;    
    
protected:    
    // CORE INDI DEVICE FUNCTIONS    
    bool Connect() override;    
    bool Disconnect() override;    
    const char *getDefaultName() override;    
    bool initProperties() override;    
    bool updateProperties() override;   
    void ISGetProperties(const char *dev) override;   
    
    // EXPOSURE CONTROL FUNCTIONS    
    virtual bool StartExposure(float duration) override;    
    virtual bool AbortExposure() override;    
    virtual void TimerHit() override;    
    
    // TEMPERATURE CONTROL FUNCTIONS    
    void setupTemperatureSensor();  
    double getTemperature();   
    void updateTemperatureProperty(); 
    
    // IMAGE CAPTURE AND FORMATTING    
    virtual bool SetCaptureFormat(uint8_t index) override;    
    virtual bool UpdateCCDFrame(int x, int y, int w, int h) override;    
    virtual bool UpdateCCDBin(int binx, int biny) override;    
    virtual void addFITSKeywords(INDI::CCDChip *targetChip,    
                                std::vector<INDI::FITSRecord> &fitsKeywords) override;    
    
    // PROPERTY HANDLING    
    virtual bool ISNewNumber(const char *dev, const char *name, double values[],    
                            char *names[], int n) override;    
    
    // CAMERA CONTROL PROPERTIES    
    INDI::PropertyNumber GainNP{1};  
    bool HasGain = false;    
        
    INDI::PropertyNumber OffsetNP{1};   
    bool HasOffset = false;    
      
    INDI::PropertyNumber TemperatureNP{1};  
    bool HasTemperature = false;    
      
    INDI::PropertySwitch CaptureFormatSP {0};  
  
    bool m_hasBayer {false};  
    std::string m_bayerPattern;  
    
    // CAMERA SETUP FUNCTIONS    
    bool setupGainSelector();    
    bool setupBlackLevel();    
  
    // New helper declaration for conversion  
    void applyPixelConversion(const uint8_t* src, uint8_t* dst, uint32_t width, uint32_t height);  
      
    void safeDeleteProperty(const char *propertyName);  
    
private:    
    // Threading infrastructure    
    typedef enum ImageState    
    {    
        StateNone = 0,    
        StateIdle,    
        StateExposure,    
        StateAbort,    
        StateTerminate,    
        StateTerminated    
    } ImageState;    
    
    bool m_propertiesReady {false};  
    bool m_captureFormatsDefined { false };  
      
    ImageState m_ThreadRequest;    
    ImageState m_ThreadState;    
    pthread_t m_ImagingThread;    
    pthread_cond_t cv = PTHREAD_COND_INITIALIZER;    
    pthread_mutex_t condMutex = PTHREAD_MUTEX_INITIALIZER;    
    std::mutex ccdBufferLock;  // For CCD buffer protection    
        
    // Exposure tracking    
    double m_ExposureRequest = 0;    
   
      
    // Thread methods    
    static void *imagingHelper(void *context);    
    void *imagingThreadEntry();    
    void getExposure();    
    int grabImage();    
    void exposureSetRequest(ImageState request);      
      
  
    // UTILITY FUNCTIONS    
    float CalcTimeLeft();    
    bool setupParams();    
    void allocateFrameBuffer();    
    void cleanupConnection();    
    void addCaptureFormat(const PixelFormatInfo &format);  
    bool validateRequiredNodes();  
    
    // BAYER HELPER FUNCTIONS  
    std::string mapCameraFormatToBayerPattern(const std::string& formatName);  
    void addBayerCaptureFormats(std::vector<PixelFormatInfo>& captureFormats,   
                               const std::string& bayerPattern,  
                               const std::vector<int>& supportedDepths);  
    
    // IDS CAMERA HARDWARE INTERFACE    
    bool initCamera();    
    void queryCameraCapabilities();    
    bool configureExposure(float duration);    
    bool startAcquisition();    
    bool stopAcquisition();    
    
    // CAMERA OBJECTS AND RESOURCES    
    std::shared_ptr<peak::core::Device> device;    
    std::shared_ptr<peak::core::DataStream> dataStream;    
    std::shared_ptr<peak::core::NodeMap> nodeMapRemoteDevice;    
  
    // Cache critical node pointers  
  
    std::shared_ptr<peak::core::nodes::EnumerationNode> pixelFormatNode;  
    std::shared_ptr<peak::core::nodes::IntegerNode> widthNode, heightNode;  
    std::shared_ptr<peak::core::nodes::IntegerNode> payloadSizeNode;  
    std::shared_ptr<peak::core::nodes::FloatNode> exposureNode;  
    std::shared_ptr<peak::core::nodes::EnumerationNode> userSetNode;  
    std::shared_ptr<peak::core::nodes::CommandNode> acquisitionStartNode, acquisitionStopNode;  
    std::shared_ptr<peak::core::nodes::FloatNode> tempNode;  
    std::shared_ptr<peak::core::nodes::EnumerationNode> tempSelectorNode;
    std::shared_ptr<peak::core::nodes::FloatNode> gainNode;  
    std::shared_ptr<peak::core::nodes::FloatNode> blackLevelNode;  
    std::shared_ptr<peak::core::nodes::FloatNode> offsetNode;  
    std::shared_ptr<peak::core::nodes::FloatNode> pixelWidthNode, pixelHeightNode;  
    
    // EXPOSURE STATE TRACKING    
    bool InExposure { false };    
    float TemperatureRequest { 0 };    
    struct timeval ExpStart;    
    
    // CAMERA HARDWARE PARAMETERS    
    int cameraWidth { 0 };    
    int cameraHeight { 0 };    
    int cameraBPP { 16 };    
    float pixelSizeX { 0.0F };    
    float pixelSizeY { 0.0F };    
    size_t m_currentPayloadSize { 0 };  
    
    // PIXEL FORMAT MANAGEMENT    
    //   
    void queryPixelFormats();  
    std::map<std::string, PixelFormatInfo> m_formatMap;    
        
    std::vector<std::string> m_availablePixelFormats;    
    std::vector<int> m_supportedBitDepths;    
    std::map<int, bool> m_compressionSupport;    
    int m_maxBitDepth { 8 };    
    uint8_t m_CurrentVideoFormat = 0;    
        
    // Refactored expansion signatures (bool isPacked removed, now internal to struct)  
    void expand10bitTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height);    
    void expand12bitTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height);    
    void expand10bitPackedTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height);  
    void expand12bitPackedTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height);  
        
    
    // USER SET MANAGEMENT    
    std::string currentUserSet = "Default";    
    bool switchUserSet(const std::string& userSet);    
    std::vector<std::string> queryAvailableUserSets();  
    std::vector<std::string> m_availableUserSets;    
    bool m_userSetsQueried = false;    
    
    // EXPOSURE LIMITS AND CONFIGURATION    
    bool queryExposureLimits();    
    double fullMin, fullMax;    
    double exposureStep = 0.001;    
    double longExposureMin;    
    double minExposure, maxExposure;    
    std::string determineUserSetForDuration(float duration);    
    void debugCurrentState();    
    int timerID = -1;  
      
    bool m_isAcquiring = false;   // tracks whether camera is in acquisition mode  
    
    // BLACK LEVEL CONTROL    
    void updateBlackLevelRange();    
    void resetBlackLevelToDefault();    
};
