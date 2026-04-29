#include "indi_ids_ccd.h"
#include <memory>
#include <cstring>
#include <sys/time.h>

// for using ntons()
#ifdef _WIN32  
#include <winsock2.h>  
#else  
#include <arpa/inet.h>  
#endif


/** + * GLOBAL MULTI-CAMERA LOADER
 */
/** 
 * GLOBAL MULTI-CAMERA LOADER
 */

std::vector<std::unique_ptr<IDS_CCD>> idsCCDs;

static bool sdkInitialized = false;

// Protects SDK initialization and idsCCDs creation.
static std::mutex sdkInitMutex;

// Protects all access to idsCCDs while routing INDI callbacks.
static std::mutex idsCCDsMutex;


void ISGetProperties(const char *dev)
{
    {
        std::lock_guard<std::mutex> initLock(sdkInitMutex);

        if (!sdkInitialized)
        {
            try
            {
                peak::Library::Initialize();

                auto &deviceManager = peak::DeviceManager::Instance();
                deviceManager.Update();

                if (deviceManager.Devices().empty())
                {
                    IDLog("No IDS cameras found on the system.\n");

                    // Do not set sdkInitialized=true here if you want a later retry
                    // when a camera is plugged in after driver startup.
                    return;
                }

                {
                    std::lock_guard<std::mutex> idsLock(idsCCDsMutex);

                    for (const auto &deviceDescriptor : deviceManager.Devices())
                    {
                        std::string model = deviceDescriptor->ModelName();
                        std::string sn    = deviceDescriptor->SerialNumber();
                        std::string uniqueName = model + "-" + sn;

                        // Avoid duplicate camera objects if ISGetProperties is called again
                        // after a partial initialization or retry.
                        bool alreadyExists = false;

                        for (const auto &cam : idsCCDs)
                        {
                            if (strcmp(cam->getDeviceName(), uniqueName.c_str()) == 0)
                            {
                                alreadyExists = true;
                                break;
                            }
                        }

                        if (alreadyExists)
                        {
                            IDLog("Camera already registered: %s\n", uniqueName.c_str());
                            continue;
                        }

                        auto cam = std::make_unique<IDS_CCD>();
                        cam->setDeviceName(uniqueName.c_str());

                        idsCCDs.push_back(std::move(cam));

                        IDLog("Discovered camera: %s\n", uniqueName.c_str());
                    }
                }

                sdkInitialized = true;
            }
            catch (const std::exception &e)
            {
                IDLog("IDS SDK Initialization failed: %s\n", e.what());
            }
        }
    }

    std::lock_guard<std::mutex> idsLock(idsCCDsMutex);

    for (auto &cam : idsCCDs)
    {
        cam->ISGetProperties(dev);
    }
}


void ISNewSwitch(const char *dev,
                 const char *name,
                 ISState *states,
                 char *names[],
                 int n)
{
    if (dev == nullptr)
        return;

    std::lock_guard<std::mutex> idsLock(idsCCDsMutex);

    for (auto &cam : idsCCDs)
    {
        if (strcmp(dev, cam->getDeviceName()) == 0)
        {
            cam->ISNewSwitch(dev, name, states, names, n);
            break;
        }
    }
}


void ISNewText(const char *dev,
               const char *name,
               char *texts[],
               char *names[],
               int n)
{
    if (dev == nullptr)
        return;

    std::lock_guard<std::mutex> idsLock(idsCCDsMutex);

    for (auto &cam : idsCCDs)
    {
        if (strcmp(dev, cam->getDeviceName()) == 0)
        {
            cam->ISNewText(dev, name, texts, names, n);
            break;
        }
    }
}


void ISNewNumber(const char *dev,
                 const char *name,
                 double values[],
                 char *names[],
                 int n)
{
    if (dev == nullptr)
        return;

    std::lock_guard<std::mutex> idsLock(idsCCDsMutex);

    for (auto &cam : idsCCDs)
    {
        if (strcmp(dev, cam->getDeviceName()) == 0)
        {
            cam->ISNewNumber(dev, name, values, names, n);
            break;
        }
    }
}


void ISNewBLOB(const char *dev,
               const char *name,
               int sizes[],
               int blobsizes[],
               char *blobs[],
               char *formats[],
               char *names[],
               int n)
{
    if (dev == nullptr)
        return;

    std::lock_guard<std::mutex> idsLock(idsCCDsMutex);

    for (auto &cam : idsCCDs)
    {
        if (strcmp(dev, cam->getDeviceName()) == 0)
        {
            cam->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
            break;
        }
    }
}


void ISSnoopDevice(XMLEle *root)
{
    if (root == nullptr)
        return;

    const char *dev = findXMLAttValu(root, "device");

    if (dev == nullptr)
        return;

    std::lock_guard<std::mutex> idsLock(idsCCDsMutex);

    for (auto &cam : idsCCDs)
    {
        if (strcmp(dev, cam->getDeviceName()) == 0)
        {
            cam->ISSnoopDevice(root);
            break;
        }
    }
}

IDS_CCD::IDS_CCD()
{
    setVersion(1, 0);
    setCurrentPollingPeriod(IDSConstants::DEFAULT_POLLING_PERIOD_MS); 
}

IDS_CCD::~IDS_CCD()  
{  
    if (isConnected())  
    {  
        cleanupConnection();  // Safe: non-virtual call  
    }  
} 

bool IDS_CCD::Connect()
{
    LOG_INFO("=== Starting Connect() ===");

    if (!initCamera())
    {
        LOG_ERROR("initCamera() failed");
        cleanupConnection();
        return false;
    }

    LOG_INFO("initCamera() completed successfully");

    // CRITICAL: Set capabilities AFTER camera initialization but BEFORE other setup.
    // This ensures all capability flags are properly set before INDI exposes properties.
    queryCameraCapabilities();

    if (!setupParams())
    {
        LOG_ERROR("setupParams() failed");
        cleanupConnection();
        return false;
    }

    LOG_INFO("setupParams() completed successfully");

    if (!queryExposureLimits())
    {
        LOG_ERROR("queryExposureLimits() failed");
        cleanupConnection();
        return false;
    }

    LOG_INFO("queryExposureLimits() completed successfully");

    // Start imaging thread.
    pthread_mutex_lock(&condMutex);
    m_ThreadRequest = StateIdle;
    m_ThreadState   = StateNone;
    pthread_mutex_unlock(&condMutex);

    int stat = pthread_create(&m_ImagingThread, nullptr, &imagingHelper, this);

    if (stat != 0)
    {
        LOGF_ERROR("Error creating imaging thread (%d)", stat);
        cleanupConnection();
        return false;
    }

    m_threadRunning = true;

    // Wait for thread to initialize.
    pthread_mutex_lock(&condMutex);

    while (m_ThreadState == StateNone)
    {
        pthread_cond_wait(&cv, &condMutex);
    }

    pthread_mutex_unlock(&condMutex);

    SetTimer(getCurrentPollingPeriod());

    LOG_INFO("=== Connect() completed successfully ===");

    return true;
}


bool IDS_CCD::Disconnect()
{
    ImageState tState = StateNone;

    LOGF_DEBUG("Closing %s...", getDeviceName());

    if (m_threadRunning)
    {
        pthread_mutex_lock(&condMutex);

        tState = m_ThreadState;
        m_ThreadRequest = StateTerminate;

        pthread_cond_broadcast(&cv);
        pthread_mutex_unlock(&condMutex);

        pthread_join(m_ImagingThread, nullptr);
        m_threadRunning = false;
    }

    if (isConnected())
    {
        if (tState == StateExposure)
        {
            try
            {
                if (acquisitionStopNode && acquisitionStopNode->IsAvailable())
                {
                    acquisitionStopNode->Execute();
                    acquisitionStopNode->WaitUntilDone();
                }
            }
            catch (...)
            {
                LOG_WARN("Could not stop exposure during disconnect");
            }
        }

        cleanupConnection();
    }

    LOG_INFO("Camera is offline.");
    return true;
}

void IDS_CCD::ISGetProperties(const char *dev)  
{  
    LOGF_INFO("=== ISGetProperties() called for device: %s ===", dev ? dev : "NULL");  
      
    INDI::CCD::ISGetProperties(dev);  
    LOG_DEBUG("Base CCD properties defined");  
}

void IDS_CCD::cleanupConnection()
{
    if (InExposure.load() || m_isAcquiring.load())
    {
        AbortExposure();
    }

    if (device)
    {
        try
        {
            stopAcquisition();
        }
        catch (...)
        {
            // Ignore errors during disconnect
        }
        device.reset();
        dataStream.reset();
        nodeMapRemoteDevice.reset();
    }
    
    // Reset user set cache  
    m_userSetsQueried = false;  
    m_availableUserSets.clear();  

    LOG_INFO("IDS camera disconnected.");
    
}

bool IDS_CCD::initProperties()  
{  
    LOG_INFO("=== Starting initProperties() ===");  
      
    INDI::CCD::initProperties();  
    
    addDebugControl();    
    addConfigurationControl();   
    
    LOG_DEBUG("Base CCD properties initialized"); 
    
    
    CaptureFormatSP.fill(getDeviceName(), "CCD_CAPTURE_FORMAT", "Format",   
                    IMAGE_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

  
    // Setup Gain Property  
    GainNP[0].fill("GAIN", "Gain", "%.2f", 0, 100, 1, 0);  
    GainNP.fill(getDeviceName(), "CCD_GAIN", "Gain", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);  
    LOGF_DEBUG("Gain property initialized for device: %s", getDeviceName());  

  
    // Setup Offset Property (ADD THIS)  
    OffsetNP[0].fill("OFFSET", "Offset", "%.2f", 0, 100, 1, 0);  
    OffsetNP.fill(getDeviceName(), "CCD_OFFSET", "Offset", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);  
    LOGF_DEBUG("Offset property initialized for device: %s", getDeviceName());  

  
    // Setup Temperature Property  
    TemperatureNP[0].fill("TEMPERATURE", "Temperature (C)", "%.2f", -50, 100, 0, 0);  
    TemperatureNP.fill(getDeviceName(), "CCD_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);  
    LOGF_DEBUG("Temperature property initialized for device: %s", getDeviceName());  

  
    LOG_INFO("=== initProperties() completed successfully ==="); 
    return true;  
}


bool IDS_CCD::updateProperties()      
{      
    LOG_INFO("checkpoint: updateProperties()");      
          
      
    if (isConnected() && !m_propertiesReady)
    {
        m_propertiesReady = true;
    }           
      
    // 1. Let the base class handle standard CCD properties      
    INDI::CCD::updateProperties();      
      
    if (isConnected() && m_propertiesReady)       
    {      

        // The base class will automatically define BayerTP if HasBayer() returns true  
        // You just need to set the values  
        if (m_hasBayer)  
        {  
            BayerTP[CFA_TYPE].setText(m_bayerPattern.c_str());  
            BayerTP[CFA_OFFSET_X].setText("0");  
            BayerTP[CFA_OFFSET_Y].setText("0");  
            LOGF_INFO("Bayer pattern set to %s", m_bayerPattern.c_str());  
        }
        else
        {
            deleteProperty(BayerTP);
        }              
      
        // 2. Define Capture Format only once after setupParams has run
        if (!m_captureFormatsDefined)
        {
            defineProperty(CaptureFormatSP);
            m_captureFormatsDefined = true;
        } 
      
        // 3. Temperature property (only if supported)    
        if (HasTemperature) {    
            TemperatureNP.setPermission(IP_RO);    
            defineProperty(TemperatureNP);    
        }    
      
        // 4. Gain & Offset    
        if (HasGain)    
            defineProperty(GainNP);    
      
        if (HasOffset)    
            defineProperty(OffsetNP);    
      
        // 5. Start polling timer for this specific instance    
        try    
        {    
            timerID = SetTimer(getCurrentPollingPeriod());    
        }    
        catch (const std::exception &e)    
        {    
            LOGF_ERROR("Failed to start polling timer: %s", e.what());    
        }    
    }      
    else      
    {      
        // 6. Safe property cleanup with validation      
        deleteProperty(CaptureFormatSP.getName());      
              
        if (HasTemperature)      
            deleteProperty(TemperatureNP.getName());      
              
        if (HasGain)      
            deleteProperty(GainNP.getName());      
              
        if (HasOffset)      
            deleteProperty(OffsetNP.getName());     

        if (m_hasBayer)
            deleteProperty(BayerTP.getName());
            
        // 7. Proper timer cleanup      
        try      
        {      
            RemoveTimer(timerID);      
        }      
        catch (const std::exception & e)      
        {      
            LOG_WARN("Failed to remove timer - may already be stopped");      
        }      
          
        // Reset for next connection    
        m_propertiesReady = false;   
        m_captureFormatsDefined = false;     
    }      
      
    return true;      
}

bool IDS_CCD::initCamera()
{
    LOG_INFO("checkpoint: initCamera()");

    try
    {
        // 1. Initialize the Library / refresh device manager safely
        std::lock_guard<std::mutex> lock(sdkInitMutex);

        auto &deviceManager = peak::DeviceManager::Instance();
        deviceManager.Update();

        // 2. Multi-instance hardware matching
        // We look for the device whose Model-Serial matches this instance's name.
        std::string myName = getDeviceName();
        bool foundHardware = false;

        for (const auto &deviceDescriptor : deviceManager.Devices())
        {
            std::string dsName =
                deviceDescriptor->ModelName() + "-" + deviceDescriptor->SerialNumber();

            if (dsName == myName)
            {
                device = deviceDescriptor->OpenDevice(peak::core::DeviceAccessType::Control);
                foundHardware = true;

                LOGF_INFO("Instance %s successfully matched and opened hardware.",
                          myName.c_str());
                break;
            }
        }

        if (!foundHardware || !device)
        {
            LOGF_ERROR("Could not find/open specific hardware for instance: %s",
                       myName.c_str());
            return false;
        }

        // 3. Cache node map
        nodeMapRemoteDevice = device->RemoteDevice()->NodeMaps().at(0);

        if (!nodeMapRemoteDevice)
        {
            LOG_ERROR("Remote device node map is not available.");
            return false;
        }

        // 4. Cache critical node pointers
        pixelFormatNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");

        widthNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width");

        heightNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height");

        try
        {
            exposureNode = findExposureNode();

            if (!exposureNode)
            {
                LOG_ERROR("Exposure node could not be initialized.");
                return false;
            }

            LOG_INFO("Exposure node initialized successfully.");
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("Failed to initialize exposure node: %s", e.what());
            return false;
        }

        userSetNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");

        try
        {
            acquisitionStartNode =
                nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart");

            acquisitionStopNode =
                nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop");
        }
        catch (const std::exception &e)
        {
            LOGF_WARN("Acquisition command nodes not found; continuing anyway: %s", e.what());
        }

        // 5. Initialize hardware-specific components
        setupTemperatureSensor();

        // 6. Set acquisition mode
        try
        {
            nodeMapRemoteDevice
                ->FindNode<peak::core::nodes::EnumerationNode>("AcquisitionMode")
                ->SetCurrentEntry("SingleFrame");
        }
        catch (const std::exception &e)
        {
            LOGF_WARN("Could not set SingleFrame mode: %s", e.what());
        }

        // 7. DataStream and buffer allocation
        dataStream = device->DataStreams().at(0)->OpenDataStream();

        payloadSizeNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize");

        const auto payloadSize = payloadSizeNode->Value();

        if (payloadSize == 0)
        {
            LOG_ERROR("PayloadSize is zero during camera initialization.");
            return false;
        }

        for (uint64_t i = 0; i < dataStream->NumBuffersAnnouncedMinRequired(); ++i)
        {
            auto buffer =
                dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);

            dataStream->QueueBuffer(buffer);
        }

        return true;
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Exception during initCamera for %s: %s",
                   getDeviceName(),
                   e.what());
        return false;
    }
}

const char *IDS_CCD::getDefaultName()  
{ 
    std::lock_guard<std::mutex> idsLock(idsCCDsMutex);
 
    // If we have real cameras detected, don't create a default instance  
    if (!idsCCDs.empty())  
        return "";  // Empty name prevents default instance creation  
      
    return "IDS CCD";  // Only return default name when no cameras detected  
}

std::vector<std::string> IDS_CCD::queryAvailableUserSets()    
{    
    std::vector<std::string> userSets;    
        
    try    
    {    
        auto selector = nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");    
        if (!selector)    
        {    
            LOG_ERROR("UserSetSelector not available");    
            return userSets;    
        }    
            
        // Get all entries and check their access status  
        auto entries = selector->Entries();    
        for (const auto& entry : entries)    
        {    
            try    
            {    
                // Check if entry is accessible using AccessStatus()  
                if ((peak::core::nodes::NodeAccessStatus::NotAvailable != entry->AccessStatus()) &&  
                    (peak::core::nodes::NodeAccessStatus::NotImplemented != entry->AccessStatus()))    
                {    
                    std::string entryName = entry->SymbolicValue();    
                    userSets.push_back(entryName);    
                    LOGF_DEBUG("Found accessible user set: %s", entryName.c_str());    
                }    
                else    
                {    
                    LOGF_DEBUG("Skipping non-accessible user set: %s (status: %d)",   
                               entry->SymbolicValue().c_str(), static_cast<int>(entry->AccessStatus()));    
                }    
            }    
            catch (const std::exception &e)    
            {    
                LOGF_DEBUG("User set entry is not accessible: %s", e.what());    
            }    
        }    
            
        LOGF_INFO("Camera has %zu writable user sets available", userSets.size());    
    }    
    catch (const std::exception &e)    
    {    
        LOGF_ERROR("Failed to query user sets: %s", e.what());    
    }    
        
    return userSets;    
}
  
void IDS_CCD::queryCameraCapabilities()
{
    // Reset all capability flags
    HasGain = false;
    HasOffset = false;
    m_hasBayer = false;
    m_bayerPattern.clear();

    uint32_t cap = CCD_CAN_ABORT | CCD_CAN_SUBFRAME;

    // Check for gain control
    try
    {
        if (!gainNode)
            gainNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain");

        if (gainNode && gainNode->IsReadable())
        {
            HasGain = true;

            const double minGain = gainNode->Minimum();
            const double maxGain = gainNode->Maximum();
            const double valGain = gainNode->Value();

            double stepGain = 1.0;
            if (gainNode->HasConstantIncrement())
                stepGain = gainNode->Increment();
            else if (maxGain > minGain)
                stepGain = (maxGain - minGain) / 100.0;

            GainNP[0].setMin(minGain);
            GainNP[0].setMax(maxGain);
            GainNP[0].setStep(stepGain);
            GainNP[0].setValue(valGain);

            LOGF_INFO("Camera supports gain: %.2f to %.2f", minGain, maxGain);
        }
    }
    catch (const std::exception &e)
    {
        LOGF_DEBUG("No gain control: %s", e.what());
    }

    // Check for offset / black level control
    try
    {
        if (!blackLevelNode)
            blackLevelNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("BlackLevel");

        if (blackLevelNode && blackLevelNode->IsReadable())
        {
            HasOffset = true;

            const double minOffset = blackLevelNode->Minimum();
            const double maxOffset = blackLevelNode->Maximum();
            const double valOffset = blackLevelNode->Value();

            double stepOffset = 1.0;
            if (blackLevelNode->HasConstantIncrement())
                stepOffset = blackLevelNode->Increment();
            else if (maxOffset > minOffset)
                stepOffset = (maxOffset - minOffset) / 100.0;

            OffsetNP[0].setMin(minOffset);
            OffsetNP[0].setMax(maxOffset);
            OffsetNP[0].setStep(stepOffset);
            OffsetNP[0].setValue(valOffset);

            LOGF_INFO("Camera supports offset: %.2f to %.2f", minOffset, maxOffset);
        }
    }
    catch (const std::exception &e)
    {
        LOGF_DEBUG("No offset control: %s", e.what());
    }

    // Centralized temperature check
    if (!HasTemperature)
    {
        LOG_INFO("Temperature sensor not available or not readable.");
    }
    else if (tempNode && tempNode->IsAvailable())
    {
        try
        {
            LOGF_INFO("Camera temperature sensor is active. Current: %.2f C",
                      tempNode->Value());
        }
        catch (const std::exception &e)
        {
            LOGF_DEBUG("Could not read temperature during capability query: %s", e.what());
        }
    }

    // Check for color / Bayer support
    try
    {
        if (!pixelFormatNode)
        {
            pixelFormatNode =
                nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
        }

        auto entries = pixelFormatNode->Entries();

        for (const auto &entry : entries)
        {
            if (!entry->IsAvailable())
                continue;

            const std::string entryName = entry->SymbolicValue();

            if (entryName.find("Bayer") != std::string::npos)
            {
                const std::string pattern = mapCameraFormatToBayerPattern(entryName);

                if (!pattern.empty())
                {
                    m_hasBayer = true;
                    m_bayerPattern = pattern;

                    LOGF_INFO("Found Bayer format: %s -> %s",
                              entryName.c_str(),
                              m_bayerPattern.c_str());

                    break;
                }
            }
        }

        if (m_hasBayer && !m_bayerPattern.empty())
        {
            cap |= CCD_HAS_BAYER;
            LOGF_INFO("Camera supports Bayer pattern: %s", m_bayerPattern.c_str());
        }
    }
    catch (const std::exception &e)
    {
        LOGF_DEBUG("Error checking color support: %s", e.what());
    }

    // Check for cooling control
    bool hasCooling = false;

    try
    {
        auto coolingNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>(
                "DeviceTemperatureControlMode");

        if (coolingNode && coolingNode->IsWriteable())
        {
            hasCooling = true;
            LOG_INFO("Camera supports cooling control");
        }
    }
    catch (const std::exception &e)
    {
        LOGF_DEBUG("No cooling control: %s", e.what());
    }

    // Check for binning support
    try
    {
        auto binningNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("BinningHorizontal");

        if (binningNode && binningNode->IsWriteable())
        {
            const int maxBin = static_cast<int>(binningNode->Maximum());

            if (maxBin > 1)
            {
                cap |= CCD_CAN_BIN;

                PrimaryCCD.setMinMaxStep("CCD_BINNING", "HOR_BIN",
                                         1, maxBin, 1, false);

                PrimaryCCD.setMinMaxStep("CCD_BINNING", "VER_BIN",
                                         1, maxBin, 1, false);

                LOGF_INFO("Camera supports binning up to %dx%d", maxBin, maxBin);
            }
        }
    }
    catch (const std::exception &e)
    {
        LOGF_DEBUG("No binning control: %s", e.what());
    }

    // Get basic sensor size only.
    // Do NOT call SetCCDParams() here, because pixel size and final BPP may not
    // be known yet. setupParams() will call SetCCDParams() once all values are valid.
    try
    {
        if (!widthNode)
            widthNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width");

        if (!heightNode)
            heightNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height");

        if (widthNode && heightNode)
        {
            cameraWidth = static_cast<int>(widthNode->Maximum());
            cameraHeight = static_cast<int>(heightNode->Maximum());

            LOGF_INFO("Sensor size: %dx%d", cameraWidth, cameraHeight);
        }
    }
    catch (const std::exception &e)
    {
        LOGF_DEBUG("Error getting sensor info: %s", e.what());
    }

    // Add cooler capability if cooling control is available
    if (hasCooling)
    {
        cap |= CCD_HAS_COOLER;
    }

    // Set final capabilities
    SetCCDCapability(cap);

    LOGF_INFO("Final CCD capabilities: 0x%08X", cap);
}

bool IDS_CCD::configureExposure(float duration)  
{  
   try  
    {  
        // Use the cached exposureNode pointer from initCamera
        if (!exposureNode)  
        {  
            LOG_ERROR("ExposureTime node is not initialized or available.");  
            return false;  
        }  
  
        // Get current limits (already in microseconds from the camera)
        // Convert to seconds for validation against the INDI 'duration'
        double minExposureSec = exposureNode->Minimum() / IDSConstants::MICROSECONDS_PER_SECOND;  
        double maxExposureSec = exposureNode->Maximum() / IDSConstants::MICROSECONDS_PER_SECOND;  
  
        // Validate exposure time  
        if (duration < minExposureSec)  
        {  
            LOGF_ERROR("Exposure time %.6fs is below minimum %.6fs", duration, minExposureSec);  
            return false;  
        }  
        if (duration > maxExposureSec)  
        {  
            LOGF_ERROR("Exposure time %.6fs exceeds maximum %.6fs", duration, maxExposureSec);  
            return false;  
        }  
  
        // Convert seconds (INDI) to microseconds (Camera) 
        // using our defined constant instead of a raw number
        double exposureMicros = static_cast<double>(duration) * IDSConstants::MICROSECONDS_PER_SECOND;  
        
        exposureNode->SetValue(exposureMicros);  
          
        LOGF_DEBUG("Exposure successfully set to %.6fs (%.0f us)", duration, exposureMicros);  
        return true;  
    }  
    catch (const std::exception &e)  
    {  
        LOGF_ERROR("Failed to configure exposure: %s", e.what());  
        return false;  
    }  
}

bool IDS_CCD::startAcquisition()
{
    if (m_isAcquiring.load())
    {
        LOG_WARN("Acquisition already in progress.");
        return true;
    }

    if (!dataStream)
    {
        LOG_ERROR("Data stream not initialized.");
        return false;
    }

    try
    {
        // 1. Start the software stream for one frame.
        dataStream->StartAcquisition(peak::core::AcquisitionStartMode::Default, 1);

        // 2. Start the camera hardware.
        if (!acquisitionStartNode || !acquisitionStartNode->IsAvailable())
        {
            throw std::runtime_error("AcquisitionStart node not available");
        }

        acquisitionStartNode->Execute();
        acquisitionStartNode->WaitUntilDone();

        m_isAcquiring.store(true);
        InExposure.store(true);

        LOG_DEBUG("Acquisition started.");
        return true;
    }
    catch (const std::exception &ex)
    {
        LOGF_ERROR("Error starting acquisition: %s", ex.what());

        // If the software stream started but the camera start failed,
        // stop and flush the stream so the next exposure starts cleanly.
        if (dataStream)
        {
            try
            {
                if (dataStream->IsGrabbing())
                {
                    dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
                    LOG_DEBUG("Stopped data stream after failed acquisition start.");
                }
            }
            catch (const std::exception &e)
            {
                LOGF_WARN("Could not stop data stream after failed acquisition start: %s", e.what());
            }
            catch (...)
            {
                LOG_WARN("Could not stop data stream after failed acquisition start.");
            }

            try
            {
                dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
                LOG_DEBUG("Flushed data stream after failed acquisition start.");
            }
            catch (const std::exception &e)
            {
                LOGF_WARN("Could not flush data stream after failed acquisition start: %s", e.what());
            }
            catch (...)
            {
                LOG_WARN("Could not flush data stream after failed acquisition start.");
            }

            // Requeue announced buffers after flushing, so the next start has buffers.
            try
            {
                for (auto &buffer : dataStream->AnnouncedBuffers())
                {
                    try
                    {
                        dataStream->QueueBuffer(buffer);
                    }
                    catch (...)
                    {
                        // Ignore buffers the SDK refuses to requeue.
                    }
                }
            }
            catch (const std::exception &e)
            {
                LOGF_WARN("Could not requeue buffers after failed acquisition start: %s", e.what());
            }
        }

        m_isAcquiring.store(false);
        InExposure.store(false);

        return false;
    }
}

bool IDS_CCD::stopAcquisition()
{
    if (!m_isAcquiring.load())
    {
        LOG_DEBUG("Acquisition not running; skipping stop.");
        return true;
    }

    try
    {
        // 1. Tell the hardware device to stop generating frames
        if (acquisitionStopNode && acquisitionStopNode->IsAvailable())
        {
            acquisitionStopNode->Execute();
            acquisitionStopNode->WaitUntilDone();
        }
        else if (nodeMapRemoteDevice)
        {
            auto stopNode =
                nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop");

            if (stopNode && stopNode->IsAvailable())
            {
                stopNode->Execute();
                stopNode->WaitUntilDone();
            }
        }

        // 2. Tell the software stream to stop receiving
        if (dataStream)
        {
            dataStream->StopAcquisition();
        }

        InExposure.store(false);
        m_isAcquiring.store(false);

        LOG_DEBUG("Acquisition and stream stopped successfully.");
        return true;
    }
    catch (const std::exception &ex)
    {
        LOGF_ERROR("Error during graceful stop: %s", ex.what());

        InExposure.store(false);
        m_isAcquiring.store(false);

        return false;
    }
}

bool IDS_CCD::StartExposure(float duration)
{
    if (InExposure.load() || m_isAcquiring.load())
    {
        LOG_ERROR("Exposure already in progress. Abort current exposure first.");
        return false;
    }

    // Validate exposure duration against available limits
    if (duration < fullMin || duration > fullMax)
    {
        LOGF_ERROR("Exposure duration %.6fs is out of range [%.6fs, %.6fs]",
                   duration, fullMin, fullMax);
        return false;
    }

    // Determine which user set is needed for this duration
    std::string targetUserSet = determineUserSetForDuration(duration);

    // Switch user set if needed
    if (targetUserSet != currentUserSet)
    {
        if (!switchUserSet(targetUserSet))
        {
            LOG_ERROR("Failed to switch exposure mode");
            return false;
        }

        currentUserSet = targetUserSet;
    }

    // Configure exposure after possible user-set switch
    if (!configureExposure(duration))
    {
        LOG_ERROR("Failed to configure exposure duration.");
        return false;
    }

    // Ensure no stale acquisition is still active
    if (dataStream && dataStream->IsGrabbing())
    {
        LOG_WARN("Data stream is already active, stopping it first.");

        if (!stopAcquisition())
        {
            LOG_ERROR("Failed to stop previous acquisition.");
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(exposureStateMutex);
        m_ExposureRequest = static_cast<double>(duration);
        gettimeofday(&ExpStart, nullptr);
    }

    PrimaryCCD.setExposureDuration(duration);
    PrimaryCCD.setExposureLeft(duration);

    if (!startAcquisition())
    {
        LOG_ERROR("Failed to start acquisition.");
        return false;
    }

    LOGF_INFO("Starting %.3f second exposure (%s mode).",
              duration, targetUserSet.c_str());

    // Signal imaging thread to handle exposure monitoring
    pthread_mutex_lock(&condMutex);
    m_ThreadRequest = StateExposure;
    pthread_cond_broadcast(&cv);
    pthread_mutex_unlock(&condMutex);

    SetTimer(getCurrentPollingPeriod());

    return true;
}

std::string IDS_CCD::determineUserSetForDuration(float duration)  
{  
    // Get available user sets  
    std::vector<std::string> availableUserSets = queryAvailableUserSets();  
      
    if (availableUserSets.empty())  
    {  
        LOG_ERROR("No user sets available");  
        return "Default"; // Fallback  
    }  
      
    // Log the requested exposure duration  
    LOGF_DEBUG("Determining user set for exposure duration: %.3f seconds", duration);  
      
    // If only Default is available, always use it  
    if (availableUserSets.size() == 1)  
    {  
        LOGF_DEBUG("Only one user set available (%s), using it", availableUserSets[0].c_str());  
        return availableUserSets[0];  
    }  
      
    // Check if LongExposure is available and duration requires it  
    if (std::find(availableUserSets.begin(), availableUserSets.end(), "LongExposure") != availableUserSets.end())  
    {  
        if (duration > longExposureMin)  
        {  
            LOGF_INFO("Duration %.3fs exceeds threshold %.3fs, selecting LongExposure mode", duration, longExposureMin);  
            return "LongExposure";  
        }  
    }  
      
    // Default to Default mode if available  
    if (std::find(availableUserSets.begin(), availableUserSets.end(), "Default") != availableUserSets.end())  
    {  
        LOGF_DEBUG("Duration %.3fs within threshold %.3fs, selecting Default mode", duration, longExposureMin);  
        return "Default";  
    }  
      
    // Fallback to first available user set  
    LOGF_DEBUG("Using first available user set: %s", availableUserSets[0].c_str());  
    return availableUserSets[0];  
}

void *IDS_CCD::imagingHelper(void *context)  
{  
    return static_cast<IDS_CCD *>(context)->imagingThreadEntry();  
}  
  
void *IDS_CCD::imagingThreadEntry()
{
    pthread_mutex_lock(&condMutex);

    m_ThreadState = StateIdle;
    pthread_cond_broadcast(&cv);

    while (true)
    {
        while (m_ThreadRequest == StateIdle)
        {
            pthread_cond_wait(&cv, &condMutex);
        }

        m_ThreadState = m_ThreadRequest;
        pthread_cond_broadcast(&cv);

        if (m_ThreadRequest == StateExposure)
        {
            getExposure();

            m_ThreadState = StateIdle;
            pthread_cond_broadcast(&cv);
        }
        else if (m_ThreadRequest == StateAbort)
        {
            m_ThreadRequest = StateIdle;
            m_ThreadState = StateIdle;
            pthread_cond_broadcast(&cv);
        }
        else if (m_ThreadRequest == StateTerminate)
        {
            break;
        }
        else
        {
            m_ThreadRequest = StateIdle;
            m_ThreadState = StateIdle;
            pthread_cond_broadcast(&cv);
        }
    }

    m_ThreadState = StateTerminated;
    pthread_cond_broadcast(&cv);

    pthread_mutex_unlock(&condMutex);
    return nullptr;
} 

void IDS_CCD::getExposure()
{
    pthread_mutex_unlock(&condMutex);

    usleep(10000);  // Small delay to let exposure start

    pthread_mutex_lock(&condMutex);

    while (m_ThreadRequest == StateExposure)
    {
        pthread_mutex_unlock(&condMutex);

        double timeleft = static_cast<double>(CalcTimeLeft());

        uint32_t uSecs = 100000;

        if (timeleft > 1.1)
        {
            timeleft = round(timeleft);
            uSecs = 1000000;
        }

        if (timeleft >= 0)
        {
            PrimaryCCD.setExposureLeft(timeleft);
        }
        else
        {
            InExposure.store(false);
            PrimaryCCD.setExposureLeft(0.0);

            double exposureRequest = 0.0;
            {
                std::lock_guard<std::mutex> lock(exposureStateMutex);
                exposureRequest = m_ExposureRequest;
            }

            if (exposureRequest * 1000 > 5 * getCurrentPollingPeriod())
                DEBUG(INDI::Logger::DBG_SESSION, "Exposure done, downloading image...");

            pthread_mutex_lock(&condMutex);
            exposureSetRequest(StateIdle);
            pthread_cond_broadcast(&cv);
            pthread_mutex_unlock(&condMutex);

            grabImage();

            pthread_mutex_lock(&condMutex);
            break;
        }

        usleep(uSecs);

        pthread_mutex_lock(&condMutex);
    }

    pthread_cond_broadcast(&cv);
}

void IDS_CCD::exposureSetRequest(ImageState request)  
{  
    if (m_ThreadRequest == StateExposure)  
    {  
        m_ThreadRequest = request;  
    }  
}

bool IDS_CCD::AbortExposure()
{
    if (!InExposure.load() && !m_isAcquiring.load())
    {
        return true;
    }

    LOG_DEBUG("Aborting camera exposure...");

    // Signal imaging thread to abort exposure monitoring.
    pthread_mutex_lock(&condMutex);

    m_ThreadRequest = StateAbort;
    pthread_cond_broadcast(&cv);

    while (m_ThreadState == StateExposure)
    {
        pthread_cond_wait(&cv, &condMutex);
    }

    pthread_mutex_unlock(&condMutex);

    try
    {
        // 1. Ask the camera hardware to abort the acquisition if supported.
        if (nodeMapRemoteDevice)
        {
            try
            {
                auto abortNode =
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionAbort");

                if (abortNode && abortNode->IsAvailable())
                {
                    abortNode->Execute();
                    abortNode->WaitUntilDone();
                    LOG_DEBUG("AcquisitionAbort command executed.");
                }
            }
            catch (const std::exception &e)
            {
                LOGF_DEBUG("AcquisitionAbort node not available or failed: %s", e.what());
            }
        }

        // 2. Stop, flush, and requeue stream buffers.
        if (dataStream)
        {
            try
            {
                if (dataStream->IsGrabbing())
                {
                    dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
                    LOG_DEBUG("Data stream stopped during abort.");
                }
            }
            catch (const std::exception &e)
            {
                LOGF_DEBUG("StopAcquisition during abort failed: %s", e.what());
            }

            try
            {
                dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
                LOG_DEBUG("Data stream flushed during abort.");
            }
            catch (const std::exception &e)
            {
                LOGF_DEBUG("DataStream flush during abort failed: %s", e.what());
            }

            // Requeue all announced buffers so the next exposure has buffers available.
            // Some SDK states may reject buffers that are already queued; ignore those.
            try
            {
                size_t requeuedCount = 0;

                for (auto &buffer : dataStream->AnnouncedBuffers())
                {
                    try
                    {
                        dataStream->QueueBuffer(buffer);
                        ++requeuedCount;
                    }
                    catch (const std::exception &e)
                    {
                        LOGF_DEBUG("QueueBuffer after abort skipped/failed: %s", e.what());
                    }
                    catch (...)
                    {
                        LOG_DEBUG("QueueBuffer after abort skipped/failed with unknown error.");
                    }
                }

                LOGF_DEBUG("Requeued %zu announced buffers after abort.", requeuedCount);
            }
            catch (const std::exception &e)
            {
                LOGF_DEBUG("Could not enumerate/requeue buffers after abort: %s", e.what());
            }
        }

        LOG_INFO("Hardware and stream aborted.");
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Failed to abort exposure: %s", e.what());
    }

    InExposure.store(false);
    m_isAcquiring.store(false);
    PrimaryCCD.setExposureLeft(0.0);

    return true;
}

float IDS_CCD::CalcTimeLeft()
{
    struct timeval start {};
    double request = 0.0;

    {
        std::lock_guard<std::mutex> lock(exposureStateMutex);
        start = ExpStart;
        request = m_ExposureRequest;
    }

    struct timeval now {};
    gettimeofday(&now, nullptr);

    const double time_start_sec =
        static_cast<double>(start.tv_sec) +
        static_cast<double>(start.tv_usec) / IDSConstants::MICROSECONDS_PER_SECOND;

    const double time_now_sec =
        static_cast<double>(now.tv_sec) +
        static_cast<double>(now.tv_usec) / IDSConstants::MICROSECONDS_PER_SECOND;

    return static_cast<float>(request - (time_now_sec - time_start_sec));
}


void IDS_CCD::TimerHit()
{
    if (!isConnected())
        return;

    if (InExposure.load())
    {
        const float timeLeft = CalcTimeLeft();
        PrimaryCCD.setExposureLeft(std::max(0.0f, timeLeft));
    }

    updateTemperatureProperty();

    SetTimer(getCurrentPollingPeriod());
}

int IDS_CCD::grabImage()
{
    std::unique_lock<std::mutex> guard(ccdBufferLock);

    try
    {
        if (!dataStream)
        {
            LOG_ERROR("Cannot grab image: data stream is not initialized.");
            return false;
        }

        // 1. Wait for the buffer from the camera
        auto buffer = dataStream->WaitForFinishedBuffer(5000);
        peak::ipl::Image image = peak::BufferTo<peak::ipl::Image>(buffer);

        InExposure.store(false);

        // 2. Extract dimensions and format from cached nodes
        uint32_t imgWidth  = static_cast<uint32_t>(widthNode->Value());
        uint32_t imgHeight = static_cast<uint32_t>(heightNode->Value());

        std::string currentFormat =
            pixelFormatNode->CurrentEntry()->SymbolicValue();

        LOGF_DEBUG("grabImage(): Processing %ux%u in format %s",
                   imgWidth,
                   imgHeight,
                   currentFormat.c_str());

        // 3. Look up format info in the map
        auto it = m_formatMap.find(currentFormat);

        if (it == m_formatMap.end())
        {
            LOGF_ERROR("Unsupported pixel format: %s", currentFormat.c_str());

            stopAcquisition();

            try
            {
                dataStream->QueueBuffer(buffer);
            }
            catch (...)
            {
                LOG_WARN("Could not requeue buffer after unsupported format.");
            }

            return false;
        }

        PixelFormatInfo &fmtInfo = it->second;

        uint8_t *src = static_cast<uint8_t *>(image.Data());
        uint8_t *dst = PrimaryCCD.getFrameBuffer();

        size_t expectedSize = (fmtInfo.bitsPerPixel / 8) * imgWidth * imgHeight;

        if (!dst || expectedSize == 0)
        {
            LOGF_ERROR("Invalid buffer: dst=%p, expectedSize=%zu", dst, expectedSize);

            stopAcquisition();

            try
            {
                dataStream->QueueBuffer(buffer);
            }
            catch (...)
            {
                LOG_WARN("Could not requeue buffer after invalid destination buffer.");
            }

            return false;
        }

        LOGF_DEBUG("Processing image: %ux%u, %u bpp, buffer size: %zu",
                   imgWidth,
                   imgHeight,
                   fmtInfo.bitsPerPixel,
                   expectedSize);

        // 4. Apply conversion/expansion
        if (fmtInfo.expandFunc)
        {
            fmtInfo.expandFunc(src, dst, imgWidth, imgHeight);
        }
        else
        {
            size_t size = (fmtInfo.bitsPerPixel / 8) * imgWidth * imgHeight;
            memcpy(dst, src, size);
        }

        LOGF_DEBUG("Calling ExposureComplete with buffer at %p, size %zu",
                   dst,
                   expectedSize);

        // 5. Finalize INDI state
        PrimaryCCD.setBPP(fmtInfo.bitsPerPixel);

        stopAcquisition();

        ExposureComplete(&PrimaryCCD);

        try
        {
            dataStream->QueueBuffer(buffer);
        }
        catch (...)
        {
            LOG_WARN("Could not requeue buffer after successful image capture.");
        }

        LOGF_INFO("Image captured and processed: %ux%u at %u bpp",
                  imgWidth,
                  imgHeight,
                  fmtInfo.bitsPerPixel);

        return true;
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Failed to grab image: %s", e.what());

        try
        {
            stopAcquisition();
        }
        catch (...)
        {
        }

        InExposure.store(false);
        m_isAcquiring.store(false);

        return false;
    }
}

bool IDS_CCD::selectGainChannel()  
{  
    try  
    {  
        // Check if GainSelector exists  
        auto gainSelectorNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector");  
        if (!gainSelectorNode)  
        {  
            LOG_INFO("Camera does not support GainSelector");  
            return false;  
        }  
  
        // Get all available entries  
        auto allEntries = gainSelectorNode->Entries();  
        std::string chosenEntry;  
  
        // Prefer AnalogAll > All > DigitalAll  
        for (const auto &entry : allEntries)  
        {  
            if (entry->AccessStatus() == peak::core::nodes::NodeAccessStatus::NotAvailable ||  
                entry->AccessStatus() == peak::core::nodes::NodeAccessStatus::NotImplemented)  
            {  
                continue;  
            }  
  
            std::string entryValue = entry->StringValue();  
            if (entryValue == "AnalogAll" || entryValue == "All")  
            {  
                chosenEntry = entry->SymbolicValue();  
                break; // AnalogAll is preferred  
            }  
            else if (entryValue == "DigitalAll" && chosenEntry.empty())  
            {  
                chosenEntry = entry->SymbolicValue();  
            }  
        }  
  
        if (chosenEntry.empty())  
        {  
            LOG_WARN("No suitable GainSelector mode found");  
            return false;  
        }  
  
        gainSelectorNode->SetCurrentEntry(chosenEntry);  
        LOGF_INFO("GainSelector set to: %s", chosenEntry.c_str());  
        return true;  
    }  
    catch (const std::exception &e)  
    {  
        LOGF_ERROR("Failed to setup GainSelector: %s", e.what());  
        return false;  
    }  
}


bool IDS_CCD::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
    
        // EXPOSURE CONTROL - Add this section  
        if (!strcmp(name, "CCD_EXPOSURE"))  
        {  
            if (values[0] < 0.001 || values[0] > 3600)  
            {  
                LOGF_ERROR("Exposure duration %.6f is out of range [0.001, 3600]", values[0]);  
                return false;  
            }  
              
            return StartExposure(values[0]);  
        }  
    
        // Gain Control
        if (HasGain && !strcmp(name, GainNP.getName()))
        {
            double oldGain = GainNP[0].value;

            if (IUUpdateNumber(GainNP.getNumber(), values, names, n) < 0)
            {
                GainNP.setState(IPS_ALERT);
                IDSetNumber(GainNP.getNumber(), nullptr);
                return true;
            }

            try
            {
                if (gainNode)
                    gainNode->SetValue(GainNP[0].value);
                else
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(GainNP[0].value);

                GainNP.setState(IPS_OK);
                LOGF_INFO("Gain set to %.2f", GainNP[0].value);
                saveConfig(true, GainNP.getName());
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("Failed to set gain: %s", e.what());
                GainNP[0].value = oldGain;
                GainNP.setState(IPS_ALERT);
            }

            IDSetNumber(GainNP.getNumber(), nullptr);
            return true;
        }

        // Offset Control
        if (HasOffset && !strcmp(name, OffsetNP.getName()))
        {
            double oldOffset = OffsetNP[0].value;

            if (IUUpdateNumber(OffsetNP.getNumber(), values, names, n) < 0)
            {
                OffsetNP.setState(IPS_ALERT);
                IDSetNumber(OffsetNP.getNumber(), nullptr);
                return true;
            }

            try
            {
                if (blackLevelNode)
                    blackLevelNode->SetValue(OffsetNP[0].value);
                else
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("BlackLevel")->SetValue(OffsetNP[0].value);

                OffsetNP.setState(IPS_OK);
                LOGF_INFO("Offset set to %.2f", OffsetNP[0].value);
                saveConfig(true, OffsetNP.getName());
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("Failed to set offset: %s", e.what());
                OffsetNP[0].value = oldOffset;
                OffsetNP.setState(IPS_ALERT);
            }

            IDSetNumber(OffsetNP.getNumber(), nullptr);
            return true;
        }
    }

    return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

void IDS_CCD::addFITSKeywords(INDI::CCDChip *targetChip, std::vector<INDI::FITSRecord> &fitsKeywords)  
{  
    INDI::CCD::addFITSKeywords(targetChip, fitsKeywords);  
      
    // Identify the specific hardware in the FITS header
    fitsKeywords.push_back(INDI::FITSRecord("INSTRUME", getDeviceName(), "Camera Model and Serial"));

    // Use cached pixelFormatNode for efficiency
    std::string currentFormat;

    if (pixelFormatNode)
    {
        currentFormat = pixelFormatNode->CurrentEntry()->SymbolicValue();

        fitsKeywords.push_back(INDI::FITSRecord("PIXFORM", currentFormat.c_str(), "Pixel Format"));

        if (currentFormat == "Mono10p" || currentFormat == "Mono10")
        {
            fitsKeywords.push_back(INDI::FITSRecord("BITDEPTH", 10.0, "Original Bit Depth"));
            fitsKeywords.push_back(INDI::FITSRecord("BSCALE", 64.0, "Scaling factor to 16-bit"));
            fitsKeywords.push_back(INDI::FITSRecord("BZERO", 0.0, "Zero offset"));
        }
        else if (currentFormat == "Mono12p" || currentFormat == "Mono12")
        {
            fitsKeywords.push_back(INDI::FITSRecord("BITDEPTH", 12.0, "Original Bit Depth"));
            fitsKeywords.push_back(INDI::FITSRecord("BSCALE", 16.0, "Scaling factor to 16-bit"));
            fitsKeywords.push_back(INDI::FITSRecord("BZERO", 0.0, "Zero offset"));
        }
    } 
      
    // Add bit depth information  
    fitsKeywords.push_back(INDI::FITSRecord("BITPIX", static_cast<double>(cameraBPP), "Bits per Pixel"));  
      
    // Add compression info if applicable  
    if (!currentFormat.empty() && currentFormat.find("p") != std::string::npos)  
    {  
        fitsKeywords.push_back(INDI::FITSRecord("COMPRESS", "PACKED", "Compression Type"));  
    }  
      
    // Add gain if supported  
    if (HasGain)  
    {  
        fitsKeywords.push_back(INDI::FITSRecord("GAIN", GainNP[0].value, 3, "Gain"));  
    }  
      
    // Add offset if supported  
    if (HasOffset)  
    {  
        fitsKeywords.push_back(INDI::FITSRecord("OFFSET", OffsetNP[0].value, 3, "Offset"));  
    }  
    
    // Add temperature to FITS keywords
    if (HasTemperature)
    {  
        fitsKeywords.push_back(INDI::FITSRecord("CCD-TEMP", TemperatureNP[0].getValue(), 3, "CCD Temperature (Celsius)"));
    }
}

bool IDS_CCD::setupBlackLevel()
{
    try
    {
        // Check if BlackLevel node exists
        if (!blackLevelNode) 
             blackLevelNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("BlackLevel");

        if (!blackLevelNode)
        {
            LOG_INFO("Camera does not support BlackLevel control");
            return false;
        }

        // Check if node is readable and writeable
        if (blackLevelNode->AccessStatus() != peak::core::nodes::NodeAccessStatus::ReadWrite)
        {
            LOG_INFO("BlackLevel is not writeable");
            return false;
        }

        LOG_INFO("BlackLevel control available");
        return true;
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Failed to check BlackLevel support: %s", e.what());
        return false;
    }
}


bool IDS_CCD::UpdateCCDFrame(int x, int y, int w, int h)    
{    
    LOGF_INFO("UpdateCCDFrame called: x=%d y=%d w=%d h=%d", x, y, w, h);    
    
    if (InExposure.load() || m_isAcquiring.load())
    {    
        LOG_ERROR("Cannot change ROI while exposure is in progress.");    
        return false;    
    }    
    
    // Validate input parameters    
    if (w <= 0 || h <= 0)    
    {    
        LOGF_ERROR("Invalid frame dimensions: w=%d h=%d", w, h);    
        return false;    
    }    
    
    // Get current binning    
    uint32_t binX = PrimaryCCD.getBinX();    
    uint32_t binY = PrimaryCCD.getBinY();    
    
    // Round up dimensions to nearest binning multiple to prevent truncation    
    // Round down offsets to keep the ROI within the requested area    
    int adj_unbinned_w = ((w + binX - 1) / binX) * binX;    
    int adj_unbinned_h = ((h + binY - 1) / binY) * binY;    
    int adj_unbinned_x = (x / binX) * binX;    
    int adj_unbinned_y = (y / binY) * binY;    
    
    // Convert to BINNED coordinates for the hardware    
    int64_t binned_x = adj_unbinned_x / binX;    
    int64_t binned_y = adj_unbinned_y / binY;    
    int64_t binned_w = adj_unbinned_w / binX;    
    int64_t binned_h = adj_unbinned_h / binY;    
    
    // Validate against sensor boundaries (unbinned)    
    if (x + w > cameraWidth || y + h > cameraHeight)    
    {    
        LOGF_ERROR("ROI exceeds sensor boundaries: sensor=%dx%d, requested=(%d,%d)+(%d,%d)", cameraWidth, cameraHeight,    
                   x, y, w, h);    
        return false;    
    }    
    
    try    
    {    
        // Use cached nodes where possible    
        auto offsetXNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX");    
        auto offsetYNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY");    
            
        // Validate cached nodes or find fresh ones    
        auto widthNode = this->widthNode;    
        auto heightNode = this->heightNode;    
            
        if (!widthNode)    
            widthNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width");    
        if (!heightNode)    
            heightNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height");    
            
        if (!widthNode || !heightNode || !offsetXNode || !offsetYNode)    
        {    
            LOG_ERROR("Required ROI nodes not available");    
            return false;    
        }    
    
        // Query minimum values and increments    
        int64_t x_min = offsetXNode->Minimum();    
        int64_t y_min = offsetYNode->Minimum();    
        int64_t w_min = widthNode->Minimum();    
        int64_t h_min = heightNode->Minimum();    
    
        int64_t x_inc = offsetXNode->Increment();    
        int64_t y_inc = offsetYNode->Increment();    
        int64_t w_inc = widthNode->Increment();    
        int64_t h_inc = heightNode->Increment();    
    
        // Normalize BINNED values    
        auto norm = [](int64_t val, int64_t min, int64_t inc) { return ((val - min) / inc) * inc + min; };    
    
        int64_t adj_w = norm(binned_w, w_min, w_inc);    
        int64_t adj_h = norm(binned_h, h_min, h_inc);    
        int64_t adj_x = norm(binned_x, x_min, x_inc);    
        int64_t adj_y = norm(binned_y, y_min, y_inc);    
    
        // Additional validation after normalization    
        if (adj_w <= 0 || adj_h <= 0)    
        {    
            LOGF_ERROR("Normalized dimensions are invalid: w=%lld h=%lld", adj_w, adj_h);    
            return false;    
        }    
    
        // Set minimal ROI first    
        offsetXNode->SetValue(x_min);    
        offsetYNode->SetValue(y_min);    
        widthNode->SetValue(w_min);    
        heightNode->SetValue(h_min);    
    
        // Set the desired ROI (binned dimensions)    
        widthNode->SetValue(adj_w);    
        heightNode->SetValue(adj_h);    
        offsetXNode->SetValue(adj_x);    
        offsetYNode->SetValue(adj_y);    
    
        LOGF_INFO("Hardware ROI set (binned): x=%lld y=%lld w=%lld h=%lld", adj_x, adj_y, adj_w, adj_h);    
    
        // Stop acquisition and reallocate buffers - IMPROVED VERSION  
        try      
        {      
            if (dataStream && dataStream->IsGrabbing())      
            {      
                dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);      
            }      
        }      
        catch (...)      
        {      
            LOG_WARN("Could not stop acquisition during ROI change");      
        }      
        
        // Flush any pending buffers - IMPROVED VERSION    
        try      
        {      
            dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);      
        }      
        catch (...)      
        {      
            LOG_WARN("Could not flush data stream during ROI change");      
        }    
    
        for (auto &oldBuffer : dataStream->AnnouncedBuffers())    
        {    
            dataStream->RevokeBuffer(oldBuffer);    
        }    
    
        // Query new payload size    
        size_t payloadSize = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();    
        LOGF_DEBUG("New PayloadSize for ROI: %zu bytes", payloadSize);    
    
        // Validate payload size    
        if (payloadSize == 0)    
        {    
            LOG_ERROR("Payload size is 0 after ROI change - camera configuration error");    
            return false;    
        }    
    
        // Reallocate buffers    
        const size_t numBuffers = 3;    
        for (size_t i = 0; i < numBuffers; ++i)    
        {    
            auto buffer = dataStream->AllocAndAnnounceBuffer(payloadSize, nullptr);    
            dataStream->QueueBuffer(buffer);    
        }    
    
        // Update INDI frame buffer size (binned dimensions)    
        allocateFrameBuffer();    
    
        // Set UNBINNED coords in INDI (multiply back)    
        PrimaryCCD.setFrame(adj_x * binX, adj_y * binY, adj_w * binX, adj_h * binY);    
    
        return true;    
    }    
    catch (const std::exception &e)    
    {    
        LOGF_ERROR("Failed to set hardware ROI and reallocate buffers: %s", e.what());    
        return false;    
    }    
}

bool IDS_CCD::UpdateCCDBin(int binX, int binY)      
{      
    LOGF_INFO("UpdateCCDBin called: binX=%d binY=%d", binX, binY);      
      
    if (binX != binY)      
    {      
        LOG_ERROR("Asymmetric binning not supported");      
        return false;      
    }      
      
    if (InExposure.load() || m_isAcquiring.load())
    {      
        LOG_ERROR("Cannot change binning while exposure is in progress.");      
        return false;      
    }      
      
    auto binHNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("BinningHorizontal");      
    auto binVNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("BinningVertical");      
      
    if (!binHNode || !binVNode)      
    {      
        LOG_ERROR("Binning nodes not available");      
        return false;      
    }      
      
    int64_t maxBinH = binHNode->Maximum();      
    int64_t maxBinV = binVNode->Maximum();      
      
    if (binX < 1 || binX > maxBinH || binY < 1 || binY > maxBinV)      
    {      
        LOGF_ERROR("Requested binning %dx%d exceeds camera limits (max %lldx%lld)", binX, binY, maxBinH, maxBinV);      
        return false;      
    }      
      
    // Check for color camera binning constraints  
    if (m_hasBayer && (binX > 1 || binY > 1))     
    {      
        LOG_WARN("Binning may not be supported with Bayer patterns on this camera");      
        // Continue anyway - some cameras support it, some don't      
    }      
      
    try      
    {      
        // Check if BinningSelector exists and is usable      
        auto binSelectorNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("BinningSelector");      
              
        if (binSelectorNode && binSelectorNode->IsWriteable())      
        {      
            // Try to find Region0 or any available entry      
            auto entries = binSelectorNode->Entries();      
            for (const auto& entry : entries)      
            {      
                std::string entryName;    
                try      
                {      
                    entryName = entry->SymbolicValue();      
                    binSelectorNode->SetCurrentEntry(entryName);      
                    LOGF_DEBUG("Using binning selector entry: %s", entryName.c_str());      
                    break;      
                }      
                catch (const std::exception &e)      
                {      
                    LOGF_DEBUG("Cannot use binning selector entry %s: %s", entryName.c_str(), e.what());      
                    continue;      
                }      
            }      
        }      
      
        // Set binning values      
        binHNode->SetValue(binX);      
        binVNode->SetValue(binY);      
      
        // Set binning modes if available - CHECK AVAILABLE MODES FIRST  
        auto binHModeNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("BinningHorizontalMode");      
        auto binVModeNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("BinningVerticalMode");      
      
        std::string availableMode = "";  
          
        if (binHModeNode && binHModeNode->IsWriteable())      
        {      
            // Find available modes  
            auto entries = binHModeNode->Entries();  
            for (const auto& entry : entries)  
            {  
                if (entry->IsAvailable())  
                {  
                    std::string modeName = entry->SymbolicValue();  
                    // Try common modes in order of preference  
                    if (modeName == "Average" || modeName == "Sum" || modeName == "Mean")  
                    {  
                        try  
                        {  
                            binHModeNode->SetCurrentEntry(modeName);  
                            availableMode = modeName;  
                            LOGF_INFO("Using horizontal binning mode: %s", modeName.c_str());  
                            break;  
                        }  
                        catch (const std::exception &e)  
                        {  
                            LOGF_DEBUG("Cannot set binning mode %s: %s", modeName.c_str(), e.what());  
                            continue;  
                        }  
                    }  
                }  
            }  
              
            // If no preferred mode found, try any available mode  
            if (availableMode.empty())  
            {  
                for (const auto& entry : entries)  
                {  
                    if (entry->IsAvailable())  
                    {  
                        std::string modeName = entry->SymbolicValue();  
                        try  
                        {  
                            binHModeNode->SetCurrentEntry(modeName);  
                            availableMode = modeName;  
                            LOGF_INFO("Using available horizontal binning mode: %s", modeName.c_str());  
                            break;  
                        }  
                        catch (const std::exception &e)  
                        {  
                            LOGF_DEBUG("Cannot set binning mode %s: %s", modeName.c_str(), e.what());  
                            continue;  
                        }  
                    }  
                }  
            }  
        }      
          
        if (binVModeNode && binVModeNode->IsWriteable())      
        {      
            if (!availableMode.empty())  
            {  
                try  
                {  
                    binVModeNode->SetCurrentEntry(availableMode);  
                    LOGF_INFO("Using vertical binning mode: %s", availableMode.c_str());  
                }  
                catch (const std::exception &e)  
                {  
                    LOGF_WARN("Cannot set vertical binning mode %s: %s", availableMode.c_str(), e.what());  
                }  
            }  
        }      
      
        LOGF_INFO("Hardware binning applied: %dx%d%s", binX, binY,   
                  availableMode.empty() ? "" : (", Mode: " + availableMode).c_str());      
      
        // Update INDI binning property FIRST      
        PrimaryCCD.setBin(binX, binY);      
      
        // Update frame with new binning    
        return UpdateCCDFrame(PrimaryCCD.getSubX(), PrimaryCCD.getSubY(), PrimaryCCD.getSubW(), PrimaryCCD.getSubH());      
    }      
    catch (const std::exception &e)      
    {      
        LOGF_ERROR("UpdateCCDBin failed: %s", e.what());      
        return false;      
    }      
}

bool IDS_CCD::switchUserSet(const std::string &userSet)      
{   
    if (currentUserSet == userSet)      
    {      
        LOGF_DEBUG("Already in %s mode, skipping switch", userSet.c_str());      
        return true;      
    }      
        
    // Query user sets only once and cache the result    
    if (!m_userSetsQueried)      
    {      
        m_availableUserSets = queryAvailableUserSets();      
        m_userSetsQueried = true;      
    }      
        
    // Check if user set is available in cached list    
    if (std::find(m_availableUserSets.begin(), m_availableUserSets.end(), userSet) == m_availableUserSets.end())      
    {      
        LOGF_ERROR("User set '%s' is not available on this camera", userSet.c_str());      
        LOGF_INFO("Available user sets: %zu", m_availableUserSets.size());      
        for (const auto& set : m_availableUserSets)      
        {      
            LOGF_INFO("  - %s", set.c_str());      
        }      
        return false;      
    }      
      
    try      
    {      
        LOGF_INFO("Switching from %s to %s UserSet", currentUserSet.c_str(), userSet.c_str());      
      
        auto selector = nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");      
        auto loader   = nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("UserSetLoad");      
      
        if (!selector || !loader)      
        {      
            LOG_ERROR("UserSetSelector or UserSetLoad not available");      
            return false;      
        }      
      
        // Additional check: ensure selector is Writeable    
        if (!selector->IsWriteable())      
        {      
            LOG_ERROR("UserSetSelector is not Writeable");      
            return false;      
        }      
      
        // Store current frame settings AND pixel format      
        int currentX = PrimaryCCD.getSubX();      
        int currentY = PrimaryCCD.getSubY();      
        int currentW = PrimaryCCD.getSubW();      
        int currentH = PrimaryCCD.getSubH();      
              
        std::string currentFormat;

        try
        {
            if (!pixelFormatNode)
            {
                pixelFormatNode =
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
            }

            if (!pixelFormatNode)
            {
                LOG_ERROR("PixelFormat node not found");
                return false;
            }

            currentFormat = pixelFormatNode->CurrentEntry()->SymbolicValue();
            LOGF_INFO("Camera current format: %s", currentFormat.c_str());
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("Failed to get pixel format: %s", e.what());
            return false;
        } 
              
        // Switch userset with additional error handling      
        try      
        {      
            selector->SetCurrentEntry(userSet);      
        }      
        catch (const peak::core::Exception &e)      
        {      
            LOGF_ERROR("Failed to set UserSetSelector to %s: %s", userSet.c_str(), e.what());      
            return false;      
        }      
      
        loader->Execute();      
      
        // Wait until the UserSetLoad command has finished    
        loader->WaitUntilDone();      
      
        // Restore the previous pixel format instead of forcing Mono8      
        try      
        {      
           pixelFormatNode->SetCurrentEntry(currentFormat);      
            LOGF_INFO("Restored pixel format to: %s after userset switch", currentFormat.c_str());      
      
            // Verify the format was applied      
            auto verifyFormat = pixelFormatNode->CurrentEntry()->SymbolicValue();      
            if (verifyFormat != currentFormat)      
            {      
                LOGF_WARN("Could not restore format %s, current is: %s", currentFormat.c_str(), verifyFormat.c_str());      
            }      
        }      
        catch (const std::exception &e)      
        {      
            LOGF_WARN("Failed to restore pixel format: %s", e.what());      
            // Don't fail the switch - continue with whatever format the camera set      
        }      
      
        currentUserSet = userSet;

        if (currentW <= 0 || currentH <= 0)
        {
            LOG_WARN("Skipping frame restore after UserSet switch because current frame is not initialized.");
            return true;
        }

        if (!UpdateCCDFrame(currentX, currentY, currentW, currentH))
        {
            LOG_WARN("UserSet switched, but frame restore failed.");
        }

        return true; 
    }      
    catch (const peak::core::Exception &e)      
    {      
        LOGF_ERROR("Failed to switch UserSet to %s: %s", userSet.c_str(), e.what());      
        return false;      
    }      
}

bool IDS_CCD::queryExposureLimits()
{
    try
    {
        if (!exposureNode)
        {
            exposureNode = findExposureNode();
        }

        if (!exposureNode)
        {
            LOG_ERROR("Exposure node is not available.");
            return false;
        }

        minExposure = exposureNode->Minimum() / IDSConstants::MICROSECONDS_PER_SECOND;
        maxExposure = exposureNode->Maximum() / IDSConstants::MICROSECONDS_PER_SECOND;

        if (exposureNode->HasConstantIncrement())
        {
            exposureStep = exposureNode->Increment() / IDSConstants::MICROSECONDS_PER_SECOND;
        }
        else
        {
            exposureStep = 0.001;
        }

        LOGF_DEBUG("Exposure limits updated: %.6f - %.6f seconds (step: %.6f)",
                   minExposure,
                   maxExposure,
                   exposureStep);

        return true;
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Failed to query exposure limits: %s", e.what());
        return false;
    }
}


void IDS_CCD::queryPixelFormats()
{
    LOG_INFO("=== queryPixelFormats() ENTRY ===");

    m_formatMap.clear();
    m_supportedBitDepths.clear();
    m_compressionSupport.clear();

    try
    {
        if (!pixelFormatNode)
        {
            LOG_INFO("Querying pixel format node...");
            pixelFormatNode =
                nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
        }
        else
        {
            LOG_INFO("Using cached PixelFormat node");
        }

        if (!pixelFormatNode)
        {
            LOG_ERROR("PixelFormat node is not available.");
            throw std::runtime_error("PixelFormat node is not available.");
        }

        auto allEntries = pixelFormatNode->Entries();
        LOGF_INFO("Found %zu pixel format entries", allEntries.size());

        bool supports8bit  = false;
        bool supports10bit = false;
        bool supports12bit = false;
        bool supports16bit = false;

        auto makeExpandFunc =
            [this](int sourceBitDepth, bool packed)
            -> std::function<void(const uint8_t *, uint8_t *, uint32_t, uint32_t)>
        {
            if (sourceBitDepth == 10 && packed)
            {
                return [this](const uint8_t *src, uint8_t *dst, uint32_t w, uint32_t h)
                {
                    expand10bitPackedTo16bit(src, dst, w, h);
                };
            }

            if (sourceBitDepth == 10 && !packed)
            {
                return [this](const uint8_t *src, uint8_t *dst, uint32_t w, uint32_t h)
                {
                    expand10bitTo16bit(src, dst, w, h);
                };
            }

            if (sourceBitDepth == 12 && packed)
            {
                return [this](const uint8_t *src, uint8_t *dst, uint32_t w, uint32_t h)
                {
                    expand12bitPackedTo16bit(src, dst, w, h);
                };
            }

            if (sourceBitDepth == 12 && !packed)
            {
                return [this](const uint8_t *src, uint8_t *dst, uint32_t w, uint32_t h)
                {
                    expand12bitTo16bit(src, dst, w, h);
                };
            }

            return nullptr;
        };

        auto detectBitDepth = [](const std::string &formatName) -> int
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
        };

        auto makeMonoLabel = [](int sourceBitDepth, bool packed) -> std::string
        {
            if (packed)
                return std::to_string(sourceBitDepth) + "-bit (packed)";

            return std::to_string(sourceBitDepth) + "-bit";
        };

        auto makeBayerLabel =
            [this](const std::string &formatName, int sourceBitDepth, bool packed) -> std::string
        {
            std::string pattern = mapCameraFormatToBayerPattern(formatName);

            if (pattern.empty())
                pattern = "Bayer";

            std::string label =
                "INDI_BAYER_" + pattern + " " + std::to_string(sourceBitDepth) + "-bit";

            if (packed)
                label += " (packed)";

            return label;
        };

        LOG_INFO("Processing pixel format entries...");

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

                const bool isMono =
                    formatName.find("Mono") == 0;

                const bool isBayer =
                    formatName.find("Bayer") != std::string::npos;

                if (!isMono && !isBayer)
                    continue;

                const int sourceBitDepth = detectBitDepth(formatName);
                const bool packed =
                    !formatName.empty() && formatName.back() == 'p';

                // Output buffer depth. 10-bit and 12-bit are expanded to 16-bit.
                const uint8_t outputBpp =
                    static_cast<uint8_t>((sourceBitDepth <= 8) ? 8 : 16);

                if (sourceBitDepth == 8)
                    supports8bit = true;
                else if (sourceBitDepth == 10)
                    supports10bit = true;
                else if (sourceBitDepth == 12)
                    supports12bit = true;
                else if (sourceBitDepth == 16)
                    supports16bit = true;

                if (packed)
                    m_compressionSupport[sourceBitDepth] = true;

                auto expandFunc = makeExpandFunc(sourceBitDepth, packed);

                if (isMono)
                {
                    const std::string label = makeMonoLabel(sourceBitDepth, packed);

                    m_formatMap[formatName] =
                        PixelFormatInfo(formatName,
                                        label,
                                        outputBpp,
                                        packed,
                                        false,
                                        expandFunc);

                    LOGF_INFO("Available Mono format: %s -> %s, source=%d-bit, output=%u-bit%s",
                              formatName.c_str(),
                              label.c_str(),
                              sourceBitDepth,
                              outputBpp,
                              packed ? ", packed" : "");
                }
                else if (isBayer)
                {
                    const std::string bayerPattern =
                        mapCameraFormatToBayerPattern(formatName);

                    if (bayerPattern.empty())
                    {
                        LOGF_DEBUG("Skipping Bayer format with unknown pattern: %s",
                                   formatName.c_str());
                        continue;
                    }

                    const std::string label =
                        makeBayerLabel(formatName, sourceBitDepth, packed);

                    m_formatMap[formatName] =
                        PixelFormatInfo(formatName,
                                        label,
                                        outputBpp,
                                        packed,
                                        false,
                                        expandFunc);

                    LOGF_INFO("Available Bayer format: %s -> %s, source=%d-bit, output=%u-bit%s",
                              formatName.c_str(),
                              label.c_str(),
                              sourceBitDepth,
                              outputBpp,
                              packed ? ", packed" : "");
                }
            }
            catch (const std::exception &e)
            {
                LOGF_DEBUG("Skipping pixel format entry because it could not be queried: %s",
                           e.what());
            }
        }

        if (supports8bit)
            m_supportedBitDepths.push_back(8);

        if (supports10bit)
            m_supportedBitDepths.push_back(10);

        if (supports12bit)
            m_supportedBitDepths.push_back(12);

        if (supports16bit)
            m_supportedBitDepths.push_back(16);

        m_maxBitDepth = 8;

        if (supports10bit)
            m_maxBitDepth = 10;

        if (supports12bit)
            m_maxBitDepth = 12;

        if (supports16bit)
            m_maxBitDepth = 16;

        if (m_formatMap.empty())
        {
            LOG_WARN("No supported Mono/Bayer pixel formats found. Falling back to Mono8.");
            m_formatMap["Mono8"] = PixelFormatInfo("Mono8", "8-bit", 8, false, false, nullptr);
            m_supportedBitDepths.push_back(8);
            m_maxBitDepth = 8;
        }

        LOGF_INFO("Camera supports %zu usable pixel formats. Max source depth: %d",
                  m_formatMap.size(),
                  m_maxBitDepth);

        LOG_INFO("=== queryPixelFormats() completed successfully ===");
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Failed to query pixel formats: %s", e.what());
        LOG_INFO("Setting fallback Mono8 format...");

        m_formatMap.clear();
        m_supportedBitDepths.clear();
        m_compressionSupport.clear();

        m_formatMap["Mono8"] =
            PixelFormatInfo("Mono8", "8-bit", 8, false, false, nullptr);

        m_supportedBitDepths.push_back(8);
        m_maxBitDepth = 8;
    }
}

bool IDS_CCD::SetCaptureFormat(uint8_t index)
{
    if (InExposure.load() || m_isAcquiring.load())
    {
        LOG_ERROR("Cannot change capture format while exposure/acquisition is active.");
        return false;
    }

    auto &formatSP = CaptureFormatSP;

    if (index >= formatSP.size())
    {
        LOGF_ERROR("Invalid capture format index: %u", index);
        return false;
    }

    // The switch name must be the real IDS / GenICam pixel format name.
    // Example: Mono8, Mono10, Mono10p, Mono12, Mono12p, BayerRG8, ...
    const std::string idsFormatName = formatSP[index].getName();

    auto it = m_formatMap.find(idsFormatName);
    if (it == m_formatMap.end())
    {
        LOGF_ERROR("Capture format '%s' not found in format map.", idsFormatName.c_str());
        return false;
    }

    const PixelFormatInfo &formatInfo = it->second;

    try
    {
        if (!dataStream)
        {
            LOG_ERROR("Cannot change capture format: data stream is not initialized.");
            return false;
        }

        if (!pixelFormatNode)
        {
            LOG_ERROR("Cannot change capture format: PixelFormat node is not initialized.");
            return false;
        }

        if (!nodeMapRemoteDevice)
        {
            LOG_ERROR("Cannot change capture format: node map is not initialized.");
            return false;
        }

        // Do not change payload-affecting features while the stream is grabbing.
        // If this happens while InExposure/m_isAcquiring are false, the internal
        // state is inconsistent and should be handled elsewhere, not silently here.
        if (dataStream->IsGrabbing())
        {
            LOG_ERROR("Cannot change capture format while data stream is grabbing.");
            return false;
        }

        // Clear old buffers before changing/reallocating payload.
        try
        {
            dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
        }
        catch (const std::exception &e)
        {
            LOGF_WARN("Could not flush data stream before pixel format change: %s", e.what());
        }

        for (auto &oldBuffer : dataStream->AnnouncedBuffers())
        {
            dataStream->RevokeBuffer(oldBuffer);
        }

        // Set the new pixel format using the real IDS / GenICam name.
        pixelFormatNode->SetCurrentEntry(formatInfo.idsName);

        // Verify that the camera accepted the format.
        const std::string appliedFormat =
            pixelFormatNode->CurrentEntry()->SymbolicValue();

        if (appliedFormat != formatInfo.idsName)
        {
            LOGF_WARN("Requested pixel format '%s', but camera reports '%s'.",
                      formatInfo.idsName.c_str(),
                      appliedFormat.c_str());
        }

        cameraBPP = formatInfo.bitsPerPixel;

        // Reallocate buffers for the new payload size.
        payloadSizeNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize");

        const size_t payloadSize = static_cast<size_t>(payloadSizeNode->Value());

        if (payloadSize == 0)
        {
            LOG_ERROR("PayloadSize is zero after pixel format change.");
            return false;
        }

        const size_t numBuffers = dataStream->NumBuffersAnnouncedMinRequired();

        for (size_t i = 0; i < numBuffers; ++i)
        {
            auto buffer = dataStream->AllocAndAnnounceBuffer(payloadSize, nullptr);
            dataStream->QueueBuffer(buffer);
        }

        allocateFrameBuffer();

        // Mark selected switch as active.
        for (uint8_t i = 0; i < formatSP.size(); ++i)
        {
            formatSP[i].setState(i == index ? ISS_ON : ISS_OFF);
        }

        formatSP.setState(IPS_OK);
        formatSP.apply();

        LOGF_INFO("Pixel format changed to: %s (%s, %u bpp)",
                  idsFormatName.c_str(),
                  formatInfo.indiName.c_str(),
                  formatInfo.bitsPerPixel);

        updateBlackLevelRange();

        return true;
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Failed to set capture format '%s': %s",
                   idsFormatName.c_str(),
                   e.what());

        formatSP.setState(IPS_ALERT);
        formatSP.apply();

        return false;
    }
}


void IDS_CCD::allocateFrameBuffer()      
{    
    LOG_INFO("top of allocateFrameBuffer");   
    uint32_t width = PrimaryCCD.getSubW() / PrimaryCCD.getBinX();      
    uint32_t height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();      
          
    if (width == 0 || height == 0)      
    {      
        LOGF_ERROR("Invalid frame dimensions: %dx%d", width, height);      
        return;      
    }      
          
    // Fetch the hardware's expected payload size  
    if (!payloadSizeNode)  
        payloadSizeNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize");  
  
    if (!payloadSizeNode)    
    {      
        LOG_ERROR("PayloadSize node not available");      
        return;      
    }      
        
    size_t actualPayloadSize = payloadSizeNode->Value();      
      
    // Get current format and determine if it's Bayer  
    std::string currentFormat = "Mono8"; // default  
      
    auto pixelFormatNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");  
    if (pixelFormatNode)  
    {  
        currentFormat = pixelFormatNode->CurrentEntry()->SymbolicValue();  
    }  
      
    // Calculate buffer size using format map for all formats  
    size_t bufferSize;
    auto it = m_formatMap.find(currentFormat);  
    if (it != m_formatMap.end()) {  
        uint8_t bpp = it->second.bitsPerPixel;  
        bufferSize = (width * height * bpp) / 8;  
        LOGF_DEBUG("Buffer size calculated using format map: %s -> %d bpp",   
                   currentFormat.c_str(), bpp);  
    } else {  
        LOGF_ERROR("Unknown format: %s", currentFormat.c_str());  
        return;  
    }
      
    // Use the larger of calculated size or actual payload  
    if (bufferSize < actualPayloadSize)  
    {  
        LOGF_WARN("Calculated buffer (%zu) is smaller than Camera Payload (%zu). Using Payload Size.",   
                  bufferSize, actualPayloadSize);  
        bufferSize = actualPayloadSize;  
    }  
          
    if (bufferSize == 0)      
    {      
        LOG_ERROR("Final buffer size is zero.");      
        return;      
    }      
          
    PrimaryCCD.setFrameBufferSize(bufferSize);      
    PrimaryCCD.setNAxis(2);      
          
    LOGF_INFO("Frame buffer allocated: %dx%d (%s). Buffer size: %zu bytes (Payload: %zu bytes)",      
               width, height, currentFormat.c_str(), bufferSize, actualPayloadSize);      
}

void IDS_CCD::expand10bitTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)  
{  
    LOG_INFO("KHK inside expand10bitTo16bit");
    const uint32_t numPixels = width * height;  
    const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src); 
    uint16_t *dst16 = reinterpret_cast<uint16_t *>(dst);  
    const uint16_t MASK_10_BITS = 0x03FF;  
                
    for (uint32_t i = 0; i < numPixels; ++i)  
    {   
        // Extract the 10-bit value and scale to 16-bit (left shift by 6)
        uint16_t value_10bit = src16[i] & MASK_10_BITS;  
        dst16[i] = static_cast<uint16_t>(value_10bit << 6);   
        
        // Debug first 10 pixels  
        if (i < 10)  
        {  
            LOGF_INFO("KHK Mono10: src16[%d]=0x%04X, value_10bit=0x%03X, dst16[%d]=0x%04X",   
                       i, src16[i], value_10bit, i, dst16[i]);  
        }  
    }
}

void IDS_CCD::expand12bitTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)  
{  

    const uint32_t numPixels = width * height;  
    const uint16_t *src16 = reinterpret_cast<const uint16_t *>(src); 
    uint16_t *dst16 = reinterpret_cast<uint16_t *>(dst);  

    // Loop without internal 'if' branches for speed
    for (uint32_t i = 0; i < numPixels; ++i)  
    {   
        dst16[i] = (src16[i] & 0x0FFF) << 4;   
    }

    // Single debug block at the end
    LOGF_DEBUG("Mono12 expanded: first pixel 0x%04X -> 0x%04X", src16[0], dst16[0]);
}

void IDS_CCD::expand10bitPackedTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)
{
    LOG_INFO("KHK - Processing Packed Mono10p");
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
            uint32_t availableBits = 16 - (bitPos % 16);
            uint32_t bitsToTake = std::min(availableBits, bitsNeeded);
            uint32_t mask = (1 << bitsToTake) - 1;
            uint32_t shifted = (src16[srcIndex] >> (bitPos % 16)) & mask;

            value |= (shifted << bitsCollected);
            bitsCollected += bitsToTake;
            bitsNeeded -= bitsToTake;
            bitPos += bitsToTake;

            if (bitPos % 16 == 0) srcIndex++;
        }
        dst16[i] = static_cast<uint16_t>(value << 6);
    }
}

void IDS_CCD::expand12bitPackedTo16bit(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)
{
    const uint32_t numPixels = width * height;
    uint16_t *dst16 = reinterpret_cast<uint16_t *>(dst);

    // Mono12p: 2 pixels occupy 3 bytes
    // Byte 0: P1[11:4]
    // Byte 1: P1[3:0] (bits 7:4) | P2[3:0] (bits 3:0)
    // Byte 2: P2[11:4]
    for (uint32_t i = 0; i < numPixels; i += 2)
    {
        uint32_t baseSrc = (i * 3) / 2;
        
        // Pixel 1
        uint16_t p1 = (static_cast<uint16_t>(src[baseSrc]) << 4) | (src[baseSrc + 1] >> 4);
        dst16[i] = p1 << 4; // Scale to 16-bit

        // Pixel 2 (check if we aren't at the very last odd pixel)
        if (i + 1 < numPixels)
        {
            uint16_t p2 = (static_cast<uint16_t>(src[baseSrc + 2]) << 4) | (src[baseSrc + 1] & 0x0F);
            dst16[i + 1] = p2 << 4; // Scale to 16-bit
        }
    }
}

void IDS_CCD::setupTemperatureSensor()
{
    HasTemperature = false;
    tempNode = nullptr;
    tempSelectorNode = nullptr;

    if (!nodeMapRemoteDevice)
    {
        LOG_WARN("Cannot setup temperature sensor: node map is not available.");
        return;
    }

    try
    {
        tempSelectorNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("DeviceTemperatureSelector");

        if (tempSelectorNode && tempSelectorNode->IsAvailable() && tempSelectorNode->IsWriteable())
        {
            tempSelectorNode->SetCurrentEntry("Mainboard");
        }

        tempNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("DeviceTemperature");

        HasTemperature =
            tempNode &&
            tempNode->IsAvailable() &&
            tempNode->IsReadable();

        if (HasTemperature)
            LOG_INFO("Temperature sensor available.");
        else
            LOG_INFO("Temperature sensor not available.");
    }
    catch (const std::exception &e)
    {
        HasTemperature = false;
        tempNode = nullptr;
        tempSelectorNode = nullptr;
        LOGF_INFO("Temperature sensor not available: %s", e.what());
    }
}

/**
 * @brief Get current sensor temperature using cached nodes.
 * @return Temperature in Celsius, or -273.15 if unavailable.
 */

double IDS_CCD::getTemperature()
{
    if (!HasTemperature || !tempNode)
        return IDSConstants::INVALID_TEMPERATURE;

    try
    {
        if (!tempNode->IsAvailable() || !tempNode->IsReadable())
            return IDSConstants::INVALID_TEMPERATURE;

        return tempNode->Value();
    }
    catch (const std::exception &e)
    {
        LOGF_DEBUG("Failed to read temperature: %s", e.what());
        return IDSConstants::INVALID_TEMPERATURE;
    }
}

void IDS_CCD::updateTemperatureProperty()
{
    if (!HasTemperature)
        return;

    const double temperature = getTemperature();

    if (temperature <= IDSConstants::INVALID_TEMPERATURE)
        return;

    if (TemperatureNP.getDeviceName() == nullptr || strlen(TemperatureNP.getDeviceName()) == 0)
    {
        LOG_WARN("Skipping temperature update: Property not yet defined to INDI.");
        return;
    }

    TemperatureNP[0].setValue(temperature);

    if (isConnected())
        TemperatureNP.apply();
}

void IDS_CCD::updateBlackLevelRange()    
{   
    if (!HasOffset) return;    
        
    try    
    {    
        if (!blackLevelNode)  
            blackLevelNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("BlackLevel");  
  
        double minOffset = blackLevelNode->Minimum();    
        double maxOffset = blackLevelNode->Maximum();    
        double currentOffset = blackLevelNode->Value();    
        double offsetStep = 1.0;    
            
        if (blackLevelNode->HasConstantIncrement())    
        {    
            offsetStep = blackLevelNode->Increment();    
        }    
            
        // Update legacy INumberVectorProperty    
        OffsetNP[0].min = minOffset;    
        OffsetNP[0].max = maxOffset;    
        OffsetNP[0].step = offsetStep;    
        OffsetNP[0].value = currentOffset;    
            
        LOGF_INFO("BlackLevel range updated: %.2f - %.2f (step: %.2f, current: %.2f)",     
                  minOffset, maxOffset, offsetStep, currentOffset);    
    }    
    catch (const std::exception &e)    
    {    
        LOGF_ERROR("Failed to update blacklevel range: %s", e.what());    
    }    
}

void IDS_CCD::addCaptureFormat(const PixelFormatInfo &format)
{
    // 1. Store in our map for exposure logic
    // Key by real IDS/GenICam pixel format name.
    m_formatMap[format.idsName] = format;

    // 2. Create the INDI structure
    CaptureFormat indiFmt;

    // Internal switch name should be the IDS format name so SetCaptureFormat()
    // and grabImage() can both look up m_formatMap by the same key.
    indiFmt.name = format.idsName;

    // Human-readable label shown to the user.
    indiFmt.label = format.indiName;

    indiFmt.bitsPerPixel = format.bitsPerPixel;
    indiFmt.isLittleEndian = true;

    // 3. Determine default safely
    bool isDefault = false;

    if (pixelFormatNode && pixelFormatNode->CurrentEntry())
    {
        const std::string currentIDS =
            pixelFormatNode->CurrentEntry()->SymbolicValue();

        if (currentIDS == format.idsName)
            isDefault = true;
    }

    indiFmt.isDefault = isDefault;

    // 4. Register with INDI base class
    INDI::CCD::addCaptureFormat(indiFmt);
}

// Helper to map camera format names to INDI Bayer patterns  
std::string IDS_CCD::mapCameraFormatToBayerPattern(const std::string& formatName)  
{  
    if (formatName.find("RG") != std::string::npos) return "RGGB";  
    if (formatName.find("BG") != std::string::npos) return "BGGR";  
    if (formatName.find("GB") != std::string::npos) return "GBRG";  
    if (formatName.find("GR") != std::string::npos) return "GRBG";  
    return "";  
}  
  
// Helper to create Bayer PixelFormatInfo  
void IDS_CCD::addBayerCaptureFormats(std::vector<PixelFormatInfo> &captureFormats,
                                     const std::string &bayerPattern,
                                     const std::vector<int> &supportedDepths)
{
    LOG_INFO("=== Adding Bayer Capture Formats ===");
    LOGF_INFO("Detected Bayer pattern: %s", bayerPattern.c_str());

    size_t addedCount = 0;

    auto alreadyAdded =
        [&captureFormats](const std::string &idsName) -> bool
    {
        return std::any_of(captureFormats.begin(),
                           captureFormats.end(),
                           [&idsName](const PixelFormatInfo &fmt)
                           {
                               return fmt.idsName == idsName;
                           });
    };

    for (const auto &kv : m_formatMap)
    {
        const PixelFormatInfo &fmt = kv.second;

        if (fmt.idsName.find("Bayer") == std::string::npos)
            continue;

        const std::string pattern = mapCameraFormatToBayerPattern(fmt.idsName);

        if (pattern != bayerPattern)
            continue;

        if (alreadyAdded(fmt.idsName))
            continue;

        captureFormats.push_back(fmt);
        ++addedCount;

        LOGF_INFO("Added Bayer capture format: %s -> %s, output=%u-bit%s",
                  fmt.idsName.c_str(),
                  fmt.indiName.c_str(),
                  fmt.bitsPerPixel,
                  fmt.packed ? ", packed" : "");
    }

    if (addedCount == 0)
    {
        LOGF_WARN("No Bayer formats were added for pattern %s. Check m_formatMap population.",
                  bayerPattern.c_str());
    }
    else
    {
        LOGF_INFO("Added %zu Bayer capture formats.", addedCount);
    }

    LOG_INFO("=== Bayer Format Addition Complete ===");
}

// helper method
std::shared_ptr<peak::core::nodes::FloatNode> IDS_CCD::findExposureNode()
{
    if (!nodeMapRemoteDevice)
    {
        throw std::runtime_error("Remote device node map is not initialized.");
    }

    try
    {
        return nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
    }
    catch (const std::exception &e1)
    {
        LOGF_DEBUG("ExposureTime node not available, trying ExposureTimeAbs: %s", e1.what());

        try
        {
            return nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTimeAbs");
        }
        catch (const std::exception &e2)
        {
            LOGF_ERROR("Neither ExposureTime nor ExposureTimeAbs is available. ExposureTimeAbs error: %s", e2.what());
            throw;
        }
    }
}

void IDS_CCD::debugCurrentState()
{
    try
    {
        LOG_DEBUG("=== Current Camera State ===");

        if (nodeMapRemoteDevice)
        {
            try
            {
                auto userSetSelector =
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");

                if (userSetSelector && userSetSelector->IsAvailable())
                {
                    LOGF_DEBUG("UserSetSelector: %s",
                               userSetSelector->CurrentEntry()->SymbolicValue().c_str());
                }
            }
            catch (...)
            {
                LOG_DEBUG("UserSetSelector: unavailable");
            }
        }

        if (pixelFormatNode && pixelFormatNode->IsAvailable())
        {
            LOGF_DEBUG("PixelFormat: %s",
                       pixelFormatNode->CurrentEntry()->SymbolicValue().c_str());
        }
        else
        {
            LOG_DEBUG("PixelFormat: unavailable");
        }

        LOGF_DEBUG("Exposure range: %.6f - %.6f seconds, step %.6f",
                   minExposure, maxExposure, exposureStep);

        LOGF_DEBUG("Full exposure range: %.6f - %.6f seconds, LongExposure threshold %.6f",
                   fullMin, fullMax, longExposureMin);

        LOGF_DEBUG("Frame: %dx%d at (%d,%d), binning %dx%d",
                   PrimaryCCD.getSubW(), PrimaryCCD.getSubH(),
                   PrimaryCCD.getSubX(), PrimaryCCD.getSubY(),
                   PrimaryCCD.getBinX(), PrimaryCCD.getBinY());

        LOGF_DEBUG("Sensor: %dx%d, BPP: %d, Pixel size: %.2f x %.2f um",
                   cameraWidth, cameraHeight, cameraBPP, pixelSizeX, pixelSizeY);

        LOGF_DEBUG("State: InExposure=%s, IsAcquiring=%s, HasGain=%s, HasOffset=%s, HasTemperature=%s, HasBayer=%s",
                   InExposure.load() ? "true" : "false",
                   m_isAcquiring.load() ? "true" : "false",
                   HasGain ? "true" : "false",
                   HasOffset ? "true" : "false",
                   HasTemperature ? "true" : "false",
                   m_hasBayer ? "true" : "false");
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Failed to debug current state: %s", e.what());
    }
}

bool IDS_CCD::setupParams()
{
    if (!nodeMapRemoteDevice)
    {
        LOG_ERROR("NodeMap not initialized!");
        return false;
    }

    try
    {
        updateTemperatureProperty();

        // Query sensor dimensions
        widthNode  = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width");
        heightNode = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height");

        if (widthNode && heightNode)
        {
            cameraWidth  = static_cast<int>(widthNode->Value());
            cameraHeight = static_cast<int>(heightNode->Value());
            LOGF_INFO("Camera sensor size: %dx%d", cameraWidth, cameraHeight);
        }

        // Query pixel size directly from camera in micrometers
        pixelWidthNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("SensorPixelWidth");

        pixelHeightNode =
            nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("SensorPixelHeight");

        if (pixelWidthNode && pixelHeightNode)
        {
            pixelSizeX = pixelWidthNode->Value();
            pixelSizeY = pixelHeightNode->Value();
        }

        if (!nodeMapRemoteDevice)
        {
            LOG_ERROR("Remote device node map is null!");
            return false;
        }

        // Query pixel formats
        LOG_INFO("=== Starting pixel format query ===");
        queryPixelFormats();
        LOG_INFO("=== Pixel format query completed ===");

        LOG_INFO("=== Camera Supported Formats ===");
        for (const auto &depth : m_supportedBitDepths)
        {
            LOGF_INFO("Supported bit depth: %d bits", depth);
        }

        if (!pixelFormatNode || !pixelFormatNode->IsAvailable())
        {
            LOG_ERROR("PixelFormat node not available!");
            return false;
        }

        LOGF_INFO("Current camera format: %s",
                  pixelFormatNode->CurrentEntry()->SymbolicValue().c_str());

        LOGF_INFO("Camera default format: %s",
                  pixelFormatNode->CurrentEntry()->SymbolicValue().c_str());

        cameraBPP = 8;

        // Create capture formats with descriptive labels
        std::vector<PixelFormatInfo> captureFormats;

        LOGF_INFO("Using cached Bayer info: hasBayer=%s pattern=%s",
                  m_hasBayer ? "true" : "false",
                  m_bayerPattern.c_str());

        // Add 8-bit formats
        if (std::find(m_supportedBitDepths.begin(), m_supportedBitDepths.end(), 8) !=
            m_supportedBitDepths.end())
        {
            captureFormats.push_back(
                PixelFormatInfo("Mono8",
                                "8-bit",
                                8,
                                false,
                                true,
                                nullptr));
        }

        // Add 10-bit formats
        if (std::find(m_supportedBitDepths.begin(), m_supportedBitDepths.end(), 10) !=
            m_supportedBitDepths.end())
        {
            if (m_compressionSupport[10])
            {
                captureFormats.push_back(
                    PixelFormatInfo("Mono10p",
                                    "10-bit (compressed)",
                                    16,
                                    true,
                                    false,
                                    std::bind(&IDS_CCD::expand10bitPackedTo16bit,
                                              this,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3,
                                              std::placeholders::_4)));
            }

            captureFormats.push_back(
                PixelFormatInfo("Mono10",
                                "10-bit",
                                16,
                                false,
                                false,
                                std::bind(&IDS_CCD::expand10bitTo16bit,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          std::placeholders::_3,
                                          std::placeholders::_4)));
        }

        // Add 12-bit formats
        if (std::find(m_supportedBitDepths.begin(), m_supportedBitDepths.end(), 12) !=
            m_supportedBitDepths.end())
        {
            if (m_compressionSupport[12])
            {
                captureFormats.push_back(
                    PixelFormatInfo("Mono12p",
                                    "12-bit (compressed)",
                                    16,
                                    true,
                                    false,
                                    std::bind(&IDS_CCD::expand12bitPackedTo16bit,
                                              this,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3,
                                              std::placeholders::_4)));
            }

            captureFormats.push_back(
                PixelFormatInfo("Mono12",
                                "12-bit",
                                16,
                                false,
                                false,
                                std::bind(&IDS_CCD::expand12bitTo16bit,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          std::placeholders::_3,
                                          std::placeholders::_4)));
        }

        // Add Bayer formats if supported
        if (m_hasBayer && !m_bayerPattern.empty())
        {
            addBayerCaptureFormats(captureFormats, m_bayerPattern, m_supportedBitDepths);
            LOGF_INFO("Camera supports Bayer pattern: %s", m_bayerPattern.c_str());
        }

        // Add all capture formats to INDI
        LOG_INFO("=== Starting capture format creation ===");
        for (const auto &format : captureFormats)
        {
            LOGF_DEBUG("Adding capture format: %s", format.indiName.c_str());
            addCaptureFormat(format);
        }
        LOG_INFO("=== Capture format creation completed ===");

        // Set default format: prioritize Bayer if available, otherwise Mono
        try
        {
            // Stop and clear stream before changing payload-affecting camera settings
            try
            {
                if (dataStream && dataStream->IsGrabbing())
                {
                    dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
                    InExposure.store(false);
                    m_isAcquiring.store(false);
                }

                if (acquisitionStopNode && acquisitionStopNode->IsAvailable())
                {
                    acquisitionStopNode->Execute();
                    acquisitionStopNode->WaitUntilDone();
                }

                if (dataStream)
                {
                    dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);

                    for (auto &buffer : dataStream->AnnouncedBuffers())
                    {
                        dataStream->RevokeBuffer(buffer);
                    }
                }
            }
            catch (const std::exception &e)
            {
                LOGF_WARN("Failed to stop acquisition cleanly before format change: %s", e.what());
            }

            auto availableEntries = pixelFormatNode->Entries();

            std::string targetFormat;
            bool formatFound = false;

            // Prefer first available Bayer format on Bayer cameras, otherwise first Mono format.
            for (const auto &entry : availableEntries)
            {
                std::string name = entry->SymbolicValue();

                if (!entry->IsAvailable() || !entry->IsWriteable())
                    continue;

                if (m_hasBayer && name.find("Bayer") != std::string::npos)
                {
                    targetFormat = name;
                    formatFound = true;
                    break;
                }

                if (!m_hasBayer && name.find("Mono") == 0)
                {
                    targetFormat = name;
                    formatFound = true;
                    break;
                }

                // Keep Mono as fallback while searching for Bayer.
                if (targetFormat.empty() && name.find("Mono") == 0)
                    targetFormat = name;
            }

            if (!formatFound && !targetFormat.empty())
                formatFound = true;

            if (!formatFound)
            {
                LOG_WARN("No preferred pixel format found, trying Mono8 fallback.");
                targetFormat = "Mono8";
            }

            // Verify target format is actually available and writable.
            bool targetApplied = false;

            for (const auto &entry : availableEntries)
            {
                if (entry->SymbolicValue() == targetFormat &&
                    entry->IsAvailable() &&
                    entry->IsWriteable())
                {
                    pixelFormatNode->SetCurrentEntry(targetFormat);
                    LOGF_INFO("Set pixel format to %s", targetFormat.c_str());
                    targetApplied = true;
                    break;
                }
            }

            // If preferred format not available, use any available writable format.
            if (!targetApplied)
            {
                for (const auto &entry : availableEntries)
                {
                    if (entry->IsAvailable() && entry->IsWriteable())
                    {
                        const std::string fallbackFormat = entry->SymbolicValue();
                        pixelFormatNode->SetCurrentEntry(fallbackFormat);
                        LOGF_WARN("Preferred format unavailable, using: %s",
                                  fallbackFormat.c_str());
                        targetApplied = true;
                        break;
                    }
                }
            }

            if (!targetApplied)
            {
                LOG_WARN("Could not find any suitable pixel format, using camera default.");
            }

            // Update cameraBPP from the selected format if known.
            const std::string selectedFormat =
                pixelFormatNode->CurrentEntry()->SymbolicValue();

            auto fmtIt = m_formatMap.find(selectedFormat);
            if (fmtIt != m_formatMap.end())
            {
                cameraBPP = fmtIt->second.bitsPerPixel;
            }
            else
            {
                LOGF_WARN("Selected pixel format %s not found in format map; keeping BPP=%d",
                          selectedFormat.c_str(), cameraBPP);
            }

            // Reallocate buffers with new payload size.
            try
            {
                payloadSizeNode =
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize");

                size_t payloadSize = static_cast<size_t>(payloadSizeNode->Value());

                if (payloadSize == 0)
                {
                    LOG_ERROR("Payload size is 0 after pixel format change.");
                    return false;
                }

                LOGF_INFO("Allocating buffers with payload size: %zu", payloadSize);

                const size_t numBuffers = dataStream->NumBuffersAnnouncedMinRequired();

                for (size_t i = 0; i < numBuffers; ++i)
                {
                    auto buffer = dataStream->AllocAndAnnounceBuffer(payloadSize, nullptr);
                    dataStream->QueueBuffer(buffer);
                }
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("Failed to allocate buffers after format change: %s", e.what());
                return false;
            }

            allocateFrameBuffer();
            LOG_INFO("Frame buffer allocated after pixel format configuration.");
        }
        catch (const std::exception &e)
        {
            LOGF_WARN("Could not set default pixel format: %s", e.what());
        }

        // CCD parameters and full-frame ROI were already initialized before user-set switching.
        // Keep this sanity check for diagnostics only.
        if (PrimaryCCD.getSubW() <= 0 || PrimaryCCD.getSubH() <= 0)
        {
            LOG_WARN("PrimaryCCD frame was unexpectedly invalid after user-set setup. Restoring full frame.");
            PrimaryCCD.setFrame(0, 0, cameraWidth, cameraHeight);
        }

        SetCCDParams(cameraWidth, cameraHeight, cameraBPP, pixelSizeX, pixelSizeY);

        if (PrimaryCCD.getSubW() == 0)
        {
            PrimaryCCD.setFrame(0, 0, cameraWidth, cameraHeight);
        }

        // Gain selector is optional. Do not let it control exposure setup.
        LOG_INFO("=== Starting gain selector setup ===");

        const bool gainSelectorSelected = selectGainChannel();

        if (!gainSelectorSelected)
        {
            LOG_WARN("GainSelector not available or no suitable selector entry found. Continuing anyway.");
        }
        else
        {
            LOG_INFO("GainSelector configured successfully.");
        }

        // Do not overwrite HasGain here. HasGain should come from queryCameraCapabilities(),
        // where the actual Gain node is checked.
        if (HasGain)
        {
            LOG_INFO("Gain control is available.");
        }
        else
        {
            LOG_INFO("Gain control is not available.");
        }
        
        // Configure INDI CCD parameters before any UserSet switching.
        // switchUserSet() may restore ROI via UpdateCCDFrame(), so PrimaryCCD frame
        // must already be valid here.
        LOGF_DEBUG("Before early SetCCDParams: width=%d, height=%d, bpp=%d, pixelX=%.2f, pixelY=%.2f",
                   cameraWidth, cameraHeight, cameraBPP, pixelSizeX, pixelSizeY);

        if (cameraWidth <= 0 ||
            cameraHeight <= 0 ||
            cameraBPP <= 0 ||
            pixelSizeX <= 0 ||
            pixelSizeY <= 0)
        {
            LOG_ERROR("Invalid camera parameters detected before user-set setup.");
            LOGF_ERROR("Parameters: width=%d, height=%d, bpp=%d, pixelX=%.2f, pixelY=%.2f",
                       cameraWidth, cameraHeight, cameraBPP, pixelSizeX, pixelSizeY);
            return false;
        }

        SetCCDParams(cameraWidth, cameraHeight, cameraBPP, pixelSizeX, pixelSizeY);
        PrimaryCCD.setFrame(0, 0, cameraWidth, cameraHeight);

        LOGF_INFO("Initialized INDI CCD frame before user-set switching: %dx%d, %d bpp",
                  cameraWidth, cameraHeight, cameraBPP);

        // User set and exposure limits must always be initialized, regardless of gain support.
        LOG_INFO("=== Starting user set queries ===");

        std::vector<std::string> availableUserSets = queryAvailableUserSets();
        LOGF_INFO("Found %zu user sets", availableUserSets.size());

        if (availableUserSets.empty())
        {
            LOG_WARN("No user sets available on camera. Falling back to current exposure limits.");

            if (!queryExposureLimits())
            {
                LOG_ERROR("Could not query exposure limits.");
                return false;
            }

            fullMin = minExposure;
            fullMax = maxExposure;
            longExposureMin = maxExposure;
        }
        else
        {
            LOG_INFO("=== Starting user set exposure-range setup ===");

            if (std::find(availableUserSets.begin(), availableUserSets.end(), "Default") !=
                availableUserSets.end())
            {
                LOG_INFO("Switching to Default user set.");
                switchUserSet("Default");

                LOG_INFO("Querying exposure limits for Default.");
                queryExposureLimits();

                const double defaultMin = minExposure;
                const double defaultMax = maxExposure;

                if (std::find(availableUserSets.begin(), availableUserSets.end(), "LongExposure") !=
                    availableUserSets.end())
                {
                    LOG_INFO("Switching to LongExposure user set.");
                    switchUserSet("LongExposure");

                    LOG_INFO("Querying exposure limits for LongExposure.");
                    queryExposureLimits();

                    const double longMin = minExposure;
                    const double longMax = maxExposure;

                    longExposureMin = longMin;

                    fullMin = std::min(defaultMin, longMin);
                    fullMax = std::max(defaultMax, longMax);

                    LOG_INFO("Using both Default and LongExposure modes.");
                    LOGF_INFO("Default range: %.6fs - %.6fs", defaultMin, defaultMax);
                    LOGF_INFO("LongExposure range: %.6fs - %.6fs", longMin, longMax);
                }
                else
                {
                    fullMin = defaultMin;
                    fullMax = defaultMax;
                    longExposureMin = defaultMax;

                    LOG_INFO("Only Default user set available.");
                    LOGF_INFO("Exposure range: %.6fs - %.6fs", defaultMin, defaultMax);
                }

                // Return to Default mode for normal operation.
                switchUserSet("Default");
            }
            else
            {
                const std::string fallbackUserSet = availableUserSets[0];

                LOGF_WARN("No Default user set found, using: %s", fallbackUserSet.c_str());

                switchUserSet(fallbackUserSet);
                queryExposureLimits();

                fullMin = minExposure;
                fullMax = maxExposure;
                longExposureMin = maxExposure;
            }
        }

        PrimaryCCD.setMinMaxStep("CCD_EXPOSURE",
                                 "CCD_EXPOSURE_VALUE",
                                 fullMin,
                                 fullMax,
                                 exposureStep,
                                 false);

        LOGF_INFO("Full exposure range: %.6fs - %.6fs", fullMin, fullMax);

        // Offset / BlackLevel setup is independent of gain support.
        LOG_INFO("=== Setting up black level ===");

        HasOffset = setupBlackLevel();
        LOGF_INFO("Black level setup result: %s", HasOffset ? "success" : "failed");

        if (HasOffset)
        {
            try
            {
                if (!blackLevelNode)
                {
                    blackLevelNode =
                        nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("BlackLevel");
                }

                const double minOffset     = blackLevelNode->Minimum();
                const double maxOffset     = blackLevelNode->Maximum();
                const double currentOffset = blackLevelNode->Value();

                double offsetStep = 1.0;
                if (blackLevelNode->HasConstantIncrement())
                {
                    offsetStep = blackLevelNode->Increment();
                }

                OffsetNP[0].fill("OFFSET",
                                 "Offset",
                                 "%.2f",
                                 minOffset,
                                 maxOffset,
                                 offsetStep,
                                 currentOffset);

                OffsetNP.fill(getDeviceName(),
                              "CCD_OFFSET",
                              "Offset",
                              MAIN_CONTROL_TAB,
                              IP_RW,
                              60,
                              IPS_IDLE);

                LOGF_INFO("Offset range: %.2f - %.2f (step: %.2f, current: %.2f)",
                          minOffset, maxOffset, offsetStep, currentOffset);
            }
            catch (const std::exception &e)
            {
                LOGF_WARN("Failed to query Offset parameters: %s", e.what());
                HasOffset = false;
            }
        }

        debugCurrentState();

        return true;
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("Failed to query camera parameters: %s", e.what());
        return false;
    }
}
