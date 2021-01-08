/**
 * Copyright (C) 2020 Daniel Turecek
 * @author  Daniel Turecek <daniel.turecek@advacam.com>
 */
#ifndef CLUSTERINGAPI_H
#define CLUSTERINGAPI_H

#include <cstdio>
#include <cstdlib>

#ifndef WIN32
    #define PXCLAPI extern "C" __attribute__ ((visibility("default")))
#else
    #define PXCLAPI extern "C" __declspec(dllexport)
#endif

// Contants:
#define CL_API_VERSION 1
#define CL_INVALID_HANDLE 0
#define CL_NO_DEVICE -1


// Error Codes:
#define CL_ERR_CANNOT_LOAD_PIXET -1
#define CL_ERR_PIXET_NOT_LOADED -2
#define CL_ERR_INVALID_HANDLE -3
#define CL_ERR_INVALID_DEVICE_INDEX -4

#define CL_ERR_INVALID_ARGUMENT -100
#define CL_ERR_CALIB_DIMENSION_MISTMATCH -101
#define CL_ERR_CANNOT_OPEN_FILE -102
#define CL_ERR_CANNOT_READ_FRAME -103
#define CL_ERR_CANNOT_MEASURE -104
#define CL_ERR_LOCK_TIMEOUT -105
#define CL_ERR_DIMENSION_MISTMATCH -106
#define CL_ERR_NO_DATA -107
#define CL_ERR_INVALID_RECT -108
#define CL_ERR_CANNOT_READ_FILE -109
#define CL_ERR_CANNOT_SAVE_FILE -110
#define CL_ERR_CANNOT_MASK_PIXEL -111

// Definition of Cluster Pixel
typedef struct _PXPPixel {
    unsigned short x; // x coordinate of pixel
    unsigned short y; // y coordinate of pixel
    double toa; // time of arrival in nano seconds
    float energy; // energy of pixel in keV or ToT count if no calibration
} PXPPixel;

// Definition of cluster
typedef struct _PXPCluster {
    unsigned eventID; // event id to recognize coincidence events (same ID)
    float x; // x coordinate of cluster
    float y; // y coordinate of cluster
    float energy; // energy of cluster in keV
    double toa; // time of arrival
    unsigned short size; // size of cluster in pixels (number of pixels)
    float height; // maximal pixel value in cluster
    float roundness; // roundness (0 - 1)
} PXPCluster;

// Definition of cluster with Pixel data
typedef struct _PXPClusterWithPixels {
    unsigned eventID; // event id to recognize coincidence events (same ID)
    float x; // x coordinate of cluster
    float y; // y coordinate of cluster
    float energy; // energy of cluster in keV
    double toa; // time of arrival
    unsigned short size; // size of cluster in pixels (number of pixels)
    float height; // maximal pixel value in cluster
    float roundness; // roundness (0 - 1)
    PXPPixel* pixels;
} PXPClusterWithPixels;


/// Internal handle used for instance of each Clustering object
typedef void* clhandle_t;

/// Callback for messages and error messages returned from the SDK
/// @param error - true if error message
/// @param message - text of the message
/// @param userData - data of the user that were set in set callback function
typedef void (*ClMessageCallback)(bool error, const char* message, void* userData);

/// Callback for progress of an operation
/// @param finished - true if operation finished
/// @param progress - percentage of progress 0 - 100
/// @param userData - data of the user that were set in set callback function
typedef void (*ClProgressCallback)(bool finished, double progress, void* userData);

/// Callback for when a single acquisition is started
/// @param acqIndex - index of the current acquisition
/// @param userData - data of the user that were set in set callback function
typedef void (*ClAcqStartedCallback)(int acqIndex, void* userData);

/// Callback for when a single acquisition is finished
/// @param acqIndex - index of the current acquisition
/// @param userData - data of the user that were set in set callback function
typedef void (*ClAcqFinishedCallback)(int acqIndex, void* userData);

/// Callback when new clusters are measured/processed
/// Note: Only one of the new clusters callback can be set, either NewClustersCallback or NewClustersWithPixelsCallback
/// @param clusters - array of new clusters
/// @param clusterCount - size of clusters array
/// @param acqIndex - index of curent acquisition
/// @param userData - data of the user that were set in set callback function
typedef void (*ClNewClustersCallback)(PXPCluster* clusters, size_t clusterCount, size_t acqIndex, void* userData);

/// Callback when new clusters are measured/processed.
/// Note: Only one of the new clusters callback can be set, either NewClustersCallback or NewClustersWithPixelsCallback
/// @param clusters - array of new clusters (with pixels)
/// @param clusterCount - size of clusters array
/// @param acqIndex - index of curent acquisition
/// @param userData - data of the user that were set in set callback function
typedef void (*ClNewClustersWithPixelsCallback)(PXPClusterWithPixels* clusters, size_t clusterCount, size_t acqIndex, void* userData);




/// Loads the pixet core library (pxcore.dll/so). When the measurement with a device is intented
/// the user has to either load pixet core with this function, or if the core is already loaded in
/// the application (pxcore.dll/so was loaded separatelly), the setIPixet function has to be called.
/// @param[in] pxCoreLibPath full path to pxcore.dll or pxcore.so library
/// @return 0 if OK, otherwise error code (SI_ERR_XXX)
PXCLAPI int pxpClLoadPixetCore(const char* pxCoreLiPath);

/// Deinitializes and unloads the pixet core
/// @return 0 if OK, otherwise error code (SI_ERR_XXX)
PXCLAPI void pxpClUnloadPixetCore();

/// Sets the internal Pixet API pointer. This is used when pxcore library is loaded separatelly in application.
/// The use must pointer obtained via function pxcGetIPixet and should not load the pixet core with pxpSiLoadPixetCore
/// function.
/// @param pixet - pointer to internal Pixet structure obtained from pxcGetIPixet
PXCLAPI void pxpClSetIPixet(void* pixet);

/// Returns internal Pixet structure pointer
/// @return internal Pixet structure pointer or 0 if not set
PXCLAPI void* pxpClGetIPixet();

/// Gets the last error message - when a function returns error code. Gets either
/// global message when handle = 0, or message for the specific SpectraImaging instance
/// @param[in] handle - spectra imaging handle received from function pxpSiCreate
/// @param[out] errorMsgBuffer - output buffer where the error message will be stored
/// @param[in] size - size of the supplied errorMsgBuffer
/// @return 0 if OK, otherwise error code (SI_ERR_XXX)
PXCLAPI int pxpClGetLastError(clhandle_t handle, char* errorMsgBuffer, unsigned size);



/// Creates a new instance of clustering
/// @param[in] deviceIndex - index of the device this clustering instance will manage.
///                          If used offline (pxcore library not loaded), use CL_NO_DEVICE.
///                          The measurement will be not possible. Only replaying of data.
/// @return returns the handle of newly create instance of Spectra Imaging, or CL_INVALID_HANDLE if error
PXCLAPI clhandle_t pxpClCreate(int deviceIndex=CL_NO_DEVICE);

/// Frees the created instance of clustering
/// @param[in] handle - spectra imaging handle
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClFree(clhandle_t handle);

/// Replays and process already measured data files (*.pmf, *.txt, *.t3r, *.t3pa, ...)
/// @param[in] handle - Clustering instance handle
/// @param[in] filePath - full path to a data file to be replayed
/// @param[in] outputFilePath - full path to a output file (e.g. cluster log *.clog). if saving to output file not required, put ""
/// @param[in] blocking - whether the function will block until all clusters processed
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClReplayData(clhandle_t handle, const char* filePath, const char* outputFilePath, bool blocking);

/// Starts measuremnt with the device for specified time and process the data
/// @param[in] handle - Clustering instance handle
/// @param[in] acqTime - acquisition time of a single frame / pixel measurement in seconds.
/// @param[in] measTime - total time of measurement in seconds.
/// @param[in] outptuFilePath - output file where the process data (clusters) will be saved (*.clog). If saving not required, put ""
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClStartMeasurement(clhandle_t handle, double acqTime, double measTime, const char* outputFilePath);

/// Aborts the measurement or replaying of the data
/// @param[in] handle - Clustering instance handle
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClAbort(clhandle_t handle);

/// Loads the calibrations from the device configuration
/// @param[in] handle - Clustering instance handle
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClLoadCalibrationFromDevice(clhandle_t handle);

/// Loads the calibration files (a,b,c,t files or a single xml device config file)
/// @param[in] handle - Clustering instance handle
/// @param[in] fielPaths - a full path to either configuration xml file or full paths to the a,b,c,t files separated by | character. e.g. /tmp/a.txt|/tmp/b.txt|/tmp/c.txt|/tmp/t.txt
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClLoadCalibrationFromFiles(clhandle_t handle, const char* filePaths);

/// Returns if the measurement or replayin of data is running
/// @param[in] handle - Clustering instance handle
/// @return > 0 if running, 0 = not running, < 0 error
PXCLAPI int pxpClIsRunning(clhandle_t handle);

/// Sets the (error) message callback function
/// @param[in] handle - Clustering instance handle
/// @param[in] callback - callback function
/// @param[in] userData - custom user data that are passed in the callback
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClSetMessageCallback(clhandle_t handle, ClMessageCallback callback, void* userData);

/// Sets the progress callback function
/// @param[in] handle - Clustering instance handle
/// @param[in] callback - callback function
/// @param[in] userData - custom user data that are passed in the callback
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClSetProgressCallback(clhandle_t handle, ClProgressCallback callback, void* userData);

/// Sets the acquisition started callback function
/// @param[in] handle - Clustering instance handle
/// @param[in] callback - callback function
/// @param[in] userData - custom user data that are passed in the callback
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClSetAcqStartedCallback(clhandle_t handle, ClAcqStartedCallback callback, void* userData);

/// Sets the acquisition finished callback function
/// @param[in] handle - Clustering instance handle
/// @param[in] callback - callback function
/// @param[in] userData - custom user data that are passed in the callback
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClSetAcqFinishedCallback(clhandle_t handle, ClAcqFinishedCallback callback, void* userData);

/// Sets the new clusters callback
/// Note: Only one of the new clusters callback can be set, either NewClustersCallback or NewClustersWithPixelsCallback
/// @param[in] handle - Clustering instance handle
/// @param[in] callback - callback function
/// @param[in] userData - custom user data that are passed in the callback
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClSetNewClustersCallback(clhandle_t handle, ClNewClustersCallback callback, void* userData);


/// Sets the new clusters (with pixel data) callback
/// Note: Only one of the new clusters callback can be set, either NewClustersCallback or NewClustersWithPixelsCallback
/// @param[in] handle - Clustering instance handle
/// @param[in] callback - callback function
/// @param[in] userData - custom user data that are passed in the callback
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClSetNewClustersWithPixelsCallback(clhandle_t handle, ClNewClustersWithPixelsCallback callback, void* userData);

/// Enables/disables filtering of noisy pixels
/// @param[in] handle - Clustering instance handle
/// @param[in] enabe - enable/disable filtering
PXCLAPI int pxpClEnableFilteringOfNoisyPixels(clhandle_t handle, bool enable);

/// Masks noisy pixels that were found during previous pixel measurement
/// @param[in] handle - Clustering instance handle
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClMaskNoisyPixels(clhandle_t handle);

/// Masks a pixel in pixel matrix
/// @param[in] handle - Clustering instance handle
/// @param[in] pixelIndex - index of pixel in pixel matrix starting with 0
/// @param[in] masked - if pixel should be masked
/// @return 0 if OK, otherwise error code (CL_ERR_XXX)
PXCLAPI int pxpClMaskPixel(clhandle_t handle, unsigned pixelIndex, bool masked);



#endif /* !CLUSTERINGAPI_H */

