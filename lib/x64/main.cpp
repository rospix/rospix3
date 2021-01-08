/**
 * Copyright (C) 2020 Daniel Turecek
 *
 * @file      main.cpp
 * @author    Daniel Turecek <daniel@turecek.de>
 * @date      2020-06-05
 *
 */
#include <cstdio>
#include <algorithm>
#include "clusteringapi.h"
#include <unistd.h>

#define SINGLE_CHIP_PIXSIZE 65536
#define ERRMSG_BUFF_SIZE    512

int printError(clhandle_t handle, const char* errorPrefix)
{
    char errorMsg[ERRMSG_BUFF_SIZE];
    pxpClGetLastError(handle, errorMsg, ERRMSG_BUFF_SIZE);
    printf("%s: %s\n", errorPrefix, errorMsg);
    return -1;
}

void sleepThread(double seconds)
{
    usleep(static_cast<__useconds_t>(seconds * 1000000));
}


void clMessageCallback(bool error, const char* message, void* userData)
{
    (void)userData;
    if (error)
        printf("ERR: %s\n", message);
    else
        printf("MSG: %s\n", message);
}

void clProgressCallback(bool finished, double progress, void* userData)
{
    (void)userData;
    printf("PRG: %f\n", progress);
    if (finished)
        printf("PRG: finished\n");
}

void clNewClustersCallback(PXPCluster* clusters, size_t clusterCount, size_t acqIndex, void* userData)
{
    (void)userData;
    printf("CLS: New clusters: count=%u, acqIndex=%u\n", static_cast<unsigned>(clusterCount), static_cast<unsigned>(acqIndex));
    for (size_t i = 0; i < std::min(clusterCount, static_cast<size_t>(15)); i++) {
        PXPCluster& cl = clusters[i];
        printf("CLS:  eid=%u, x=%f, y=%g, e=%g, toa=%g, size=%u, height=%g, roundnes=%g\n",
               cl.eventID, cl.x, cl.y, cl.energy, cl.toa, cl.size, cl.height, cl.roundness);
        fflush(stdout);
    }
}



int main(int arvc, char* argv[])
{
    printf("Loading Pixet core...\n");
    int rc = pxpClLoadPixetCore("pxcore.so");
    printf("Loading pixet core: rc=%d\n", rc);


    // wait for CdTe chip's bias to calm down
    printf("Waiting for bias to calm\n");
    sleepThread(1);
    printf("OK\n");


    clhandle_t handle = pxpClCreate(0);

    pxpClSetMessageCallback(handle, clMessageCallback, nullptr);
    pxpClSetProgressCallback(handle, clProgressCallback, nullptr);
    pxpClSetNewClustersCallback(handle, clNewClustersCallback, nullptr);

    rc = pxpClStartMeasurement(handle, 1.0, 3.0, "test.t3pa");
    //rc = pxpClReplayData(handle, "02_hodinky.t3pa", "output.t3pa", false);
    printf("rc=%d\n", rc); fflush(stdout);

    sleepThread(0.1);
    while (pxpClIsRunning(handle)) {
        sleepThread(0.1);
    }


    sleepThread(0.1);
    printf("Finished.\n");
    pxpClFree(handle);
    pxpClUnloadPixetCore();
    printf("Unloaded.\n");
    return 0;
}



