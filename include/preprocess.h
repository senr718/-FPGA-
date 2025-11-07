/*
@author: Yume-naka
@date: 2025-10-20
@qq：412110095
*/
#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <linux/input.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <stdbool.h>
#include <libgen.h>
#include <sys/select.h>
#include <signal.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <atomic> 

#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace pcie_dma 
{
    #define DMA_DEVICE_PATH "/dev/pcie_dma_memcpy"
    #define VERSION_STRING "2.0"
    #define DEFAULT_TIMEOUT_MS 1000
    #define BUFFER_ALIGNMENT 64

    #define I2C_DEVICE "/dev/i2c-0"
    #define I2C_ADDR 0x66
    #define REGISTER0 0x00
    #define REGISTER1 0x01
    #define WAIT_TIMEOUT_MS 1000 // 总超时1秒（防止无限阻塞）
    #define WAIT_INTERVAL_MS 1   // 每1毫秒重试一次（避免I2C总线频繁操作）


    /* IOCTL Command Codes */
    enum DMA_COMMANDS
    {
        DMA_CMD_SETUP = 0x01000000,
        DMA_CMD_START = 0x02000000,
        DMA_CMD_GET_TIMING = 0x03000000,
        DMA_CMD_SHUTDOWN = 0x04000000
    };

    typedef struct
    {
        uint32_t device_address;
        uint32_t transfer_size;
        uint32_t test_cycles;
        char input_device[64];
    } AppConfig;

    typedef struct
    {
        uint32_t src_phys_addr;
        uint32_t dst_phys_addr;
        uint32_t direction;
        uint32_t chunk_size;
        uint32_t total_size;
    } DMAConfig;

    typedef struct
    {
        float dma_read_rate;
        float dma_write_rate;
        float cpu_read_rate;
        float cpu_write_rate;
        uint32_t dma_read_time;
        uint32_t dma_write_time;
        uint32_t cpu_read_time;
        uint32_t cpu_write_time;
        uint32_t error_count;
    } BenchmarkMetrics;

    typedef struct
    {
        int dma_fd;
        int input_fd;
        void *mapped_memory;
        AppConfig config;
        BenchmarkMetrics metrics;
    } DMAContext;

    extern std::atomic<bool> g_terminate;  // 改为atomic确保线程安全

 
    int Mat_read(cv::Mat& ori_img, DMAContext& ctx, int& i2c_fd, uint32_t& read_frequency); 
    void cleanup_resources(DMAContext& ctx);
    int device_init(DMAContext& ctx, int& i2c_fd); 
}


#endif // PREPROCESS_H