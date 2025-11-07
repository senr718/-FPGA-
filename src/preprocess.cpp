/**
 * PCIe DMA Benchmark Tool
 * C++20 optimized version with enhanced safety and modern features
 */

#include "preprocess.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <cstdint>
#include <chrono>
#include <memory> 
#include <stdexcept>
#include <system_error>
#include <thread>  

// 使用命名空间组织代码
namespace pcie_dma 
{

    // 全局终止标志（使用原子类型确保线程安全）
    std::atomic<bool> g_terminate(false);

    // 信号处理函数
    static void signal_handler(int signal) 
    {
        g_terminate = true;
    }

    // 为 DMA 传输准备硬件资源
    static int initialize_dma(DMAContext& ctx) 
    {
        // Open DMA device
        if ((ctx.dma_fd = open(DMA_DEVICE_PATH, O_RDWR)) < 0) {
            std::perror("DMA device open failed");
            return -1;
        }

        // Open input device
        if ((ctx.input_fd = open(ctx.config.input_device, O_RDONLY | O_NONBLOCK)) < 0) {
            std::perror("Input device open failed");
            close(ctx.dma_fd);
            return -1;
        }

        std::printf("DMA and input devices opened successfully\n");
        return 0;
    }

    // 使用 NEON 指令实现的 64 字节对齐的内存复制函数
    static void aligned_memcpy(void* dest, const void* src, size_t size) 
    {
        /* 64-byte aligned memory copy using NEON instructions */
        if (size & (BUFFER_ALIGNMENT - 1)) {
            size = (size & -BUFFER_ALIGNMENT) + BUFFER_ALIGNMENT;
        }

        asm volatile(
            "sub %[dst], %[dst], #64 \n"
            "1: \n"
            "ldnp q0, q1, [%[src]] \n"
            "ldnp q2, q3, [%[src], #32] \n"
            "add %[dst], %[dst], #64 \n"
            "subs %[sz], %[sz], #64 \n"
            "add %[src], %[src], #64 \n"
            "stnp q0, q1, [%[dst]] \n"
            "stnp q2, q3, [%[dst], #32] \n"
            "b.gt 1b \n"
            : [dst] "+r"(dest), [src] "+r"(src), [sz] "+r"(size)
            :
            : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "cc", "memory");
    }

    static int perform_read_test(DMAContext& ctx, uint32_t* read_buf) 
    {
        DMAConfig cfg;
        cfg.src_phys_addr = ctx.config.device_address;
        cfg.dst_phys_addr = 0;
        cfg.direction = 1;      // Read operation
        cfg.chunk_size = ctx.config.transfer_size;
        cfg.total_size = ctx.config.transfer_size;
        
        // Setup DMA transfer
        uint32_t dev_addr = ioctl(ctx.dma_fd, DMA_CMD_SETUP, &cfg);
        if (!dev_addr) {
            std::fprintf(stderr, "DMA setup failed\n");
            return -1;
        }

        // Start DMA transfer
        if (ioctl(ctx.dma_fd, DMA_CMD_START, 0) < 0) {
            std::perror("DMA start failed");
            return -1;
        }

        // Wait for completion
        fd_set fds;
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        FD_ZERO(&fds);
        FD_SET(ctx.input_fd, &fds);

        int ret = select(ctx.input_fd + 1, &fds, nullptr, nullptr, &timeout);
        if (ret <= 0) {
            std::fprintf(stderr, "DMA read timeout\n");
            ioctl(ctx.dma_fd, DMA_CMD_SHUTDOWN, 0);
            return -1;
        }

        // Map memory for access
        ctx.mapped_memory = mmap(nullptr, cfg.total_size,
                                PROT_READ | PROT_WRITE,
                                MAP_SHARED, ctx.dma_fd, 0);
        if (ctx.mapped_memory == MAP_FAILED) {
            std::perror("Memory mapping failed");
            return -1;
        }

        // Perform memory copy and measure time
        auto start = std::chrono::high_resolution_clock::now();
        aligned_memcpy(read_buf, ctx.mapped_memory, cfg.total_size);
        auto end = std::chrono::high_resolution_clock::now();

        // Calculate metrics
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        ctx.metrics.cpu_read_time = duration.count();
        ctx.metrics.cpu_read_rate = (cfg.total_size / (ctx.metrics.cpu_read_time / 1000000.0)) / (1024 * 1024);


        // Get DMA timing
        ctx.metrics.dma_read_time = ioctl(ctx.dma_fd, DMA_CMD_GET_TIMING, 0);
        ctx.metrics.dma_read_rate = (cfg.total_size / (ctx.metrics.dma_read_time / 1000000.0)) / (1024 * 1024);

        // Cleanup
        munmap(ctx.mapped_memory, cfg.total_size);
        return ioctl(ctx.dma_fd, DMA_CMD_SHUTDOWN, 0);
    }

    static int i2c_read(int i2c_fd, unsigned char reg) 
    {
        unsigned char buf[1];
        if (write(i2c_fd, &reg, 1) != 1) {
            std::perror("Failed to write register address");
            return -1;
        }
        if (read(i2c_fd, buf, 1) != 1) {
            std::perror("Failed to read register value");
            return -1;
        }
        return buf[0];
    }

    static int i2c_write(int i2c_fd, unsigned char reg, unsigned char val) 
    {
        unsigned char buf[2] = {reg, val};
        if (write(i2c_fd, buf, 2) != 2) {
            return -1;
        }
        // std::printf("Wrote to register: 0x%02X  value 0x%02X\n", reg, val);
        return 0;
    }

    static int wait_for_reg_value(int i2c_fd, uint8_t reg_addr, uint8_t target_val) 
    {
        using namespace std::chrono;
        uint32_t elapsed_ms = 0;
        uint8_t reg_val;

        auto start = high_resolution_clock::now();
        
        while (elapsed_ms < WAIT_TIMEOUT_MS) 
        {
            reg_val = static_cast<uint8_t>(i2c_read(i2c_fd, reg_addr));
            if (reg_val == target_val) 
            {
                // std::printf("Success: Register 0x%02X value is 0x%02X (waited %d ms)\n", reg_addr, reg_val, elapsed_ms);
                return 0;
            }
            std::this_thread::sleep_for(milliseconds(WAIT_INTERVAL_MS));
            elapsed_ms = duration_cast<milliseconds>(high_resolution_clock::now() - start).count();
        }

        std::fprintf(stderr, "Error: Wait for register 0x%02X value 0x%02X timed out (total %d ms)\n",reg_addr, target_val, WAIT_TIMEOUT_MS);

        return -1;
    }



    static int buffer_to_mat(cv::Mat& ori_img, uint16_t width, uint16_t height, uint8_t* buffer) //1280*720
    {

        if (buffer == nullptr) 
        {
            return -1; 
        }
        
        cv::Mat temp(height, width, CV_8UC3, buffer);

        temp.copyTo(ori_img);

        if (ori_img.empty()) 
        {
            std::fprintf(stderr, "Failed to copy buffer to Mat\n");
            return -1;
        }

        // cv::namedWindow("Display");
        // cv::imshow("Display", ori_img);
        // cv::waitKey(0); 
        // std::printf("display-------\n");   

        
        return 0;
    }
    
    static void print_all_buffer_rgb_efficient(uint8_t* buffer, uint32_t buffer_size) 
    {
        std::printf("\n=== All RGB Buffer Data (%u pixels) ===\n", buffer_size / 3);
        
        uint32_t total_pixels = buffer_size / 3;
        const int PIXELS_PER_LINE = 16; 
        
        for (uint32_t line_start = 0; line_start < total_pixels; line_start += PIXELS_PER_LINE) 
        {
            uint32_t line_end = std::min(line_start + PIXELS_PER_LINE, total_pixels);
            
            // 打印行号范围
            std::printf("Pixels %6u-%6u: ", line_start, line_end - 1);
            
            // 打印该行所有像素
            for (uint32_t pixel = line_start; pixel < line_end; pixel++)
            {
                uint32_t base_index = pixel * 3;
                std::printf("(%3d,%3d,%3d) ", 
                        buffer[base_index],
                        buffer[base_index + 1],
                        buffer[base_index + 2]);
            }
            std::printf("\n");
            
            // 调整进度显示频率
            if (line_start % 400 == 0 && line_start > 0) {  // 400像素显示一次进度
                std::printf("--- Progress: %u/%u pixels (%.1f%%) ---\n", 
                        line_start, total_pixels, 
                        (float)line_start / total_pixels * 100.0f);
            }
        }
        
        std::printf("=== Completed: %u total pixels ===\n\n", total_pixels);
    }
    /**
     * @brief 
     * 
     * @param ori_img 
     * @param ctx 
     * @param i2c_fd 
     * @param read_frequency 
     * 
     * @return 0 读取到图像数据, -1 没有读取到图像
     */
    int Mat_read(cv::Mat& ori_img, DMAContext& ctx, int& i2c_fd, uint32_t& read_frequency)
    {
        // 分配读取缓冲区,申请65536个字节数量
        std::unique_ptr<uint32_t[]> read_buffer(new uint32_t[ctx.config.transfer_size/4]); // read_buffer,前面10位元素自定义，后面每个元素包含24位rgb888的数据以及8位自定义数据
        if (!read_buffer) 
        {
            std::perror("Memory allocation failed for read_buffer");
            return -1;
        }

        const uint16_t width  = 1280;
        const uint16_t height = 720;
        const uint32_t total_pixels = width * height;                       // 计算图像总像素数 (1280 * 720)
        const uint32_t pixel_data_size = total_pixels * 3;                  // 每个像素3个字节(RGB)
        std::unique_ptr<uint8_t[]> buffer(new uint8_t[pixel_data_size]);    // buffer，每个元素都包含一个通道的8位数据
        if (!buffer)
        {
            std::perror("Memory allocation failed for buffer");
            return -1;
        }

        // 记录buffer当前写入位置的偏移量
        uint32_t buffer_offset = 0;
        bool     frame_started = false;
        
        // 初始化要循环的次数
        // uint32_t time_cycles = ctx.config.test_cycles;
        // if (time_cycles == 0)
        // {
        //     std::fprintf(stderr, "test_cycles is zero\n");
        //     return -1;
        // }

        uint8_t last_high_byte = 0;
        uint8_t now_high_byte =0;
        uint32_t high_byte_repeat = 0;
        bool    break_should = false;

        // ANSI 转义码定义
        const char *HIGHLIGHT_START = "\033[1;31;43m"; // 粗体、红色文字、黄色背景
        const char *HIGHLIGHT_END = "\033[0m";         // 重置所有属性

        while (break_should == false)
        {           

            if (read_frequency % 2 == 0) 
            {
                if (wait_for_reg_value(i2c_fd, REGISTER0, 1) != 0) 
                {
                    std::printf("   Register 0x%02X value is 0x%02X \n", REGISTER0, i2c_read(i2c_fd, REGISTER0));
                    std::printf("   Register 0x%02X value is 0x%02X \n", REGISTER1, i2c_read(i2c_fd, REGISTER1));
                    return -1;
                }
                if (perform_read_test(ctx, read_buffer.get()) != 0)
                {
                    std::fprintf(stderr, "Read test failed\n");           
                    return -1;
                }
                if (i2c_write(i2c_fd, REGISTER1, 0x02) != 0) 
                {   
                    std::perror("Failed to write to register");      
                    return -1;
                }
                // std::printf("cur_fre==%u\n", read_frequency);
                read_frequency++;







                // 处理读取到的数据
                for (uint32_t i = 0; i < ctx.config.transfer_size/4 && buffer_offset < (pixel_data_size + 10); i++)
                {
                    // if (i % 8 == 0)
                    // printf("\n");
                    // printf("[%03u]: 0x%08X ",i, read_buffer[i]);   

                    uint32_t data = read_buffer[i];
                    uint8_t high_byte = (data >> 24) & 0xFF;
                    
                    if (high_byte == 0xFE)  // 帧开始信号
                    {
                        std::printf("%sfind 0xFE%s \n", HIGHLIGHT_START, HIGHLIGHT_END);
                        if (frame_started == false)
                        {

                            last_high_byte = now_high_byte;
                            now_high_byte = high_byte;

                            // 像素数据，提取RGB
                            uint8_t b = (data >> 16) & 0xFF;
                            uint8_t g = (data >> 8) & 0xFF;
                            uint8_t r = data & 0xFF;
                            
                            // 存储RGB数据到buffer
                            if (buffer_offset + 2 < pixel_data_size)
                            {
                                // buffer[buffer_offset++] = r;
                                // buffer[buffer_offset++] = g;
                                // buffer[buffer_offset++] = b;
                                buffer[buffer_offset++] = b;
                                buffer[buffer_offset++] = g;
                                buffer[buffer_offset++] = r;
                            }
                            else
                            {
                                std::printf("buffer have enough\n");                               
                                break_should = true;
                                break;
                            }

                            frame_started = true;
                            std::printf("%sFrame start%s \n", HIGHLIGHT_START, HIGHLIGHT_END);
                        }
                        else if (frame_started == true)
                        {
                            std::printf("problem: there is tow '0xFE' in one flame \n");
                        }
                        

                    }
                    // else if (high_byte == 0xFD) // 帧结尾信号
                    // {
                    //     std::printf("%sfind 0xFD%s \n", HIGHLIGHT_START, HIGHLIGHT_END);
                    //     // 帧开始信号
                    //     if (frame_started == true) 
                    //     {
                    //         frame_started = false;
                    //         std::printf("%sFrame end%s \n", HIGHLIGHT_START, HIGHLIGHT_END);
                    //     }
                    //     else if (false == frame_started)
                    //     {
                    //         std::printf("problem: find '0xFD' before '0xFE' \n");
                    //     }

                    //     break_should = true;
                    //     break;
                    // }
                    else if (frame_started)
                    {
                        last_high_byte = now_high_byte;
                        now_high_byte = high_byte;
                        if (now_high_byte != last_high_byte)
                        {
                            // 像素数据，提取RGB
                            uint8_t b = (data >> 16) & 0xFF;
                            uint8_t g = (data >> 8) & 0xFF;
                            uint8_t r = data & 0xFF;
                            
                            // 存储RGB数据到buffer
                            if (buffer_offset + 2 < pixel_data_size)
                            {
                                // buffer[buffer_offset++] = r;
                                // buffer[buffer_offset++] = g;
                                // buffer[buffer_offset++] = b;
                                buffer[buffer_offset++] = b;
                                buffer[buffer_offset++] = g;
                                buffer[buffer_offset++] = r;
                            }
                            else
                            {
                                std::printf("%sbuffer have enough%s\n", HIGHLIGHT_START, HIGHLIGHT_END);   
                                break_should = true;
                                break;
                            }
                        }
                        else if (now_high_byte == last_high_byte)
                        {
                            high_byte_repeat++;
                        }
                    }
                }
            } 
            else 
            {
                if (wait_for_reg_value(i2c_fd, REGISTER1, 1) != 0) 
                {
                    std::printf("   Register 0x%02X value is 0x%02X \n", REGISTER0, i2c_read(i2c_fd, REGISTER0));
                    std::printf("   Register 0x%02X value is 0x%02X \n", REGISTER1, i2c_read(i2c_fd, REGISTER1));
                    return -1;
                }
                if (perform_read_test(ctx, read_buffer.get()) != 0)
                {
                    std::fprintf(stderr, "Read test failed\n");
                    return -1;
                }
                if (i2c_write(i2c_fd, REGISTER0, 0x02) != 0) 
                {
                    std::perror("Failed to write to register");
                    return -1;
                }
                // std::printf("cur_fre==%u\n", read_frequency);
                read_frequency++;







                // 处理读取到的数据
                for (uint32_t i = 0; i < ctx.config.transfer_size/4 && buffer_offset < (pixel_data_size + 10); i++)
                {
                    // if (i % 8 == 0)
                    // printf("\n");
                    // printf("[%03u]: 0x%08X ",i, read_buffer[i]);   

                    uint32_t data = read_buffer[i];
                    uint8_t high_byte = (data >> 24) & 0xFF;
                    
                    if (high_byte == 0xFE)  // 帧开始信号
                    {
                        std::printf("%sfind 0xFE%s \n", HIGHLIGHT_START, HIGHLIGHT_END);
                        if (frame_started == false)
                        {

                            last_high_byte = now_high_byte;
                            now_high_byte = high_byte;

                            // 像素数据，提取RGB
                            uint8_t b = (data >> 16) & 0xFF;
                            uint8_t g = (data >> 8) & 0xFF;
                            uint8_t r = data & 0xFF;
                            
                            // 存储RGB数据到buffer
                            if (buffer_offset + 2 < pixel_data_size)
                            {
                                // buffer[buffer_offset++] = r;
                                // buffer[buffer_offset++] = g;
                                // buffer[buffer_offset++] = b;
                                buffer[buffer_offset++] = b;
                                buffer[buffer_offset++] = g;
                                buffer[buffer_offset++] = r;
                            }
                            else
                            {
                                std::printf("buffer have enough\n");                               
                                break_should = true;
                                break;
                            }

                            frame_started = true;
                            std::printf("%sFrame start%s \n", HIGHLIGHT_START, HIGHLIGHT_END);
                        }
                        else if (frame_started == true)
                        {
                            std::printf("problem: there is tow '0xFE' in one flame \n");
                        }
                        

                    }
                    // else if (high_byte == 0xFD) // 帧结尾信号
                    // {
                    //     std::printf("%sfind 0xFD%s \n", HIGHLIGHT_START, HIGHLIGHT_END);
                    //     // 帧开始信号
                    //     if (frame_started == true) 
                    //     {
                    //         frame_started = false;
                    //         std::printf("%sFrame end%s \n", HIGHLIGHT_START, HIGHLIGHT_END);
                    //     }
                    //     else if (false == frame_started)
                    //     {
                    //         std::printf("problem: find '0xFD' before '0xFE' \n");
                    //     }

                    //     break_should = true;
                    //     break;
                    // }
                    else if (frame_started)
                    {
                        last_high_byte = now_high_byte;
                        now_high_byte = high_byte;
                        if (now_high_byte != last_high_byte)
                        {
                            // 像素数据，提取RGB
                            uint8_t b = (data >> 16) & 0xFF;
                            uint8_t g = (data >> 8) & 0xFF;
                            uint8_t r = data & 0xFF;
                            
                            // 存储RGB数据到buffer
                            if (buffer_offset + 2 < pixel_data_size)
                            {
                                // buffer[buffer_offset++] = r;
                                // buffer[buffer_offset++] = g;
                                // buffer[buffer_offset++] = b;
                                buffer[buffer_offset++] = b;
                                buffer[buffer_offset++] = g;
                                buffer[buffer_offset++] = r;
                            }
                            else
                            {
                                std::printf("%sbuffer have enough%s\n", HIGHLIGHT_START, HIGHLIGHT_END);    
                                break_should = true;
                                break;
                            }
                        }
                        else if (now_high_byte == last_high_byte)
                        {
                            high_byte_repeat++;
                        }
                    }
                }
            }
        }


        std::printf("the repeat number is '%u' \n",high_byte_repeat);    

        if (buffer_offset != pixel_data_size)
        {
            std::printf("buffer offset is not enough\n");
            return -1;
        }
        
        if (buffer_to_mat(ori_img, width, height, buffer.get()) !=0)
        {
            std::printf("buffer to mat failed\n");
            return -1;
        }

        // print_all_buffer_rgb_efficient(buffer.get(),pixel_data_size);
        // while (1)
        // {
        //     /* code */
        // }
        return 0;
    }


    /**
     * @brief cleanup_resources
     * 
     * @param ctx
     * 
     * @note  DMA 相关的资源
     */
    void cleanup_resources(DMAContext& ctx) 
    {
        if (ctx.mapped_memory)   munmap(ctx.mapped_memory, ctx.config.transfer_size);
        if (ctx.dma_fd != -1)    close(ctx.dma_fd);
        if (ctx.input_fd != -1)  close(ctx.input_fd);
    }


    
    /**
     * @brief device init
     * 
     * @param ctx 
     * @param i2c_fd  
     * 
     * @return 0 init success,-1 init failed
     */
    int device_init(DMAContext& ctx, int& i2c_fd) 
    {
        // 注册信号处理
        std::signal(SIGINT, signal_handler);

        // pcie and dma device init
        AppConfig config = {};
        config.device_address = 0xf0200000;
        const char* dev_path = "/dev/input/event0";
        size_t dev_path_len = std::strlen(dev_path);
        std::strncpy(config.input_device, dev_path, std::min(dev_path_len, sizeof(config.input_device) - 1));
        config.input_device[sizeof(config.input_device) - 1] = '\0';
        config.test_cycles    = 57;
        config.transfer_size  = 65536;

        ctx.config = config;

        // dma init
        if (initialize_dma(ctx) < 0) 
        {
            std::exit(EXIT_FAILURE);
        }
        
        // i2c device init
        i2c_fd = open(I2C_DEVICE, O_RDWR);
        if (i2c_fd < 0) 
        {
            std::perror("Failed to open I2C device");
            return -1;
        }
        
        if (ioctl(i2c_fd, I2C_SLAVE, I2C_ADDR) < 0) 
        {
            std::perror("Failed to set I2C slave address");
            close(i2c_fd);
            return -1;
        }
        
        if (i2c_write(i2c_fd, REGISTER0, 0x02) != 0) // 初始化
        {
            std::perror("Failed to write to register");
            return -1;
        }

        return 0;
    }


} // namespace pcie_dma