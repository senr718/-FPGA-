#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono> // For std::chrono for timing
#include <thread> // For std::this_thread::sleep_for

// POSIX Socket API headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> // For close()

// --- FPGA 端配置参数 ---
const std::string FPGA_IP = "192.168.1.10";
const int FPGA_UDP_PORT = 12345; // 确保与FPGA的UDP接收端口一致

// --- 视频流参数 ---
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
// 对于 YUV420p，一帧是 width * height * 1.5 字节
// 对于 RGB24，是 width * height * 3 字节
const double PIXEL_FORMAT_BPP = 1.5; // Bytes Per Pixel for YUV420p
const int TARGET_FPS = 30;
const std::chrono::milliseconds FRAME_DELAY_MS(static_cast<long long>(1000.0 / TARGET_FPS));

// --- 数据包参数 ---
const int MAX_UDP_PAYLOAD_SIZE = 1400; // 建议小于1472 (IPv4)

// --- 视频文件路径 ---
const std::string VIDEO_FILE_PATH = "output.yuv"; // 替换为你的YUV文件路径

int main() {
    // 1. 计算一帧的字节数
    int frame_size = static_cast<int>(FRAME_WIDTH * FRAME_HEIGHT * PIXEL_FORMAT_BPP);
    if (frame_size <= 0) {
        std::cerr << "Error: Frame size is zero or negative. Check FRAME_WIDTH, FRAME_HEIGHT, PIXEL_FORMAT_BPP." << std::endl;
        return 1;
    }
    std::cout << "Calculated frame size (bytes): " << frame_size << std::endl;

    // 2. 创建 UDP Socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error: Failed to create socket." << std::endl;
        return 1;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(FPGA_UDP_PORT); // Convert port to network byte order
    if (inet_pton(AF_INET, FPGA_IP.c_str(), &dest_addr.sin_addr) <= 0) {
        std::cerr << "Error: Invalid IP address or address not supported." << std::endl;
        close(sockfd);
        return 1;
    }

    std::cout << "UDP Socket created for sending to " << FPGA_IP << ":" << FPGA_UDP_PORT << std::endl;

    // 3. 打开视频文件
    std::ifstream video_file(VIDEO_FILE_PATH, std::ios::binary);
    if (!video_file.is_open()) {
        std::cerr << "Error: Could not open video file at " << VIDEO_FILE_PATH << std::endl;
        close(sockfd);
        return 1;
    }

    std::vector<char> frame_data_buffer(frame_size);
    int frame_count = 0;

    try {
        while (true) {
            auto start_time = std::chrono::high_resolution_clock::now();

            // 读取一帧数据
            video_file.read(frame_data_buffer.data(), frame_size);
            if (video_file.gcount() < frame_size) {
                // 如果文件读取不足一帧，说明已到文件尾或文件太小
                if (video_file.eof()) {
                    std::cout << "End of video file. Looping back..." << std::endl;
                    video_file.clear(); // 清除EOF标志
                    video_file.seekg(0, std::ios::beg); // 回到文件开头
                    video_file.read(frame_data_buffer.data(), frame_size); // 再次尝试读取
                    if (video_file.gcount() < frame_size) { // 如果仍不足一帧，可能是文件太小
                        std::cerr << "Error: Video file is too small to contain a full frame, or empty." << std::endl;
                        break;
                    }
                } else {
                    std::cerr << "Error: Failed to read a full frame from file." << std::endl;
                    break;
                }
            }
            
            frame_count++;
            std::cout << "\n--- Sending Frame " << frame_count << " ---" << std::endl;

            // 将帧数据分片发送
            int bytes_sent_in_frame = 0;
            int total_frame_bytes = frame_size; // 或者 frame_data_buffer.size()

            while (bytes_sent_in_frame < total_frame_bytes) {
                int chunk_size = std::min(MAX_UDP_PAYLOAD_SIZE, total_frame_bytes - bytes_sent_in_frame);
                const char* chunk_data = frame_data_buffer.data() + bytes_sent_in_frame;

                ssize_t sent_bytes = sendto(sockfd, chunk_data, chunk_size, 0,
                                            (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                
                if (sent_bytes < 0) {
                    std::cerr << "Error sending chunk: " << strerror(errno) << std::endl;
                    break; // 退出当前帧的发送
                }
                // else {
                //     std::cout << "  Sent chunk of " << sent_bytes << " bytes (offset " << bytes_sent_in_frame << ")" << std::endl;
                // }

                bytes_sent_in_frame += sent_bytes;
            }
            std::cout << "  Total " << bytes_sent_in_frame << " bytes sent for Frame " << frame_count << std::endl;

            // 模拟帧率控制
            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            if (elapsed_time_ms < FRAME_DELAY_MS) {
                auto time_to_sleep = FRAME_DELAY_MS - elapsed_time_ms;
                std::this_thread::sleep_for(time_to_sleep);
                // std::cout << "  Sleeping for " << time_to_sleep.count() << " ms." << std::endl;
            } else {
                // std::cout << "  Warning: Frame " << frame_count << " took " << elapsed_time_ms.count()
                //           << "ms, exceeding target " << FRAME_DELAY_MS.count() << "ms." << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "An unexpected error occurred: " << e.what() << std::endl;
    }

    // 清理资源
    video_file.close();
    close(sockfd);
    std::cout << "Socket closed and resources cleaned up." << std::endl;

    return 0;
}