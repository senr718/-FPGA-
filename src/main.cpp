
#include <stdio.h>
#include <sys/time.h>
#include <thread>
#include <queue>
#include <vector>
#define _BASETSD_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "rknnPool.hpp"
#include "ThreadPool.hpp"
#include "preprocess.h"

using std::queue;
using std::time;
using std::time_t;
using std::vector;
int main(int argc, char *argv[])
{

  if (argc != 2)
  {
    printf("Usage: %s <rknn model> <jpg> \n", argv[0]);
    return -1;
  }

  char *model_name = argv[1];   
  printf("模型:\t%s\n", model_name);


  cv::namedWindow("Camera ");
  
  // device初始化
  pcie_dma::DMAContext ctx;
  int i2c_fd;
  if (!pcie_dma::device_init(ctx, i2c_fd))
  {
    printf("device init \n"); 
  }
  else
  {
    printf("device init failed\n");
    return -1;
  }

  // rk futs 初始化
  uint32_t thread_numbers = 8, frames = 0, read_frequency = 0;  
  vector<rknn_lite*> rkpool;    
  dpool::ThreadPool pool(thread_numbers);    
  queue<std::future<int>> futs; 

  for (int i = 0; i < thread_numbers; i++)
  {
    rknn_lite* ptr = new rknn_lite(model_name, thread_numbers);

    rkpool.push_back(ptr); 

    if (pcie_dma::Mat_read(ptr->ori_img, ctx, i2c_fd, read_frequency) != 0) 
    {
      std::printf("read mat failed --when rknn_lite init\n");
      return -1;
    }

    futs.push(pool.submit(&rknn_lite::interf, ptr));
  }


  struct timeval time;
  gettimeofday(&time, nullptr);
  auto initTime = time.tv_sec * 1000 + time.tv_usec / 1000;
  gettimeofday(&time, nullptr);
  long tmpTime, lopTime = time.tv_sec * 1000 + time.tv_usec / 1000;


  long time1 = 0;
  long time2 = 0;

  while (1)
  {
 
    if (futs.front().get() != 0)  
      {break;}

    futs.pop();

    cv::imshow("Camera FPS", rkpool[frames % thread_numbers]->ori_img); 
    std::printf("frames : '%u'\n",frames);
    if (cv::waitKey(1) == 'q')
    {break;}

    gettimeofday(&time, nullptr);
    time1 = time.tv_sec * 1000 + time.tv_usec / 1000;
    if (pcie_dma::Mat_read(rkpool[frames % thread_numbers]->ori_img, ctx, i2c_fd, read_frequency) != 0)
    {
      std::printf("read mat failed --when futs\n");
      break;
    }
    gettimeofday(&time, nullptr);
    time2 = time.tv_sec * 1000 + time.tv_usec / 1000;

    std::printf("time : '%f'\n", (float)(time2 - time1));

  
    futs.push(pool.submit(&rknn_lite::interf, rkpool[frames++ % thread_numbers]));
    if(frames % 60 == 0)
    {
        gettimeofday(&time, nullptr);
        tmpTime = time.tv_sec * 1000 + time.tv_usec / 1000;
        printf("fps is '%f' \n", 60000.0 / (float)(tmpTime - lopTime));
        lopTime = tmpTime;
    }
  }

  gettimeofday(&time, nullptr);
  printf("\nthe fps is '%f' \n", float(frames) / (float)(time.tv_sec * 1000 + time.tv_usec / 1000 - initTime + 0.0001) * 1000.0);
  printf("\nthe thread numbers is '%d' \n", thread_numbers);

  while (!futs.empty())
  {
    if (futs.front().get())    
      break;
    futs.pop();
  }

  for (int i = 0; i < thread_numbers; i++)  
    delete rkpool[i];

  cv::destroyAllWindows();

  pcie_dma::cleanup_resources(ctx);
  close(i2c_fd);

  return 0;
}
