/*-----------------------------------------------------------------------------
/
/
/
/----------------------------------------------------------------------------*/ 
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "serial_lib.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
/*---------------------------------------------------------------------------*/
using namespace cv;
using namespace std;
/*---------------------------------------------------------------------------*/
#define WIDTH 80
#define HEIGHT 60
#define UPSAMPLE 4  
/*---------------------------------------------------------------------------*/
void sync_image(int* port);
/*---------------------------------------------------------------------------*/
int main(int argc, const char** argv)
{
  int fd;
  int key;
  Mat dst;
  char msg;
  double fps;
  double sec; 
  time_t end;
  time_t start;       
  int frameCounter = 0;
  Size size(WIDTH*UPSAMPLE,HEIGHT*UPSAMPLE);
  Mat frame = Mat::ones(HEIGHT,WIDTH,CV_8U);

  fd = serialport_init("/dev/ttyUSB0",B3000000,'n');

  printf("Hello!\n");

  namedWindow("Frame!", 0);

  time(&start);

  while(1)
  {       
    /* sync the data stream by finding preamble */      
    sync_image(&fd);

    frameCounter++;

    /* get the image one pixel at a time */
    for(int i=0; i<frame.rows; i++)
    {
      for(int j=0; j<frame.cols; j++)
      {
        readRawBytes(fd,&msg,1,10000);

        frame.at<uchar>(i,j) = msg;
      }
    }

    #if 0
      printf(">");
      for(int i=0; i<frame.rows; i++)
      {
        printf("\r\n    ");
        for(int j=0; j<frame.cols; j++)
        {               
          printf("%2X ",frame.at<uchar>(i,j));
        }
      }
      printf("\r\n");
    #endif

    /* resize the image */
    resize(frame,dst,size);

    /* show it */
    imshow("Frame!",dst);

    time(&end);
    fps = frameCounter / sec;
    sec = difftime(end, start);    

    if((frameCounter % 20) == 0)
    {       
      printf("FPS = %.2f\n", fps);
    }

    /* press q to exit */
    key = waitKey(1);
    if(key > 0)
    {
      if(key == 'q')
      {
        return 0;
      }           
    } 

  }

  return 0;
}
/*---------------------------------------------------------------------------*/
void sync_image(int* port)
{   
  char ch;
  uint8_t data;   
  int state = 0;
  uint8_t run = 1;

  while(run)
  {
    if(readRawBytes(*port,&ch,1,10000) == 0)
    {
      data = ch;
      switch(state)
      {
        case 0:
        {
          if(data == 0xAA)
          {
            state = 1;
          }
          break;
        }
        case 1:
        {
          if(data == 0x55)
          {
            state = 2;
          }
          else
          {
            state = 0;
          }
          break;
        }
        case 2:
        {
          if(data == 0xAA)
          {
            run = 0;
          }
          else
          {
            state = 0;
          }
          break;
        }
      }
    }
  }

}
/*---------------------------------------------------------------------------*/

