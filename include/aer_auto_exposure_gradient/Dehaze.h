#include <vector>
#include <string>
#include <ros/ros.h>
#include <cstdio>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

// This code is to implement the haze removal technique proposed by Kaiming He et al., 2011
// This code does not implement the soft matting part (i.e. Sec. 4.2 in the original paper)
// This code is referenced to https://github.com/terry77228/Image-Dehase by terry77228 on GitHub

class ImageDehazer
{
    public:
        bool LoadImage(const std::string& _filename);
        bool Dehaze(const cv::Mat m_Image, const int& _patchsize, const double& _t, const double& _w);
        bool WriteImage(const std::string& _filename);

        cv::Mat TransmissionMap;
        cv::Mat m_DarkChannelImage;
        cv::Mat m_Image;
        cv::Mat m_RecoveredImage;


    private:
        IplImage *m_InputImage;
        
        cv::Mat m_DoubleImage;
        

        IplImage *InputImg;
        double m_AtmosLight;

        void DarkChannelImage_Create(const int& _patchsize);
        double Atmospheric_Light_Estimate();
        void TransMap_Create(const int& _patchsize, const double& _t, const double& _w);
};
