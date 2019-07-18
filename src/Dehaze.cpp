#include "aer_auto_exposure_gradient/Dehaze.h"
#include <queue>
#include <functional>
using namespace std;
using namespace cv;

bool ImageDehazer::LoadImage(const std::string& _filename)
{
    m_Image = cv::imread(_filename, CV_LOAD_IMAGE_COLOR);
    if(m_Image.empty()){
        cout << "Load image failed" << endl;
        return false;
    }

    m_Image.convertTo(m_DoubleImage, CV_64FC3);
    cout << "Image is loaded." << endl;
    return true;
}



bool ImageDehazer::Dehaze(cv::Mat m_Image, const int& _patchsize, const double& _t, const double& _w, cv::Mat &ToTransmission)

{
    if (m_Image.empty())
        {
        	return false;
        }
    
    m_Image.convertTo(m_DoubleImage, CV_64FC3); // Convert the image from uint8 to double
    cout << "pre-Dehaze Image is loaded." << endl;
    
    DarkChannelImage_Create(m_Image,_patchsize);
    m_AtmosLight = Atmospheric_Light_Estimate();

    TransMap_Create( m_Image, _patchsize, _t, _w, ToTransmission);

    return true;
}

bool ImageDehazer::WriteImage(const std::string& _filename)
{
    if (m_RecoveredImage.empty())
        return false;
    return imwrite(_filename.c_str(), m_RecoveredImage);
}



//void ShowImg(char *windowname, IplImage *image)
//{
    //cv::namedWindow(windowname, CV_WINDOW_NORMAL);
    //cv::imshow(windowname, image);
    //cv::waitKey(1);
    //cv::cvReleaseImage(&image);
    //cv::destroyWindow(windowname);
//}


void ImageDehazer::DarkChannelImage_Create(cv::Mat m_Image, const int& _patchsize)
{

    m_DarkChannelImage.create(m_Image.rows, m_Image.cols, CV_8UC1);

    for (int i = 0; i < m_Image.rows; ++i){
		for (int j = 0; j < m_Image.cols; ++j){
			unsigned char DarkVal = 255;
			for (int m = i - _patchsize / 2; m <= i + _patchsize / 2; ++m){
				for (int n = j - _patchsize / 2; n <= j + _patchsize / 2; ++n){
					if (m < 0 || n < 0 || m >= m_Image.rows || n >= m_Image.cols)
						continue;

					DarkVal = std::min(std::min(m_Image.at<Vec3b>(m, n)[0], m_Image.at<Vec3b>(m, n)[1]), m_Image.at<Vec3b>(m, n)[2]);
				}
			}
			m_DarkChannelImage.at<uchar>(i, j) = DarkVal;
		}
	}

	// cv::imwrite("dark.jpg", m_DarkChannelImage);
	std::cout << "The image dark channel is created" << std::endl;
	return;
}


double ImageDehazer::Atmospheric_Light_Estimate()
{

    std::priority_queue<uchar, vector<uchar>, std::greater<uchar>> TopValues;

	//find out the 0.1% highest pixels in the dark channel
	int TopAmounts = m_DarkChannelImage.rows * m_DarkChannelImage.cols / 1000;
	double total = 0;
	for(int i = 0 ; i < m_DarkChannelImage.rows; i++)
	{
		for(int j = 0 ;j < m_DarkChannelImage.cols; j++)
		{
			uchar pixel = m_DarkChannelImage.at<uchar>(i, j);
			if (TopValues.size() < TopAmounts){
				TopValues.push(pixel);
				total += pixel;
			}
			else{
				if (TopValues.top() >= pixel)
					continue;
				total -= TopValues.top();
				total += pixel;
				TopValues.pop();
				TopValues.push(pixel);
			}
		}
	}

	total /= TopAmounts;
	cout << "rows and cols are: " << m_DarkChannelImage.rows << "  " << m_DarkChannelImage.cols << endl;
	cout << "total is: " << total << endl;
	return total;
}



void ImageDehazer::TransMap_Create(cv::Mat m_Image, const int& _patchsize, const double& _t, const double& _w, cv::Mat &ToTransmission)

{
    TransmissionMap.create(m_Image.rows, m_Image.cols, CV_64FC1);
    m_RecoveredImage.create(m_Image.rows, m_Image.cols, CV_8UC3);

    for (int i = 0; i < m_Image.rows; i++)
    {

        for (int j = 0; j < m_Image.cols; j++)
        {
            double t = std::max( 1 - (_w*m_DarkChannelImage.at<uchar>(i, j)/m_AtmosLight), _t);

			m_RecoveredImage.at<Vec3b>(i, j)[0] = static_cast<uchar>(std::min(((m_Image.at<Vec3b>(i, j)[0] - m_AtmosLight) / t + m_AtmosLight), 255.0));
			m_RecoveredImage.at<Vec3b>(i, j)[1] = static_cast<uchar>(std::min(((m_Image.at<Vec3b>(i, j)[1] - m_AtmosLight) / t + m_AtmosLight), 255.0));
			m_RecoveredImage.at<Vec3b>(i, j)[2] = static_cast<uchar>(std::min(((m_Image.at<Vec3b>(i, j)[2] - m_AtmosLight) / t + m_AtmosLight), 255.0));


			TransmissionMap.at<double>(i, j) = t;
			//cout << "TransmissionMap is : " << TransmissionMap.at<double>(i, j) << endl;

        }
    }

    TransmissionMap.copyTo(ToTransmission);
    cout << " zzzz TransmissionMap is : " << ToTransmission.rows << endl;
    //cv::imshow("view", m_RecoveredImage); //comment this line in actual implementation
	//cv::waitKey(0); //comment this line in actual implementation


}















