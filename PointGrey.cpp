#include "PointGrey.h"

#include <graphics/sys.h>
#include <opencv2/world.hpp>
#include <opencv2/opencv.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;


// This function retrieves a single image using the trigger. In this example,
// only a single image is captured and made available for acquisition - as such,
// attempting to acquire two images for a single trigger execution would cause
// the example to hang. This is different from other examples, whereby a
// constant stream of images are being captured and made available for image
// acquisition.
int PointGreyCapture::GrabNextImageByTrigger(INodeMap& nodeMap, CameraPtr pCam)
{
	int result = 0;

	try
	{
		//
		// Use trigger to capture image
		//
		// *** NOTES ***
		// The software trigger only feigns being executed by the Enter key;
		// what might not be immediately apparent is that there is not a
		// continuous stream of images being captured; in other examples that
		// acquire images, the camera captures a continuous stream of images.
		// When an image is retrieved, it is plucked from the stream.
		//
		if (!trigger_)
		{
			// Get user input
			//LOG( "Press the Enter key to initiate software trigger.\n");

			// Execute software trigger
			CCommandPtr ptrSoftwareTriggerCommand = nodeMap.GetNode("TriggerSoftware");
			if (!IsAvailable(ptrSoftwareTriggerCommand))
			{
				LOG("Unable to execute trigger. Aborting...\n");
				return -1;
			}

			ptrSoftwareTriggerCommand->Execute();
			//Sleep(20);
			//LOG("trigger!!!");
			// TODO: Blackfly and Flea3 GEV cameras need 2 second delay after software trigger
		}
		else
		{
			// Execute hardware trigger
			//LOG("Use the hardware to trigger image acquisition.\n");
		}
	}
	catch (Spinnaker::Exception& e)
	{
		//LOG("Error: %s\n", e.what());
		result = -1;
	}

	return result;
}

void PointGreyCapture::SaveToDisk()
{
	for (int i = 0; i < images_.size(); i++) {
		
			ImagePtr convertedImage = images_[i]->Convert(PixelFormat_RGB8, HQ_LINEAR);
			char filename[500]; 
			sprintf(filename, "pt_grey\\%d_%02d.bmp", cam_id_, i);

			int h = convertedImage->GetHeight();
			int w = convertedImage->GetWidth();
			int new_w = convertedImage->GetHeight();
			int new_h = convertedImage->GetWidth();
			cv::Mat mColorImage = cv::Mat(h, w, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
			cv::Mat mGrayImage;
			if (rot_) {
				cv::Mat rotImage = cv::Mat(w, h, CV_8UC3);
				for (int x = 0; x < new_w; x++) {
					for (int y = 0; y < new_h; y++) {
						rotImage.at<cv::Vec3b>(y, x) = mColorImage.at<cv::Vec3b>(h - x - 1, y);
					}
				}
				
				cv::cvtColor(rotImage, mGrayImage, CV_BGR2GRAY);
			}
			else {
				cv::cvtColor(mColorImage, mGrayImage, CV_BGR2GRAY);
			}


			cv::imwrite(filename, mGrayImage);
	}
}

int PointGreyCapture::ConfigureTrigger(bool hardware)
{
	trigger_ = hardware;
	INodeMap& nodeMap = camera_->GetNodeMap();
	int result = 0;

	LOG("*** CONFIGURING TRIGGER ***\n");

	if (!hardware)
	{
		LOG("Software trigger chosen...\n");
	}
	else
	{
		LOG("Hardware trigger chosen...\n");
	}

	try
	{
		CEnumerationPtr ptrTriggerSelector = nodeMap.GetNode("TriggerSelector");
		if (!IsAvailable(ptrTriggerSelector) || !IsReadable(ptrTriggerSelector))
		{
			LOG("Unable to disable trigger selector (node retrieval). Aborting...\n");
			return -1;
		}
		CEnumEntryPtr mode_frame_start = ptrTriggerSelector->GetEntryByName("FrameStart");
		if (!IsAvailable(mode_frame_start) || !IsReadable(mode_frame_start))
		{
			LOG("Unable to disable trigger selector (enum entry retrieval). Aborting...\n");
			return -1;
		}
		ptrTriggerSelector->SetIntValue(mode_frame_start->GetValue());

		CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
		if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
		{
			LOG("Unable to disable trigger mode (node retrieval). Aborting...\n");
			return -1;
		}

		CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
		if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
		{
			LOG("Unable to disable trigger mode (enum entry retrieval). Aborting...\n");
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());
		LOG("Trigger mode disabled......\n");


		CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
		if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource))
		{
			LOG("Unable to set trigger mode (node retrieval). Aborting...\n");
			return -1;
		}

		if (!hardware)
		{
			// Set trigger mode to software
			CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
			if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware))
			{
				LOG("Unable to set trigger mode (enum entry retrieval). Aborting...\n");
				return -1;
			}

			ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
			LOG("Trigger source set to software...\n");

			CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("On");
			if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
			{
				LOG("Unable to set trigger mode (enum entry retrieval). Aborting...\n");
				return -1;
			}

			ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());
			LOG("Trigger mode turned back on...\n");
		}
		else
		{
			// Set trigger mode to hardware ('Line0')
			CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line3");
			if (!IsAvailable(ptrTriggerSourceHardware) || !IsReadable(ptrTriggerSourceHardware))
			{
				LOG("Unable to set trigger mode (enum entry retrieval). Aborting...\n");
				return -1;
			}

			ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());
			//CEnumerationPtr triggerActivation = nodeMap.GetNode("TriggerActivation");
			//triggerActivation->SetIntValue(triggerActivation->GetEntryByName("RisingEdge")->GetValue());
			LOG("Trigger source set to hardware...\n");

			CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
			if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
			{
				LOG("Unable to set trigger mode (enum entry retrieval). Aborting...\n");
				return -1;
			}

			ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());
			LOG("Trigger mode turned back on...\n");

			// Set Trigger Overlap to Readout
			CEnumerationPtr ptrTriggerOverlap = nodeMap.GetNode("TriggerOverlap");
			if (!IsAvailable(ptrTriggerOverlap) || !IsReadable(ptrTriggerOverlap))
			{
				LOG("Unable to set Trigger Overlap (enum retrieval). Aborting...");
				return -1;
			}

			CEnumEntryPtr ptrTriggerOverlapRO = ptrTriggerOverlap->GetEntryByName("ReadOut");
			if (!IsAvailable(ptrTriggerOverlapRO) || !IsReadable(ptrTriggerOverlapRO))
			{
				LOG("Unable to set TriggerOverlap (entry retrieval). Aborting...");
				return -1;
			}

			int64_t TriggerOverlapRO = ptrTriggerOverlapRO->GetValue();

			ptrTriggerOverlap->SetIntValue(TriggerOverlapRO);
		}





	}
	catch (Spinnaker::Exception& e)
	{
		LOG("error\n");
		result = -1;
	}

	return result;
}
void PointGreyCapture::SetExposureTriggerWidth()
{
	INodeMap& nodeMap = camera_->GetNodeMap();

	CEnumerationPtr ptrExposureMode = nodeMap.GetNode("ExposureMode");
	if (!IsAvailable(ptrExposureMode) || !IsWritable(ptrExposureMode))
	{
		LOG("Unable to set exposure mode  Aborting...");
		return;
	}

	CEnumEntryPtr ptrTrigerWidth = ptrExposureMode->GetEntryByName("TriggerWidth");
	if (!IsAvailable(ptrTrigerWidth) || !IsReadable(ptrTrigerWidth))
	{
		cout << "Unable to set exposure mode triggerwidth. Aborting..." << endl
			<< endl;
		return;
	}

	int64_t triggerwidth = ptrTrigerWidth->GetValue();

	ptrExposureMode->SetIntValue(triggerwidth);

	LOG("exposure mode set to triggerwidth...");
}

void PointGreyCapture::SetMisc(double Blance, double Gain, double BlackLevel)
{
	INodeMap& nodeMap = camera_->GetNodeMap();
	CEnumerationPtr balanceSelector = nodeMap.GetNode("BalanceRatioSelector");
	if (!IsAvailable(balanceSelector) || !IsWritable(balanceSelector))
	{
		LOG("Unable to set balance selector  Aborting...");
		return;
	}
	CEnumEntryPtr red = balanceSelector->GetEntryByName("Red");
	if (!IsAvailable(red) || !IsReadable(red))
	{
		LOG("Unable to set red. Aborting...\n");
		return;
	}

	int64_t v = red->GetValue();

	balanceSelector->SetIntValue(v);


	CEnumerationPtr whiteauto = nodeMap.GetNode("BalanceWhiteAuto");
	CEnumEntryPtr whiteoff = whiteauto->GetEntryByName("Off");
	whiteauto->SetIntValue(whiteoff->GetValue());

	CFloatPtr valanceRatio = nodeMap.GetNode("BalanceRatio");
	if (!IsAvailable(valanceRatio) || !IsWritable(valanceRatio))
	{
		LOG("Unable to set balance ratio. Aborting...");
		return;
	}
	valanceRatio->SetValue(Blance);
	CEnumerationPtr gainauto = nodeMap.GetNode("GainAuto");
	CEnumEntryPtr gainoff = gainauto->GetEntryByName("Off");
	gainauto->SetIntValue(gainoff->GetValue());

	CFloatPtr gain = nodeMap.GetNode("Gain");
	if (!IsAvailable(gain) || !IsWritable(gain))
	{
		LOG("Unable to set gain. Aborting...");
		return;
	}
	gain->SetValue(Gain);

	CEnumerationPtr blacklevelselector = nodeMap.GetNode("BlackLevelSelector");
	CEnumEntryPtr all = blacklevelselector->GetEntryByName("All");
	blacklevelselector->SetIntValue(all->GetValue());

	CFloatPtr blacklevel = nodeMap.GetNode("BlackLevel");
	if (!IsAvailable(blacklevel) || !IsWritable(blacklevel))
	{
		LOG("Unable to set blacklevel. Aborting...");
		return;
	}
	blacklevel->SetValue(BlackLevel);
}
int PointGreyCapture::SetExposure(double exposureTimeToSet)
{
	INodeMap& nodeMap = camera_->GetNodeMap();
	int result = 0;

	LOG("*** CONFIGURING EXPOSURE ***\n");

	try
	{

		CEnumerationPtr ptrExposureMode = nodeMap.GetNode("ExposureMode");
		if (!IsAvailable(ptrExposureMode) || !IsWritable(ptrExposureMode))
		{
			LOG("Unable to set exposure mode  Aborting...");
			return -1;
		}

		CEnumEntryPtr ptrTimed = ptrExposureMode->GetEntryByName("Timed");
		if (!IsAvailable(ptrTimed) || !IsReadable(ptrTimed))
		{
			LOG("Unable to set exposure mode triggerwidth. Aborting...\n");
			return -1;
		}

		int64_t timed = ptrTimed->GetValue();

		ptrExposureMode->SetIntValue(timed);

		//
		// Turn off automatic exposure mode
		//
		// *** NOTES ***
		// Automatic exposure prevents the manual configuration of exposure
		// time and needs to be turned off.
		//
		// *** LATER ***
		// Exposure time can be set automatically or manually as needed. This
		// example turns automatic exposure off to set it manually and back
		// on in order to return the camera to its default state.
		//
		CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
		if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
		{
			LOG("Unable to disable automatic exposure (node retrieval). Aborting...");
			return -1;
		}

		CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
		if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
		{
			LOG("Unable to disable automatic exposure (enum entry retrieval). Aborting...");
			return -1;
		}

		ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());

		LOG("Automatic exposure disabled...");

		//
		// Set exposure time manually; exposure time recorded in microseconds
		//
		// *** NOTES ***
		// The node is checked for availability and writability prior to the
		// setting of the node. Further, it is ensured that the desired exposure
		// time does not exceed the maximum. Exposure time is counted in
		// microseconds. This information can be found out either by
		// retrieving the unit with the GetUnit() method or by checking SpinView.
		//
		CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
		if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
		{
			LOG("Unable to set exposure time. Aborting...");
			return -1;
		}

		// Ensure desired exposure time does not exceed the maximum
		const double exposureTimeMax = ptrExposureTime->GetMax();

		if (exposureTimeToSet > exposureTimeMax)
		{
			exposureTimeToSet = exposureTimeMax;
		}

		ptrExposureTime->SetValue(exposureTimeToSet);

		LOG("Exposure time set to %f\n", exposureTimeToSet);
	}
	catch (Spinnaker::Exception& e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int PointGreyCapture::PreviewCapture(QImage& img)
{
	int result = 0;

	INodeMap& nodeMap = camera_->GetNodeMap();

	result = result | GrabNextImageByTrigger(nodeMap, camera_);
	ImagePtr pResultImage = camera_->GetNextImage();
	ImageStatus imageStatus = pResultImage->GetImageStatus();
	if (imageStatus) {
		LOG("image has error!!!!!!\n");
	}

	ImagePtr convertedImage = pResultImage->Convert(PixelFormat_RGB8, HQ_LINEAR);
	uchar* data = (uchar*)convertedImage->GetData();
	int w = convertedImage->GetWidth();
	int h = convertedImage->GetHeight();
	int bps = convertedImage->GetBitsPerPixel();
	int stride = convertedImage->GetStride();
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			uchar* pdata = data + stride * y;
			if (rot_) img.setPixel(h - y - 1, x, qRgb(pdata[x * 3 + 0], pdata[x * 3 + 1], pdata[x * 3 + 2]));
			else img.setPixel(x, y, qRgb(pdata[x * 3 + 0], pdata[x * 3 + 1], pdata[x * 3 + 2]));
		}
	}

	pResultImage->Release();

	return result;
}
void PointGreyCapture::init_camera_buffers(int num)
{
	if (init_) return;
	INodeMap& nodeMap = camera_->GetNodeMap();

	int result = result | GrabNextImageByTrigger(nodeMap, camera_);
	ImagePtr pResultImage = camera_->GetNextImage();
	ImageStatus imageStatus = pResultImage->GetImageStatus();
	if (imageStatus) {
		LOG("image has error!!!!!!\n");
	}
	if (init_ == false) {
		init_ = true;
		format_ = pResultImage->GetPixelFormat();
		offset_x_ = pResultImage->GetXOffset();
		offset_y_ = pResultImage->GetXOffset();

		height_ = pResultImage->GetHeight();
		width_ = pResultImage->GetWidth();
	
		for (int i = 0; i < num; i++) {
			im_data_.push_back((unsigned char*)malloc(sizeof(unsigned char)*width_*height_));
			images_.push_back(Image::Create(width_, height_, offset_x_, offset_y_, PixelFormat_BayerRG8, im_data_[i]));
		}
		
	}
	pResultImage->Release();
}
int PointGreyCapture::CaptureSequence(int num_imgs)
{
	std::unique_lock<std::mutex> lck(mutex_);
	while (invoked_) {
		cond_.wait(lck);
	}
	//lck.lock();
	invoked_ = true;
	num_seq_ = num_imgs;
	lck.unlock();
	cond_.notify_all();
	return 1;
}

void PointGreyCapture::waitForCompletion()
{

	std::unique_lock<std::mutex> lck(mutex_);
	while (invoked_) {
		cond_.wait(lck);
	}
	lck.unlock();
}
void PointGreyCapture::Capture(int ith)
{
	INodeMap& nodeMap = camera_->GetNodeMap();
	int result = 0;
	result = result | GrabNextImageByTrigger(nodeMap, camera_);
	ImagePtr pResultImage = camera_->GetNextImage();

	
	memcpy(im_data_[ith], pResultImage->GetData(), pResultImage->GetWidth()*pResultImage->GetHeight() * sizeof(unsigned char));
	pResultImage->Release();
}
void PointGreyCapture::run()
{
	//thread_.detach();

	while (!stopped_) {
		std::unique_lock<std::mutex> lck(mutex_);


		while (!invoked_) {
			cond_.wait(lck);
		}

		lck.unlock();

		for (int i = 0; i < num_seq_; i++) {
			ImagePtr pResultImage = camera_->GetNextImage();

			memcpy(im_data_[i], pResultImage->GetData(), pResultImage->GetWidth()*pResultImage->GetHeight() * sizeof(unsigned char));
			pResultImage->Release();
		}
		SaveToDisk();
		lck.lock();
		invoked_ = false;
		
		lck.unlock();
		cond_.notify_all();
	}
}
void PointGreyCapture::start_camera()
{
	std::unique_lock<std::mutex> lck(mutex_);
	while (invoked_) {
		cond_.wait(lck);
	}
	lck.unlock();
	INodeMap& nodeMap = camera_->GetNodeMap();
	camera_->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
	camera_->BeginAcquisition();
	
	//cond_.notify_all();


}

void PointGreyCapture::end_camera()
{
	std::unique_lock<std::mutex> lck(mutex_);
	while (invoked_) {
		cond_.wait(lck);
	}
	//lck.lock();
	camera_->EndAcquisition();
	lck.unlock();
	cond_.notify_all();
}

graphics::ivec2 PointGreyCapture::resolution() const
{
	if (rot_) return graphics::ivec2(camera_->Height(), camera_->Width());
	return graphics::ivec2(camera_->Width(), camera_->Height());
}

PointGreyCapture::PointGreyCapture(Spinnaker::CameraPtr camptr, int cam_id, bool trigger)
	: camera_(camptr), trigger_(trigger), cam_id_(cam_id), invoked_(false), stopped_(false), mutex_(), cond_()
{
	rot_ = false;
	num_buffer_ = 22;
	init_ = false;

	thread_ = std::thread(std::bind(&PointGreyCapture::run, this));
}

PointGreyCapture::~PointGreyCapture()
{
	for (int i = 0; i < num_buffer_; i++) {
		free(im_data_[i]);
	}
	thread_.join();
}