#pragma once
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <graphics/ivec.h>

#include <mutex>
#include <QtGui/qimage.h>
class PointGreyCapture {
public:

	// This function retrieves a single image using the trigger. In this example,
	// only a single image is captured and made available for acquisition - as such,
	// attempting to acquire two images for a single trigger execution would cause
	// the example to hang. This is different from other examples, whereby a
	// constant stream of images are being captured and made available for image
	// acquisition.
	int GrabNextImageByTrigger(Spinnaker::GenApi::INodeMap& nodeMap, Spinnaker::CameraPtr pCam);

	void SetExposureTriggerWidth();
	
	int ConfigureTrigger(bool hardware);
	int SetExposure(double val);
	void SetMisc(double Blance, double Gain, double BlackLevel);
	// This function acts as the body of the example; please see NodeMapInfo example
	// for more in-depth comments on setting up cameras.
	int PreviewCapture(QImage& img);
	void Capture(int ith);
	void init_camera_buffers(int num);
	int CaptureSequence(int num_imgs);

	void SaveToDisk();
	void start_camera();
	void end_camera();
	void waitForCompletion();
	void run();
	void set_rot(bool val) { rot_ = val; }
	graphics::ivec2 resolution() const;
public:
	explicit PointGreyCapture(Spinnaker::CameraPtr camptr, int cam_id, bool trigger);
	~PointGreyCapture();

private:

	bool trigger_;
	bool init_;
	Spinnaker::CameraPtr camera_;
	int cam_id_;

	std::mutex mutex_;
	std::condition_variable cond_;
	volatile bool invoked_;
	volatile bool stopped_;
	std::thread thread_;
	int num_seq_;
	int ith_image_;
	int num_buffer_;
	std::vector<unsigned char*> im_data_;
	std::vector< Spinnaker::ImagePtr> images_;
	Spinnaker::PixelFormatEnums format_;
	int offset_x_;
	int offset_y_;
	int width_;
	int height_;
	bool rot_;
};