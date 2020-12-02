#include "ProjectorScan.h"
/*
#include <user/Row.h>
#include <kernel/NodeWidget.h>
#include <user/UserKernel.h>
#include <knobs/Int_Knob.h>
#include <user/KoonMath.h>
*/
#include <graphics/sys.h>
#include <graphics/gl.h>
#include <graphics/gl_extension.h>
#include <graphics/frame.h>
#include <graphics/Timer.h>
// #include <user/ViewerContext.h>
#include <QtWidgets/qdesktopwidget.h>
//#include <user/UserMaterial.h>
//#include <user/UserKernel.h>
//#include <user/DiskCache.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <libmv/prec_lib/geometric_resection.h>
#include "libmv/camera/pinhole_camera.h"
#include "libmv/multiview/robust_euclidean_resection.h"
#include "libmv/multiview/bundle.h"
#include "libmv/correspondence/gaussian_pyramid.h"
#include "graphics/image_sample.h"
#include "graphics/seed_fill.h"
#include "libmv/tsai/camera_calibration.h"
#include "graphics/barycentric.h"
//#include "neural_net/NNLayer.h"
//#include "user_app/NNFace.h"

#include <random>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;


namespace user{

static int kStart_y = 960;
static int kEnd_y = 1320;
bool ProjectorScan::initialized_;
// KImageDisplayDialog* ProjectorScan::device_;


int ProjectorScan::ConfigureTrigger(INodeMap& nodeMap, bool hardware)
{
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
		if (!global_shutter_) {
			CEnumerationPtr sensor = nodeMap.GetNode("SensorShutterMode");
			if (!IsAvailable(sensor) || !IsReadable(sensor))
			{
				LOG("Unable to set sensor shutter mode (node retrieval). Aborting...\n");
				return -1;
			}
			CEnumEntryPtr greset = sensor->GetEntryByName("GlobalReset");
			sensor->SetIntValue(greset->GetValue());
		}
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
int ProjectorScan::ResetTrigger(INodeMap& nodeMap)
{
	int result = 0;

	try
	{
		CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
		if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
		{
			return -1;
		}

		CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
		if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
		{
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());


	}
	catch (Spinnaker::Exception& e)
	{
		result = -1;
	}

	return result;
}
struct cam_id_struct {
	CameraPtr cam;
	size_t id;
	cam_id_struct() {}
	cam_id_struct(CameraPtr pcam, size_t pid): cam(pcam), id(pid) {}
	cam_id_struct(const cam_id_struct& a): cam(a.cam), id(a.id) {}
};
bool cam_id_compare(cam_id_struct& i, cam_id_struct& j) { return (i.id < j.id); }

void ProjectorScan::initCameraSystem()
{
	camera_system_ = System::GetInstance();
	CameraList camList = camera_system_->GetCameras();
	unsigned int numCameras = camList.GetSize();
	std::vector<cam_id_struct> cameras;
	for (unsigned int i = 0; i < numCameras; i++)
	{
		CameraPtr pCam = camList.GetByIndex(i);
		pCam->Init();
		// Retrieve GenICam nodemap
		INodeMap& nodeMap = pCam->GetNodeMap();
		INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

		// Retrieve device serial number for filename
		CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
		size_t val = std::stoull(std::string(ptrStringSerial->GetValue()));
		cameras.push_back(cam_id_struct(pCam, val));
		std::string serialNumber(ptrStringSerial->GetValue()); //= ptrStringSerial->GetValue();

		LOG("device serial number %s\n", serialNumber.c_str());
	}
	std::sort(cameras.begin(), cameras.end(), cam_id_compare);
	for (int i = 0; i < cameras.size(); i++) {
		camList_.push_back(cameras[i].cam);
	}

	for (int i = 0; i < camList_.size(); i++) {
		CameraPtr pCam = camList_[i];

		INodeMap& nodeMap = pCam->GetNodeMap();
		ConfigureTrigger(nodeMap, i== 0?false:true);		
		cameras_.push_back(new PointGreyCapture(camList_[/*camList_.size()-1-*/i], i, i == 0 ? false : true));
	}	
	for (int i = 0; i < cameras_.size(); i++) {
		cameras_[i]->set_rot(true);
	}
	num_camera_ = cameras.size();
	if (num_camera_ == 0) num_camera_ = 2;
}
void ProjectorScan::DeinitCamera(Spinnaker::CameraPtr pCam)
{
	pCam->EndAcquisition();

	INodeMap& nodeMap = pCam->GetNodeMap();
	// Reset trigger
	ResetTrigger(nodeMap);

	// Deinitialize camera
	pCam->DeInit();
}


void ProjectorScan::ReleaseCameraSystem()
{

	for (unsigned int i = 0; i < camList_.size(); i++)
	{
		DeinitCamera(camList_[i]);
	}
	for (int i = 0; i < cameras_.size(); i++)
		delete cameras_[i];
	cameras_.clear();
	camList_.clear();
	camera_system_->ReleaseInstance();
}

void ProjectorScan::initialize()
{
	if (initialized_) {
		return;
	}
	
	graphics::glExtensionInitialize();

	glEnable(GL_TEXTURE_2D);
	initialized_ = true;

	int tw = 2;
	for (int i = 0; i < 100; i++) {
		tw *= 2;
		if (tw >= proj_width_) break;
	}
	int cur_half = tw / 2;
	std::vector<int> pattern(tw, 1);
	level_images_.push_back(std::vector<int>(tw, 1)); 
	level_images_.push_back(std::vector<int>(tw, 0));// reference BLACK
	for (int i = 0; i < 50; i++) {
		int j = 0;
		while (true) {
			bool out = false;
			for (int k = 0; k < cur_half; k++) {
				if (j*cur_half + k >= proj_width_) {
					out = true;
					break;
				}
				pattern[j*cur_half + k] = ((j / 2) % 2 == 0) ? (j % 2 ? 0 : 1) : (j % 2 ? 1 : 0);
			}
			if (out) break;
			j++;
		}
		level_images_.push_back(pattern);
		cur_half /= 2;
		if (cur_half == 4) break;
	}

	

	int level_cnt = level_images_.size();
	LOG("level image count %d\n", level_images_.size());
	for (int j = 0; j < 8; j++) {
		for (int i = 0; i < tw; i++) {
			pattern[i] = ((i + 8 - j) / 4) % 2 ? 0 : 1;
		}
		level_images_.push_back(pattern);
	}
	inverse_1_.resize(tw);
	inverse_2_.resize(tw);
	int j = 0;
	while (true) {
		bool out = false;
		for (int k = 0; k < 8; k++) {
			if (j * 8 + k >= proj_width_) {
				out = true;
				break;
			}
			inverse_1_[j * 8 + k] = (j % 2 == 0) ? 1 : 0;
			inverse_2_[j * 8 + k] = (j % 2 == 0) ? 0 : 1;
		}
		if (out) break;
		j++;
	}
	level_images_.push_back(inverse_1_);
	level_images_.push_back(inverse_2_);
	//level_images_.push_back(inverse_2_);
	LOG("total image count %d, shift pattern count %d\n", level_images_.size(), level_images_.size() - level_cnt);
	initOutBuffer();
}

ProjectorScan::~ProjectorScan()
{

	free(camera_buffer_);
	free(invmap1x);
	free(invmap1y);
	free(invmap2x);
	free(invmap2y);
	ReleaseCameraSystem();
}

ProjectorScan::ProjectorScan(void) // UserNode* node) : Iop (node)
{
	mode_ = Preview;
	preview_count_ = 0;
	global_shutter_ = true;
	proj_stat_ = -1;
	context_ = KLTCreateTrackingContext();
	context_->subsampling = 2;
	context_->smooth_sigma_fact = 0.1;
	filter_win_ = 4;
	initCameraSystem();
	count_ = 100;
	exposure_ = 8000;
	graphics::ivec2 resolution(2448, 2048);
	if (cameras_.size()) 
		resolution = cameras_[0]->resolution();

	proj_width_ = 1920;
	proj_height_ = 1080;
	
	cam_width_ = resolution[0];
	cam_height_ = resolution[1] ;
	LOG("camera resolution %d, %d\n", cam_width_, cam_height_);
	preview_height_ = cam_height_ /3; // preview height
	preview_width_ = cam_width_/3 * cameras_.size(); 
	//info_.format(*UserKernel::Instance()->CreateFormat("Cameras Preview", preview_width_, preview_height_, 1.0));
	// inputs(0);
	camera_buffer_ = 0;


	invmap1x = 0;
	invmap1y = 0;
	invmap2x = 0; 
	invmap2y = 0;

	x_offset_ = 31.4; // cm
	y_offset_ = -4.2;
	z_offset_ = -16.9;
	board_size_w_ = 42;
	board_size_h_ = 26;

	preview_timer_.start();
	initCameraProjectorGeometry();
}

/* 
bool ProjectorScan::test_input(int in, Op* op) const
{
	if (in == 0) {
		return (dynamic_cast<NNLayer*>(op) != 0);
	}

	return false;
}
*/

void 
ProjectorScan::initCameraProjectorGeometry()
{
	for (int i = 0; i < num_camera_; i++) {
		char name[100];
		sprintf(name, "scanner\\pt_grey_%d.cal", i);
		int w, h;
		cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

		FILE* fp = fopen(name, "r");
		if (!fp) continue;


		fscanf(fp, "%d %d\n", &w, &h);
		fscanf(fp, "%lf %lf\n", &cameraMatrix.at<double>(0, 2), &cameraMatrix.at<double>(1, 2));
		fscanf(fp, "%lf %lf\n", &cameraMatrix.at<double>(0, 0), &cameraMatrix.at<double>(1, 1));
		fscanf(fp, "%lf %lf %lf %lf %lf %lf\n", &distCoeffs.at<double>(0, 0), &distCoeffs.at<double>(1, 0),
			&distCoeffs.at<double>(4, 0), &distCoeffs.at<double>(5, 0), &distCoeffs.at<double>(6, 0),
			&distCoeffs.at<double>(7, 0));
		fclose(fp);

		matrix3x3 K_;
		K_.makeIdentity();

		matrix3x3 R;
		vec3 t;
		K_(0, 2) = cameraMatrix.at<double>(0, 2);
		K_(1, 2) = cameraMatrix.at<double>(1, 2);
		K_(0, 0) = K_(1, 1) = cameraMatrix.at<double>(0, 0);
		sprintf(name, "scanner\\camera%d.txt", i);
		fp = fopen(name, "r");
		if (!fp) continue;
		fscanf(fp, "%Lf %Lf %Lf\n %Lf %Lf %Lf\n %Lf %Lf %Lf\n", &R(0, 0), &R(0, 1), &R(0, 2), 
			&R(1, 0), &R(1, 1), &R(1, 2), &R(2, 0), &R(2, 1), &R(2, 2));
		fscanf(fp, "%Lf %Lf %Lf\n", &t[0], &t[1], &t[2]);
		fclose(fp);
		matrix3x3 k_inv = K_.inverse();
		pinhole_cameras_[i].SetIntrinsicExtrinsicParameters(K_, R, t);
		pinhole_cameras_[i].set_image_size(vec2(w, h));
	}
	int w, h;
	matrix3x3 K;
	K.makeIdentity();
	matrix3x3 R;
	vec3 t;
	FILE* fp = fopen("scanner\\projector.dat", "rb");
	if (!fp) return;
	fread(&w, sizeof(int), 1, fp);
	fread(&h, sizeof(int), 1, fp);
	fread(&K(0, 2), sizeof(double), 1, fp);
	fread(&K(1, 2), sizeof(double), 1, fp);
	fread(&K(0, 0), sizeof(double), 1, fp);
	fread(&K(1, 1), sizeof(double), 1, fp);
	fread(&R(0, 0), sizeof(double), 1, fp);
	fread(&R(0, 1), sizeof(double), 1, fp);
	fread(&R(0, 2), sizeof(double), 1, fp);
	fread(&R(1, 0), sizeof(double), 1, fp);
	fread(&R(1, 1), sizeof(double), 1, fp);
	fread(&R(1, 2), sizeof(double), 1, fp);
	fread(&R(2, 0), sizeof(double), 1, fp);
	fread(&R(2, 1), sizeof(double), 1, fp);
	fread(&R(2, 2), sizeof(double), 1, fp);
	fread(&t[0], sizeof(double), 1, fp);
	fread(&t[1], sizeof(double), 1, fp);
	fread(&t[2], sizeof(double), 1, fp);
	fclose(fp);
	projector_.SetIntrinsicProjection(K, R, t);
	projector_.set_image_size(vec2(w, h));
}
//ProjectorScan::ProjectorScan() : Iop(0)
//{

//}

void ProjectorScan::initOutBuffer()
{

	//camera_buffer_ = (unsigned char*)malloc(info_.w()*info_.h()*3 * cameras_.size());
	// ksk : this is a preview width and height
	camera_buffer_ = (unsigned char*)malloc(1920 * 1080 * 3 * cameras_.size());

	proj_buffer_ = (unsigned char*)malloc(proj_width_*proj_height_ * 3 * level_images_.size());
	for (int i = 0; i < level_images_.size(); i++)
	for (int x = 0; x < proj_width_; x++) {
		for (int y = 0; y < proj_height_; y++) {
			proj_buffer_[i*proj_width_*proj_height_ * 3 + y * proj_width_ * 3 + x * 3 + 0] = level_images_[i][x] * 255;
			proj_buffer_[i*proj_width_*proj_height_ * 3 + y * proj_width_ * 3 + x * 3 + 1] = level_images_[i][x] * 255;
			proj_buffer_[i*proj_width_*proj_height_ * 3 + y * proj_width_ * 3 + x * 3 + 2] = level_images_[i][x] * 255;
		}
	}
}
void ProjectorScan::project_pattern(int ith)
{
	//if (device_) {
		QImage im(&proj_buffer_[ith*proj_width_*proj_height_ * 3], proj_width_, proj_height_, QImage::Format_RGB888);
		//device_->set_image(&im);

	//}

}

/* 
void ProjectorScan::_validate(bool for_real) {
	LOG("validate called\n");
	info_.channels(Mask_RGBA);
	info_.set(format());
	set_out_channels(info_.channels());
	initialize();
	if (!device_) {
		QDesktopWidget* desktop = QApplication::desktop();
		if (desktop->screenCount() > 1) {
			LOG("connected screen %d\n", desktop->screenCount());
			device_ = new KImageDisplayDialog(UserKernel::Instance()->GetApplicationMainWidget(), UserKernel::Instance()->GetMainViewWidget());
			QRect qrect = desktop->screenGeometry(1);
			device_->setGeometry(qrect);
			device_->showFullScreen();
		}
		else if (desktop->screenCount() == 1) {
			device_ = new KImageDisplayDialog(UserKernel::Instance()->GetApplicationMainWidget(), UserKernel::Instance()->GetMainViewWidget());
			device_->setGeometry(0, 0, proj_width_, 200);
			device_->show();
		}
		QGLWidget* widget = (QGLWidget*)UserKernel::Instance()->GetMainViewWidget();
		widget->makeCurrent();
	}
	if (mode_ == Capture || mode_ == Calibrate) {
		scan();
	}
	else if (mode_ == Preview) {
		DiskCache dk(hash().value());
		if (dk.read()) {
			memcpy(camera_buffer_, dk.image_, dk.w_*dk.h_*dk.n_channels_);
			free(dk.image_);

		}
		else {
			preview();
			dk.write(preview_width_, preview_height_, 3, camera_buffer_);
		}

		//preview();
	}


}
*/

//void ProjectorScan::draw_handle(ViewerContext* context)
//{
//}

/*
void ProjectorScan::append(user::Hash& a)
{

	double t = preview_timer_.getElapsedTime();
	if (t > 1.0) {
		preview_count_++;
		preview_timer_.start();
	}
	a.append(preview_count_);

}
*/

void ProjectorScan::preview()
{
	project_pattern(0);

	for (auto i : cameras_) {
		i->start_camera();
	} 

	for (int i = 0; i < cameras_.size(); i++) {
		QImage img(cam_width_, cam_height_, QImage::Format::Format_RGB888);
		cameras_[i]->init_camera_buffers(21);
		cameras_[i]->PreviewCapture(img);
		img.save("preview_output.bmp");
		int offset = i * cam_width_/3;

		for (int x = 0; x < cam_width_/3; x++)
			for (int y = 0; y < cam_height_/3; y++) {
				QRgb a = img.pixel(x*3, y*3);
				int yy = preview_height_ - y - 1;
				yy = yy < 0 ? 0 : (yy >= preview_height_ ? preview_height_ - 1 : yy);
				camera_buffer_[(offset + x) * 3 + yy * preview_width_ * 3 + 0] = qRed(a);
				camera_buffer_[(offset + x) * 3 + yy * preview_width_ * 3 + 1] = qGreen(a);
				camera_buffer_[(offset + x) * 3 + yy * preview_width_ * 3 + 2] = qBlue(a);
			}
	}
	
	
	for (auto i : cameras_) {
		i->end_camera();
	}
}

static void calcBoardCornerPositions( cv::Size boardSize, float squareSize, float y_offset, float z_offset, std::vector<cv::Point3f>& corners)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(cv::Point3f(float(j*squareSize), float(i*squareSize), 0));

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(cv::Point3f(float(j*squareSize), y_offset, z_offset + float(i*squareSize)));
}
static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, float y_offset, float z_offset, std::vector<std::vector<cv::Point3f> >& corners)
{
	corners.clear();
	corners.resize(2);
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners[0].push_back(cv::Point3f(float(j*squareSize), float(i*squareSize), 0));

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners[1].push_back(cv::Point3f(float(j*squareSize), float(i*squareSize), 0));
}

inline bool has_this(std::vector<vec2>& s, vec2&b)
{
	for (auto&i : s) {
		if (apx_equal(i, b)) {
			return true;
		}
	}
	return false;
}
#ifndef min
#define min(a,b) (a>b?b:a)
#endif
// ksk
#ifndef max
#define max(a,b) (a>b?a:b)
#endif

void ProjectorScan::compute_projector_plane()
{
	int imsize = cam_width_ * cam_height_;

	float* proj_map = (float*)malloc(cam_width_*cam_height_ * sizeof(float));

	for (int i = 0; i < imsize; i++) proj_map[i] = -1;
	std::vector<QImage*> imgs;
	load_images(0, imgs);
	QImage& black = *imgs[1];
	QImage& white = *imgs[0];

	compute_grey_code(imgs, proj_map, 35.0, 245.0);

	std::vector<vec3> xz_boards(4);
	xz_boards[0] = vec3(0);
	xz_boards[1] = vec3(board_size_h_, 0, 0);
	xz_boards[2] = vec3(board_size_h_, 0, board_size_w_);
	xz_boards[3] = vec3(0, 0, board_size_w_);
	std::vector<vec3> yz_boards(4);
	yz_boards[0] = vec3(0);
	yz_boards[1] = vec3(0, board_size_h_, 0);
	yz_boards[2] = vec3(0, board_size_h_, board_size_w_);
	yz_boards[3] = vec3(0, 0, board_size_w_);

	std::vector<vec2> xz_boards_2d(4);
	std::vector<vec2> yz_boards_2d(4);

	for (int i = 0; i < 4; i++) {
		xz_boards[i] = board_geom_.to_world(xz_boards[i]);
		xz_boards_2d[i] = pinhole_cameras_[0].project_point(xz_boards[i]);
		yz_boards[i] = board_geom_.to_world(yz_boards[i]);
		yz_boards_2d[i] = pinhole_cameras_[0].project_point(yz_boards[i]);
	}
	plane xz_plane(xz_boards[0], xz_boards[1], xz_boards[2]);
	plane yz_plane(yz_boards[0], yz_boards[1], yz_boards[2]);

	float min_v = 1000000;
	float max_v = 0;
	std::map<int, std::vector<graphics::vec2>> xz_points;
	std::map<int, std::vector<graphics::vec2>> yz_points;
	for (int x = 0; x < cam_width_; x++)
	{
		for (int y = 0; y < cam_height_; y++)
		{
			graphics::vec2 pnt(x, y);
			float v = proj_map[x + y * cam_width_];
			if (v < 2 || v > proj_width_) continue;
			if (graphics::kai_contain(xz_boards_2d, pnt)) {
				xz_points[v].push_back(pnt);
				max_v = max(max_v, v); // ksk std::max( max_v, v);
				min_v = min(min_v, v);
			}
			else if (graphics::kai_contain(yz_boards_2d, pnt)) {
				yz_points[v].push_back(pnt);
				max_v = max(max_v, v); // std::max( max_v, v);
				min_v = min(min_v, v);
			}
		}
	}
	int inc_count = 0;
	int max_count = 0;
	float small_v = min_v, large_v = max_v;
	int i = min_v;
	while (true) {
		int crv = xz_points[i].size() + yz_points[i].size();
		if (crv > 500 && crv > max_count) {
			max_count = crv;
			small_v = i;
		}
		if (crv > 500) inc_count++;
		if (inc_count > 40) break;
		i++;
	}
	max_count = 0;
	inc_count = 0;
	i = max_v;
	while (true) {
		int crv = xz_points[i].size() + yz_points[i].size();
		if (crv > 500 && crv > max_count) {
			max_count = crv;
			large_v = i;
		}
		if (crv > 500) inc_count++;
		if (inc_count > 40) break;
		i--;
	}
	std::vector<graphics::vec3> plane_points1, plane_points2;
	std::vector<graphics::vec2>& pnt_set = xz_points[small_v];
	for (int i = 0; i < pnt_set.size(); i++) {
		graphics::line ll = pinhole_cameras_[0].ray(pnt_set[i]);
		ll = pinhole_cameras_[0].to_world(ll);
		vec3 pnt;
		if (xz_plane.intersect(ll, pnt)) {
			plane_points1.push_back(pnt);
		}
	}
	pnt_set = yz_points[small_v];
	for (int i = 0; i < pnt_set.size(); i++) {
		graphics::line ll = pinhole_cameras_[0].ray(pnt_set[i]);
		ll = pinhole_cameras_[0].to_world(ll);
		vec3 pnt;
		if (yz_plane.intersect(ll, pnt)) {
			plane_points1.push_back(pnt);
		}
	}
	pnt_set = xz_points[large_v];
	for (int i = 0; i < pnt_set.size(); i++) {
		graphics::line ll = pinhole_cameras_[0].ray(pnt_set[i]);
		ll = pinhole_cameras_[0].to_world(ll);
		vec3 pnt;
		if (xz_plane.intersect(ll, pnt)) {
			plane_points2.push_back(pnt);
		}
	}
	pnt_set = yz_points[large_v];
	for (int i = 0; i < pnt_set.size(); i++) {
		graphics::line ll = pinhole_cameras_[0].ray(pnt_set[i]);
		ll = pinhole_cameras_[0].to_world(ll);
		vec3 pnt;
		if (yz_plane.intersect(ll, pnt)) {
			plane_points2.push_back(pnt);
		}
	}

	graphics::plane projector_plane1 = ransac_plane_as_number(plane_points1, 1.0);
	graphics::plane projector_plane2 = ransac_plane_as_number(plane_points2, 1.0);

	line intersect_line;
	projector_plane1.intersect(projector_plane2, intersect_line);
	vec3 plane_pnt1 = projector_plane1.d * projector_plane1.n;
	vec3 dir = plane_points1[0] - plane_pnt1;
	plane_pnt1 = plane_points1[0] - inner(projector_plane1.n, dir) * projector_plane1.n;
	vec3 plane_pnt2 = projector_plane2.d * projector_plane2.n;
	dir = plane_points2[0] - plane_pnt2;
	plane_pnt2 = plane_points2[0] - inner(projector_plane2.n, dir) * projector_plane2.n;

	vec3 axis_pnt = intersect_line.get_closest_point(plane_pnt1);
	vec3 dir1 = unit(plane_pnt1 - axis_pnt);
	axis_pnt = intersect_line.get_closest_point(plane_pnt2);
	vec3 dir2 = unit(plane_pnt2 - axis_pnt);
	vec3 rot_axis = unit(cross(dir1, dir2));
	real ang = degree(angle(dir1, dir2));
	real step = ang / real(large_v - small_v);
	quater q = orient(radian(step*-real(small_v)), rot_axis);
	vec3 out_dir = rot(q, dir1);
	graphics::frame cam_frame = pinhole_cameras_[0].get_camera_frame();
	vec3 pos = intersect_line.get_position();
	pos = cam_frame.to_model(pos);
	rot_axis = cam_frame.to_model_normal(rot_axis);
	out_dir = cam_frame.to_model_normal(out_dir);

	

	double focal_length = (1.0 / tan(radian((step*proj_width_) / 2.0)))*(proj_width_ / 2.0);
	float cx = proj_width_ * 0.5;
	float cy = proj_height_ * 0.5;
	matrix3x3 K(focal_length, 0, cx,
		0, focal_length, cy,
		0, 0, 1.0);

	vec3 y_dir = rot_axis;
	q = orient(radian((step*proj_width_) / 2.0), rot_axis);
	vec3 z_dir = unit(rot(q, out_dir));
	vec3 x_dir = cross(y_dir, z_dir);
	matrix3x3 orientation_mat(x_dir[0], x_dir[1], x_dir[2],
		y_dir[0], y_dir[1], y_dir[2],
		z_dir[0], z_dir[1], z_dir[2]);

	libmv::PinholeCamera projector(K, orientation_mat, pos);
	//vec2 ppp = projector.project_point(world_points[0]);
	vec3 t;
	projector.GetIntrinsicProjection(&K, &orientation_mat, &t);

	// now random sampling
	auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
	std::mt19937 random_gen_(seed);

	vec3 xz_n = cam_frame.to_model_normal(xz_plane.n);
	vec3 xz_p = cam_frame.to_model(xz_plane.n * xz_plane.get_dist());
	plane xz_plane2(xz_n, xz_p);

	vec3 yz_n = cam_frame.to_model_normal(yz_plane.n);
	vec3 yz_p = cam_frame.to_model(yz_plane.n * yz_plane.get_dist());
	plane yz_plane2(yz_n, yz_p);

	std::vector<vec2> space_points2;
	std::vector<vec3> world_points2;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	while (true) {
		double x = distribution(random_gen_);
		double y = distribution(random_gen_);
		vec2 pos(x*(cam_width_ - 1), y*(cam_height_ - 1));
		int xx = pos[0] + 0.5;
		int yy = pos[1] + 0.5;
		float v = proj_map[xx + yy * cam_width_];
		if (v < 2.0) continue;

		// ksk
		vec2 v2(v, xx);

		if (has_this(space_points2, v2)) continue; // ksk  vec2(v, xx))) continue;

		if (graphics::kai_contain(xz_boards_2d, pos)) {
			line ray = pinhole_cameras_[0].ray(pos);
			vec3 ipoint;
			if (xz_plane2.intersect(ray, ipoint)) {
				space_points2.push_back(vec2(v, xx));
				world_points2.push_back(ipoint);
			}
			continue;
		}
		if (graphics::kai_contain(yz_boards_2d, pos)) {
			line ray = pinhole_cameras_[0].ray(pos);
			vec3 ipoint;
			if (yz_plane2.intersect(ray, ipoint)) {
				space_points2.push_back(vec2(v, xx));
				world_points2.push_back(ipoint);
			}
			continue;
		}
		if (world_points2.size() > 10000) break;
	}

	// optimization
	real rms = libmv::EuclideanBA_Stripe_Projector_X(space_points2,
		K,
		orientation_mat,
		t,
		world_points2);
	matrix3x3 R = orientation_mat;

	FILE* fp = fopen("scanner\\projector.dat", "wb");
	fwrite(&proj_width_, sizeof(int), 1, fp);
	fwrite(&proj_height_, sizeof(int), 1, fp);
	fwrite(&K(0, 2), sizeof(double), 1, fp);
	fwrite(&K(1, 2), sizeof(double), 1, fp);
	fwrite(&K(0, 0), sizeof(double), 1, fp);
	fwrite(&K(1, 1), sizeof(double), 1, fp);
	fwrite(&R(0, 0), sizeof(double), 1, fp);
	fwrite(&R(0, 1), sizeof(double), 1, fp);
	fwrite(&R(0, 2), sizeof(double), 1, fp);
	fwrite(&R(1, 0), sizeof(double), 1, fp);
	fwrite(&R(1, 1), sizeof(double), 1, fp);
	fwrite(&R(1, 2), sizeof(double), 1, fp);
	fwrite(&R(2, 0), sizeof(double), 1, fp);
	fwrite(&R(2, 1), sizeof(double), 1, fp);
	fwrite(&R(2, 2), sizeof(double), 1, fp);
	fwrite(&t[0], sizeof(double), 1, fp);
	fwrite(&t[1], sizeof(double), 1, fp);
	fwrite(&t[2], sizeof(double), 1, fp);
	fclose(fp);

}


void ProjectorScan::do_calibration()
{
	for (int ith_cam = 0; ith_cam < num_camera_; ith_cam++) {
		std::vector<std::vector<cv::Point2f> > imagePoints;
		std::vector<std::vector<cv::Point3f> > objectPoints(1);

		std::vector<QImage*> imgs;
		load_images(ith_cam, imgs);

		QImage& src = *imgs[0];

		src.load("preview_output.bmp");

		cv::Mat view = cv::Mat::zeros(cam_height_, cam_width_, CV_8UC3);

		for (int y = 0; y < cam_height_; y++) {
			for (int x = 0; x < cam_width_; x++) {
				QRgb rgb = src.pixel(x, y);
				view.at<cv::Vec3b>(y, x) = cv::Vec3b(qBlue(rgb), qGreen(rgb), qRed(rgb));
			}
		}

		std::vector<cv::Point2f> pointBuf;
		cv::Size imageSize = view.size();
		bool found1 = false, found2 = false;

		found1 = findChessboardCorners(view, cv::Size(7, 5), pointBuf,
		cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		
		std::vector<cv::Point2f> pointBuf2;
		if (found1) {
			for (int i = 0; i < pointBuf.size(); i++) {
				for (int x = 0 ; x < 200; x++)
					for (int y = 0; y < 200; y++) {
						int xx = pointBuf[i].x + x - 100;
						int yy = pointBuf[i].y + y - 100;
						if (xx >= 0 && xx < cam_width_ && yy >= 0 && yy < cam_height_)
							view.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 255, 255);
					}
			}
		}
		found2 = findChessboardCorners(view, cv::Size(7, 5), pointBuf2,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		
		for (int y = 0; y < cam_height_; y++) {
			for (int x = 0; x < cam_width_; x++) {
				QRgb rgb = src.pixel(x, y);
				view.at<cv::Vec3b>(y, x) = cv::Vec3b(qBlue(rgb), qGreen(rgb), qRed(rgb));
			}
		}

		if (found1 && found2) {
			if (pointBuf[0].x < pointBuf[pointBuf.size() - 1].x) { // 카메라 기울어질땐 x
				std::vector<cv::Point2f> tmp = pointBuf;
				for (int i = 0; i < pointBuf.size(); i++) {
					pointBuf[i] = tmp[int(pointBuf.size()) - 1 - i];
				}
			}
			if (pointBuf2[0].x < pointBuf2[pointBuf2.size() - 1].x) { // 카메라 기울어질땐 x
				std::vector<cv::Point2f> tmp = pointBuf2;
				for (int i = 0; i < pointBuf2.size(); i++) {
					pointBuf2[i] = tmp[int(pointBuf2.size()) - 1 - i];
				}
			}
			if (pointBuf[0].x > pointBuf2[0].x) {					// // 카메라 기울어질땐 x
				std::vector<cv::Point2f> tmp = pointBuf2;
				pointBuf2 = pointBuf;
				pointBuf = tmp;

			}
		}
		if (found1 && found2)                // If done with success,
		{
			//opencv calibration
			LOG("image %d success finding corner\n", ith_cam);

			for (int y = 0; y < cam_height_; y++) {
				for (int x = 0; x < cam_width_; x++) {
					QRgb rgb = src.pixel(x, y);
					view.at<cv::Vec3b>(y, x) = cv::Vec3b(qBlue(rgb), qGreen(rgb), qRed(rgb));
				}
			}
			
			cv::Mat viewGray;
			cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
			cv::cornerSubPix(viewGray, pointBuf, cv::Size(11, 11),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			cv::cornerSubPix(viewGray, pointBuf2, cv::Size(11, 11),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

			imagePoints.push_back(pointBuf);
			imagePoints.push_back(pointBuf2);

			//cv::drawChessboardCorners(view, cv::Size(4, 11), cv::Mat(pointBuf), found);
			cv::drawChessboardCorners(view, cv::Size(7, 5), cv::Mat(pointBuf), found1);
			cv::drawChessboardCorners(view, cv::Size(7, 5), cv::Mat(pointBuf2), found2);
			char out_fname[100];
			sprintf(out_fname, "out%02d.jpg", ith_cam);
			cv::imwrite(out_fname, view);
			int val = cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO;
			val |= cv::CALIB_FIX_K1;
			val |= cv::CALIB_FIX_K2;
			val |= cv::CALIB_FIX_K3;
			val |= cv::CALIB_FIX_K4;
			val |= cv::CALIB_FIX_K5;
			val |= cv::CALIB_FIX_K6;
			val |= cv::CALIB_ZERO_TANGENT_DIST;

			//calcBoardCornerPositions(cv::Size(4, 11), 2.0, objectPoints[0]);
			calcBoardCornerPositions(cv::Size(7, 5), 3.0, y_offset_, z_offset_, objectPoints);
			cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
			std::vector<cv::Mat> rvecs, tvecs;
			std::vector<float> reprojErrs;
			double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
				distCoeffs, rvecs, tvecs, val);

			LOG("rms %lf\n", rms);
			matrix3x3 R;
			vec3 t;
			cv::Mat out;
			cv::Rodrigues(rvecs[0], out);

			for (int x = 0; x < 3; x++) {
				t[x] = tvecs[0].at<double>(x);

				for (int y = 0; y < 3; y++) {
					R[x][y] = out.at<double>(x, y);
				}
			}
			t = R.inverse()*t*-1;

			libmv::PinholeCamera  cam;
			cam.SetIntrinsicParameters(cameraMatrix.at<double>(0, 0), vec2(cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2)));
			cam.SetExtrinsicParameters(R, t);
			cam.set_image_size(ivec2(cam_width_, cam_height_));
			pinhole_cameras_[ith_cam] = cam;

			char name[100];
			sprintf(name, "scanner\\pt_grey_%d.cal", ith_cam);
			FILE* fp = fopen(name, "w");
			fprintf(fp, "%d %d\n", (int)cam.image_width(), (int)cam.image_height());
			fprintf(fp, "%lf %lf\n", cam.intrinsic_matrix()(0, 2), cam.intrinsic_matrix()(1, 2));
			fprintf(fp, "%lf %lf\n", cam.intrinsic_matrix()(0, 0), cam.intrinsic_matrix()(1, 1));
			fprintf(fp, "%Lf %Lf %lf %lf %lf %lf\n", distCoeffs.at<double>(0, 0), 0,
				0, 0, 0,
				0);
			fclose(fp);

			sprintf(name, "scanner\\camera%d.txt", ith_cam);
			fp = fopen(name, "w");
			fprintf(fp, "%Lf %Lf %Lf\n %Lf %Lf %Lf\n %Lf %Lf %Lf\n",
				cam.orientation_matrix()(0, 0),
				cam.orientation_matrix()(0, 1),
				cam.orientation_matrix()(0, 2),
				cam.orientation_matrix()(1, 0),
				cam.orientation_matrix()(1, 1),
				cam.orientation_matrix()(1, 2),
				cam.orientation_matrix()(2, 0),
				cam.orientation_matrix()(2, 1),
				cam.orientation_matrix()(2, 2));
			fprintf(fp, "%Lf %Lf %Lf\n", cam.position()[0], cam.position()[1], cam.position()[2]);

			fclose(fp);


			//tsai-calibration

			/*
			LOG("image %d success finding corner\n", 0);
			for (int y = 0; y < cam_height_; y++) {
				for (int x = 0; x < cam_width_; x++) {
					QRgb rgb = src.pixel(x, y);
					view.at<cv::Vec3b>(y, x) = cv::Vec3b(qBlue(rgb), qGreen(rgb), qRed(rgb));
				}
			}
			cv::Mat viewGray;
			cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
			cv::cornerSubPix(viewGray, pointBuf, cv::Size(11, 11),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			cv::cornerSubPix(viewGray, pointBuf2, cv::Size(11, 11),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			std::vector<cv::Point2f> points = pointBuf;
			for (auto i : pointBuf2) {
				points.push_back(i);
			}
			imagePoints.push_back(points);

			//cv::drawChessboardCorners(view, cv::Size(4, 11), cv::Mat(pointBuf), found);
			cv::drawChessboardCorners(view, cv::Size(7, 5), cv::Mat(pointBuf), found1);
			cv::drawChessboardCorners(view, cv::Size(7, 5), cv::Mat(pointBuf2), found2);
			char out_fname[100];
			sprintf(out_fname, "out%04d.jpg", ith_cam);
			cv::imwrite(out_fname, view);
			
			//calcBoardCornerPositions(cv::Size(4, 11), 2.0, objectPoints[0]);
			calcBoardCornerPositions(cv::Size(7, 5),3.0, y_offset_, z_offset_, objectPoints[0]);

			graphics::vector<graphics::vec2> image_points(imagePoints[0].size());
			graphics::vector<graphics::vec3> world_points(objectPoints[0].size());
			for (int i = 0; i < imagePoints[0].size(); i++) {
				image_points[i] = vec2(imagePoints[0][i].x,imagePoints[0][i].y);
				world_points[i] = vec3(objectPoints[0][i].x, objectPoints[0][i].y, objectPoints[0][i].z);
			}


			camera_calibration calib;
			calib.init(cam_width_, cam_height_);
			calib.perform_calib(image_points, world_points, false);
			double kappa;
			libmv::PinholeCamera cam = calib.get_camera(0, kappa);
			pinhole_cameras_[ith_cam] = cam;
			double err_sum = 0;

			char name[100];
			sprintf(name, "scanner\\pt_grey_%d.cal", ith_cam);
			FILE* fp = fopen(name, "w");
			fprintf(fp, "%d %d\n", (int)cam.image_width(), (int)cam.image_height());
			fprintf(fp, "%lf %lf\n", cam.intrinsic_matrix()(0, 2), cam.intrinsic_matrix()(1, 2));
			fprintf(fp, "%lf %lf\n", cam.intrinsic_matrix()(0, 0), cam.intrinsic_matrix()(1, 1));
			fprintf(fp, "%Lf %Lf %lf %lf %lf %lf\n", kappa, 0,
				0, 0, 0,
				0);
			fclose(fp);

			sprintf(name, "scanner\\camera%d.txt", ith_cam);
			fp = fopen(name, "w");
			fprintf(fp, "%Lf %Lf %Lf\n %Lf %Lf %Lf\n %Lf %Lf %Lf\n", 
			cam.orientation_matrix()(0, 0),
				cam.orientation_matrix()(0, 1),
				cam.orientation_matrix()(0, 2),
				cam.orientation_matrix()(1, 0),
				cam.orientation_matrix()(1, 1),
				cam.orientation_matrix()(1, 2),
				cam.orientation_matrix()(2, 0),
				cam.orientation_matrix()(2, 1),
				cam.orientation_matrix()(2, 2));
			fprintf(fp, "%Lf %Lf %Lf\n", cam.position()[0], cam.position()[1], cam.position()[2]);

			fclose(fp);
			err_sum = 0;
			for (int i = 0; i < imagePoints[0].size(); i++) {
				vec2 p = pinhole_cameras_[ith_cam].project_point(vec3(objectPoints[0][i].x, objectPoints[0][i].y, objectPoints[0][i].z));
				err_sum += norm(p - vec2(imagePoints[0][i].x, imagePoints[0][i].y));
			}
			LOG("reprojection error %llf\n", err_sum / double(imagePoints[0].size()));
			*/
		}

		for (auto i : imgs)
			delete i;
	}
	board_geom_ = frame(vec3(x_offset_,y_offset_, 0), vec3(-1,0,0), vec3(0, 0, -1));

	// now construct projector
	compute_projector_plane();
	
}

void ProjectorScan::load_images(int ith_cam, std::vector<QImage*>& imgs)
{
	for (int i = 0; i < level_images_.size(); i++) {
		char fname[100];
		sprintf(fname, "pt_grey\\%d_%02d.bmp", ith_cam, i+1);
		QImage* n = new QImage;
		n->load(fname);
		imgs.push_back(n);
	}
}
static bool write_binaryheader(FILE* fp)
{
	typedef union
	{
		int  int_value;
		char byte_values[sizeof(int)];
	} endian_test_type;

	endian_test_type test;

	test.int_value = 0;
	test.int_value = 1;
	if (test.byte_values[0] == 1)
		fprintf(fp, "format binary_little_endian 1.0\n");
	else if (test.byte_values[sizeof(int) - 1] == 1)
		fprintf(fp, "format binary_big_endian 1.0\n");
	else
	{
		LOG("ply: Couldn't determine machine endianness.\n");
		return false;
	}

	return true;
}
static bool write_PLYfile(char* out_ply, std::vector<vec3>& points, std::vector<vec3>& points_normals, std::vector<vec3>& image_points_color, int filetype)
{
	FILE *fp;

	fp = fopen(out_ply, "wb");
	if (fp == NULL) {
		LOG("fail to open ply file \n");
		return false;
	}

	fprintf(fp, "ply\n");

	if (filetype == 0)
		fprintf(fp, "format ascii 1.0\n");
	else
		if (!write_binaryheader(fp))
		{
			fclose(fp);
			return false;
		}

	fprintf(fp, "element vertex %d\n", points.size());
	fprintf(fp, "property float x \n");
	fprintf(fp, "property float y \n");
	fprintf(fp, "property float z \n");
	fprintf(fp, "property float nx \n");
	fprintf(fp, "property float ny \n");
	fprintf(fp, "property float nz \n");
	fprintf(fp, "property uchar red \n");
	fprintf(fp, "property uchar green \n");
	fprintf(fp, "property uchar blue \n");
	fprintf(fp, "element face 0 \n");
	fprintf(fp, "property list uchar int vertex_indices \n");
	fprintf(fp, "end_header\n");

	float pt[3], n[3];
	uchar c[3];
	void* data;
	for (int i = 0; i < points.size(); i++)
	{
		pt[0] = points[i][0];
		pt[1] = points[i][1];
		pt[2] = points[i][2];

		if (_isnan(points_normals[i][0]) || _isnan(points_normals[i][1]) || _isnan(points_normals[i][2]))
		{
			points_normals[i] = vec3(0, 0, 1);
		}
		if (apx_equal(norm(points_normals[i]), 0, zero_epsilon)) {
			points_normals[i] = vec3(0, 0, 1);
		}

		n[0] = points_normals[i][0];
		n[1] = points_normals[i][1];
		n[2] = points_normals[i][2];
		//n[0] = 0.0; n[1] = 0.0; n[2] = 0.0;

		c[0] = image_points_color[i][0] * 255;
		c[1] = image_points_color[i][1] * 255;
		c[2] = image_points_color[i][2] * 255;

		if (filetype == 0)
			fprintf(fp, "%E %E %E %E %E %E %u %u %u\n", pt[0], pt[1], pt[2], n[0], n[1], n[2], c[0], c[1], c[2]);
		else {
			data = pt;
			fwrite(data, sizeof(float), 3, fp);
			data = n;
			fwrite(data, sizeof(float), 3, fp);
			data = c;
			fwrite(data, sizeof(uchar), 3, fp);
		}
	}

	fclose(fp);

	return true;
}
static bool write_PLYfile(char* out_ply, std::vector<vec3>& points, std::vector<vec3>& image_points_color, int filetype)
{
	FILE *fp;

	fp = fopen(out_ply, "wb");
	if (fp == NULL) {
		LOG("fail to open ply file \n");
		return false;
	}

	fprintf(fp, "ply\n");

	if (filetype == 0)
		fprintf(fp, "format ascii 1.0\n");
	else
		if (!write_binaryheader(fp))
		{
			fclose(fp);
			return false;
		}

	fprintf(fp, "element vertex %d\n", points.size());
	fprintf(fp, "property float x \n");
	fprintf(fp, "property float y \n");
	fprintf(fp, "property float z \n");
	fprintf(fp, "property uchar red \n");
	fprintf(fp, "property uchar green \n");
	fprintf(fp, "property uchar blue \n");
	fprintf(fp, "element face 0 \n");
	fprintf(fp, "property list uchar int vertex_indices \n");
	fprintf(fp, "end_header\n");

	float pt[3], n[3];
	uchar c[3];
	void* data;
	for (int i = 0; i < points.size(); i++)
	{
		pt[0] = points[i][0];
		pt[1] = points[i][1];
		pt[2] = points[i][2];

		c[0] = image_points_color[i][0] * 255;
		c[1] = image_points_color[i][1] * 255;
		c[2] = image_points_color[i][2] * 255;

		if (filetype == 0)
			fprintf(fp, "%E %E %E %u %u %u\n", pt[0], pt[1], pt[2], c[0], c[1], c[2]);
		else {
			data = pt;
			fwrite(data, sizeof(float), 3, fp);
			data = c;
			fwrite(data, sizeof(uchar), 3, fp);
		}
	}

	fclose(fp);

	return true;
}
static const int merge_cnt = 40;
static const int merge_half_cnt = 30;
void process_string(int s, int e, int w, int h, int x, bool s_flag, bool e_flag, unsigned char* src, unsigned char* mask)
{
	if (e - s <= merge_cnt) {
		if (s_flag && e_flag) {
			int cnt = 0, p_cnt = 0;
			for (int i = s; i <= e; i++) {
				if (src[x + i * w] == 255 && mask[x + i * w]) p_cnt++;
				if (mask[x + i * w]) cnt++;
			}
			if (cnt) {
				for (int i = s; i < e; i++) {
					src[x + i * w] = (p_cnt * 3 >= cnt) ? 255 : 0;
				}
			}
		}
		if (s_flag && !e_flag) {
			int cnt = 0, p_cnt = 0;
			for (int i = s; i <= s + merge_half_cnt && i < e; i++) {
				if (src[x + i * w] == 255 && mask[x + i * w]) p_cnt++;
				if (mask[x + i * w]) cnt++;
			}
			if (cnt) {
				for (int i = s; i <= s + merge_half_cnt && i < e; i++) {
					src[x + i * w] = (p_cnt * 3 >= cnt) ? 255 : 0;
				}
			}
		}
		if (!s_flag && e_flag) {
			int cnt = 0, p_cnt = 0;
			for (int i = e; i >= e - merge_half_cnt && i >= s; i--) {
				if (src[x + i * w] == 255 && mask[x + i * w]) p_cnt++;
				if (mask[x + i * w]) cnt++;
			}
			if (cnt) {
				for (int i = e; i >= e - merge_half_cnt && i >= s; i--) {
					src[x + i * w] = (p_cnt * 3 >= cnt) ? 255 : 0;
				}
			}
		}
	}
	else {
		if (s_flag) {
			int cnt = 0, p_cnt = 0;
			for (int i = s; i <= s + merge_half_cnt && i < e; i++) {
				if (src[x + i * w] == 255 && mask[x + i * w]) p_cnt++;
				if (mask[x + i * w]) cnt++;
			}
			if (cnt) {
				for (int i = s; i <= s + merge_half_cnt && i < e; i++) {
					src[x + i * w] = (p_cnt * 3 >= cnt) ? 255 : 0;
				}
			}
		}
		if (e_flag) {
			int cnt = 0, p_cnt = 0;
			for (int i = e; i >= e - merge_half_cnt && i >= s; i--) {
				if (src[x + i * w] == 255 && mask[x + i * w]) p_cnt++;
				if (mask[x + i * w]) cnt++;
			}
			if (cnt) {
				for (int i = e; i >= e - merge_half_cnt && i >= s; i--) {
					src[x + i * w] = (p_cnt * 3 >= cnt) ? 255 : 0;
				}
			}
		}
	}
}
static void hill_flat3(int rep_v, int s, int e, int w, int h, int x, float* src, float* shift_code, short* code, unsigned char* mask)
{
	float pv = src[x + s * w];
	int p_mark = 0, a_mark = 1000000;
	for (int i = s + 1; i <= e; i++) {
		float cv = src[x + i * w];
		float dif = cv - pv;
		float size = abs(dif);
		if (dif > 0 && size > 5.0 && i * 2 <= (s + e) && mask[x + (i - 1)*w] && mask[x + i * w]) {
			p_mark = i;
		}
		else if (dif > 0 && size > 5.0 && i * 2 > (s + e) && mask[x + (i - 1)*w] && mask[x + i * w]) {
			a_mark = i;
		}
		pv = cv;
	}
	for (int i = p_mark; i > s; i--) {
		src[x + (i - 1) * w] = (float)code[x + (s - 1)*w] + shift_code[x + (i - 1) * w];
	}
	for (int i = a_mark; i <= e; i++) {
		src[x + i * w] = (float)code[x + (e + 1)*w] + shift_code[x + i * w];
	}
}

void ProjectorScan::compute_3d_point_cloud(int ith_cam)
{
	int imsize = cam_width_ * cam_height_;

	// mask generation

	float* proj_map = (float*)malloc(cam_width_*cam_height_ * sizeof(float));
	for (int i = 0; i < imsize; i++)
		proj_map[i] = -1;


	std::vector<QImage*> imgs;
	load_images(ith_cam, imgs);
	QImage& black = *imgs[1];
	QImage& white = *imgs[0];

	compute_grey_code(imgs, proj_map, 5.0, 245.0);
	
	frame projector_frame = projector_.get_camera_frame();
	std::vector<vec3> points(cam_width_*cam_height_, vec3(-1000));
	std::vector<uchar> noise_map(cam_width_*cam_height_, 0);

	for (int x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			float v = proj_map[x + y * cam_width_];
			if (v < 2.0 || v > float(proj_width_)) continue;

			line ll = pinhole_cameras_[ith_cam].ray(vec2(x, y));
			//ll = pinhole_cameras_[ith_cam].to_world(ll);
			line ray1 = projector_.ray(vec2(v, 0));
			line ray2 = projector_.ray(vec2(v, proj_height_));
			ray1 = projector_frame.to_world(ray1);
			ray2 = projector_frame.to_world(ray2);
			plane pl(ray1.get_position(), ray1.get_position() + ray1.get_direction()* 100.0, ray2.get_position() + ray2.get_direction()*100.0);

			vec3	pnt;
			if (pl.intersect(ll, pnt) && norm(pnt)< 200) {
				points[x + y * cam_width_] = pnt; // pinhole_cameras_[ith_cam].to_world(pnt);
				noise_map[x + y * cam_width_] = 1;
			}
		}
	}

	int x, y;

	int filter_w = 4;
	int step = filter_w * 2 + 1;
	for (int i = 0; i < step; i++) {
		#pragma omp parallel for private(x)
		for (x = filter_w + i; x < cam_width_ - filter_w; x += step) {
			for (int y = filter_w; y < cam_height_ - filter_w; y++) {
				vec3 query_point = points[x + y * cam_width_];
				if (query_point[0] < -999.0) continue;
				real sum_d = 0.0;
				int cnt = 0;
				for (int xx = x - filter_w; xx <= x + filter_w; xx++) {
					for (int yy = y - filter_w; yy <= y + filter_w; yy++) {
						vec3 cur = points[xx + yy * cam_width_];
						if (cur[0] < -999.0) continue;
						real d = norm(query_point - cur);
						sum_d += d;
						cnt++;
					}
				}
				if (cnt < 10)  continue;
				real mean = sum_d / double(cnt);
				sum_d = 0;
				for (int xx = x - filter_w; xx <= x + filter_w; xx++) {
					for (int yy = y - filter_w; yy <= y + filter_w; yy++) {
						vec3 cur = points[xx + yy * cam_width_];
						if (cur[0] < -999.0) continue;
						real d = norm(query_point - cur);
						sum_d += (d - mean)*(d - mean);
					}
				}
				real s_dev = sqrt(sum_d / (double)cnt);
				for (int xx = x - filter_w; xx <= x + filter_w; xx++) {
					for (int yy = y - filter_w; yy <= y + filter_w; yy++) {
						vec3 cur = points[xx + yy * cam_width_];
						if (cur[0] < -999.0) continue;
						real d = norm(query_point - cur);
						d = sqrt((d - mean)*(d - mean));
						if (d > s_dev *2.9) {
							noise_map[xx + yy * cam_width_] = 0;
						}
					}
				}

			}
		}
	}
	for (int x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			if (noise_map[x + y * cam_width_] == 0) {
				points[x + y * cam_width_] = vec3(-1000);
			}
		}
	}
	for (int i = 0; i < step; i++) {
#pragma omp parallel for private(x)
		for (x = filter_w + i; x < cam_width_ - filter_w; x += step) {
			for (int y = filter_w; y < cam_height_ - filter_w; y++) {
				vec3 query_point = points[x + y * cam_width_];
				if (query_point[0] < -999.0) continue;
				real sum_d = 0.0;
				int cnt = 0;
				for (int xx = x - filter_w; xx <= x + filter_w; xx++) {
					for (int yy = y - filter_w; yy <= y + filter_w; yy++) {
						vec3 cur = points[xx + yy * cam_width_];
						if (cur[0] < -999.0) continue;
						real d = norm(query_point - cur);
						sum_d += d;
						cnt++;
					}
				}
				if (cnt < 10)  continue;
				real mean = sum_d / double(cnt);
				sum_d = 0;
				for (int xx = x - filter_w; xx <= x + filter_w; xx++) {
					for (int yy = y - filter_w; yy <= y + filter_w; yy++) {
						vec3 cur = points[xx + yy * cam_width_];
						if (cur[0] < -999.0) continue;
						real d = norm(query_point - cur);
						sum_d += (d - mean)*(d - mean);
					}
				}
				real s_dev = sqrt(sum_d / (double)cnt);
				for (int xx = x - filter_w; xx <= x + filter_w; xx++) {
					for (int yy = y - filter_w; yy <= y + filter_w; yy++) {
						vec3 cur = points[xx + yy * cam_width_];
						if (cur[0] < -999.0) continue;
						real d = norm(query_point - cur);
						d = sqrt((d - mean)*(d - mean));
						if (d > s_dev *2.9) {
							noise_map[xx + yy * cam_width_] = 0;
						}
					}
				}

			}
		}
	}
	std::vector<vec3> save_points;
	std::vector<vec3> save_colors;
	for (int x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			if (noise_map[x + y * cam_width_] == 1) {
				save_points.push_back(points[x + y * cam_width_]);
				QRgb vv = white.pixel(x, y);
				int r = qRed(vv);
				int g = qGreen(vv);
				int b = qBlue(vv);
				vec3 col(real(r) / 255.0, real(g) / 255.0, real(b) / 255.0);
				save_colors.push_back(col);
			}
		}
	}
	char name[1000];
	sprintf(name, "result%d.ply", ith_cam);
	write_PLYfile(name, save_points, save_colors, 1/*binary*/);

	QImage save_im2(cam_width_, cam_height_, QImage::Format::Format_RGB888);
	for (int x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			unsigned int v = proj_map[x + y * cam_width_];
			unsigned int r = (v & 0x0000FF00) >> 8, g = v & 0x000000FF;
			save_im2.setPixel(x, y, qRgb(r, g, 0));
		}
	}
	sprintf(name, "projector_new%d.bmp", ith_cam);
	save_im2.save(name);

	for (auto i : imgs)
		delete i;

	free(proj_map);

}
inline uint pixel(uint* im, int x, int y, int w)
{
	return im[x + y * w];
}

inline void set_pixel(uint* im, int x, int y, int w, uint r, uint g, uint b)
{
	uint val = (r << 16 | g << 8 | b);
	im[x + y * w] = val;
}

inline uint my_blue(uint v)
{	
	return v & 0xFF;
}

inline uint my_green(uint v)
{
	return (v>>8) & 0xFF;
}
inline uint my_red(uint v)
{
	return (v >> 16) & 0xFF;
}
#define SINE_CNT 160
#ifndef max
#define max(a,b) (a>b?a:b)
#endif
void ProjectorScan::compute_grey_code(std::vector<QImage*>& imgs, float* proj_map_float, float min_BW_diff, float max_BW_diff)
{
	int imsize = cam_width_ * cam_height_;

	float* sine_table = (float*)malloc(8 * SINE_CNT * sizeof(float));
	for (int i = 0; i < SINE_CNT; i++) {
		for (int j = 0; j < 8; j++) {
			sine_table[i * 8 + j] = sin(2.0*M_PI*(float(i) / 160.0 + float(j) / 8.0));
		}
	}
	// mask generation
	unsigned char* mask = (unsigned char*)malloc(cam_width_*cam_height_);
	unsigned char* code = (unsigned char*)malloc(cam_width_*cam_height_ * sizeof(unsigned char));
	unsigned char* save = (unsigned char*)malloc(cam_width_*cam_height_ * sizeof(unsigned char));
	short* code_proj = (short*)malloc(cam_width_*cam_height_ * sizeof(short));
	short* proj_rep = (short*)malloc(cam_width_*cam_height_ * sizeof(short));
	float* value = (float*)malloc(cam_width_*cam_height_ * sizeof(float));
	float* shift_code = (float*)malloc(cam_width_*cam_height_ * sizeof(float));
	memset(code, 0, imsize * sizeof(unsigned char));
	memset(proj_rep, 0, imsize * sizeof(short));

	QImage& black = *imgs[1];
	QImage& white = *imgs[0];

	QImage* shift[8];
	for (int i = 0; i < 8; i++) {
		shift[i] = imgs[imgs.size() - 10 + i];
	}
	// remove shadow and highlight
	int x;
#pragma omp parallel for private(x)
	for (x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			float v = value[x + y * cam_width_] = qBlue(white.pixel(x, y)) - qBlue(black.pixel(x, y));
			mask[x + y * cam_width_] = (v < min_BW_diff || v > max_BW_diff) ? 0 : 1;
		}
	}
	uint* save_im1 = (uint*)malloc(cam_width_*cam_height_ * sizeof(uint));
	uint* save_im2 = (uint*)malloc(cam_width_*cam_height_ * sizeof(uint));

#pragma omp parallel for private(x)
	for (x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			if (mask[x + y * cam_width_] == 0) continue;
			int v = (qBlue(imgs[imgs.size() - 2]->pixel(x, y)) - qBlue(imgs[imgs.size() - 1]->pixel(x, y)));
			unsigned int r = v > 0 ? v : 0;
			unsigned int g = v < 0 ? -v : 0;
			unsigned int b = abs(v) < 1 ? 255 : 0;
			set_pixel(save_im1, x, y, cam_width_, r, g, b);
			set_pixel(save_im2, x, y, cam_width_, r, g, b);
		}
	}
#pragma omp parallel for private(x)
	for (x = 1; x < cam_width_ - 1; x++) {
		for (int y = 1; y < cam_height_ - 1; y++) {
			if (mask[x + y * cam_width_] == 0) continue;
			uint v = pixel(save_im1, x, y, cam_width_);
			if (my_blue(v) || my_red(v) == 0) {
				continue;
			}

			uint v0 = pixel(save_im1, x - 1, y, cam_width_);
			uint v1 = pixel(save_im1, x + 1, y, cam_width_);
			uint v2 = pixel(save_im1, x, y - 1, cam_width_);
			uint v3 = pixel(save_im1, x, y + 1, cam_width_);
			if (my_green(v0) || my_green(v1) || my_green(v2) || my_green(v3)) {
				set_pixel(save_im2, x, y, cam_width_, my_red(v), my_green(v), 255);
			}
		}
	}
#pragma omp parallel for private(x)
	for (x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			if (mask[x + y * cam_width_] == 0) continue;
			uint v = pixel(save_im2, x, y, cam_width_);
			if (my_blue(v)) set_pixel(save_im2, x, y, cam_width_, 0, 0, 255); // this should be selected
			else if (my_red(v)) set_pixel(save_im2, x, y, cam_width_, 255, 0, 0);
			else if (my_green(v)) set_pixel(save_im2, x, y, cam_width_, 0, 255, 0);
		}
	}
	//char name[100];
	//sprintf(name, "inverse.bmp");
	//save_im2.save(name);

	int start_level = 2;
	int end_level = imgs.size() - 10; // 8 shift pattern
	int total_level = end_level - start_level - 1;
	std::vector<float> level_thresholds(total_level, 0.5);
	int level;

#pragma omp parallel for private(level)
	for (level = start_level; level < end_level; level++) {

		std::vector<float> thresholds(20, 0);
		for (int i = 0; i < 20; i++) {
			thresholds[i] = 0.5 + (float(i) - 10.0)*0.1 / 20.0;
		}
		std::vector<int> threshold_zero_count(thresholds.size(), 0);
		int startx = cam_width_ * 0.3;
		int endx = cam_width_ * 0.7;
		for (int x = startx; x < endx; x++) {
			for (int y = 1; y < cam_height_; y++) {


				float t = qBlue(imgs[level]->pixel(x, y)) - qBlue(black.pixel(x, y));
				for (int i = 0; i < thresholds.size(); i++) {
					if (apx_equal(value[x + y * cam_width_] * thresholds[i] - t, 0.0, 0.1)
						&& my_blue(pixel(save_im2, x, y, cam_width_))) {
						threshold_zero_count[i]++;
					}
				}
			}
		}
		int max_count = 0;
		int max_i = 0;
		for (int i = 0; i < thresholds.size(); i++) {
			if (threshold_zero_count[i] > max_count) {
				max_count = threshold_zero_count[i];
				max_i = i;
			}
		}
		level_thresholds[level - start_level] = thresholds[max_i];
		//level_thresholds.push_back(thresholds[max_i]);
		LOG("level %d threshold %f\n", level, thresholds[max_i]);
	}

	for (level = start_level; level < end_level; level++) {

		memset(save, 100, imsize);
		int x;
#pragma omp parallel for private(x)
		for (x = 0; x < cam_width_; x++) {
			int start = 0, end = 1;
			bool s_flag = false, e_flag = false;
			for (int y = 1; y < cam_height_; y++) {


				float t = qBlue(imgs[level]->pixel(x, y)) - qBlue(black.pixel(x, y));
				if (t >= value[x + y * cam_width_] * level_thresholds[level - start_level]) save[x + y * cam_width_] = 255;
				else save[x + y * cam_width_] = 0;

				if (my_blue(pixel(save_im2, x, y, cam_width_))) {
					end = y;
					e_flag = true;
					process_string(start, end, cam_width_, cam_height_, x, s_flag, e_flag, save, mask);
					s_flag = true;
					e_flag = false;
					start = y + 1;
					y++;
				}
				else if (y == cam_height_ - 1) {
					e_flag = false;
					process_string(start, end, cam_width_, cam_height_, x, s_flag, e_flag, save, mask);
				}
			}
		}
#pragma omp parallel for private(x)
		for (x = 0; x < cam_width_; x++) {
			for (int y = 0; y < cam_height_; y++) {
				if (mask[x + y * cam_width_] == 0) continue;
				if (save[x + y * cam_width_] == 255)
					code[x + y * cam_width_] |= 1 << (total_level - (level - start_level));
			}
		}
	}

	std::vector<uchar> proj_code(proj_width_, 0);
	for (int level = start_level; level < end_level; level++) {
		for (int i = 0; i < proj_width_; i++) {
			proj_code[i] |= (level_images_[level][i] ? (1 << (total_level - (level - start_level))) : 0);
		}
	}

	std::vector<short> inv_code(proj_width_, 0);
	for (short i = 0; i < proj_code.size(); i++) {
		inv_code[(int)proj_code[i]] = i;
	}
#pragma omp parallel for private(x)
	for (x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			if (mask[x + y * cam_width_] == 0) continue;
			unsigned char val = code[x + y * cam_width_];
			code_proj[x + y * cam_width_] = (short)inv_code[(int)val];
		}
	}

	//int filter_size = 5;
	//int filter_half = 2;
	//float gaus_filter[5] = { 0.06136,	0.24477, 	0.38774,	0.24477, 0.06136 };
	int filter_size = 3;
	int filter_half = 1;
	float gaus_filter[3] = { 0.27901,	0.44198,	0.27901 };


	int* same_regions[256];
	for (int i = 0; i < 256; i++) {
		same_regions[i] = (int*)malloc(sizeof(int) * 1920);
		for (int j = 0; j < proj_width_; j++) {
			same_regions[i][j] = -1;
			if ((int)proj_code[j] == i) {
				same_regions[i][j] = j;
			}
		}
	}



#pragma omp parallel for private(x)
	for (x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			if (mask[x + y * cam_width_] == 0) continue;

			unsigned char val = code[x + y * cam_width_];
			//std::vector<int>& same_region = same_regions[val];
			int *same_region = same_regions[(int)val];
			// Note that I use graphics::vector because that allows circular access to the elment
			graphics::vector<float> shift_code_captured(8);

			int same_region_v = 0;

			for (int i = 0; i < proj_width_; i++) {

				uchar idx = 0;
				if (same_region[i] == -1) continue;
				for (int j = 0; j < 8; j++) {
					idx |= level_images_[end_level + j][(same_region)[i]] << (7 - j);
				}
				if ((int)idx >= 256) continue;
				same_region_v = same_region[i];
				break;
			}

			for (int i = 0; i < 8; i++) {
				shift_code_captured[i] = (float(qBlue(shift[i]->pixel(x, y)) - qBlue(shift[(i + 4) % 8]->pixel(x, y))));
			}
			graphics::vector<float> save_ = shift_code_captured;
			float min_v(10000.0);
			float max_v(-10000.0);
			for (int i = 0; i < 8; i++) {
				float sum = 0;
				for (int j = 0; j < filter_size; j++) {
					sum += (save_[i - j - 1] * gaus_filter[j]);
				}
				shift_code_captured[i] = sum;
				min_v = min(sum, min_v);
				max_v = max(sum, max_v);
			}
			float k_value = (fabs(min_v) + fabs(max_v)) * 0.5;

			int min_diff_i;
			float min_diff = 100000.0;
			for (int i = 0; i < SINE_CNT; i++) {
				float sum = 0;
				for (int j = 0; j < 8; j++) {
					sum += fabs(shift_code_captured[j] - k_value * sine_table[i * 8 + j]);
				}
				if (sum < min_diff) {
					min_diff_i = i;
					min_diff = sum;
				}
			}
			float sc = 8.0 - (float)min_diff_i / 20.0;
			shift_code[x + y * cam_width_] = sc;
			proj_map_float[x + y * cam_width_] = (float)same_region_v + sc;
			proj_rep[x + y * cam_width_] = same_region_v;
		}
	}


#pragma omp parallel for private(x)
	for (x = 0; x < cam_width_ - 1; x++) {
		int start = 1, end = 1;
		short v1 = code_proj[x];
		for (int y = 1; y < cam_height_ - 1; y++) {
			if (mask[x + y * cam_width_] == 0) continue;

			short v2 = code_proj[x + y * cam_width_];
			if (v1 != v2) {
				end = y;
				if (start != end && end - start > 5)
					hill_flat3(v1, start, end, cam_width_, cam_height_, x, proj_map_float, shift_code, proj_rep, mask);
				start = end;
				if (start == 0) start = 1;
				v1 = code_proj[x + start * cam_width_];
			}
			else if (y == cam_height_ - 1) {
				end = y;
			}
		}
	}
	QImage save_im3(cam_width_, cam_height_, QImage::Format::Format_RGB888);

	for (x = 0; x < cam_width_; x++) {
		for (int y = 0; y < cam_height_; y++) {
			if (mask[x + y * cam_width_] == 0) continue;
			unsigned int v = (unsigned int)proj_map_float[x + y * cam_width_];
			unsigned int r = (v & 0x0000FF00) >> 8, g = v & 0x000000FF;
			save_im3.setPixel(x, y, qRgb(r, g, 0));
		}
	}


	save_im3.save("projector_new.bmp");
	for (int i = 0; i < 256; i++) {
		free(same_regions[i]);
	}

	free(save_im1);
	free(save_im2);

	free(shift_code);
	free(proj_rep);
	free(mask);
	free(code);
	free(value);
	free(code_proj);
	free(save);
	free(sine_table);
}
static void change_basis(frame& f1, frame& f2) {
	frame fnew;
	vec3 v1 = f1.to_model_normal(f2.basis[0]);
	vec3 v2 = f1.to_model_normal(f2.basis[1]);
	vec3 v3 = f1.to_model_normal(f2.basis[2]);
	vec3 e = f1.to_model(f2.get_origin());
	f2 = frame(e, v1, v2, v3);
	f1 = fnew;
}

matrix3x3 convert3x3(cv::Mat& r)
{
	matrix3x3 ret;
	for (int i =0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			ret(i, j) = r.at<double>(i, j);
		}
	return ret;
}
matrix3x4 convert3x4(cv::Mat& P)
{
	matrix3x4 ret;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 4; j++) {
			ret(i, j) = P.at<double>(i, j);
		}
	return ret;
}
#ifndef min
#define min(a,b) (a>b?b:a)
#endif
#ifndef max
#define max(a,b) (a>b?a:b)
#endif

void compute_inverse_map(cv::Mat& mat_x, cv::Mat& mat_y, float* inv_mat_x, float* inv_mat_y)
{
	int w = mat_x.cols, h = mat_x.rows;
	int tot = w * h;
	for (int x = 0; x < tot; x++) {
		inv_mat_x[x] = -1;
		inv_mat_y[x] = -1;
	}

	for (int y = 1; y < h; y++) {
		for (int x = 0; x < w-1; x++) {
			vec2 p1(mat_x.at<float>(y - 1, x), mat_y.at<float>(y - 1, x));
			vec2 p2(mat_x.at<float>(y -1, x + 1), mat_y.at<float>(y - 1, x + 1));
			vec2 p3(mat_x.at<float>(y, x), mat_y.at<float>(y, x));
			vec2 p4(mat_x.at<float>(y, x + 1), mat_y.at<float>(y, x + 1));
			float v1 = x;
			float v2 = x + 1;
			float v3 = x;
			float v4 = x + 1;
			float vv1 = y - 1;
			float vv2 = y - 1;
			float vv3 = y;
			float vv4 = y;
			box2 box;
			box.extend(p1);
			box.extend(p2);
			box.extend(p3);
			box.extend(p4);
			int xs = min(w-1, max(0,floor(box.minimum[0])));
			int xe = min(w - 1, max(0, ceil(box.maximum[0])));
			int ys = min(h - 1, max(0, floor(box.minimum[1])));
			int ye = min(h - 1, max(0, ceil(box.maximum[1])));
			for (int xx = xs; xx <= xe; xx++) {
				for (int yy = ys; yy <= ye; yy++) {
					vec3 bcoord = barycentric_coordinate(p1, p2, p3, vec2(xx,yy));
					if (bcoord[0] >= 0.0-zero_epsilon && bcoord[0] <= 1.0 + zero_epsilon 
						&& bcoord[1] >= 0.0 - zero_epsilon && bcoord[1] <= 1.0 + zero_epsilon 
						&& bcoord[2] >= 0.0 - zero_epsilon && bcoord[2] <= 1.0 + zero_epsilon)
					{
						float v = v1 * bcoord[0] + v2 * bcoord[1] + v3 * bcoord[2];
						inv_mat_x[xx + yy * w] = v;
						v = vv1 * bcoord[0] + vv2 * bcoord[1] + vv3 * bcoord[2];
						inv_mat_y[xx + yy * w] = v;
						continue;
					}
					bcoord = barycentric_coordinate(p2, p3, p4, vec2(xx, yy));
					if (bcoord[0] >= 0.0 - zero_epsilon && bcoord[0] <= 1.0 + zero_epsilon
						&& bcoord[1] >= 0.0 - zero_epsilon && bcoord[1] <= 1.0 + zero_epsilon
						&& bcoord[2] >= 0.0 - zero_epsilon && bcoord[2] <= 1.0 + zero_epsilon)
					{
						float v = v2 * bcoord[0] + v3 * bcoord[1] + v4 * bcoord[2];
						inv_mat_x[xx + yy * w] = v;
						v = vv1 * bcoord[0] + vv2 * bcoord[1] + vv3 * bcoord[2];
						inv_mat_y[xx + yy * w] = v;
						continue;
					}
				}
			}
		}
	}
}
void save_float_image(float* im, int w, int h, char* name)
{
	float* save = (float*)malloc(w*h * sizeof(float));
	memset(save, 0, w*h * sizeof(float));
	float min_v = 100000000;
	float max_v = -10000000;
	for (int x = 0; x < w; x++) {
		for (int y = 0; y < h; y++) {
			float v = im[x + y * w];
			min_v = min(v, min_v);
			max_v = max(v, max_v);
		}
	}
	for (int x = 0; x < w; x++) {
		for (int y = 0; y < h; y++) {
			float v = im[x + y * w];
			min_v = min(v, min_v);
			max_v = max(v, max_v);
			save[w*y + x] = ((v - min_v) / (max_v - min_v))*255.0;
		}
	}
	cv::Mat img(h, w, CV_32FC1, save);
	cv::imwrite(name, img);
	free(save);
}

void ProjectorScan::undistort_image(QImage& im,int cam_id)
{
	cv::Mat view = cv::Mat::zeros(im.height(), im.width(), CV_8UC3);
	int w = im.width();
	int h = im.height();
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			QRgb rgb = im.pixel(x, y);
			view.at<cv::Vec3b>(y, x) = cv::Vec3b(qBlue(rgb), qGreen(rgb), qRed(rgb));
		}
	}
	cv::Mat ret;
	cv::undistort(view, ret, cam_id == 0?CM1:CM2, cam_id == 0 ? D1:D2);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			QRgb rgb = im.pixel(x, y);
			cv::Vec3b v = ret.at<cv::Vec3b>(y, x);
			im.setPixel(x, y, qRgb(v(2), v(1), v(0)));
		}
	}
}


void ProjectorScan::scan()
{
	for (auto i : cameras_) {
		i->ConfigureTrigger(false);
	}

	for (auto i : cameras_) {
		i->start_camera();
	}
	gl_stat stat;
	stat.save_stat();

	for (int i = 0; i < level_images_.size(); i++) {
		project_pattern(i);
		Sleep(100);
		for (auto j : cameras_) {
			j->Capture(i);
		}
		
	}
	project_pattern(level_images_.size()-1);
	Sleep(100);
	for (auto j : cameras_) {
		j->Capture(level_images_.size());
	}

	for (auto i : cameras_) {
		i->SaveToDisk();
	}

	for (auto i : cameras_) {
		i->end_camera();
	}

	if (mode_ == Capture) {

	}
	else if (mode_ == Calibrate) {
		do_calibration();
	}
	mode_ = Preview;
	preview_timer_.start();
	
	//UserKernel::Instance()->UpdateViews();
	stat.restore_stat();
}

/*
void ProjectorScan::engine(int y, int x, int r, ChannelMask channels, Row& row) 
{

	foreach(z, channels) 
	{
		if (z == Chan_Red || z == Chan_Blue || z == Chan_Green) {

			real_t* out = row.writable(z);

			for (int xx = x ; xx < r ; xx++) {
				out[xx] = (real_t)(camera_buffer_[(xx)*3 + (y) * preview_width_*3 + (z-1)])/255.0;
			}
		}
		else if (z == Chan_Alpha) {

			real_t* out = row.writable(z);

			for (int xx = x ; xx < r ; xx++) {
				out[xx] = 1.0;
			}
		}
	}
}


#include "knobs/Knobs.h"
int ProjectorScan::knob_changed(Knob* k)
{
	if (k == scan_button_) {
		count_ = 0;
		mode_ = Capture;

	}
	if (k == calibration_button_) {
		count_ = 0;
		mode_ = Calibrate;
		//calibrate();
	}
	if (k == exposure_knob_) {
		for (auto i : cameras_) {
			//i->waitForCompletion();
			i->SetExposure(exposure_);
		}
	}
	if (k == camera_release_button_)
	{
		ReleaseCameraSystem();
	}
	if (k == test_button_) {
		// construct_rectification_env();
		// compute_stereo(); 
		//for (int i = 0; i < num_camera_; i++) {
			//compute_3d_point_cloud(0);
		//}

	}
	return 1;
}
void ProjectorScan::knobs(Knob_Callback f) {
	scan_button_ = Button(f, "scan", "scan");
	Divider(f, "a");
	exposure_knob_ = Double_knob(f, &exposure_, IRange(0, 33250), "exposure");
	Int_knob(f, &proj_width_, "projector width");
	Int_knob(f, &proj_height_, "projector height");
	Divider(f, "b");
	Newline(f, "c");
	calibration_button_ = Button(f, "calibration", "calibration");
	Newline(f, "d");
	Float_knob(f, &board_size_w_, IRange(1, 500), "board_width");
	Float_knob(f, &board_size_h_, IRange(1, 500), "board_height");
	Float_knob(f, &x_offset_, IRange(-500, 500), "x offset");
	Float_knob(f, &y_offset_, IRange(-500, 500), "y offset");
	Float_knob(f, &z_offset_, IRange(-500, 500), "z offset");
	Newline(f, "e");
	camera_release_button_ = Button(f, "shutdown camera", "shutdown camera");
	test_button_ = Button(f, "test", "test");
}


static Iop* AddInputsCreate(UserNode* node) {return new ProjectorScan(node);}
const Op::Description ProjectorScan::d("ProjectorScan", "ProjectorScan", AddInputsCreate);

};
*/

};

 void KoonInitProjectorScan()
{
}

 