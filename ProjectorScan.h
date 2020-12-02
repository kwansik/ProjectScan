#ifndef __ProjectorScan_h
#define __ProjectorScan_h

#include <user/Iop.h>
#include <knobs/Knobs.h>
#include <user/Row.h>
#include <graphics/sys.h>
#include <user_app/shuffle_view.h>
#include <custom_widgets/KImageDisplayDialog.h>
#include "libmv/image/array_nd.h"
#include "libmv/image/image.h"
#include <user/LightNode.h>
#include <graphics/crop_ms.h>
#include <graphics/texture_buffer_ms.h>
#include <user_app/PointGrey.h>
#include <graphics/Timer.h>
#include "libmv/camera/pinhole_camera.h"
#include <opencv2/core.hpp>

#include <libmv/correspondence/klt.h>


namespace user {

class ProjectorScan :  public Iop
{
	enum scan_mode
	{
		Preview,
		Capture,
		Calibrate,
	};

	scan_mode mode_;

	bool global_shutter_;

	Spinnaker::SystemPtr camera_system_;
	std::vector< Spinnaker::CameraPtr> camList_;
	std::vector< PointGreyCapture*> cameras_;
	unsigned char*	camera_buffer_;
	unsigned char* proj_buffer_;
	float* sine_table;

    int current_buffer_;
	static bool initialized_;
	double exposure_;

	int preview_width_;
	int preview_height_;
	int proj_width_;
	int proj_height_;
	int cam_width_;
	int cam_height_;

	float x_offset_;
	float y_offset_;
	float z_offset_;
	float board_size_w_;
	float board_size_h_;

	frame board_geom_;
	libmv::PinholeCamera pinhole_cameras_[2];
	libmv::PinholeCamera projector_;
	cv::Mat D1, D2, CM1, CM2;
	cv::Mat map1x, map1y, map2x, map2y;
	float* invmap1x, *invmap1y, *invmap2x, *invmap2y;
	vec3 proj_rot_axis_;
	vec3 proj_pos_;
	vec3 proj_start_dir_;
	double proj_rot_step_;

	graphics::Timer preview_timer_;
	size_t count_;
	int num_camera_;
	size_t preview_count_;
	Knob* scan_button_;
	Knob* calibration_button_;
	Knob* camera_release_button_;
	Knob* test_button_;
	Knob* exposure_knob_;
	std::vector<std::vector<int>> level_images_;
	int proj_stat_;
	std::vector<int> inverse_1_;
	std::vector<int> inverse_2_;
	static KImageDisplayDialog* device_;
	HANDLE handlePort_;

	KLT_TrackingContext context_;
public:



	ProjectorScan(UserNode* node);
	ProjectorScan();
	
	~ProjectorScan();
	int knob_changed(Knob* k);
	void draw_handle(ViewerContext*);
	void knobs(Knob_Callback f);
	void _validate(bool);
	void engine ( int y, int x, int r, ChannelMask channels, Row& out );
	virtual void append(user::Hash& a); 
	const char* Class() const { return "ProjectorScan"; }
	const char* node_help() const { return "ProjectorScan Iop"; }
	int minimum_inputs() const { return 1; }
	int maximum_inputs() const { return 1; }
	virtual std::string input_name(int i) const
	{
		if (i <= minimum_inputs()) {
			if (i == 0) return std::string("FaceNet");
		}
		return std::string("FaceNet");
	}
	bool test_input(int, Op*) const;
	static const user::Op::Description d;
	//int knob_changed(Knob*);

private:
	int filter_win_;

	void initCameraProjectorGeometry();
	void initCameraSystem();
	int ConfigureTrigger(Spinnaker::GenApi::INodeMap& nodeMap, bool hardware = false);
	int ResetTrigger(Spinnaker::GenApi::INodeMap& nodeMap);
	void DeinitCamera(Spinnaker::CameraPtr pCam);
	void ReleaseCameraSystem();
	void releaseComPort();
	void initialize();
	void initOutBuffer();
	void scan();

	void project_pattern(int ith);
	void undistort_image(QImage& im, int cam_id);
	void preview();
	void do_calibration();
	void compute_projector_plane();
	void compute_3d_point_cloud(int ith_cam);
	void compute_grey_code(std::vector<QImage*>& imgs, float* proj_map_float, float min_BW_diff, float max_BW_diff);

	void load_images(int ith_cam, std::vector<QImage*>& imgs);


};


}

#endif