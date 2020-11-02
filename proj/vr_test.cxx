#include <cgv/base/node.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/gui/event_handler.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/frame_buffer.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <random>
///@ingroup VR
///@{

//
//#define __STDC_CONSTANT_MACROS
//extern "C" {
//#include "libavcodec/avcodec.h"
//#include "libavformat/avformat.h"
//#include "libavfilter/avfilter.h"
//}

//#include "opencv2/opencv.hpp"
//#include "opencv2/highgui/highgui.hpp"
//using namespace cv;

//
//#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include "intersection.h"
#include <cgv\media\illum\surface_material.cxx>

using namespace cgv::math;
using namespace cgv::media::illum;

// different interaction states for the controllers
enum InteractionState
{
	IS_NONE,
	IS_OVER,
	IS_GRAB
};
//
//char* avformatinfo() {
//
//	char* info = (char*)malloc(40000);
//	memset(info, 0, 40000);
//
//	av_register_all();
//
//	AVInputFormat* if_temp = av_iformat_next(NULL);
//	AVOutputFormat* of_temp = av_oformat_next(NULL);
//	//Input
//	while (if_temp != NULL) {
//		sprintf(info, "%s[In ] %10s\n", info, if_temp->name);
//		if_temp = if_temp->next;
//	}
//	//Output
//	while (of_temp != NULL) {
//		sprintf(info, "%s[Out] %10s\n", info, of_temp->name);
//		of_temp = of_temp->next;
//	}
//	return info;
//}

class SceneObject : public cgv::render::render_types {
public:
	// explain the ids in the rendering phrase, use switch case 
	/* e.g.
		0 - cube
		1 - cylinder
		2 - sphere

	*/
	int obj_id;
	int obj_resolution;
	// or other para. deps on id. 
	vec3 position;
	vec3 scale;
	vec3 orientation;
	rgb color;
	cgv::render::shader_program* obj_prog;
	// or, more para. 

	/// construct func. 
	SceneObject(int id, int res, vec3 posi, vec3 scal, vec3 ori, rgb col) {
		obj_id = id;
		obj_resolution = res;
		position = posi;
		scale = scal;
		orientation = ori;
		color = col;
	}
};

class Scene {
public:
	std::vector<SceneObject*> list_of_objs;
	// lightings, cams possible 
};

/// the plugin class vr_test inherits like other plugins from node, drawable and provider
class vr_test :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
protected:
	// store the scene as colored boxes
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;

	// rendering style for boxes
	cgv::render::box_render_style style;


	// sample for rendering a mesh
	double mesh_scale;
	dvec3 mesh_location;
	dquat mesh_orientation;

	// render information for mesh
	cgv::render::mesh_render_info MI;


	// sample for rendering text labels
	std::string label_text;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;

	bool label_outofdate; // whether label texture is out of date
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label

	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;
	
	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type;


	// keep deadzone and precision vector for left controller
	cgv::gui::vr_server::vec_flt_flt left_deadzone_and_precision;
	// store handle to vr kit of which left deadzone and precision is configured
	void* last_kit_handle;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

	// store the movable boxes
	std::vector<box3> movable_boxes;
	std::vector<rgb> movable_box_colors;
	std::vector<vec3> movable_box_translations;
	std::vector<quat> movable_box_rotations;

	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;

	// state of current interaction with boxes for each controller
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;

	std::vector<vec3> pointlist;
	std::vector<vec3> splist;
	std::vector<vec3> eplist;
	bool is_even_point = false; // start from -1

	cgv::render::shader_program cube_prog;
	SceneObject* baseObject;

	cgv::render::texture img_tex;
	cgv::render::shader_program skyprog;

	unsigned int VBO, VAO;
	unsigned int cube_VBO, cube_VAO;
	cgv::render::shader_code vs, fs, ls;
	cgv::render::shader_program myprog;

	cgv::render::texture img_tex_floor;
	cgv::render::shader_program img_prog;
	// add- private vars.

	// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
	{
		for (size_t i = 0; i < movable_boxes.size(); ++i) {
			vec3 origin_box_i = origin - movable_box_translations[i];
			movable_box_rotations[i].inverse_rotate(origin_box_i);
			vec3 direction_box_i = direction;
			movable_box_rotations[i].inverse_rotate(direction_box_i);
			float t_result;
			vec3  p_result;
			vec3  n_result;
			if (cgv::media::ray_axis_aligned_box_intersection(
				origin_box_i, direction_box_i,
				movable_boxes[i],
				t_result, p_result, n_result, 0.000001f)) {

				// transform result back to world coordinates
				movable_box_rotations[i].rotate(p_result);
				p_result += movable_box_translations[i];
				movable_box_rotations[i].rotate(n_result);

				// store intersection information
				intersection_points.push_back(p_result);
				intersection_colors.push_back(color);
				intersection_box_indices.push_back((int)i);
				intersection_controller_indices.push_back(ci);
			}
		}
	}
	/// register on device change events
	void on_device_change(void* kit_handle, bool attach)
	{
		if (attach) {
			if (last_kit_handle == 0) {
				vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
				if (kit_ptr) {
					last_kit_handle = kit_handle;
					left_deadzone_and_precision = kit_ptr->get_controller_throttles_and_sticks_deadzone_and_precision(0);
					cgv::gui::ref_vr_server().provide_controller_throttles_and_sticks_deadzone_and_precision(kit_handle, 0, &left_deadzone_and_precision);
					post_recreate_gui();
				}
			}
		}
		else {
			if (kit_handle == last_kit_handle) {
				last_kit_handle = 0;
				post_recreate_gui();
			}
		}
	}
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_table(float tw, float td, float th, float tW);
	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
	/// construct boxes for environment
	void construct_environment(float s, float ew, float ed, float eh, float w, float d, float h);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_movable_boxes(float tw, float td, float th, float tW, size_t nr);
	/// construct a scene with a table
	void build_scene(float w, float d, float h, float W,
		float tw, float td, float th, float tW)
	{
		//construct_room(w, d, h, W, false, false);
		//construct_table(tw, td, th, tW);
		construct_environment(0.2f, w + 2 * 4 * 0.2f, d + 2 * 4 * 0.2f, h, w, d, h);
		construct_movable_boxes(tw, td, th, tW, 20);
		//construct_scene_from_json();
	}
	// add- private func.
public:
	vr_test():img_tex("uint8[R,G,B]", cgv::render::TextureFilter::TF_NEAREST, cgv::render::TextureFilter::TF_NEAREST),
		img_tex_floor("uint8[R,G,B]", cgv::render::TextureFilter::TF_NEAREST, cgv::render::TextureFilter::TF_NEAREST)
	{
		set_name("vr_test");
		build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, 0.9f, 0.03f);
		vr_view_ptr = 0;
		ray_length = 2;
		last_kit_handle = 0;
		connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_test::on_device_change);

		mesh_scale = 2;// 0.001f;
		mesh_location = dvec3(0, 1.1f, 0);
		mesh_orientation = dquat(vec3(0,1,0),var1);

		srs.radius = 0.005f;

		label_outofdate = true;
		label_text = "Info Board";
		label_font_idx = 0;
		label_upright = true;
		label_face_type = cgv::media::font::FFA_BOLD;
		label_resolution = 256;
		label_size = 20.0f;
		label_color = rgb(1, 1, 1);

		cgv::media::font::enumerate_font_names(font_names);
		font_enum_decl = "enums='";
		for (unsigned i = 0; i < font_names.size(); ++i) {
			if (i>0)
				font_enum_decl += ";";
			std::string fn(font_names[i]);
			if (cgv::utils::to_lower(fn) == "calibri") {
				label_font_face = cgv::media::font::find_font(fn)->get_font_face(label_face_type);
				label_font_idx = i;
			}
			font_enum_decl += std::string(fn);
		}
		font_enum_decl += "'";
		state[0] = state[1] = state[2] = state[3] = IS_NONE;

		pointlist.push_back(fvec<double, 3>(0, 1, 0));
		pointlist.push_back(fvec<double, 3>(0, 2, 0));
		pointlist.push_back(fvec<double, 3>(0, 3, 0));
		pointlist.push_back(fvec<double, 3>(0, 4, 0));
		pointlist.push_back(fvec<double, 3>(0, 5, 0));

		/*char* infostr = NULL;
		infostr = avformatinfo();
		printf("\n<<AVFormat>>\n%s", infostr);
		free(infostr);*/

		//using namespace cv;
		//Mat output = Mat::zeros(120, 350, CV_8UC3);

		////write text on the matrix:
		//putText(output,
		//	"Hello World :)",
		//	cvPoint(15, 70),
		//	FONT_HERSHEY_PLAIN,
		//	3,
		//	cvScalar(0, 255, 0),
		//	4);

		////display the image:
		//imshow("Output", output);

		//char* infostr = NULL;
		//infostr = avformatinfo();
		//printf("\n<<AVFormat>>\n%s", infostr);
		//free(infostr);

		//typedef OpenMesh::PolyMesh_ArrayKernelT<>  MyMesh;
		//MyMesh mesh;
		//// generate vertices
		//MyMesh::VertexHandle vhandle[8];
		//vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, -1, 1));
		//vhandle[1] = mesh.add_vertex(MyMesh::Point(1, -1, 1));
		//vhandle[2] = mesh.add_vertex(MyMesh::Point(1, 1, 1));
		//vhandle[3] = mesh.add_vertex(MyMesh::Point(-1, 1, 1));
		//vhandle[4] = mesh.add_vertex(MyMesh::Point(-1, -1, -1));
		//vhandle[5] = mesh.add_vertex(MyMesh::Point(1, -1, -1));
		//vhandle[6] = mesh.add_vertex(MyMesh::Point(1, 1, -1));
		//vhandle[7] = mesh.add_vertex(MyMesh::Point(-1, 1, -1));
		//// generate (quadrilateral) faces
		//std::vector<MyMesh::VertexHandle>  face_vhandles;
		//face_vhandles.clear();
		//face_vhandles.push_back(vhandle[0]);
		//face_vhandles.push_back(vhandle[1]);
		//face_vhandles.push_back(vhandle[2]);
		//face_vhandles.push_back(vhandle[3]);
		//mesh.add_face(face_vhandles);

		//face_vhandles.clear();
		//face_vhandles.push_back(vhandle[7]);
		//face_vhandles.push_back(vhandle[6]);
		//face_vhandles.push_back(vhandle[5]);
		//face_vhandles.push_back(vhandle[4]);
		//mesh.add_face(face_vhandles);
		//face_vhandles.clear();
		//face_vhandles.push_back(vhandle[1]);
		//face_vhandles.push_back(vhandle[0]);
		//face_vhandles.push_back(vhandle[4]);
		//face_vhandles.push_back(vhandle[5]);
		//mesh.add_face(face_vhandles);
		//face_vhandles.clear();
		//face_vhandles.push_back(vhandle[2]);
		//face_vhandles.push_back(vhandle[1]);
		//face_vhandles.push_back(vhandle[5]);
		//face_vhandles.push_back(vhandle[6]);
		//mesh.add_face(face_vhandles);
		//face_vhandles.clear();
		//face_vhandles.push_back(vhandle[3]);
		//face_vhandles.push_back(vhandle[2]);
		//face_vhandles.push_back(vhandle[6]);
		//face_vhandles.push_back(vhandle[7]);
		//mesh.add_face(face_vhandles);
		//face_vhandles.clear();
		//face_vhandles.push_back(vhandle[0]);
		//face_vhandles.push_back(vhandle[3]);
		//face_vhandles.push_back(vhandle[7]);
		//face_vhandles.push_back(vhandle[4]);
		//mesh.add_face(face_vhandles);

		//OpenMesh::IO::write_mesh(mesh, "output.off");

	}
	std::string get_type_name() const
	{
		return "vr_test";
	}
	int var1 = 0;
	Scene* iscene = new Scene();
	void construct_scene_from_json() {
		//// can read those from json file 
		/*SceneObject* so0 = new SceneObject(0, 10, vec3(0, 0, 0), vec3(1, 0.1, 1), vec3(1, 1, 1), rgb(1, 0, 0));
		SceneObject* so1 = new SceneObject(0, 10, vec3(2.1, 0, 0), vec3(1, 0.1, 1), vec3(1, 1, 1), rgb(1, 0, 0));
		SceneObject* so2 = new SceneObject(0, 10, vec3(4.2, 0, 0), vec3(1, 0.1, 1), vec3(1, 1, 1), rgb(1, 0, 0));
		SceneObject* so3 = new SceneObject(0, 10, vec3(6.3, 0, 0), vec3(1, 0.1, 1), vec3(1, 1, 1), rgb(1, 0, 0));*/
		// add some object para. here 
		/*iscene->list_of_objs.push_back(so0);
		iscene->list_of_objs.push_back(so1);
		iscene->list_of_objs.push_back(so2);
		iscene->list_of_objs.push_back(so3);*/

		// gen. a quad with some inter space
		/*SceneObject* stmp;
		for (int i = 0; i < 10; i++) {
			for (int j = 0; j < 10; j++) {
				stmp = new SceneObject(0, 10, vec3(i * 2.1f, 0, j * 2.1f), vec3(1, 0.1, 1), vec3(1, 1, 1), rgb(1, 0, 0));
				iscene->list_of_objs.push_back(stmp);
			}
		}*/

		// setting up mesh1 and add to the scene 
		// gen. grid with given obj file 
		SceneObject* stmp;
		//for (int i = 0; i < 10; i++) {
		//	for (int j = 0; j < 10; j++) {
		//		stmp = new SceneObject(-1, 10, vec3(i * 2.1f, 0, j * 2.1f), vec3(2, 2, 2), vec3(1, 1, 1), rgb(1, 0, 0));
		//		stmp->obj_prog = &cube_prog;
		//		iscene->list_of_objs.push_back(stmp);
		//	}
		//}

		int N = 10; // from https://www.redblobgames.com/grids/hexagons/#range
		float scale_h = 1;
		std::default_random_engine generator;
		std::uniform_real_distribution<float> distribution(0.5, 1);
		for (int q = -N; q <= N; q++) {
			int r1 = std::max(-N, -q - N);
			int r2 = std::min(N, -q + N);
			for (int r = r1; r <= r2; r++) {
				scale_h = 1;
				if (q == N || q == -N || r == r1 || r == r2) // border case 
					scale_h = 80 * distribution(generator);
				if (q == N -1  || q == -N + 1 || r == r1 + 1  || r == r2 -1 ) // border case 
					scale_h = 40 * distribution(generator);
				stmp = new SceneObject(-1, 10, 
					vec3(q + r / 2.0f, 0, sqrt(3.0f) / 2.0f * r), // posi
					vec3(0.95, scale_h, 0.95), // scale
					vec3(1, 1, 1), // ori.
					rgb(0.2, 0.2, 0.2)); // color(may not used)
				stmp->obj_prog = &cube_prog;
				iscene->list_of_objs.push_back(stmp);
			}
		}

		// setting up bkg and add to the scene 
		stmp = new SceneObject(-1, 10, vec3(0,-0.1,0), vec3(2 * N, 1, 2 * N), vec3(1, 1, 1), rgb(55.0f / 255.0f, 190.0f / 255.0f, 240.0f / 255.0f));
		stmp->obj_prog = &cube_prog;
		baseObject = stmp;
		//iscene->list_of_objs.push_back(stmp);

	}
	void create_gui()
	{
		add_decorator("vr_test", "heading", "level=2");
		add_member_control(this, "mesh_scale", mesh_scale, "value_slider", "min=0.1;max=10;log=true;ticks=true");
		add_member_control(this, "var1", var1, "value_slider", "min=-30;max=30;log=true;ticks=true");
		add_gui("mesh_location", mesh_location, "vector", "options='min=-3;max=3;ticks=true");
		add_gui("mesh_orientation", static_cast<dvec4&>(mesh_orientation), "direction", "options='min=-1;max=1;ticks=true");
		add_member_control(this, "ray_length", ray_length, "value_slider", "min=0.1;max=10;log=true;ticks=true");
		if (last_kit_handle) {
			vr::vr_kit* kit_ptr = vr::get_vr_kit(last_kit_handle);
			const std::vector<std::pair<int, int> >* t_and_s_ptr = 0;
			if (kit_ptr)
				t_and_s_ptr = &kit_ptr->get_controller_throttles_and_sticks(0);
			add_decorator("deadzone and precisions", "heading", "level=3");
			int ti = 0;
			int si = 0;
			for (unsigned i = 0; i < left_deadzone_and_precision.size(); ++i) {
				std::string prefix = std::string("unknown[") + cgv::utils::to_string(i) + "]";
				if (t_and_s_ptr) {
					if (t_and_s_ptr->at(i).second == -1)
						prefix = std::string("throttle[") + cgv::utils::to_string(ti++) + "]";
					else
						prefix = std::string("stick[") + cgv::utils::to_string(si++) + "]";
				}
				add_member_control(this, prefix + ".deadzone", left_deadzone_and_precision[i].first, "value_slider", "min=0;max=1;ticks=true;log=true");
				add_member_control(this, prefix + ".precision", left_deadzone_and_precision[i].second, "value_slider", "min=0;max=1;ticks=true;log=true");
			}
		}
		if (begin_tree_node("box style", style)) {
			align("\a");
			add_gui("box style", style);
			align("\b");
			end_tree_node(style);
		}
		if (begin_tree_node("movable box style", movable_style)) {
			align("\a");
			add_gui("movable box style", movable_style);
			align("\b");
			end_tree_node(movable_style);
		}
		if (begin_tree_node("intersections", srs)) {
			align("\a");
			add_gui("sphere style", srs);
			align("\b");
			end_tree_node(srs);
		}
		if (begin_tree_node("mesh", mesh_scale)) {
			align("\a");
			add_member_control(this, "scale", mesh_scale, "value_slider", "min=0.0001;step=0.0000001;max=100;log=true;ticks=true");
			add_gui("location", mesh_location, "", "main_label='';long_label=true;gui_type='value_slider';options='min=-2;max=2;step=0.001;ticks=true'");
			add_gui("orientation", static_cast<dvec4&>(mesh_orientation), "direction", "main_label='';long_label=true;gui_type='value_slider';options='min=-1;max=1;step=0.001;ticks=true'");
			align("\b");
			end_tree_node(mesh_scale);
		}

		if (begin_tree_node("label", label_size)) {
			align("\a");
			add_member_control(this, "text", label_text);
			add_member_control(this, "upright", label_upright);
			add_member_control(this, "font", (cgv::type::DummyEnum&)label_font_idx, "dropdown", font_enum_decl);
			add_member_control(this, "face", (cgv::type::DummyEnum&)label_face_type, "dropdown", "enums='regular,bold,italics,bold+italics'");
			add_member_control(this, "size", label_size, "value_slider", "min=8;max=64;ticks=true");
			add_member_control(this, "color", label_color);
			add_member_control(this, "resolution", (cgv::type::DummyEnum&)label_resolution, "dropdown", "enums='256=256,512=512,1024=1024,2048=2048'");
			align("\b");
			end_tree_node(label_size);
		}
	}
	void on_set(void* member_ptr)
	{
		if (member_ptr == &label_face_type || member_ptr == &label_font_idx) {
			label_font_face = cgv::media::font::find_font(font_names[label_font_idx])->get_font_face(label_face_type);
			label_outofdate = true;
		}
		if ((member_ptr >= &label_color && member_ptr < &label_color + 1) ||
			member_ptr == &label_size || member_ptr == &label_text) {
			label_outofdate = true;
		}
		update_member(member_ptr);
		post_redraw();
	}
	void stream_help(std::ostream& os)
	{
		os << "vr_test: no shortcuts defined" << std::endl;
	}
	bool keydown = false;
	vec3 cur_right_hand_posi = vec3(0,0,0);
	bool handle(cgv::gui::event& e)
	{
		
		// check if vr event flag is not set and don't process events in this case
		if ((e.get_flags() & cgv::gui::EF_VR) == 0)
			return false;
		// check event id
		switch (e.get_kind()) {
		case cgv::gui::EID_KEY:
		{
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			if (vrke.get_action() != cgv::gui::KA_RELEASE) {
				switch (vrke.get_key()) {
					case vr::VR_LEFT_BUTTON0:
						std::cout << "button 0 of left controller pressed" << std::endl;
						return true;
					case vr::VR_RIGHT_BUTTON0:
						std::cout << "---button 0 of right controller pressed" << std::endl;
						keydown = true;
						is_even_point = !is_even_point;
						//vec3 pos;
						//vrke.get_device_handle
						//std::cout << "position: "<< pos << std::endl;
						//pointlist.push_back(pos);

						return true;
					case vr::VR_RIGHT_STICK_RIGHT:
						std::cout << "touch pad of right controller pressed at right direction" << std::endl;
						return true;
					}
			}
			break;
		}
		case cgv::gui::EID_THROTTLE:
		{
			cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
			std::cout << "throttle " << vrte.get_throttle_index() << " of controller " << vrte.get_controller_index()
				<< " adjusted from " << vrte.get_last_value() << " to " << vrte.get_value() << std::endl;
			return true;
		}
		case cgv::gui::EID_STICK:
		{
			cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
			switch (vrse.get_action()) {
			case cgv::gui::SA_TOUCH:
				if (state[vrse.get_controller_index()] == IS_OVER)
					state[vrse.get_controller_index()] = IS_GRAB;
				break;
			case cgv::gui::SA_RELEASE:
				if (state[vrse.get_controller_index()] == IS_GRAB)
					state[vrse.get_controller_index()] = IS_OVER;
				break;
			case cgv::gui::SA_PRESS:
			case cgv::gui::SA_UNPRESS:
				std::cout << "stick " << vrse.get_stick_index()
					<< " of controller " << vrse.get_controller_index()
					<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
					<< " at " << vrse.get_x() << ", " << vrse.get_y() << std::endl;
				return true;
			case cgv::gui::SA_MOVE:
			case cgv::gui::SA_DRAG:
				std::cout << "stick " << vrse.get_stick_index()
					<< " of controller " << vrse.get_controller_index()
					<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
					<< " from " << vrse.get_last_x() << ", " << vrse.get_last_y()
					<< " to " << vrse.get_x() << ", " << vrse.get_y() << std::endl;
				return true;
			}
			return true;
		}

		case cgv::gui::EID_POSE:
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			// check for controller pose events
			int ci = vrpe.get_trackable_index();
			if (keydown) {
				vec3 origin, direction;
				vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));
				vec3 headpos = vrpe.get_position();
				std::cout << "---current controller posi :" << origin << std::endl;
				pointlist.push_back(origin);
				if (is_even_point) {
					splist.push_back(origin);
				}
				else {
					eplist.push_back(origin);
				}
					
				post_redraw();
				keydown = false;
			}
			if (ci != -1) {
				if (state[ci] == IS_GRAB) {
					// in grab mode apply relative transformation to grabbed boxes

					// get previous and current controller position
					vec3 last_pos = vrpe.get_last_position();
					vec3 pos = vrpe.get_position();
					// get rotation from previous to current orientation
					// this is the current orientation matrix times the
					// inverse (or transpose) of last orientation matrix:
					// vrpe.get_orientation()*transpose(vrpe.get_last_orientation())
					mat3 rotation = vrpe.get_rotation_matrix();
					// iterate intersection points of current controller
					for (size_t i = 0; i < intersection_points.size(); ++i) {
						if (intersection_controller_indices[i] != ci)
							continue;
						// extract box index
						unsigned bi = intersection_box_indices[i];
						// update translation with position change and rotation
						movable_box_translations[bi] = 
							rotation * (movable_box_translations[bi] - last_pos) + pos;
						// update orientation with rotation, note that quaternions
						// need to be multiplied in oposite order. In case of matrices
						// one would write box_orientation_matrix *= rotation
						movable_box_rotations[bi] = quat(rotation) * movable_box_rotations[bi];
						// update intersection points
						intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;
					}
				}
				else {// not grab
					// clear intersections of current controller 
					size_t i = 0;
					while (i < intersection_points.size()) {
						if (intersection_controller_indices[i] == ci) {
							intersection_points.erase(intersection_points.begin() + i);
							intersection_colors.erase(intersection_colors.begin() + i);
							intersection_box_indices.erase(intersection_box_indices.begin() + i);
							intersection_controller_indices.erase(intersection_controller_indices.begin() + i);
						}
						else
							++i;
					}

					// compute intersections
					vec3 origin, direction;
					vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
					compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					label_outofdate = true;

					// mark cur. posi as global var.
					cur_right_hand_posi = origin;

					// update state based on whether we have found at least 
					// one intersection with controller ray
					if (intersection_points.size() == i)
						state[ci] = IS_NONE;
					else
						if (state[ci] == IS_NONE)
							state[ci] = IS_OVER;
				}
				post_redraw();
			}
			return true;
		}
		return false;
	}
	bool init(cgv::render::context& ctx)
	{
		if (!cgv::utils::has_option("NO_OPENVR"))
			ctx.set_gamma(1.0f);
		cgv::media::mesh::simple_mesh<> M;
		if (M.read("D:/smalldataset/obj/gen_with_unity/s2_floor.obj")) {
		//if (M.read("D:/smalldataset/obj/human/dc.obj")) {
			MI.construct_vbos(ctx, M);
			MI.bind(ctx, ctx.ref_surface_shader_program(true));
		}
		cgv::gui::connect_vr_server(true);

		auto view_ptr = find_view_as_node();
		if (view_ptr) {
			view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
			// if the view points to a vr_view_interactor
			vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
			if (vr_view_ptr) {
				// configure vr event processing
				vr_view_ptr->set_event_type_flags(
					cgv::gui::VREventTypeFlags(
						cgv::gui::VRE_KEY +
						cgv::gui::VRE_THROTTLE +
						cgv::gui::VRE_STICK +
						cgv::gui::VRE_STICK_KEY +
						cgv::gui::VRE_POSE
					));
				vr_view_ptr->enable_vr_event_debugging(false);
				// configure vr rendering
				vr_view_ptr->draw_action_zone(false);
				vr_view_ptr->draw_vr_kits(true);
				vr_view_ptr->enable_blit_vr_views(true);
				vr_view_ptr->set_blit_vr_view_width(200);

			}
		}
		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_sphere_renderer(ctx, 1);

		cube_prog.build_program(ctx, "color_cube.glpr");

		skyprog.build_program(ctx, "skycube.glpr");
		img_tex.create_from_images(ctx,
			"D:/zrtechdev-all/zr_GFPV_cgv/plugins/myworks/easyLearningVR/proj/cm_{xp,xn,yp,yn,zp,zn}.jpg"
			//"D:/newmodels/3Skyboxes/Skybox_BluePinkNebular_Textures/BluePinkNebular_{xp,xn,yp,yn,zp,zn}.jpg"
			//"D:/newmodels/sky/sky01_sum/sky_{xp,xn,yp,yn,zp,zn}.png"
		);

		img_prog.build_program(ctx, "image.glpr");

		img_tex_floor.create_from_image(ctx, "D:/newmodels/floortex/3.jpg");

		//use my shder 
		/*if (!vs.read_and_compile(ctx, "ishader.glvs")) {
			std::cout << "error reading vertex shader\n" << vs.last_error.c_str() << std::endl;
			return false;
		}
		if (!fs.read_and_compile(ctx, "ishader.glfs")) {
			std::cout << "error reading fragment shader\n" << vs.last_error.c_str() << std::endl;
			return false;
		}
		if (!myprog.create(ctx)) {
			std::cout << "error creating program\n" << myprog.last_error.c_str() << std::endl;
			return false;
		}
		myprog.attach_code(ctx, vs);
		myprog.attach_code(ctx, fs);
		if (!myprog.link(ctx)) {
			std::cout << "link error\n" << myprog.last_error.c_str() << std::endl;
			return false;
		}*/
		myprog.build_program(ctx, "ishader.glpr");

		// set up vertex data (and buffer(s)) and configure vertex attributes
		// ------------------------------------------------------------------
		float vertices[] = {
			// positions         // colors
			 0.5f, -0.5f, 0.0f,  1.0f, 0.0f, 0.0f,  // bottom right
			-0.5f, -0.5f, 0.0f,  0.0f, 1.0f, 0.0f,  // bottom left
			 0.0f,  0.5f, 0.0f,  0.0f, 0.0f, 1.0f   // top 

		};

		/// for a cube, color only 
			float colors_for_cube[] = {
				1.0f, 0.0f, 0.0f, 1.0f,  
				0.0f, 1.0f, 0.0f, 1.0f,
				0.0f, 0.5f, 0.0f, 1.0f,
				0.0f, 0.0f, 1.0f, 1.0f,
				0.0f, 0.5f, 0.0f, 1.0f,
				0.0f, 0.5f, 0.0f, 1.0f,
				0.0f, 0.5f, 0.0f, 1.0f,
				0.0f, 0.5f, 0.0f, 1.0f

			};
			glGenVertexArrays(1, &cube_VAO);
			glGenBuffers(1, &cube_VBO);
			// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
			glBindVertexArray(cube_VAO);
			glBindBuffer(GL_ARRAY_BUFFER, cube_VBO);
			glBufferData(GL_ARRAY_BUFFER, sizeof(colors_for_cube), colors_for_cube, GL_STATIC_DRAW);
			// color attribute
			glVertexAttribPointer(3, 8, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(3);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);


		/// for the triangle 
			glGenVertexArrays(1, &VAO);
			glGenBuffers(1, &VBO);
			// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
			glBindVertexArray(VAO);

			glBindBuffer(GL_ARRAY_BUFFER, VBO);
			glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

			// position attribute
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);
			// color attribute
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(1);

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);

		//t_ptr->create_from_image(ctx, "res://alhambra.png");
		
		//Mat output = Mat::zeros(120, 350, CV_8UC3);

		// add- initialization 
		return true;
	}
	void clear(cgv::render::context& ctx)
	{
		cgv::render::ref_box_renderer(ctx, -1);
		cgv::render::ref_sphere_renderer(ctx, -1);
	}
	void init_frame(cgv::render::context& ctx)
	{
		if (label_fbo.get_width() != label_resolution) {
			label_tex.destruct(ctx);
			label_fbo.destruct(ctx);
		}
		if (!label_fbo.is_created()) {
			label_tex.create(ctx, cgv::render::TT_2D, label_resolution, label_resolution);
			label_fbo.create(ctx, label_resolution, label_resolution);
			label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
			label_tex.set_mag_filter(cgv::render::TF_LINEAR);
			label_fbo.attach(ctx, label_tex);
			label_outofdate = true;
		}
		if (label_outofdate && label_fbo.is_complete(ctx)) {
			glPushAttrib(GL_COLOR_BUFFER_BIT);
			label_fbo.enable(ctx);
			label_fbo.push_viewport(ctx);
			ctx.push_pixel_coords();
				glClearColor(0.5f,0.5f,0.5f,1.0f);
				glClear(GL_COLOR_BUFFER_BIT);

				glColor4f(label_color[0], label_color[1], label_color[2], 1);
				ctx.set_cursor(20, (int)ceil(label_size) + 20);
				ctx.enable_font_face(label_font_face, label_size);
				ctx.output_stream() << label_text << "\n";
				ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face

				ctx.enable_font_face(label_font_face, 0.7f*label_size);
				for (size_t i = 0; i < intersection_points.size(); ++i) {
					ctx.output_stream()
						<< "box " << intersection_box_indices[i]
						<< " at (" << intersection_points[i]
						<< ") with controller " << intersection_controller_indices[i] << "\n";
				}
				ctx.output_stream().flush();

			ctx.pop_pixel_coords();
			label_fbo.pop_viewport(ctx);
			label_fbo.disable(ctx);
			glPopAttrib();
			label_outofdate = false;

			label_tex.generate_mipmaps(ctx);
		}
	}
	void draw_axes(cgv::render::context& ctx, bool transformed)
	{
		float c = transformed ? 0.7f : 1;
		float d = 1 - c;
		float l = 1;
		ctx.set_color(rgb(c, d, d));
		ctx.tesselate_arrow(cgv::math::fvec<double, 3>(0, 0, 0), cgv::math::fvec<double, 3>(l, 0, 0), 0.02);
		ctx.set_color(rgb(d, c, d));
		ctx.tesselate_arrow(cgv::math::fvec<double, 3>(0, 0, 0), cgv::math::fvec<double, 3>(0, l, 0), 0.02);
		ctx.set_color(rgb(d, d, c));
		ctx.tesselate_arrow(cgv::math::fvec<double, 3>(0, 0, 0), cgv::math::fvec<double, 3>(0, 0, l), 0.02);
	}
	surface_material cube_mat = surface_material(BT_OREN_NAYAR, surface_material::color_type(0.2f, 0.2f, 0.2f), 0.5f);
	void draw(cgv::render::context& ctx) // add- draw call 
	{
		// a skybox shall be rendered, working along with the scene above 
		/*
		
		*/
		//	glDepthMask(GL_FALSE);
		//	glDisable(GL_CULL_FACE);
		//	// enable textures in different texture units
		//	//glDepthFunc(GL_LEQUAL);
		//	img_tex.enable(ctx, 1);
		//		skyprog.enable(ctx);
		//		ctx.set_color(rgb(1.0f));
		//		skyprog.set_uniform(ctx, "img_tex", 1);
		//		ctx.tesselate_unit_cube();
		//		skyprog.disable(ctx);
		//	img_tex.disable(ctx);
		//	//glDepthFunc(GL_LESS);
		//	glDepthMask(GL_TRUE);

		//// floor
		//	img_tex_floor.enable(ctx, 0);
		//		img_prog.enable(ctx);
		//		img_prog.set_uniform(ctx, "image", 0);
		//		ctx.push_modelview_matrix();
		//			ctx.mul_modelview_matrix(scale4<double>(vec3(2.5, 1, 3.5) )* rotate4<double>(90, vec3(1,0,0)) );
		//			ctx.tesselate_unit_square();
		//		ctx.pop_modelview_matrix();
		//		img_prog.disable(ctx);
		//	img_tex_floor.disable(ctx);

		// render the mesh 
		/*if (MI.is_constructed()) {
			
			ctx.set_material(cube_mat);
			MI.clear_material_group();
			dmat4 R;
			mesh_orientation.put_homogeneous_matrix(R);
			ctx.push_modelview_matrix();
			ctx.mul_modelview_matrix(
				cgv::math::translate4<double>(mesh_location)*
				cgv::math::scale4<double>(mesh_scale, mesh_scale, mesh_scale) *
				R);
			MI.render_mesh(ctx, ctx.ref_surface_shader_program(true));
			ctx.pop_modelview_matrix();
		}*/

		//the vr_view_ptr
		/* 
		if (vr_view_ptr) {
			std::vector<vec3> P;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 4; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P.push_back(ray_origin);
					P.push_back(ray_origin + ray_length * ray_direction);
					rgb c(float(1 - ci), 0.5f*(int)state[ci], float(ci));
					C.push_back(c);
					C.push_back(c);
				}
			}
			if (P.size() > 0) {
				cgv::render::shader_program& prog = ctx.ref_default_shader_program();
				int pi = prog.get_position_index();
				int ci = prog.get_color_index();
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
				cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
				cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
				glLineWidth(3);
				prog.enable(ctx);
				glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
				prog.disable(ctx);
				cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
				glLineWidth(1);
			}
		}
		*/
		
		// draw static and dynamic boxes 
		
			// draw static boxes
			cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
			renderer.set_render_style(style);
			renderer.set_box_array(ctx, boxes);
			renderer.set_color_array(ctx, box_colors);
			if (renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
			}
			renderer.disable(ctx);

			// draw dynamic boxes 
			/*renderer.set_render_style(movable_style);
			renderer.set_box_array(ctx, movable_boxes);
			renderer.set_color_array(ctx, movable_box_colors);
			renderer.set_translation_array(ctx, movable_box_translations);
			renderer.set_rotation_array(ctx, movable_box_rotations);
			if (renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)movable_boxes.size());
			}
			renderer.disable(ctx);*/
		
			
		
		
		// draw intersection points and label 
		/*
		if (!intersection_points.empty()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_position_array(ctx, intersection_points);
			sr.set_color_array(ctx, intersection_colors);
			sr.set_render_style(srs);
			if (sr.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)intersection_points.size());
				sr.disable(ctx);
			}
		}

		// draw label
		if (label_tex.is_created()) {
			cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
			int pi = prog.get_position_index();
			int ti = prog.get_texcoord_index();
			vec3 p(0, 1.5f, 0);
			vec3 y = label_upright ? vec3(0, 1.0f, 0) : normalize(vr_view_ptr->get_view_up_dir_of_kit());
			vec3 x = normalize(cross(vec3(vr_view_ptr->get_view_dir_of_kit()), y));
			float w = 0.5f, h = 0.5f;
			std::vector<vec3> P;
			std::vector<vec2> T;
			P.push_back(p - 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(0.0f, 0.0f));
			P.push_back(p + 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(1.0f, 0.0f));
			P.push_back(p - 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(0.0f, 1.0f));
			P.push_back(p + 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(1.0f, 1.0f));
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
			cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
			cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
			prog.enable(ctx);
			label_tex.enable(ctx);
			ctx.set_color(rgb(1, 1, 1));
			glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
			label_tex.disable(ctx);
			prog.disable(ctx);
			cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
		}
		*/
		
		// construct sth. for test @yzy
		//surface_material cube_mat = surface_material(BT_OREN_NAYAR, surface_material::color_type(0.9f, 0.2f, 0.2f), 0.5f);
		//ctx.ref_surface_shader_program().enable(ctx);
		//	ctx.ref_surface_shader_program().set_uniform(ctx, "map_color_to_material", 0);
		//	//glEnable(GL_POLYGON_OFFSET_FILL);
		//		//ctx.set_color(rgb(0, 1, 0));
		//		//glPolygonOffset(1,0);		
		//	//ctx.set_material(cube_mat);
		//	//ctx.tesselate_unit_cube();
		//	ctx.push_modelview_matrix();
		//		glEnable(GL_BLEND);
		//		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//		/*ctx.mul_modelview_matrix(translate4<double>(pointlist.at(i).x(), pointlist.at(i).y(), pointlist.at(i).z()));
		//		ctx.mul_modelview_matrix(scale4<double>(s, s, s));*/
		//		//ctx.tesselate_unit_sphere();
		//		//ctx.tesselate_unit_torus();
		//		//ctx.tesselate_unit_octahedron(); // use gui to choose 
		//		ctx.tesselate_unit_cube();
		//		glDisable(GL_BLEND);
		//	ctx.pop_modelview_matrix();
		//	//glDisable(GL_POLYGON_OFFSET_FILL)
		//ctx.ref_surface_shader_program().disable(ctx);
		/*
			double s = 0.1;
			surface_material cube_mat = surface_material(BT_OREN_NAYAR, surface_material::color_type(0.6f, 0.5f, 0.4f), 0.5f);
			if (eplist.size() > 0)
			for (int i = 0; i < splist.size() - 1; i++) {
				//ctx.ref_surface_shader_program().enable(ctx);
				//	ctx.ref_surface_shader_program().set_uniform(ctx, "map_color_to_material", 0);
				//	//glEnable(GL_POLYGON_OFFSET_FILL);
				//		//ctx.set_color(rgb(0, 1, 0));
				//		//glPolygonOffset(1,0);		
				//	ctx.set_material(cube_mat);
				//	//ctx.tesselate_unit_cube();
				//	ctx.push_modelview_matrix();
				//		
				//		ctx.mul_modelview_matrix(translate4<double>(pointlist.at(i).x(), pointlist.at(i).y(), pointlist.at(i).z()));
				//		ctx.mul_modelview_matrix(scale4<double>(s, s, s));
				//		//ctx.tesselate_unit_sphere();
				//		//ctx.tesselate_unit_torus();
				//	ctx.tesselate_unit_octahedron(); // use gui to choose 
				//	ctx.pop_modelview_matrix();
				//	//glDisable(GL_POLYGON_OFFSET_FILL)
				//ctx.ref_surface_shader_program().disable(ctx);

				ctx.ref_surface_shader_program().enable(ctx);
					ctx.push_modelview_matrix();
					ctx.set_color(rgb(0,1,0));
					ctx.tesselate_arrow(splist.at(i), eplist.at(i), 0.02);
					ctx.pop_modelview_matrix();
				ctx.ref_surface_shader_program().disable(ctx);
			}
			if (is_even_point) { // spec. care needed 
				ctx.ref_surface_shader_program().enable(ctx);
				ctx.push_modelview_matrix();
				ctx.set_color(rgb(0, 1, 0));
				ctx.tesselate_arrow(splist.at(splist.size() - 1), cur_right_hand_posi, 0.02);
				ctx.pop_modelview_matrix();
				ctx.ref_surface_shader_program().disable(ctx);
			}
			else if (eplist.size() > 0) { // the last arrow 
				ctx.ref_surface_shader_program().enable(ctx);
				ctx.push_modelview_matrix();
				ctx.set_color(rgb(0, 1, 0));
				ctx.tesselate_arrow(splist.at(splist.size()-1), eplist.at(eplist.size() - 1), 0.02);
				ctx.pop_modelview_matrix();
				ctx.ref_surface_shader_program().disable(ctx);
			}

			ctx.ref_surface_shader_program().enable(ctx);
			ctx.push_modelview_matrix();
				ctx.tesselate_unit_cylinder(var1);
			ctx.pop_modelview_matrix();
			ctx.ref_surface_shader_program().disable(ctx);
		*/
		
		// render scene from json 
		/*
		for (int i = 0; i < iscene->list_of_objs.size(); i++) {
				switch (iscene->list_of_objs.at(i)->obj_id)
				{
				case 0: 
					// render a cube 
					ctx.ref_default_shader_program().enable(ctx);
						ctx.push_modelview_matrix();
						ctx.mul_modelview_matrix(
							translate4<double>(iscene->list_of_objs.at(i)->position)
							* scale4<double>(iscene->list_of_objs.at(i)->scale)
							//* rotate4<double>(iscene->list_of_objs.at(i)->orientation)
							);
							ctx.set_color(iscene->list_of_objs.at(i)->color);
							ctx.tesselate_unit_cube();
						ctx.pop_modelview_matrix();
					ctx.ref_default_shader_program().disable(ctx);
					break; 
				case 1:
					ctx.push_modelview_matrix();
						//ctx.mul_modelview_matrix(scale4<double>(0.4, 0.4, 0.4) * translate4<double>(-10.5, 1.2, 0));
							ctx.ref_default_shader_program().enable(ctx);
								ctx.set_color(iscene->list_of_objs.at(i)->color); //border color 
								//ctx.tesselate_unit_cube();
							ctx.ref_default_shader_program().disable(ctx);
					ctx.pop_modelview_matrix();
					break;
				case -1:
					// 1st mesh primetive, render it 
					if (MI.is_constructed()) {

						MI.clear_material_group();
						ctx.set_material(cube_mat);
						dmat4 R;
						dquat(vec3(0, 1, 0), 0).put_homogeneous_matrix(R); // tested on gui 
						ctx.push_modelview_matrix();
						ctx.mul_modelview_matrix(
							cgv::math::translate4<double>(iscene->list_of_objs.at(i)->position) *
							cgv::math::scale4<double>(iscene->list_of_objs.at(i)->scale) *
							R);
						MI.render_mesh(ctx, 
							ctx.ref_surface_shader_program(true)
							//*iscene->list_of_objs.at(i)->obj_prog
						);
						ctx.pop_modelview_matrix();
					}
					break;
				case -2:
					// the 2nd mesh file 
					break;
				// ... can dynamically be loaded later... 
				default:
					break;
				}
			}
		*/
		
		// base obj.
		/*
		MI.clear_material_group();
			ctx.set_material(cube_mat);
			dmat4 R;
			dquat(vec3(0, 1, 0), 0).put_homogeneous_matrix(R); // tested on gui 
			ctx.push_modelview_matrix();
				ctx.mul_modelview_matrix(
					cgv::math::translate4<double>(baseObject->position) *
					cgv::math::scale4<double>(baseObject->scale) *
					R);
				MI.render_mesh(ctx,
					//ctx.ref_surface_shader_program(true)
					*baseObject->obj_prog
				);
			ctx.pop_modelview_matrix();
		*/
			
		// render a cube with custom shader 
			cube_prog.enable(ctx);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				//glBindVertexArray(cube_VAO);
					dmat4 R;
					dquat(vec3(0, 1, 0), 3.14 * (45.0f / 180.0f)).put_homogeneous_matrix(R); // tested on gui 
					ctx.mul_modelview_matrix(translate4<double>(1, 1, 1) * R);
					ctx.tesselate_unit_cube(false, false); //_with_color
				//glBindVertexArray(0);
				glDisable(GL_BLEND);
			cube_prog.disable(ctx);

		// render a cube with custom 
			//cgv::render::shader_program& prog = ctx.ref_default_shader_program();
			myprog.enable(ctx);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glBindVertexArray(VAO);
					//myprog.set_uniform(ctx,"f_color",rgb(1,0,0));
					glDrawArrays(GL_TRIANGLES, 0, 3);
				glBindVertexArray(0);
				glDisable(GL_BLEND);
			myprog.disable(ctx);
	}
};

/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void vr_test::construct_table(float tw, float td, float th, float tW)
{
	// construct table
	rgb table_clr(0.3f, 0.2f, 0.0f);
	boxes.push_back(box3(
		vec3(-0.5f*tw - 2*tW, th, -0.5f*td - 2*tW), 
		vec3( 0.5f*tw + 2*tW, th + tW, 0.5f*td + 2*tW)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f*tw, 0, -0.5f*td), vec3(-0.5f*tw - tW, th, -0.5f*td - tW)));
	boxes.push_back(box3(vec3(-0.5f*tw, 0, 0.5f*td), vec3(-0.5f*tw - tW, th, 0.5f*td + tW)));
	boxes.push_back(box3(vec3(0.5f*tw, 0, -0.5f*td), vec3(0.5f*tw + tW, th, -0.5f*td - tW)));
	boxes.push_back(box3(vec3(0.5f*tw, 0, 0.5f*td), vec3(0.5f*tw + tW, th, 0.5f*td + tW)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}
/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_test::construct_room(float w, float d, float h, float W, bool walls, bool ceiling)
{
	// construct floor
	boxes.push_back(box3(vec3(-0.6f*w, -W, -0.6f*d), vec3(0.6f*w, 0, 0.6f*d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if (walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if (ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

/// construct boxes for environment
void vr_test::construct_environment(float s, float ew, float ed, float eh, float w, float d, float h)
{
	/*std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - 0.5f*ew;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - 0.5f*ed;
			if ( (x + 0.5f*s > -0.5f*w && x < 0.5f*w) && (z + 0.5f*s > -0.5f*d && z < 0.5f*d) )
				continue;
			float h = 0.2f*(std::max(abs(x)-0.5f*w,0.0f)+std::max(abs(z)-0.5f*d,0.0f))*distribution(generator)+0.1f;
			boxes.push_back(box3(vec3(x, 0, z), vec3(x+s, h, z+s)));
			box_colors.push_back(
				rgb(0.3f*distribution(generator)+0.3f, 
					0.3f*distribution(generator)+0.2f, 
					0.2f*distribution(generator)+0.1f));
		}
	}*/

	//ew = w + 2 * 4 * s;
	//ed = d + 2 * 4 * s;

	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - 0.5f * ew;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - 0.5f * ed;
			if ((x + 0.5f * s > -0.5f * w && x < 0.5f * w) && (z + 0.5f * s > -0.5f * d && z < 0.5f * d))
				continue;
			float h =0.1f + 0.2f * (std::max(abs(x) - 0.5f * w, 0.0f) + std::max(abs(z) - 0.5f * d, 0.0f)) * distribution(generator) + 0.1f;
			boxes.push_back(box3(vec3(x, 0, z), vec3(x + s, h, z + s)));
			box_colors.push_back(
				/*rgb(0.3f * distribution(generator) + 0.3f,
					0.3f * distribution(generator) + 0.3f,
					0.2f * distribution(generator) + 0.1f)*/
				rgb(0.3,0.3,0.3)
			);
		}
	}
}

/// construct boxes that can be moved around
void vr_test::construct_movable_boxes(float tw, float td, float th, float tW, size_t nr)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);
	for (size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.1f;
		extent *= std::min(tw, td)*0.2f;

		vec3 center(-0.5f*tw + x * tw, th + tW, -0.5f*td + y * td);
		movable_boxes.push_back(box3(-0.5f*extent, 0.5f*extent));
		movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}
}

#include <cgv/base/register.h>

cgv::base::object_registration<vr_test> vr_test_reg("");

///@}