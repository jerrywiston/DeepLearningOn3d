#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "Eigen/Eigen"
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <vector>
#include "skeleton3d.h"
#include "cloud.h"
#include "camera.h"
#include "shader.hpp"
#include "ICP_fusion.h"
#include "text2d.h"
#include "config.hpp"

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

static void setUniformVec3(const GLuint program, const char* name, const glm::vec3 &vec);
static glm::mat4 computeTransform(Eigen::Matrix3f&& rot, Eigen::Vector3f&& trans);
static glm::mat4 computeTranslation(vec&& trans);
static void convertToVec3(std::vector<vec> &v_in, std::vector<glm::vec3> &v_out);
static glm::vec3 vectorToVec(Vector3f&& v_in);
static MatrixXf arr_to_mat(vector <Vector3f> &v_pc);
static void get_normal_map(MatrixXf &pc, MatrixXf &nm);
static void DepthToWorld(int cx, int cy, unsigned short int cz, float &wx, float &wy, float &wz);
static MatrixXf read_pc(const char *fn);



int main(int argc, char* argv[])
{
	if(argc < 2){
		std::cout << "ERROR: Too few argument" << std::endl;
		exit(1);
	}


	//Read point cloud & normal data==============================================
	vector<MatrixXf> pc, normal;
	MatrixXf normal_tmp(3, IMG_HEIGHT*IMG_WIDTH);
	int total = atoi(argv[2]);
	char str[128];

	for (int i = 0; i < total; i++){
		//load point cloud
		std::cout << "Load " << argv[1] << "/PointCloud_" << i + 1 << " ..." << std::endl;
		sprintf(str, "%s/PointCloud_%d.txt", argv[1], i + 1);
		MatrixXf pc_mid = read_pc(str);
		pc.push_back(pc_mid);

		//load normal
		std::cout << "Load" << argv[1] << "/NormalMap_" << i + 1 << " ..." << std::endl;
		sprintf(str, "%s/NormalMap_%d.txt", argv[1], i + 1);
		MatrixXf normal_mid = read_pc(str);
		normal.push_back(normal_mid);
	}
	cout << "Load Success !" << endl << endl;


	//Init GLFW===================================================================
	std::cout << "Initializing GLFW..." << std::endl;

	if (!glfwInit()) exit(EXIT_FAILURE);
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


	//Open GLFW window============================================================
	std::cout << "Openning GLFW window..." << std::endl;

	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "MLAR ver3", NULL, NULL);
	if (!window){
		glfwTerminate();
		return EXIT_FAILURE;
	}
	glfwMakeContextCurrent(window);


	//Init ICP fusion=============================================================
	std::cout << "Initializing ICP fusion..." << std::endl;

	ICP_fusion fusion_test(ALIGN_SAMPLE, MODEL_SAMPLE, WINDOW_SIZE, ITER_LOOPS);


	//Init GLEW===================================================================
	std::cout << "Initializing GLEW..." << std::endl;

	glewExperimental = GL_TRUE;
	glewInit();

	//Set input mode
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPos(window, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);


	//Add camera==================================================================
	std::cout << "Initializing camera..." << std::endl;

	camera* camera_p1 = new camera(vec3(MAX_LEN / 2, MAX_LEN + 500, -1600), WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
	camera_p1->setKeySpeed(1000.f);
	camera_p1->setDirection(0.f, -0.6f);


	//Add 2D text=================================================================
	GLuint program_txt = loadShader("./shader/vs_text2d.txt", "./shader/fs_text2d.txt");
	text2d* txtFps = new text2d(program_txt, "Holstein.DDS", true);


	//Add voxel mesh==============================================================
	std::cout << "Initializing voxel mesh..." << std::endl;

	std::vector<glm::vec3> vertexData_vol(24);
	vertexData_vol[0] = vec3(0, 0, 0);
	vertexData_vol[1] = vec3(MAX_LEN_VOL, 0, 0);
	vertexData_vol[2] = vec3(MAX_LEN_VOL, 0, 0);
	vertexData_vol[3] = vec3(MAX_LEN_VOL, MAX_LEN_VOL, 0);
	vertexData_vol[4] = vec3(MAX_LEN_VOL, MAX_LEN_VOL, 0);
	vertexData_vol[5] = vec3(0, MAX_LEN_VOL, 0);
	vertexData_vol[6] = vec3(0, MAX_LEN_VOL, 0);
	vertexData_vol[7] = vec3(0, 0, 0);

	vertexData_vol[8] = vec3(0, 0, 0);
	vertexData_vol[9] = vec3(0, 0, MAX_LEN_VOL);
	vertexData_vol[10] = vec3(0, 0, MAX_LEN_VOL);
	vertexData_vol[11] = vec3(0, MAX_LEN_VOL, MAX_LEN_VOL);
	vertexData_vol[12] = vec3(0, MAX_LEN_VOL, MAX_LEN_VOL);
	vertexData_vol[13] = vec3(0, MAX_LEN_VOL, 0);

	vertexData_vol[14] = vec3(MAX_LEN_VOL, 0, 0);
	vertexData_vol[15] = vec3(MAX_LEN_VOL, 0, MAX_LEN_VOL);
	vertexData_vol[16] = vec3(MAX_LEN_VOL, 0, MAX_LEN_VOL);
	vertexData_vol[17] = vec3(MAX_LEN_VOL, MAX_LEN_VOL, MAX_LEN_VOL);
	vertexData_vol[18] = vec3(MAX_LEN_VOL, MAX_LEN_VOL, MAX_LEN_VOL);
	vertexData_vol[19] = vec3(MAX_LEN_VOL, MAX_LEN_VOL, 0);

	vertexData_vol[20] = vec3(0, MAX_LEN_VOL, MAX_LEN_VOL);
	vertexData_vol[21] = vec3(MAX_LEN_VOL, MAX_LEN_VOL, MAX_LEN_VOL);
	vertexData_vol[22] = vec3(0, 0, MAX_LEN_VOL);
	vertexData_vol[23] = vec3(MAX_LEN_VOL, 0, MAX_LEN_VOL);

	GLuint program_vol = loadShader("./shader/vs.txt", "./shader/fs.txt");;
	skeleton3d* grid = new skeleton3d(program_vol, vertexData_vol);
	setUniformVec3(program_vol, "color_input", glm::vec3(0, 0, 1.0f));

	std::vector<glm::vec3> vertexData_vol_max(24);
	vertexData_vol_max[0] = vec3(0, 0, 0);
	vertexData_vol_max[1] = vec3(MAX_LEN, 0, 0);
	vertexData_vol_max[2] = vec3(MAX_LEN, 0, 0);
	vertexData_vol_max[3] = vec3(MAX_LEN, MAX_LEN, 0);
	vertexData_vol_max[4] = vec3(MAX_LEN, MAX_LEN, 0);
	vertexData_vol_max[5] = vec3(0, MAX_LEN, 0);
	vertexData_vol_max[6] = vec3(0, MAX_LEN, 0);
	vertexData_vol_max[7] = vec3(0, 0, 0);

	vertexData_vol_max[8] = vec3(0, 0, 0);
	vertexData_vol_max[9] = vec3(0, 0, MAX_LEN);
	vertexData_vol_max[10] = vec3(0, 0, MAX_LEN);
	vertexData_vol_max[11] = vec3(0, MAX_LEN, MAX_LEN);
	vertexData_vol_max[12] = vec3(0, MAX_LEN, MAX_LEN);
	vertexData_vol_max[13] = vec3(0, MAX_LEN, 0);

	vertexData_vol_max[14] = vec3(MAX_LEN, 0, 0);
	vertexData_vol_max[15] = vec3(MAX_LEN, 0, MAX_LEN);
	vertexData_vol_max[16] = vec3(MAX_LEN, 0, MAX_LEN);
	vertexData_vol_max[17] = vec3(MAX_LEN, MAX_LEN, MAX_LEN);
	vertexData_vol_max[18] = vec3(MAX_LEN, MAX_LEN, MAX_LEN);
	vertexData_vol_max[19] = vec3(MAX_LEN, MAX_LEN, 0);

	vertexData_vol_max[20] = vec3(0, MAX_LEN, MAX_LEN);
	vertexData_vol_max[21] = vec3(MAX_LEN, MAX_LEN, MAX_LEN);
	vertexData_vol_max[22] = vec3(0, 0, MAX_LEN);
	vertexData_vol_max[23] = vec3(MAX_LEN, 0, MAX_LEN);

	GLuint program_vol_max = loadShader("./shader/vs.txt", "./shader/fs.txt");;
	skeleton3d* grid_max = new skeleton3d(program_vol_max, vertexData_vol_max);
	setUniformVec3(program_vol_max, "color_input", glm::vec3(0.8f, 0.5f, 0.5f));
	grid_max->setModel(glm::mat4());
	grid_max->setUniformMat4Model("M");


	//Add camera mesh=============================================================
	std::cout << "Initializing camera mesh..." << std::endl;

	std::vector<glm::vec3> vertexData_cam(24);
	vertexData_cam[0] = vec3(5.77f, 5.77f, 10);
	vertexData_cam[1] = vec3(5.77f, -5.77f, 10);
	vertexData_cam[2] = vec3(5.77f, -5.77f, 10);
	vertexData_cam[3] = vec3(-5.77f, -5.77f, 10);
	vertexData_cam[4] = vec3(-5.77f, -5.77f, 10);
	vertexData_cam[5] = vec3(-5.77f, 5.77f, 10);
	vertexData_cam[6] = vec3(-5.77f, 5.77f, 10);
	vertexData_cam[7] = vec3(5.77f, 5.77f, 10);

	vertexData_cam[8] = vec3(5.77f, 5.77f, 10);
	vertexData_cam[9] = vec3(230.8f, 230.8f, 400);
	vertexData_cam[10] = vec3(5.77f, -5.77f, 10);
	vertexData_cam[11] = vec3(230.8f, -230.8f, 400);
	vertexData_cam[12] = vec3(-5.77f, 5.77f, 10);
	vertexData_cam[13] = vec3(-230.8f, 230.8f, 400);
	vertexData_cam[14] = vec3(-5.77f, -5.77f, 10);
	vertexData_cam[15] = vec3(-230.8f, -230.8f, 400);

	vertexData_cam[16] = vec3(230.8f, 230.8f, 400);
	vertexData_cam[17] = vec3(230.8f, -230.8f, 400);
	vertexData_cam[18] = vec3(230.8f, -230.8f, 400);
	vertexData_cam[19] = vec3(-230.8f, -230.8f, 400);
	vertexData_cam[20] = vec3(-230.8f, -230.8f, 400);
	vertexData_cam[21] = vec3(-230.8f, 230.8f, 400);
	vertexData_cam[22] = vec3(-230.8f, 230.8f, 400);
	vertexData_cam[23] = vec3(230.8f, 230.8f, 400);

	GLuint program_cam = loadShader("./shader/vs.txt", "./shader/fs.txt");
	skeleton3d* cam = new skeleton3d(program_cam, vertexData_cam);
	glm::mat4 scal_cam = glm::scale(glm::mat4(1.0f), glm::vec3(2.f, 1.5f, 2.f));
	setUniformVec3(program_cam, "color_input", glm::vec3(1.0f, 1.0f, 0));
	cam->setModel(computeTransform(fusion_test.get_camera_R(), fusion_test.get_camera_T()) * scal_cam);
	cam->setUniformMat4Model("M");


	//Add trajectory==============================================================
	std::vector<glm::vec3> vertexData_traj;
	vertexData_traj.push_back(vectorToVec(fusion_test.get_camera_T()));
	vertexData_traj.push_back(vectorToVec(fusion_test.get_camera_T()));

	GLuint program_traj = loadShader("./shader/vs.txt", "./shader/fs.txt");
	skeleton3d* traj = new skeleton3d(program_traj, vertexData_traj);
	setUniformVec3(program_traj, "color_input", glm::vec3(1.0f, 0.2f, 0.2f));
	traj->setModel(glm::mat4());
	traj->setUniformMat4Model("M");


	//Add point cloud model=======================================================
	std::cout << "Initializing point cloud model..." << std::endl;

	std::vector<vec> pointCloudData = fusion_test.getPointCloud();
	std::vector<glm::vec3> pcData;
	convertToVec3(pointCloudData, pcData);

	GLuint program_fused = loadShader("./shader/vs.txt", "./shader/fs.txt");
	cloud* pointCloud_fused = new cloud(program_fused, pcData);

	pointCloud_fused->setModel(glm::mat4());
	pointCloud_fused->setUniformMat4Model("M");
	setUniformVec3(program_fused, "color_input", glm::vec3(0.15f, 1.0f, 0.15f));

	//Enable vsync
	glfwSwapInterval(1);

	//Set background color
	glClearColor(0, 0, 0, 0);

	//Enable depth test
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glCullFace(GL_BACK);


	//Initialize fusion===========================================================
	std::cout << "Initializing fusion..." << std::endl;

	float lastTime, curTime;
	char strFps[16];
	bool isStart = false, isCam = true, isSave = false;
	int frame = 1, lastFrame = 0;


	//Infinite loop===============================================================
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS
		&& glfwWindowShouldClose(window) == 0){

		if (isStart){
			if (frame > lastFrame && frame < total){
				//Update TSDF & camera pose
				get_normal_map(pc[frame], normal_tmp);
				fusion_test.update(pc[frame], normal[frame]);

				//Update trajectory
				vertexData_traj.push_back(vertexData_traj.back());
				vertexData_traj.push_back(vectorToVec(fusion_test.get_camera_T()));
				traj->setVertex(vertexData_traj);

				//Update point cloud set
				pointCloudData = fusion_test.getPointCloud();
				convertToVec3(pointCloudData, pcData);
				pointCloud_fused->uploadPoints(pcData);

				lastFrame++;
			}
			else{
				get_normal_map(pc[lastFrame], normal_tmp);
				fusion_test.update(pc[lastFrame], normal[lastFrame]);
			}

			//Compute fps
			curTime = glfwGetTime();
			sprintf(strFps, "%.2f fps", 1.f / (curTime - lastTime));
			txtFps->print(strFps, 10, 10, 20);
			lastTime = curTime;
		}

		//Input control
		if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS){
			if (!isStart){
				isStart = true;
				fusion_test.init_fusion(pc[0]);
				lastTime = glfwGetTime();
			}
		}

		if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){
			isStart = false;
			isCam = false;
		}

		if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS){
			isSave = true;
			break;
		}

		if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS){
			frame++;
		}

		//Set uniform matrix in the vertex shader
		camera_p1->computeFirstPersonVP(window);
		camera_p1->setUniformMat4VP(program_vol_max, "VP");
		camera_p1->setUniformMat4VP(program_vol, "VP");
		camera_p1->setUniformMat4VP(program_cam, "VP");
		camera_p1->setUniformMat4VP(program_fused, "VP");
		camera_p1->setUniformMat4VP(program_traj, "VP");

		cam->setModel(computeTransform(fusion_test.get_camera_R(), fusion_test.get_camera_T()) * scal_cam);
		cam->setUniformMat4Model("M");

		grid->setModel(computeTranslation(fusion_test.getVoxOffset()));
		grid->setUniformMat4Model("M");

		//Render
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		grid_max->render();
		grid->render();
		if(isCam) cam->render();
		traj->render();
		pointCloud_fused->render();
		txtFps->render();

		//Swap buffer
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	if (isSave){
		std::cout << "Saving data..." << std::endl << std::endl;
		fusion_test.savePointCloud("model.npts");
	}


	//Release data=====================================================
	std::cout << "Releasing data..." << std::endl << std::endl;

	glDeleteProgram(program_vol);
	glDeleteProgram(program_vol_max);
	glDeleteProgram(program_cam);
	glDeleteProgram(program_fused);
	glDeleteProgram(program_txt);
	glDeleteProgram(program_traj);
	glfwDestroyWindow(window);
	glfwTerminate();
	delete txtFps;
	delete grid, grid_max, cam;
	delete traj;
	delete camera_p1;
	delete pointCloud_fused;
	return 0;
}


/*=================================
Set uniform matrix in GLSL
=================================*/
static void setUniformVec3(const GLuint program, const char* name, const glm::vec3 &vec)
{
	glUseProgram(program);
	GLint loc = glGetUniformLocation(program, name);
	if (loc == -1) return;

	glUniform3fv(loc, 1, glm::value_ptr(vec));

	return;
}


/*=================================
Compute transformation matrix
=================================*/
static glm::mat4 computeTransform(Eigen::Matrix3f&& rot, Eigen::Vector3f&& trans)
{
	return glm::mat4(rot(0, 0), rot(0, 1), rot(0, 2), 0.f,
					 				 rot(1, 0), rot(1, 1), rot(1, 2), 0.f,
					 			 	 rot(2, 0), rot(2, 1), rot(2, 2), 0.f,
					 			 	 trans(0), trans(1), trans(2), 1.f);
}


/*=================================
Compute translation matrix
=================================*/
static glm::mat4 computeTranslation(vec&& trans)
{
	return glm::mat4(1.f, 0.f, 0.f, 0.f,
					 				 0.f, 1.f, 0.f, 0.f,
					 			 	 0.f, 0.f, 1.f, 0.f,
					 			 	 trans.x, trans.y, trans.z, 1.f);
}


/*=================================
Convert vec to vec3
=================================*/
static void convertToVec3(std::vector<vec> &v_in, std::vector<glm::vec3> &v_out)
{
	v_out.resize(v_in.size());

	for (int i = 0; i < v_in.size(); ++i){
		v_out[i].x = v_in[i].x;
		v_out[i].y = v_in[i].y;
		v_out[i].z = v_in[i].z;
	}
}


/*=================================
Convert vec to vec3
=================================*/
static glm::vec3 vectorToVec(Vector3f&& v_in)
{
	return glm::vec3(v_in(0), v_in(1), v_in(2));
}


/*=================================
vector to MatrixXf
=================================*/
static MatrixXf arr_to_mat(vector <Vector3f> &v_pc){
	int size = v_pc.size();
	MatrixXf pc_mat(3, size);

	for (int i = 0; i < size; i++)
		pc_mat.col(i) = v_pc[i];

	return pc_mat;
}


/*===================================
Get normal map form the depth map
====================================*/
static void get_normal_map(MatrixXf &pc, MatrixXf &nm)
{
	Vector3f point;
	int count = 0;
	Vector3f v1, v2, v3, v4;
	Vector3f vx, vy, result;

	for (int j = 0; j < IMG_HEIGHT; ++j){
		for (int i = 0; i < IMG_WIDTH; ++i){
			if (i == 0 || j == 0 || i == IMG_WIDTH - 1 || j == IMG_HEIGHT - 1){
				nm.col(count) = Vector3f(0, 0, 1);
				++count;
				continue;
			}

			v1 = pc.col(INDEX(i, j - 1));
			v2 = pc.col(INDEX(i, j + 1));
			v3 = pc.col(INDEX(i + 1, j));
			v4 = pc.col(INDEX(i - 1, j));

			vx = v3 - v4;
			vy = v1 - v2;
			result = vx.cross(vy);
			result /= result.norm();

			if (result == result)
				nm.col(count) = result;
			else
				nm.col(count) = Vector3f(0, 0, 1);

			++count;
		}
	}
}


/*====================================
Convert depth to world
====================================*/
static void DepthToWorld(int cx, int cy, unsigned short int cz, float &wx, float &wy, float &wz)
{
	if (cz == 0){
		wx = 0;
		wy = 0;
		wz = 0;
		return;
	}

	wx = (cx - IMG_WIDTH_OVER_TWO) * cz / FOCAL_LEN;
	wy = (IMG_HEIGHT_OVER_TWO - cy) * cz / FOCAL_LEN;
	wz = (float)cz;
}


/*====================================
Read point cloud data
====================================*/
static MatrixXf read_pc(const char *fn)
{
	fstream file;
	file.open(fn, ios::in);
	Vector3f mid;
	vector <Vector3f> pc_arr;

	float x, y, z;
	while (file){
		file >> x >> y >> z;
		mid << x, y, z;
		pc_arr.push_back(mid);
	}
	file.close();
	return arr_to_mat(pc_arr);
}
