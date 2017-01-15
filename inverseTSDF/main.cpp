#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include "Eigen/Eigen"
#include "skeleton3d.h"
#include "cloud.h"
#include "camera.h"
#include "shader.hpp"
#include "ICP_fusion.h"
#include "text2d.h"
#include "config.hpp"
#include "img2d.h"
#include "tensorUtil.hpp"


#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

static void setUniformVec3(const GLuint program, const char* name, const glm::vec3 &vec);
static glm::mat4 computeTranslation(vec&& trans);
static void convertToVec3(std::vector<vec> &v_in, std::vector<glm::vec3> &v_out);
static glm::vec3 vectorToVec(Vector3f&& v_in);
static MatrixXf arr_to_mat(vector <Vector3f> &v_pc);
static void get_normal_map(MatrixXf &pc, MatrixXf &nm);
static void DepthToWorld(int cx, int cy, unsigned short int cz, float &wx, float &wy, float &wz);
static glm::mat4 TransEigen2Mat4(Eigen::Vector3f trans);
static glm::mat4 RotEigen2Mat4(Eigen::Matrix3f rot);
static bool readPointCloud(const std::string fileName, std::vector<vec> &pc);
static bool saveTSDF(const std::string fileName, voxel *vox);


int main(int argc, char* argv[])
{
	//Read point cloud==========================================================
	std::vector<vec> pc;
	readPointCloud("./model/out.npts", pc);


	//Init GLFW=================================================================
	std::cout << "Initializing GLFW..." << std::endl;

	if (!glfwInit()) exit(EXIT_FAILURE);
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


	//Open GLFW window==========================================================
	std::cout << "Openning GLFW window..." << std::endl;

	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Fusion", NULL, NULL);
	if (!window){
		glfwTerminate();
		return EXIT_FAILURE;
	}
	glfwMakeContextCurrent(window);


	//Init ICP fusion===========================================================
	std::cout << "Initializing ICP fusion..." << std::endl;

	ICP_fusion fusion_test(ALIGN_SAMPLE, MODEL_SAMPLE, WINDOW_SIZE, ITER_LOOPS);


	//Init GLEW=================================================================
	std::cout << "Initializing GLEW..." << std::endl;

	glewExperimental = GL_TRUE;
	glewInit();

	//Set input mode
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPos(window, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);


	//Add camera================================================================
	std::cout << "Initializing camera..." << std::endl;

	camera* camera_p1 = new camera(vec3(MAX_LEN_X / 2, MAX_LEN_Y, -500), WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
	camera_p1->setKeySpeed(1000.f);
	camera_p1->setDirection(0.f, -0.6f);


	//Add 2D text===============================================================
	text2d* txtFps = new text2d(loadShader("./shader/vs_text2d.txt", "./shader/fs_text2d.txt"), "Holstein.DDS", true);
	txtFps->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	txtFps->setUniformVec3Color("color_input");


	//Add voxel mesh============================================================
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

	skeleton3d* grid = new skeleton3d(loadShader("./shader/vs.txt", "./shader/fs.txt"), vertexData_vol);
	grid->setColor(glm::vec3(0, 0, 1.0f));
	grid->setUniformVec3Color("color_input");

	std::vector<glm::vec3> vertexData_vol_max(24);
	vertexData_vol_max[0] = vec3(0, 0, 0);
	vertexData_vol_max[1] = vec3(MAX_LEN_X, 0, 0);
	vertexData_vol_max[2] = vec3(MAX_LEN_X, 0, 0);
	vertexData_vol_max[3] = vec3(MAX_LEN_X, MAX_LEN_Y, 0);
	vertexData_vol_max[4] = vec3(MAX_LEN_X, MAX_LEN_Y, 0);
	vertexData_vol_max[5] = vec3(0, MAX_LEN_Y, 0);
	vertexData_vol_max[6] = vec3(0, MAX_LEN_Y, 0);
	vertexData_vol_max[7] = vec3(0, 0, 0);

	vertexData_vol_max[8] = vec3(0, 0, 0);
	vertexData_vol_max[9] = vec3(0, 0, MAX_LEN_Z);
	vertexData_vol_max[10] = vec3(0, 0, MAX_LEN_Z);
	vertexData_vol_max[11] = vec3(0, MAX_LEN_Y, MAX_LEN_Z);
	vertexData_vol_max[12] = vec3(0, MAX_LEN_Y, MAX_LEN_Z);
	vertexData_vol_max[13] = vec3(0, MAX_LEN_Y, 0);

	vertexData_vol_max[14] = vec3(MAX_LEN_X, 0, 0);
	vertexData_vol_max[15] = vec3(MAX_LEN_X, 0, MAX_LEN_Z);
	vertexData_vol_max[16] = vec3(MAX_LEN_X, 0, MAX_LEN_Z);
	vertexData_vol_max[17] = vec3(MAX_LEN_X, MAX_LEN_Y, MAX_LEN_Z);
	vertexData_vol_max[18] = vec3(MAX_LEN_X, MAX_LEN_Y, MAX_LEN_Z);
	vertexData_vol_max[19] = vec3(MAX_LEN_X, MAX_LEN_Y, 0);

	vertexData_vol_max[20] = vec3(0, MAX_LEN_Y, MAX_LEN_Z);
	vertexData_vol_max[21] = vec3(MAX_LEN_X, MAX_LEN_Y, MAX_LEN_Z);
	vertexData_vol_max[22] = vec3(0, 0, MAX_LEN_Z);
	vertexData_vol_max[23] = vec3(MAX_LEN_X, 0, MAX_LEN_Z);

	skeleton3d* grid_max = new skeleton3d(loadShader("./shader/vs.txt", "./shader/fs.txt"), vertexData_vol_max);
	grid_max->setColor(glm::vec3(0.8f, 0.5f, 0.5f));
	grid_max->setUniformVec3Color("color_input");
	grid_max->setUniformMat4Model("M");


	//Add camera mesh===========================================================
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

	skeleton3d* cam = new skeleton3d(loadShader("./shader/vs.txt", "./shader/fs.txt"), vertexData_cam);
	glm::mat4 scal_cam = glm::scale(glm::mat4(1.0f), glm::vec3(2.f, 1.5f, 2.f));
	cam->setTrans(TransEigen2Mat4(fusion_test.get_camera_T()));
	cam->setRot(RotEigen2Mat4(fusion_test.get_camera_R()));
	cam->setScal(scal_cam);
	cam->setUniformMat4Model("M");
	cam->setColor(glm::vec3(1.0f, 1.0f, 0));
	cam->setUniformVec3Color("color_input");


	//Add point cloud model=====================================================
	std::cout << "Initializing point cloud model..." << std::endl;

	std::vector<vec> pointCloudData;
	std::vector<glm::vec3> pcData;
	convertToVec3(pointCloudData, pcData);
	cloud* pointCloud_fused = new cloud(loadShader("./shader/vs.txt", "./shader/fs.txt"), pcData);
	pointCloud_fused->setColor(glm::vec3(0.15f, 1.0f, 0.15f));
	pointCloud_fused->setUniformVec3Color("color_input");
	pointCloud_fused->setUniformMat4Model("M");

	std::vector<glm::vec3> pcData_origin;
	convertToVec3(pc, pcData_origin);
	cloud* pointCloud_origin = new cloud(loadShader("./shader/vs.txt", "./shader/fs.txt"), pcData_origin);
	pointCloud_origin->setUniformMat4Model("M");
	pointCloud_origin->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	pointCloud_origin->setUniformVec3Color("color_input");


	//Add 2d image==============================================================
	unsigned char depthImgData[IMG_WIDTH*IMG_HEIGHT*4];
	for(int i = 0; i < IMG_WIDTH*IMG_HEIGHT*4; ++i) depthImgData[i] = 255;

	img2d* depthImg = new img2d(loadShader("./shader/vs_img2d.txt", "./shader/fs_img2d.txt"), depthImgData, IMG_WIDTH, IMG_HEIGHT);
	depthImg->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	depthImg->setUniformVec3Color("color_input");
	depthImg->show(0, 420, IMG_WIDTH/2, IMG_HEIGHT/2);


	//Enable vsync
	glfwSwapInterval(1);

	//Set background color
	glClearColor(0, 0, 0, 0);

	//Enable depth test
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glCullFace(GL_BACK);


	//Initialize fusion=========================================================
	std::cout << "Initializing fusion..." << std::endl;

	int frameNum = 0;
	float lastTime, curTime;
	char strFps[16];
	bool isStart = false, isSave = false;

	fusion_test.inverseTSDF(pc);


	//Infinite loop=============================================================
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS
		&& glfwWindowShouldClose(window) == 0){

		if (isStart){
			//Update point cloud set
			pointCloudData = fusion_test.getPointCloud();
			convertToVec3(pointCloudData, pcData);
			pointCloud_fused->uploadPoints(pcData);

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
				lastTime = glfwGetTime();
			}
		}

		if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){
			isStart = false;

			//Update point cloud set
			pointCloudData = fusion_test.getPointCloud();
			convertToVec3(pointCloudData, pcData);
			pointCloud_fused->uploadPoints(pcData);
		}

		if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS){
			isSave = true;
			break;
		}

		//Set uniform matrix in the vertex shader
		camera_p1->computeFirstPersonVP(window);
		camera_p1->setUniformMat4VP(grid_max->getProgram(), "VP");
		camera_p1->setUniformMat4VP(cam->getProgram(), "VP");
		camera_p1->setUniformMat4VP(pointCloud_fused->getProgram(), "VP");
		camera_p1->setUniformMat4VP(pointCloud_origin->getProgram(), "VP");
		camera_p1->setUniformMat4VP(grid->getProgram(), "VP");

		//Render
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		grid_max->render();
		//grid->render();
		//cam->render();
		//pointCloud_origin->render();
		pointCloud_fused->render();
		txtFps->render();
		//depthImg->render();

		//Swap buffer
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	if (isSave){
		std::cout << "Saving data..." << std::endl << std::endl;
		saveTSDF("test.tensor", fusion_test.getVox());
	}


	//Release data==============================================================
	std::cout << "Releasing data..." << std::endl << std::endl;

	glfwDestroyWindow(window);
	glfwTerminate();
	delete txtFps;
	delete grid_max;
	delete grid;
	delete cam;
	delete camera_p1;
	delete pointCloud_fused;
	delete pointCloud_origin;
	delete depthImg;
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

	for (int j = 0; j<IMG_HEIGHT; ++j){
		for (int i = 0; i<IMG_WIDTH; ++i){
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

	wx = (cx - IMG_WIDTH/2) * cz / FOCAL_LEN;
	wy = (IMG_HEIGHT/2 - cy) * cz / FOCAL_LEN;
	wz = (float)cz;
}


static glm::mat4 TransEigen2Mat4(Eigen::Vector3f trans)
{
	return glm::mat4(1.f,0.f,0.f,0.f,
					 0.f,1.f,0.f,0.f,
					 0.f,0.f,1.f,0.f,
					 trans(0), trans(1), trans(2), 1.f);
}


static glm::mat4 RotEigen2Mat4(Eigen::Matrix3f rot)
{
	return glm::mat4(rot(0, 0), rot(0, 1), rot(0, 2), 0.f,
					 rot(1, 0), rot(1, 1), rot(1, 2), 0.f,
					 rot(2, 0), rot(2, 1), rot(2, 2), 0.f,
					 0.f,0.f,0.f,1.0f);
}


static bool readPointCloud(const std::string fileName, std::vector<vec> &pc)
{
	vec point;
	FILE *fp;

	if(!(fp = fopen(fileName.c_str(), "r"))) return false;

	while(!feof(fp)){
		try{
			if(fscanf(fp, "%f\t%f\t%f\n", &(point.x), &(point.y), &(point.z)) > 0)
				pc.push_back(point);
		}
		catch(...){

		}
	}

	fclose(fp);
	return true;
}


static bool saveTSDF(const std::string fileName, voxel *vox)
{
	tensor_t tensor;

	tensor.type = TYPE_FLOAT;
	tensor.size = 4;
	tensor.name = "TSDF";
	tensor.numDim = 5;
	tensor.dims = new int32_t[5];
	float* data = new float[RESOL_X*RESOL_Y*RESOL_Z];

	tensor.dims[0] = 1;
	tensor.dims[1] = 1;
	tensor.dims[2] = RESOL_X;
	tensor.dims[3] = RESOL_Y;
	tensor.dims[4] = RESOL_Z;

	for(int i = 0; i < RESOL_X*RESOL_Y*RESOL_Z; ++i){
		float tsdf = vox[i].tsdf;

	 	if(vox[i].isActive) data[i] = tsdf;
		else data[i] = 0;
	}

	tensor.value = (void*)data;

	if(writeTensor(fileName, &tensor)){
		releaseTensor(&tensor);
		return true;
	}

	releaseTensor(&tensor);
	return false;
}
