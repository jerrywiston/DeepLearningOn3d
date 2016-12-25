#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <ctime>
#include "Eigen/Eigen"
#include "skeleton3d.h"
#include "cloud.h"
#include "camera.h"
#include "shader.hpp"
#include "text2d.h"
#include "ransac.hpp"
#include "orientation.hpp"

using namespace Eigen;

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768
#define MAX_LEN 2048

static void setUniformVec3(const GLuint program, const char* name, const glm::vec3 &vec);
static glm::mat4 computeTransform(Eigen::Matrix3f&& rot, Eigen::Vector3f&& trans);
static Eigen::MatrixXf arr_to_mat(vector <Eigen::Vector3f> &v_pc);
static Eigen::MatrixXf read_pc(const char *fn);


int main(int argc, char* argv[])
{

	//RANSAC===============================================================
	srand(time(0));
	int arg_plane = 4;
	int arg_random = 5000;
	float arg_dist = 10.0f;
	float scale = 2048.0f;

	MatrixXf PointCloud = scale * RANSAC::read_pc(argv[1]);
	MatrixXf PointCloudG[arg_plane + 1];

	int count = RANSAC::plane_group(arg_plane, arg_random, arg_dist, PointCloud, PointCloudG);
	cout << "Total : " << PointCloud.cols() << endl;

	//rotation
	if(atoi(argv[2]) != 1){
		MatrixXf rotf = OrientationFloor(arg_plane, PointCloudG);
		for(int i=0; i<arg_plane+1; ++i)
			PointCloudG[i] = rotf * PointCloudG[i];
		MatrixXf rotw = OrientationWall(arg_plane, PointCloudG);
		for(int i=0; i<arg_plane+1; ++i)
			PointCloudG[i] = rotw * PointCloudG[i];
	}

	//translation to origin
	Vector3f PointCloudAve = Vector3f::Zero();
	for(int i=0; i<arg_plane+1; ++i){
		PointCloudAve(0) += PointCloudG[i].row(0).sum();
		PointCloudAve(1) += PointCloudG[i].row(1).sum();
		PointCloudAve(2) += PointCloudG[i].row(2).sum();
	}
	PointCloudAve /= PointCloud.cols();
	for(int i=0; i<arg_plane+1; ++i)
		for(int j=0; j<PointCloudG[i].cols(); ++j)
			PointCloudG[i].col(j) -= PointCloudAve;

	//Init GLFW============================================================
	std::cout << "Initializing GLFW..." << std::endl;

	if (!glfwInit()) exit(EXIT_FAILURE);
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


	//Open GLFW window======================================================
	std::cout << "Openning GLFW window..." << std::endl;

	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Fusion", NULL, NULL);
	if (!window){
		glfwTerminate();
		return EXIT_FAILURE;
	}
	glfwMakeContextCurrent(window);


	//Init GLEW============================================================
	std::cout << "Initializing GLEW..." << std::endl;

	glewExperimental = GL_TRUE;
	glewInit();

	//Set input mode
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPos(window, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);


	//Add camera===========================================================
	std::cout << "Initializing camera..." << std::endl;

	//camera* camera_p1 = new camera(vec3(1000, 1000, -1600), WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
	//camera_p1->setDirection(0.f, -0.6f);
	camera* camera_p1 = new camera(vec3(0, 0, -1600), WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
	camera_p1->setKeySpeed(1000.f);
	camera_p1->setDirection(0.f, -0.0f);

	//Add 2D text==========================================================
	GLuint program_txt = loadShader("./shader/vs_text2d.txt", "./shader/fs_text2d.txt");
	text2d* txtFps = new text2d(program_txt, "Holstein.DDS", true);

	//Add space box==========================================================
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
	grid_max->setModel(glm::translate(glm::mat4(), glm::vec3(-MAX_LEN/2, -MAX_LEN/2, -MAX_LEN/2)));
	grid_max->setUniformMat4Model("M");

	//Add point cloud model=======================================================
	std::cout << "Initializing point cloud model..." << std::endl;

	std::vector<glm::vec3> pcData[arg_plane+1];
	for(int j = 0; j < arg_plane + 1; ++j)
		for(int i = 0; i < PointCloudG[j].cols(); ++i)
			pcData[j].push_back(glm::vec3(PointCloudG[j](0,i), PointCloudG[j](1,i), PointCloudG[j](2,i)));

	GLuint program_fused[arg_plane+1];
	cloud *pointCloud_fused[arg_plane+1];
	glm::vec3 class_color;
	for(int i=0; i<arg_plane+1; ++i){
		program_fused[i] = loadShader("./shader/vs.txt", "./shader/fs.txt");
		pointCloud_fused[i] = new cloud(program_fused[i], pcData[i]);
		pointCloud_fused[i]->setModel(glm::mat4());
		pointCloud_fused[i]->setUniformMat4Model("M");
		if(atoi(argv[2]) == 1)
			class_color = glm::vec3(1.0f, 1.0f, 1.0f);
		else if(i == 0)
			class_color = glm::vec3(1.0f, 0.0f, 0.0f);
		else if(i == 1)
			class_color = glm::vec3(0.0f, 1.0f, 0.0f);
		else if(i == 2)
			class_color = glm::vec3(0.0f, 0.0f, 1.0f);
		else if(i == arg_plane)
			class_color = glm::vec3(1.0f, 1.0f, 1.0f);
		else
			class_color = glm::vec3((float)(rand()%500)/1000+0.5, (float)(rand()%500)/1000+0.5, (float)(rand()%500)/1000+0.5);

		setUniformVec3(program_fused[i], "color_input",class_color);
	}

	//Enable vsync
	glfwSwapInterval(1);

	//Set background color
	glClearColor(0, 0, 0, 0);

	//Enable depth test
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glCullFace(GL_BACK);

	float lastTime = glfwGetTime();
	char strFps[64];

	//Infinite loop===========================================================
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS
		&& glfwWindowShouldClose(window) == 0){

		//Compute fps
		sprintf(strFps, "%.2f fps", 1.f / (glfwGetTime() - lastTime));
		txtFps->print(strFps, 10, 10, 20);
		lastTime = glfwGetTime();

		//Input control
		if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS){

		}

		if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){

		}

		if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS){

		}

		if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS){

		}

		//Set uniform matrix in the vertex shader
		camera_p1->computeFirstPersonVP(window);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		for(int i=0; i<arg_plane+1; ++i){
			camera_p1->setUniformMat4VP(program_fused[i], "VP");
			pointCloud_fused[i]->render();
		}
		camera_p1->setUniformMat4VP(program_vol_max, "VP");
		grid_max->render();
		txtFps->render();

		//Swap buffer
		glfwSwapBuffers(window);
		glfwPollEvents();
	}


	//Release data=====================================================
	std::cout << "Releasing data..." << std::endl << std::endl;

	for(int i=0; i<arg_plane+1; ++i)
		glDeleteProgram(program_fused[i]);
	glDeleteProgram(program_txt);
	glfwDestroyWindow(window);
	glDeleteProgram(program_vol_max);
	glfwTerminate();
	delete txtFps;
	delete camera_p1;
	delete grid_max;

	for(int i=0; i<arg_plane+1; ++i)
		delete pointCloud_fused[i];
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
vector to MatrixXf
=================================*/
static Eigen::MatrixXf arr_to_mat(vector <Eigen::Vector3f> &v_pc)
{
	int size = v_pc.size();
	Eigen::MatrixXf pc_mat(3, size);

	for (int i = 0; i < size; i++)
		pc_mat.col(i) = v_pc[i];

	return pc_mat;
}


static Eigen::MatrixXf read_pc(const char *fn)
{
	fstream file;
	file.open(fn, ios::in);
	Eigen::Vector3f mid;
	vector <Eigen::Vector3f> pc_arr;

	float x, y, z;
	while (file){
		file >> x >> y >> z;
		mid << x, y, z;
		pc_arr.push_back(mid);
	}
	file.close();
	return arr_to_mat(pc_arr);
}
