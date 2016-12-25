#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <vector>
#include "Eigen/Eigen"
#include "skeleton3d.h"
#include "cloud.h"
#include "camera.h"
#include "shader.hpp"
#include "text2d.h"

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

static void setUniformVec3(const GLuint program, const char* name, const glm::vec3 &vec);
static glm::mat4 computeTransform(Eigen::Matrix3f&& rot, Eigen::Vector3f&& trans);
static Eigen::MatrixXf arr_to_mat(vector <Eigen::Vector3f> &v_pc);
static Eigen::MatrixXf read_pc(const char *fn);


int main(int argc, char* argv[])
{
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

	camera* camera_p1 = new camera(vec3(1000, 1000, -1600), WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
	camera_p1->setKeySpeed(1000.f);
	camera_p1->setDirection(0.f, -0.6f);


	//Add 2D text==========================================================
	GLuint program_txt = loadShader("./shader/vs_text2d.txt", "./shader/fs_text2d.txt");
	text2d* txtFps = new text2d(program_txt, "Holstein.DDS", true);


	//Add point cloud model=======================================================
	std::cout << "Initializing point cloud model..." << std::endl;

	std::vector<glm::vec3> pcData;

	for(int i = 0; i < 500; i += 5)
		for(int j = 0; j < 500; j += 5)
			pcData.push_back(glm::vec3(i, j, 0));

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
		camera_p1->setUniformMat4VP(program_fused, "VP");

		//Render
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		pointCloud_fused->render();
		txtFps->render();

		//Swap buffer
		glfwSwapBuffers(window);
		glfwPollEvents();
	}


	//Release data=====================================================
	std::cout << "Releasing data..." << std::endl << std::endl;


	glDeleteProgram(program_fused);
	glDeleteProgram(program_txt);
	glfwDestroyWindow(window);
	glfwTerminate();
	delete txtFps;
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
