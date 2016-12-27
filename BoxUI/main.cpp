#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/norm.hpp"

#include "Eigen/Eigen"

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <vector>

#include "shader.hpp"
#include "screen.hpp"
#include "skeleton3d.h"
#include "cloud.h"
#include "camera.h"
#include "text2d.h"
#include "img2d.h"
#include "cursor.h"
#include "visibleObj.h"

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 768
#define MAX_LEN 4096

bool isScroll = false;
int scrollDir = 0;

static void setUniformVec3(const GLuint program, const char* name, const glm::vec3 &vec);
static Eigen::MatrixXf arr_to_mat(vector <Eigen::Vector3f> &v_pc);
static Eigen::MatrixXf read_pc(const char *fn);
static void loadProp(visibleObj* obj, text2d* txtProp[], const int size);
static void mouseSelect(GLFWwindow* window, camera *cam, vector<skeleton3d*> &box, text2d* txt[], int *boxIndex, int *txtIndex);
static void setBoxProperties(vector<skeleton3d*> &box, text2d* txtProp[], const int boxIndex, const int txtIndex);
static void addBox(vector<skeleton3d*> &box, vector<glm::vec3> &vertexData);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);


int main(int argc, char* argv[])
{
	if(argc < 2){
		std::cout << "ERROR: Too few argument" << std::endl;
		exit(1);
	}


	//Read point cloud==========================================================
	float scaleX = 1, scaleY = 1, scaleZ = 1;
	Eigen::MatrixXf pointCloud = read_pc(argv[1]);

	for(int i = 0; i < pointCloud.cols(); ++i){
		pointCloud(0, i) *= scaleX;
		pointCloud(1, i) *= scaleY;
		pointCloud(2, i) *= scaleZ;
	}


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

	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "BOX UI", NULL, NULL);
	if (!window){
		glfwTerminate();
		return EXIT_FAILURE;
	}
	glfwMakeContextCurrent(window);


	//Init GLEW=================================================================
	std::cout << "Initializing GLEW..." << std::endl;

	glewExperimental = GL_TRUE;
	glewInit();

	//Set input mode
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

	//Enable vsync
	glfwSwapInterval(1);

	//Set background color
	glClearColor(0, 0, 0, 0);

	//Enable depth test
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glCullFace(GL_BACK);


	//Add camera================================================================
	std::cout << "Initializing camera..." << std::endl;

	camera* camera_p1 = new camera(vec3(0, 0, -1000), WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
	camera_p1->setKeySpeed(2000.f);
	camera_p1->setDirection(0.f, -0.6f);


	//Add 2D text===============================================================
	text2d *txtFps = new text2d(loadShader("./shader/vs_text2d.txt", "./shader/fs_text2d.txt"), "Holstein.DDS", true);
	txtFps->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	txtFps->setUniformVec3Color("color_input");

	text2d *txtScalTitle = new text2d(loadShader("./shader/vs_text2d.txt", "./shader/fs_text2d.txt"), "Holstein.DDS", true);
	txtScalTitle->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	txtScalTitle->setUniformVec3Color("color_input");

	text2d *txtTransTitle = new text2d(loadShader("./shader/vs_text2d.txt", "./shader/fs_text2d.txt"), "Holstein.DDS", true);
	txtTransTitle->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	txtTransTitle->setUniformVec3Color("color_input");

	text2d *txtProp[6];

	for(int i = 0; i < 6; ++i){
		txtProp[i] = new text2d(loadShader("./shader/vs_text2d.txt", "./shader/fs_text2d.txt"), "Holstein.DDS", true);
		txtProp[i]->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
		txtProp[i]->setUniformVec3Color("color_input");
	}

	txtScalTitle->print("Scale:", 0, 500, 15);
	txtTransTitle->print("Position:", 650, 500, 15);


	//Add voxel mesh==============================================================
	std::cout << "Initializing voxel mesh..." << std::endl;

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

	skeleton3d* grid_max = new skeleton3d(loadShader("./shader/vs.txt", "./shader/fs.txt"), vertexData_vol_max);
	grid_max->setColor(glm::vec3(0.2f, 0.2f, 1.0f));
	grid_max->setUniformVec3Color("color_input");
	grid_max->setTrans(glm::translate(glm::mat4(), glm::vec3(-MAX_LEN/2, -MAX_LEN/2, -MAX_LEN/2)));
	grid_max->setUniformMat4Model("M");


	//Add point cloud model=======================================================
	std::cout << "Initializing point cloud model..." << std::endl;

	std::vector<glm::vec3> pcData;

	for(int i = 0; i < pointCloud.cols(); ++i)
		pcData.push_back(glm::vec3(pointCloud(0, i), pointCloud(1, i), pointCloud(2, i)));

	cloud* pointCloud_fused = new cloud(loadShader("./shader/vs.txt", "./shader/fs.txt"), pcData);
	pointCloud_fused->setColor(glm::vec3(0.f, 0.8f, 0.f));
	pointCloud_fused->setUniformVec3Color("color_input");
	pointCloud_fused->setUniformMat4Model("M");


	//Add bounding boxes model====================================================
	std::cout << "Initializing camera mesh..." << std::endl;

	std::vector<glm::vec3> vertexData_box(24);
	vertexData_box[0] = vec3(-1, -1, -1);
	vertexData_box[1] = vec3(1, -1, -1);
	vertexData_box[2] = vec3(1, -1, -1);
	vertexData_box[3] = vec3(1, 1, -1);
	vertexData_box[4] = vec3(1, 1, -1);
	vertexData_box[5] = vec3(-1, 1, -1);
	vertexData_box[6] = vec3(-1, 1, -1);
	vertexData_box[7] = vec3(-1, -1, -1);

	vertexData_box[8] = vec3(-1, -1, -1);
	vertexData_box[9] = vec3(-1, -1, 1);
	vertexData_box[10] = vec3(-1, -1, 1);
	vertexData_box[11] = vec3(-1, 1, 1);
	vertexData_box[12] = vec3(-1, 1, 1);
	vertexData_box[13] = vec3(-1, 1, -1);

	vertexData_box[14] = vec3(1, -1, -1);
	vertexData_box[15] = vec3(1, -1, 1);
	vertexData_box[16] = vec3(1, -1, 1);
	vertexData_box[17] = vec3(1, 1, 1);
	vertexData_box[18] = vec3(1, 1, 1);
	vertexData_box[19] = vec3(1, 1, -1);

	vertexData_box[20] = vec3(-1, 1, 1);
	vertexData_box[21] = vec3(1, 1, 1);
	vertexData_box[22] = vec3(-1, -1, 1);
	vertexData_box[23] = vec3(1, -1, 1);

	glm::vec3 aabb_min(-1, -1, -1);
	glm::vec3 aabb_max(1, 1, 1);
	vector<skeleton3d*> box;


	//Add cursor==================================================================
	cursor* cursor_arrow = new cursor(loadShader("./shader/vs.txt", "./shader/fs.txt"), "./cur.obj", nullptr, false);
	cursor_arrow->init(window, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, "VP");
	cursor_arrow->setUniformMat4Model("M");
	cursor_arrow->setColor(glm::vec3(1.0f, 0.8f, 0.3f));
	cursor_arrow->setUniformVec3Color("color_input");


	//Initialize fusion===========================================================
	std::cout << "Initializing fusion..." << std::endl;

	float lastTime, curTime, btnTime;
	char buf[16];
	bool isSave = false, isLeftPress = false, isRightPress = false;
	int boxIndex = -1, txtIndex = -1;

	btnTime = glfwGetTime();

	glfwSetScrollCallback(window, scroll_callback);


	//Infinite loop===============================================================
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS
		&& glfwWindowShouldClose(window) == 0){
		//Compute fps
		curTime = glfwGetTime();
		sprintf(buf, "%.2f fps", 1.f / (curTime - lastTime));
		txtFps->print(buf, 10, 10, 20);
		lastTime = curTime;

		//Input control
		if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS){
			isSave = true;
			break;
		}

		if(glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS){
			if(glfwGetTime() - btnTime > 1){
				addBox(box, vertexData_box);
				btnTime = glfwGetTime();
			}
		}

		if(glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS){
			camera_p1->setDirection(0.f, 0.0f);
			camera_p1->setPosition(glm::vec3(0, 0, -2400.0f));

			for(int i = 0; i < 6; ++i){
				txtProp[i]->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	   			txtProp[i]->setUniformVec3Color("color_input");
			}

			if(boxIndex > -1){
				txtProp[3]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
	   			txtProp[3]->setUniformVec3Color("color_input");
				txtIndex = 3;
			}
		}

		if(glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS){
			camera_p1->setDirection(PI/2, 0.0f);
			camera_p1->setPosition(glm::vec3(-2400.0f, 0, 0));

			for(int i = 0; i < 6; ++i){
				txtProp[i]->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	   			txtProp[i]->setUniformVec3Color("color_input");
			}

			if(boxIndex > -1){
				txtProp[4]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
	   			txtProp[4]->setUniformVec3Color("color_input");
				txtIndex = 4;
			}
		}

		if(glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS){
			camera_p1->setDirection(PI, 0.0f);
			camera_p1->setPosition(glm::vec3(0, 0, MAX_LEN + 2400.0f));

			for(int i = 0; i < 6; ++i){
				txtProp[i]->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	   			txtProp[i]->setUniformVec3Color("color_input");
			}

			if(boxIndex > -1){
				txtProp[3]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
	   			txtProp[3]->setUniformVec3Color("color_input");
				txtIndex = 3;
			}
		}

		if(glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS){
			camera_p1->setDirection(-PI/2, 0.0f);
			camera_p1->setPosition(glm::vec3(MAX_LEN + 2400.0f, 0, 0));

			for(int i = 0; i < 6; ++i){
				txtProp[i]->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	   			txtProp[i]->setUniformVec3Color("color_input");
			}

			if(boxIndex > -1){
				txtProp[4]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
	   			txtProp[4]->setUniformVec3Color("color_input");
				txtIndex = 4;
			}
		}

		if(glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS){
			camera_p1->setDirection(0.0f, -PI/2);
			camera_p1->setPosition(glm::vec3(0, MAX_LEN + 2400.0f, 0));

			for(int i = 0; i < 6; ++i){
				txtProp[i]->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	   			txtProp[i]->setUniformVec3Color("color_input");
			}

			if(boxIndex > -1){
				txtProp[5]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
	   			txtProp[5]->setUniformVec3Color("color_input");
				txtIndex = 5;
			}
		}

		if(glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS){
			camera_p1->setDirection(0.0f, PI/2);
			camera_p1->setPosition(glm::vec3(0, -2400.0f, 0));

			for(int i = 0; i < 6; ++i){
				txtProp[i]->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	   			txtProp[i]->setUniformVec3Color("color_input");
			}

			if(boxIndex > -1){
				txtProp[5]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
	   			txtProp[5]->setUniformVec3Color("color_input");
				txtIndex = 5;
			}
		}

		if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS){
			if(!isRightPress){
				glfwSetCursorPos(window, WINDOW_WIDTH/2, WINDOW_HEIGHT/2);
				isRightPress = true;
			}
		}
		else if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE){
			isRightPress = false;
		}

		if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS){
			if(!isLeftPress){
				mouseSelect(window, camera_p1, box, txtProp, &boxIndex, &txtIndex);
				isLeftPress = true;
			}
		}
		else if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE){
			isLeftPress = false;
		}

		if(isScroll){
			setBoxProperties(box, txtProp, boxIndex, txtIndex);
			isScroll = false;
		}

		//Set uniform matrix in the vertex shader
		camera_p1->computeFirstPersonVP(window, isRightPress);
		camera_p1->setUniformMat4VP(grid_max->getProgram(), "VP");
		camera_p1->setUniformMat4VP(pointCloud_fused->getProgram(), "VP");
		for(int i = 0; i < box.size(); ++i) camera_p1->setUniformMat4VP(box[i]->getProgram(), "VP");

		cursor_arrow->computeCursorPos(window);
		cursor_arrow->computeCursorRot();
		cursor_arrow->setUniformMat4Cursor("M");

		//Render
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		grid_max->render();
		pointCloud_fused->render();
		txtFps->render();
		txtScalTitle->render();
		txtTransTitle->render();
		for(int i = 0; i < 6; ++i) txtProp[i]->render();
		for(int i = 0; i < box.size(); ++i) box[i]->render();
		if(!isRightPress) cursor_arrow->render();

		//Swap buffer
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	if (isSave){
		std::cout << "Saving data..." << std::endl << std::endl;
		//Save model
	}


	//Release data=====================================================
	std::cout << "Releasing data..." << std::endl << std::endl;

	glfwDestroyWindow(window);
	glfwTerminate();

	delete txtFps, txtScalTitle, txtTransTitle;
	for(int i = 0; i < 3; ++i) delete txtProp[i];
	delete grid_max;
	delete camera_p1;
	delete pointCloud_fused;
	for(int i = 0; i < box.size(); ++i) delete box[i];
	delete cursor_arrow;
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
vector to MatrixXf
=================================*/
static Eigen::MatrixXf arr_to_mat(vector <Eigen::Vector3f> &v_pc){
	int size = v_pc.size();
	Eigen::MatrixXf pc_mat(3, size);

	for (int i = 0; i < size; i++)
		pc_mat.col(i) = v_pc[i];

	return pc_mat;
}


/*====================================
Read point cloud data
====================================*/
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


/*====================================
Load box properties
====================================*/
static void loadProp(visibleObj* obj, text2d* txt[], const int size)
{
	if(obj == nullptr) return;

	char buf[64];
	vec3 scal = obj->getScalXYZ();
	vec3 pos = obj->getTransXYZ();

	sprintf(buf, "% 2.2f", scal.x);
	txt[0]->print(buf, 0, 400, size);

	sprintf(buf, "% 2.2f", scal.y);
	txt[1]->print(buf, 0, 300, size);

	sprintf(buf, "% 2.2f", scal.z);
	txt[2]->print(buf, 0, 200, size);

	sprintf(buf, "% 2.2f", pos.x);
	txt[3]->print(buf, 650, 400, size);

	sprintf(buf, "% 2.2f", pos.y);
	txt[4]->print(buf, 650, 300, size);

	sprintf(buf, "% 2.2f", pos.z);
	txt[5]->print(buf, 650, 200, size);
}


/*====================================
Mouse selection
====================================*/
static void mouseSelect(GLFWwindow* window, camera *cam, vector<skeleton3d*> &box, text2d* txtProp[], int *boxIndex, int *txtIndex)
{
	glm::vec3 rayOri, rayDir;
	double Xpos, Ypos;

	glfwGetCursorPos(window, &Xpos, &Ypos);

	//Select properties=========================================================
	if(*boxIndex > -1){
		*txtIndex = -1;

		for(int i = 0; i < 6; ++i){
			txtProp[i]->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
   			txtProp[i]->setUniformVec3Color("color_input");
		}

		if(   Xpos > 0 && Xpos < 200
		   && Ypos > 200 && Ypos < 300){
			txtProp[0]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
   			txtProp[0]->setUniformVec3Color("color_input");
			*txtIndex = 0;
			return;
		}
		else if(   Xpos > 0 && Xpos < 200
		   		&& Ypos > 330 && Ypos < 430){
			txtProp[1]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
		   	txtProp[1]->setUniformVec3Color("color_input");
			*txtIndex = 1;
			return;
		}
		else if(   Xpos > 0 && Xpos < 200
		   		&& Ypos > 460 && Ypos < 560){
			txtProp[2]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
		   	txtProp[2]->setUniformVec3Color("color_input");
			*txtIndex = 2;
			return;
		}
		else if(   Xpos > 1030 && Xpos < 1230
		   		&& Ypos > 200 && Ypos < 300){
			txtProp[3]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
			txtProp[3]->setUniformVec3Color("color_input");
			*txtIndex = 3;
			return;
		}
		else if(   Xpos > 1030 && Xpos < 1230
		   		&& Ypos > 330 && Ypos < 430){
			txtProp[4]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
			txtProp[4]->setUniformVec3Color("color_input");
			*txtIndex = 4;
			return;
		}
		else if(   Xpos > 1030 && Xpos < 1230
		   		&& Ypos > 460 && Ypos < 560){
			txtProp[5]->setColor(glm::vec3(0.2f, 1.0f, 0.2f));
			txtProp[5]->setUniformVec3Color("color_input");
			*txtIndex = 5;
			return;
		}
	}

	//Select boxes==============================================================
	*boxIndex = -1;

	screenPosToWorldRay(Xpos, Ypos, WINDOW_WIDTH, WINDOW_HEIGHT, cam->getProj()*cam->getView(), rayOri, rayDir);

	for(int i = 0; i < box.size(); ++i){
		box[i]->setColor(glm::vec3(0.3f, 0.3f, 1.0f));
		box[i]->setUniformVec3Color("color_input");
	}

	for(int i = 0; i < box.size(); ++i){
		if(TestRayOBBIntersection(rayOri, rayDir,
								  glm::mat3(box[i]->getScal())*glm::vec3(-1, -1, -1),
								  glm::mat3(box[i]->getScal())*glm::vec3(1, 1, 1),
								  box[i]->getTrans()*box[i]->getRot()))
		{
			box[i]->setColor(glm::vec3(1.0f, 0.3f, 0.3f));
			box[i]->setUniformVec3Color("color_input");

			loadProp(box[i], txtProp, 15);
			*boxIndex = i;
			break;
		}
	}
}


static void setBoxProperties(vector<skeleton3d*> &box, text2d* txtProp[], const int boxIndex, const int txtIndex)
{
	glm::vec3 v;

	if(txtIndex != -1 && boxIndex != -1){
		switch(txtIndex){
			case 0:
				v = box[boxIndex]->getScalXYZ();
				v.x += 10*scrollDir;
				box[boxIndex]->setScal(v);
				break;

			case 1:
				v = box[boxIndex]->getScalXYZ();
				v.y += 10*scrollDir;
				box[boxIndex]->setScal(v);
				break;

			case 2:
				v = box[boxIndex]->getScalXYZ();
				v.z += 10*scrollDir;
				box[boxIndex]->setScal(v);
				break;

			case 3:
				v = box[boxIndex]->getTransXYZ();
				v.x += 10*scrollDir;
				box[boxIndex]->setTrans(v);
				break;

			case 4:
				v = box[boxIndex]->getTransXYZ();
				v.y += 10*scrollDir;
				box[boxIndex]->setTrans(v);
				break;
			case 5:
				v = box[boxIndex]->getTransXYZ();
				v.z += 10*scrollDir;
				box[boxIndex]->setTrans(v);
				break;
		};
		box[boxIndex]->setUniformMat4Model("M");
		loadProp(box[boxIndex], txtProp, 15);
	}
}


static void addBox(vector<skeleton3d*> &box, vector<glm::vec3> &vertexData)
{
	skeleton3d* b = new skeleton3d(loadShader("./shader/vs.txt", "./shader/fs.txt"), vertexData);
	b->setColor(glm::vec3(0.3f, 0.3f, 1.0f));
	b->setUniformVec3Color("color_input");
	b->setScal(glm::scale(glm::mat4(), glm::vec3(100.0f, 100.0f, 100.0f)));
	b->setTrans(glm::translate(glm::mat4(), glm::vec3(0.0f, 0.0f, 0.0f)));
	b->setUniformMat4Model("M");

	box.push_back(b);
}


/*====================================
Mouse wheel scroll callback
====================================*/
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	isScroll = true;
	scrollDir = yoffset;
}
