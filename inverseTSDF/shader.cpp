#include "shader.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <string>

using namespace std;

static string readFile(const char* filename)
{
	string data;
	ifstream ifs(filename, std::ios::in);
	
	if(ifs.is_open()){
		string line = "";
		
		while(getline(ifs, line)) data += "\n" + line;
		ifs.close();
		return data;
	}
	else{
		fprintf(stderr, "Cannot open the file: %s\n", filename);
		return "";
	}
}


GLuint loadShader(const char* path_vs, const char* path_fs)
{
	int status, maxLen;
	char* infoLog = nullptr;
	
	//Read code of shaders=====================================
	string data_vs = readFile(path_vs);
	string data_fs = readFile(path_fs);
	
	//Compile vertex shader====================================
	printf("Compiling vertex shader: %s\n", path_vs);
	
	GLuint vs = glCreateShader(GL_VERTEX_SHADER);
	char const* data_vs_ptr = data_vs.c_str();
	
	glShaderSource(vs, 1, (const GLchar**)&data_vs, nullptr);
	glCompileShader(vs);
	
	glGetShaderiv(vs, GL_COMPILE_STATUS, &status);
	if(status == GL_FALSE){
		glGetShaderiv(vs, GL_INFO_LOG_LENGTH, &maxLen);
		
		infoLog = new char[maxLen];
		glGetShaderInfoLog(vs, maxLen, &maxLen, infoLog);
		fprintf(stderr, "Vertex shader error: %s\n", infoLog);
		
		delete [] infoLog;
		return 0;
	}
	
	//Compile fragment shader==================================
	printf("Compiling fragment shader: %s\n", path_fs);
	
	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
	char const*	data_fs_ptr = data_fs.c_str();
	
	glShaderSource(fs, 1, (const GLchar**)&data_fs, nullptr);
	glCompileShader(fs);
	
	glGetShaderiv(fs, GL_COMPILE_STATUS, &status);
	if(status == GL_FALSE){
		glGetShaderiv(fs, GL_INFO_LOG_LENGTH, &maxLen);
		
		infoLog = new char[maxLen];
		glGetShaderInfoLog(fs, maxLen, &maxLen, infoLog);
		fprintf(stderr, "Fragment shader error: %s\n", infoLog);
		
		delete [] infoLog;
		return 0;
	}
	
	//Create program===========================================
	GLuint program = glCreateProgram();
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	
	glGetProgramiv(program, GL_LINK_STATUS, &status);
	if(status == GL_FALSE){
		glGetProgramiv(program, GL_INFO_LOG_LENGTH, &maxLen);
		
		infoLog = new char[maxLen];
		glGetProgramInfoLog(program, maxLen, &maxLen, infoLog);
		fprintf(stderr, "Link error: %s\n", infoLog);
		
		delete [] infoLog;
		return 0;
	}
	
	printf("Compile shaders successfully!!\n");
	
	return program;
}