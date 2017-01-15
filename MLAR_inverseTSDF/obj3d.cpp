#include "obj3d.h"
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <map>
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

using namespace glm;

/*====================================
Constructor
====================================*/
obj3d::obj3d()
{
	this->program = -1;
}


obj3d::obj3d(GLuint program, const char* filename_obj, const char* filename_texture, const bool isDDS)
{
	this->program = program;
	loadObj(filename_obj, filename_texture, isDDS);
}


/*====================================
Destructor
====================================*/
obj3d::~obj3d()
{
	releaseBuffer();
}


/*====================================
Load the object
====================================*/
bool obj3d::loadObj(const char* filename_obj, const char* filename_texture, const bool isDDS)
{
	std::vector<unsigned int> vertexIndex, uvIndex, normalIndex, index;
	std::vector<vec3> vertex, vertex_out, indexed_vertex;
	std::vector<vec2> uv, uv_out, indexed_uv;
	std::vector<vec3> normal, normal_out, indexed_normal;

	printf("Loading the obj file: %s\n", filename_obj);

	//Open obj file=================================
	FILE* fp = fopen(filename_obj, "r");
	if(!fp){
		printf("Cannot open the obj file\n");
		isLoaded = false;
		return false;
	}

	//Parse data====================================
	while(1){
		char lineHeader[128];

		//Read the 1st word of the line
		int res = fscanf(fp, "%s", lineHeader);
		if(res == EOF) break;

		//Read data
		//Vertex====================================
		if(strcmp(lineHeader, "v") == 0){
			vec3 vertex_tmp;
			res = fscanf(fp, "%f %f %f\n", &vertex_tmp.x, &vertex_tmp.y, &vertex_tmp.z);
			vertex.push_back(vertex_tmp);
		}
		//UV========================================
		else if(strcmp(lineHeader, "vt") == 0){
			vec2 uv_tmp;
			res = fscanf(fp, "%f %f\n", &uv_tmp.x, &uv_tmp.y);
			if(isDDS) uv_tmp.y = -uv_tmp.y;
			uv.push_back(uv_tmp);
		}
		//Normal====================================
		else if(strcmp(lineHeader, "vn") == 0){
			vec3 normal_tmp;
			res = fscanf(fp, "%f %f %f\n", &normal_tmp.x, &normal_tmp.y, &normal_tmp.z);
			normal.push_back(normal_tmp);
		}
		//Index=====================================
		else if(strcmp(lineHeader, "f") == 0){
			unsigned int vIndex[3], uIndex[3], nIndex[3];
			int matches = fscanf(fp, "%d/%d/%d %d/%d/%d %d/%d/%d\n"
									,&vIndex[0], &uIndex[0], &nIndex[0]
									,&vIndex[1], &uIndex[1], &nIndex[1]
									,&vIndex[2], &uIndex[2], &nIndex[2]
								);
			if(matches != 9){
				printf("File cannot be read by this parser\n");
				return false;
			}

			vertexIndex.push_back(vIndex[0]);
			vertexIndex.push_back(vIndex[1]);
			vertexIndex.push_back(vIndex[2]);
			uvIndex.push_back(uIndex[0]);
			uvIndex.push_back(uIndex[1]);
			uvIndex.push_back(uIndex[2]);
			normalIndex.push_back(nIndex[0]);
			normalIndex.push_back(nIndex[1]);
			normalIndex.push_back(nIndex[2]);
		}
		//Probably a comment, eat up the rest of line
		else{
			char c;
			do{
				c = fgetc(fp);
			}while(c != '\n' && c != EOF);
		}
	}

	//For each vertex of each triangle
	for(unsigned int i = 0; i < vertexIndex.size(); i++){
		//Get the attribbutes thanks to the index
		vec3 v = vertex[vertexIndex[i]-1];
		vec2 u = uv[uvIndex[i]-1];
		vec3 n = normal[normalIndex[i]-1];

		//Put the attribbutes in buffers
		vertex_out.push_back(v);
		uv_out.push_back(u);
		normal_out.push_back(n);
	}

	indexVBO(vertex_out, uv_out, normal_out, index, indexed_vertex, indexed_uv, indexed_normal);

	printf("Uploading obj data...\n");

	//Create VAO/VBO===========================
	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
	texture = (isDDS) ? loadDDS(filename_texture) : loadBMP(filename_texture);
	glBindVertexArray(this->vao);

	//Upload position array====================
	if(vertexIndex.size() > 0){
		uploadPos(indexed_vertex, 0);
	}
	else{
		printf("Failed to upload obj data\n");
		isLoaded = false;
		return false;
	}

	//Upload texCoord array====================
	if(uvIndex.size() > 0)
		uploadUV(indexed_uv, 1);

	//Upload normal array======================
	if(normalIndex.size() > 0)
		uploadNormal(indexed_normal, 2);

	//Setup index buffer for glDrawElements====
	if(vertexIndex.size() > 0)
		uploadVBOIndex(index);

	glBindVertexArray(0);
	fclose(fp);
	printf("Load the obj file successfully!!\n");
	isLoaded = true;
	return true;
}


/*====================================
Return true if v1 can be considered equal to v2
====================================*/
bool obj3d::is_near(float v1, float v2)
{
	return fabs(v1 - v2) < 0.01f;
}


/*====================================
Search through all already-exported vertices for a similar one
Similar = same position + same UVs + same normal
====================================*/
bool obj3d::getSimilarVertexIndex(
	packedVertex &packed,
	std::map<packedVertex, unsigned int> &vertexToOutIndex,
	unsigned int &result
){
	std::map<packedVertex, unsigned int>::iterator it = vertexToOutIndex.find(packed);

	if(it == vertexToOutIndex.end()) return false;

	result = it->second;
	return true;
}


/*====================================
index VBO
====================================*/
void obj3d::indexVBO(
	std::vector<vec3> &in_vertex,
	std::vector<vec2> &in_uv,
	std::vector<vec3> &in_normal,

	std::vector<unsigned int> &out_index,
	std::vector<vec3> &out_vertex,
	std::vector<vec2> &out_uv,
	std::vector<vec3> &out_normal
){
	std::map<packedVertex, unsigned int> vertexToOutIndex;

	for(unsigned int i = 0; i < in_vertex.size(); i++){
		packedVertex packed = {in_vertex[i], in_uv[i], in_normal[i]};

		//Try to find a similar vertex
		unsigned int index;
		bool found = getSimilarVertexIndex(packed, vertexToOutIndex, index);

		if(found) out_index.push_back(index);
		else{
			out_vertex.push_back(in_vertex[i]);
			out_uv.push_back(in_uv[i]);
			out_normal.push_back(in_normal[i]);

			unsigned int newIndex = (unsigned int)out_vertex.size() - 1;
			out_index.push_back(newIndex);
			vertexToOutIndex[packed] = newIndex;
		}
	}
}


/*====================================
If the object is loaded
====================================*/
bool obj3d::is_loaded()
{
	return isLoaded;
}


/*====================================
Render the object
====================================*/
void obj3d::render()
{
	glUseProgram(program);
	glBindVertexArray(vao);
	glBindTexture(GL_TEXTURE_2D, texture);
	glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
}


/*====================================
Release buffer data
====================================*/
void obj3d::releaseBuffer()
{
	glDeleteVertexArrays(1, &vao);
	glDeleteTextures(1, &texture);
	glDeleteBuffers(4, vbo);
}
