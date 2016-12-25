#include "visibleObj.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace glm;


/*====================================
Load BMP texture
====================================*/
GLuint visibleObj::loadBMP(const char* filename)
{
	//Open bmp file===================================
	size_t s;
	FILE* fp = fopen(filename, "rb");
	if(!fp){
		printf("Failed to load BMP texture\n");
		return -1;
	}
	
	GLuint textureID;
	unsigned char header[54];
	unsigned int dataPos;
	unsigned int imgSize;
	unsigned int width;
	unsigned int height;
	unsigned int bits;
	unsigned char* data;
	
	printf("Loading BMP texture: %s\n", filename);
	
	//Read the header (54 bit)========================
	if(fread(header, 1, 54, fp) != 54){
		printf("Not a correct BMP file\n");
		return -1;
	}
	
	//BMP begins with "BM"============================
	if(header[0] != 'B' || header[1] != 'M'){
		printf("Not a correct BMP file\n");
		return -1;
	}
	
	//Read the info about the image===================
	dataPos 	= *(int*)&(header[0x0A]);
	imgSize 	= *(int*)&(header[0x22]);
	width 		= *(int*)&(header[0x12]);
	height 		= *(int*)&(header[0x16]);
	bits		= *(int*)&(header[0x1C]);
	
	//Some BMP are misformatted, guess missing info
	if(imgSize == 0) imgSize = width * height * 3;
	if(dataPos == 0) dataPos = 54;
	
	//Read the actual data=============================
	data = new unsigned char [imgSize];
	s = fread(data , 1, imgSize, fp);
	fclose(fp);
	
	//Generate a texture===============================
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	
	GLenum format = (bits == 24) ? GL_BGR : GL_BGRA;
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0
				, format, GL_UNSIGNED_BYTE, data);
	
	delete [] data;
	
	//Trilinear filtering===============================
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glGenerateMipmap(GL_TEXTURE_2D);

	printf("Load BMP texture successfully!!\n");
	return textureID;
}


/*====================================
Load DDS texture
====================================*/
GLuint visibleObj::loadDDS(const char* filename)
{
	//Open dds file===============================
	size_t s;
	FILE* fp = fopen(filename, "rb");
	if(!fp){
		printf("Failed to load DDS texture\n");
		return -1;
	}
	
	GLuint textureID;
	unsigned char header[124];
	char fileCode[4];
	unsigned char *data = nullptr;
	unsigned int width, height;
	unsigned int dataSize;
	unsigned int linearSize;
	unsigned int blockSize;
	unsigned int offset = 0;
	unsigned int fourCC;
	unsigned int mipMapCount;
	unsigned int format;
	
	printf("Loading DDS texture: %s\n", filename);

	//Verify the type=============================
	s = fread(fileCode, 1, 4, fp);
	if(strncmp(fileCode, "DDS ", 4) != 0){
		printf("Wrong DDS file type\n");
		fclose(fp);
		return -1;
	}
	
	//Get the surface desciption==================
	s = fread(&header, 124, 1, fp);
	height 	 	= *(unsigned int*)&(header[8]);
	width 		= *(unsigned int*)&(header[12]);
	linearSize 	= *(unsigned int*)&(header[16]);
	mipMapCount = *(unsigned int*)&(header[24]);
	fourCC 		= *(unsigned int*)&(header[80]);
	
	//Calculate size & read data===================
	dataSize = (mipMapCount > 1) ? linearSize*2 : linearSize;
	data = new unsigned char[dataSize];
	s = fread(data, 1, dataSize, fp);
	fclose(fp);
	
	//Switch format================================
	switch(fourCC){
		case FOURCC_DXT1:
			format = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT;
			break;
		case FOURCC_DXT3:
			format = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT;
			break;
		case FOURCC_DXT5:
			format = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
			break;
		default:
			break;
	}
	
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	
	blockSize = (format == GL_COMPRESSED_RGBA_S3TC_DXT1_EXT) ? 8 : 16;
	
	//Bind the newly created texture
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		
	//Load the mipmaps
	for(unsigned int level = 0; level < mipMapCount && (width || height); ++level){
		unsigned int size = ((width+3)/4) * ((height+3)/4) * blockSize;
		
		glCompressedTexImage2D(GL_TEXTURE_2D, level, format, width, height, 0, size, data+offset);
	
		offset += size;
		width /= 2;
		height /= 2;
	
		//Deal with non-power-of-two textures
		if(width < 1) width = 1;
		if(width < 1) height = 1;
	}
	
	free(data);
	
	printf("Load DDS texture successfully!!\n");
	return textureID;
}


/*====================================
Upload position array (vbo[0])
====================================*/
void visibleObj::uploadPos(const std::vector<vec3> &vertex, const unsigned int attribNum)
{
	glBindBuffer(GL_ARRAY_BUFFER, this->vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, vertex.size()*sizeof(vec3), &vertex[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(attribNum);
	glVertexAttribPointer(attribNum, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
}


void visibleObj::uploadPos(const std::vector<vec2> &vertex, const unsigned int attribNum)
{
	glBindBuffer(GL_ARRAY_BUFFER, this->vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, vertex.size()*sizeof(vec2), &vertex[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(attribNum);
	glVertexAttribPointer(attribNum, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
}


/*====================================
Upload texture array (vbo[1])
====================================*/
void visibleObj::uploadUV(const std::vector<vec2> &uv, const unsigned int attribNum)
{
	unsigned int width, height;
	
	glBindBuffer(GL_ARRAY_BUFFER, this->vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, uv.size()*sizeof(vec2), &uv[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(attribNum);
	glVertexAttribPointer(attribNum, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
}


/*====================================
Upload normal array (vbo[2])
====================================*/
void visibleObj::uploadNormal(const std::vector<vec3> &normal, const unsigned int attribNum)
{
	glBindBuffer(GL_ARRAY_BUFFER, this->vbo[2]);
	glBufferData(GL_ARRAY_BUFFER, normal.size()*sizeof(vec3), &normal[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(attribNum);
	glVertexAttribPointer(attribNum, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
}


/*====================================
Upload index array (vbo[3])
====================================*/
void visibleObj::uploadVBOIndex(const std::vector<unsigned int> &index)
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vbo[3]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, index.size()*sizeof(unsigned int), &index[0], GL_STATIC_DRAW);
	this->indexCount = index.size();
}


/*====================================
Access program
====================================*/
void visibleObj::setProgram(GLuint program)
{
	this->program = program;
}


GLuint visibleObj::getProgram()
{
	return program;
}
