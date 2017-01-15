#ifndef IMG2D_H
#define IMG2D_H

#include "visibleObj.h"

class img2d : public visibleObj
{
private:
	bool isLoaded;

public:
	img2d();
	img2d(GLuint program, const char* filename, bool isDDS);
	img2d(GLuint program, unsigned char* imgData, unsigned int width, unsigned int height);

	virtual ~img2d();

	void loadText(const char* filename, bool isDDS);
	void uploadImg(unsigned char* imgData, unsigned int width, unsigned int height);
	void show(int x, int y, unsigned int width, unsigned int height);
	bool is_loaded();

	virtual void render();
	virtual void releaseBuffer();
};

#endif
