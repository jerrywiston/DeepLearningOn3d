#ifndef TEXT2D_H
#define TEXT2D_H

#include "visibleObj.h"

class text2d : public visibleObj
{
private:
	unsigned int vertexSize;
	bool isLoaded;

public:
	text2d(GLuint program, const char* filename, bool isDDS);
	virtual ~text2d();

	void loadText(const char* filename, bool isDDS);
	void print(const char* text, int x, int y, int size);

	bool is_loaded();

	virtual void render();
	virtual void releaseBuffer();
};

#endif
