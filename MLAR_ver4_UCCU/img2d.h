#ifndef IMG2D_H
#define IMG2D_H

#include "visibleObj.h"

class img2d : public visibleObj
{
private:
	bool isLoaded;

public:
	img2d(GLuint program, const char* filename, bool isDDS);
	virtual ~img2d();

	void loadText(const char* filename, bool isDDS);
	void show(int x, int y, int size);
	bool is_loaded();

	virtual void render();
	virtual void releaseBuffer();
};

#endif
