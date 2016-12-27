#ifndef SCREEN_HPP
#define SCREEN_HPP

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/norm.hpp"

using namespace glm;

/*====================================
Cast a ray from the screen
====================================*/
void screenPosToWorldRay(float Xpos, float Ypos, int winWidth, int winHeight, mat4 VP, vec3 &rayOrigin, vec3 &rayDir)
{
	Xpos = (Xpos / (float)winWidth - 0.5f) * 2.0f;		//[0, 1024] => [-1, 1]
	Ypos = (0.5f - Ypos / (float)winHeight) * 2.0f;		//[0, 768] => [-1, 1]

	//The ray start & end position, in normalized device coordinates
	vec4 lRayStart_NDC(Xpos, Ypos, 0.0f, 1.0f);
	vec4 lRayEnd_NDC(Xpos, Ypos, 1.0f, 1.0f);

	mat4 VP_inverse = inverse(VP);
	vec4 lRayStart_world = VP_inverse * lRayStart_NDC;
	vec4 lRayEnd_world = VP_inverse * lRayEnd_NDC;

	lRayStart_world /= lRayStart_world.w;
	lRayEnd_world /= lRayEnd_world.w;

	vec3 lRayDir_world = lRayEnd_world - lRayStart_world;

	rayOrigin = vec3(lRayStart_world);
	rayDir = normalize(lRayDir_world);
}


/*====================================
Test the intersection of ray & OBB
====================================*/
bool TestRayOBBIntersection(
	vec3 rayOri,        // Ray origin, in world space
	vec3 rayDir,     	// Ray direction (NOT target position!), in world space. Must be normalize()'d.
	vec3 aabb_min,      // Minimum X,Y,Z coords of the mesh when not transformed at all.
	vec3 aabb_max,      // Maximum X,Y,Z coords. Often aabb_min*-1 if your mesh is centered, but it's not always the case.
	mat4 model          // Transformation applied to the mesh (which will thus be also applied to its bounding box)
){
	// Intersection method from Real-Time Rendering and Essential Mathematics for Games

	float tMin = 0.0f;
	float tMax = 100000.0f;

	vec3 OBBposition_worldspace(model[3].x, model[3].y, model[3].z);
	vec3 delta = OBBposition_worldspace - rayOri;

	// Test intersection with the 2 planes perpendicular to the OBB's X axis
	{
		vec3 xaxis(model[0].x, model[0].y, model[0].z);
		float e = dot(xaxis, delta);
		float f = dot(rayDir, xaxis);

		if ( fabs(f) > 0.001f ){ // Standard case

			float t1 = (e+aabb_min.x)/f; // Intersection with the "left" plane
			float t2 = (e+aabb_max.x)/f; // Intersection with the "right" plane
			// t1 and t2 now contain distances betwen ray origin and ray-plane intersections

			// We want t1 to represent the nearest intersection,
			// so if it's not the case, invert t1 and t2
			if (t1>t2){
				float w=t1;t1=t2;t2=w; // swap t1 and t2
			}

			// tMax is the nearest "far" intersection (amongst the X,Y and Z planes pairs)
			if ( t2 < tMax )
				tMax = t2;
			// tMin is the farthest "near" intersection (amongst the X,Y and Z planes pairs)
			if ( t1 > tMin )
				tMin = t1;

			// And here's the trick :
			// If "far" is closer than "near", then there is NO intersection.
			// See the images in the tutorials for the visual explanation.
			if (tMax < tMin )
				return false;

		}else{ // Rare case : the ray is almost parallel to the planes, so they don't have any "intersection"
			if(-e+aabb_min.x > 0.0f || -e+aabb_max.x < 0.0f)
				return false;
		}
	}


	// Test intersection with the 2 planes perpendicular to the OBB's Y axis
	// Exactly the same thing than above.
	{
		vec3 yaxis(model[1].x, model[1].y, model[1].z);
		float e = dot(yaxis, delta);
		float f = dot(rayDir, yaxis);

		if ( fabs(f) > 0.001f ){

			float t1 = (e+aabb_min.y)/f;
			float t2 = (e+aabb_max.y)/f;

			if (t1>t2){float w=t1;t1=t2;t2=w;}

			if ( t2 < tMax )
				tMax = t2;
			if ( t1 > tMin )
				tMin = t1;
			if (tMin > tMax)
				return false;

		}else{
			if(-e+aabb_min.y > 0.0f || -e+aabb_max.y < 0.0f)
				return false;
		}
	}


	// Test intersection with the 2 planes perpendicular to the OBB's Z axis
	// Exactly the same thing than above.
	{
		vec3 zaxis(model[2].x, model[2].y, model[2].z);
		float e = dot(zaxis, delta);
		float f = dot(rayDir, zaxis);

		if ( fabs(f) > 0.001f ){

			float t1 = (e+aabb_min.z)/f;
			float t2 = (e+aabb_max.z)/f;

			if (t1>t2){float w=t1;t1=t2;t2=w;}

			if ( t2 < tMax )
				tMax = t2;
			if ( t1 > tMin )
				tMin = t1;
			if (tMin > tMax)
				return false;

		}else{
			if(-e+aabb_min.z > 0.0f || -e+aabb_max.z < 0.0f)
				return false;
		}
	}

	return true;
}

#endif
