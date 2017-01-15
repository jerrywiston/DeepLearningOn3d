#include "voxelGrid.h"

/*==================================
Volumetric fusion CUDA
==================================*/
__global__ void fusion_CUDA(voxel *vox, vec *vertexMap, vec *trans, mat *rot, vec *offset, const int n)
{
	//Compute the index of the voxel
	const int index_vol = n*RESOL_VOL*RESOL_VOL + blockIdx.x * 512 + threadIdx.x;
	const int i = index_vol / (RESOL_VOL*RESOL_VOL);
	const int j = (index_vol % (RESOL_VOL*RESOL_VOL)) / RESOL_VOL;
	const int k = (index_vol % (RESOL_VOL*RESOL_VOL)) % RESOL_VOL;

	//Add offset to the voxel
	int a = i + offset->z;
	int b = j + offset->y;
	int c = k + offset->x;
	const int index = a * RESOL_Y*RESOL_Z + b * RESOL_Z + c;

	//Varify the condition
	if (a < 0 || a > RESOL_X ||
		b < 0 || b > RESOL_Y ||
		c < 0 || c > RESOL_Z ||
		(vox[index].isActive && vox[index].weight > 8)) return;

	vec tmp, point_vox, dist;

	//Voxel centers in view space
	tmp.x = vox[index].pos.x - trans->x;
	tmp.y = vox[index].pos.y - trans->y;
	tmp.z = vox[index].pos.z - trans->z;

	point_vox.x = rot->x1 * tmp.x + rot->y1 * tmp.y + rot->z1 * tmp.z;
	point_vox.y = rot->x2 * tmp.x + rot->y2 * tmp.y + rot->z2 * tmp.z;
	point_vox.z = rot->x3 * tmp.x + rot->y3 * tmp.y + rot->z3 * tmp.z;

	//Projected onto the img plane
	if (point_vox.z > 0){
		int x = (int)(point_vox.x * FOCAL_LEN / point_vox.z + IMG_WIDTH/2);
		int y = (int)(point_vox.y * -FOCAL_LEN / point_vox.z + IMG_HEIGHT/2);
		int pix = IMG_WIDTH * y + x;

		//The pixel is within the depth map
		if (x > 0 && x < IMG_WIDTH && y > 0 && y < IMG_HEIGHT){
			//Point cloud in view space
			dist.x = vertexMap[pix].x - point_vox.x;
			dist.y = vertexMap[pix].y - point_vox.y;
			dist.z = vertexMap[pix].z - point_vox.z;

			//Compute SDF
			float norm = sqrtf(dist.x * dist.x + dist.y * dist.y + dist.z * dist.z);
			float tsdf = (dist.z > 0) ? norm / TRUNCATE : -norm / TRUNCATE;

			//Truncate SDF
			if (tsdf >= 1)
				vox[index].tsdf = (vox[index].weight * vox[index].tsdf + 1) / (vox[index].weight + 1);
			else if (tsdf > -1){
				vox[index].tsdf = (vox[index].weight * vox[index].tsdf + tsdf) / (vox[index].weight + 1);
				vox[index].isActive = true;
			}
			else
				vox[index].tsdf = (vox[index].weight * vox[index].tsdf - 1) / (vox[index].weight + 1);

			++vox[index].weight;
		}
	}
}


/*==================================
Ray-casting for a depth map CUDA
==================================*/
__global__ void rayCast_CUDA(voxel *vox, vec *vertexMap, vec *vertexMap_fused, vec *bound1, vec *bound2, vec *center, vec *trans, mat *rot)
{
	//Compute the index of the pixel
	const int x = (blockIdx.x * 512 + threadIdx.x) % IMG_WIDTH;
	const int y = (blockIdx.x * 512 + threadIdx.x) / IMG_WIDTH;
	const int pix = IMG_WIDTH * y + x;

	vec tmp, rayDir, rayPos, rayStep;
	float norm;
	float tsdf_prev, tsdf_cur = 1;
	bool isActive_prev, isActive_cur = false;

	//Compute ray direction & position
	tmp.x = (x - IMG_WIDTH/2) / FOCAL_LEN;
	tmp.y = (IMG_HEIGHT/2 - y) / FOCAL_LEN;
	tmp.z = 1;

	rayDir.x = rot->x1 * tmp.x + rot->x2 * tmp.y + rot->x3 * tmp.z;
	rayDir.y = rot->y1 * tmp.x + rot->y2 * tmp.y + rot->y3 * tmp.z;
	rayDir.z = rot->z1 * tmp.x + rot->z2 * tmp.y + rot->z3 * tmp.z;

	norm = sqrtf(rayDir.x * rayDir.x + rayDir.y * rayDir.y + rayDir.z * rayDir.z);

	//Stepping value per iteration
	rayStep.x = STEP * rayDir.x / norm;
	rayStep.y = STEP * rayDir.y / norm;
	rayStep.z = STEP * rayDir.z / norm;

	rayPos.x = center->x + rayStep.x;
	rayPos.y = center->y + rayStep.y;
	rayPos.z = center->z + rayStep.z;

	//Start casting
	//If rayPos is in the volume
	while(rayPos.x > VOX_LEN && rayPos.x < MAX_LEN_X
	   && rayPos.y > VOX_LEN && rayPos.y < MAX_LEN_Y
	   && rayPos.z > VOX_LEN && rayPos.z < MAX_LEN_Z){

		int index = ((int)(rayPos.z / VOX_LEN)) * RESOL_Y*RESOL_Z + ((int)(rayPos.y / VOX_LEN)) * RESOL_Z + ((int)(rayPos.x / VOX_LEN));

		tsdf_prev = tsdf_cur;
		tsdf_cur = vox[index].tsdf;

		isActive_prev = isActive_cur;
		isActive_cur = vox[index].isActive;

		//Detect a zero-crossing
		if (tsdf_cur * tsdf_prev < 0 && (isActive_cur || isActive_prev)){
			//Interpolation & Transform to the view space
			tmp.x = vox[index].pos.x - vox[index].tsdf * rayDir.x - trans->x;
			tmp.y = vox[index].pos.y - vox[index].tsdf * rayDir.y - trans->y;
			tmp.z = vox[index].pos.z - vox[index].tsdf * rayDir.z - trans->z;

			vertexMap_fused[pix].x = rot->x1 * tmp.x + rot->y1 * tmp.y + rot->z1 * tmp.z;
			vertexMap_fused[pix].y = rot->x2 * tmp.x + rot->y2 * tmp.y + rot->z2 * tmp.z;
			vertexMap_fused[pix].z = rot->x3 * tmp.x + rot->y3 * tmp.y + rot->z3 * tmp.z;
			return;
		}
		rayPos.x += rayStep.x;
		rayPos.y += rayStep.y;
		rayPos.z += rayStep.z;
	}

	//If the ray didn't hit a point, use the raw data
	vertexMap_fused[pix].x = vertexMap[pix].x;
	vertexMap_fused[pix].y = vertexMap[pix].y;
	vertexMap_fused[pix].z = vertexMap[pix].z;
}


/*==================================
Ray-casting (All: x direction)
==================================*/
__global__ void rayCastAll_X_CUDA(voxel *vox, vec *pc, unsigned int *pc_count)
{
	//Compute the index of the voxel
	const int a = (blockIdx.x * 512 + threadIdx.x) / RESOL_Z;
	const int b = (blockIdx.x * 512 + threadIdx.x) % RESOL_Z;

	float tsdf_prev, tsdf_cur = 0;
	bool isActive_prev, isActive_cur = false;
	int index_tmp, index;

	index_tmp = a * RESOL_Z + b;

	//Start casting
	for (int c = 1; c < RESOL_X; ++c){
		index = index_tmp + c * RESOL_Y*RESOL_Z;

		tsdf_prev = tsdf_cur;
		tsdf_cur = vox[index].tsdf;

		isActive_prev = isActive_cur;
		isActive_cur = vox[index].isActive;

		//Detect a zero-crossing
		if ((isActive_cur || isActive_prev) && tsdf_cur * tsdf_prev < 0 && *pc_count < MAX_PC_COUNT){
			//Interpolation
			vec p = vox[index].pos;
			p.x -= vox[index].tsdf;

			pc[atomicAdd(pc_count, 1)] = p;
		}
	}
}


/*==================================
Ray-casting (All: y direction)
==================================*/
__global__ void rayCastAll_Y_CUDA(voxel *vox, vec *pc, unsigned int *pc_count)
{
	//Compute the index of the voxel
	const int a = (blockIdx.x * 512 + threadIdx.x) / RESOL_X;
	const int b = (blockIdx.x * 512 + threadIdx.x) % RESOL_X;

	float tsdf_prev, tsdf_cur = 0;
	bool isActive_prev, isActive_cur = false;
	int index_tmp, index;

	index_tmp = a * RESOL_Y*RESOL_Z + b;

	//Start casting
	for (int c = 1; c < RESOL_Y; ++c){
		index = index_tmp + c * RESOL_Z;

		tsdf_prev = tsdf_cur;
		tsdf_cur = vox[index].tsdf;

		isActive_prev = isActive_cur;
		isActive_cur = vox[index].isActive;

		//Detect a zero-crossing
		if ((isActive_cur || isActive_prev) && tsdf_cur * tsdf_prev < 0 && *pc_count < MAX_PC_COUNT){
			//Interpolation
			vec p = vox[index].pos;
			p.y -= vox[index].tsdf;

			pc[atomicAdd(pc_count, 1)] = p;
		}
	}
}


/*==================================
Ray-casting (All: z direction)
==================================*/
__global__ void rayCastAll_Z_CUDA(voxel *vox, vec *pc, unsigned int *pc_count)
{
	//Compute the index of the voxel
	const int a = (blockIdx.x * 512 + threadIdx.x) / RESOL_X;
	const int b = (blockIdx.x * 512 + threadIdx.x) % RESOL_X;

	float tsdf_prev, tsdf_cur = 0;
	bool isActive_prev, isActive_cur = false;
	int index_tmp, index;

	index_tmp = a * RESOL_Y*RESOL_Z + b * RESOL_Z;

	//Start casting
	for (int c = 1; c < RESOL_Z; ++c){
		index = index_tmp + c;

		tsdf_prev = tsdf_cur;
		tsdf_cur = vox[index].tsdf;

		isActive_prev = isActive_cur;
		isActive_cur = vox[index].isActive;

		//Detect a zero-crossing
		if ((isActive_cur || isActive_prev) && tsdf_cur * tsdf_prev < 0 && *pc_count < MAX_PC_COUNT){
			//Interpolation
			vec p = vox[index].pos;
			p.z -= vox[index].tsdf;

			pc[atomicAdd(pc_count, 1)] = p;
		}
	}
}


/*======================================
Constructor
======================================*/
voxelGrid::voxelGrid()
{
	//Initialize the properties
	center.x = MAX_LEN_X / 2.0f;
	center.y = MAX_LEN_Y / 2.0f;
	center.z = MAX_LEN_Z / 2.0f;

	dir.x = 0;
	dir.y = 0;
	dir.z = 1;

	trans.x = 0;
	trans.y = 0;
	trans.z = 0;

	rot.x1 = 1; rot.x2 = 0; rot.x3 = 0;
	rot.y1 = 0; rot.y2 = 1; rot.y3 = 0;
	rot.z1 = 0; rot.z2 = 0; rot.z3 = 1;

	//Allocate memory for each voxel
	vox = new voxel[RESOL_X*RESOL_Y*RESOL_Z];
	pc = new vec[MAX_PC_COUNT];

	//Initialize each voxel
	for (int i = 0; i < RESOL_X; i++)
		for (int j = 0; j < RESOL_Y; j++)
			for (int k = 0; k < RESOL_Z; k++){
				int index = i*RESOL_Y*RESOL_Z + j*RESOL_Z + k;

				vox[index].pos.x = k * VOX_LEN + VOX_LEN / 2.f;
				vox[index].pos.y = j * VOX_LEN + VOX_LEN / 2.f,
				vox[index].pos.z = i * VOX_LEN + VOX_LEN / 2.f;
				vox[index].tsdf = 1;
				vox[index].weight = 0;
				vox[index].isActive = false;
			}

	pc_count = 0;

	//Allocate the GPU memory
	cudaMalloc(&d_vox, RESOL_X*RESOL_Y*RESOL_Z * sizeof(voxel));
	cudaMalloc(&d_pc, MAX_PC_COUNT * sizeof(vec));
	cudaMalloc(&d_vertexMap, IMG_WIDTH*IMG_HEIGHT * sizeof(vec));
	cudaMalloc(&d_vertexMap_fused, IMG_WIDTH*IMG_HEIGHT * sizeof(vec));
	cudaMalloc(&d_bound1, sizeof(vec));
	cudaMalloc(&d_bound2, sizeof(vec));
	cudaMalloc(&d_center, sizeof(vec));
	cudaMalloc(&d_trans, sizeof(vec));
	cudaMalloc(&d_rot, sizeof(mat));
	cudaMalloc(&d_offset, sizeof(vec));
	cudaMalloc(&d_pc_count, sizeof(unsigned int));

	//Upload data to the GPU memory
	cudaMemcpy(d_vox, vox, RESOL_X*RESOL_Y*RESOL_Z * sizeof(voxel), cudaMemcpyHostToDevice);
	cudaMemcpy(d_pc, pc, MAX_PC_COUNT * sizeof(vec), cudaMemcpyHostToDevice);
	cudaMemcpy(d_trans, &trans, sizeof(vec), cudaMemcpyHostToDevice);
	cudaMemcpy(d_rot, &rot, sizeof(mat), cudaMemcpyHostToDevice);
	cudaMemcpy(d_offset, &offset, sizeof(vec), cudaMemcpyHostToDevice);
	cudaMemcpy(d_pc_count, &pc_count, sizeof(unsigned int), cudaMemcpyHostToDevice);
}


/*======================================
Destructor
======================================*/
voxelGrid::~voxelGrid()
{
	//Free the GPU memory
	cudaFree(d_vox);
	cudaFree(d_pc);
	cudaFree(d_vertexMap);
	cudaFree(d_vertexMap_fused);
	cudaFree(d_bound1);
	cudaFree(d_bound2);
	cudaFree(d_center);
	cudaFree(d_trans);
	cudaFree(d_rot);
	cudaFree(d_offset);
	cudaFree(d_pc_count);

	delete [] vox;
	delete [] pc;
}


/*======================================
TSDF Fusion
======================================*/
void voxelGrid::fusion(vec *vertexMap)
{
	//Upload data to GPU
	cudaMemcpy(d_vertexMap, vertexMap, IMG_WIDTH*IMG_HEIGHT * sizeof(vec), cudaMemcpyHostToDevice);

	//Start fusion
	//RESOL_VOL = 128
	//for (int j = 0; j < RESOL_VOL; j += 8)
	//	fusion_CUDA << <512, 512 >> > (d_vox, d_vertexMap, d_trans, d_rot, d_offset, j);

	//RESOL_VOL = 192
	//for (int j = 0; j < RESOL_VOL; j += 8)
	//	fusion_CUDA << <576, 512 >> > (d_vox, d_vertexMap, d_trans, d_rot, d_offset, j);

	//RESOL_VOL = 256
	for (int j = 0; j < RESOL_VOL; j += 4)
		fusion_CUDA << <512, 512 >> > (d_vox, d_vertexMap, d_trans, d_rot, d_offset, j);
}


/*======================================
Ray-casting
======================================*/
void voxelGrid::rayCast(vec *vertexMap_fused)
{
	//Compute boundary
	center.x = trans.x;
	center.y = trans.y;
	center.z = trans.z;

	bound1.x = (center.x <= VOX_LEN) ? center.x : VOX_LEN;
	bound1.y = (center.y <= VOX_LEN) ? center.y : VOX_LEN;
	bound1.z = (center.z <= VOX_LEN) ? center.z : VOX_LEN;

	bound2.x = (center.x >= MAX_LEN_X) ? center.x : MAX_LEN_X;
	bound2.y = (center.y >= MAX_LEN_Y) ? center.y : MAX_LEN_Y;
	bound2.z = (center.z >= MAX_LEN_Z) ? center.z : MAX_LEN_Z;

	//Upload data to the GPU memory
	cudaMemcpy(d_center, &center, sizeof(vec), cudaMemcpyHostToDevice);
	cudaMemcpy(d_bound1, &bound1, sizeof(vec), cudaMemcpyHostToDevice);
	cudaMemcpy(d_bound2, &bound2, sizeof(vec), cudaMemcpyHostToDevice);

	//Start ray-casting
	rayCast_CUDA << <168, 512 >> > (d_vox, d_vertexMap, d_vertexMap_fused, d_bound1, d_bound2, d_center, d_trans, d_rot);

	//Download data from the GPU memory
	cudaMemcpy(vertexMap_fused, d_vertexMap_fused, IMG_WIDTH*IMG_HEIGHT * sizeof(vec), cudaMemcpyDeviceToHost);
}


/*======================================
Ray-casting (All: x, y, z direction)
======================================*/
void voxelGrid::rayCastAll(std::vector<vec> &pcData)
{
	//Upload data to GPU
	pc_count = 0;
	cudaMemcpy(d_pc_count, &pc_count, sizeof(unsigned int), cudaMemcpyHostToDevice);

	rayCastAll_X_CUDA<<<256, 512>>>(d_vox, d_pc, d_pc_count);
	rayCastAll_Y_CUDA<<<512, 512>>>(d_vox, d_pc, d_pc_count);
	rayCastAll_Z_CUDA<<<256, 512>>>(d_vox, d_pc, d_pc_count);

	//Download data from GPU
	cudaMemcpy(&pc_count, d_pc_count, sizeof(unsigned int), cudaMemcpyDeviceToHost);
	cudaMemcpy(pc, d_pc, pc_count * sizeof(vec), cudaMemcpyDeviceToHost);

	for(int i = 0; i < pc_count; ++i)
		pcData.push_back(pc[i]);
}


/*======================================
Ray-casting (All: only z direction)
======================================*/
void voxelGrid::rayCastAll_approx(std::vector<vec> &pcData)
{
	//Upload data to GPU
	pc_count = 0;
	cudaMemcpy(d_pc_count, &pc_count, sizeof(unsigned int), cudaMemcpyHostToDevice);

	rayCastAll_X_CUDA<<<208, 512>>>(d_vox, d_pc, d_pc_count);

	//Download data from GPU
	cudaMemcpy(&pc_count, d_pc_count, sizeof(unsigned int), cudaMemcpyDeviceToHost);
	cudaMemcpy(pc, d_pc, pc_count * sizeof(vec), cudaMemcpyDeviceToHost);

	//pcData.insert(pcData.being(), pc, pc+pc_count);

	for(int i = 0; i < pc_count; ++i)
		pcData.push_back(pc[i]);
}


/*======================================
Set translation vector
======================================*/
void voxelGrid::setTrans(float x, float y, float z)
{
	trans.x = x;
	trans.y = y;
	trans.z = z;

	cudaMemcpy(d_trans, &trans, sizeof(vec), cudaMemcpyHostToDevice);
}

void voxelGrid::setTrans(vec t)
{
	trans = t;
	cudaMemcpy(d_trans, &trans, sizeof(vec), cudaMemcpyHostToDevice);
}


/*======================================
Set rotation matrix
======================================*/
void voxelGrid::setRot(float x1, float x2, float x3,
					   float y1, float y2, float y3,
					   float z1, float z2, float z3)
{
	rot.x1 = x1; rot.x2 = x2; rot.x3 = x3;
	rot.y1 = y1; rot.y2 = y2; rot.y3 = y3;
	rot.z1 = z1; rot.z2 = z2; rot.z3 = z3;

	cudaMemcpy(d_rot, &rot, sizeof(mat), cudaMemcpyHostToDevice);
}

void voxelGrid::setRot(mat r)
{
	rot = r;
	cudaMemcpy(d_rot, &rot, sizeof(mat), cudaMemcpyHostToDevice);
}


/*======================================
Compute offset
======================================*/
void voxelGrid::computeOffset()
{
	vec dir_cur;

	//Rotate the orientation of the camera
	dir_cur.x = rot.x1 * dir.x + rot.x2 * dir.y + rot.x3 * dir.z;
	dir_cur.y = rot.y1 * dir.x + rot.y2 * dir.y + rot.y3 * dir.z;
	dir_cur.z = rot.z1 * dir.x + rot.z2 * dir.y + rot.z3 * dir.z;

	//Compute the offset of the voxel index
	offset.x = int((trans.x + SHIFT * dir_cur.x - MAX_LEN_VOL/2) / VOX_LEN);
	offset.y = int((trans.y + SHIFT * dir_cur.y - MAX_LEN_VOL/2) / VOX_LEN);
	offset.z = int((trans.z + SHIFT * dir_cur.z - MAX_LEN_VOL/2) / VOX_LEN);

	//Upload data to the GPU memory
	cudaMemcpy(d_offset, &offset, sizeof(vec), cudaMemcpyHostToDevice);
}


/*======================================
Get volume offset
======================================*/
vec voxelGrid::getOffsetTrans()
{
	vec offsetTrans;
	offsetTrans.x = offset.x * VOX_LEN;
	offsetTrans.y = offset.y * VOX_LEN;
	offsetTrans.z = offset.z * VOX_LEN;

	return offsetTrans;
}


/*====================================
Compute TSDF from the point cloud
====================================*/
void voxelGrid::inverseTSDF(std::vector<vec> &pc)
{
	vec vertexMap[IMG_WIDTH*IMG_HEIGHT];

	for(int k = 0; k < 16; ++k)
	for(int j = 0; j < 16; ++j){
		for(int i = 0; i < IMG_WIDTH*IMG_HEIGHT; ++i){
			vertexMap[i].x = 0;
			vertexMap[i].y = 0;
			vertexMap[i].z = 0;
		}

		setTrans(256*j, 256*k, 2048);
		setRot(1, 0, 0,
			   0, 1, 0,
		       0, 0, 1);

		for(int i = 0; i < pc.size(); ++i){
			vec tmp, point, point_viewSpace;

			point = pc[i];

			//Varify the condition
			if(point.x < 0 || point.x > MAX_LEN_X ||
			   point.y < 0 || point.y > MAX_LEN_Y ||
		   	   point.z < 0 || point.z > MAX_LEN_Z) continue;

			//Point in view space
			tmp.x = point.x - trans.x;
			tmp.y = point.y - trans.y;
			tmp.z = point.z - trans.z;

			point_viewSpace.x = rot.x1 * tmp.x + rot.y1 * tmp.y + rot.z1 * tmp.z;
			point_viewSpace.y = rot.x2 * tmp.x + rot.y2 * tmp.y + rot.z2 * tmp.z;
			point_viewSpace.z = rot.x3 * tmp.x + rot.y3 * tmp.y + rot.z3 * tmp.z;

			//Projected onto the img plane
			if(point_viewSpace.z > 0){
				int x = (int)(point_viewSpace.x * FOCAL_LEN / point_viewSpace.z + IMG_WIDTH/2);
				int y = (int)(point_viewSpace.y * -FOCAL_LEN / point_viewSpace.z + IMG_HEIGHT/2);
				int pix = IMG_WIDTH * y + x;

				//The pixel is within the depth map
				if (x > 0 && x < IMG_WIDTH && y > 0 && y < IMG_HEIGHT){
					vertexMap[pix] = point_viewSpace;
				}
			}
		}

		computeOffset();
		fusion(vertexMap);
	}
}


voxel* voxelGrid::getVox()
{
	//Download voxel data
	cudaMemcpy(vox, d_vox, RESOL_X*RESOL_Y*RESOL_Z * sizeof(voxel), cudaMemcpyDeviceToHost);

	return vox;
}
