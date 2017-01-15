#ifndef TENSOR_UTIL
#define TENSOR_UTIL

#include <cstdio>
#include <string>
#include <vector>
#include <stdint.h>
#include "half.hpp"

#define TYPE_HALF   0
#define TYPE_FLOAT  1
#define TYPE_DOUBLE 2
#define TYPE_UINT8  3
#define TYPE_UINT16 4
#define TYPE_UINT32 5
#define TYPE_UINT64 6
#define TYPE_INT8   7
#define TYPE_INT16  8
#define TYPE_INT32  9
#define TYPE_INT64  10
#define TYPE_CHAR   11
#define TYPE_BOOL   12


typedef struct{
    int32_t magic;
    int32_t num;
    int32_t rows;
    int32_t cols;
}imgHeader_t;

typedef struct{
    int32_t magic;
    int32_t num;
}labelHeader_t;

typedef union{
    int32_t data;
    unsigned char b[4];
}buffer32_t;

typedef struct{
    uint8_t type;
    uint32_t size;
    std::string name;
    int32_t numDim;
    int32_t* dims;
    void* value;
}tensor_t;


/*===================================================================
Little endian to big endian
====================================================================*/
int32_t toBigEndian(int32_t x)
{
    buffer32_t buf;
    unsigned char tmp;

    buf.data = x;

    tmp = buf.b[0];
    buf.b[0] = buf.b[3];
    buf.b[3] = tmp;

    tmp = buf.b[1];
    buf.b[1] = buf.b[2];
    buf.b[2] = tmp;

    return buf.data;
}


/*===================================================================
Load mnist images
====================================================================*/
unsigned char* loadMNISTImg(std::string filename, imgHeader_t *header)
{
    FILE* fp = fopen(filename.c_str(), "rb");
    if(!fp){
        printf("ERROR: cannot open %s\n", filename.c_str());
        return nullptr;
    }

    size_t size = 0;

    size = fread(header, sizeof(imgHeader_t), 1, fp);

    header->magic = toBigEndian(header->magic);
    header->num = toBigEndian(header->num);
    header->rows = toBigEndian(header->rows);
    header->cols = toBigEndian(header->cols);

    if(header->magic != 2051){
        printf("ERROR: bad magic number in %s\n", filename.c_str());
        return nullptr;
    }

    unsigned char* imgs = new unsigned char[header->num*header->rows*header->cols];

    size = fread(imgs, sizeof(unsigned char), header->num*header->rows*header->cols, fp);

    fclose(fp);
    return imgs;
}


/*===================================================================
Load mnist labels
====================================================================*/
unsigned char* loadMNISTLabel(std::string filename, labelHeader_t *header)
{
    FILE* fp = fopen(filename.c_str(), "rb");
    if(!fp){
        printf("ERROR: cannot open %s\n", filename.c_str());
        return nullptr;
    }

    size_t size = 0;

    size = fread(header, sizeof(labelHeader_t), 1, fp);

    header->magic = toBigEndian(header->magic);
    header->num = toBigEndian(header->num);

    if(header->magic != 2049){
        printf("ERROR: bad magic number in %s\n", filename.c_str());
        return nullptr;
    }

    unsigned char* labels = new unsigned char[header->num];

    size = fread(labels, sizeof(unsigned char), header->num, fp);

    fclose(fp);
    return labels;
}


/*===================================================================
Write *.tensor
====================================================================*/
bool writeTensor(std::string filename, tensor_t *t)
{
    if(t->type > 12 || t->type < 0){
        printf("ERROR: unsupported format\n");
        return false;
    }

    FILE *fp = fopen(filename.c_str(), "wb");
    if(!fp){
        printf("ERROR: failed to open %s\n", filename.c_str());
        return false;
    }

    int32_t len_name = t->name.length();
    size_t valueSize = 1;

    for(int i = 0; i < t->numDim; ++i)
        valueSize *= t->dims[i];

    fwrite(&t->type, sizeof(uint8_t), 1, fp);
    fwrite(&t->size, sizeof(uint32_t), 1, fp);
    fwrite(&len_name, sizeof(int32_t), 1, fp);
    fwrite(t->name.c_str(), sizeof(unsigned char), len_name, fp);
    fwrite(&t->numDim, sizeof(int32_t), 1, fp);
    fwrite(t->dims, sizeof(int32_t), t->numDim, fp);
    fwrite(t->value, t->size, valueSize, fp);

    fclose(fp);
    return true;
}


/*===================================================================
Read *.tensor
====================================================================*/
bool readTensor(std::string filename, tensor_t *t)
{
    FILE *fp = fopen(filename.c_str(), "rb");
    if(!fp){
        printf("ERROR: failed to open %s\n", filename.c_str());
        return false;
    }

    int32_t len_name;
    size_t size, valueSize = 1;
    char buf[256];

    size = fread(&t->type, sizeof(uint8_t), 1, fp);
    size = fread(&t->size, sizeof(uint32_t), 1, fp);
    size = fread(&len_name, sizeof(int32_t), 1, fp);
    size = fread(buf, sizeof(char), len_name, fp);

    buf[len_name] = '\0';
    t->name = std::string(buf);

    size = fread(&t->numDim, sizeof(int32_t), 1, fp);

    t->dims = new int32_t[t->numDim];

    size = fread(t->dims, sizeof(int32_t), t->numDim, fp);

    for(int i = 0; i < t->numDim; ++i)
        valueSize *= t->dims[i];

    switch(t->type){
        case TYPE_HALF   : t->value = (void*)(new half_float::half[valueSize]); break;
        case TYPE_FLOAT  : t->value = (void*)(new float[valueSize]);            break;
        case TYPE_DOUBLE : t->value = (void*)(new double[valueSize]);           break;
        case TYPE_UINT8  : t->value = (void*)(new uint8_t[valueSize]);          break;
        case TYPE_UINT16 : t->value = (void*)(new uint16_t[valueSize]);         break;
        case TYPE_UINT32 : t->value = (void*)(new uint32_t[valueSize]);         break;
        case TYPE_UINT64 : t->value = (void*)(new uint64_t[valueSize]);         break;
        case TYPE_INT8   : t->value = (void*)(new int8_t[valueSize]);           break;
        case TYPE_INT16  : t->value = (void*)(new int16_t[valueSize]);          break;
        case TYPE_INT32  : t->value = (void*)(new int32_t[valueSize]);          break;
        case TYPE_INT64  : t->value = (void*)(new int64_t[valueSize]);          break;
        case TYPE_CHAR   : t->value = (void*)(new char[valueSize]);             break;
        case TYPE_BOOL   : t->value = (void*)(new bool[valueSize]);             break;
        default:
            printf("ERROR: unsupported format\n");
            delete [] t->dims;
            return false;
    }

    size = fread(t->value, t->size, valueSize, fp);

    fclose(fp);
    return true;
}


/*===================================================================
Release tensor data
====================================================================*/
void releaseTensor(tensor_t *t)
{
    delete [] t->dims;

    switch(t->type){
        case TYPE_HALF   : delete [] (half_float::half*)(t->value);     break;
        case TYPE_FLOAT  : delete [] (float*)(t->value);                break;
        case TYPE_DOUBLE : delete [] (double*)(t->value);               break;
        case TYPE_UINT8  : delete [] (uint8_t*)(t->value);              break;
        case TYPE_UINT16 : delete [] (uint16_t*)(t->value);             break;
        case TYPE_UINT32 : delete [] (uint32_t*)(t->value);             break;
        case TYPE_UINT64 : delete [] (uint64_t*)(t->value);             break;
        case TYPE_INT8   : delete [] (int8_t*)(t->value);               break;
        case TYPE_INT16  : delete [] (int16_t*)(t->value);              break;
        case TYPE_INT32  : delete [] (int32_t*)(t->value);              break;
        case TYPE_INT64  : delete [] (int64_t*)(t->value);              break;
        case TYPE_CHAR   : delete [] (char*)(t->value);                 break;
        case TYPE_BOOL   : delete [] (bool*)(t->value);                 break;
        default:
            printf("ERROR: unsupported format\n");
            return;
    }
}


/*===================================================================
Print tensor data
====================================================================*/
template <class T>
void printTensor(tensor_t &t, const int n)
{
    int upper = (t.dims[0] < n) ? t.dims[0] : n;

    T* ptr = (T*)t.value;

    if(t.numDim == 4){
        for(int i = 0; i < upper; ++i){
            printf("%d\n", i);
            printf("=======================================================\n");

            for(int j = 0; j < t.dims[1]; ++j){
                for(int k = 0; k < t.dims[2]; ++k){
                    for(int m = 0; m < t.dims[3]; ++m)
                        printf("% 3.2f ", (float)ptr[i*t.dims[1]*t.dims[2]*t.dims[3] + j*t.dims[2]*t.dims[3] + k*t.dims[3] + m]);
                    printf("\n");
                }
                printf("\n");
            }
            printf("\n");
        }
    }
    else if(t.numDim == 5){
        for(int i = 0; i < upper; ++i){
            printf("%d\n", i);
            printf("=======================================================\n");

            for(int j = 0; j < t.dims[1]; ++j){
                for(int k = 0; k < t.dims[2]; ++k){
                    for(int m = 0; m < t.dims[3]; ++m){
                        for(int n = 0; n < t.dims[4]; ++n)
                            printf("% 3.2f ", (float)ptr[i*t.dims[1]*t.dims[2]*t.dims[3]*t.dims[4] + j*t.dims[2]*t.dims[3]*t.dims[4] + k*t.dims[3]*t.dims[4] + m*t.dims[4] + n]);
                        printf("\n");
                    }
                    printf("\n");
                }
                printf("\n");
            }
            printf("\n");
        }
    }
}


/*===================================================================
Show tensor data
====================================================================*/
void showTensor(tensor_t &t, const int n)
{
    printf("Data type: %d\n", t.type);
    printf("Name: %s\n", t.name.c_str());
    printf("Sizeof: %d\n", t.size);
    printf("NumDim: %d\n", t.numDim);

    for(int i = 0; i < t.numDim; ++i)
        printf("Dim %d: %d\n", i, t.dims[i]);
    printf("\n");

        switch(t.type){
            case TYPE_HALF   : printTensor<half_float::half>(t, n);     break;
            case TYPE_FLOAT  : printTensor<float>(t, n);                break;
            case TYPE_DOUBLE : printTensor<double>(t, n);               break;
            case TYPE_UINT8  : printTensor<uint8_t>(t, n);              break;
            case TYPE_UINT16 : printTensor<uint16_t>(t, n);             break;
            case TYPE_UINT32 : printTensor<uint32_t>(t, n);             break;
            case TYPE_UINT64 : printTensor<uint64_t>(t, n);             break;
            case TYPE_INT8   : printTensor<int8_t>(t, n);               break;
            case TYPE_INT16  : printTensor<int16_t>(t, n);              break;
            case TYPE_INT32  : printTensor<int32_t>(t, n);              break;
            case TYPE_INT64  : printTensor<int64_t>(t, n);              break;
            case TYPE_CHAR   : printTensor<char>(t, n);                 break;
            case TYPE_BOOL   : printTensor<bool>(t, n);                 break;
            default:
                printf("ERROR: unsupported format\n");
                return;
        }
}


/*===================================================================
Get *.tensor as labels
====================================================================*/
template<class T>
T* getAsLabel(tensor_t &t)
{
    T* ptr = nullptr;

    if(t.numDim == 4 && t.dims[0] > 0 && t.dims[1] == 1 && t.dims[2] == 1 && t.dims[3] == 1){
        ptr = (T*)t.value;
    }

    return ptr;
}


/*===================================================================
Get *.tensor as vectors
====================================================================*/
template<class T>
std::vector<T*> getAsVec(tensor_t &t)
{
    std::vector<T*> ptr;
    int totalDim = 1;

    if(t.numDim > 0 && t.dims[0] > 0){
        for(int i = 1; i < t.numDim; ++i)
            totalDim *= t.dims[i];

        for(int i = 0; i < t.dims[0]; ++i)
            ptr.push_back((T*)t.value + i * totalDim * t.size);
    }

    return ptr;
}

#endif
