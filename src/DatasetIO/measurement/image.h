// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <memory>
#include <core/common/enums.h>
#include <stdlib.h>
#include <string.h>
#include <stdexcept>
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include "core/common/cuda_defs.h"
#include "cuda/cuda_image.h"

namespace VGUGV
{
    namespace Common
    {
/** \brief
	Represents image blocks, templated on the data type
 */
        template<typename T>
        class Image
        {
          public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          public:
            Image(size_t height, size_t width, MemoryDeviceType memoryType);
            Image(size_t height, size_t width, bool allocate_CPU, bool allocate_CUDA);
            Image(bool allocate_cpu, bool allocate_CUDA);

            // Suppress the default copy constructor and assignment operator
            Image(const Image &);
            Image &operator=(const Image &);

            ~Image()
            { this->Free(); }

          public:
            T *GetData(MemoryDeviceType memoryType);
            const T *GetData(MemoryDeviceType memoryType) const;

            T GetElement(int r, int c, MemoryDeviceType memoryType) const;
            T GetElement(int index, MemoryDeviceType memoryType) const;

            Cuda::CudaImage<T> GetCudaImage();
            Cuda::CudaImage<T> GetCudaImage() const;

            bool IsAllocatedOnCpu() const
            { return isAllocated_CPU; }
            bool IsAllocatedOnCuda() const
            { return isAllocated_CUDA; }

            size_t Width() const
            { return mWidth; }
            size_t Height() const
            { return mHeight; }
            size_t Size() const
            { return mWidth * mHeight; }
            size_t Pitch() const
            { return mCudaMemPitch; }

          public:
            void UpdateDeviceFromHost() const;
            void UpdateHostFromDevice() const;

            void SetFrom(const Image<T> *source, MemoryCopyDirection memoryCopyDirection);
            void SetFrom(const Image<T> &source, MemoryCopyDirection memoryCopyDirection);

            void SetElement(int r, int c, T value, MemoryDeviceType memoryType) const;

            void Allocate(size_t height, size_t width, bool allocate_CPU, bool allocate_CUDA);
            void Free();

          public:
            void Clear(unsigned char defaultValue = 0);
            void Resize(size_t height, size_t width);
            void Normalize(float scalingFactor);

          protected:
            bool isAllocated_CPU, isAllocated_CUDA;
            T *data_cpu;
            T *data_cuda;

            size_t mWidth;
            size_t mHeight;
            size_t mCudaMemPitch; // by default we malloc 2D memory for cuda data
        };

        template<class T>
        Image<T>::Image(size_t height, size_t width, bool allocate_CPU, bool allocate_CUDA)
        {
            this->isAllocated_CPU = false;
            this->isAllocated_CUDA = false;
            Allocate(height, width, allocate_CPU, allocate_CUDA);
        }

        template<class T>
        Image<T>::Image(size_t height, size_t width, MemoryDeviceType memoryType)
        {
            this->isAllocated_CPU = false;
            this->isAllocated_CUDA = false;
            switch (memoryType)
            {
                case MEMORYDEVICE_CPU:Allocate(height, width, true, false);
                    break;
                case MEMORYDEVICE_CUDA:
                {
                    Allocate(height, width, false, true);
                    break;
                }
            }
        }

        template<class T>
        Image<T>::Image(bool allocate_cpu, bool allocate_CUDA)
        {
            isAllocated_CPU = allocate_cpu;
            isAllocated_CUDA = allocate_CUDA;
            mWidth = 0;
            mHeight = 0;
            data_cpu = NULL;
            data_cuda = NULL;
        }

        template<class T>
        T *Image<T>::GetData(MemoryDeviceType memoryType)
        {
            switch (memoryType)
            {
                case MEMORYDEVICE_CPU:return data_cpu;
                case MEMORYDEVICE_CUDA:return data_cuda;
            }
            return 0;
        }

        template<class T>
        const T *Image<T>::GetData(MemoryDeviceType memoryType) const
        {
            switch (memoryType)
            {
                case MEMORYDEVICE_CPU:return data_cpu;
                case MEMORYDEVICE_CUDA:return data_cuda;
            }
            return 0;
        }

        template<class T>
        T Image<T>::GetElement(int r, int c, MemoryDeviceType memoryType) const
        {
            switch (memoryType)
            {
                case MEMORYDEVICE_CPU:
                {
                    return this->data_cpu[r * mWidth + c];
                }
#ifndef COMPILE_WITHOUT_CUDA
                case MEMORYDEVICE_CUDA:
                {
                    T result;
                    CUDA_SAFE_CALL(cudaMemcpy(&result,
                                              (T *) ((char *) this->data_cuda + r * mCudaMemPitch) + c,
                                              sizeof(T),
                                              cudaMemcpyDeviceToHost));
                    return result;
                }
#endif
                default:throw std::runtime_error("Invalid memory type");
            }
        }

        template<class T>
        T Image<T>::GetElement(int index, MemoryDeviceType memoryType) const
        {
            switch (memoryType)
            {
                case MEMORYDEVICE_CPU:
                {
                    return data_cpu[index];
                }
#ifndef COMPILE_WITHOUT_CUDA
                case MEMORYDEVICE_CUDA:
                {
                    T result;
                    int r = index / mWidth;
                    int c = index % mWidth;
                    CUDA_SAFE_CALL(cudaMemcpy(&result,
                                              (T *) ((char *) this->data_cuda + r * mCudaMemPitch) + c,
                                              sizeof(T),
                                              cudaMemcpyDeviceToHost));
                    return result;
                }
#endif
                default:throw std::runtime_error("Invalid memory type");
            }
        }

        template<class T>
        Cuda::CudaImage<T> Image<T>::GetCudaImage()
        {
            Cuda::CudaImage<T> cuda_image;
            cuda_image.cuda_image = this->data_cuda;
            cuda_image.nRows = this->Height();
            cuda_image.nCols = this->Width();
            cuda_image.pitch = this->Pitch();
            return cuda_image;
        }

        template<class T>
        Cuda::CudaImage<T> Image<T>::GetCudaImage() const
        {
            Cuda::CudaImage<T> cuda_image;
            cuda_image.cuda_image = this->data_cuda;
            cuda_image.nRows = this->Height();
            cuda_image.nCols = this->Width();
            cuda_image.pitch = this->Pitch();
            return cuda_image;
        }

        template<class T>
        void Image<T>::Clear(unsigned char defaultValue)
        {
            if (isAllocated_CPU) memset(data_cpu, defaultValue, mWidth * mHeight * sizeof(T));
#ifndef COMPILE_WITHOUT_CUDA
            if (isAllocated_CUDA)
            CUDA_SAFE_CALL(cudaMemset2D(data_cuda, mCudaMemPitch, defaultValue, mWidth, mHeight));
#endif
        }

        template<class T>
        void Image<T>::Resize(size_t height, size_t width)
        {
            if (width == mWidth && height == mHeight) return;

            bool allocate_CPU = this->isAllocated_CPU;
            bool allocate_CUDA = this->isAllocated_CUDA;

            this->Free();
            this->Allocate(height, width, allocate_CPU, allocate_CUDA);
        }

        template<class T>
        void Image<T>::UpdateDeviceFromHost() const
        {
#ifndef COMPILE_WITHOUT_CUDA
            if (isAllocated_CUDA && isAllocated_CPU)
            {
                CUDA_SAFE_CALL(cudaMemcpy2D(data_cuda,
                                            mCudaMemPitch,
                                            data_cpu,
                                            mWidth * sizeof(T),
                                            mWidth * sizeof(T),
                                            mHeight,
                                            cudaMemcpyHostToDevice));
            }
            else
            {
                printf("%s_%s: data is not allocated on either GPU or CPU", __FUNCTION__, __LINE__);
                exit(0);
            }
#endif
        }

        template<class T>
        void Image<T>::UpdateHostFromDevice() const
        {
#ifndef COMPILE_WITHOUT_CUDA
            if (isAllocated_CUDA && isAllocated_CPU)
            {
                CUDA_SAFE_CALL(cudaMemcpy2D(data_cpu,
                                            mWidth * sizeof(T),
                                            data_cuda,
                                            mCudaMemPitch,
                                            mWidth * sizeof(T),
                                            mHeight,
                                            cudaMemcpyDeviceToHost));
            }
            else
            {
                printf("%s_%s: data is not allocated on either GPU or CPU", __FUNCTION__, __LINE__);
                exit(0);
            }
#endif
        }

        template<class T>
        void Image<T>::SetFrom(const Image<T> *const source, MemoryCopyDirection memoryCopyDirection)
        {
            Resize(source->Height(), source->Width());
            switch (memoryCopyDirection)
            {
                case CPU_TO_CPU:
                    memcpy(this->data_cpu,
                           source->data_cpu,
                           source->Width() * source->Height() * sizeof(T));
                    break;
#ifndef COMPILE_WITHOUT_CUDA
                case CPU_TO_CUDA:
                CUDA_SAFE_CALL(cudaMemcpy2D(this->data_cuda,
                                            this->mCudaMemPitch,
                                            source->data_cpu,
                                            source->Width() * sizeof(T),
                                            source->Width() *
                                                sizeof(T),
                                            source->Height(),
                                            cudaMemcpyHostToDevice));
                    break;
                case CUDA_TO_CPU:
                CUDA_SAFE_CALL(cudaMemcpy2D(this->data_cpu,
                                            this->Width() * sizeof(T),
                                            source->data_cuda,
                                            source->mCudaMemPitch,
                                            source->Width() *
                                                sizeof(T),
                                            source->Height(),
                                            cudaMemcpyDeviceToHost));
                    break;
                case CUDA_TO_CUDA:
                CUDA_SAFE_CALL(
                    cudaMemcpy2DAsync(this->data_cuda,
                                      this->mCudaMemPitch,
                                      source->data_cuda,
                                      source->mCudaMemPitch,
                                      source->Width() * sizeof(T),
                                      source->Height(),
                                      cudaMemcpyDeviceToDevice));
                    break;
#endif
                default:break;
            }
        }

        template<class T>
        void Image<T>::SetFrom(const Image<T> &source, MemoryCopyDirection memoryCopyDirection)
        {
            SetFrom(&source, memoryCopyDirection);
        }

        template<class T>
        void Image<T>::SetElement(int r, int c, T value, MemoryDeviceType memoryType) const
        {
            switch (memoryType)
            {
                case MEMORYDEVICE_CPU:
                {
                    this->data_cpu[r * mWidth + c] = value;
                    break;
                }
#ifndef COMPILE_WITHOUT_CUDA
                case MEMORYDEVICE_CUDA:
                {
                    T result = value;
                    CUDA_SAFE_CALL(cudaMemcpy((T *) ((char *) data_cuda + r * mCudaMemPitch) + c,
                                              &result,
                                              sizeof(T),
                                              cudaMemcpyHostToDevice));
                    break;
                }
#endif
                default:throw std::runtime_error("Invalid memory type");
            }
        }

        template<class T>
        void Image<T>::Allocate(size_t height, size_t width, bool allocate_CPU, bool allocate_CUDA)
        {
            if (allocate_CPU && isAllocated_CPU)
            {
                allocate_CPU = false;
            }
            if (allocate_CUDA && isAllocated_CUDA)
            {
                allocate_CUDA = false;
            }

            this->mWidth = width;
            this->mHeight = height;

            if (allocate_CPU)
            {
                if (width == 0 || height == 0) data_cpu = NULL;
                else data_cpu = new T[width * height];
                this->isAllocated_CPU = allocate_CPU;
            }

            if (allocate_CUDA)
            {
#ifndef COMPILE_WITHOUT_CUDA
                if (width == 0 || height == 0) data_cuda = NULL;
                else
                CUDA_SAFE_CALL(cudaMallocPitch((void **) &data_cuda, &mCudaMemPitch, mWidth * sizeof(T), mHeight));
                this->isAllocated_CUDA = allocate_CUDA;
#endif
            }
        }

        template<class T>
        void Image<T>::Free()
        {
            mWidth = 0;
            mHeight = 0;
            if (isAllocated_CPU)
            {
                if (data_cpu != NULL) delete[] data_cpu;
                isAllocated_CPU = false;
            }

            if (isAllocated_CUDA)
            {
#ifndef COMPILE_WITHOUT_CUDA
                if (data_cuda != NULL)
                CUDA_SAFE_CALL(cudaFree(data_cuda));
#endif
                isAllocated_CUDA = false;
            }
        }

        template<class T>
        void Image<T>::Normalize(float scalingFactor)
        {
            const size_t numPixels = Size();
            T *pData = GetData(Common::MEMORYDEVICE_CPU);
            for (size_t i = 0; i < numPixels; ++i, ++pData)
            {
                *pData *= scalingFactor;
            }
        }

    } /* namespace common */
} /* namespace VGUGV */
