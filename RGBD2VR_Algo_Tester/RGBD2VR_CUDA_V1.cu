#include <cuda_runtime.h>
#include <device_launch_parameters.h>

__global__ void generateStereoImage(const double* Depdata, const unsigned char* RGBdata, const unsigned char* MaskPtr,
    double* ImgTagLPtr, double* ImgTagRPtr, unsigned char* ImgOutPtr, double* MvImgLPtr, double* MvImgRPtr,
    int width, int height, double cx, double cy, double f, double k, double Baseline, double WL,
    int isRGBFill, int FWin)
{
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    double d = 10.0;
    for (int i = index; i < width * height; i += stride)
    {
        int x = i % width;
        int y = i / width;

        double DepPixel = Depdata[i];
        double D = DepPixel;
        int r = RGBdata[i * 3 + 0];
        int g = RGBdata[i * 3 + 1];
        int b = RGBdata[i * 3 + 2];

        if (MaskPtr[i] == 0 && D > 0)
        {
            double xI_l = d / D * (D * (x - cx) / f + Baseline * k);
            int I_l = static_cast<int>((1 + xI_l / WL) * width / 2);
            ImgTagLPtr[I_l + y * width] = I_l + y * width;
            ImgOutPtr[i * 3 + 0] = r;
            ImgOutPtr[i * 3 + 1] = g;
            ImgOutPtr[i * 3 + 2] = b;
            MvImgLPtr[i] = I_l;

            if (isRGBFill == 3)
            {
                if (FWin == 0)
                    continue;

                for (int fj = -FWin; fj <= FWin; fj++)
                {
                    for (int fi = -FWin; fi <= FWin; fi++)
                    {
                        int new_y = y + fj;
                        int new_x = x + fi;

                        if (new_y >= 0 && new_y < height && new_x >= 0 && new_x < width)
                        {
                            int maskIndex = new_y * width + new_x;
                            if (MaskPtr[maskIndex] == 0)
                            {
                                int imgOutIndex = (maskIndex * 3) + (fi * width + fj);
                                ImgOutPtr[imgOutIndex + 0] = r;
                                ImgOutPtr[imgOutIndex + 1] = g;
                                ImgOutPtr[imgOutIndex + 2] = b;
                            }
                        }
                    }
                }
            }
        }
    }
}

void runStereoImageGenerationCUDA(const double* Depdata, const unsigned char* RGBdata, const unsigned char* MaskPtr,
    double* ImgTagLPtr, double* ImgTagRPtr, unsigned char* ImgOutPtr, double* MvImgLPtr, double* MvImgRPtr,
    int width, int height, double cx, double cy, double f, double k, double Baseline, double WL,
    int isRGBFill, int FWin)
{
    // Allocate device memory
    double* dev_Depdata;
    unsigned char* dev_RGBdata;
    unsigned char* dev_MaskPtr;
    double* dev_ImgTagLPtr;
    double* dev_ImgTagRPtr;
    unsigned char* dev_ImgOutPtr;
    double* dev_MvImgLPtr;
    double* dev_MvImgRPtr;

    cudaMalloc((void**)&dev_Depdata, width * height * sizeof(double));
    cudaMalloc((void**)&dev_RGBdata, width * height * 3 * sizeof(unsigned char));
    cudaMalloc((void**)&dev_MaskPtr, width * height * sizeof(unsigned char));
    cudaMalloc((void**)&dev_ImgTagLPtr, width * height * sizeof(double));
    cudaMalloc((void**)&dev_ImgTagRPtr, width * height * sizeof(double));
    cudaMalloc((void**)&dev_ImgOutPtr, width * height * 3 * sizeof(unsigned char));
    cudaMalloc((void**)&dev_MvImgLPtr, width * height * sizeof(double));
    cudaMalloc((void**)&dev_MvImgRPtr, width * height * sizeof(double));

    // Copy data from host to device
    cudaMemcpy(dev_Depdata, Depdata, width * height * sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(dev_RGBdata, RGBdata, width * height * 3 * sizeof(unsigned char), cudaMemcpyHostToDevice);
    cudaMemcpy(dev_MaskPtr, MaskPtr, width * height * sizeof(unsigned char), cudaMemcpyHostToDevice);

    // Launch kernel
    int threadsPerBlock = 256;
    int numBlocks = (width * height + threadsPerBlock - 1) / threadsPerBlock;
    //generateStereoImage<<<numBlocks, threadsPerBlock >>>(dev_Depdata, dev_RGBdata, dev_MaskPtr, dev_ImgTagLPtr, dev_ImgTagRPtr,
    //    dev_ImgOutPtr, dev_MvImgLPtr, dev_MvImgRPtr, width, height, cx, cy, f, k,
    //    Baseline, WL, isRGBFill, FWin);
    generateStereoImage<<<numBlocks, threadsPerBlock >>>(dev_Depdata, dev_RGBdata, dev_MaskPtr, dev_ImgTagLPtr, dev_ImgTagRPtr,
        dev_ImgOutPtr, dev_MvImgLPtr, dev_MvImgRPtr, width, height, cx, cy, f, k,
        Baseline, WL, isRGBFill, FWin);



    // Copy data from device to host
    cudaMemcpy(ImgTagLPtr, dev_ImgTagLPtr, width * height * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(ImgTagRPtr, dev_ImgTagRPtr, width * height * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(ImgOutPtr, dev_ImgOutPtr, width * height * 3 * sizeof(unsigned char), cudaMemcpyDeviceToHost);
    cudaMemcpy(MvImgLPtr, dev_MvImgLPtr, width * height * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(MvImgRPtr, dev_MvImgRPtr, width * height * sizeof(double), cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(dev_Depdata);
    cudaFree(dev_RGBdata);
    cudaFree(dev_MaskPtr);
    cudaFree(dev_ImgTagLPtr);
    cudaFree(dev_ImgTagRPtr);
    cudaFree(dev_ImgOutPtr);
    cudaFree(dev_MvImgLPtr);
    cudaFree(dev_MvImgRPtr);
}
