#pragma once
#include <vector>

#ifdef CANNYEDGEDETECTIONCPP_EXPORTS
#define CANNYEDGEDETECTIONCPP_API __declspec(dllexport)
#else
#define CANNYEDGEDETECTIONCPP_API __declspec(dllimport)
#endif

//extern "C" MATHLIBRARY_API void ChangeBrightness(std::vector<unsigned char>* img, int brightness);

extern "C" CANNYEDGEDETECTIONCPP_API void ChangeBrightness(unsigned char* outputPtr, unsigned char* inputPtr, signed short brightness, size_t imageSize);

extern "C" CANNYEDGEDETECTIONCPP_API void SetupImageDimensions(const int width, const int height);

extern "C" CANNYEDGEDETECTIONCPP_API void SetupImageExtensions(unsigned char* inputPtr);
extern "C" CANNYEDGEDETECTIONCPP_API void SetupGaussKernel(const float variance);
extern "C" CANNYEDGEDETECTIONCPP_API void ConvoluteWithGaussKernel(unsigned char* outputPtr, unsigned char* inputPtr, const int startOffset, const int pixelAmountArea);
extern "C" CANNYEDGEDETECTIONCPP_API void CleanAfterGauss();

extern "C" CANNYEDGEDETECTIONCPP_API void InitializeIntensityAngleTables();
extern "C" CANNYEDGEDETECTIONCPP_API void ConvoluteWithSobelKernel(unsigned char* inputPtr, const int startOffset, const int pixelAmountArea);
extern "C" CANNYEDGEDETECTIONCPP_API void NormalizeSobel(unsigned char* outputPtr, const int startOffset, const int pixelAmountArea);

extern "C" CANNYEDGEDETECTIONCPP_API void ThinEdges(unsigned char* outputPtr, unsigned char* inputPtr, const int startOffset, const int pixelAmountArea);

extern "C" CANNYEDGEDETECTIONCPP_API void CleanIntensityAngleTables();

extern "C" CANNYEDGEDETECTIONCPP_API void CategorizeEdges(unsigned char* outputPtr, unsigned char* inputPtr, const int startOffset, const int pixelAmountArea, const uint8_t thresholdH, const uint8_t thresholdL);

extern "C" CANNYEDGEDETECTIONCPP_API void ConnectEdges(unsigned char* outputPtr, unsigned char* inputPtr, const int startOffset, const int pixelAmountArea);
