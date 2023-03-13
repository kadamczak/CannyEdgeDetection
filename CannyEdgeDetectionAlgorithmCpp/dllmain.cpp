// MathLibrary.cpp : Defines the exported functions for the DLL.
#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier
#include <utility>
#include <limits.h>
#include <cmath>
#include <mutex>
#include "CannyEdgeDetectionAlgorithmCpp.h"

#define M_PI 3.14159265358979323846
#define M_E  2.71828182845904523536

std::mutex maxIntensityMutex;

// DLL internal state variables:
static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position

//------------------------------------------------------------------------------------------------------
//IMAGE DATA
int imageWidth;
int imageHeight;

void SetupImageDimensions(const int width, const int height)
{
	imageWidth = width;
	imageHeight = height;
}

//------------------------------------------------------------------------------------------------------
//GAUSSIAN BLUR STAGE

void Move(int& x, int& y, const int& startingX, const int& endingX)
{
	if (x + 1 < endingX)
	{
		x++;
	}
	else
	{
		x = startingX;
		y++;
	}
}

class ImageExtension
{
	uint8_t imageExtensionTop[7000];
	uint8_t imageExtensionRight[7000];
	uint8_t imageExtensionBottom[7000];
	uint8_t imageExtensionLeft[7000];
	uint8_t imageExtensionCorners[7000];

	void Fill(unsigned char*& edgePixel, const int& size, const int& interval, const int& start, uint8_t* ext)
	{
		if (start == 0)
		{
			for (int i = 0; i < size; i++)
			{
				ext[i] = *edgePixel;
				edgePixel += interval;
			}
			edgePixel -= interval;
		}
		else
		{
			for (int i = start; i >= 0; i--)
			{
				ext[i] = *edgePixel;
				edgePixel += interval;
			}
			edgePixel -= interval;
		}

	}

public:
	ImageExtension()
	{
		/*imageExtensionTop = new uint8_t[imageWidth];
		imageExtensionRight = new uint8_t[imageHeight];
		imageExtensionBottom = new uint8_t[imageWidth];
		imageExtensionLeft = new uint8_t[imageHeight];
		imageExtensionCorners = new uint8_t[4];*/
	}

	uint8_t GetImageExtensionCorners(const int& index)
	{
		return imageExtensionCorners[index];
	}
	uint8_t GetImageExtensionTop(const int& index)
	{
		return imageExtensionTop[index];
	}
	uint8_t GetImageExtensionRight(const int& index)
	{
		return imageExtensionRight[index];
	}
	uint8_t GetImageExtensionBottom(const int& index)
	{
		return imageExtensionBottom[index];
	}
	uint8_t GetImageExtensionLeft(const int& index)
	{
		return imageExtensionLeft[index];
	}

	void FillExtensions(unsigned char* edgePixel)
	{
		imageExtensionCorners[0] = *edgePixel;
		Fill(edgePixel, imageWidth, 1, 0, imageExtensionTop);
		imageExtensionCorners[1] = *edgePixel;
		Fill(edgePixel, imageHeight, imageWidth, 0, imageExtensionRight);
		imageExtensionCorners[2] = *edgePixel;
		Fill(edgePixel, imageWidth, -1, imageWidth - 1, imageExtensionBottom);
		imageExtensionCorners[3] = *edgePixel;
		Fill(edgePixel, imageHeight, -imageWidth, imageHeight - 1, imageExtensionLeft);
	}

	~ImageExtension()
	{
		/*delete[] imageExtensionTop;
		delete[] imageExtensionRight;
		delete[] imageExtensionBottom;
		delete[] imageExtensionLeft;
		delete[] imageExtensionCorners;*/
	}
};

class GaussKernel
{
	float kernel[41][41];
	int size;
	int radius;
	float kernelSum;
	float sigma;

	double GetKernelValue(const int& X, const int& Y)
	{
		float result;
		result = 1 / (2 * M_PI * sigma * sigma) * std::pow(M_E, -(((X * X) + (Y * Y)) / (2 * sigma * sigma)));
		return result;
	}
	void CalculateSizeAndRadius()
	{
		//Get initial kernel size resulting from the variance
		size = (int)(sigma * 2);
		//Make sure kernel size is an odd number
		size += (size % 2 == 0) ? 1 : 0;
		//If kernel size is 1, adjust it to be at least 3
		size = (size == 1) ? 3 : size;

		radius = size / 2;
	}

	void InitializeKernel()
	{
		//Initialize Gauss kernel matrix
		/*kernel = new float* [size];
		for (int i = 0; i < size; i++)
			kernel[i] = new float[size];*/
	}

	void CalculateKernel()
	{
		int distanceFromCenterX;
		int distanceFromCenterY;
		float kernelValue;

		//Calculate values present in the kernel
		for (int i = 0; i < size; i++)
		{
			distanceFromCenterY = radius - i;
			for (int j = 0; j < size; j++)
			{
				distanceFromCenterX = radius - j;
				kernelValue = GetKernelValue(distanceFromCenterX, distanceFromCenterY);
				kernel[i][j] = kernelValue;
				kernelSum += kernelValue;
			}
		}
	}

	uint8_t GetPixelValue(unsigned char* selectedPixel, const int& x, const int& y)
	{
		if (x < 0)
		{
			if (y < 0) return extensions->GetImageExtensionCorners(0);
			else if (y >= imageHeight) return extensions->GetImageExtensionCorners(3);
			else return extensions->GetImageExtensionLeft(y);
		}
		else if (x >= imageWidth)
		{
			if (y < 0) return extensions->GetImageExtensionCorners(1);
			else if (y >= imageHeight) return extensions->GetImageExtensionCorners(2);
			else return extensions->GetImageExtensionRight(y);
		}
		else if (y < 0) extensions->GetImageExtensionTop(x);
		else if (y >= imageHeight) extensions->GetImageExtensionBottom(x);

		else return *selectedPixel;
	}


public:
	ImageExtension* extensions;

	GaussKernel(const float& variance)
		: sigma(variance), kernelSum(0.0)
	{
		this->CalculateSizeAndRadius();
		this->InitializeKernel();
		this->CalculateKernel();
	}
	~GaussKernel()
	{
		/*for (int i = 0; i < size; ++i)
			delete[] kernel[i];
		delete[] kernel;*/

		extensions = nullptr;
	}

	uint8_t BlurPixel(unsigned char* selectedPixel, int x, int y)
	{
		float result = 0;
		float newValue = 0;

		x -= radius;
		y -= radius;
		selectedPixel -= imageWidth * radius + radius;
		int startingX = x;
		int endingX = startingX + size;

		uint8_t imageValue;
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				imageValue = GetPixelValue(selectedPixel, x, y);
				newValue = imageValue * kernel[i][j];
				result += newValue;

				selectedPixel++;
				Move(x, y, startingX, endingX);
			}
			selectedPixel += imageWidth - size;
		}
		result /= kernelSum;

		uint8_t blurredPixel = (uint8_t)result;
		return blurredPixel;
	}
};

ImageExtension* imgExtension;
GaussKernel* gaussKernel;

void SetupImageExtensions(unsigned char* inputPtr)
{
	imgExtension = new ImageExtension();
	imgExtension->FillExtensions(inputPtr);
}

void SetupGaussKernel(const float variance)
{
	gaussKernel = new GaussKernel(variance);
	gaussKernel->extensions = imgExtension;
}

void ConvoluteWithGaussKernel(unsigned char* outputPtr, unsigned char* inputPtr, const int startOffset, const int pixelAmountArea)
{
	inputPtr += startOffset;
	outputPtr += startOffset;

	int x = startOffset % imageWidth;
	int y = startOffset / imageWidth;


	for (int i = 0; i < pixelAmountArea; i++)
	{
		*outputPtr = gaussKernel->BlurPixel(inputPtr, x, y);
		inputPtr++;
		outputPtr++;

		Move(x, y, 0, imageWidth);
	}

}

void CleanAfterGauss()
{
	delete imgExtension;
	delete gaussKernel;
}

//------------------------------------------------------------------------------------------------------
//SOBEL OPERATOR STAGE

const int sobelKernelX[3][3] = { {1, 0, -1},
								 {2, 0, -2},
								 {1, 0, -1} };

const int sobelKernelY[3][3] = { {1,   2,  1},
								 {0,   0,  0},
								 {-1, -2, -1} };

float intensityTable[7000][7000];
int angleTable[7000][7000];

void InitializeIntensityAngleTables()
{
	/*intensityTable = new float* [imageHeight];
	for (int i = 0; i < imageHeight; i++)
		intensityTable[i] = new float[imageWidth];

	angleTable = new int* [imageHeight];
	for (int i = 0; i < imageHeight; i++)
		angleTable[i] = new int[imageWidth];*/
}

void CleanIntensityAngleTables()
{
	/*for (int i = 0; i < imageHeight; i++)
		delete[] intensityTable[i];
	delete[] intensityTable;

	for (int i = 0; i < imageHeight; i++)
		delete[] angleTable[i];
	delete[] angleTable;*/
}

int GetRoundedAngle(const float& Gx, const float& Gy)
{
	float angle = (Gx != 0) ? atan(Gy / Gx) * 180 / M_PI : 90.0;
	//angle += 90;

	if (angle >= 67.5) return 90;
	else if (angle >= 22.5) return 45;
	else if (angle >= -22.5) return 0;
	else if (angle >= -67.5) return 135;
	else return 90;
}

float ApplySobel(unsigned char* selectedPixel, const int kernel[3][3])
{
	float result = 0;
	selectedPixel -= imageWidth + 1;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			result += *selectedPixel * kernel[i][j];
			selectedPixel++;
		}
		selectedPixel += imageWidth - 3;
	}
	return result;
}

float maxG = 0;
float CalculateGradientIntensityAndAngle(unsigned char* selectedPixel, int x, int y)
{
	float resultX = ApplySobel(selectedPixel, sobelKernelX);
	float resultY = ApplySobel(selectedPixel, sobelKernelY);

	float G = static_cast<int>(sqrt(resultX * resultX + resultY * resultY));
	int a;

	maxIntensityMutex.lock();
	maxG = (G > maxG) ? G : maxG;
	maxIntensityMutex.unlock();

	a = GetRoundedAngle(resultX, resultY);
	angleTable[y][x] = a;
	return G;
}

void ConvoluteWithSobelKernel(unsigned char* inputPtr, const int startOffset, const int pixelAmountArea)
{
	inputPtr += startOffset;

	int x = startOffset % imageWidth;
	int y = startOffset / imageWidth;

	for (int i = 0; i < pixelAmountArea; i++)
	{
		if (x == 0 || y == 0 || y == imageHeight - 1 || x == imageWidth - 1)
		{
			intensityTable[y][x] = 0;
			angleTable[y][x] = 0;
		}
		else
		{
			intensityTable[y][x] = CalculateGradientIntensityAndAngle(inputPtr, x, y);
		}

		inputPtr++;
		Move(x, y, 0, imageWidth);
	}

}

void NormalizeSobel(unsigned char* outputPtr, const int startOffset, const int pixelAmountArea)
{
	outputPtr += startOffset;

	int x = startOffset % imageWidth;
	int y = startOffset / imageWidth;

	float G;
	uint8_t normalizedG;

	for (int i = 0; i < pixelAmountArea; i++)		//   /	VEC
	{
		G = intensityTable[y][x];
		normalizedG = static_cast<uint8_t>(G / maxG * 255);
		*outputPtr = normalizedG;

		outputPtr++;
		Move(x, y, 0, imageWidth);
	}

	//%	SCALAR
}

//--------------------------------------------------------------------------------------------------------
uint8_t ThinEdge(unsigned char* selectedPixel, const int& x, const int& y)
{
	int angle = angleTable[y][x];
	uint8_t currentPixelValue = *selectedPixel;
	uint8_t comparedPixel1;
	uint8_t comparedPixel2;

	switch (angle)
	{
	case 0:
		comparedPixel1 = *(selectedPixel + 1);
		comparedPixel2 = *(selectedPixel - 1);
		break;
	case 45:
		comparedPixel1 = *(selectedPixel - imageWidth + 1);
		comparedPixel2 = *(selectedPixel + imageWidth - 1);
		break;
	case 90:
		comparedPixel1 = *(selectedPixel - imageWidth);
		comparedPixel2 = *(selectedPixel + imageWidth);
		break;
	case 135:
		comparedPixel1 = *(selectedPixel - imageWidth - 1);
		comparedPixel2 = *(selectedPixel + imageWidth + 1);
		break;
	}

	return (currentPixelValue >= comparedPixel1 && currentPixelValue >= comparedPixel2) ? currentPixelValue : 0;
}

void ThinEdges(unsigned char* outputPtr, unsigned char* inputPtr, const int startOffset, const int pixelAmountArea)
{
	inputPtr += startOffset;
	outputPtr += startOffset;

	int x = startOffset % imageWidth;
	int y = startOffset / imageWidth;


	for (int i = 0; i < pixelAmountArea; i++)
	{
		if (x == 0 || y == 0 || y == imageHeight - 1 || x == imageWidth - 1)
		{
			*outputPtr = 0;
		}
		else
		{
			*outputPtr = ThinEdge(inputPtr, x, y);
		}

		inputPtr++;
		outputPtr++;
		Move(x, y, 0, imageWidth);
	}

}


//------------------------------------------------------------------------------------------------------

uint8_t CONST STRONG_EDGE = 255;
uint8_t CONST WEAK_EDGE = 40;
uint8_t CONST NO_EDGE = 0;

void CategorizeEdges(unsigned char* outputPtr, unsigned char* inputPtr, const int startOffset, const int pixelAmountArea, const uint8_t thresholdH, const uint8_t thresholdL)
{
	inputPtr += startOffset;
	outputPtr += startOffset;

	int x = startOffset % imageWidth;
	int y = startOffset / imageWidth;

	uint8_t selectedPixel;
	for (int i = 0; i < pixelAmountArea; i++)
	{
		if (x == 0 || y == 0 || y == imageHeight - 1 || x == imageWidth - 1)
		{
			*outputPtr = NO_EDGE;
		}
		else
		{
			selectedPixel = *inputPtr;
			if (selectedPixel >= thresholdH) *outputPtr = STRONG_EDGE;
			else if (selectedPixel >= thresholdL)
				*outputPtr = WEAK_EDGE;
			else *outputPtr = NO_EDGE;
		}

		inputPtr++;
		outputPtr++;
		Move(x, y, 0, imageWidth);
	}

}

//------------------------------------------------------------------------------------------------------

uint8_t CheckNeighbours(unsigned char* neighbourPixel, uint8_t& selectedPixel)
{
	uint8_t value;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			value = *neighbourPixel;
			if (value == STRONG_EDGE)
				selectedPixel = STRONG_EDGE;

			neighbourPixel++;
		}
		neighbourPixel += imageWidth - 3;
	}

	selectedPixel = (selectedPixel == STRONG_EDGE) ? STRONG_EDGE : NO_EDGE;
	return selectedPixel;
}

void ConnectEdges(unsigned char* outputPtr, unsigned char* inputPtr, const int startOffset, const int pixelAmountArea)
{
	inputPtr += startOffset;
	outputPtr += startOffset;

	int x = startOffset % imageWidth;
	int y = startOffset / imageWidth;

	uint8_t selectedPixel;
	unsigned char* neighbourPixel;

	for (int i = 0; i < pixelAmountArea; i++)
	{
		selectedPixel = *inputPtr;

		if (x == 0 && y == 0 || y == imageHeight - 1 || x == imageWidth - 1)
		{
			selectedPixel = NO_EDGE;
		}
		else if (selectedPixel == WEAK_EDGE)
		{
			neighbourPixel = inputPtr;
			neighbourPixel -= imageWidth + 1;

			CheckNeighbours(neighbourPixel, selectedPixel);
		}

		*outputPtr = selectedPixel;

		inputPtr++;
		outputPtr++;
		Move(x, y, 0, imageWidth);
	}

}



void ChangeBrightness(unsigned char* outputPtr, unsigned char* inputPtr, signed short brightness, size_t imageSize)
{
	int newBrightness;
	for (int i = 0; i < imageSize; i++)
	{
		newBrightness = *inputPtr;
		newBrightness += brightness;
		if (newBrightness > 255)
		{
			*outputPtr = 255;
		}
		else if (newBrightness < 0)
		{
			*outputPtr = 0;
		}
		else
		{
			*outputPtr = newBrightness;
		}
		inputPtr++;
		outputPtr++;
	}
}