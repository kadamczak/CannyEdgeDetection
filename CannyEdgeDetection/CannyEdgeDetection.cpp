//#include "framework.h"
//#include <objidl.h>
//#include <gdiplus.h>
//using namespace Gdiplus;

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <shobjidl.h>
#include <filesystem>
#include <utility>
#include <limits.h>
#include <cmath>
#include <mutex>
#include "windows.h"
#include "windowsx.h"
#include "atlstr.h"
#include "CannyEdgeDetection.h"
#include "bitmap_image.hpp"
#include <thread>

using namespace std::chrono;

#define MAX_LOADSTRING 100
#define WIN32_LEAN_AND_MEAN

//EVENTS
#define START_ALGORITHM 101
#define GET_INPUT_FILE_PATH 102   
#define GAUSS_CHECK 200
#define SOBEL_CHECK 201
#define THIN_CHECK 202
#define HYS_CHECK 203
#define TRACKING_CHECK 204
#define CPP_RADIO 108

//PROGRAM VARIABLES
HINSTANCE hInst;                      //Current instance
WCHAR szTitle[MAX_LOADSTRING];        //The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];  //the main window class name

//LABELS
HWND hErrorLabel;
HWND hTimeLabel;
HWND hFullTimeLabel;

//LABEL ERROR STRINGS
const std::wstring emptyMessage = L"";
const std::wstring libraryNotSelectedMessage = L"Library not selected!";
const std::wstring fileNotChosenMessage = L"Files haven't been selected!";
const std::wstring outputInvalidMessage = L"Invalid output filename!";
const std::wstring valuesInvalidMessage = L"Invalid custom values!";
const std::wstring invalidThreadNumberMessage = L"Invalid number of threads!";
const std::wstring imageTooLargeMessage = L"Image too large!";

//CLICKABLE ELEMENTS
//FILE
HWND hStartAlgorithmButton;           //Handler of button starting the algorithm
HWND hInputFilePathButton;            //Handler of button opening file dialog
HWND hOutputFilePathTextBox;          //Handler of textbox holding name of output file
//LIBRARY
HWND hRadioButtonCpp;
HWND hRadioButtonAsm;
//THREADS
HWND hThreadsTextBox;
//VARIABLES
HWND hVarianceTextBox;
HWND hHigherThresholdTextBox;
HWND hLowerThresholdTextBox;
//ALGORITHM STEPS
HWND hGaussFilterCheckBox;
HWND hSobelFilterCheckBox;
HWND hEdgeThinningCheckBox;
HWND hHysteresisCheckBox;
HWND hEdgeTrackingCheckBox;

//IMAGE VARIABLES
HWND hInputImageContainer;            //Handlers for displaying images in the program
HWND hOutputImageContainer;

HBITMAP hInputImageHandler;           //Handlers for the image data itself
HBITMAP hOutputImageHandler;

//FILE VARIABLES
std::string inputFilePath;
bool inputFileHasBeenSelected = false;

//LIBRARY VARIABLES
typedef void(__cdecl* brightnessPROC)(unsigned char*, unsigned char*, short, int);

typedef void(__cdecl* setupPROC)(const int, const int);

typedef void(__cdecl* setupExtensionsPROC)(unsigned char*);
typedef void(__cdecl* setupGaussPROC)(const float);
typedef void(__cdecl* blurPROC)(unsigned char*, unsigned char*, const int, const int);
typedef void(__cdecl* cleanAfterGaussPROC)();

typedef void(__cdecl* setupSobelPROC)();
typedef void(__cdecl* applySobelPROC)(unsigned char*, const int, const int);
typedef void(__cdecl* normalizeSobelPROC)(unsigned char*, const int, const int);
typedef void(__cdecl* thinEdgesPROC)(unsigned char*, unsigned char*, const int, const int);
typedef void(__cdecl* cleanIntensityAnglePROC)();

typedef void(__cdecl* hysteresisPROC)(unsigned char*, unsigned char*, const int, const int, const uint8_t, const uint8_t);
typedef void(__cdecl* trackEdgesPROC)(unsigned char*, unsigned char*, const int, const int);

typedef void(__cdecl* voidPROC)();
HINSTANCE library;
BOOL fFreeResult, fRunTimeLinkSuccess = FALSE;

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR    lpCmdLine,
    _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // TODO: Place code here.

    // Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_CANNYEDGEDETECTION, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // Perform application initialization:
    if (!InitInstance(hInstance, nCmdShow))
    {
        return FALSE;
    }

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_CANNYEDGEDETECTION));

    MSG msg;

    // Main message loop:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int)msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_CANNYEDGEDETECTION));
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
    wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_CANNYEDGEDETECTION);
    wcex.lpszClassName = szWindowClass;
    wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
    hInst = hInstance; // Store instance handle in our global variable

    HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

    if (!hWnd)
    {
        return FALSE;
    }

    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);

    return TRUE;
}

//Loads images from files to HANDLERS
//LOADING TO HANDLERS IS OPTIONAL
//void LoadImage(HWND hWnd)
//{
    //hInputImageHandler = (HBITMAP)LoadImage(nullptr, L"tiny_image.bmp", IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE);
    //hOutputImageHandler = (HBITMAP)LoadImage(nullptr, L"small_image.bmp", IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE);
//}

//Add elements to GUI
void AddControls(HWND hWnd)
{
    hInputFilePathButton = CreateWindowW(L"Button", L"Load file", WS_VISIBLE | WS_CHILD, 10, 10, 70, 30, hWnd, (HMENU)GET_INPUT_FILE_PATH, nullptr, nullptr);
    hOutputFilePathTextBox = CreateWindowEx(WS_EX_PALETTEWINDOW, TEXT("Edit"), NULL, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_LEFT, 10, 60, 200, 25, hWnd, NULL, NULL, NULL);
    hStartAlgorithmButton = CreateWindowW(L"Button", L"Apply algorithm", WS_VISIBLE | WS_CHILD, 10, 100, 300, 31, hWnd, (HMENU)START_ALGORITHM, nullptr, nullptr);

    hThreadsTextBox = CreateWindowEx(WS_EX_PALETTEWINDOW, TEXT("Edit"), TEXT("1"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_LEFT, 320, 39, 22, 20, hWnd, NULL, NULL, NULL);
    HWND hThreadsLabel = CreateWindow(L"STATIC", L"Number of threads (1-64)", WS_CHILD | WS_VISIBLE, 350, 40, 180, 20, hWnd, NULL, NULL, NULL);

    HWND hCppAsm = CreateWindow(L"STATIC", L"C++ / ASM", WS_CHILD | WS_VISIBLE, 980, 3, 70, 15, hWnd, NULL, NULL, NULL);
    HWND hCpp = CreateWindow(L"STATIC", L"C++", WS_CHILD | WS_VISIBLE, 997, 103, 30, 15, hWnd, NULL, NULL, NULL);


    hRadioButtonCpp = CreateWindowEx(WS_EX_WINDOWEDGE,
        L"BUTTON",
        L"C++ library",
        WS_VISIBLE | WS_CHILD | BS_AUTORADIOBUTTON | WS_GROUP | BST_CHECKED,
        350, 70,
        180, 20,
        hWnd,
        (HMENU)CPP_RADIO,
        hInst, NULL);
    hRadioButtonAsm = CreateWindowEx(WS_EX_WINDOWEDGE,
        L"BUTTON",
        L"Assembly library",
        WS_VISIBLE | WS_CHILD | BS_AUTORADIOBUTTON,
        350, 100,
        180, 20,
        hWnd,
        NULL,
        hInst, NULL);

    hErrorLabel = CreateWindow(L"STATIC", L"", WS_CHILD | WS_VISIBLE, 350, 130, 190, 20, hWnd, NULL, NULL, NULL);

    ShowWindow(hErrorLabel, SW_SHOW);

    hVarianceTextBox = CreateWindowEx(WS_EX_PALETTEWINDOW, TEXT("Edit"), TEXT("2.0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_LEFT, 550, 30, 100, 25, hWnd, NULL, NULL, NULL);
    hHigherThresholdTextBox = CreateWindowEx(WS_EX_PALETTEWINDOW, TEXT("Edit"), TEXT("70"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_LEFT, 550, 70, 100, 25, hWnd, NULL, NULL, NULL);
    hLowerThresholdTextBox = CreateWindowEx(WS_EX_PALETTEWINDOW, TEXT("Edit"), TEXT("40"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_LEFT, 550, 110, 100, 25, hWnd, NULL, NULL, NULL);

    HWND hVarianceLabel = CreateWindow(L"STATIC", L"Standard deviation", WS_CHILD | WS_VISIBLE, 670, 32, 200, 20, hWnd, NULL, NULL, NULL);
    HWND hHigherThresholdLabel = CreateWindow(L"STATIC", L"Higher threshold", WS_CHILD | WS_VISIBLE, 670, 72, 200, 20, hWnd, NULL, NULL, NULL);
    HWND hLoweThresholdLabel = CreateWindow(L"STATIC", L"Lower threshold", WS_CHILD | WS_VISIBLE, 670, 112, 200, 20, hWnd, NULL, NULL, NULL);

    hGaussFilterCheckBox = CreateWindow(L"BUTTON", L"Apply Gauss filter", WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 900, 20, 250, 20, hWnd, (HMENU)GAUSS_CHECK, NULL, NULL);
    hSobelFilterCheckBox = CreateWindow(L"BUTTON", L"Apply Sobel filter", WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 900, 45, 250, 20, hWnd, (HMENU)SOBEL_CHECK, NULL, NULL);
    hEdgeThinningCheckBox = CreateWindow(L"BUTTON", L"Thin up edges", WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 900, 70, 250, 20, hWnd, (HMENU)THIN_CHECK, NULL, NULL);
    hHysteresisCheckBox = CreateWindow(L"BUTTON", L"Apply hysteresis thresholding", WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 900, 120, 250, 20, hWnd, (HMENU)HYS_CHECK, NULL, NULL);
    hEdgeTrackingCheckBox = CreateWindow(L"BUTTON", L"Perform hysteresis edge tracking", WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 900, 145, 250, 20, hWnd, (HMENU)TRACKING_CHECK, NULL, NULL);

    hTimeLabel = CreateWindow(L"STATIC", L"Time (1-3 steps): ", WS_CHILD | WS_VISIBLE, 350, 150, 240, 20, hWnd, NULL, NULL, NULL);
    hFullTimeLabel = CreateWindow(L"STATIC", L"Time (1-5 steps): ", WS_CHILD | WS_VISIBLE, 350, 180, 240, 20, hWnd, NULL, NULL, NULL);
    ShowWindow(hTimeLabel, SW_HIDE);
    ShowWindow(hFullTimeLabel, SW_HIDE);

    CheckDlgButton(hWnd, CPP_RADIO, BST_CHECKED);
    CheckDlgButton(hWnd, GAUSS_CHECK, BST_CHECKED);
    CheckDlgButton(hWnd, SOBEL_CHECK, BST_CHECKED);
    CheckDlgButton(hWnd, THIN_CHECK, BST_CHECKED);
    CheckDlgButton(hWnd, HYS_CHECK, BST_CHECKED);
    CheckDlgButton(hWnd, TRACKING_CHECK, BST_CHECKED);

    //OPTIONAL DISPLAY
    //Fill visible container with image data
    hInputImageContainer = CreateWindowW(L"Static", NULL, WS_VISIBLE | WS_CHILD | SS_BITMAP, 10, 230, 300, 300, hWnd, NULL, NULL, NULL);

    //Fill visible container with image data
    //hOutputImageContainer = CreateWindowW(L"Static", NULL, WS_VISIBLE | WS_CHILD | SS_BITMAP, 310, 150, 300, 300, hWnd, NULL, NULL, NULL);
    //SendMessageW(hOutputImageContainer, STM_SETIMAGE, IMAGE_BITMAP, (LPARAM)hOutputImageHandler);
}

//Convert 3-channel .bmp to 1-channel .bmp
std::vector<unsigned char>* ConvertToGreyscale(bitmap_image img)
{
    std::vector<unsigned char>* greyscaleImg = new std::vector<unsigned char>();

    unsigned char red, green, blue;
    uint8_t avg;

    for (int i = 0; i < img.height(); i++)
    {
        for (int j = 0; j < img.width(); j++)
        {
            red = img.red_channel(j, i);
            green = img.green_channel(j, i);
            blue = img.blue_channel(j, i);
            avg = (red + green + blue) / 3;
            greyscaleImg->push_back(avg);
        }
    }

    return greyscaleImg;
}

void GetInputFilePath(HWND hWnd)
{
    ShowWindow(hInputImageContainer, SW_HIDE);
    inputFileHasBeenSelected = false;

    HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    IFileOpenDialog* pFileOpen;
    hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL,
        IID_IFileOpenDialog, reinterpret_cast<void**>(&pFileOpen));
    hr = pFileOpen->Show(NULL);
    if (SUCCEEDED(hr))
    {
        IShellItem* pItem;
        hr = pFileOpen->GetResult(&pItem);
        PWSTR pszFilePath;
        hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);
        pItem->Release();

        //important
        inputFilePath = CW2A(pszFilePath);
        hInputImageHandler = (HBITMAP)LoadImage(nullptr, pszFilePath, IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE);
        SendMessageW(hInputImageContainer, STM_SETIMAGE, IMAGE_BITMAP, (LPARAM)hInputImageHandler);
        inputFileHasBeenSelected = true;
        ShowWindow(hInputImageContainer, SW_SHOW);
    }
    pFileOpen->Release();
}

bool IsOutputFileValid(const std::wstring& filePath)
{
    std::filesystem::path path(filePath);
    return (path.extension() == ".bmp");
}

bool AreValuesValid(const double& variance, const int& highThreshold, const int& lowThreshold)
{
    if (variance <= 0 || variance > 20.01)
        return false;
    if (highThreshold <= 0 || highThreshold > 255)
        return false;
    if (lowThreshold <= 0 || lowThreshold > 255 || lowThreshold > highThreshold)
        return false;

    return true;
}

std::wstring TextBoxToWstring(HWND hTextBox)
{
    std::wstring buffer(GetWindowTextLength(hTextBox) + 1, 0);
    int size = GetWindowText(hTextBox, &buffer[0], buffer.size());
    buffer.resize(size);
    return buffer;
}

double TextBoxToFloat(HWND hTextField)
{
    std::wstring buffer = TextBoxToWstring(hTextField);
    float d = (float)_wtof(buffer.c_str());
    return d;
}

int TextBoxToInt(HWND hTextField)
{
    std::wstring buffer = TextBoxToWstring(hTextField);
    int i = (int)_wtof(buffer.c_str());
    return i;
}

void SwapPointers(unsigned char*& inputPtr, unsigned char*& outputPtr, std::vector<unsigned char>*& inputImage, std::vector<unsigned char>*& outputImage)
{
    if (inputPtr == inputImage->data())
    {
        inputPtr = outputImage->data();
        outputPtr = inputImage->data();
    }
    else
    {
        inputPtr = inputImage->data();
        outputPtr = outputImage->data();
    }
}

void DeleteBorder(unsigned char* image, const int& width, const int& height)
{
    if (height < 5)
    {
        return;
    }

    unsigned char* imgCopy = image;

    for (int i = 0; i < (2 * width); i++)
    {
        *image = 0;
        image++;
    }

    int gapBetweenBorders = height - 4;
    for (int i = 0; i < gapBetweenBorders; i++)
    {
        image += width;
    }

    for (int i = 0; i < (2 * width); i++)
    {
        *image = 0;
        image++;
    }

    if (width < 5)
    {
        return;
    }

    image = imgCopy;
    for (int i = 0; i < height; i++)
    {
        *image = 0;
        image++;
        *image = 0;
        image += width - 1;
    }

    image = imgCopy;
    image += width - 2;
    for (int i = 0; i < height; i++)
    {
        *image = 0;
        image++;
        *image = 0;
        image += width - 1;
    }

}

void StartAlgorithm(HWND hWnd)
{
    ShowWindow(hTimeLabel, SW_HIDE);
    ShowWindow(hFullTimeLabel, SW_HIDE);
    ShowWindow(hErrorLabel, SW_SHOW);

    LPCWSTR errorMessage = emptyMessage.c_str();
    SetWindowText(hErrorLabel, errorMessage);

    //Check if input file has been selected
    if (!inputFileHasBeenSelected)
    {
        errorMessage = fileNotChosenMessage.c_str();
        SetWindowText(hErrorLabel, errorMessage);
        return;
    }

    //Check if output file is correct
    std::wstring outputFilePath = TextBoxToWstring(hOutputFilePathTextBox);
    if (!IsOutputFileValid(outputFilePath))
    {
        errorMessage = outputInvalidMessage.c_str();
        SetWindowText(hErrorLabel, errorMessage);
        return;
    }

    //Get values
    float variance = TextBoxToFloat(hVarianceTextBox);
    int thresholdH = TextBoxToInt(hHigherThresholdTextBox);
    int thresholdL = TextBoxToInt(hLowerThresholdTextBox);

    //Check if values are correct
    if (!AreValuesValid(variance, thresholdH, thresholdL))
    {
        errorMessage = valuesInvalidMessage.c_str();
        SetWindowText(hErrorLabel, errorMessage);
        return;
    }

    //Cast threshold values to type uint8_t
    uint8_t highThreshold = static_cast<uint8_t>(thresholdH);
    uint8_t lowThreshold = static_cast<uint8_t>(thresholdL);

    //Get thread amount
    int threadAmount = TextBoxToInt(hThreadsTextBox);

    //Check if thread amount is correct
    if (threadAmount < 1 || threadAmount > 64)
    {
        errorMessage = invalidThreadNumberMessage.c_str();
        SetWindowText(hErrorLabel, errorMessage);
        return;
    }

    //LOAD IMAGE FROM FILE
    bitmap_image image(inputFilePath);

    //Get image dimensions
    int imageWidth = image.width();
    int imageHeight = image.height();

    //Calculate pixel ranges covered by threads
    int pixelAmountWholeImage = imageWidth * imageHeight;
    threadAmount = (pixelAmountWholeImage < threadAmount) ? pixelAmountWholeImage : threadAmount;
    int pixelAmountArea = pixelAmountWholeImage / threadAmount;
    int pixelAmountAreaRemaining = pixelAmountArea + (pixelAmountWholeImage % threadAmount);

    if (pixelAmountWholeImage > 1500000)
    {
        errorMessage = imageTooLargeMessage.c_str();
        SetWindowText(hErrorLabel, errorMessage);
        return;
    }

    //Convert to greyscale
    std::vector<unsigned char>* inputImage = ConvertToGreyscale(image);
    std::vector<unsigned char>* outputImage = new std::vector<unsigned char>(inputImage->size());

    //Set pointers to first pixel of input image and first pixel of output image
    unsigned char* inputPtr = inputImage->data();
    unsigned char* outputPtr = outputImage->data();

    bool cppLoaded;

    bool applyBorder = false;

    //Load C++ or assembly library depending on selected radio button
    if (Button_GetState(hRadioButtonCpp) == BST_CHECKED)
    {
        //cpp
        library = LoadLibrary(TEXT("CannyEdgeDetectionAlgorithmCpp.dll"));
        cppLoaded = true;
    }
    else
    {
        //asm
        library = LoadLibrary(TEXT("CannyEdgeDetectionAlgorithmAsm.dll"));
        cppLoaded = false;
    }

    fRunTimeLinkSuccess = TRUE;
    ShowWindow(hErrorLabel, SW_HIDE);

    auto start = high_resolution_clock::now();

    //Load image dimensions to library
    setupPROC loadImageDimensions = (setupPROC)GetProcAddress(library, "SetupImageDimensions");
    loadImageDimensions(imageWidth, imageHeight);

    //Perform enabled algorithm steps
    if (Button_GetState(hGaussFilterCheckBox) == BST_CHECKED)
    {
        setupExtensionsPROC createImageExtensions = (setupExtensionsPROC)GetProcAddress(library, "SetupImageExtensions");
        createImageExtensions(inputPtr);

        setupGaussPROC createGaussKernel = (setupGaussPROC)GetProcAddress(library, "SetupGaussKernel");
        createGaussKernel(variance);

        blurPROC blurImage = (blurPROC)GetProcAddress(library, "ConvoluteWithGaussKernel");

        std::vector<std::thread> blurThreads;
        for (int i = 0; i < threadAmount - 1; i++)
        {
            blurThreads.emplace_back(std::thread(blurImage, outputPtr, inputPtr, i * pixelAmountArea, pixelAmountArea));
        }
        blurThreads.emplace_back(std::thread(blurImage, outputPtr, inputPtr, (threadAmount - 1) * pixelAmountArea, pixelAmountAreaRemaining));

        for (auto& thread : blurThreads)
        {
            if (thread.joinable())
                thread.join();
        }

        cleanAfterGaussPROC cleanAfterGauss = (cleanAfterGaussPROC)GetProcAddress(library, "CleanAfterGauss");
        cleanAfterGauss();

        SwapPointers(inputPtr, outputPtr, inputImage, outputImage);
    }

    if (Button_GetState(hSobelFilterCheckBox) == BST_CHECKED)
    {
        applyBorder = true;

        //Setup intensity table and angle table
        setupSobelPROC createIntensityAngleTables = (setupSobelPROC)GetProcAddress(library, "InitializeIntensityAngleTables");
        createIntensityAngleTables();

        //Apply sobel filter
        applySobelPROC applySobel = (applySobelPROC)GetProcAddress(library, "ConvoluteWithSobelKernel");
        std::vector<std::thread> sobelThreads;

        for (int i = 0; i < threadAmount - 1; i++)
        {
            sobelThreads.emplace_back(std::thread(applySobel, inputPtr, i * pixelAmountArea, pixelAmountArea));
        }

        sobelThreads.emplace_back(std::thread(applySobel, inputPtr, (threadAmount - 1) * pixelAmountArea, pixelAmountAreaRemaining));

        for (auto& thread : sobelThreads)
        {
            if (thread.joinable())
                thread.join();
        }

        //Normalize results
        normalizeSobelPROC normalizeSobel = (normalizeSobelPROC)GetProcAddress(library, "NormalizeSobel");

        std::vector<std::thread> normalizeThreads;
        for (int i = 0; i < threadAmount - 1; i++)
        {
            normalizeThreads.emplace_back(std::thread(normalizeSobel, outputPtr, i * pixelAmountArea, pixelAmountArea));
        }
        normalizeThreads.emplace_back(std::thread(normalizeSobel, outputPtr, (threadAmount - 1) * pixelAmountArea, pixelAmountAreaRemaining));

        for (auto& thread : normalizeThreads)
        {
            if (thread.joinable())
                thread.join();
        }

        SwapPointers(inputPtr, outputPtr, inputImage, outputImage);

        if (Button_GetState(hEdgeThinningCheckBox) == BST_CHECKED)
        {
            thinEdgesPROC thinEdges = (thinEdgesPROC)GetProcAddress(library, "ThinEdges");
            std::vector<std::thread> thinThreads;
            for (int i = 0; i < threadAmount - 1; i++)
            {
                thinThreads.emplace_back(std::thread(thinEdges, outputPtr, inputPtr, i * pixelAmountArea, pixelAmountArea));
            }
            thinThreads.emplace_back(std::thread(thinEdges, outputPtr, inputPtr, (threadAmount - 1) * pixelAmountArea, pixelAmountAreaRemaining));

            for (auto& thread : thinThreads)
            {
                if (thread.joinable())
                    thread.join();
            }

            SwapPointers(inputPtr, outputPtr, inputImage, outputImage);
        }

        cleanIntensityAnglePROC cleanIntensityAngle = (cleanIntensityAnglePROC)GetProcAddress(library, "CleanIntensityAngleTables");
        cleanIntensityAngle();
    }

    //Count time
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);

    float miliseconds = duration.count() / 1000.0;

    std::string durationMessage = "Time (1-3 steps): ";
    durationMessage += std::to_string(miliseconds);
    durationMessage += " ms";

    std::wstring stemp = std::wstring(durationMessage.begin(), durationMessage.end());
    LPCWSTR sw = stemp.c_str();

    SetWindowText(hTimeLabel, sw);
    ShowWindow(hTimeLabel, SW_HIDE);
    ShowWindow(hTimeLabel, SW_SHOW);


    if (Button_GetState(hHysteresisCheckBox) == BST_CHECKED)
    {
        applyBorder = true;
        //--------------------

        if (!cppLoaded)
        {
            fFreeResult = FreeLibrary(library);

            library = LoadLibrary(TEXT("CannyEdgeDetectionAlgorithmCpp.dll"));
            fRunTimeLinkSuccess = TRUE;

            setupPROC loadImageDimensions = (setupPROC)GetProcAddress(library, "SetupImageDimensions");
            loadImageDimensions(imageWidth, imageHeight);
        }

        //--------------------


        hysteresisPROC applyHysteresis = (hysteresisPROC)GetProcAddress(library, "CategorizeEdges");
        std::vector<std::thread> hysteresisThreads;

        for (int i = 0; i < threadAmount - 1; i++)
        {
            hysteresisThreads.emplace_back(std::thread(applyHysteresis, outputPtr, inputPtr, i * pixelAmountArea, pixelAmountArea, thresholdH, thresholdL));
        }
        hysteresisThreads.emplace_back(std::thread(applyHysteresis, outputPtr, inputPtr, (threadAmount - 1) * pixelAmountArea, pixelAmountAreaRemaining, thresholdH, thresholdL));

        for (auto& thread : hysteresisThreads)
        {
            if (thread.joinable())
                thread.join();
        }

        SwapPointers(inputPtr, outputPtr, inputImage, outputImage);
    }

    if (Button_GetState(hEdgeTrackingCheckBox) == BST_CHECKED)
    {
        applyBorder = true;
        trackEdgesPROC trackEdges = (trackEdgesPROC)GetProcAddress(library, "ConnectEdges");
        std::vector<std::thread> trackingThreads;

        for (int i = 0; i < threadAmount - 1; i++)
        {
            trackingThreads.emplace_back(std::thread(trackEdges, outputPtr, inputPtr, i * pixelAmountArea, pixelAmountArea));
        }
        trackingThreads.emplace_back(std::thread(trackEdges, outputPtr, inputPtr, (threadAmount - 1) * pixelAmountArea, pixelAmountAreaRemaining));

        for (auto& thread : trackingThreads)
        {
            if (thread.joinable())
                thread.join();
        }

        SwapPointers(inputPtr, outputPtr, inputImage, outputImage);
    }

    //Count time
    auto stopFull = high_resolution_clock::now();
    auto durationFull = duration_cast<microseconds>(stopFull - start);

    float milisecondsFull = durationFull.count() / 1000;

    std::string durationMessageFull = "Time (1-5 steps): ";
    durationMessageFull += std::to_string(milisecondsFull);
    durationMessageFull += " ms";

    std::wstring stempFull = std::wstring(durationMessageFull.begin(), durationMessageFull.end());
    LPCWSTR swFull = stempFull.c_str();

    SetWindowText(hFullTimeLabel, swFull);
    ShowWindow(hFullTimeLabel, SW_HIDE);
    ShowWindow(hFullTimeLabel, SW_SHOW);

    fFreeResult = FreeLibrary(library);

    if (outputPtr == inputImage->data())
    {
        if (applyBorder)
        {
            DeleteBorder(inputPtr, imageWidth, imageHeight);
        }

        for (int i = 0; i < image.data_.size(); i++)
        {
            image.data_[i] = outputImage->at(i / 3);
        }
    }
    else
    {
        if (applyBorder)
        {
            DeleteBorder(inputPtr, imageWidth, imageHeight);
        }

        for (int i = 0; i < image.data_.size(); i++)
        {
            image.data_[i] = inputImage->at(i / 3);
        }
    }

    image.save_image(outputFilePath);

    delete inputImage;
    delete outputImage;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_COMMAND:
    {
        int wmId = LOWORD(wParam);
        // Parse the menu selections:
        switch (wmId)
        {
        case IDM_ABOUT:
            DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
            break;
        case START_ALGORITHM:
            //GetOutputFileName(hOutputFileNameTextBox);
            StartAlgorithm(hStartAlgorithmButton);
            break;
        case GET_INPUT_FILE_PATH:
            GetInputFilePath(hInputFilePathButton);
            break;
        case IDM_EXIT:
            DestroyWindow(hWnd);
            break;
        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
        }
    }
    break;
    case WM_CREATE:
    {
        AddControls(hWnd);
    }
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);
        // TODO: Add any drawing code that uses hdc here...
        EndPaint(hWnd, &ps);
    }
    break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}