# Canny Edge Detection

A low-level C++ Win32API program made to compare execution speed between Canny Edge Detection algorithm written using both C++ and x64 Assembly. The two versions of the algorithm are stored in separate DLLs and can be dynamically linked to the main GUI app.

The program's input is a .bmp image chosen by the user, and the output is a black&white 

The GUI allows the user to:
- load an image (24-bit .bmp, max size 7000x7000, total pixel count cannot exceede 1 500 000),
- specify the output image's filename,
- choose DLL (C++ / Assembly),
- choose number of threads (1 - 64),
- manipulate values used by the algorithm (modifiable values: Gaussian blur's standard deviation, high and low hysteresis threshold),
- enable/disable steps of the algorithm,
- read execution time.

The Assembly library uses vector operations. The program was written on an older processor (Intel Core i7-2700K), so the newest instruction set used is AVX.

The C++ library covers all five steps of the algorithm and the Assembly library covers the first three. This disrepancy is a result of the latter's immense length (1500+ lines including comments). There are two consequences of this:
- the last two steps (if enabled) are always done by the C++ DLL,
- execution time is shown separately for steps 1 - 3 and for steps 1 - 5.

To make the comparison more fair, both libraries use statically defined arrays.

To load and save image, this program uses the Bitmap Image Reader Writer Library by Arash Partow (on the MIT License): https://github.com/ArashPartow/bitmap

Technologies used: C++, Win32API, x64 Assembly