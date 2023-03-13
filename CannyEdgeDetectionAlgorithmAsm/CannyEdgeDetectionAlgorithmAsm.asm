;Autor: Kinga Adamczak
;Semestr 5 studiów I stopnia, rok akademicki 2022/2023
;Kierunek: Informatyka SSI Gliwice
;Grupa 4, sekcja 2
;Temat projektu: wykrywanie krawêdzi w obrazie za pomoc¹ metody Canny'ego
;Data prezentacji projektu: 15.02.2023
;Prowadz¹cy: mgr in¿. Oleg Antemijczuk

printf proto
includelib msvcrt.lib

.data
;Maximum possible value of imageWidth and imageHeight is 7000.
imageWidth dd ?     ;Stores width of image, measured in pixels
imageHeight dd ?    ;Stores height of image, measured in pixels

;-----------------------------------------------------------------------------------
;VARIABLES USED DURING GAUSSIAN BLUR
;-----------------------------------------------------------------------------------

;IMAGE EXTENSIONS
;There are many approaches to dealing with border pixels, ranging from simple ones
;(e.g. ignoring them and setting their value to 0) to more complex solutions.

;This Gaussian blur implementation takes the approach of saving borders of the image in
;five readily available, separate arrays. When the algorithm needs a pixel whose
;coordinates land outside of image, the corresponding border pixel is taken
;in place of the non-existing one.

;As a result, the whole image can be blurred without leaving out black border pixels.
imageExtensionsTop db 7000 dup (0)
imageExtensionsRight db 7000 dup (0)
imageExtensionsBottom db 7000 dup (0)
imageExtensionsLeft db 7000 dup (0)
imageExtensionsCorners db 4 dup (0)

;Offsets of arrays mentioned above
topOffset dq OFFSET imageExtensionsTop
rightOffset dq OFFSET imageExtensionsRight
bottomOffset dq OFFSET imageExtensionsBottom
leftOffset dq OFFSET imageExtensionsLeft

;GAUSS KERNEL INFORMATION
gaussSize dd ?          ;Stores width/height of the rectangular Gauss Kernel
gaussRadius dd ?        ;Stores radius of the Gauss Kernel
gaussSum dd ?           ;Stores sum of the Gauss Kernel's elements

;GAUSS APPROXIMATION VARIABLES
;FULL EQUATION: 1/(2 * PI * sigma) * e^(-((X * X) + (Y * Y) / (2 * sigma * sigma))

gaussTempValue real4 ?  ;Stores a temporary value of e^(-((X * X) + (Y * Y) / (2 * sigma * sigma))
ePower real4 ?          ;Stores a temporary value of (-((X * X) + (Y * Y) / (2 * sigma * sigma))
_1div2PI real4 0.159155 ;Stores 1/(2*PI)

;GAUSS KERNEL
gaussKernel real4 1681 dup (0.0)            ;Stores Gauss Kernel values
                                            ;its length equals 1681 because
                                            ;the maximum width/height is 41
                                            ;41 * 41 = 1681
gaussKernelOffset dq OFFSET gaussKernel     ;Offset of Gauss Kernel

;-----------------------------------------------------------------------------------
;MATH CONSTANTS ETC.
;-----------------------------------------------------------------------------------
_0 real4 0.0
_1 real4 1.0
_2 real4 2.0

;Used for calculating gradient angles
_minus_67_5 real4 -67.5
_minus_22_5 real4 -22.5
_22_5 real4 22.5
_67_5 real4 67.5
_90 real4 90.0
_255 real4 255.0
_0b db 0
_45b db 45
_90b db 90
_135b db 135
_toDegrees real4 57.295795  ;rad -> deg conversion

M_PI real4 3.1415926
M_E  real4 2.7182818

;-----------------------------------------------------------------------------------
;VARIABLES USED DURING SOBEL CONVOLUTION
;-----------------------------------------------------------------------------------

;SOBEL KERNEL
sobelKernelX db 1, 0, -1, 2, 0, -2,  1, 0,  -1  ;Vertical Sobel Kernel, used to calculate Gx                     
sobelKernelY db 1, 2,  1, 0, 0,  0, -1, -2, -1  ;Horizontal Sobel Kernel, used to calculate Gy

;SOBEL KERNEL OFFSETS
sobelKernelXoffset dq OFFSET sobelKernelX
sobelKernelYoffset dq OFFSET sobelKernelY

;Tables containing gradient intensity and gradient angle for every
;pixel in the image (max amount of pixels: 1 500 000)
intensityTable real4 1500000 dup (0.0)
angleTable db 1500000 dup (0)

;Offsets of tables above
intensityTableOffset dq OFFSET intensityTable
angleTableOffset dq OFFSET angleTable

maxG real4 ?            ;Highest gradient intensity encountered in the image
Gx real4 ?              ;Used to temporarily store Gx to calculate arctanResult = arctan(Gy/Gx)
Gy real4 ?              ;Used to temporarily store Gy to calculate arctanResult = arctan(Gy/Gx)
arctanResult real4 ?    ;Result of calculation mentioned above (in radians)

_pattern db 0, 4, 8, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ;used to select bytes during
                                                            ;gradient intensity normalization
                                                            ;to value range 0-255
                                                            ;in function NormalizeSobel

.code
;-----------------------------------------------------------------------------------
;SETUP
;-----------------------------------------------------------------------------------
SetupImageDimensions proc EXPORT
    mov imageWidth, ECX     ;257
    mov imageHeight, EDX    ;410
    ret
SetupImageDimensions endp

;-----------------------------------------------------------------------------------
;GAUSS
;-----------------------------------------------------------------------------------

;nazwa
;krotki opis
;parametry wej:

;parametry wyj:
;
;jedna z procedur - w raporcie
;kawalek assemblera, poczatek (od data)
;200~300 linii (do tresci sprawozdania)
;test (rozne rozdzielczosci obrazu)
;asm->c++->watki
;wyniki w tabelce (excel)
;wykres

;favor size/speed
;pdf -> teams (tylko)
;unsigned char* edgePixel   -   RCX
SetupImageExtensions proc EXPORT   
    ;FILL LEFT-TOP CORNER
    PUSH RBX

    mov RAX, 0

    mov AL, byte ptr [RCX]
    mov byte ptr [imageExtensionsCorners], AL   ;was I

    ;FILL TOP EXTENSION
    mov EDX, 0                                      ;EDX - i counter, start with 0

    FillTopLoop: mov AL, byte ptr [RCX]             ;read byte (pixel) of image
        mov RBX, topOffset                          ;load address of top array
        mov byte ptr [RBX + RDX], AL                ;save byte (pixel) of image to top extension array
        inc RCX                                     ;move image pointer by 1 BYTE
        inc EDX                                     ;increase counter
        cmp EDX, imageWidth                         ;
        JL FillTopLoop                              ;if counter < IMAGEWIDTH, continue reading top of image
    dec RCX

    ;FILL RIGHT-TOP CORNER
    mov AL, byte ptr [RCX]
    mov byte ptr [imageExtensionsCorners + 1], AL

    ;FILL RIGHT EXTENSION
    mov EDX, 0                                      ;EDX - i counter, start with 0
                               
    FillRightLoop: mov AL, byte ptr [RCX]           ;read byte (pixel) of image 
        mov RBX, rightOffset                        ;load address of right array
        mov byte ptr [RBX + RDX], AL                ;save byte (pixel) of image to right extension array
        mov EAX, imageWidth                         ;
        add RCX, RAX                                ;move image pointer by IMAGE WIDTH
        inc EDX                                     ;increase counter
        cmp EDX, imageHeight                        ;
        JL FillRightLoop                            ;if counter < IMAGEHEIGHT, continue reading right edge of image
    mov EAX, imageWidth
    sub RCX, RAX
    
    ;FILL RIGHT-BOTTOM CORNER
    mov AL, byte ptr [RCX]
    mov byte ptr [imageExtensionsCorners + 2], AL

    ;FILL BOTTOM EXTENSION
    mov EAX, imageWidth
    dec EAX
    mov EDX, EAX                                    ;EDX - i counter, start with (imageWidth - 1)

    FillBottomLoop: mov AL, byte ptr [RCX]          ;read byte (pixel) of image
        mov RBX, bottomOffset                       ;load address of bottom array
        mov byte ptr [RBX + RDX], AL                ;save byte (pixel) of image to bottom extension array
        dec RCX                                     ;move image pointer by 1 BYTE back
        dec EDX                                     ;decrease counter
        cmp EDX, 0                                  ;
        JGE FillBottomLoop                          ;if counter >= 0, continue reading bottom of image
    inc RCX  

    ;FILL LEFT-BOTTOM CORNER
    mov AL, byte ptr [RCX]
    mov byte ptr [imageExtensionsCorners + 3], AL

    ;FILL LEFT EXTENSION
    mov EAX, imageHeight
    dec EAX
    mov EDX, EAX                                    ;EDX - i counter, start with (imageHeight - 1)

   FillLeftLoop: mov AL, byte ptr [RCX]             ;read byte (pixel) of image
        mov RBX, leftOffset                         ;load address of left array
        mov [RBX + RDX], AL                         ;save byte (pixel) of image to left extension array
        mov EAX, imageWidth                         ;
        sub RCX, RAX                                ;move image pointer by IMAGEWIDTH back
        dec EDX                                     ;decrease counter
        cmp EDX, 0                                  ;
        JGE FillLeftLoop                            ;if counter >= 0, continue reading left edge of image
    
    POP RBX
    ret
SetupImageExtensions endp

;const float variance - XMM0    FLOAT UNSIGNED
;XMM - 128 bit registers
;YMM - 256 bit registers
SetupGaussKernel proc EXPORT
    PUSH RBX

    ;CALCULATE SIZE AND RADIUS
    ;save VARIANCE (SIGMA) to xmm2
    movss xmm2, xmm0

    ;Get initial kernel size resulting from the variance
    movss xmm1, real4 ptr [_2]
    mulss xmm0, xmm1        ;multiply VARIANCE by 2.0
    cvttss2si RAX, xmm0     ;convert result with truncation into KERNEL SIZE

    ;Make sure kernel size is an odd number
    mov RDX, RAX            ;copy size to RDX
    inc RDX                 ;store (size + 1) in RDX
    mov RCX, RAX            ;copy size to RCX
    shr RCX, 1              ;divide by 2

    cmovnc RAX, RDX         ;if (carry = 1), the number is odd and (size) remains
                            ;if (carry = 0), the number is even and (size + 1) IS COPIED

    ;If kernel size is 1, adjust it to be at least 3
    mov RDX, 3
    cmp RAX, 1
    cmove RAX, RDX

    ;Save final GAUSS KERNEL SIZE
    mov gaussSize, EAX
    
    ;Save RADIUS
    shr RAX, 1
    mov gaussRadius, EAX

    ;CALCULATE KERNEL
    mov gaussSum, 0               ;initialize SUM OF GAUSS KERNEL ELEMENTS with 0

    mov ECX, 0                    ;ECX - i counter
    mov EDX, 0                    ;EDX - j counter

    mov R8, 0                     ;distance from center X
    mov R9, 0                     ;distance from center Y
    movss xmm0, real4 ptr [_0]    ;current KERNEL SUM  
    
    mulss xmm2, xmm2              ;save (sigma * sigma) in xmm2
    movss xmm6, xmm2              ;copy to xmm6

    iLoop: mov EAX, gaussRadius         ;
        sub EAX, ECX                    ;save (gaussRadius - i) in EAX
        
        test EAX, EAX
        JNS DistanceUnsignedY

        mov R10d, ECX                   ;save (i - gaussRadius) in EAX if i is actually higher
        mov EAX, gaussRadius
        sub R10d, EAX
        mov EAX, R10d

        DistanceUnsignedY: mov R9, RAX  ;distanceFromCenterY = abs(radius - i);       
        
        jLoop: mov EAX, gaussRadius     ;
               movss xmm2, xmm6         ;restore xmm2 with (sigma * sigma)
               sub EAX, EDX             ;save (gaussRadius - j) in EAX
               
               test EAX, EAX
               JNS DistanceUnsignedX
               mov R10d, EDX            ;save (j - gaussRadius) in EAX if j is actually higher
               mov EAX, gaussRadius
               sub R10d, EAX
               mov EAX, R10d

               DistanceUnsignedX: mov R8d, EAX   ;R8 - X
               
               ;calculate [1 / (2 * M_PI * sigma * sigma)] and save in xmm1
               movss xmm1, real4 ptr [_1div2PI]   ;save 1/(2*PI) in xmm1
               divss xmm1, xmm2                   ;divide 1/(2*PI) by (sigma*sigma) and SAVE IN XMM1
               
               ;calculate [-((X * X) + (Y * Y) / (2 * sigma * sigma)] and save in ePower
               cvtsi2ss xmm4, R8                  ;convert "distance from center X" to float and save it in xmm4
               mulss xmm4, xmm4                   ;save (X * X) in xmm4
               cvtsi2ss xmm5, R9                  ;convert "distance from center Y" to float and save it in xmm5
               mulss xmm5, xmm5                   ;save (Y * Y) in xmm5
               addss xmm4, xmm5                   ;save (X * X) + (Y * Y) in xmm4
               movss xmm3, real4 ptr [_2]         ;move 2 into xmm3
               mulss xmm2, xmm3                   ;save (2 * sigma * sigma) into xmm2
               divss xmm4, xmm2                   ;divide (X * X) + (Y * Y) by (2 * sigma * sigma) ;and save in xmm4
               movss xmm5, xmm4                   ;copy result to xmm5
               subss xmm4, xmm5                   ;
               subss xmm4, xmm5                   ;invert xmm4 by using substraction 2 times
               movss real4 ptr [ePower], xmm4       ;save power from xmm4

               ;calculate and save e^(ePower) to gaussTempValue
               finit
               fld real4 ptr [ePower]
               fld real4 ptr [M_E]
               fyl2x
               fld1
               fld st(1)
               fprem
               f2xm1
               fadd
               fscale
               fstp gaussTempValue

               ;get final result (multiply xmm1 by gaussTempValue stored in xmm2)
               movss xmm2, real4 ptr [gaussTempValue]
               mulss xmm1, xmm2

               ;XMM0 - KERNEL SUM
               ;XMM1 - CURRENT VALUE

               ;add current value to KERNEL SUM
               addss xmm0, xmm1

               ;store xmm1 value at gaussKernel[i][j]
               ;width of gauss kernel - gaussSize
               ;index in array is then [gaussSize * i + j]

               ;save EDX before MUL
               mov R10d, EDX

               ;RAX - index of element
               mov EAX, dword ptr [gaussSize]   ;get gaussSize in EAX
               mul ECX                          ;get [gaussSize * i] in EAX
               mov EDX, R10D                    ;bring EDX value back
               add EAX, EDX                     ;get [gaussSize * i + j] in EAX
               mov R8d, 4                       ;
               mul R8d                          ;multiply index by 4 (for floats)
               mov EDX, R10D                    ;bring EDX value back

               ;RBX - base offset of array
               mov RBX, gaussKernelOffset

               ;MOVE VALUE INTO ARRAY (movups)
               movss real4 ptr [RBX + RAX], xmm1

               inc EDX
               cmp EDX, gaussSize
               JL jLoop
        MOV EDX, 0
        
        inc ECX
        cmp ECX, gaussSize
        JL iLoop

    movss real4 ptr [gaussSum], xmm0
    POP RBX
    ret
SetupGaussKernel endp

GetPixelValue proc
    ;SAVE REGISTERS
    PUSH RBX
    PUSH RCX
    PUSH RDX
    PUSH RSI
    PUSH RDI
    PUSH R8
    PUSH R9
    PUSH R10
    PUSH R11
    PUSH R12

    ;REGISTERS
    ;RAX - imageValue
    ;RBX - offset of selected table
    ;RCX - first selected value
    ;RDX - selectedPixel
    ;RDI - imageWidth
    ;R10 - x
    ;R11 - y
    ;R12 - imageHeight

    ;Check if pointer is out of image's bounds
    ;(image edge extension will be used)
    test R10d, R10d         ;check if (x < 0)
    JS xBelow0
    CMP R10d, EDI           ;check if (x >= imageWidth)
    JGE xGreaterEqualWidth
    test R11d, R11d         ;check if (y < 0)
    JS yBelow0
    CMP R11d, R12d          ;check if (y >= imageHeight)
    JGE yGreaterEqualHeight

    ;Pointer is in bounds of image, a pixel can be directly used
    JMP PointerInImageRange

    ;-----------------------------------------------------------------------------------------------------------------
    xBelow0: mov RBX, leftOffset
             
             ;if (y < 0), return ImageExtensionCorners[0]
             mov CL, byte ptr [imageExtensionsCorners]        ;save imageExtensionsCorners[0] in CL
             test R11d, R11d                                  ;if (y < 0),             
             CMOVS EAX, ECX                                   ;save imageExtensionCorners[0] in AL
             JS EndXbelow0

             ;if (y >= imageHeight) return ImageExtensionCorners[3]
             mov CL, byte ptr [imageExtensionsCorners + 3]    ;save imageExtensionsCorners[3] in CL     
             CMP R11d, R12d                                   ;if (y >= imageHeight),
             CMOVGE EAX, ECX                                  ;save imageExtensionCorners[3] in AL
             JGE EndXbelow0

             ;if X BELOW 0, Y IS IN BOUNDS, return ImageExtensionLeft[y]
             mov AL, byte ptr [RBX + R11]                     ;SAVE imageExtensionLeft[y] in AL

             EndXbelow0: JMP PixelValueReturned
    ;-----------------------------------------------------------------------------------------------------------------      
    
    
    ;-----------------------------------------------------------------------------------------------------------------
    xGreaterEqualWidth: mov RBX, rightOffset

             ;if (y < 0), return ImageExtensionCorners[1]
             mov CL, byte ptr [imageExtensionsCorners + 1]    ;save imageExtensionsCorners[1] in CL
             test R11d, R11d                                  ;if (y < 0),             
             CMOVS EAX, ECX                                   ;save imageExtensionCorners[1] in AL
             JS EndXgreater

             ;if (y >= imageHeight), return ImageExtensionCorners[2]
             mov CL, byte ptr [imageExtensionsCorners + 2]    ;save imageExtensionsCorners[2] in CL     
             CMP R11d, R12d                                   ;if (y >= imageHeight),
             CMOVGE EAX, ECX                                  ;save imageExtensionCorners[2] in AL
             JGE EndXgreater

             ;if X EQUAL/ABOVE IMAGEHEIGHT, Y IS IN BOUNDS, return ImageExtensionRight[y]
             mov AL, byte ptr [RBX + R11]                     ;SAVE imageExtensionRight[y] in AL

             EndXgreater: JMP PixelValueReturned
    ;-----------------------------------------------------------------------------------------------------------------


    ;-----------------------------------------------------------------------------------------------------------------
    ;if X IN BOUNDS, Y BELOW 0, return imageExtensionsTop[x]
    yBelow0: mov RBX, topOffset
             mov RSI, RBX
             add RSI, R10
             mov AL, byte ptr [RSI]                     ;SAVE imageExtensionTop[x] in AL

             JMP PixelValueReturned
    ;-----------------------------------------------------------------------------------------------------------------
    
    
    ;-----------------------------------------------------------------------------------------------------------------
    ;if X IN BOUNDS, Y EQUAL/ABOVE IMAGEHEIGHT, return imageExtensionsBottom[x]
    yGreaterEqualHeight: mov RBX, bottomOffset
             mov AL, byte ptr [RBX + R10]                     ;SAVE imageExtensionBottom[x] in AL
             
             JMP PixelValueReturned
    ;-----------------------------------------------------------------------------------------------------------------

    ;if POINTER IS IN IMAGE RANGE, return pixel
    PointerInImageRange: mov AL, byte ptr [RDX]

    ;RESTORE REGISTERS
    PixelValueReturned: POP R12
           
    POP R11
    POP R10
    POP R9
    POP R8
    POP RDI
    POP RSI
    POP RDX
    POP RCX
    POP RBX
    ret
GetPixelValue endp

;uint8_t BlurPixel(unsigned char* selectedPixel, int x, int y)
BlurPixel proc
    ;save registers
    PUSH RBX
    PUSH RCX
    PUSH RDX
    PUSH RSI
    PUSH RDI
    PUSH R8
    PUSH R9
    PUSH R10
    PUSH R11
    PUSH R12
    PUSH R13
    PUSH R14
    PUSH R15

    ;REGISTERS                                      X - continued to be used by GetPixelValue
    ;RAX    -    imageValue                         
    ;RBX    -    
    ;RCX    -    i counter                          
    ;RDX    -    COPY of inputPtr (selectedPixel)   X
    ;RSI    -    kernelSize 
    ;RDI    -    imageWidth                         X
    ;R8     -    endingX
    ;R9     -    j counter
    ;R10    -    COPY of x                          X
    ;R11    -    COPY of y                          X
    ;R12    -    imageHeight                        X
    ;R13    -    startingX

    ;xmm0   -    result
    ;xmm1   -    newValue
    
    movss xmm0, real4 ptr [_0]       ;set RESULT to 0
    movss xmm1, real4 ptr [_0]       ;set NEWVALUE to 0

    mov EBX, dword ptr [gaussRadius] ;store GAUSSRADIUS in EBX
    mov ESI, dword ptr [gaussSize]   ;store GAUSSSIZE in ESI

    sub R10, RBX                    ;save (x - gaussRadius) in R10
    sub R11, RBX                    ;save (y - gaussRadius) in R11

    mov EDI, dword ptr [imageWidth]  ;store IMAGEWIDTH in EDI

    ;SAVE RDX BEFORE MUL
    PUSH RDX

    ;calculate [radius * imageWidth + radius]
    mov RAX, RBX                     ;copy radius to RAX    
    mul RDI                          ;perform (radius * imageWidth), result in RAX
    add RAX, RBX                     ;perform (radius * imageWidth) + radius, result in RAX

    ;RESTORE RDX AFTER MUL
    POP RDX

    ;substract [radius * imageWidth + radius] from selectedPixel pointer
    sub RDX, RAX

    ;save startingX to R13
    mov R13, R10
    
    ;save endingX
    mov R8, R10                      ;save x to R8
    mov EAX, dword ptr [gaussSize]   ;save gaussSize to EAX
    add R8d, EAX                     ;save (x + gaussSize) as endingX, R8

    ;KERNEL LOOP
    mov ECX, 0                       ;initialize i counter with 0
    mov R9, 0                        ;initialize j counter with 0

    mov R12d, dword ptr [imageHeight]

    iLoop: mov R9, 0  
        jLoop: CALL GetPixelValue ;RESULT IN AL (imageValue)

            ;save imageValue and selectedPixel before MUL
            PUSH RDX            ;save selectedPixel
            PUSH RAX            ;save imageValue
            
            ;newValue = imageValue * kernel[i][j];	        
            ;result += newValue;

            ;calculate [i][j] = i * kernelSize + j
            ;and save to RAX
            mov RAX, RCX            ;move i into RAX
            mul RSI                 ;get (i * kernelSize) in RAX
            add RAX, R9             ;save (i * kernelSize + j) in RAX

            ;mul RAX by 4 (because floats are stored in the array)
            mov RDX, 4
            mul RDX

            ;save gaussKernel[i][j] to xmm2
            mov RBX, gaussKernelOffset
            movss xmm2, real4 ptr [RBX + RAX]

            ;restore imageValue
            POP RAX

            ;convert imageValue to float (save in xmm3)
            CVTSI2SS xmm3, EAX

            ;perform imageValue * gaussKernel[i][j], save in xmm3
            mulss xmm3, xmm2

            ;add xmm3 to result
            addss xmm0, xmm3

            ;restore selectedPixel
            POP RDX

            ;selectedPixel++;
            inc RDX

            ;Move(x, y, startingX, endingX);
            mov RAX, R8             ;move endingX into RAX
            dec EAX                 ;save (endingX - 1) into RAX
            CMP R10d, EAX
            JNL NextRow             ;if [ x < endingX - 1], stay in row        
            inc R10d                ;x++
            JMP EndMove
            NextRow: mov R10, R13   ;x = startingX
            inc R11d                ;y++
            EndMove: inc R9
            MOV RAX, 0
            CMP R9d, ESI
            JL jLoop
        
        add RDX, RDI    ;move selectedPixel forward by imageWidth
        sub RDX, RSI    ;move selectedPixel back by kernelSize

        inc RCX
        CMP RCX, RSI
        JL iLoop

    ;load kernelSum
    movss xmm4, REAL4 PTR [gaussSum]

    ;divide result by kernelSum (xmm0 / xmm4) and store result in xmm0
    divss xmm0, xmm4

    ;convert result to uint8_t
    CVTTSS2SI EAX, xmm0

    ;restore registers
    POP R15
    POP R14
    POP R13
    POP R12
    POP R11
    POP R10
    POP R9
    POP R8
    POP RDI
    POP RSI
    POP RDX
    POP RCX
    POP RBX
    ret
BlurPixel endp

;RCX    -   unsigned char* outputPtr
;RDX    -   unsigned char* inputPtr
;R8     -   const int startOffset
;R9     -   const int pixelAmountArea
ConvoluteWithGaussKernel proc EXPORT
    PUSH RBX
    PUSH R12

    add RCX, R8                         ;move inputPtr by startOffset
    add RDX, R8                         ;move outputPtr by startOffset
    mov RBX, RDX                        ;save RDX before division

    ;int x = startOffset % imageWidth;
    ;int y = startOffset / imageWidth;
    ;x - R10
    ;y - R11
                                        ;startOffset is in R8
    mov R11d, dword ptr [imageWidth]    ;load imageWidth

    ;perform division
    xor RDX, RDX                        ;clear EDX
    mov RAX, R8                         ;move startOffset into RAX
    div R11                             ;perform (startOffset / imageWidth) as RAX/R11
    mov R10, RDX                        ;save (startOffset % imageWidth) from RDX into R10 (x)
    mov R11, RAX                        ;save (startOffset / imageWidth) from RAX into R11 (y)

    mov RDX, RBX                        ;restore RDX after division

    mov ESI, dword ptr [imageWidth]     ;load imageWidth

    ;----------------------------------
    ;CONVOLUTION
    ;----------------------------------
    ;RSI    -   imageWidth
    ;RCX    -   unsigned char* outputPtr
    ;RDX    -   unsigned char* inputPtr
    ;R8     -   const int startOffset (no longer needed)
    ;R9     -   const int pixelAmountArea

    ;R10    -   x
    ;R11    -   y

    ;R12    -   i counter
    mov R12, 0                          ;start i with 0

    gaussLoop: 
        ;get blurred pixel value in EAX
        call BlurPixel
        
        ;move blurred pixel value to *outputPtr
        mov byte ptr [RCX], AL
        
        inc RCX     ;go to next output image pixel
        inc RDX     ;go to next input image pixel

        ;Move(x, y, 0, imageWidth);
        mov RAX, RSI            ;move imageWidth into RAX
        dec EAX                 ;save (imageWidth - 1) into RAX
        CMP R10d, EAX
        JNL NextRow             ;if [ x < imageWidth - 1], stay in row        
        inc R10d                ;x++
        JMP EndMove
        NextRow: mov R10, 0     ;x = 0
        inc R11d                ;y++
        EndMove: inc R12        ;increment i
        cmp R12, R9             ;
        JL gaussLoop            ;loop if (i < pixelAmountArea)
        
    POP R12
    POP RBX
    ret
ConvoluteWithGaussKernel endp

CleanAfterGauss proc EXPORT
    ret
CleanAfterGauss endp

;-----------------------------------------------------------------------------------
;SOBEL
;-----------------------------------------------------------------------------------

;no arguments
InitializeIntensityAngleTables proc EXPORT
    ret
InitializeIntensityAngleTables endp

;Gx - xmm3
;Gy - xmm4
GetRoundedAngle proc
    movss xmm2, real4 ptr [_90]      ;initialize result
    movss xmm5, real4 ptr [_0]
    mov RAX, 0

    ucomiss xmm3, xmm5
    je return90
    ja unsignedGx

    movss xmm6, xmm3
    subss xmm3, xmm6
    subss xmm3, xmm6

    ;save Gx
    unsignedGx: movss real4 ptr [Gx], xmm3

    ;save Gy
    movss real4 ptr [Gy], xmm4


    ;ST(1)/ST(0)
    finit
    fld real4 ptr [Gy]      ;ST(1)
    fld real4 ptr [Gx]      ;ST(0)
    fpatan
    fstp arctanResult

    movss xmm2, real4 ptr [arctanResult]
    mulss xmm2, xmm7        ;convert radians to degrees
    ucomiss xmm6, xmm5
    jae round

    movss xmm6, xmm2
    subss xmm2, xmm6
    subss xmm2, xmm6

    ;angle in xmm2
    round: movss xmm6, real4 ptr [_67_5]
    ucomiss xmm2, xmm6
    jae return90
    movss xmm6, real4 ptr [_22_5]
    ucomiss xmm2, xmm6
    jae return45
    movss xmm6, real4 ptr [_minus_22_5]
    ucomiss xmm2, xmm6
    jae return0
    movss xmm6, real4 ptr [_minus_67_5]
    ucomiss xmm2, xmm6
    jae return135
    
    return90: mov RAX, 90
        ret
    return45: mov RAX, 45
        ret
    return0: mov RAX, 0
        ret
    return135: mov RAX, 135
        ret


GetRoundedAngle endp

;RCX - selectedPixel
;RBX - kernel pointer
;return float after convolution

;FUNCTION DESCRIPTION:
;Calculates Gx or Gy of a selected pixel
;depending on which Sobel Kernel
;(vertical or horizontal)
;was passed by pointer into the function.

;INPUT REGISTERS:
;RCX - points to currently selected pixel of input image
;RBX - points to currently used Sobel Kernel
;R12 - contains width of image

;OUTPUT REGISTERS:
;xmm1   -   contains calculated Gx or Gy

;MODIFIES REGISTERS:
;xmm1, xmm2, RAX, RDX

ApplySobel proc 
    PUSH RBX    ;Save BASE REGISTER on the stack before performing
                ;operations with arrays
    PUSH R8     ;Save R8 on the stack before using it as loop counter
    PUSH R14    ;Save R14 on the stack before using it as index of Sobel Kernel elements

    ;Save original pointer of input image pixel on the stack
    PUSH RCX
    
    ;Initialize return value (stored in xmm1) with 0
    movss xmm1, real4 ptr [_0]

    ;Move pixel pointer back by [imageWidth + 1].
    ;RCX will now point at the pixel to the North-West
    ;of the original pixel.
    sub RCX, R12                ;Reduce selectedPixel by imageWidth
    dec RCX                     ;Reduce selectedPixel by 1

    ;SET STARTING VALUE OF 0 TO i counter
    mov R8, 0

    ;Set R14 to be the index of next elements in the Sobel Kernel
    ;Start with 0
    mov R14, 0

    iLoop: mov R9, 0                            ;Set j counter to 0 before iterating
           jLoop: xor RAX, RAX                  ;Clear RAX register
           xor RDX, RDX                         ;Clear RDX register
           mov AL, [RCX]                        ;Move pixel value to AL
           mov DL, byte ptr [RBX + R14]         ;Move next value from Sobel Kernel to DL
           cmp DL, 0                            ;Compare Sobel Kernel element value to 0
           JL signed                            ;If value is below 0

           ;SOBEL KERNEL ELEMENT VALUE IS ABOVE OR EQUAL TO 0
           mul DL                               ;Multiply value of pixel by value from Sobel Kernel and save in RAX
           cvtsi2ss xmm2, RAX                   ;Convert multiplication result to float
           addss xmm1, xmm2                     ;Add multiplication result to the total sum stored in xmm1
           JMP endMul                           ;Jump to end of j loop

           ;SOBEL KERNEL ELEMENT VALUE IS BELOW 0
           signed: sub DL, byte ptr [RBX + R14] ;Calculate absolute value of kernel element
           sub DL, byte ptr [RBX + R14]         ;and store it in DL
           mul DL                               ;Multiply value of pixel by value from Sobel Kernel and save in RAX
           cvtsi2ss xmm2, RAX                   ;Convert multiplication result to float     
           subss xmm1, xmm2                     ;Reduce total sum stored in xmm1 by the multiplication result

           endMul: inc R14                      ;Set Sobel Kernel index to the next element
           inc RCX                              ;Select pixel to the right of the current one
           inc R9                               ;Increase j counter
           cmp R9, 3                            ;Compare j counter to 3
           JL jLoop                             ;If j counter < 3, continue j loop

        add RCX, R12                            ;move pixel pointer forward by one image width,
        sub RCX, 3                              ;move pixel pointer back by 3,
                                                ;effectively making it point to the leftmost pixel of the next row
                                                ;in the 3x3 area surrounding the original pixel passed into the function
    
        inc R8                                  ;Increase i counter
        cmp R8, 3                               ;Compare i counter to 3
        JL iLoop                                ;If i counter < 3, continue i loop


    ;Restore saved registers
    POP RCX
    POP R14
    POP R8
    POP RBX
    RET
ApplySobel endp

;ECX - selectedPixel

;FUNCTION DESCRIPTION:
;Calculates gradient intensity and gradient angle
;for selected pixel.
;Results are saved in intensityTable and angleTable.
;If a new highest gradient intensity in the image
;is discovered, the "maxG" global variable gets updated.

;INPUT REGISTERS:
;RCX  - points to currently selected pixel of input image
;R12  - contains width of image
;R13  - index of current pixel (distance from beginning of image)
;xmm7 - contains loaded variable "_toDegrees", allowing to convert
;       radians to degrees inside of GetRoundedAngle function

;OUTPUT REGISTERS:
;none
;(results are stored in global variables:
;intensityTable, angleTable and maxG)

;MODIFIED REGISTERS:
;xmm1, xmm3, xmm4, xmm5, xmm6
;Registers modified by called function ApplySobel: RAX, RDX, xmm2
;Called function GetRoundedAngle does not modify other registers.

CalculateSobel proc
    PUSH RBX ;Save BASE REGISTER on the stack before performing
             ;operations with arrays

    ;Convolute with SobelKernelX and save resulting Gx to xmm3
    mov RBX, sobelKernelXoffset     ;Prepare pointer to SobelKernelX
    call ApplySobel                 ;Calculate Gx (result stored in xmm1)
    movss xmm3, xmm1                ;Move Gx from xmm1 to xmm3

    ;Convolute with SobelKernelY and save resulting Gy to xmm4
    mov RBX, sobelKernelYoffset     ;Prepare pointer to SobelKernelY
    call ApplySobel                 ;Calculate Gy (result stored in xmm1)
    movss xmm4, xmm1                ;Move Gy from xmm1 to xmm4

    ;Calculate (Gx * Gx) and save in xmm5
    movss xmm5, xmm3
    mulss xmm5, xmm5

    ;Calculate (Gy * Gy) and save in xmm6
    movss xmm6, xmm4
    mulss xmm6, xmm6

    ;Calculate (Gx * Gx + Gy * Gy) and save in xmm5
    addss xmm5, xmm6

    ;G = SQRT(Gx * Gx + Gy + Gy)
    sqrtss xmm5, xmm5       ;XMM5 NOW CONTAINS GRADIENT INTENSITY (G)

    ;Move maxG value into xmm6
    movss xmm6, real4 ptr [maxG]
    
    ;Update maxG if current G is higher
    ucomiss xmm5, xmm6              ;Compare current G with maxG
    jbe maxNotChanged               ;If G <= maxG, skip updating
    
    movss real4 ptr [maxG], xmm5    ;If G > maxG, save current G value into maxG

    ;Save GRADIENT INTENSITY (stored in xmm5) to intensityTable
    maxNotChanged: mov RBX, intensityTableOffset
    movss real4 ptr [RBX + R13 * 4], xmm5

    ;Calculate and save GRADIENT ANGLE in angleTable
    CALL GetRoundedAngle            ;Calculate angle (result in RAX)
    mov RBX, angleTableOffset
    mov byte ptr [RBX + R13], AL    ;Save gradient angle to angleTable

    ;Restore base register
    POP RBX
    ret
CalculateSobel endp

;RCX    -   unsigned char* inputPtr
;RDX    -   const int startOffset
;R8     -   const int pixelAmountArea

;FUNCTION DESCRIPTION:
;This function is exported and can be called by external sources.
;It iterates through all pixels of the input image
;and calculates gradient intensity and gradient angle for each of them.
;If the function is multithreaded, all threads operate on separate areas of the image.

;Results are not saved in the output image at this stage - gradient intensity
;values need to be normalized first to the 0-255 range,
;which takes place in the NormalizeSobel function.

;INPUT REGISTERS:
;RCX    -   contains pointer to first pixel of the whole image
;RDX    -   index of start pixel (distance from beginning of image),
;           is also called "startOffset"
;R8     -   number of pixels that need to be processed by this thread

;OUTPUT REGISTERS:
;none
;(results are stored in global variables:
;intensityTable, angleTable and maxG)

;MODIFIED REGISTERS:
;RAX, RDX, RDI, R10, R11, xmm0, xmm7
;Registers modified by called function CalculateSobel: xmm1 - xmm6

ConvoluteWithSobelKernel proc EXPORT
    PUSH R12    ;Save R12 register before using it to store image width
    PUSH R13    ;Save R13 register before using it as index of current pixel
   
    mov R12d, dword ptr [imageWidth] ;Save image width in R12d

    ;Move index of start pixel (startOffset) to R13
    mov R13, RDX
    ;Move pixel pointer by startOffset, making it point to the start pixel of this thread
    add RCX, R13

    ;Set x in R10 and y in R11
    ;x is the horizontal distance of current pixel from the beginning of the image
    ;y is the vertical distance of current pixel from the beginning of the image
    xor RDX, RDX                        ;Clear RDX
    mov RAX, R13                        ;Move startOffset into RAX
    div R12                             ;Calculate (startOffset / imageWidth)
    mov R10, RDX                        ;Save (startOffset % imageWidth) from RDX into R10 (x)
    mov R11, RAX                        ;Save (startOffset / imageWidth) from RAX into R11 (y)

    ;Clear registers
    xor RAX, RAX
    xor RDX, RDX

    ;Initialize i counter with 0
    mov RSI, 0

    ;Save (imageWidth - 1) in RBX
    mov RBX, R12
    dec RBX

    ;Save (imageHeight - 1) in RDI
    mov EDI, dword ptr [imageHeight]
    DEC RDI

    ;Store 0.0 in xmm0, its going to be used to fill gradient intensities
    ;on the borders of the image
    movss xmm0, real4 ptr [_0]

    ;Save rad->degrees conversion in xmm7
    movss xmm7, real4 ptr [_toDegrees]

    ;Check if pixel is on any border of the image
    sobelLoop: cmp R10, 0       ;if (x == 0) -> border
        JE imageBorder          ;
        cmp R11, 0              ;if (y == 0) -> border
        JE imageBorder          ;
        cmp R10, RBX            ;if (x == imageWidth - 1) -> border
        JE imageBorder          ;
        cmp R11, RDI            ;if (y == imageHeight - 1) -> border
        JE imageBorder
        JMP imageInside

        ;Pixel is on the border -> set gradient intensity and angle to 0
        imageBorder: PUSH RBX
               mov RBX, intensityTableOffset

               ;MOVE 0.0 INTO INTENSITY ARRAY
               movss real4 ptr [RBX + R13 * 4], xmm0
               mov RBX, angleTableOffset

               ;MOVE 0 INTO ANGLE ARRAY
               mov RDX, 0
               mov byte ptr [RBX + R13], 0

               POP RBX
               JMP sobelEnd

        ;Pixel is not on the border -> calculate gradient intensity and angle
        imageInside: call CalculateSobel    
        
        sobelEnd: inc RCX       ;Move to next pixel   

        ;Move x and y coordinates one step forward
        CMP R10d, EBX           ;
        JNL NextRow             ;if [ x < imageWidth - 1], stay in row        
        inc R10d                ;x++
        JMP EndMove             ;
        NextRow: mov R10, 0     ;x = 0
        inc R11d                ;y++

        EndMove: inc R13        ;Move array index by 1
        inc RSI
        cmp RSI, R8             ;Compare i counter with amount of pixels assigned to thread
        JL sobelLoop            ;Continue loop if (i < pixelAmountArea)

    ;Restore registers
    POP R13
    POP R12
    ret
ConvoluteWithSobelKernel endp

;RCX    -   unsigned char* outputPtr
;RDX    -   const int startOffset
;R8     -   const int pixelAmountArea

;FUNCTION DESCRIPTION
;This function is exported and can be called by external sources.
;It takes a pointer to output image and replaces the image's
;pixels with normalized gradient intensity values taken
;from the intensityTable.
;If the function is multithreaded, all threads operate on separate areas of the output image.

;INPUT REGISTERS
;RCX    -   contains pointer to first pixel of output image
;RDX    -   index of start pixel (distance from beginning of image),
;           is also called "startOffset"
;R8     -   number of pixels that need to be processed by this thread  

;OUTPUT REGISTERS
;none
;(output data is in the image's modified pixels)

;MODIFIED REGISTERS
;RAX, RDX, R8 - R12, RSI, xmm2-xmm4, ymm4, ymm5
NormalizeSobel proc EXPORT
    PUSH RBX    ;Save BASE REGISTER on the stack before performing
                ;operations with arrays
    PUSH R13    ;Save R13 register before using it as index of current pixel

    mov R12d, dword ptr [imageWidth] ;Save image width in R12d
    
    ;Move index of start pixel (startOffset) to R13
    mov R13, RDX
    ;Move pixel pointer by startOffset, making it point to the start pixel of this thread
    add RCX, R13

    ;Set x in R10 and y in R11
    ;x is the horizontal distance of current pixel from the beginning of the image
    ;y is the vertical distance of current pixel from the beginning of the image
    xor RDX, RDX                        ;clear EDX
    mov RAX, R13                         ;move startOffset into RAX
    div R12                             ;perform (startOffset / imageWidth)
    mov R10, RDX                        ;save (startOffset % imageWidth) from RDX into R10 (x)
    mov R11, RAX                        ;save (startOffset / imageWidth) from RAX into R11 (y)

    ;Clear registers
    xor RAX, RAX
    xor RDX, RDX

    ;Initialize i counter with 0
    mov RSI, 0

    ;CALCULATE HOW MANY TIMES PACKED AVX INSTRUCTIONS NEED TO BE REPEATED
    ;AND HOW MANY PIXELS ARE LEFT OUT AND NEED TO BE PROCESSED WITHOUT VECTORS
    ;VECTOR AMOUNT         -   pixelAmountArea / 4
    ;REMAINING SCALAR      -   pixelAmountArea % 4
    mov RAX, R8
    mov R8, 4
    div R8                 
    ;Each vector instruction operating on xmm registers can process 4 float values
    ;(ymm registers cannot be used because certain necessary instructions weren't
    ;implemented on them in AVX - Intel Core i7-2700K, the processor used for
    ;developing the program, does not have AVX2 or later extensions)

    ;Save AMOUNT OF VECTORS to R8
    mov R8, RAX 
    ;Save AMOUNT OF SCALARS to R9
    mov R9, RDX

    xor RAX, RAX           ;Clear RAX register

    ;Gradient intensity normalization formula:
    ;Gnormalized = G/maxG * 255

    ;To operate on 4 float values at once, save needed variables in ymm4 and ymm5 registers:
    ;Set maxG everywhere in ymm4
    vbroadcastss ymm4, real4 ptr [maxG]
    ;Set 255 everywhere in ymm5
    vbroadcastss ymm5, real4 ptr [_255]

    ;BYTE PATTERN USED FOR CONVERTING GRADIENT VALUES SAVED AS FLOATS
    ;TO 1-BYTE PIXELS:
    ;- - - X, - - - X, - - - X, - - - X

    movss xmm2, real4 ptr [_0]              ;Clear xmm2
    movupd xmm2, xmmword ptr [_pattern]     ;Fill xmm2 with the specified byte pattern

    mov RBX, intensityTableOffset

    vectorLoop: movups xmm3, xmmword ptr [RBX + R13 * 4]   ;Load next 4 gradient intensity values
                                                           ;(of type float) into xmm3
       
        ;-------------------------------------------------------------
        ;FLOAT -> 1-BYTE VALUE CONVERSION
        ;-------------------------------------------------------------

        ;Divide all GRADIENT INTENSITIES by maxG
        divps xmm3, xmm4
        ;Multiply all by 255
        mulps xmm3, xmm5
        ;TRUNCATE FLOATING POINT PART
        cvttps2dq xmm3, xmm3
        ;Get only LOWEST BYTES from 4-BYTE INTEGERS
        pshufb xmm3, xmm2
        ;-------------------------------------------------------------

        ;xmm3 NOW CONTAINS 4 VALUES THAT ARE 4 VALID PIXELS

        ;get 4 NEXT PIXELS to EAX
        movd EAX, xmm3
        ;SAVE 4 NEXT PIXELS TO OUTPUT IMAGE
        mov dword ptr [RCX], EAX
        
        add RCX, 4      ;Move pixel pointer by 4 bytes / 4 pixels
        add R13, 4      ;Move pixel index by 4 floats (length of xmm)
        inc RSI         ;Increase i counter
        cmp RSI, R8     ;If i < vectorAmount, continue loop
        JL vectorLoop

    ;Set i counter to 0 again
    mov RSI, 0

    cmp R9, 0   ;If no scalar values are left, go to end of function
    JE endNormalize

    scalarLoop: movss xmm3, real4 ptr [RBX + R13 * 4]     ;Get 1 gradient intensity
        divss xmm3, xmm4    ;Divide by maxG
        mulss xmm3, xmm5    ;Multiply by 255
        cvtss2si EAX, xmm3  ;Convert to int

        ;SAVE PIXEL TO OUTPUT IMAGE
        mov dword ptr [RCX], EAX

        inc R13         ;Move pixel index by 1 float
        inc RCX         ;Move pixel pointer by 1 byte / 1 pixel
        inc RSI         ;Increase i counter
        cmp RSI, R9     ;If i < scalarAmount, continue loop
        JL scalarLoop

    ;Restore registers
    endNormalize: POP R13
    POP RBX
    RET
NormalizeSobel endp


;-----------------------------------------------------------------------------------
;THIN EDGES
;-----------------------------------------------------------------------------------

;RBX    -   angleTableOffset
;RCX    -   outputPtr
;R14    -   inputPtr
;R12    -   imageWidth
ThinEdge proc EXPORT
    PUSH RBX
    PUSH R13
    PUSH R14

    xor RAX, RAX
    ;get angle
    mov AL, byte ptr [RBX]

    cmp AL, 0
    JE angle0
    cmp AL, 45
    JE angle45
    CMP AL, 90
    JE angle90
    
    ;angle 135
    sub R14, R12   ;-imageWidth - 1
    dec R14
    mov RBX, R14
    mov DL, byte ptr [RBX]
    add R14, R12
    add R14, 2
    add R14, R12   ;+imageWidth +1 1
    mov RBX, R14
    mov R13B, byte ptr [RBX]
    JMP compare

    ;DL     -   pixel 1
    ;R13B   -   pixel 2
    angle0: inc R14     ;+1
            mov RBX, R14
            mov DL, byte ptr [RBX]
            sub R14, 2  ;-1
            mov RBX, R14
            mov R13b, byte ptr [RBX]
            JMP compare

    angle45: sub R14, R12   ;-imageWidth + 1
             inc R14
             mov RBX, R14
             mov DL, byte ptr [RBX]
             add R14, R12
             sub R14, 2
             add R14, R12   ;+imageWidth - 1
             mov RBX, R14
             mov R13B, byte ptr [RBX]
             JMP compare
   
    angle90: add R14, R12   ;+ imageWidth
             mov RBX, R14
             mov DL, byte ptr [RBX]
             sub R14, R12
             sub R14, R12   ;- imageWidth
             mov RBX, R14
             mov R13B, byte ptr [RBX]
             JMP compare


    compare: POP R14
             mov RBX, R14   ;get pixel
             xor RAX, RAX
             mov AL, byte ptr [RBX]

             CMP AL, 255
             JE cm
             inc AL

             cm: CMP AL, DL
             JB set0
             CMP AL, R13b
             JB set0

             POP R13
             POP RBX

             CMP AL, 255
             JE cm2
             dec AL

             cm2: mov byte ptr [RCX], AL
             ret

    set0: mov byte ptr [RCX], 0
          POP R13
          POP RBX
          ret

ThinEdge endp

;RCX    -   outputPtr
;RDX    -   inputPtr
;R8     -   startOffset
;R9     -   pixelAmountArea
ThinEdges proc EXPORT
    PUSH R12
    PUSH R13
    PUSH R14

    ;REGISTERS
    ;RAX    -   
    ;RBX    -   
    ;RCX    -   outputPtr
    ;RDX
    ;RSI    -   i counter
    ;RDI    -   imageHeight - 1
    ;R8     -   imageWeight - 1
    ;R9     -   pixelAmountArea
    ;R10    -   x
    ;R11    -   y
    ;R12    -   imageWidth
    ;R14    -   inputPtr

    ;move inputPtr to R14
    mov R14, RDX

    ;move imageHeight to RDI
    mov EDI, dword ptr [imageHeight]

    ;add startOffset to outputPtr and inputPtr
    add RCX, R8
    add R14, R8

    ;load imageWidth
    mov R12d, dword ptr [imageWidth]

    ;set x and y
    xor RDX, RDX                        ;clear EDX
    mov RAX, R8                         ;move startOffset into RAX
    div R12                             ;perform (startOffset / imageWidth) as RAX/R11
    mov R10, RDX                        ;save (startOffset % imageWidth) from RDX into R10 (x)
    mov R11, RAX                        ;save (startOffset / imageWidth) from RAX into R11 (y)

    ;set beginning of angle table
    mov RBX, angleTableOffset
    add RBX, R8

    ;load i counter with 0
    mov RSI, 0

    ;save (imageWidth - 1) in R8
    mov R8, R12
    dec R8

    ;save (imageHeight - 1) in RDI
    DEC RDI

    edgeLoop: cmp R10, 0       ;if (x == 0)
        JE imageBorder          ;
        cmp R11, 0              ;if (y == 0)
        JE imageBorder          ;
        cmp R10, R8             ;if (x == imageWidth - 1)
        JE imageBorder          ;
        cmp R11, RDI            ;if (y == imageHeight - 1)
        JE imageBorder
        JMP imageInside

        ;pixel is on the border, just set OUTPUT PIXEL to 0
        imageBorder: mov byte ptr [RCX], 0
               JMP sobelEnd

        ;calculate gradient intensity and direction
        imageInside: call ThinEdge    ;(RCX (outputPtr), R14 (inputPtr))
        
        sobelEnd: inc RCX       ;outputPtr++  
        inc R14                 ;inputPtr++

        CMP R10d, R8d           ;
        JNL NextRow             ;if [ x < imageWidth - 1], stay in row        
        inc R10d                ;x++
        JMP EndMove             ;
        NextRow: mov R10, 0     ;x = 0
        inc R11d                ;y++

        EndMove: inc RBX        ;move array pointer by 1
        inc RSI                 ;increase i counter
        cmp RSI, R9             ;compare i with pixelAmountArea
        JL edgeLoop             ;loop if (i < pixelAmountArea)

    POP R14
    POP R13
    POP R12
    ret
ThinEdges endp

;-----------------------------------------------------------------------------------
;GAUSS, THIN EDGES - END
;-----------------------------------------------------------------------------------

CleanIntensityAngleTables proc EXPORT
    ret
CleanIntensityAngleTables endp

;-----------------------------------------------------------------------------------
;CATEGORIZE EDGES (HYSTERESIS)
;-----------------------------------------------------------------------------------

;RCX    -   outputPtr
;RDX    -   inputPtr
;R8     -   startOffset
;R9     -   area
;STACK  -   threshold H
;STACK  -   threshold L
CategorizeEdges proc EXPORT   

    ret
CategorizeEdges endp

;-----------------------------------------------------------------------------------
;HYSTERESIS TRACKING
;-----------------------------------------------------------------------------------

ConnectEdges proc EXPORT
    ret
ConnectEdges endp


;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------



;unsigned char* newImage               RCX
;unsigned char* oldImage               RDX
;short value    addedBrightness        R8W  WORD
;int            imageSizeInBytes       R9D  DOUBLEWORD (COUNTER)
;OFFSET                                R10
ChangeBrightness proc EXPORT
    mov r10, 0                    ;set the offset pointer to 0
    cmp r8w, 0                    ;check if addedBrightness + or -
    jl SubstractBrightness        ;if less than 0
    mov r11w, 0ffffh              ;set overflow value to 255

MainLoopAdd:
    mov al, byte ptr [rdx + r10]    ;read next byte from old image
    add al, r8b                     ;add brightness (BYTE)
    cmovc ax, r11w                  ;set 255 on overflow (MOVE IF CARRY)
    mov byte ptr [rcx + r10], al    ;store result in new image
    inc r10                         ;move to next pixel (next byte)
    dec r9d                         ;decrement counter
    jnz MainLoopAdd
    ret

SubstractBrightness:
    mov r11w, 0                  ;move 0 into overflow
    neg r8w                      ;neg r8w for substraction

MainLoopSubstract:
    mov al, byte ptr [rdx + r10] ;read next byte from old image
    sub al, r8b                  ;reduce brightness (BYTE)
    cmovc ax, r11w               ;set 255 on overflow (MOVE IF CARRY)
    mov byte ptr [rcx + r10], al ;store result in new image
    inc r10                      ;move to next pixel (next byte)
    dec r9d                      ;decrement counter
    jnz MainLoopSubstract
    ret

ChangeBrightness endp

end