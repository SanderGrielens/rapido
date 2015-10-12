/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this header file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------
 
  File:        VmbTransform.h

  Description: Definition of image transform functions for the Vimba APIs.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#ifndef VMB_TRANSFORM_H_
#define VMB_TRANSFORM_H_
#ifndef VMB_TRANSFORM
#define VMB_TRANSFORM
#endif
#pragma once

#include "VmbTransformTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VMB_WIN32DLL_API
#ifndef VMB_NO_EXPORT
#   ifdef VMB_EXPORTS
#       if defined(__clang__) || defined(__GNUC__)
#           define VMB_WIN32DLL_API __attribute__((visibility("default")))
#       else
#           ifndef _WIN64
#               define VMB_WIN32DLL_API __declspec(dllexport) __stdcall
#           else
#               define VMB_WIN32DLL_API __stdcall
#           endif
#       endif
#   else
#       if defined(__clang__) || defined(__GNUC__)
#           define VMB_WIN32DLL_API
#       else
#           define VMB_WIN32DLL_API __declspec(dllimport) __stdcall
#       endif
#   endif
#else
#       define VMB_WIN32DLL_API 
#endif
#endif
/**  Get library version.
* \param[in,out] Version  pointer, returns the library version (Major,Minor,SubMinor,Build)
* \return will always return vmbErrorSuccess
*/
VmbError_t VMB_WIN32DLL_API VmbGetVersion ( VmbUint32Ptr_t   Version );

/** Get information about processor supported features.
* \param[in,out] TechnoInfo returns the supported SIMD technologies
* \note This should be called before using any SIMD (MMX,SSE) optimmized functions
*/
VmbError_t VMB_WIN32DLL_API VmbGetTechnoInfo( VmbTechInfo_t *TechnoInfo);

/**Get information about received result code.
* \param[in]     ErrorCode      code to get the info for
* \param[in,out] InfoString     will contain as much info as MaxInfoLength allows minus terminating zero
* \param[in]     MaxInfoLength  length of the info string buffer
*/
VmbError_t VMB_WIN32DLL_API VmbGetErrorInfo(   VmbError_t ErrorCode, VmbANSIChar_t *InfoString, VmbUint32_t MaxInfoLength );
                                                
/**Get information about the build parameters of the API.
* \param[in]     info_type      type of info to inquire
* \param[in,out] InfoString     will contain API information up to MaxInfoLength characters minus terminating zero
* \param[in]     MaxInfoLength  max length of the info string buffer
*/
VmbError_t VMB_WIN32DLL_API VmbGetApiInfoString(VmbAPIInfo_t info_type, VmbANSIChar_t *InfoString, VmbUint32_t MaxInfoLength);

/**Set a debayer mode to a transform info structure.
* \param[in]     DebayerMode    mode to set
* \param[in,out] TransformInfo  pointer to a TransformInfo to set the mode
*/
VmbError_t VMB_WIN32DLL_API VmbSetDebayerMode( VmbDebayerMode_t  DebayerMode, VmbTransformInfo  *TransformInfo);

/**Set color correction matrix 3x3 to transform info structure.
* \param[in]     Matrix         row order floating point matrix to set as color correction
* \param[in,out] TransformInfo  transform info to set the matrix too
*/
VmbError_t VMB_WIN32DLL_API VmbSetColorCorrectionMatrix3x3( const VmbFloat_t *Matrix, VmbTransformInfo *TransformInfo);

/**Set gamma correction to transform info structure.
* \param[in]     Gamma          float gamma correction to set
* \param[in,out] TransformInfo  transform info to set gamma correction to
*/
VmbError_t VMB_WIN32DLL_API VmbSetGammaCorrection( VmbFloat_t Gamma, VmbTransformInfo *TransformInfo);

/**Set image info from VmbPixelFormat.
* \param[in]     PixelFormat    pixel format used in buffer assigned to the Data member
* \param[in]     Width          width of the image in pixel
* \param[in]     Height         height of the image in pixel
* \param[in,out] Image          image to set the ImageInfo member
* \note function does not read or write to VmbImage::Data member
*/
VmbError_t VMB_WIN32DLL_API VmbSetImageInfoFromPixelFormat( VmbPixelFormat_t PixelFormat, VmbUint32_t Width, VmbUint32_t Height, VmbImage *Image);

/**Set image info from pixel format string.
* \param[in]     ImageFormat        pixel format string either as VmbPixelformat or format string
* \param[in]     StringLength       length of the image format buffer
* \param[in]     Width              width of the image in pixel
* \param[in]     Height             height of the image in pixel
* \param[in,out] Image              image to set the ImageInfo member 
* \note function does not read or write to VmbImage::Data member
*/
VmbError_t VMB_WIN32DLL_API VmbSetImageInfoFromString( const VmbANSIChar_t *ImageFormat, VmbUint32_t StringLength, VmbUint32_t Width, VmbUint32_t Height, VmbImage *Image);

/**transform source image to destination using the ImageInfo and parameter.
* \param[in]     Source             source image of transform
* \param[in,out] Destination        destination image of transform
* \param[in]     Parameter          optional parameter for transform
* \param[in]     ParameterCount     number of optional parameters
* \note create the source and destination image info structure with VmbSetImageInfoFromPixelFormat or VmbSetimageInfoFromString and keep those structures as template.
*       for calls to transform simply attache the image to the Data member.
*       the optional parameters, when set, are constraints on the transform, if there is no transform using the parameter the function returns VmbErrorBadParameter
*/
VmbError_t VMB_WIN32DLL_API VmbImageTransform( const VmbImage *Source, VmbImage *Destination, const VmbTransformInfo  *Parameter, VmbUint32_t ParameterCount);

#ifdef __cplusplus
 }
#endif // #ifdef __cplusplus
#endif //#ifndef UNI_TRANSFORM_H_
