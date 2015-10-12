/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ApiController.h

  Description: Implementation file for the ApiController helper class that
               demonstrates how to implement a synchronous single image
               acquisition with VimbaCPP.

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

#ifndef AVT_VMBAPI_EXAMPLES_APICONTROLLER
#define AVT_VMBAPI_EXAMPLES_APICONTROLLER

#include <string>

#include "VimbaCPP/Include/VimbaCPP.h"

#include "CameraObserver.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

class ApiController
{
  public:
    ApiController();
    ~ApiController();

    VmbErrorType        StartUp();
    void                ShutDown();

    VmbErrorType        AcquireSingleImage( const std::string &rStrCameraID, FramePtr &rpFrame );

    int                 GetWidth()          const;
    int                 GetHeight()         const;
    VmbPixelFormatType  GetPixelFormat()    const;
    CameraPtrVector     GetCameraList()     const;
   
    QObject*            GetCameraObserver();

    std::string         ErrorCodeToMessage( VmbErrorType eErr ) const;
    std::string         GetVersion() const;

  private:
    // A reference to our Vimba singleton
    VimbaSystem &       m_system;
    // The currently streaming camera
    CameraPtr           m_pCamera;
    // Our camera observer
    CameraObserver*     m_pCameraObserver;
    // The current pixel format
    VmbInt64_t          m_nPixelFormat;
    // The current width
    VmbInt64_t          m_nWidth;
    // The current height
    VmbInt64_t          m_nHeight;
};

}}} // namespace AVT::VmbAPI::Examples

#endif
