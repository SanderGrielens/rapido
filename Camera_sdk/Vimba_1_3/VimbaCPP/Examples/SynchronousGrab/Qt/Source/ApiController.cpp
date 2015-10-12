/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ApiController.cpp

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


#include <sstream>
#include <iostream>

#include "ApiController.h"

#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

enum { NUM_FRAMES = 3, };

ApiController::ApiController()
    // Get a reference to the Vimba singleton
    : m_system ( VimbaSystem::GetInstance() )
{
}

ApiController::~ApiController()
{
}

// Translates Vimba error codes to readable error messages
std::string ApiController::ErrorCodeToMessage( VmbErrorType eErr ) const
{
    return AVT::VmbAPI::Examples::ErrorCodeToMessage( eErr );
}

VmbErrorType ApiController::StartUp()
{
    VmbErrorType res;

    // Start Vimba
    res = m_system.Startup();
    if ( VmbErrorSuccess == res )
    {
        // This will be wrapped in a shared_ptr so we don't delete it
        m_pCameraObserver = new CameraObserver();
        // Register an observer whose callback routine gets triggered whenever a camera is plugged in or out
        res = m_system.RegisterCameraListObserver( ICameraListObserverPtr( m_pCameraObserver ));
    }
    
    return res;
}

void ApiController::ShutDown()
{
    // Release Vimba
    m_system.Shutdown();
}

VmbErrorType ApiController::AcquireSingleImage( const std::string &rStrCameraID, FramePtr &rpFrame )
{
    // Open the desired camera by its ID
    VmbErrorType res = m_system.OpenCameraByID( rStrCameraID.c_str(), VmbAccessModeFull, m_pCamera );
    if ( VmbErrorSuccess == res )
    {
        // Set the GeV packet size to the highest possible value
        // (In this example we do not test whether this cam actually is a GigE cam)
        FeaturePtr pCommandFeature;
        if ( VmbErrorSuccess == m_pCamera->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ))
        {
            if ( VmbErrorSuccess == pCommandFeature->RunCommand() )
            {
                bool bIsCommandDone = false;
                do
                {
                    if ( VmbErrorSuccess != pCommandFeature->IsCommandDone( bIsCommandDone ))
                    {
                        break;
                    }
                } while ( false == bIsCommandDone );
            }
        }
        FeaturePtr pFormatFeature;
        // Save the current width
        res = m_pCamera->GetFeatureByName( "Width", pFormatFeature );
        if ( VmbErrorSuccess == res )
        {
            res = pFormatFeature->GetValue( m_nWidth );
            if ( VmbErrorSuccess == res )
            {
                // Save the current height
                res = m_pCamera->GetFeatureByName( "Height", pFormatFeature );
                if ( VmbErrorSuccess == res )
                {
                    pFormatFeature->GetValue( this->m_nHeight );
                    if ( VmbErrorSuccess == res )
                    {
                        // Set pixel format. For the sake of simplicity we only support Mono and RGB in this example.
                        res = m_pCamera->GetFeatureByName( "PixelFormat", pFormatFeature );
                        if ( VmbErrorSuccess == res )
                        {
                            // Try to set RGB
                            res = pFormatFeature->SetValue( VmbPixelFormatRgb8 );
                            if ( VmbErrorSuccess != res )
                            {
                                // Fall back to Mono
                                res = pFormatFeature->SetValue( VmbPixelFormatMono8 );
                            }

                            // Read back the currently selected pixel format
                            pFormatFeature->GetValue( m_nPixelFormat );

                            if ( VmbErrorSuccess == res )
                            {
                                // Acquire
                                res = m_pCamera->AcquireSingleImage( rpFrame, 2000 );
                            }
                        }
                    }
                }
            }
        }
        m_pCamera->Close();
    }

    return res;
}

CameraPtrVector ApiController::GetCameraList() const
{
    CameraPtrVector cameras;
    // Get all known cameras
    if ( VmbErrorSuccess == m_system.GetCameras( cameras ))
    {
        // And return them
        return cameras;
    }
    return CameraPtrVector();
}

int ApiController::GetWidth() const
{
    return static_cast<int>( m_nWidth );
}

int ApiController::GetHeight() const
{
    return static_cast<int>( m_nHeight );
}

VmbPixelFormatType ApiController::GetPixelFormat() const
{
    return static_cast<VmbPixelFormatType>( m_nPixelFormat );
}

// Returns the camera observer as QObjects to connect their signals to the view's sots
QObject* ApiController::GetCameraObserver()
{
    return m_pCameraObserver;
}

std::string ApiController::GetVersion() const
{
    std::ostringstream os;
    os<<m_system;
    return os.str();
}
}}} // namespace AVT::VmbAPI::Examples
