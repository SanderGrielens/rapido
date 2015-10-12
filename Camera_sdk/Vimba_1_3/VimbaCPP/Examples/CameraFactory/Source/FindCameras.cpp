/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FindCameras.cpp

  Description: Find and print a custom string for each known customised camera.

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
#include <vector>

#include "FindCameras.h"
#include "CameraFactory.h"

#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"
namespace AVT {
namespace VmbAPI {
namespace Examples {

void FindCameras::Print()
{
    VimbaSystem&        sys         = VimbaSystem::GetInstance();   // Get a reference to the VimbaSystem singleton
    VmbErrorType        err         = sys.Startup();                // Initialize the Vimba API
    CameraPtrVector     cameras;                                    // A vector of std::shared_ptr<AVT::VmbAPI::Camera> objects

    std::string         strName;                                    // The name of the cam
    VmbInterfaceType    interfaceType;                              // The interface type of the cam
    std::string         strInfo;                                    // The custom information
    
    std::stringstream   strError;

    

    if( VmbErrorSuccess == err )
    {
        std::cout<<"Vimba Version V"<<sys<<"\n";

        // Set user factory as default camera object creator.
        ICameraFactoryPtr factPtr = UserCameraFactory_t( new UserCameraFactory() );
        err = sys.RegisterCameraFactory( factPtr );
        if( VmbErrorSuccess == err )
        {
            err = sys.GetCameras( cameras );        // Fetch all cameras known to Vimba
            if( VmbErrorSuccess == err )
            {
                std::cout << "Cameras found: " << cameras.size() <<"\n\n";

                // Query the name and interface of all known cameras and print them out.
                // We don't have to open the cameras for that.
                for( CameraPtrVector::const_iterator iter = cameras.begin();
                        cameras.end() != iter;
                        ++iter )
                {
                    err = (*iter)->GetName( strName );
                    if( VmbErrorSuccess != err )
                    {
                        strError << "[Could not get camera name. Error code: " << err <<"("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "]";
                        strName.assign( strError.str() );
                    }

                    err = (*iter)->GetInterfaceType( interfaceType );
                    if( VmbErrorSuccess != err )
                    {
                        strError << "[Could not get camera interface. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "]";
                        strInfo = "";
                    }
                    else
                    {
                        strInfo = "none";
                        switch( interfaceType )
                        {
                        case VmbInterfaceFirewire:
                            {
                                FirewireCamera_t fcam = SP_DYN_CAST( *iter, FirewireCamera );
                                if(fcam != NULL)
                                    fcam->addonFireWire(strInfo);
                                break;
                            }
                        case VmbInterfaceEthernet: 
                            {
                                GigECamera_t gcam = SP_DYN_CAST( *iter, GigECamera );
                                if(gcam != NULL)
                                    gcam->addonGigE(strInfo);
                                break;
                            }
                        case VmbInterfaceUsb: 
                            {
                                USBCamera_t ucam = SP_DYN_CAST( *iter, USBCamera );
                                if(ucam != NULL)
                                    ucam->addonUSB(strInfo);
                                break;
                            }
                        default:
                            {
                                break;
                            }
                        }
                    }
                    
                    std::cout   <<"/// Camera Name: " << strName 
                                <<"\n/// Custom Info: " << strInfo 
                                <<"\n\n";
                }
            }
            else
            {
                std::cout << "Could not list cameras. Error code: " << err <<"("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
            }

            sys.Shutdown();                             // Close Vimba
        }
        else
        {
             strError << "[Could not set user camera factory. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "]";
        }
    }
    else
    {
        std::cout << "Could not start system. Error code: " << err <<"("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
    }
}



}}} // namespace AVT::Vimba::Examples

