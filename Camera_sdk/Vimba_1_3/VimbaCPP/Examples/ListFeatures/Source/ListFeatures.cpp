/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ListFeatures.cpp

  Description: The ListFeatures example will list all available features of a
               camera that are found by VimbaCPP.

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
#include <algorithm>
#include <ListFeatures.h>

#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {


// Prints out the value of a given feature
void PrintFeatureValue( const FeaturePtr &feature )
{
    VmbFeatureDataType  eType;
    VmbErrorType        err     = feature->GetDataType( eType );
    if( VmbErrorSuccess != err )
    {
        std::cout << "[Could not get feature Data Type. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]\n";
    }
    else
    {
        std::cout << "/// Value          : ";
        switch( eType )
        {
            case VmbFeatureDataBool:
                {
                    VmbBool_t bValue;
                    err = feature->GetValue( bValue );
                    if ( VmbErrorSuccess == err )
                    {
                        std::cout << bValue << "\n";
                    }
                }
                break;
            case VmbFeatureDataEnum:
                {
                    std::string strValue;
                    err = feature->GetValue( strValue );
                    if ( VmbErrorSuccess == err )
                    {
                        std::cout << strValue << "\n";
                    }
                }
                break;
            case VmbFeatureDataFloat:
                {
                    double fValue;
                    err = feature->GetValue( fValue );
                    if( VmbErrorSuccess == err)
                    {
                        std::cout << fValue << "\n";
                    }
                }
                break;
            case VmbFeatureDataInt:
                {
                    VmbInt64_t nValue;
                    err = feature->GetValue( nValue );
                    if( VmbErrorSuccess == err)
                    {
                        std::cout << nValue << "\n";
                    }
                }
                break;
            case VmbFeatureDataString:
                {
                    std::string strValue;
                    err = feature->GetValue( strValue );
                    if( VmbErrorSuccess == err)
                    {
                        std::cout << strValue << "\n";
                    }
                }
                break;
            case VmbFeatureDataCommand:
            default:
                std::cout << "[None]" << "\n";
                break;
        }
        if( VmbErrorSuccess == err )
        {
            std::cout << "\n";
        }
        else
        {
            std::cout << "Could not get feature value. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n\n";
        }
    }

}

// Prints all details of a feature
void PrintFeatures( const FeaturePtr &feature )
{
    std::string strName;                                                    // The name of the feature
    std::string strDisplayName;                                             // The display name of the feature
    std::string strToolTip;                                                 // A short description of the feature
    std::string strDescription;                                             // A long description of the feature
    std::string strCategory;                                                // A category to group features
    std::string strSFNCNamespace;                                           // The Standard Feature Naming Convention namespace
    std::string strUnit;                                                    // The measurement unit of the value

    std::ostringstream ErrorStream;

    VmbErrorType err = feature->GetName( strName );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Name. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        strName = ErrorStream.str();
    }

    err = feature->GetDisplayName( strDisplayName );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Display Name. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        strDisplayName = ErrorStream.str();
    }

    err = feature->GetToolTip( strToolTip );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Tooltip. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        strToolTip = ErrorStream.str();
    }

    err = feature->GetDescription( strDescription );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Description. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        strDescription = ErrorStream.str();
    }

    err = feature->GetCategory( strCategory );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Category. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        strCategory = ErrorStream.str();
    }

    err = feature->GetSFNCNamespace( strSFNCNamespace );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature SNFC Namespace. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        strSFNCNamespace = ErrorStream.str();
    }

    err = feature->GetUnit( strUnit );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get feature Unit. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "]";
        strUnit = ErrorStream.str();
    }

    std::cout << "/// Feature Name   : " << strName             << "\n";
    std::cout << "/// Display Name   : " << strDisplayName      << "\n";
    std::cout << "/// Tooltip        : " << strToolTip          << "\n";
    std::cout << "/// Description    : " << strDescription      << "\n";
    std::cout << "/// SNFC Namespace : " << strSFNCNamespace    << "\n";
    PrintFeatureValue( feature );
}

// Prints all features and their values of a camera
void ListFeatures::Print( std::string CameraID )
{
    VimbaSystem&        sys         = VimbaSystem::GetInstance();           // Get a reference to the VimbaSystem singleton
    std::cout << "Vimba Version V" << sys << "\n";                          // Print out version of Vimba
    VmbErrorType        err         = sys.Startup();                        // Initialize the Vimba API
    FeaturePtrVector    features;                                           // A vector of std::shared_ptr<AVT::VmbAPI::Feature> objects
    CameraPtr           pCamera     = CameraPtr();                          // Our camera

    std::stringstream strError;

    if( VmbErrorSuccess == err )
    {
        if( CameraID.empty() )                                                // If no ID was provided use the first camera
        {
            CameraPtrVector cameras;
            err = sys.GetCameras( cameras );
            if( VmbErrorSuccess == err
                    &&  !cameras.empty() )
            {
                pCamera = cameras[0];                                       // Get the camera
                err = pCamera->Open( VmbAccessModeFull );                   // Open the camera
                if( VmbErrorSuccess == err )
                {
                    err = pCamera->GetID( CameraID );
                }
            }
        }
        else
        {
            err = sys.OpenCameraByID( CameraID.c_str(), VmbAccessModeFull, pCamera ); // Get and open the camera
        }
        if( NULL != pCamera )
        {
            std::cout << "Printing all features of camera with ID: " << CameraID << "\n";
            err = pCamera->GetFeatures( features );                         // Fetch all features of our cam
            if( VmbErrorSuccess == err )
            {
                // Query all static details as well as the value of all fetched features and print them out.
                std::for_each( features.begin(), features.end(), PrintFeatures );
            }
            else
            {
                std::cout << "Could not get features. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
            }

            pCamera->Close();
        }
        else
        {
            std::cout << "Could not open camera or no camera available. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
        }
        sys.Shutdown();
    }
    else
    {
        std::cout << "Could not start system. Error code: " << err << " (" << AVT::VmbAPI::Examples::ErrorCodeToMessage( err ) << ")" << "\n";
    }
}

}}} // namespace AVT::Vimba::Examples
