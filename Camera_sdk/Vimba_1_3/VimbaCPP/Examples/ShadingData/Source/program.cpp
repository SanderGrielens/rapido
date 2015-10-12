/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        program.cpp

  Description: Main entry point of ShadingData example of VimbaCPP.

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

#include <iostream>
#include <cstring>
#include <cstdlib>

#include <ShadingData.h>
#include <Exception.h>

#include "Common/StreamSystemInfo.h"

using namespace std;

enum Mode
{
    ModeUnknown     = 0,
    ModeSave        = 1,
    ModeLoad        = 2,
    ModeClear       = 3,
    ModeBuild       = 4,
    ModeEnable      = 5,
    ModeIsEnabled   = 6,
    ModeShowData    = 7,
    ModeIsDataShown = 8
};

bool StartsWith(const char *pString, const char *pStart)
{
    if(NULL == pString)
    {
        return false;
    }
    if(NULL == pStart)
    {
        return false;
    }

    if(std::strlen(pString) < std::strlen(pStart))
    {
        return false;
    }

    if(std::memcmp(pString, pStart, std::strlen(pStart)) != 0)
    {
        return false;
    }

    return true;
}

int main( int argc, char* argv[] )
{
    cout << "//////////////////////////////////////////\n";
    cout << "/// AVT Vimba API Shading Data Example ///\n";
    cout << "//////////////////////////////////////////\n\n";

    VmbErrorType err = VmbErrorSuccess;

    string      cameraID;
    string      parameter;
    bool        bValue = false;
    VmbInt64_t  nImageCount;
    Mode        eMode  = ModeUnknown; 
    bool        printHelp = false;

    //////////////////////
    //Parse command line//
    //////////////////////

    for(int i = 1; i < argc; i++)
    {
        char *pParameter = argv[i];
        if(std::strlen(pParameter) < 0)
        {
            err = VmbErrorBadParameter;
            break;
        }

        if(pParameter[0] == '/')
        {
            if(std::strcmp(pParameter, "/s") == 0)
            {
                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeSave;
            }
            else if(std::strcmp(pParameter, "/l") == 0)
            {
                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeLoad;
            }
            else if(std::strcmp(pParameter, "/c") == 0)
            {
                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeClear;
            }
            else if(StartsWith(pParameter, "/b:"))
            {
                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if(parameter.empty() == false)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                parameter = pParameter + 3;
                if(parameter.size() <= 0)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                nImageCount = std::atoi( parameter.c_str() );

                eMode = ModeBuild;
            }
            else if(StartsWith(pParameter, "/e:"))
            {
                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if(parameter.empty() == false)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                parameter = pParameter + 3;
                if(parameter.size() <= 0)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if ( std::strcmp(parameter.c_str(), "on") == 0 )
                {
                    bValue = true;
                }
                else if ( std::strcmp(parameter.c_str(), "off") == 0 )                    
                {
                    bValue = false;
                }
                else
                {
                    cout << "Could not enable shading data. Wrong parameter!\n";
                    err = VmbErrorBadParameter;
                    break;
                }    
                eMode = ModeEnable;
            }
            else if(std::strcmp(pParameter, "/e") == 0)
            {
                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeIsEnabled;
            }
            else if(StartsWith(pParameter, "/p:"))
            {
                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if(parameter.empty() == false)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                parameter = pParameter + 3;
                if(parameter.size() <= 0)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if ( std::strcmp(parameter.c_str(), "on") == 0 )
                {
                    bValue = true;
                }
                else if ( std::strcmp(parameter.c_str(), "off") == 0 )                    
                {
                    bValue = false;
                }
                else
                {
                    cout << "Could not show shading data. Wrong parameter!\n";
                    err = VmbErrorBadParameter;
                    break;
                }    
                eMode = ModeShowData;
            }
            else if(std::strcmp(pParameter, "/p") == 0)
            {
                if(ModeUnknown != eMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeIsDataShown;
            }
            else if(std::strcmp(pParameter, "/h") == 0)
            {
                if(     (cameraID.empty() == false)
                    ||  (ModeUnknown != eMode)
                    ||  (true == printHelp))
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                printHelp = true;
            }
            else
            {
                err = VmbErrorBadParameter;
                break;
            }
        }
        else
        {
            if(cameraID.empty() == false)
            {
                err = VmbErrorBadParameter;
                break;
            }

            cameraID = pParameter;
        }
    }

    //Write out an error if we could not parse the command line
    if(VmbErrorBadParameter == err)
    {
        cout << "Invalid parameters!" << endl << endl;
        printHelp = true;
    }

    //Print out help and end program
    if(true == printHelp)
    {
        cout << "Usage: ShadingData [CameraID] [/h] [/{s|l|c|b:ImageCount|e(:Enable)|p(:ShowData)}]\n";
        cout << "Parameters:   CameraID       ID of the camera to use\n";
        cout << "                             (using first camera if not specified)\n";
        cout << "              /h             Print out help\n";
        cout << "              /s             Save shading data to flash\n";
        cout << "              /l             Load shading data from flash\n";
        cout << "                             (default if not specified)\n";
        cout << "              /c             Clear shading data from flash\n";
        cout << "              /b:ImageCount  Build shading data with image count\n";
        cout << "              /e             Is shading data enabled\n";
        cout << "              /e:Enable      Enable shading data [on/off]\n";
        cout << "              /p             Is shading data shown\n";
        cout << "              /p:ShowData    Show shading data [on/off]\n";

        return err;
    }

    bool bVimbaStarted = false;
    AVT::VmbAPI::CameraPtr pCamera;
    AVT::VmbAPI::VimbaSystem * pVimbaSystem = NULL;

    try
    {
        // Get a pointer to the VimbaSystem singleton
        pVimbaSystem = &AVT::VmbAPI::VimbaSystem::GetInstance();
        
        // Print out version of Vimba
        std::cout<<"Vimba Version V"<<*pVimbaSystem<<"\n";

        //Startup API
        if(VmbErrorSuccess == err)
        {
            err = pVimbaSystem->Startup();
            if(VmbErrorSuccess != err)
            {
                throw AVT::VmbAPI::Examples::Exception("Could not start system", err);
            }
            bVimbaStarted = true;
        }

        //Open camera
        if(VmbErrorSuccess == err)
        {
            if(cameraID.empty())
            {
                //Open first available camera

                //Fetch all cameras known to Vimba
                AVT::VmbAPI::CameraPtrVector cameras;
                err = pVimbaSystem->GetCameras(cameras);
                if(VmbErrorSuccess == err)
                {
                    if(cameras.size() > 0)
                    {
                        for (   AVT::VmbAPI::CameraPtrVector::const_iterator iter = cameras.begin();
                                cameras.end() != iter;
                                ++iter )
                        {
                            //Check if we can open the camera in full mode
                            VmbAccessModeType eAccessMode = VmbAccessModeNone;
                            err = (*iter)->GetPermittedAccess(eAccessMode);
                            if(VmbErrorSuccess == err)
                            {
                                if( (VmbAccessModeFull == (VmbAccessModeFull & eAccessMode)) ||
                                    ((cameras.end() - 1) == iter) )
                                {
                                    //Now get the camera ID
                                    err = (*iter)->GetID(cameraID);
                                    if(VmbErrorSuccess == err)
                                    {
                                        //Try to open the camera
                                        err = (*iter)->Open(VmbAccessModeFull);
                                        if(VmbErrorSuccess == err)
                                        {
                                            pCamera = *iter;
                                            break;
                                        }
                                    }
                                }
                            }
                        }

                        if(VmbErrorSuccess != err)
                        {
                            err = VmbErrorNotFound;
                            throw AVT::VmbAPI::Examples::Exception("Could not open any camera.", err);
                        }
                    }
                    else
                    {
                        err = VmbErrorNotFound;
                        throw AVT::VmbAPI::Examples::Exception("Could not open any camera.", err);
                    }
                }
                else
                {
                    throw AVT::VmbAPI::Examples::Exception("Could not list cameras.", err);
                }
            }
            else
            {
                //Open specific camera
                err = pVimbaSystem->OpenCameraByID(cameraID.c_str(), VmbAccessModeFull, pCamera);
                if(VmbErrorSuccess != err)
                {
                    throw AVT::VmbAPI::Examples::Exception("Could not open camera", err);
                }
            }
        }

        if(VmbErrorSuccess == err)
        {
            cout << "Camera ID: " << cameraID <<"\n\n";

            AVT::VmbAPI::Examples::ShadingDataControl control(pCamera);

            switch(eMode)
            {
            default:
            case ModeLoad:
                {                
                    //Load shading data from flash
                    err = control.LoadFromFlash();
                    if(VmbErrorSuccess != err)
                    {
                        throw AVT::VmbAPI::Examples::Exception("Could not load shading data from flash.", err);
                    }

                    cout << "Shading data successfully loaded from flash.\n";
                }
                break;

            case ModeSave:
                {
                    //Save shading data to flash
                    err = control.SaveToFlash();
                    if(VmbErrorSuccess != err)
                    {
                        throw AVT::VmbAPI::Examples::Exception("Could not save shading data to flash", err);
                    }

                    cout << "Shading data successfully saved to flash.\n";
                }
                break;

            case ModeClear:
                {                
                    //Clear shading data from flash
                    err = control.ClearFlash();
                    if(VmbErrorSuccess != err)
                    {
                        throw AVT::VmbAPI::Examples::Exception("Could not clear shading data from flash.", err);
                    }

                    cout << "Shading data successfully cleared from flash.\n";
                }
                break;

            case ModeBuild:
                {                

                    //Build shading data with image count
                    err = control.BuildImages( nImageCount );
                    if(VmbErrorSuccess != err)
                    {
                        throw AVT::VmbAPI::Examples::Exception("Could not build shading data.", err);
                    }
                        
                    cout << "Shading data successfully build.\n";
                }
                break;

            case ModeEnable:
                {
                    //Enable shading data
                    err = control.Enable( bValue );
                    if(VmbErrorSuccess != err)
                    {
                        throw AVT::VmbAPI::Examples::Exception("Could not set enable state of shading data.", err);
                    }

                    cout << "Shading data enable state was set successfully.\n";
                }
                break;

            case ModeIsEnabled:
                {
                    //Is shading data enabled 
                    bool bEnable;
                    err = control.IsEnabled( bEnable );
                    if ( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception("Could not get enable state of shading data.", err);
                    }

                    cout << "Shading data enable state requested successfully. Enable = " << bEnable <<"\n";
                }
                break;

            case ModeShowData:
                {
                    //Show data
                    err = control.ShowData( bValue );
                    if(VmbErrorSuccess != err)
                    {
                        throw AVT::VmbAPI::Examples::Exception("Could not show shading data.", err);
                    }

                    cout << "Shading data show state was set successfully.\n";
                }
                break;

            case ModeIsDataShown:
                {
                    //Is data shown
                    bool bShowData;
                    err = control.IsDataShown( bShowData );
                    if ( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception("Could not get data show status for shading data", err);
                    }

                    cout << "Shading data shown state requested successfully. DataShown = " << bShowData <<"\n";
                }
                break;
            }    
        }
    }

    catch( AVT::VmbAPI::Examples::Exception& ex )
    {
        cout << ex.GetMessageStr() << " VimbaException: " << ex.GetError() << " = " << ex.ErrorCodeToMessage(ex.GetError()) <<"\n";
    }

    //Close camera
    if( NULL != pCamera )
    {
        pCamera->Close();
    }
    
    //Shutdown API
    if( true == bVimbaStarted )
    {
        pVimbaSystem->Shutdown();
    }

    return err;
}
