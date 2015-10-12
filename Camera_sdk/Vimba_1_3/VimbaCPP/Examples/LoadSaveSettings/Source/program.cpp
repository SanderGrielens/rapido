/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        program.cpp

  Description: Main entry point of LoadSaveSettings example of VimbaCPP.

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

#include "LoadSaveSettings.h"

#include "Common/StreamSystemInfo.h"

enum SettingsMode
{
    SettingsModeUnknown = 0,
    SettingsModeSave    = 1,
    SettingsModeLoad    = 2
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
    std::string     cameraID;
    std::string     fileName;
    SettingsMode    settingsMode        = SettingsModeUnknown; 
    bool            printHelp           = false;
    bool            ignoreStreamable    = false;

    std::cout << "/////////////////////////////////////////////\n";
    std::cout << "/// AVT Vimba API Manage Settings Example ///\n";
    std::cout << "/////////////////////////////////////////////\n\n";

    VmbErrorType err = VmbErrorSuccess;

    //////////////////////
    //Parse command line//
    //////////////////////

    for(int i = 1; i < argc; i++)
    {
        char *pParameter = argv[i];
        if( std::strlen(pParameter) <= 0)
        {
            err = VmbErrorBadParameter;
            break;
        }

        if(pParameter[0] == '/')
        {
            if( std::strcmp(pParameter, "/s") == 0)
            {
                if(SettingsModeUnknown != settingsMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                settingsMode = SettingsModeSave;
            }
            else if(std::strcmp(pParameter, "/l") == 0)
            {
                if(SettingsModeUnknown != settingsMode)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                settingsMode = SettingsModeLoad;
            }
            else if(StartsWith(pParameter, "/f:"))
            {
                if(fileName.empty() == false)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                fileName = pParameter + 3;
                if(fileName.size() <= 0)
                {
                    err = VmbErrorBadParameter;
                    break;
                }
            }
            else if( std::strcmp(pParameter, "/h") == 0)
            {
                if(true == printHelp)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                printHelp = true;
            }
            else if(std::strcmp(pParameter, "/i") == 0)
            {
                if(true == ignoreStreamable)
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                ignoreStreamable = true;
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

    if(     (       (cameraID.empty() == false)
                ||  (fileName.empty() == false)
                ||  (SettingsModeUnknown != settingsMode)
                ||  (true == ignoreStreamable))
        &&  (true == printHelp))
    {
        err = VmbErrorBadParameter;
    }

    //Write out an error if we could not parse the command line
    if(VmbErrorBadParameter == err)
    {
        std::cout << "Invalid parameter found.\n\n";
        printHelp = true;
    }

    //Print out help and end program
    if(true == printHelp)
    {
        std::cout << "Usage: LoadSaveSettings [CameraID] [/h] [/{s|l}] [/f:FileName] [/i]\n";
        std::cout << "Parameters:   CameraID    ID of the camera to use (using first camera if not specified)\n";
        std::cout << "              /h          Print out help\n";
        std::cout << "              /s          Save settings to file (default if not specified)\n";
        std::cout << "              /l          Load settings from file\n";
        std::cout << "              /f:FileName File name for operation\n";
        std::cout << "                          (default is \"CameraSettings.xml\" if not specified)\n";
        std::cout << "              /i          Ignore streamable property of features\n";

        return err;
    }

    if(fileName.empty() == true)
    {
        fileName = "CameraSettings.xml";
    }

    // Get a reference to the VimbaSystem singleton
    AVT::VmbAPI::VimbaSystem &rVimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();
    
    // Print out version of Vimba
    std::cout<<"Vimba Version V"<<rVimbaSystem<<"\n";

    //Startup API
    bool vimbaStarted = false;
    if(VmbErrorSuccess == err)
    {
        err = rVimbaSystem.Startup();
        if(VmbErrorSuccess != err)
        {
            std::cout << "Could not start system. Error code: " << err << "\n";
        }
        else
        {
            vimbaStarted = true;
        }
    }

    //Open camera
    AVT::VmbAPI::CameraPtr pCamera;
    if(VmbErrorSuccess == err)
    {
        if(cameraID.empty())
        {
            //Open first available camera

            //Fetch all cameras known to Vimba
            AVT::VmbAPI::CameraPtrVector cameras;
            err = rVimbaSystem.GetCameras(cameras);
            if(VmbErrorSuccess == err)
            {
                if( !cameras.empty() )
                {
                    for (   AVT::VmbAPI::CameraPtrVector::const_iterator iter = cameras.begin();
                            cameras.end() != iter;
                            ++iter )
                    {
                        //Check if we can open the camera in full mode
                        VmbAccessModeType accessMode = VmbAccessModeNone;
                        err = (*iter)->GetPermittedAccess(accessMode);
                        if(VmbErrorSuccess == err)
                        {
                            if(VmbAccessModeFull & accessMode)
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

                    if(NULL == pCamera)
                    {
                        std::cout << "Could not open any camera.\n";
                        err = VmbErrorNotFound;
                    }
                }
                else
                {
                    std::cout << "No camera available.\n";
                    err = VmbErrorNotFound;
                }
            }
            else
            {
                std::cout << "Could not list cameras. Error code: " << err <<"\n";
            }
        }
        else
        {
            //Open specific camera
            err = rVimbaSystem.OpenCameraByID(cameraID.c_str(), VmbAccessModeFull, pCamera);
            if(VmbErrorSuccess != err)
            {
                std::cout << "Could not open camera. Error code: " << err <<"\n";
            }
        }
    }

    if(VmbErrorSuccess == err)
    {
        std::cout << "File name: " << fileName << "\n";
        std::cout << "Camera ID: " << cameraID << "\n\n";

        switch(settingsMode)
        {
        default:
        case SettingsModeSave:
            {
                //Save settings to file
                err = AVT::VmbAPI::Examples::LoadSaveSettings::SaveToFile(pCamera, fileName.c_str(), ignoreStreamable);
                if(VmbErrorSuccess != err)
                {
                    std::cout << "Could not save settings to file. Error code: " << err <<"\n";
                }
                else
                {
                    std::cout << "Settings successfully written to file.\n";
                }
            }
            break;

        case SettingsModeLoad:
            {
                //Load settings from file
                AVT::VmbAPI::StringVector loadedFeatures;
                AVT::VmbAPI::StringVector missingFeatures;
                err = AVT::VmbAPI::Examples::LoadSaveSettings::LoadFromFile(pCamera, fileName.c_str(), loadedFeatures, missingFeatures, ignoreStreamable);
                if(VmbErrorSuccess != err)
                {
                    switch( err)
                    {
                    default:
                        std::cout << "Could not load settings from file. Error code: " << err <<"\n";
                        break;
                    case VmbErrorResources:
                        std::cout <<"a required resource could not be acquired\n";
                        break;
                    case VmbErrorBadParameter:
                        std::cout <<"input parameter not valid\n";
                        break;
                    case VmbErrorNotFound:
                        std::cout <<"camera was not found in settings file\n";
                        break;
                    }
                }
                else
                {
                    //Even if we don't get an error there may be features missing
                    //that haven't been restored. Let's print them out.
                    if(missingFeatures.size() > 0)
                    {
                        std::cout << "Settings loaded from file but not all features have be restored.\n";
                        std::cout << "Missing features: ";

                        bool first = true;
                        for(    AVT::VmbAPI::StringVector::iterator i = missingFeatures.begin();
                                i != missingFeatures.end();
                                ++i)
                        {
                            if(true == first)
                            {
                                first = false;
                            }
                            else
                            {
                                std::cout << ", ";
                            }

                            std::cout << *i;
                        }

                        std::cout << std::endl;

                        err = VmbErrorOther;
                    }
                    else
                    {
                        std::cout << "Settings successfully loaded from file.\n";
                    }
                }
            }
            break;
        }
    }

    //Close camera
    if(NULL != pCamera)
    {
        pCamera->Close();
    }
    
    //Shutdown API
    if(true == vimbaStarted)
    {
        rVimbaSystem.Shutdown();
    }

    return err;
}
