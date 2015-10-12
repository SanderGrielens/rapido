/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        program.cpp

  Description: Main entry point of UserSet example of VimbaCPP.

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

#include "UserSet.h"
#include "Exception.h"

#include "Common/StreamSystemInfo.h"

enum Mode
{
    ModeUnknown         = 0,
    ModeSave            = 1,
    ModeLoad            = 2,
    ModeCount           = 3,
    ModeIndex           = 4,
    ModeMakeDefault     = 5,
    ModeIsDefault       = 6,
    ModeOperationResult = 7,
    ModeOperationStatus = 8,
};

bool StartsWith( const char *pString, const char *pStart )
{
    if( NULL == pString )
    {
        return false;
    }
    if( NULL == pStart )
    {
        return false;
    }

    if( std::strlen(pString) < std::strlen(pStart) )
    {
        return false;
    }

    if( std::memcmp(pString, pStart, std::strlen(pStart)) != 0 )
    {
        return false;
    }

    return true;
}

int main( int argc, char* argv[] )
{
    std::cout << "//////////////////////////////////////" << std::endl;
    std::cout << "/// AVT Vimba API User Set Example ///" << std::endl;
    std::cout << "//////////////////////////////////////" << std::endl << std::endl;

    VmbErrorType err = VmbErrorSuccess;

    std::string     cameraID;
    std::string     controlIndex;
    Mode            eMode           = ModeUnknown; 
    bool            printHelp       = false;

    //////////////////////
    //Parse command line//
    //////////////////////

    for(int i = 1; i < argc; i++)
    {
        char *pParameter = argv[i];
        if( std::strlen( pParameter ) < 0 )
        {
            err = VmbErrorBadParameter;
            break;
        }

        if( pParameter[0] == '/' )
        {
            if( std::strcmp( pParameter, "/s" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeSave;
            }
            else if( std::strcmp( pParameter, "/l" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeLoad;
            }
            else if( std::strcmp( pParameter, "/n" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeCount;
            }
            else if( std::strcmp( pParameter, "/i" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeIndex;
            }
            else if( StartsWith( pParameter, "/i:" ))
            {
                if( controlIndex.empty() == false )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                controlIndex = pParameter + 3;
                if( controlIndex.size() <= 0 )
                {
                    err = VmbErrorBadParameter;
                    break;
                }
            }
            else if( std::strcmp( pParameter, "/m" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeMakeDefault;
            }
            else if( std::strcmp( pParameter, "/d" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeIsDefault;
            }
            else if( std::strcmp( pParameter, "/or" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeOperationResult;
            }
            else if( std::strcmp( pParameter, "/os" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeOperationStatus;
            }
            else if( std::strcmp( pParameter, "/h" ) == 0 )
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
            if( cameraID.empty() == false )
            {
                err = VmbErrorBadParameter;
                break;
            }

            cameraID = pParameter;
        }
    }

    //Write out an error if we could not parse the command line
    if( VmbErrorBadParameter == err )
    {
        std::cout << "Invalid parameters!\n\n";
        printHelp = true;
    }

    //Print out help and end program
    if( true == printHelp )
    {
        std::cout << "Usage: UserSet [CameraID] [/i:Index] [/h] [/{s|l|i|m|d|or|os|n}]\n";
        std::cout << "Parameters:   CameraID       ID of the camera to use\n";
        std::cout << "                             (using first camera if not specified)\n";
        std::cout << "              /i:Index       Set user set index\n";
        std::cout << "              /h             Print out help\n";
        std::cout << "              /s             Save user set to flash\n";
        std::cout << "              /l             Load user set from flash\n";
        std::cout << "                             (default if not specified)\n";
        std::cout << "              /i             Get selected user set index\n";
        std::cout << "              /m             Make user set default\n";
        std::cout << "              /d             Is user set default\n";
        std::cout << "              /or            Get user set operation result\n";
        std::cout << "              /os            Get user set operation status\n";
        std::cout << "              /n             Get user set count\n\n";
        std::cout << "For example to load user set 0 (factory set) from flash in order to\nactivate it call\n";
        std::cout << "UserSet /i:0 /l\n\n" ;
        std::cout << "To save the current settings to user set 1 call\n";
        std::cout << "UserSet /i:1 /s\n\n";

        return err;
    }

    bool                        bVimbaStarted   = false;
    AVT::VmbAPI::CameraPtr      pCamera;
    AVT::VmbAPI::VimbaSystem *  pVimbaSystem    = NULL;

    try
    {
        // Get a pointer to the VimbaSystem singleton
        pVimbaSystem = &AVT::VmbAPI::VimbaSystem::GetInstance();
        
        // Print out version of Vimba
        std::cout<<"Vimba Version V"<<*pVimbaSystem<<"\n";

        //Startup API
        if( VmbErrorSuccess == err )
        {
            err = pVimbaSystem->Startup();
            if( VmbErrorSuccess != err )
            {
                throw AVT::VmbAPI::Examples::Exception( "Could not start system.", err );
            }
            bVimbaStarted = true;
        }

        //Open camera
        if( VmbErrorSuccess == err )
        {
            if( cameraID.empty() )
            {
                //Open first available camera

                //Fetch all cameras known to Vimba
                AVT::VmbAPI::CameraPtrVector cameras;
                err = pVimbaSystem->GetCameras(cameras);
                if( VmbErrorSuccess == err )
                {
                    if( cameras.size() > 0 )
                    {
                        for (   AVT::VmbAPI::CameraPtrVector::const_iterator iter = cameras.begin();
                                cameras.end() != iter;
                                ++iter )
                        {
                            //Check if we can open the camera in full mode
                            VmbAccessModeType eAccessMode = VmbAccessModeNone;
                            err = (*iter)->GetPermittedAccess( eAccessMode );
                            if( VmbErrorSuccess == err )
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

                        if( VmbErrorSuccess != err )
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
                if( VmbErrorSuccess != err )
                {
                    throw AVT::VmbAPI::Examples::Exception("Could not open camera.", err);
                }
            }
        }

        if( VmbErrorSuccess == err )
        {
            std::cout << "Camera ID: " << cameraID << std::endl << std::endl;

            AVT::VmbAPI::Examples::UserSetCollection collection( pCamera );

            VmbInt64_t nIndex;

            if ( controlIndex.empty() == false )
            {
                nIndex = atoi( controlIndex.c_str() );
            }
            else
            {
                err = collection.GetSelectedIndex( nIndex );
                if( VmbErrorSuccess != err )
                {
                    throw AVT::VmbAPI::Examples::Exception( "Could not get selected user set index.", err );
                }
            }

            AVT::VmbAPI::Examples::UserSetControl control( pCamera, nIndex );
            if ( (ModeCount != eMode) && (ModeIndex != eMode) )
            {
                err = collection.GetControl( nIndex, control );
                if( VmbErrorSuccess != err )
                {
                    throw AVT::VmbAPI::Examples::Exception( "Could not get user set control.", err );
                }
            }

            switch( eMode )
            {
            default:
            case ModeLoad:
                {
                    //Load user set
                    err = control.LoadFromFlash();
                    if( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception( "Could not load user set from flash.", err );
                    }

                    std::cout << "User set successfully loaded from flash." << std::endl;
                }
                break;

            case ModeSave:
                {
                    //Save user set
                    err = control.SaveToFlash();
                    if( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception( "Could not save user set to flash.", err );
                    }

                    std::cout << "User set successfully saved to flash." << std::endl;
                }
                break;

            case ModeCount:
                {
                    //Get user set count
                    VmbInt64_t nCount;
                    err = collection.GetCount( nCount );
                    if( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception( "Could not get user set count.", err );
                    }

                    std::cout << "Get user set count was successful. Count = " << nCount << std::endl;
                }
                break;

            case ModeIndex:
                {
                    //Get selected user set index
                    VmbInt64_t nSelectedIndex;
                    err = collection.GetSelectedIndex( nSelectedIndex );
                    if( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception( "Could not get user set index.", err );
                    }

                    std::cout << "Get selected user set was successful. Index = " << nSelectedIndex << std::endl;
                }
                break;

            case ModeMakeDefault:
                {
                    err = control.MakeDefault();

                    if( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception( "Could not set user set default.", err );
                    }

                    std::cout << "Make user set default was successful. " << std::endl;
                }
                break;

            case ModeIsDefault:
                {
                    bool bIsDefault = false;
                    if( VmbErrorSuccess == err )
                    {
                        err = control.IsDefault( bIsDefault );
                    }

                    if( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception( "Could not get user set default.", err );
                    }

                    std::cout << "Is user set default was successful. Result = " << bIsDefault << std::endl;
                }
                break;


            case ModeOperationResult:
                {
                    VmbInt64_t nResult;
                    err = control.GetOperationResult( nResult );
                    if( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception( "Could not get user set opration result.", err );
                    }
                        
                    std::cout << "Get user set operation result was successful. Operation Result = " << nResult << std::endl;
                }
                break;
            case ModeOperationStatus:
                {
                    VmbInt64_t nStatus;
                    err = control.GetOperationStatus( nStatus );
                    if( VmbErrorSuccess != err )
                    {
                        throw AVT::VmbAPI::Examples::Exception( "Could not get user set operation status.", err );
                    }

                    std::cout << "Get user set operation status was successful. Operation Status = " << nStatus << std::endl;
                }
                break;

            }
        }
    }

    catch( AVT::VmbAPI::Examples::Exception& ex )
    {
        std::cout << ex.GetMessageStr() << " VimbaException: " << ex.GetError() << " = " << ex.ErrorCodeToMessage( ex.GetError() ) << std::endl;
    }

    //Close camera
    if( NULL != pCamera )
    {
        pCamera->Close();
    }
    
    //Shutdown API
    if(true == bVimbaStarted)
    {
        pVimbaSystem->Shutdown();
    }

    return err;
}
