/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        program.cpp

  Description: Main entry point of LookUpTable example of VimbaCPP.

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
#include <stdlib.h>

#include "LookUpTable.h"
#include "Exception.h"

#include "Common/StreamSystemInfo.h"

using namespace std;

enum Mode
{
    ModeUnknown         = 0,
    ModeSave            = 1,
    ModeLoad            = 2,
    ModeSaveCSV         = 3,
    ModeLoadCSV         = 4,
    ModeEnable          = 7,
    ModeIsEnabled       = 8,
    ModeSetValue        = 9,
    ModeGetValue        = 10,
    ModeBitIn           = 11,
    ModeBitOut          = 12,
    ModeCount           = 13
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

    if( std::strlen( pString ) < std::strlen( pStart ) )
    {
        return false;
    }

    if( std::memcmp( pString, pStart, std::strlen( pStart ) ) != 0 )
    {
        return false;
    }

    return true;
}

int main( int argc, char* argv[] )
{
    cout << "///////////////////////////////////////////\n";
    cout << "/// AVT Vimba API Look Up Table Example ///\n";
    cout << "///////////////////////////////////////////\n\n";

    VmbErrorType err = VmbErrorSuccess;

    string      cameraID;
    string      fileName;
    string      controlIndex;
    bool        bValue          = false;
    VmbInt64_t  nValue          = 0;
    string      parameter;
    Mode        eMode           = ModeUnknown;
    bool        printHelp       = false;

    //////////////////////
    //Parse command line//
    //////////////////////

    for( int i = 1; i < argc; i++ )
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
            else if( std::strcmp( pParameter, "/sc" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeSaveCSV;
            }
            else if( StartsWith( pParameter, "/lc:" ) )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if( parameter.empty() == false )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                parameter = pParameter + 4;
                if( parameter.size() <= 0 )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                nValue = atoi( parameter.c_str() );

                eMode = ModeLoadCSV;
            }
            else if( StartsWith( pParameter, "/lc" ) )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if( parameter.empty() == false )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                nValue =  0;
                eMode = ModeLoadCSV;
            }
            else if( std::strcmp( pParameter, "/bi" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeBitIn;
            }
            else if( std::strcmp( pParameter, "/bo" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeBitOut;
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
            else if( StartsWith( pParameter, "/e:" ) )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if( parameter.empty() == false )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                parameter = pParameter + 3;
                if( parameter.size() <= 0 )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if( std::strcmp( parameter.c_str(), "on" ) == 0 )
                {
                    bValue = true;
                }
                else if( std::strcmp( parameter.c_str(), "off" ) == 0 )
                {
                    bValue = false;
                }
                else
                {
                    cout << "Could not set look up table enable. Wrong parameter!\n";
                    err = VmbErrorBadParameter;
                    break;
                }
                eMode = ModeEnable;
            }
            else if( std::strcmp( pParameter, "/e" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeIsEnabled;
            }
            else if( StartsWith( pParameter, "/v:" ) )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if( parameter.empty() == false )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                parameter = pParameter + 3;
                if( parameter.size() <= 0 )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                nValue = atoi( parameter.c_str() );

                eMode = ModeSetValue;
            }
            else if( std::strcmp( pParameter, "/v" ) == 0 )
            {
                if( ModeUnknown != eMode )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eMode = ModeGetValue;
            }
            else if( StartsWith( pParameter, "/f:" ) )
            {
                if( (ModeUnknown != eMode) &&
                    (ModeSaveCSV != eMode) &&
                    (ModeLoadCSV != eMode) )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                if( fileName.empty() == false )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                fileName = pParameter + 3;
                if( fileName.size() <= 0 )
                {
                    err = VmbErrorBadParameter;
                    break;
                }
            }
            else if( StartsWith( pParameter, "/i:" ) )
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
        cout << "Invalid parameters!\n\n";
        printHelp = true;
    }

    //Print out help and end program
    if( true == printHelp )
    {
        cout << "Usage: LookUpTable [CameraID] [/i:Index] [/h] [/{s|l|sc|lc:Column|u|d|v(:Value)|e(:Enable)|bi|bo|n}] [/f:FileName]\n";
        cout << "Parameters:   CameraID       ID of the camera to use\n";
        cout << "                             (using first camera if not specified)\n";
        cout << "              /i:Index       Set look up table index\n";
        cout << "              /h             Print out help\n";
        cout << "              /s             Save look up table to flash\n";
        cout << "              /l             Load look up table from flash\n";
        cout << "              /sc            Save look up table to Csv\n";
        cout << "                             (Look up table previously downloaded)\n";
        cout << "              /lc:Column     Load look up table from Csv using specified column\n";
        cout << "                             (default if not specified)\n";
        cout << "              /e:Enable      Set look up table enable [on/off]\n";
        cout << "              /e             Get look up table enable\n";
        cout << "                             (default if not specified)\n";
        cout << "              /v:Value       Set look up table value\n";
        cout << "              /v             Get look up table value\n";
        cout << "              /bi            Get look up table bit depth in\n";
        cout << "              /bo            Get look up table bit depth out\n";
        cout << "              /n             Get look up table count\n";
        cout << "              /f:FileName    File name for operation\n\n";
        cout << "For example to load a look up table from the csv file C:\\lut.csv and\n"
             << "write it to the camera's flash as LUT1 call\n\n";
        cout << "LookUpTable /i:0 /lc:0 /f:\"C:\\lut.csv\"\n\n";
        cout << "To load the look up table LUT2 from the camera and write it\n"
             << "to the csv file C:\\lut.csv call\n\n";
        cout << "LookUpTable /i:1 /sc /f:\"C:\\lut.csv\"\n\n";

        return err;
    }

    bool                        bVimbaStarted   = false;
    AVT::VmbAPI::CameraPtr      pCamera;
    AVT::VmbAPI::VimbaSystem&   vimbaSystem     = AVT::VmbAPI::VimbaSystem::GetInstance();

    try
    {
        // Print out version of Vimba
        std::cout<<"Vimba Version V"<<vimbaSystem<<"\n";

        //Startup API
        if( VmbErrorSuccess == err )
        {
            err = vimbaSystem.Startup();
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
                err = vimbaSystem.GetCameras( cameras );
                if( VmbErrorSuccess == err )
                {
                    if( cameras.size() > 0 )
                    {
                        for( AVT::VmbAPI::CameraPtrVector::const_iterator iter = cameras.begin();
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
                                    err = ( *iter )->GetID( cameraID );
                                    if( VmbErrorSuccess == err )
                                    {
                                        //Try to open the camera
                                        err = ( *iter )->Open( VmbAccessModeFull );
                                        if( VmbErrorSuccess == err )
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
                            throw AVT::VmbAPI::Examples::Exception( "Could not open any camera.", err );
                        }
                    }
                    else
                    {
                        err = VmbErrorNotFound;
                        throw AVT::VmbAPI::Examples::Exception( "Could not open any camera.", err );
                    }
                }
                else
                {
                    throw AVT::VmbAPI::Examples::Exception( "Could not list cameras.", err );
                }
            }
            else
            {
                //Open specific camera
                err = vimbaSystem.OpenCameraByID( cameraID.c_str(), VmbAccessModeFull, pCamera );
                if( VmbErrorSuccess != err )
                {
                    throw AVT::VmbAPI::Examples::Exception( "Could not open camera.", err );
                }
            }
        }

        if( VmbErrorSuccess == err )
        {
            cout << "Camera ID: " << cameraID << "\n\n";

            AVT::VmbAPI::Examples::LookUpTableCollection collection( pCamera );


            VmbInt64_t nIndex;

            if( controlIndex.empty() == false )
            {
                nIndex = atoi( controlIndex.c_str() );
            }
            else
            {
                nIndex = collection.GetActiveIndex( );
                if( VmbErrorSuccess != err )
                {
                    throw AVT::VmbAPI::Examples::Exception( "Could not get active index of look up table collection.", err );
                }
            }

            AVT::VmbAPI::Examples::LookUpTableControl control( pCamera, nIndex );
            control = collection.GetControl( nIndex );

            if( VmbErrorSuccess == err )
            {
                switch( eMode )
                {
                default:
                case ModeIsEnabled:
                    {
                        //Is look up table enabled
                        cout << "Get look up table enable was successful. Enable = " << control.IsEnabled( ) <<"\n";
                    }
                    break;

                case ModeLoad:
                    {                
                        //Load look up table from flash
                        control.LoadFromFlash();
                        cout << "Look up table successfully loaded from flash.\n";
                    }
                    break;

                case ModeSave:
                    {
                        //Save look up table to flash
                        control.SaveToFlash();
                        cout << "Look up table successfully saved to flash.\n";
                    }
                    break;

                case ModeSaveCSV:
                    {
                        //Download LUT
                        control.Download();
                        //Save look up table to file
                        control.SaveToCsv( fileName.c_str() );
                        cout << "Look up table successfully saved to CSV.\n";
                    }
                    break;

                case ModeLoadCSV:
                    {                
                        //Load look up table from file
                        control.LoadFromCsv( fileName, (int)nValue );
                        control.Upload();
                        cout << "Look up table successfully loaded from CSV.\n";
                    }
                    break;

                case ModeEnable:
                    {
                        //Set look up table enable
                        control.Enable( bValue );
                        cout << "Look up table was enabled successfully.\n";
                    }
                    break;

                case ModeBitIn:
                    {
                        //Get bit depth in of look up table
                        cout << "Get look up table 'bit depth in' was successful. BitDepthIn = " << control.GetBitDepthIn( ) <<"\n";
                    }
                    break;

                case ModeBitOut:
                    {
                        //Get bit depth out of look up table
                        cout << "Get look up table 'bit depth out' was successful. BitDepthOut = " << control.GetBitDepthOut( ) <<"\n";
                    }
                    break;

                case ModeSetValue:
                    {
                        //Set look up table value
                        control.SetValue( nValue );
                        cout << "Look up table value was set successfully." <<"\n";
                    }
                    break;

                case ModeGetValue:
                    {
                        //Get look up table value
                        cout << "Get look up table value was successful. Value = " << control.GetValue()  <<"\n";
                    }
                    break;

                case ModeCount:
                    {
                        //Get look up table count
                        cout << "Get look up table count was successful. Count = " << collection.GetCount( ) <<"\n";
                    }
                    break;

                }
            }
        }
    }

    catch( AVT::VmbAPI::Examples::Exception& ex )
    {
        cout << ex.GetMessageStr() << " VimbaException: " << ex.GetError() << " = " << ex.ErrorCodeToMessage( ex.GetError() ) <<"\n";
    }

    //Close camera
    if( NULL != pCamera )
    {
        pCamera->Close();
    }

    //Shutdown API
    if( true == bVimbaStarted )
    {
        vimbaSystem.Shutdown();
    }

    return err;
}