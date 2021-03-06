/*=============================================================================
  Copyright (C) 2013 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        program.c

  Description: Implementation of main entry point of AsynchronousGrab example
               of VimbaC.

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

#include <stdio.h>
#include <string.h>

#include <AsynchronousGrab.h>
#include "Common/ErrorCodeToMessage.h"
int main( int argc, char* argv[] )
{
    VmbError_t      err                     = VmbErrorSuccess;

    char*           pCameraID               = NULL;             // The ID of the camera to use
    FrameInfos      eFrameInfos             = FrameInfos_Off;   // Show frame infos
    VmbBool_t       bEnableColorProcessing  = VmbBoolFalse;     // enables color processing of frames
    unsigned char   bPrintHelp              = 0;                // Output help?
    int             i                       = 0;                // Counter for some iteration
    char*           pParameter              = NULL;             // The command line parameter

    printf( "///////////////////////////////////////////////\n" );
    printf( "/// AVT Vimba API Asynchronous Grab Example ///\n" );
    printf( "///////////////////////////////////////////////\n\n" );

    //////////////////////
    //Parse command line//
    //////////////////////

    for( i = 1; i < argc; ++i )
    {
        pParameter = argv[i];
        if( 0 > strlen( pParameter ))
        {
            err = VmbErrorBadParameter;
            break;
        }

        if( '/' == pParameter[0] )
        {
            if( 0 == strcmp( pParameter, "/i" ))
            {
                if(     ( FrameInfos_Off != eFrameInfos )
                    ||  ( bPrintHelp ))
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eFrameInfos = FrameInfos_Show;
            }
            if( 0 == strcmp( pParameter, "/c" ))
            {
                if (bPrintHelp )
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                bEnableColorProcessing = VmbBoolTrue;
            }
            else if( 0 == strcmp( pParameter, "/a" ))
            {
                if(     ( FrameInfos_Off != eFrameInfos )
                    ||  ( bPrintHelp ))
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                eFrameInfos = FrameInfos_Automatic;
            }
            else if( 0 == strcmp( pParameter, "/h" ))
            {
                if(     ( NULL != pCameraID )
                    ||  ( bPrintHelp )
                    ||  ( VmbBoolTrue    != bEnableColorProcessing)
                    ||  ( FrameInfos_Off != eFrameInfos ))
                {
                    err = VmbErrorBadParameter;
                    break;
                }

                bPrintHelp = 1;
            }
            else
            {
                err = VmbErrorBadParameter;
                break;
            }
        }
        else
        {
            if( NULL != pCameraID )
            {
                err = VmbErrorBadParameter;
                break;
            }

            pCameraID = pParameter;
        }
    }

    //Write out an error if we could not parse the command line
    if ( VmbErrorBadParameter == err )
    {
        printf( "Invalid parameters!\n\n" );
        bPrintHelp = 1;
    }

    //Print out help and end program
    if ( bPrintHelp )
    {
        printf( "Usage: AsynchronousGrab [CameraID] [/i] [/h]\n" );
        printf( "Parameters:   CameraID    ID of the camera to use (using first camera if not specified)\n" );
        printf( "              /c          enable color processing\n" );
        printf( "              /i          Show frame infos\n" );
        printf( "              /a          Automatically only show frame infos of corrupt frames\n" );
        printf( "              /h          Print out help\n" );
    }
    else
    {
        err = StartContinuousImageAcquisition(pCameraID, eFrameInfos,bEnableColorProcessing);
        if ( VmbErrorSuccess == err )
        {
           printf( "Press <enter> to stop acquision...\n" );
           getchar();

           StopContinuousImageAcquisition();
        }
        
        if ( VmbErrorSuccess == err )
        {
            printf( "\nAcquisition stopped.\n" );
        }
        else
        {
            printf( "\nAn error occurred: %s\n", ErrorCodeToMessage( err ) );
        }
    }

    return err;
}
