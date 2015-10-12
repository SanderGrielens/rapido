/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ShadingData.cpp

  Description: The ShadingData example will demonstrate how to use
               the shading data feature of the camera using VimbaCPP.
               

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

#include <string>
#include <sstream>
#include <vector>

#include "ShadingData.h"
#include "Exception.h"

#include "VimbaCPP/Include/VimbaCPP.h"


namespace AVT {
namespace VmbAPI {
namespace Examples {


CameraPtr      ShadingDataControl::m_pCamera;
UcharVector    ShadingDataControl::m_data;

ShadingDataControl::ShadingDataControl( CameraPtr pCamera )
{
    //Check parameters
    if( NULL == pCamera )
    {
        throw Exception("ShadingDataControl failed.", VmbErrorBadParameter);
    }

    m_pCamera = pCamera;
}

//Is shading data enabled
VmbErrorType ShadingDataControl::IsEnabled (  bool &bEnabled )
{
    VmbErrorType err = VmbErrorSuccess;

    FeaturePtr pFeature;

    err = m_pCamera->GetFeatureByName ( "ShadingCorrectionEnable", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->GetValue ( bEnabled );
    }

    return err;
}

//Enable shading data
VmbErrorType ShadingDataControl::Enable ( bool bEnable )
{
    VmbErrorType err = VmbErrorSuccess;

    FeaturePtr pFeature;

    err = m_pCamera->GetFeatureByName ( "ShadingCorrectionEnable", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->SetValue ( bEnable );
    }

    return err;
}

//Is shading data shown
VmbErrorType ShadingDataControl::IsDataShown (  bool &bDataShown )
{
    VmbErrorType err = VmbErrorSuccess;

    FeaturePtr pFeature;

    err = m_pCamera->GetFeatureByName ( "ShadingCorrectionShowData", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->GetValue ( bDataShown );
    }

    return err;
}

//Show shading data
VmbErrorType ShadingDataControl::ShowData ( bool bShowData )
{
    VmbErrorType err = VmbErrorSuccess;

    FeaturePtr pFeature;

    err = m_pCamera->GetFeatureByName ( "ShadingCorrectionShowData", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->SetValue ( bShowData );
    }

    return err;
}

//Upload shading data
VmbErrorType ShadingDataControl::Upload ()
{
    VmbErrorType err = VmbErrorSuccess;

    //Shading raw data empty
    if ( 0 == m_data.size() )
    {
        err = VmbErrorOther;
    }

    FeaturePtr    pFileOperationSelector;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOperationSelector", pFileOperationSelector );
    }

    FeaturePtr    pFileOperationExecute;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOperationExecute", pFileOperationExecute );
    }

    FeaturePtr    pFileOpenMode;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOpenMode", pFileOpenMode );
    }

    FeaturePtr    pFileAccessBuffer;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileAccessBuffer", pFileAccessBuffer );
    }

    FeaturePtr    pFileAccessOffset;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileAccessOffset", pFileAccessOffset );
    }

    FeaturePtr    pFileAccessLength;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileAccessLength", pFileAccessLength );
    }
            
    FeaturePtr    pFileOperationStatus;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOperationStatus", pFileOperationStatus );
    }

    FeaturePtr    pFileOperationResult;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOperationResult", pFileOperationResult );
    }

    FeaturePtr    pFileStatus;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileStatus", pFileStatus );
    }

    FeaturePtr    pFileSize;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileSize", pFileSize );
    }

    FeaturePtr    pFileSelector;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileSelector", pFileSelector );
    }

    //Select current LUTSelector as selected file
    string FileSelectorValue;
    if ( VmbErrorSuccess == err )
    {
        FileSelectorValue = "ShadingData";
        err = pFileSelector->SetValue ( FileSelectorValue.c_str() );
    }

    //Select Write as open mode
    if ( VmbErrorSuccess == err )
    {
        err = pFileOpenMode->SetValue( "Write" );
    }

    //Select Open as file operation and Execute
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationSelector->SetValue( "Open" );
    }
            
    //Execute file operation
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationExecute->RunCommand ();
    }

    //File size
    VmbInt64_t nFileSize;
    if ( VmbErrorSuccess == err )
    {
        err = pFileSize->GetValue ( nFileSize );
    }

    //File access length
    VmbInt64_t nMinFileAccessLength;
    VmbInt64_t nMaxFileAccessLength;
    if ( VmbErrorSuccess == err )
    {
        err = pFileAccessLength->GetRange( nMinFileAccessLength, nMaxFileAccessLength );
    }

    //Select write as file operation
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationSelector->SetValue ( "Write" );
    }

    VmbInt64_t nFileAccessOffset = 0;
    VmbInt64_t nFileAccessLength = min( nFileSize, nMaxFileAccessLength );
    UcharVector data( (size_t)nFileAccessLength );
    string LUTOperationStatus;

    do
    {
        //Set FileAccessLength
        if ( VmbErrorSuccess == err )
        {
            err = pFileAccessLength->SetValue ( nFileAccessLength );
        }

        //Fill buffer
        if ( VmbErrorSuccess == err )
        {
            copy( m_data.begin() + (size_t)nFileAccessOffset, m_data.begin() + (size_t)(nFileAccessLength + nFileAccessOffset), data.begin() ); 
            err = pFileAccessBuffer->SetValue( data );
        }

        //Execute file operation
        if ( VmbErrorSuccess == err )
        {
            err = pFileOperationExecute->RunCommand ();
        }

        //Get file operation status
        if ( VmbErrorSuccess == err )
        {
            err = pFileOperationStatus->GetValue ( LUTOperationStatus );
        }

        if ( VmbErrorSuccess == err )
        {
            if ( LUTOperationStatus != "Success")
            {
                err = VmbErrorOther;
            }
        }

        //File access offset
        if ( VmbErrorSuccess == err )
        {
            err = pFileAccessOffset->GetValue( nFileAccessOffset );
        }

                
        if ( VmbErrorSuccess != err )
        {
            break;
        }

        nFileAccessLength = min( nFileSize - nFileAccessOffset, nMaxFileAccessLength );
    }
    while ( nFileSize != nFileAccessOffset );

    //Select close as file operation
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationSelector->SetValue ( "Close" );
    }

    //Execute file operation
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationExecute->RunCommand ();
    }

    //Get file operation status
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationStatus->GetValue ( LUTOperationStatus );
    }

    if ( VmbErrorSuccess == err )
    {
        if ( LUTOperationStatus != "Success")
        {
            err = VmbErrorOther;
        }
    }

    return err;
}

//Download shading data
VmbErrorType ShadingDataControl::Download ()
{
    VmbErrorType err = VmbErrorSuccess;

    //Shading raw data empty
    if ( 0 == m_data.size() )
    {
        err = VmbErrorOther;
    }

    FeaturePtr    pFileOperationSelector;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOperationSelector", pFileOperationSelector );
    }

    FeaturePtr    pFileOperationExecute;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOperationExecute", pFileOperationExecute );
    }

    FeaturePtr    pFileOpenMode;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOpenMode", pFileOpenMode );
    }

    FeaturePtr    pFileAccessBuffer;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileAccessBuffer", pFileAccessBuffer );
    }

    FeaturePtr    pFileAccessOffset;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileAccessOffset", pFileAccessOffset );
    }

    FeaturePtr    pFileAccessLength;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileAccessLength", pFileAccessLength );
    }
            
    FeaturePtr    pFileOperationStatus;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOperationStatus", pFileOperationStatus );
    }

    FeaturePtr    pFileOperationResult;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileOperationResult", pFileOperationResult );
    }

    FeaturePtr    pFileStatus;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileStatus", pFileStatus );
    }

    FeaturePtr    pFileSize;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileSize", pFileSize );
    }

    FeaturePtr    pFileSelector;
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "FileSelector", pFileSelector );
    }

    //Select current LUTSelector as selected file
    string FileSelectorValue;
    if ( VmbErrorSuccess == err )
    {
        FileSelectorValue = "ShadingData";
        err = pFileSelector->SetValue ( FileSelectorValue.c_str() );
    }

    //Select Write as open mode
    if ( VmbErrorSuccess == err )
    {
        err = pFileOpenMode->SetValue( "Read" );
    }

    //Select Open as file operation and Execute
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationSelector->SetValue( "Open" );
    }
            
    //Execute file operation
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationExecute->RunCommand ();
    }

    //File size
    VmbInt64_t nFileSize;
    if ( VmbErrorSuccess == err )
    {
        err = pFileSize->GetValue ( nFileSize );
    }

    //Allocate buffer
    if ( VmbErrorSuccess == err )
    {
        m_data.resize( (size_t)nFileSize );
    }

    //File access length
    VmbInt64_t nMinFileAccessLength;
    VmbInt64_t nMaxFileAccessLength;
    if ( VmbErrorSuccess == err )
    {
        err = pFileAccessLength->GetRange( nMinFileAccessLength, nMaxFileAccessLength );
    }

    //Select write as file operation
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationSelector->SetValue ( "Read" );
    }

    
    VmbInt64_t nFileAccessOffset = 0;
    VmbInt64_t nFileAccessLength = min( nFileSize, nMaxFileAccessLength );
    UcharVector data( (size_t)nFileAccessLength );
    string LUTOperationStatus;

    do
    {
        //Set FileAccessLength
        if ( VmbErrorSuccess == err )
        {
            err = pFileAccessLength->SetValue ( nFileAccessLength );
        }

        //Execute file operation
        if ( VmbErrorSuccess == err )
        {
            err = pFileOperationExecute->RunCommand ();
        }

        //Get file operation status
        if ( VmbErrorSuccess == err )
        {
            err = pFileOperationStatus->GetValue ( LUTOperationStatus );
        }

        if ( VmbErrorSuccess == err )
        {
            if ( LUTOperationStatus != "Success")
            {
                err = VmbErrorOther;
            }
        }

        //Get file access buffer
        if ( VmbErrorSuccess == err )
        {
            err = pFileAccessBuffer->GetValue( data );
        }

        //Copy to buffer
        if ( VmbErrorSuccess == err )
        {
            copy( data.begin(), data.end(), m_data.begin() + (size_t)nFileAccessOffset ); 
        }

        //File access offset
        if ( VmbErrorSuccess == err )
        {
            err = pFileAccessOffset->GetValue( nFileAccessOffset );
        }

                
        if ( VmbErrorSuccess != err )
        {
            break;
        }

        nFileAccessLength = min( nFileSize - nFileAccessOffset, nMaxFileAccessLength );
    }
    while ( nFileSize != nFileAccessOffset );

    //Select close as file operation
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationSelector->SetValue ( "Close" );
    }

    //Execute file operation
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationExecute->RunCommand ();
    }

    //Get file operation status
    if ( VmbErrorSuccess == err )
    {
        err = pFileOperationStatus->GetValue ( LUTOperationStatus );
    }
    
    if ( VmbErrorSuccess == err )
    {
        if ( LUTOperationStatus != "Success")
        {
            err = VmbErrorOther;
        }
    }

    return err;
}

//Get raw data
VmbErrorType ShadingDataControl::GetRawData ( UcharVector& data )
{
    VmbErrorType err = VmbErrorSuccess;

    data = m_data;

    return err;
}

//Set raw data
VmbErrorType ShadingDataControl::SetRawData ( UcharVector data )
{
    VmbErrorType err = VmbErrorSuccess;

    m_data = data;

    return err;
}

//Build shading data with image count
VmbErrorType ShadingDataControl::BuildImages ( VmbInt64_t nImageCount )
{
    VmbErrorType err = VmbErrorSuccess;

    FeaturePtr pFeature;

    //Set image count
    err = m_pCamera->GetFeatureByName ( "ShadingDataBuildImages", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->SetValue ( nImageCount );
    }

    //Build shading data
    if ( VmbErrorSuccess == err )
    {
        err = m_pCamera->GetFeatureByName ( "ShadingDataBuild", pFeature );

        if ( VmbErrorSuccess == err )
        {
            err = pFeature->RunCommand ();
        }
    }

    return err;
}

//Load shading data from flash
VmbErrorType ShadingDataControl::LoadFromFlash ()
{
    VmbErrorType err = VmbErrorSuccess;

    FeaturePtr pFeature;

    err = m_pCamera->GetFeatureByName ( "ShadingDataLoadFromFlash", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->RunCommand ();
    }

    return err;
}

//Save shading data to flash
VmbErrorType ShadingDataControl::SaveToFlash ()
{
    VmbErrorType err = VmbErrorSuccess;

    FeaturePtr pFeature;

    err = m_pCamera->GetFeatureByName ( "ShadingDataSaveToFlash", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->RunCommand ();
    }

    return err;
}

//Clear shading data from flash
VmbErrorType ShadingDataControl::ClearFlash ()
{
    VmbErrorType err = VmbErrorSuccess;

    FeaturePtr pFeature;

    err = m_pCamera->GetFeatureByName ( "ShadingDataClearFlash", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->RunCommand ();
    }

    return err;
}

}}} // namespace AVT::VmbAPI::Examples

