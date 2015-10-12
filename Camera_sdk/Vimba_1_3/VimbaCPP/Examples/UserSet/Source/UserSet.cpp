/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        UserSet.cpp

  Description: The UserSet example will demonstrate how deal with the user sets
               stored inside the cameras using VimbaCPP.
               

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

#include "UserSet.h"
#include "Exception.h"

#include "VimbaCPP/Include/VimbaCPP.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

CameraPtr                   UserSetCollection::m_pCamera;
std::vector<UserSetControl> UserSetCollection::m_usersets;

UserSetCollection::UserSetCollection( CameraPtr pCamera )
{
    //Check parameters
    if ( NULL == pCamera )
    {
        throw Exception( "UserSetCollection failed.", VmbErrorBadParameter );
    }

    m_pCamera = pCamera;

    VmbErrorType err = VmbErrorSuccess;

    VmbInt64_t nCount;
    err = GetCount ( nCount );
    if ( VmbErrorSuccess == err )
    {
        m_usersets.reserve( (size_t)nCount );
    }

    if ( VmbErrorSuccess == err )
    {
        for ( int i = 0; i < nCount; i++ )
        {
            AVT::VmbAPI::Examples::UserSetControl control( m_pCamera, i ) ;
            m_usersets.push_back( control );
        }
    }

    if ( VmbErrorSuccess  != err )
    {
        throw Exception( "UserSetCollection failed.", err );
    }
}

//Get user set count
VmbErrorType UserSetCollection::GetCount ( VmbInt64_t& rCount )
{
    VmbErrorType                err = VmbErrorSuccess;

    FeaturePtr                  pFeature;
    AVT::VmbAPI::StringVector   selector;

    err = m_pCamera->GetFeatureByName ( "UserSetSelector", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->GetValues ( selector );
        rCount = selector.size();
    }

    return err;
}

//Get user set control
VmbErrorType UserSetCollection::GetControl ( VmbInt64_t nIndex, UserSetControl& rUserSet )
{
    VmbErrorType    err = VmbErrorSuccess;

    VmbInt64_t      nCount;
    err = GetCount ( nCount );
    if ( VmbErrorSuccess == err )
    {
        if ( nIndex >= nCount )
        {
            err = VmbErrorBadParameter;
        }

        FeaturePtr pFeature;

        //Access to user set changes the selector
        if ( VmbErrorSuccess == err )
        {
            err = m_pCamera->GetFeatureByName( "UserSetSelector", pFeature );
        }

        if(VmbErrorSuccess == err)
        {
            err = pFeature->SetValue( nIndex );
        }

        if(VmbErrorSuccess == err)
        {
            rUserSet = m_usersets[ (size_t)nIndex ];
        }
    }

    return err;
}

//Get selected user set index
VmbErrorType UserSetCollection::GetSelectedIndex ( VmbInt64_t& rIndex )
{
    VmbErrorType    err = VmbErrorSuccess;

    FeaturePtr      pFeature;

    err = m_pCamera->GetFeatureByName ( "UserSetSelector", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->GetValue ( rIndex );
    }

    return err;
}

//Get default user set index
VmbErrorType UserSetCollection::GetDefaultIndex ( VmbInt64_t& rDefaultIndex )
{
    VmbErrorType    err = VmbErrorSuccess;

    FeaturePtr      pFeature;

    err = m_pCamera->GetFeatureByName ( "UserSetDefaultSelector", pFeature );

    if ( VmbErrorSuccess == err )
    {
        err = pFeature->GetValue ( rDefaultIndex );
    }

    return err;
}

UserSetControl::UserSetControl( CameraPtr pCamera, VmbInt64_t nIndex )
{
    //Check parameters
    if ( NULL == pCamera )
    {
        throw Exception( "UserSetControl failed.", VmbErrorBadParameter );
    }

    m_pCamera = pCamera;
    m_nIndex = nIndex;
}

//Is user set default
VmbErrorType UserSetControl::IsDefault ( bool& bIsDefault )
{
    bIsDefault = false;

    VmbErrorType    err = VmbErrorSuccess;

    FeaturePtr      pFeature;

    err = m_pCamera->GetFeatureByName( "UserSetDefaultSelector", pFeature );

    VmbInt64_t      nDefaultIndex;
    if(VmbErrorSuccess == err)
    {
        err = pFeature->GetValue( nDefaultIndex );
    }

    if(VmbErrorSuccess == err)
    {
        if ( nDefaultIndex == m_nIndex )
        {
            bIsDefault = true;
        }
    }

    return err;
}

//Make user set default
VmbErrorType UserSetControl::MakeDefault ()
{
    VmbErrorType    err = VmbErrorSuccess;

    FeaturePtr      pFeature;

    err = m_pCamera->GetFeatureByName( "UserSetDefaultSelector", pFeature );
    if ( VmbErrorNotFound == err )
    {
        err = m_pCamera->GetFeatureByName ( "UserSetMakeDefault", pFeature );
        if(VmbErrorSuccess == err)
        {
            err = pFeature->RunCommand();
        }
    }
    else if(VmbErrorSuccess == err)
    {
        err = pFeature->SetValue( m_nIndex );
    }

    return err;
}

//Save user set to flash
VmbErrorType UserSetControl::SaveToFlash ()
{
    VmbErrorType    err = VmbErrorSuccess;

    FeaturePtr      pFeature;

    err = m_pCamera->GetFeatureByName( "UserSetSave", pFeature );
    if(VmbErrorSuccess == err)
    {
        err = pFeature->RunCommand();
    }

    return err;
}

//Load user set from flash
VmbErrorType UserSetControl::LoadFromFlash ()
{
    VmbErrorType    err = VmbErrorSuccess;

    FeaturePtr      pFeature;

    err = m_pCamera->GetFeatureByName( "UserSetLoad", pFeature );
    if(VmbErrorSuccess == err)
    {
        err = pFeature->RunCommand();
    }

    return err;
}

//Get user set operation result
VmbErrorType UserSetControl::GetOperationResult( VmbInt64_t& nResult )
{
    VmbErrorType    err = VmbErrorSuccess;

    FeaturePtr      pFeature;

    err = m_pCamera->GetFeatureByName( "UserSetOperationResult", pFeature );
    if(VmbErrorSuccess == err)
    {
        err = pFeature->GetValue( nResult );
    }

    return err;
}

//Get user set operation status
VmbErrorType UserSetControl::GetOperationStatus( VmbInt64_t& nStatus )
{
    VmbErrorType    err = VmbErrorSuccess;

    FeaturePtr      pFeature;

    err = m_pCamera->GetFeatureByName( "UserSetOperationStatus", pFeature );
    if(VmbErrorSuccess == err)
    {
        err = pFeature->GetValue( nStatus );
    }

    return err;
}


}}} // namespace AVT::VmbAPI::Examples

