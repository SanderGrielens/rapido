/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        UserSet.h

  Description: The UserSet example will demonstrate how deal with the user sets
               stored inside the cameras.
               

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

#ifndef AVT_VMBAPI_EXAMPLES_USERSET
#define AVT_VMBAPI_EXAMPLES_USERSET

#include "VimbaCPP/Include/VimbaCPP.h"


namespace AVT {
namespace VmbAPI {
namespace Examples {

class UserSetControl
{
private:
    CameraPtr   m_pCamera;
    VmbInt64_t  m_nIndex;

public:
    
    //ctor
    UserSetControl( CameraPtr pCamera, VmbInt64_t nIndex );

    //Is user set default
    VmbErrorType IsDefault ( bool& bIsDefault );

    //Make user set default
    VmbErrorType MakeDefault ();

    //Save current camera settings to user set
    VmbErrorType SaveToFlash ();

    //Load settings from user set into the camera
    VmbErrorType LoadFromFlash ();

    //Get operation result
    VmbErrorType GetOperationResult( VmbInt64_t& nResult );

    //Get operation status
    VmbErrorType GetOperationStatus( VmbInt64_t& nStatus );
};

class UserSetCollection
{
private:
    static CameraPtr                    m_pCamera;
    static std::vector<UserSetControl>  m_usersets;

public:
    //ctor
    UserSetCollection( CameraPtr pCamera );

    //Get count
    static VmbErrorType GetCount ( VmbInt64_t& nCount );
    
    //Get control
    static VmbErrorType GetControl ( VmbInt64_t nIndex, UserSetControl& control );

    //Get selected index
    static VmbErrorType GetSelectedIndex ( VmbInt64_t& nIndex );

    //Get default index
    static VmbErrorType GetDefaultIndex ( VmbInt64_t& nDefaultIndex );
};

}}} // namespace AVT::Vimba::Examples

#endif
