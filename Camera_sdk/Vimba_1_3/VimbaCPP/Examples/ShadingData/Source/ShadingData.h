/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ShadingData.h

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

#ifndef AVT_VMBAPI_EXAMPLES_SHADINGDATA
#define AVT_VMBAPI_EXAMPLES_SHADINGDATA

#include "VimbaCPP/Include/VimbaCPP.h"


namespace AVT {
namespace VmbAPI {
namespace Examples {

class ShadingDataControl
{
private:
    static CameraPtr    m_pCamera;
    static UcharVector  m_data;

public:

    //ctor
    ShadingDataControl ( CameraPtr pCamera );

    //Is enabled
    static VmbErrorType IsEnabled (  bool &bEnabled );

    //Enable
    static VmbErrorType Enable (  bool bEnable );

    //Is data shown
    static VmbErrorType IsDataShown (  bool &bIsDataShown );

    //Show data
    static VmbErrorType ShowData (  bool bShowData );

    //Upload
    static VmbErrorType Upload ();

    //Download
    static VmbErrorType Download ();

    //Get data
    static VmbErrorType GetRawData ( UcharVector& data );

    //Set data
    static VmbErrorType SetRawData ( UcharVector data );

    //Set Build images
    static VmbErrorType BuildImages (  VmbInt64_t nImageCount );

    //Load from flash
    static VmbErrorType LoadFromFlash ();

    //Save to flash
    static VmbErrorType SaveToFlash ();

    //Clear flash
    static VmbErrorType ClearFlash ();

};

}}} // namespace AVT::Vimba::Examples

#endif
