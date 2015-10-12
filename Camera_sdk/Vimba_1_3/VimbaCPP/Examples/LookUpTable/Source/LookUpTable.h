/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        LookUpTable.h

  Description: The LookUpTable example will demonstrate how to use
               the look up table feature of the camera using VimbaCPP.


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

#ifndef AVT_VMBAPI_EXAMPLES_LOOKUPTABLE
#define AVT_VMBAPI_EXAMPLES_LOOKUPTABLE

#include "VimbaCPP/Include/VimbaCPP.h"


namespace AVT {
namespace VmbAPI {
namespace Examples {

class LookUpTableControl
{
private:
    CameraPtr       m_pCamera;
    VmbInt64_t      m_nIndex;
    UcharVector     m_data;

public:
    //ctor
    LookUpTableControl( CameraPtr pCamera, VmbInt64_t nIndex );

    //Enable
    void Enable( VmbBool_t bEnable );

    //Is enabled
    VmbBool_t IsEnabled() const;

    //Get index
    VmbInt64_t GetIndex( ) const;

    //Get value count
    VmbInt64_t GetValueCount( ) const;

    //Get Value
    VmbInt64_t GetValue( ) const;

    //Set Value
    void SetValue( VmbInt64_t nValue );

    //Get bit depth in
    VmbInt64_t GetBitDepthIn( ) const;

    //Get bit depth out
    VmbInt64_t GetBitDepthOut( ) const;

    //Download look up table from internal memory into the camera
    void Download();

    //Upload the selected look up table into internal memory of the camera
    void Upload();


    //Load from Csv
    void LoadFromCsv( const std::string &FileName, int nIndex );

    //Save to Csv
    void SaveToCsv( const std::string &FileName );

    //Load from flash
    void LoadFromFlash();

    //Save to flash
    void SaveToFlash();
};

class LookUpTableCollection
{
private:
    CameraPtr                        m_pCamera;
    std::vector<LookUpTableControl>  m_tables;

public:
    //ctor
    LookUpTableCollection( CameraPtr pCamera );

    //Get count
    VmbInt64_t GetCount( );

    // Get control
    LookUpTableControl GetControl( VmbInt64_t nIndex );

    //Get active index
    VmbInt64_t GetActiveIndex( );
};

}}} // namespace AVT::Vimba::Examples

#endif
