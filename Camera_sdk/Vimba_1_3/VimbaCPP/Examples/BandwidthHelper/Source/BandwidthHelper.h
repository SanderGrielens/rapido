/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        BandwidthHelper.h

  Description: The BandwidthHelper example demonstrates how to get and set the
               bandwidth used by a camera using VimbaCPP.

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

#ifndef AVT_VMBAPI_HELPER_BANDWIDTHHELPER_H
#define AVT_VMBAPI_HELPER_BANDWIDTHHELPER_H

#include "VimbaCPP/Include/VimbaCPP.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

class BandwidthHelper
{
  public:
    static VmbErrorType GetBandwidthUsage( CameraPtr pCamera, double &bandwidth );
    static VmbErrorType SetBandwidthUsage( CameraPtr pCamera, double bandwidth );
    static VmbErrorType GetMinPossibleBandwidthUsage( CameraPtr pCamera, double &bandwidth );
    static VmbErrorType GetMaxPossibleBandwidthUsage( CameraPtr pCamera, double &bandwidth );

    static std::string InterfaceToString( VmbInterfaceType interfaceType );

  private:
    // No default ctor
    BandwidthHelper();
};

}}} // AVT:VmbAPI::Examples

#endif
