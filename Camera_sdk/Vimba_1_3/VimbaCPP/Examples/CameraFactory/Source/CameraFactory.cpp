/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        CameraFactory.cpp

  Description: The CameraFactory example will create a suitable object for
               each known interface. The user can create his own factory and 
               camera classes for customisation.

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

#include "CameraFactory.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

/*=============================================================================
    FirewireCamera class
=============================================================================*/

    FirewireCamera::FirewireCamera(     const char         *pCameraID,
                                        const char         *pCameraName,
                                        const char         *pCameraModel,
                                        const char         *pCameraSerialNumber,
                                        const char         *pInterfaceID,
                                        VmbInterfaceType    interfaceType,
                                        const char         *pInterfaceName,
                                        const char         *pInterfaceSerialNumber,
                                        VmbAccessModeType   interfacePermittedAccess )
        : Camera( pCameraID, pCameraName, pCameraModel, 
                  pCameraSerialNumber, pInterfaceID, interfaceType)
    {
    }
    
    // advanced camera function
    void FirewireCamera::addonFireWire( std::string &info )
    {
        info = "1394 interface connection detected";
    }

/*=============================================================================
    GigECamera class
=============================================================================*/

    GigECamera::GigECamera(     const char         *pCameraID,
                                const char         *pCameraName,
                                const char         *pCameraModel,
                                const char         *pCameraSerialNumber,
                                const char         *pInterfaceID,
                                VmbInterfaceType    interfaceType,
                                const char         *pInterfaceName,
                                const char         *pInterfaceSerialNumber,
                                VmbAccessModeType   interfacePermittedAccess )
        : Camera( pCameraID, pCameraName, pCameraModel, 
                  pCameraSerialNumber, pInterfaceID, interfaceType)
    {
    }

    // advanced camera function
    void GigECamera::addonGigE( std::string &info )
    {
        info = "ethernet interface connection detected";
    }

/*=============================================================================
    USBCamera class
=============================================================================*/

    USBCamera::USBCamera(      const char         *pCameraID,
                               const char         *pCameraName,
                               const char         *pCameraModel,
                               const char         *pCameraSerialNumber,
                               const char         *pInterfaceID,
                               VmbInterfaceType    interfaceType,
                               const char         *pInterfaceName,
                               const char         *pInterfaceSerialNumber,
                               VmbAccessModeType   interfacePermittedAccess )
        : Camera( pCameraID, pCameraName, pCameraModel, 
                  pCameraSerialNumber, pInterfaceID, interfaceType)
    {
    }
    
    // advanced camera function
    void USBCamera::addonUSB( std::string &info )
    {
        info = "usb interface connection detected";
    }

/*=============================================================================
    UserCameraFactory class
=============================================================================*/

    CameraPtr UserCameraFactory::CreateCamera(  const char *pCameraID,
                                                const char *pCameraName,
                                                const char *pCameraModel,
                                                const char *pCameraSerialNumber,
                                                const char *pInterfaceID,
                                                VmbInterfaceType interfaceType,
                                                const char *pInterfaceName,
                                                const char *pInterfaceSerialNumber,
                                                VmbAccessModeType interfacePermittedAccess)
    {
        // create camera class, depending on camera interface type
        if(VmbInterfaceFirewire == interfaceType)
        {
            return FirewireCamera_t( new FirewireCamera(pCameraID, 
                                         pCameraName, 
                                         pCameraModel, 
                                         pCameraSerialNumber, 
                                         pInterfaceID, 
                                         interfaceType, 
                                         pInterfaceName, 
                                         pInterfaceSerialNumber, 
                                         interfacePermittedAccess));
        }
        else if(VmbInterfaceEthernet == interfaceType)
        {
            return GigECamera_t( new GigECamera(pCameraID, 
                                                pCameraName, 
                                                pCameraModel, 
                                                pCameraSerialNumber, 
                                                pInterfaceID, 
                                                interfaceType, 
                                                pInterfaceName, 
                                                pInterfaceSerialNumber, 
                                                interfacePermittedAccess));
        }
        else if(VmbInterfaceUsb == interfaceType)
        {
            return USBCamera_t( new USBCamera(pCameraID, 
                                    pCameraName, 
                                    pCameraModel, 
                                    pCameraSerialNumber, 
                                    pInterfaceID, 
                                    interfaceType, 
                                    pInterfaceName, 
                                    pInterfaceSerialNumber, 
                                    interfacePermittedAccess));
        }
        else // unknown camera interface
        {
            // use default camera class
            return Camera_t( new Camera(pCameraID, 
                                 pCameraName, 
                                 pCameraModel, 
                                 pCameraSerialNumber, 
                                 pInterfaceID, 
                                 interfaceType));
        }
    }

}}} // namespace AVT::Vimba::Examples

