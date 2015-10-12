/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        BandwidthHelper.cpp

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

#include <stdlib.h>

#include "BandwidthHelper.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

#define PACKET_SIZE_MAX_1394_S100   1024
#define PACKET_SIZE_MAX_1394_S200   2048
#define PACKET_SIZE_MAX_1394_S400   4096
#define PACKET_SIZE_MAX_1394_S800   8192

VmbErrorType BandwidthHelper::GetBandwidthUsage( CameraPtr pCamera, double &rfBandwidth )
{
    VmbErrorType        res;
    VmbInt64_t          nValue;
    FeaturePtr          pFeature;
    InterfacePtr        pInterface;
    VmbInterfaceType    interfaceType;
    std::string         strInterfaceID;
    VimbaSystem &       system          = VimbaSystem::GetInstance();

    res = pCamera->GetInterfaceID( strInterfaceID );
    if( VmbErrorSuccess == res )
    {
        res = system.GetInterfaceByID( strInterfaceID.c_str(), pInterface );
        if( VmbErrorSuccess == res )
        {
            res = pInterface->GetType( interfaceType );
            if( VmbErrorSuccess == res )
            {
                switch( interfaceType )
                {
                    case VmbInterfaceEthernet:
                        res = pCamera->GetFeatureByName( "StreamBytesPerSecond", pFeature );
                        if ( VmbErrorSuccess == res )
                        {
                            res = pFeature->GetValue( nValue );
                            if ( VmbErrorSuccess == res )
                            {
                                VmbInt64_t nMin, nMax;
                                res = pFeature->GetRange( nMin, nMax );
                                if ( VmbErrorSuccess == res )
                                {
                                    rfBandwidth = (double)nValue / nMax;
                                }
                            }
                        }
                        break;

                    case VmbInterfaceFirewire:
                        res = pCamera->GetFeatureByName( "IIDCPacketSize", pFeature );
                        if ( VmbErrorSuccess == res )
                        {
                            res = pFeature->GetValue( nValue );
                            if ( VmbErrorSuccess == res )
                            {
                                res = pCamera->GetFeatureByName( "IIDCPhyspeed", pFeature );
                                if ( VmbErrorSuccess == res )
                                {
                                    std::string strPhySpeed;
                                    res = pFeature->GetValue( strPhySpeed );
                                    if ( VmbErrorSuccess == res )
                                    {
                                        int nPhySpeed = atoi( strPhySpeed.substr( 1 ).c_str() );
                                        switch ( nPhySpeed )
                                        {
                                            case 100 : nPhySpeed = PACKET_SIZE_MAX_1394_S100;
                                                break;
                                            case 200 : nPhySpeed = PACKET_SIZE_MAX_1394_S200;
                                                break;
                                            case 400 : nPhySpeed = PACKET_SIZE_MAX_1394_S400;
                                                break;
                                            case 800 : nPhySpeed = PACKET_SIZE_MAX_1394_S800;
                                                break;
                                            default: return VmbErrorInternalFault;
                                        }
                                        rfBandwidth = (double)nValue / (double)nPhySpeed;
                                    }
                                }
                            }
                        }
                        break;

                    default:
                        res = VmbErrorWrongType;
                        break;
                }
            }
        }
    }

    return res;
}

VmbErrorType BandwidthHelper::SetBandwidthUsage( CameraPtr pCamera, double fBandwidth )
{
    VmbErrorType        res;
    VmbInt64_t          nValue;
    FeaturePtr          pFeature;
    InterfacePtr        pInterface;
    VmbInterfaceType    interfaceType;
    std::string         strInterfaceID;
    VimbaSystem&        system          = VimbaSystem::GetInstance();

    res = pCamera->GetInterfaceID( strInterfaceID );
    if( VmbErrorSuccess == res )
    {
        res = system.GetInterfaceByID( strInterfaceID.c_str(), pInterface );
        if( VmbErrorSuccess == res )
        {
            res = pInterface->GetType( interfaceType );
            if( VmbErrorSuccess == res )
            {
                switch( interfaceType )
                {
                    case VmbInterfaceEthernet:
                        res = pCamera->GetFeatureByName( "StreamBytesPerSecond", pFeature );
                        if ( VmbErrorSuccess == res )
                        {
                            VmbInt64_t nMin, nMax;
                            res = pFeature->GetRange( nMin, nMax );
                            if ( VmbErrorSuccess == res )
                            {
                                nValue = (VmbUint64_t)(fBandwidth * nMax);
                                res = pFeature->SetValue( nValue );
                            }
                        }
                    break;
                    case VmbInterfaceFirewire:
                        res = pCamera->GetFeatureByName( "IIDCPacketSizeAuto", pFeature );
                        if ( VmbErrorSuccess == res )
                        {
                            res = pFeature->SetValue( "Off" );
                            if ( VmbErrorSuccess == res )
                            {
                                res = pCamera->GetFeatureByName( "IIDCPhyspeed", pFeature );
                                if ( VmbErrorSuccess == res )
                                {
                                    std::string strPhySpeed;
                                    res = pFeature->GetValue( strPhySpeed );
                                    if ( VmbErrorSuccess == res )
                                    {
                                        int nPhySpeed = atoi( strPhySpeed.substr( 1 ).c_str() );
                                        switch ( nPhySpeed )
                                        {
                                            case 100 : nPhySpeed = PACKET_SIZE_MAX_1394_S100;
                                                break;
                                            case 200 : nPhySpeed = PACKET_SIZE_MAX_1394_S200;
                                                break;
                                            case 400 : nPhySpeed = PACKET_SIZE_MAX_1394_S400;
                                                break;
                                            case 800 : nPhySpeed = PACKET_SIZE_MAX_1394_S800;
                                                break;
                                            default: return VmbErrorInternalFault;
                                        }
                                        // Set size to new percentage
                                        nValue = (VmbUint64_t)(fBandwidth * nPhySpeed);
                                        res = pCamera->GetFeatureByName( "IIDCPacketSize", pFeature );
                                        if ( VmbErrorSuccess == res )
                                        {
                                            // Adjust new value to fit increment
                                            VmbInt64_t nInc;
                                            res = pFeature->GetIncrement( nInc );
                                            if ( VmbErrorSuccess == res )
                                            {
                                                nValue -= (nValue % nInc);
                                                // Write new value
                                                res = pFeature->SetValue( nValue );
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    break;
                    default:
                        res = VmbErrorWrongType;
                    break;
                }
            }
        }
    }

    return res;
}

VmbErrorType BandwidthHelper::GetMinPossibleBandwidthUsage( CameraPtr pCamera, double &rfBandwidth )
{
    VmbErrorType        res;
    VmbInt64_t          nMinValue;
    VmbInt64_t          nMaxValue;
    FeaturePtr          pFeature;
    InterfacePtr        pInterface;
    VmbInterfaceType    interfaceType;
    std::string         strInterfaceID;
    VimbaSystem &       system          = VimbaSystem::GetInstance();

    res = pCamera->GetInterfaceID( strInterfaceID );
    if( VmbErrorSuccess == res )
    {
        res = system.GetInterfaceByID( strInterfaceID.c_str(), pInterface );
        if( VmbErrorSuccess == res )
        {
            res = pInterface->GetType( interfaceType );
            if( VmbErrorSuccess == res )
            {
                switch( interfaceType )
                {
                    case VmbInterfaceEthernet:
                        res = pCamera->GetFeatureByName( "StreamBytesPerSecond", pFeature );
                        if ( VmbErrorSuccess == res )
                        {
                            res = pFeature->GetRange( nMinValue, nMaxValue );
                            if ( VmbErrorSuccess == res )
                            {
                                rfBandwidth = (double)nMinValue / nMaxValue;
                            }
                        }
                        break;

                    case VmbInterfaceFirewire:
                        res = pCamera->GetFeatureByName( "IIDCPacketSize", pFeature );
                        if ( VmbErrorSuccess == res )
                        {
                            res = pFeature->GetRange( nMinValue, nMaxValue );
                            if ( VmbErrorSuccess == res )
                            {
                                res = pCamera->GetFeatureByName( "IIDCPhyspeed", pFeature );
                                if ( VmbErrorSuccess == res )
                                {
                                    std::string strPhySpeed;
                                    res = pFeature->GetValue( strPhySpeed );
                                    if ( VmbErrorSuccess == res )
                                    {
                                        int nPhySpeed = atoi( strPhySpeed.substr( 1 ).c_str() );
                                        switch ( nPhySpeed )
                                        {
                                            case 100 : nPhySpeed = PACKET_SIZE_MAX_1394_S100;
                                                break;
                                            case 200 : nPhySpeed = PACKET_SIZE_MAX_1394_S200;
                                                break;
                                            case 400 : nPhySpeed = PACKET_SIZE_MAX_1394_S400;
                                                break;
                                            case 800 : nPhySpeed = PACKET_SIZE_MAX_1394_S800;
                                                break;
                                            default: return VmbErrorInternalFault;
                                        }
                                        rfBandwidth = (double)nMinValue / (double)nPhySpeed;
                                    }
                                }
                            }
                        }
                        break;

                    default:
                        res = VmbErrorWrongType;
                        break;
                }
            }
        }
    }

    return res;
}

VmbErrorType BandwidthHelper::GetMaxPossibleBandwidthUsage( CameraPtr pCamera, double &rfBandwidth )
{
    VmbErrorType        res;
    VmbInt64_t          nMinValue;
    VmbInt64_t          nMaxValue;
    FeaturePtr          pFeature;
    InterfacePtr        pInterface;
    VmbInterfaceType    interfaceType;
    std::string         strInterfaceID;
    VimbaSystem &       system          = VimbaSystem::GetInstance();

    res =  pCamera->GetInterfaceID( strInterfaceID );
    if( VmbErrorSuccess == res )
    {
        res = system.GetInterfaceByID( strInterfaceID.c_str(), pInterface );
        if( VmbErrorSuccess == res )
        {
            res = pInterface->GetType( interfaceType );
            if( VmbErrorSuccess == res )
            {
                switch ( interfaceType )
                {
                    case VmbInterfaceEthernet:
                        rfBandwidth = 1.0;
                        break;

                    case VmbInterfaceFirewire:
                        res = pCamera->GetFeatureByName( "IIDCPacketSize", pFeature );
                        if ( VmbErrorSuccess == res )
                        {
                            res = pFeature->GetRange( nMinValue, nMaxValue );
                            if ( VmbErrorSuccess == res )
                            {
                                res = pCamera->GetFeatureByName( "IIDCPhyspeed", pFeature );
                                if ( VmbErrorSuccess == res )
                                {
                                    std::string strPhySpeed;
                                    res = pFeature->GetValue( strPhySpeed );
                                    if ( VmbErrorSuccess == res )
                                    {
                                        int nPhySpeed = atoi( strPhySpeed.substr( 1 ).c_str() );
                                        switch ( nPhySpeed )
                                        {
                                            case 100 : nPhySpeed = PACKET_SIZE_MAX_1394_S100;
                                                break;
                                            case 200 : nPhySpeed = PACKET_SIZE_MAX_1394_S200;
                                                break;
                                            case 400 : nPhySpeed = PACKET_SIZE_MAX_1394_S400;
                                                break;
                                            case 800 : nPhySpeed = PACKET_SIZE_MAX_1394_S800;
                                                break;
                                            default: return VmbErrorInternalFault;
                                        }
                                        rfBandwidth = (double)nMaxValue / (double)nPhySpeed;
                                    }
                                }
                            }
                        }
                        break;

                    default:
                        res = VmbErrorWrongType;
                        break;
                }
            }
        }
    }

    return res;
}

std::string BandwidthHelper::InterfaceToString( VmbInterfaceType interfaceType )
{
    switch ( interfaceType )
    {
        case VmbInterfaceFirewire:  return "FireWire";
        case VmbInterfaceEthernet:  return "GigE";
        case VmbInterfaceUsb:       return "USB";
        default:                    return "Unknown";
    }
}

}}} // AVT:VmbAPI::Examples
