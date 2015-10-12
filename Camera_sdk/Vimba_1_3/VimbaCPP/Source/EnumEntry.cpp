/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        EnumEntry.cpp

  Description: Implementation of class AVT::VmbAPI::EnumEntry.

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

#include <VimbaCPP/Include/EnumEntry.h>

namespace AVT {
namespace VmbAPI {

EnumEntry::EnumEntry(   const char              *pStrName,
                        const char              *pStrDisplayName,
                        const char              *pStrDescription,
                        const char              *pStrTooltip,
                        const char              *pStrSNFCNamespace,
                        VmbFeatureVisibility_t  visibility,
                        VmbInt64_t              nValue)
    : m_nValue( nValue )
    , m_Visibility ( (VmbFeatureVisibilityType)visibility )
{
    m_strName = pStrName != NULL ? std::string( pStrName ) : "";
    m_strDisplayName = pStrDisplayName != NULL ? std::string( pStrDisplayName ) : "";
    m_strDescription = pStrDescription != NULL ? std::string( pStrDescription ) : "";
    m_strTooltip = pStrTooltip != NULL ? std::string( pStrTooltip ) : "";
    m_strNamespace = pStrSNFCNamespace != NULL ? std::string( pStrSNFCNamespace ) : "";
}

EnumEntry::EnumEntry()
{
    // No default ctor
}

EnumEntry::~EnumEntry()
{
}

VmbErrorType EnumEntry::GetName( char * const pStrName, VmbUint32_t &rnSize ) const
{
    VmbErrorType res;

    if ( NULL == pStrName )
    {
        rnSize = (VmbUint32_t)m_strName.size();
        res = VmbErrorSuccess;
    }
    else if ( m_strName.size() <= rnSize )
    {
        std::copy( m_strName.begin(), m_strName.end(), pStrName );
        rnSize = (VmbUint32_t)m_strName.size();
        res = VmbErrorSuccess;
    }
    else
    {
        res = VmbErrorMoreData;
    }

    return res;
}

VmbErrorType EnumEntry::GetDisplayName( char * const pStrDisplayName, VmbUint32_t &rnSize ) const
{
    VmbErrorType res;

    if ( NULL == pStrDisplayName )
    {
        rnSize = (VmbUint32_t)m_strDisplayName.size();
        res = VmbErrorSuccess;
    }
    else if ( m_strDisplayName.size() <= rnSize )
    {
        std::copy( m_strDisplayName.begin(), m_strDisplayName.end(), pStrDisplayName );
        rnSize = (VmbUint32_t)m_strDisplayName.size();
        res = VmbErrorSuccess;
    }
    else
    {
        res = VmbErrorMoreData;
    }

    return res;
}

VmbErrorType EnumEntry::GetDescription( char * const pStrDescription, VmbUint32_t &rnSize ) const
{
    VmbErrorType res;

    if ( NULL == pStrDescription )
    {
        rnSize = (VmbUint32_t)m_strDescription.size();
        res = VmbErrorSuccess;
    }
    else if ( m_strDescription.size() <= rnSize )
    {
        std::copy( m_strDescription.begin(), m_strDescription.end(), pStrDescription );
        rnSize = (VmbUint32_t)m_strDescription.size();
        res = VmbErrorSuccess;
    }
    else
    {
        res = VmbErrorMoreData;
    }

    return res;
}

VmbErrorType EnumEntry::GetTooltip( char * const pStrTooltip, VmbUint32_t &rnSize ) const
{
    VmbErrorType res;

    if ( NULL == pStrTooltip )
    {
        rnSize = (VmbUint32_t)m_strTooltip.size();
        res = VmbErrorSuccess;
    }
    else if ( m_strTooltip.size() <= rnSize )
    {
        std::copy( m_strTooltip.begin(), m_strTooltip.end(), pStrTooltip );
        rnSize = (VmbUint32_t)m_strTooltip.size();
        res = VmbErrorSuccess;
    }
    else
    {
        res = VmbErrorMoreData;
    }

    return res;
}

VmbErrorType EnumEntry::GetSFNCNamespace( char * const pStrNamespace, VmbUint32_t &rnSize ) const
{
    VmbErrorType res;

    if ( NULL == pStrNamespace )
    {
        rnSize = (VmbUint32_t)m_strNamespace.size();
        res = VmbErrorSuccess;
    }
    else if ( m_strNamespace.size() <= rnSize )
    {
        std::copy( m_strNamespace.begin(), m_strNamespace.end(), pStrNamespace );
        rnSize = (VmbUint32_t)m_strNamespace.size();
        res = VmbErrorSuccess;
    }
    else
    {
        res = VmbErrorMoreData;
    }

    return res;
}

VmbErrorType EnumEntry::GetValue( VmbInt64_t &rnValue ) const
{
    rnValue = m_nValue;

    return VmbErrorSuccess;
}

VmbErrorType EnumEntry::GetVisibility( VmbFeatureVisibilityType &rVisibility ) const
{
    rVisibility = m_Visibility;

    return VmbErrorSuccess;
}


}} // namespace AVT::VmbAPI
