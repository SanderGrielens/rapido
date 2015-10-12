/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        Exception.h

  Description: Helper for exceptions.


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

#ifndef AVT_VMBAPI_EXAMPLES_EXCEPTION
#define AVT_VMBAPI_EXAMPLES_EXCEPTION

#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/ErrorCodeToMessage.h"
using namespace std;

namespace AVT {
namespace VmbAPI {
namespace Examples {

class Exception: public exception
{
private:
    std::string m_sMsg;
    // Vimba error code
    VmbErrorType      m_eError;
public:
    Exception( const std::string &rsMessage, const VmbErrorType eError )
    : m_eError(eError)
    {
        try
        {
            m_sMsg = rsMessage;
        } catch(...){}
    }
    Exception( const std::string &Fun, const std::string &rsMessage, const VmbErrorType eError )
    : m_eError(eError)
    {
        try
        {
            m_sMsg =  Fun + ": " + rsMessage;
        } catch(...){}
    }

    virtual ~Exception() throw()
    {
    }

    VmbErrorType GetError() const
    {
        return m_eError;
    }

    std::string GetMessageStr() const
    {
        return m_sMsg;
    }

    // Translates Vimba error codes to readable error messages
    std::string ErrorCodeToMessage( VmbErrorType eErr )
    {
        return AVT::VmbAPI::Examples::ErrorCodeToMessage( eErr );
    }
};

}}} // namespace AVT::VmbAPI::Examples

#endif