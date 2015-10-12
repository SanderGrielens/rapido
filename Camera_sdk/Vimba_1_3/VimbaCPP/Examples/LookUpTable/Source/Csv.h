/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        Csv.h

  Description: Helper to access a CSV file using VimbaCPP.


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

#ifndef AVT_VMBAPI_EXAMPLES_LOOKUPTABLE_CSV
#define AVT_VMBAPI_EXAMPLES_LOOKUPTABLE_CSV

#include <iostream>
#include <fstream>

#include "VimbaCPP/Include/VimbaCPP.h"


namespace AVT {
namespace VmbAPI {
namespace Examples {

class Csv
{
private:
    char m_VD;
    char m_RD;
    bool m_bHasHeader;

public:
    //ctor
    Csv()
        : m_VD( ';' )
        , m_RD( '\n' )
        , m_bHasHeader( false )
    {   }
    char VD( )                          const   { return m_VD; }
    void VD( char vd )                          { m_VD = vd;}
    char RD ( )                         const   { return m_RD; }
    bool HasHeader()                    const   { return m_bHasHeader; }
    void HasHeader( bool bHasHeader )           { m_bHasHeader = bHasHeader; }

};

class CsvLoad
{
private:
    std::ifstream   m_Stream;
    Csv             m_CsvParameter;
public:
    //ctor
    CsvLoad( const std::string &filename );
    bool  Row( std::vector<std::string> &row );
};

class CsvSave
{
private:
    std::ofstream   m_Stream;
    Csv             m_CsvParameter;
public:
    //ctor
    CsvSave( const std::string &filename );

    bool Row( const std::string &text );
};


}}} // namespace AVT::Vimba::Examples

#endif