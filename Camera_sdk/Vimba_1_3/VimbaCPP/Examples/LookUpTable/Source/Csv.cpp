/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        Csv.cpp

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

#include <vector>

#include "Csv.h"

#include "VimbaCPP/Include/VimbaCPP.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {


// Class to write data to a CSV file
CsvSave::CsvSave( const std::string &FileName )
    :m_Stream( FileName.c_str() )
{  }

// Writes a single row to a CSV file.
bool CsvSave::Row( const std::string &text )
{
    if( 0 == text.size() )
    {
        return false;
    }
    std::string line = text + m_CsvParameter.RD() ;
    m_Stream.write( line.c_str(), line.length() );
    return true;
}

// Class to load data from CSV
CsvLoad::CsvLoad( const std::string &filename )
    : m_Stream( filename.c_str() )
{}

// Reads a row of data from CSV
bool CsvLoad::Row( std::vector<std::string> &row )
{
    row.clear();
    char pLineText[256];
    if( m_Stream.getline( pLineText, 256, m_CsvParameter.RD() ) )
    {

        std::string lineText = std::string( pLineText );
        if( 0 == lineText.size() )
        {
            return false;
        }
        typedef std::string::size_type pos_type;
        pos_type pos = 0;
        while( pos != std::string::npos )
        {
            pos_type start = pos;
            pos = lineText.find( m_CsvParameter.VD(), start );
            const std::string value = lineText.substr( start, pos );
            row.push_back( value );
            if( pos != std::string::npos )
            {
                pos++;
            }
        }
    }
    else
    {
        return false;
    }

    return ( row.size() > 0 );
}

}}} // namespace AVT::VmbAPI::Examples