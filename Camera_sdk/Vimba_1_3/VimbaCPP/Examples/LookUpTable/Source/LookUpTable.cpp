/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        LookUpTable.cpp

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

#include <string>
#include <fstream>
#include <sstream>
#include <iterator>
#include <vector>
#include <algorithm>

#include "LookUpTable.h"
#include "Csv.h"
#include "Exception.h"
#include "VimbaFeatures.h"
#include "VimbaCPP/Include/VimbaCPP.h"



namespace AVT {
namespace VmbAPI {
namespace Examples {

namespace
{   // toolset to handle network byte order, for production, use htons ntohs for int16
    static const union
    {
        VmbUint8_t  bytes[4];
        VmbUint32_t v;
    } host_byte_order = {{0,1,2,3}};
    enum 
    {
        byte_oder_little_endian = 0x03020100ul,
        byte_oder_big_endian    = 0x01000302ul,
    };

    inline UcharVector ConvertToNetworkInt16( const UcharVector &d)
    {
        if (host_byte_order.v == byte_oder_little_endian)
        {
            UcharVector tmp( d );
            for( size_t i = 0 ;i < tmp.size(); i +=2)
            {
                VmbUint16_t *p = reinterpret_cast<VmbUint16_t*>( &tmp[i] );
                *p = ((*p>>8) & 0xff) | ((*p<<8) & 0xff00);
            }
            return tmp;
        }
        return d;
    }
    inline UcharVector ConvertFromNetworkInt16( const UcharVector &d)
    {
        return ConvertToNetworkInt16( d );
    }
}
LookUpTableCollection::LookUpTableCollection( CameraPtr pCamera )
    : m_pCamera( pCamera )
{
    //Check parameters
    if( NULL == m_pCamera )
    {
        throw Exception( __FUNCTION__,"LookUpTableCollection failed camera pointer NULL.", VmbErrorBadParameter );
    }


    VmbInt64_t nCount = GetCount( );
    if( 0 == nCount )
    {
        throw Exception( __FUNCTION__,"LookUpTableCollection no entries.", VmbErrorNotFound );
    }
    // for each lookup table create a control
    for( int i = 0; i < nCount; i++ )
    {
        m_tables.push_back( LookUpTableControl( m_pCamera, i ));
    }
}

//Get look up table count
VmbInt64_t LookUpTableCollection::GetCount( )
{
    // number of LUTs is the number of enum values fo the "LUTSelector" Feature
    VimbaFeatureEnum LutSelector( m_pCamera, "LUTSelector" );
    return LutSelector.ValueCount();
}

//Get look up table control
LookUpTableControl  LookUpTableCollection::GetControl( VmbInt64_t nIndex )
{
    VimbaFeatureEnum    LutSelector( m_pCamera, "LUTSelector" );                            //< get enum feature for supported LUTs
    StringVector        Values = LutSelector.GetValues();                                   //< get list of supported LUTs
    if( static_cast<size_t>(nIndex) >= Values.size() )                                                           //< test index for out of range
    {
        throw Exception( __FUNCTION__,"selected index out of range",VmbErrorBadParameter );
    }
    std::string SelectedValue = Values[ static_cast<size_t>( nIndex ) ];
    if( !LutSelector.IsAvailable( SelectedValue ) )
    {
        throw Exception(__FUNCTION__,"selected LUT is not available", VmbErrorBadParameter );
    }
    LutSelector.SetStringValue( SelectedValue );                  //< activate the LUT at nIndex

    return m_tables[ static_cast<size_t>( nIndex ) ];                                       //< get stored LUT control
}

//Get look up table active index
VmbInt64_t LookUpTableCollection::GetActiveIndex( )
{
    VimbaFeatureEnum    LutSelector( m_pCamera, "LUTSelector" );                                                //< get enum feture for supported luts
    const StringVector  SupportedLuts   = LutSelector.GetValues();                                              //< get list of supported LUTs
    std::string         CurrentLut      = LutSelector.GetStringValue( );                                        //< get the currently selected LUT
    StringVector::const_iterator find_pos = std::find( SupportedLuts.begin(), SupportedLuts.end(), CurrentLut );//< find current vallue in list
    if( SupportedLuts.end() == find_pos )
    {
        throw Exception( __FUNCTION__,"could not find current selected lut index",VmbErrorNotFound );
    }
    return std::distance( SupportedLuts.begin(), find_pos );
}

LookUpTableControl::LookUpTableControl( CameraPtr pCamera, VmbInt64_t nIndex )
    :   m_pCamera( pCamera )
    ,   m_nIndex( nIndex )

{
    //Check parameters
    if( NULL == m_pCamera )
    {
        throw Exception( __FUNCTION__,"LookUpTableControl failed.", VmbErrorBadParameter );
    }
}

//Enable look up table
void LookUpTableControl::Enable( VmbBool_t bEnable )
{
    VimbaFeatureBool LutEnable( m_pCamera, "LUTEnable" );   //< create bool feature for lut enable
    LutEnable( bEnable );                                   //< set state to feature
}

//Is look up table enabled
VmbBool_t LookUpTableControl::IsEnabled( ) const
{
    VimbaFeatureBool LutEnable( m_pCamera,"LUTEnable" );    //< create bool feature for Lut enable
    return LutEnable();                                     //< return current value
}

//Get look up table index
VmbInt64_t LookUpTableControl::GetIndex( ) const
{
    VimbaFeatureInt LutIndex( m_pCamera, "LUTIndex" );      //< get feature for LUT index
    return LutIndex();                                      // return current index
}

//Get value count
VmbInt64_t LookUpTableControl::GetValueCount( ) const
{
    VimbaFeatureInt LutSize                 ( m_pCamera,"LUTSize");             //< get feature for LUT size in bytes
    VimbaFeatureInt LutBitDepthOut          ( m_pCamera, "LUTBitDepthOut");     //< get feature for output bits per lut values
    VmbInt64_t      nLUTBytePerValue    =   LutBitDepthOut() > 8 ? 2 : 1;       //< calculate bytes used its either 2 bytes if larger than 8bits or 1 byte
    
    return LutSize() / nLUTBytePerValue;                                        //< lut are elements without padding consecutive in memory
}

//Get look up table value
VmbInt64_t LookUpTableControl::GetValue( ) const
{
    VimbaFeatureInt LutValue( m_pCamera, "LUTValue" );                          //< get feature for lUT element value this depends on LUTIndex
    return LutValue();                                                          //< return current value
}

//Set look up table value
void LookUpTableControl::SetValue( VmbInt64_t nValue )
{
    VimbaFeatureInt LutValue( m_pCamera,"LUTValue" );                           //< get feature for LUT element value this depends on LUTIndex
    LutValue( nValue );                                                         //< set new value
}


//Get bit depth in
VmbInt64_t LookUpTableControl::GetBitDepthIn( ) const
{
    VimbaFeatureInt LutBitDepthIn( m_pCamera, "LUTBitDepthIn" );                //< get feature for input bit depth of LUT
    return LutBitDepthIn();                                                     //<  return bit depth
}

//Get bit depth out
VmbInt64_t LookUpTableControl::GetBitDepthOut( ) const
{
    VimbaFeatureInt LutBitDepthOut( m_pCamera,"LUTBitDepthOut" );               //< get feature for output bit depth of LUT
    return LutBitDepthOut();                                                    // return bit dept
}


//Download look up table
void  LookUpTableControl::Download()
{
    // if any kind of LUT is implemented we should find the LUTSelector feature
    if( VmbBoolTrue != VimbaFeature::HasFeature( m_pCamera,"LUTSelector" ) )
    {
        throw Exception( __FUNCTION__,"no LUT feature implemented",VmbErrorNotFound );
    }
    // try reading LUT with FileAccess
    if( VmbBoolTrue == VimbaFeature::HasFeature( m_pCamera, "FileSelector" ) )
    {
        VimbaFeatureEnum        LutSelector ( m_pCamera, "LUTSelector");                        //< selector feature for lookup table
        VimbaFeatureEnum        FileSelector( m_pCamera, "FileSelector");                       //< file selector
        VimbaFeatureEnum        FileOpenMode( m_pCamera, "FileOpenMode");                       //< selector for file open mode
        FileSelector.SetStringValue ( std::string("LUT") + LutSelector.GetStringValue());       //< get the lookup table name and set it to file selector

        if( VmbBoolTrue != FileOpenMode.IsAvailable( "Read" ) )                                 //< test if LUT feature is readable, Firewire does not support LUT read
        {
            throw Exception( __FUNCTION__,"camera does not support LUT download to host computer", VmbErrorNotSupported );
        }
        FileOpenMode.SetStringValue( "Read" );                                                  //< set the open mode to read

        VimbaFeatureInt LutSize( m_pCamera,"LUTSizeBytes" );                                    //< integer feature for bytes of lookup table
        m_data.resize( static_cast<size_t>( LutSize() ) );                                      //< get the bytes

        VimbaFeatureEnum        FileOperationSelector   ( m_pCamera, "FileOperationSelector");  //< file operation selector to selector the file command
        VimbaFeatureCommand     FileOperationExecute    ( m_pCamera, "FileOperationExecute");   //< file operation command executer
        VimbaFeatureRaw         FileAccessBuffer        ( m_pCamera, "FileAccessBuffer");       //< raw data feature to access file buffer
        VimbaFeatureInt         FileAccessOffset        ( m_pCamera, "FileAccessOffset");       //< integer feature to get the data offset into the buffer
        VimbaFeatureInt         FileAccessLength        ( m_pCamera, "FileAccessLength");       //< integer feature for length of file access
        VimbaFeatureEnum        FileOperationStatus     ( m_pCamera, "FileOperationStatus");    //< enumeration value for state of file operation
        VimbaFeatureInt         FileOperationResult     ( m_pCamera, "FileOperationResult");    //< integer for file operation result
        VimbaFeatureEnum        FileStatus              ( m_pCamera, "FileStatus");             //< file state feature
        VimbaFeatureInt         FileSize                ( m_pCamera, "FileSize");               //< file size feature


        FileOperationSelector.SetStringValue( "Open" );                                         //< set open command for file access
        FileOperationExecute.Run();                                                             //< execute command

        //File size
        VmbInt64_t nFileSize = FileSize( );                                                     //< get lut file size


        FileOperationSelector.SetStringValue( "Read" );                                         //< set file operation to read

        VmbInt64_t  nFileAccessOffset = 0;                                                      //< set start offset
        VmbInt64_t  nFileAccessLength = min( LutSize(), FileAccessLength.Max() );               //< determine if we can read the lut in one go, or are limited by max access length
        UcharVector data( static_cast<size_t>( nFileAccessLength ) );                           //< prepare the temporary data buffer

        do
        {
            FileAccessLength( nFileAccessLength );                                              //< set the amount of data we want to read
            FileOperationExecute.Run();                                                         //< commit the read request


            if( FileOperationStatus.GetStringValue( ) != "Success" )                            //< test if read failed
            {
                throw Exception( __FUNCTION__,"reading LUT failed", VmbErrorOther );
            }

            FileAccessBuffer.GetBuffer( data );                                                 //< if read succeeded we can read the raw buffer
            data = ConvertFromNetworkInt16( data );
            copy( data.begin(), data.end(), m_data.begin() + ( size_t )nFileAccessOffset );     //< copy it from temp storage to internal data buffer

            nFileAccessOffset = FileAccessOffset( );                                            //< get the new file offset

            nFileAccessLength = min( nFileSize - nFileAccessOffset, FileAccessLength.Max() );   //< get the next file length
        }
        while( nFileSize != nFileAccessOffset );

        //Select close as file operation
        FileOperationSelector.SetStringValue( "Close" );                                        //< set file operation to close
        FileOperationExecute.Run();                                                             //< commit the close operation

        if( FileOperationStatus.GetStringValue() != "Success" )                                 //< test if close succeeded
        {
            throw Exception( __FUNCTION__,"lut file operation failed", VmbErrorOther );
        }
    }
    // try reading LUT address for GigE (indicator for direct memory access)
    else if( VmbBoolTrue ==  VimbaFeature::HasFeature( m_pCamera, "LUTAddress" ) )
    {
        VimbaFeatureInt     LutSize( m_pCamera,"LUTSizeBytes" );                                                    //< get feature for lut byte size
        m_data.resize( static_cast<size_t>( LutSize() ) );                                                          //< prepare storage for LUT data
        
        VimbaFeatureInt     LutAddress      (m_pCamera, "LUTAddress");                                              //< try to get the AVT LUT address feature
        VmbInt64_t          nLUTAddress     = LutAddress();                                                         //< store address
        VmbUint32_t         nCompletedReads = 0;
        VmbErrorType        err             = m_pCamera->ReadMemory( nLUTAddress, m_data, nCompletedReads );
        if( VmbErrorSuccess != err )
        {
            throw Exception( __FUNCTION__,": could not read memory from camera",err );
        }
        m_data = ConvertFromNetworkInt16( m_data );
    }
    // if non of the buffer read features are available try to read it value by value
    else
    {
        VimbaFeatureInt   LUTIndex( m_pCamera, "LUTIndex" );                            //< get the LUTIndex feature to control the index of the current value
        VimbaFeatureInt   LUTValue( m_pCamera, "LUTValue" );                            //< get the LUTValue feature to access the current value
        VimbaFeatureInt   LutSize ( m_pCamera, "LUTSizeBytes" );                        //< get the LUTSizeBytes feature to get LUT size info

        m_data.resize( static_cast<size_t>( LutSize() ) );                              //< resize storage for LUT data

        VmbInt64_t          nLUTBitDepthOut     = GetBitDepthOut( );                    //< get bits per LUT value needed

        //Evaluate number of LUT entries
        VmbInt64_t          nLUTBytePerValue    = ( nLUTBitDepthOut > 8 ) ? 2 : 1;      //< determine what size the LUT values occupy
        VmbInt64_t          nLUTEntries         = LutSize() / nLUTBytePerValue;         //< entries are stored consecutive without padding

        //Get LUT values by iteration over indexes
        int                 iter                = 0;
        for( VmbInt64_t i = 0; i < nLUTEntries ; i++ )
        {
            LUTIndex( i );                                      //< set index
            VmbInt64_t nValue = LUTValue( );                    //< get current value
            switch( nLUTBytePerValue )
            {
            case 1:
                m_data[iter++] = static_cast<VmbUint8_t>( nValue );
                break;
            case 2:
                {
                    VmbUint16_t * p = (VmbUint16_t*)( &m_data[iter] );
                    *p              = static_cast<VmbUint16_t>( nValue );
                    iter +=2;
                }
                break;
            }
        }
    }
}

//Upload look up table
void LookUpTableControl::Upload()
{
    //Look up table raw data empty
    if( m_data.empty() )
    {
        throw Exception( __FUNCTION__,": lut data vector is empty", VmbErrorOther );
    }

    if( VmbBoolTrue == VimbaFeature::HasFeature( m_pCamera, "FileSelector" ) )
    {

        //Evaluate size of LUT
        VimbaFeatureInt     LUTSize                 ( m_pCamera, "LUTSizeBytes");
        //file access control
        VimbaFeatureEnum    FileOperationSelector   ( m_pCamera, "FileOperationSelector");
        VimbaFeatureCommand FileOperationExecute    ( m_pCamera, "FileOperationExecute" );
        VimbaFeatureEnum    FileOpenMode            ( m_pCamera, "FileOpenMode" );
        VimbaFeatureRaw     FileAccessBuffer        ( m_pCamera, "FileAccessBuffer" );
        VimbaFeatureInt     FileAccessOffset        ( m_pCamera, "FileAccessOffset" );
        VimbaFeatureInt     FileAccessLength        ( m_pCamera, "FileAccessLength" );
        VimbaFeatureEnum    FileOperationStatus     ( m_pCamera, "FileOperationStatus" );

        VimbaFeatureInt     FileOperationResult     ( m_pCamera, "FileOperationResult" );
        VimbaFeatureEnum    FileStatus              ( m_pCamera, "FileStatus" );
        VimbaFeatureInt     FileSize                ( m_pCamera, "FileSize" );
        VimbaFeatureEnum    FileSelector            ( m_pCamera, "FileSelector" );
        VimbaFeatureEnum    LUTSelector             ( m_pCamera, "LUTSelector" );

        FileSelector.SetStringValue( std::string( "LUT" )+LUTSelector.GetStringValue() );
        if( FileOpenMode.IsAvailable( "Write" ) )
        {
            FileOpenMode.SetStringValue( "Write" );
            FileOperationSelector.SetStringValue( "Open" );
            FileOperationExecute.Run();
            VmbInt64_t  nFileSize = FileSize( );
            if( m_data.size() != nFileSize )
            {
                throw Exception( __FUNCTION__,"LUT to upload does not seam to have the correct size for this camera",VmbErrorBadParameter );
            }
            //File access length
            VmbInt64_t  nMaxFileAccessLength = FileAccessLength.Max( );
            FileOperationSelector.SetStringValue( "Write" );

            VmbInt64_t  nFileAccessOffset = 0;
            VmbInt64_t  nFileAccessLength = min( nFileSize, nMaxFileAccessLength );
            UcharVector data( (size_t)nFileAccessLength );
            do
            {
                //Set FileAccessLength
                FileAccessLength( nFileAccessLength );
                //Fill buffer
                copy( &m_data[(size_t)nFileAccessOffset ], &m_data[(size_t)( nFileAccessLength+nFileAccessOffset-1 ) ], data.begin() );
                data = ConvertToNetworkInt16( data );
                FileAccessBuffer( data );
                //Execute file operation
                FileOperationExecute.Run( );
                //Get file operation status
                if( FileOperationStatus.GetStringValue() != "Success" )
                {
                    throw Exception( __FUNCTION__,"writing data failed",VmbErrorOther );
                }
                nFileAccessOffset =FileAccessOffset();

                nFileAccessLength = min( nFileSize - nFileAccessOffset, nMaxFileAccessLength );
            }
            while( nFileSize != nFileAccessOffset );
            FileOperationSelector.SetStringValue( "Close" );
            FileOperationExecute.Run();
            //Get file operation status
            if( FileOperationStatus.GetStringValue() != "Success" )
            {
                throw Exception( __FUNCTION__,"could not close file", VmbErrorOther );
            }
        }
    }
    else if( VmbBoolTrue == VimbaFeature::HasFeature( m_pCamera,"LUTIndex" ) )
    {
        //Get LUT address for GigE (indicator for direct memory access)
        VimbaFeatureInt     LUTAddress( m_pCamera, "LUTAddress" );                      //< get avt feature for lut address
        VmbUint32_t         nCompletedWrites = 0;
        UcharVector         network_data( ConvertToNetworkInt16( m_data ) );            //< expect our camera to be big endian device, so if we are intel little endian we have to swap
        m_pCamera->WriteMemory( LUTAddress() , network_data, nCompletedWrites );        //< write data direct to camera
    }
    //Camera doesn't support direct memory access for LUT
    else
    {
        //Used LUT index and value as indicator for GigE
        VimbaFeatureInt     LUTIndex( m_pCamera, "LUTIndex" );                          //< get lUT index feature to set current index
        VimbaFeatureInt     LUTValue( m_pCamera, "LUTValue" );                          //< get LUT value feature to set current value
        VimbaFeatureInt     LUTSize ( m_pCamera, "LUTSizeBytes");                       //< get feature for size of LUT in bytes
        if( m_data.size() != LUTSize() )                                                //< compare data size to LUT size 
        {
            throw Exception( __FUNCTION__,"data size is not equal lut size, they are incompatible",VmbErrorBadParameter );
        }
        //Evaluate number of LUT entries
        VmbInt64_t          nLUTBytePerValue  = ( GetBitDepthOut() > 8 ) ? 2 : 1;       //< determine if LUT values take one or two bytes
        VmbInt64_t          nLUTEntries       = LUTSize() / nLUTBytePerValue;           //< calculate lement count, LUT values are unpadded and consecutive in memory
        //Set LUT values by iteration over indexes
        int                 iter( 0 );
        VmbInt64_t          nValue( 0 );
        for( VmbInt64_t i = 0; i < nLUTEntries ; i++ )
        {
            LUTIndex( i );                                          //< set current index
            switch( nLUTBytePerValue )
            {
            case 1:
                nValue = m_data[iter++];                            //< copy 8bit value
                break;

            case 2:
                {
                    VmbUint16_t* p = (VmbUint16_t*)(&m_data[iter]); //< interpret data as host byte order uint 16
                    nValue = *p;
                    iter +=2;
                }
            }
            LUTValue( nValue );                                     //< write data to camera
        }
    }
}

//Load look up table from Csv
void LookUpTableControl::LoadFromCsv( const std::string &FileName, int nIndex )
{

    //Evaluate size of LUT
    VmbInt64_t                  nLUTBitDepthOut     = GetBitDepthOut( );            //< get output bit depth of LUT
    VmbInt64_t                  nLUTBytePerValue    = nLUTBitDepthOut > 8 ? 2 : 1;  //< determine if LUT values take one or two byte
    // Load LUT from CSV
    CsvLoad                     load( FileName );                                   //< import the LUT from CSV file
    std::vector<std::string>    row;
    while( load.Row( row ) )                                                        //< load rows until none left
    {
        if( row.size() <= ( size_t )nIndex )                                        //< test if index in range
        {
            throw Exception( __FUNCTION__,"index for the csv file is out of bounds",VmbErrorBadParameter );
        }

        int                 nData;                                                  //< coonvert string to int
        std::stringstream   ss( row[nIndex] );
        ss >> nData;

        char                data[2];                                               //< pack data for storage
        if( 2 == nLUTBytePerValue )
        {
            VmbUint16_t *p = ( VmbUint16_t* )( data );
            *p = static_cast<VmbUint16_t>( nData );
        }
        else
        {
            data[0] = static_cast<VmbUint8_t>( nData&0xFF );
        }

        copy( data, data + nLUTBytePerValue, std::back_inserter( m_data ) ); //< append data to internal storage
    }
}

//Save look up table to CSV
void LookUpTableControl::SaveToCsv( const std::string &FileName )
{

    //Raw data empty
    if( m_data.empty() )
    {
        throw Exception( __FUNCTION__,"no data to write to csv file", VmbErrorOther );
    }

    //Evaluate size of LUT
    VimbaFeatureInt    LUTSize( m_pCamera, "LUTSizeBytes" );                //< get feature for LUT size in bytes

    VmbInt64_t          nLUTBitDepthOut     = GetBitDepthOut( );            //< get output bit depth of LUT
    VmbInt64_t          nLUTBytePerValue    = nLUTBitDepthOut > 8 ? 2 : 1;  //< determine if LUT values take one or two bytes
    VmbInt64_t          nLUTEntries         = LUTSize()/nLUTBytePerValue;   //< calculate LUT element count, LUT values are unpadded and consecutive
    if( m_data.size() != LUTSize() )                                        //< if lut size and data size miss match they are not compatible
    {
        throw Exception( __FUNCTION__,"data does not equal the current cameras LUT",VmbErrorBadParameter );
    }
    // Save LUT data to CSV

    CsvSave save( FileName );                                           //< create CSV file
    {
        for( int i = 0; i < nLUTEntries; i++ )                          //< for each LUT value
        {
            int data ;                                                  //< unpack data to int
            if( 2 == nLUTBytePerValue )
            {
                data = *( ( VmbUint16_t* )( &m_data[i*2] ) );
            }
            else
            {
                data = m_data[i];
            }
            stringstream ss;                                            //< stream int to string
            ss << data;
            save.Row( ss.str() );                                       //< write a csv row
        }
    }
}

//Load look up table from flash
void LookUpTableControl::LoadFromFlash()
{
    VimbaFeatureCommand LUTLoad = CreateOneOf<VimbaFeatureCommand>( m_pCamera, "LUTLoad", "LUTLoadAll" ); // there are two possible names for this feature
    LUTLoad.Run();
}

//Save look up table to flash
void LookUpTableControl::SaveToFlash()
{
    VimbaFeatureCommand LUTSave = CreateOneOf<VimbaFeatureCommand>( m_pCamera, "LUTSave", "LUTSaveAll" ); //< there are two possible names for this feature
    LUTSave.Run();
}

}}} // namespace AVT::VmbAPI::Examples

