#ifndef AVT_VIMBA_FEATURES_H_
#define AVT_VIMBA_FEATURES_H_
/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        VimbaFeatures.h

  Description: Helper for features.
               

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
#include <vector>

#include "Exception.h"
#include "VimbaCPP/Include/VimbaCPP.h"

namespace AVT{
namespace VmbAPI{
namespace Examples{

    using AVT::VmbAPI::StringVector;
    //
    // basic feature.
    //
    class VimbaFeature
    {
        FeaturePtr  m_Feature;
    public:
                
        //
        // Method: VimbaFeature
        //
        // Purpose: constructor from feature pointer.
        //
        // Parameters:
        //
        // [in] feature  feature to wrap
        //
        VimbaFeature(const FeaturePtr &feature)
            : m_Feature( feature )
        { }
        //
        // Method: VimbaFeature
        //
        // Purpose: constructor from camera and feature name.
        //
        // Parameters:
        //
        // [in] cam    camera to get feature from
        // [in] name   name of the feature
        //
        VimbaFeature(const CameraPtr &cam, const std::string &name)
        {
            if( cam.get() == NULL )
            {
                throw Exception( __FUNCTION__,"camera pointer is null",VmbErrorBadParameter);
            }
            VmbErrorType Result = cam->GetFeatureByName( name.c_str() , m_Feature);
            if( VmbErrorSuccess != Result)
            {
                throw Exception( std::string( __FUNCTION__,"could not get vimba feature :")+name, Result);
            }
        }
        //
        // Method: GetType
        // 
        // Purpose: get feature data type.
        //
        VmbFeatureDataType GetType() const
        {
            VmbFeatureDataType DataType;
            VmbErrorType Result = m_Feature->GetDataType( DataType);
            if( VmbErrorSuccess != Result)
            {
                throw Exception( __FUNCTION__, "could not get feature data type", Result);
            }
            return DataType;
        }
        //
        // Method: HasFeature
        //
        // Purpose: static test if a feature is implemented.
        //
        // Parameters:
        //
        // [in] cam        camera to ask
        // param[in] name       feature name
        //
        static VmbBool_t HasFeature( const CameraPtr &cam, const std::string &name)
        {
            if( cam.get() == NULL )
            {
                throw Exception( __FUNCTION__,"camera pointer is null",VmbErrorBadParameter);
            }
            FeaturePtr dummy;
            VmbErrorType Result = cam->GetFeatureByName(name.c_str(), dummy);
            if( VmbErrorSuccess != Result)
            {
                if( VmbErrorNotFound == Result)
                {
                    return VmbBoolFalse;
                }
                throw Exception(__FUNCTION__,"could not get feature",Result);
            }
            return VmbBoolTrue;
        }
        //
        // Method: operator() const
        // 
        // Purpose : get the const feature pointer.
        //
        const FeaturePtr&       operator()()    const   { return m_Feature;    }
        //
        // Method: operator()
        //
        // Purpose:get the feature pointer.
        //
        FeaturePtr&             operator()()            { return m_Feature;    }
    };
    //
    // integer feature
    //
    class VimbaFeatureInt
    {
        VimbaFeature m_Feature;
    public:
        //
        // Method: VimbaFeatureInt
        //
        // Purpose: constructor from feature pointer.
        // [in] feature feature pointer
        //
        VimbaFeatureInt(const FeaturePtr &feature)
            : m_Feature( feature)
        {  
            if( VmbFeatureDataInt != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: VimbaFeatureInt
        //
        // Purpose: constructor from camera and feature name.
        //
        // Parameters:
        //
        // [in] cam        camera to get feature from
        // [in] name       feature name
        //
        VimbaFeatureInt(const CameraPtr &cam, const std::string &name)
            : m_Feature( cam, name)
        {  
            if( VmbFeatureDataInt != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: operator() const
        //
        // Purpose: get feature int value.
        //
        VmbInt64_t operator()() const
        {
            VmbInt64_t      Value;
            VmbErrorType    Result = m_Feature()->GetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get value for feature", Result);
            }
            return Value;
        }
        //
        // Method: operator( VmbInt64_t)
        //
        // Purpose: set feature int value.
        //
        // Parameters:
        //
        // [in] Value  value to set
        //
        void operator()( VmbInt64_t Value)
        {
            VmbErrorType Result = m_Feature()->SetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not set feature value", Result);
            }
        }
        //
        // Method: Min
        //
        // Purpose: get feature minimum value.
        //
        VmbInt64_t Min() const
        {
            VmbInt64_t MinValue,MaxValue;
            VmbErrorType Result = m_Feature()->GetRange(MinValue,MaxValue);
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get integer feature range",Result);
            }
            return MinValue;
        }
        //
        // Method: Max
        //
        // Purpose: get feature maximum value.
        //
        VmbInt64_t Max() const
        {
            VmbInt64_t MinValue,MaxValue;
            VmbErrorType Result = m_Feature()->GetRange(MinValue,MaxValue);
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get integer feature range",Result);
            }
            return MaxValue;
        }
        //
        // Method: Increment()
        //
        // Purpose: get the increment to advance an integer feature
        // Value = Min() + n * Increment() <= Max()
        //
        VmbInt64_t Increment() const
        {
            VmbInt64_t IncValue;
            VmbErrorType Result = m_Feature()->GetIncrement( IncValue );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get integer feature increment",Result);
            }
            return IncValue;
        }
    };
    //
    // Vimba bool feature.
    //
    class VimbaFeatureBool
    {
        VimbaFeature m_Feature;
    public:
        //
        // Method: VimbaFeatureBool
        //
        // Purpose:constructor from feature pointer.
        //
        // Parameters:
        //
        // [in] feature    feature pointer
        //
        VimbaFeatureBool(const FeaturePtr &feature)
            : m_Feature( feature )
        {
            if( VmbFeatureDataBool != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: VimbaFeatureBool
        //
        // Purpose: constructor from camera and feature name.
        //
        // Parameters:
        //
        // [in] cam        camera to get feature from
        // [in] name       name of the feature
        //
        VimbaFeatureBool(const CameraPtr &cam, const std::string &name)
            : m_Feature( cam, name)
        {
            if( VmbFeatureDataBool != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: operator() const
        //
        // Purpose: get feature bool value.
        //
        VmbBool_t operator()() const
        {
            VmbBool_t       Value;
            VmbErrorType    Result = m_Feature()->GetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get value for feature", Result);
            }
            return Value;
        }
        //
        // Method: operator()
        //
        // Purpose: set feature bool value.
        //
        void operator()( VmbBool_t Value)
        {
            VmbErrorType Result = m_Feature()->SetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not set feature value", Result);
            }
        }
    };
    //
    // Vimba float feature
    //
    class VimbaFeatureFloat
    {
        VimbaFeature m_Feature;
    public:
        //
        // Method: VimbaFeatureFloat
        //
        // Purpose:constructor from feature pointer.
        //
        // Parameters:
        //
        // [in] feature    feature pointer
        //
        VimbaFeatureFloat(const FeaturePtr &feature)
            : m_Feature( feature )
        {
            if( VmbFeatureDataFloat != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: VimbaFeatureFloat
        //
        // Purpose: constructor from camera and feature name.
        //
        // Parameters:
        //
        // [in] cam        camera to get feature from
        // [in] name       name of the feature
        //
        VimbaFeatureFloat(const CameraPtr &cam, const std::string &name)
            : m_Feature( cam, name)
        {
            if( VmbFeatureDataFloat != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: operator() const
        //
        // Purpose: get feature float value.
        //
        double operator()() const
        {
            double          Value;
            VmbErrorType    Result = m_Feature()->GetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get value for feature", Result);
            }
            return Value;
        }
        //
        // Method: operator()
        //
        // Purpose: set feature float value.
        //
        void operator()( double Value)
        {
            VmbErrorType Result = m_Feature()->SetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not set feature value", Result);
            }
        }
        double Min() const
        {
            double MinValue, MaxValue;
            VmbErrorType Result = m_Feature()->GetRange( MinValue, MaxValue );
            if( VmbErrorSuccess != Result)
            {
                throw Exception( __FUNCTION__,"could not get range for float feature", Result );
            }
            return MinValue;
        }
        double Max() const
        {
            double          MinValue, MaxValue;
            VmbErrorType    Result = m_Feature()->GetRange( MinValue, MaxValue );
            if( VmbErrorSuccess != Result)
            {
                throw Exception( __FUNCTION__,"could not get range for float feature", Result );
            }
            return MaxValue;
        }
    };
    //
    // feature enum.
    //
    class VimbaFeatureEnum
    {
        VimbaFeature    m_Feature;
    public:
        //
        // Method: VimbaFeatureEnum()
        //
        // Purpose: constructor from feature pointer.
        //
        // Parameters:
        //
        // [in] feature    feature pointer
        //
        VimbaFeatureEnum(const FeaturePtr &feature)
            : m_Feature( feature )
        {
            if( VmbFeatureDataEnum != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: VimbaFeatureEnum
        //
        // Purpose: constructor from camera and feature name.
        //
        // Parameters:
        //
        // [in] cam        camera to get feature from
        // [in] name       feature name
        //
        VimbaFeatureEnum(const CameraPtr &cam, const std::string &name)
            : m_Feature( cam, name )
        {
            if( VmbFeatureDataEnum != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Methods: GetValues
        //
        // Purpose:get all supported feature value strings.
        //
        AVT::VmbAPI::StringVector GetValues() const
        {
            StringVector Values;
            VmbErrorType Result = m_Feature()->GetValues( Values );
            if( VmbErrorSuccess != Result)
            {
                throw Exception( __FUNCTION__,"could not get values",Result);
            }
            return Values;
        }
        //
        // Method: GetEntries
        //
        // Purpose: get enum entries
        //
        EnumEntryVector GetEntries( ) const
        {
            EnumEntryVector Values;
            VmbErrorType    Result = m_Feature()->GetEntries( Values) ;
            if( VmbErrorSuccess != Result)
            {
                throw Exception( __FUNCTION__, "could not get entry values for enum feature", Result);
            }
            return Values;
        }
        //
        // Method: ValueCount
        //
        // Purpose: get number of supported feature values.
        //
        VmbInt64_t ValueCount() const
        {
            return static_cast< VmbInt64_t>( GetValues().size() );
        }
        //
        // Method: IsAvailable
        //
        // Purpose: test if string value is currently available.
        //
        // Parameters:
        //
        // [in] Value  string value to test
        //
        VmbBool_t IsAvailable( const std::string &Value)
        {
            VmbBool_t       Available;
            VmbErrorType    Result = m_Feature()->IsValueAvailable( Value.c_str(), Available );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not read enums available flag",Result);
            }
            return Available;
        }
        //
        // Method: IsAvailable
        //
        // Purpose: test if integer feature value is available.
        //
        // Parameters:
        //
        // [in] Value  integer feature value to test
        //
        VmbBool_t IsAvailable( VmbInt64_t Value)
        {
            VmbBool_t       Available;
            VmbErrorType    Result = m_Feature()->IsValueAvailable( Value, Available );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not read enums available flag",Result);
            }
            return Available;
        }
        // 
        // Method: GetStringValue
        //
        // Purpose:get current feature value as string.
        //
        std::string GetStringValue() const
        {
            std::string     Value;
            VmbErrorType    Result = m_Feature()->GetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get string value", Result);
            }
            return Value;
        }
        //
        // Method: SetStringValue
        //
        // Purpose: set new feature value from string.
        //
        // Parameters:
        //
        // [in] Value  string value to set
        //
        void SetStringValue( const std::string &Value)
        {
            VmbErrorType Result = m_Feature()->SetValue( Value.c_str() );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not set string value",Result);
            }
        }
        //
        // Method: GetEntry
        //
        // Purpose: get current enum value as EnumEntry
        //
        EnumEntry GetEntry() const
        {
            EnumEntry       Value;
            VmbErrorType    Result = m_Feature()->GetEntry( Value, GetStringValue().c_str() );
            if( VmbErrorSuccess != Result )
            {
                throw Exception( __FUNCTION__,"could not get enum feature entry", Result);
            }
            return Value;
        }
        //
        // Method: operator() const
        //
        // Purpose: get current integer value.
        //
        VmbInt64_t operator()() const
        {
            VmbInt64_t      Value;
            VmbErrorType    Result = m_Feature()->GetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get value for feature", Result);
            }
            return Value;
        }
        //
        // Method: operator()
        //
        // Purpose: set new integer value.
        //
        // Parameters:
        //
        // [in] Value  new value to set
        //
        void operator()( VmbInt64_t Value)
        {
            VmbErrorType Result = m_Feature()->SetValue( Value );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not set feature value", Result);
            }
        }
    };

    //
    // Vimba string feature
    class VimbaFeatureString
    {
        VimbaFeature m_Feature;
    public:
        VimbaFeatureString( const FeaturePtr & feature)
            : m_Feature( feature )
        {
            if( VmbFeatureDataString != m_Feature.GetType() )
            {
                throw Exception( __FUNCTION__,"wrong feature data type", VmbErrorWrongType );
            }
        }
        VimbaFeatureString( const CameraPtr & cam, const std::string &name)
            :m_Feature( cam, name )
        {
            if( VmbFeatureDataString != m_Feature.GetType() )
            {
                throw Exception( __FUNCTION__,"wrong feature data type", VmbErrorWrongType );
            }
        }
        std::string operator()() const
        {
            std::string     Value;
            VmbErrorType    Result = m_Feature()->GetValue( Value );
            if( VmbErrorSuccess != Result )
            {
                throw Exception( __FUNCTION__,"could not get value for string feature", Result);
            }
            return Value;
        }
        void operator()( const std::string & value)
        {
            VmbErrorType Result = m_Feature()->SetValue( value.c_str() );
            if( VmbErrorSuccess != Result)
            {
                throw Exception( __FUNCTION__,"could not set string feature value" , Result );
            }
        }
    };
    //
    // feature command
    //
    class VimbaFeatureCommand
    {
        VimbaFeature m_Feature;
    public:
        //
        // Method: VimbaFeatureCommand
        //
        // Purpose: constructor from feature pointer.
        //
        // Parameters:
        //
        // [in] feature    feature pointer to use
        //
        VimbaFeatureCommand( const FeaturePtr &feature)
            :m_Feature( feature )
        {
            if( VmbFeatureDataCommand != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: VimbaFeatureCommand
        //
        // Purpose: constructor from camera and feature name.
        // 
        // Parameters:
        //
        // [in] cam        camera to get feature from
        // [in] name       name of the feature
        //
        VimbaFeatureCommand( const CameraPtr &cam, const std::string &name)
            :m_Feature( cam, name)
        {
            if( VmbFeatureDataCommand != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: Run
        //
        // Purpose: run command.
        //
        void Run()
        {
            VmbErrorType Result = m_Feature()->RunCommand();
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not run feature command",Result);
            }
        }
        //
        // Method: IsDone
        //
        // Purpose:test if command has finished.
        //
        VmbBool_t IsDone() const
        {
            VmbBool_t Value;
            VmbErrorType Result = m_Feature()->IsCommandDone( Value);
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not get is done state of feature command",Result);
            }
            return Value;
        }
    };
    //
    // raw buffer feature.
    //
    class VimbaFeatureRaw
    {
        VimbaFeature m_Feature;
    public:
        //
        // Method: VimbaFeatureRaw
        //
        // Purpose: constructor from feature pointer.
        //
        // Parameters:
        //
        // [in] feature    feature pointer to use
        //
        VimbaFeatureRaw( const FeaturePtr &feature)
            :m_Feature( feature )
        {
            if( VmbFeatureDataRaw != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: VimbaFeatureRaw
        //
        // Purpose: constructor from camera and feature name.
        //
        // Parameters:
        //
        // [in] cam        camera to get feature from
        // [in] name       feature name
        //
        VimbaFeatureRaw( const CameraPtr &cam, const std::string &name)
            :m_Feature( cam, name)
        {
            if( VmbFeatureDataRaw != m_Feature.GetType() )
            {
                throw Exception(__FUNCTION__,"wrong feature data type", VmbErrorWrongType);
            }
        }
        //
        // Method: GetBuffer
        //
        // Purpose: get raw buffer, reuses supplied data vector .
        //
        // Parameters:
        //
        // [out] data      returns read data
        //
        void GetBuffer( UcharVector &data) const
        {
            VmbErrorType Result = m_Feature()->GetValue( data );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not read raw data feature",Result);
            }
        }
        //
        // Method: operator() const
        //
        // Purpose: get raw data buffer.
        //
        UcharVector operator()() const
        {
            UcharVector     Data;
            VmbErrorType    Result = m_Feature()->GetValue( Data );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not read raw data feature",Result);
            }
            return Data;
        }
        //
        // Method: operator( UcharVector )
        //
        // Purpose: set raw data buffer.
        //
        // Parameters:
        //
        // [in] data   data to set
        //
        void operator()( const UcharVector & Data)
        {
            VmbErrorType Result =m_Feature()->SetValue( Data );
            if( VmbErrorSuccess != Result)
            {
                throw Exception(__FUNCTION__,"could not set raw feature data",Result);
            }
        }
    };
    //
    // Method: CreateOneOf
    //
    // Purpose: create a feature from two possible names.
    // Parameters:
    //
    // [in] cam        camera to get feature from
    // [in] name_one   first name to try
    // [in] name_two   second name to try
    //
    template <typename FEATURE_TYPE>
    FEATURE_TYPE CreateOneOf(const CameraPtr &cam, const std::string& name_one, const std::string &name_two)
    {
        FeaturePtr feature;
        VmbErrorType Result = cam->GetFeatureByName( name_one.c_str(), feature);
        if( VmbErrorSuccess != Result)
        {
            if( VmbErrorNotFound == Result)
            {
                return FEATURE_TYPE( cam, name_two);
            }
            throw Exception(__FUNCTION__,"could not generate feature",Result);
        }
        return FEATURE_TYPE(feature);
    }
}}}
#endif