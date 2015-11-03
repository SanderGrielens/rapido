#ifndef __NXLIB_FUNCTIONS_H__
#define __NXLIB_FUNCTIONS_H__

#include "nxLibConstants.h"
#ifdef __cplusplus
extern "C" {
#endif
#ifndef NXLIB_DYNAMIC_LOAD
	NXLIB_EXPORT void        nxLibErase             (NXLIBERR* result, NXLIBSTR itemPath);

	NXLIB_EXPORT void        nxLibSetNull           (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT void        nxLibSetJson           (NXLIBERR* result, NXLIBSTR itemPath, NXLIBSTR    value, NXLIBBOOL onlyWriteableNodes);
	NXLIB_EXPORT void        nxLibSetInt            (NXLIBERR* result, NXLIBSTR itemPath, NXLIBINT    value);
	NXLIB_EXPORT void        nxLibSetDouble         (NXLIBERR* result, NXLIBSTR itemPath, NXLIBDOUBLE value);
	NXLIB_EXPORT void        nxLibSetBool           (NXLIBERR* result, NXLIBSTR itemPath, NXLIBBOOL   value);
	NXLIB_EXPORT void        nxLibSetString         (NXLIBERR* result, NXLIBSTR itemPath, NXLIBSTR    value);
	NXLIB_EXPORT void        nxLibSetBinary         (NXLIBERR* result, NXLIBSTR itemPath, void const* buffer, NXLIBINT bufferSize);
	NXLIB_EXPORT void        nxLibSetBinaryFormatted(NXLIBERR* result, NXLIBSTR itemPath, void const* buffer, NXLIBINT width, NXLIBINT height, NXLIBINT channelCount, NXLIBINT bytesPerElement, NXLIBBOOL isFloat);

	NXLIB_EXPORT NXLIBSTR    nxLibGetJson           (NXLIBINT* result, NXLIBSTR itemPath, NXLIBBOOL prettyPrint, NXLIBINT numberPrecision, NXLIBBOOL scientificNumberFormat);
	NXLIB_EXPORT NXLIBSTR    nxLibGetJsonMeta       (NXLIBINT* result, NXLIBSTR itemPath, NXLIBINT numLevels, NXLIBBOOL prettyPrint, NXLIBINT numberPrecision, NXLIBBOOL scientificNumberFormat);
	NXLIB_EXPORT NXLIBINT    nxLibGetInt            (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT NXLIBDOUBLE nxLibGetDouble         (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT NXLIBBOOL   nxLibGetBool           (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT NXLIBSTR    nxLibGetString         (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT void        nxLibGetBinary         (NXLIBERR* result, NXLIBSTR itemPath, void* destinationBuffer, NXLIBINT bufferSize, NXLIBINT* bytesCopied, NXLIBDOUBLE* timestamp);

	NXLIB_EXPORT NXLIBINT    nxLibGetType           (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT NXLIBINT    nxLibGetCount          (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT NXLIBSTR    nxLibGetName           (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT void        nxLibGetBinaryInfo     (NXLIBERR* result, NXLIBSTR itemPath, NXLIBERR* width, NXLIBINT* height, NXLIBINT* channelCount, NXLIBINT* bytesPerElement, NXLIBBOOL* isFloat, NXLIBDOUBLE* timestamp);

	NXLIB_EXPORT void        nxLibWaitForChange     (NXLIBERR* result, NXLIBSTR itemPath);
	NXLIB_EXPORT void        nxLibWaitForType       (NXLIBERR* result, NXLIBSTR itemPath, NXLIBINT  nxLibItemType, NXLIBBOOL waitForEqual);
	NXLIB_EXPORT void        nxLibWaitForStringValue(NXLIBERR* result, NXLIBSTR itemPath, NXLIBSTR          value, NXLIBBOOL waitForEqual);
	NXLIB_EXPORT void        nxLibWaitForIntValue   (NXLIBERR* result, NXLIBSTR itemPath, NXLIBINT          value, NXLIBBOOL waitForEqual);
	NXLIB_EXPORT void        nxLibWaitForDoubleValue(NXLIBERR* result, NXLIBSTR itemPath, NXLIBDOUBLE       value, NXLIBBOOL waitForEqual);
	NXLIB_EXPORT void        nxLibWaitForBoolValue  (NXLIBERR* result, NXLIBSTR itemPath, NXLIBBOOL         value, NXLIBBOOL waitForEqual);

	NXLIB_EXPORT void        nxLibWriteDebugMessage (NXLIBERR* result, NXLIBSTR message);
	NXLIB_EXPORT NXLIBSTR    nxLibGetDebugMessages  (NXLIBINT* result);
#endif
#ifdef __cplusplus
}
#endif

static char const* ATTR_UNUSED nxLibTranslateReturnCode(NXLIBINT returnCode) {
	switch (returnCode) {
		#ifdef NXLIB_APIERROR
		#undef NXLIB_APIERROR
		#endif
		#define NXLIB_APIERROR(NAME, VALUE) case VALUE: return #NAME; break
		#include "nxLibApiErrors.h"
	default: 
		return 0; break;
	}
	return 0;
}

#ifdef __cplusplus

	#include <string>
	#include <cstring>
	#include <sstream>
	#include <vector>
	#include <assert.h>

	class NxLibException {
	protected:
		std::string itemPath;
		int errorCode;
		std::string errorText;
	public:
		NxLibException(std::string const& itemPath, int errorCode) {
			this->itemPath = itemPath;
			this->errorCode = errorCode;
			char const* text = nxLibTranslateReturnCode(errorCode);
			this->errorText = text ? text : "";
		};
		/// The NxLib API return code that caused this exception
		int getErrorCode() const { return errorCode; }
		/// The text description of the failure
		std::string const& getErrorText() const { return errorText; }
		/// The path to the item on which the failed operation was attempted
		std::string const& getItemPath()  const { return itemPath;  }
	};

	class NxLibItem {
	protected:
		template <typename T> void assignIfGiven(T* ptr, T const& value) const { if (ptr) (*ptr) = value; }
		void checkReturnCode(int returnCode) const {
			if (returnCode != NxLibOperationSucceeded) {
				throw NxLibException(path, returnCode);
			}
		}
	public:
		std::string path;

		explicit NxLibItem(std::string const& path = "") { // references root node by default
			this->path = path;
		}

		void operator<<(NxLibItem const& otherItem) const {
			setJson(otherItem.asJson(), true);
		}
		void operator<<(std::string const& jsonString) const {
			setJson(jsonString, true);
		}
		void operator<<(char const * const jsonString) const {
			setJson(jsonString, true);
		}

		NxLibItem const operator[](std::string const& subItemName) const {
			return NxLibItem(path + NxLibItemSeparator + subItemName);
		}

		NxLibItem const operator[](int subItemIndex) const {
			std::ostringstream newPath;
			newPath << path << NxLibItemSeparator << NxLibIndexEscapeChar << subItemIndex;
			return NxLibItem(newPath.str());
		}

		int compare(int* returnCode, char const* const value) const {
			return compare(returnCode, std::string(value));
		}
		template <typename T>
		int compare(T const& value) const { 
			int r; 
			int result = compare(&r, value); 
			checkReturnCode(r); 
			return result; 
		}

		// Specialization for integers below (will always compare as double!)
		template <typename T>
		int compare(int* returnCode, T const& value) const {
			int result;
			T itemValue = as<T>(&result);
			assignIfGiven(returnCode, result);
			if (result == NxLibItemTypeNotCompatible) return 2;
			return itemValue == value ? 0 : itemValue < value ? -1 : 1;
		}

		template <typename T>
		void set(T const& value) const {
			int result;
			set(&result, value);
			checkReturnCode(result);
		}
		void set(int* returnCode, int value) const {
			nxLibSetInt(returnCode, path.c_str(), value);
		}
		void set(int* returnCode, double value) const {
			nxLibSetDouble(returnCode, path.c_str(), value);
		}
		void set(int* returnCode, bool value) const {
			nxLibSetBool(returnCode, path.c_str(), value);
		}
		void set(int* returnCode, std::string const& value) const { 
			set(returnCode, value.c_str()); 
		}
		void set(int* returnCode, char const * const value) const {
			nxLibSetString(returnCode, path.c_str(), value);
		}


		void setNull() const {
			int result;
			setNull(&result);
			checkReturnCode(result);
		}
		void setNull(int* returnCode) const {
			nxLibSetNull(returnCode, path.c_str());
		}
		void setJson(std::string const& value, bool onlyWriteableNodes = false) const {
			int result;
			setJson(&result, value, onlyWriteableNodes);
			checkReturnCode(result);
		}
		void setJson(int* returnCode, std::string const& value, bool onlyWriteableNodes = false) const {
			nxLibSetJson(returnCode, path.c_str(), value.c_str(), onlyWriteableNodes);
		}

		template <typename T> T const& operator= (T const& value) const { set(value); return value; }

		template <typename T> bool operator==(T const& value) const { return compare(value) == 0; }
		template <typename T> bool operator!=(T const& value) const { return compare(value) != 0; }
		template <typename T> bool operator> (T const& value) const { return compare(value) >  0; }
		template <typename T> bool operator< (T const& value) const { return compare(value) <  0; }
		template <typename T> bool operator>=(T const& value) const { return compare(value) >= 0; }
		template <typename T> bool operator<=(T const& value) const { return compare(value) <= 0; }

		// Access functions returning API errors with an int*
		template <typename T> T as(int* returnCode) const;
		// Specializations after class declaration

		std::string asString(int* returnCode) const {
			NXLIBSTR ret = nxLibGetString(returnCode, path.c_str());
			return ret ? std::string((char const*) ret) : "";
		}
		int asInt(int* returnCode) const {
			NXLIBINT ret = nxLibGetInt(returnCode, path.c_str());
			return ret;
		}
		double asDouble(int* returnCode) const {
			double ret = nxLibGetDouble(returnCode, path.c_str());
			return ret;
		}
		bool asBool(int* returnCode) const {
			bool ret = nxLibGetBool(returnCode, path.c_str()) != NXLIBFALSE;
			return ret;
		}
		std::string asJson(int* returnCode, bool prettyPrint = false, int numberPrecision = 18, bool scientificNumberFormat = true) const {
			NXLIBSTR ret;
			ret = nxLibGetJson(returnCode, path.c_str(), prettyPrint, numberPrecision, scientificNumberFormat);
			return ret? std::string((char const*) ret) : "";
		}
		std::string asJsonMeta(int* returnCode, int numLevels = -1, bool prettyPrint = false, int numberPrecision = 18, bool scientificNumberFormat = true) const {
			NXLIBSTR ret;
			ret = nxLibGetJsonMeta(returnCode, path.c_str(), numLevels, prettyPrint, numberPrecision, scientificNumberFormat);
			return ret? std::string((char const*) ret) : "";
		}

		std::string asJson(bool prettyPrint = false, int numberPrecision = 18, bool scientificNumberFormat = false) const {
			int returnCode;
			std::string result = asJson(&returnCode, prettyPrint, numberPrecision, scientificNumberFormat);
			checkReturnCode(returnCode);
			return result;
		}
		std::string asJsonMeta(int numLevels = -1, bool prettyPrint = false, int numberPrecision = 18, bool scientificNumberFormat = true) const {
			int returnCode;
			std::string result = asJsonMeta(&returnCode, numLevels, prettyPrint, numberPrecision, scientificNumberFormat);
			checkReturnCode(returnCode);
			return result;
		}
		template <typename T> T as() const { 
			int returnCode;
			T value = as<T>(&returnCode);
			checkReturnCode(returnCode);
			return value;
		}
		std::string asString() const { return as<std::string>(); }
		int         asInt   () const { return as<int>        (); }
		double      asDouble() const { return as<double>     (); }
		bool        asBool  () const { return as<bool>       (); }

		bool isNull  (int* returnCode) const { return type(returnCode) == NxLibItemTypeNull;   }
		bool isString(int* returnCode) const { return type(returnCode) == NxLibItemTypeString; }
		bool isNumber(int* returnCode) const { return type(returnCode) == NxLibItemTypeNumber; }
		bool isBool  (int* returnCode) const { return type(returnCode) == NxLibItemTypeBool;   }
		bool isArray (int* returnCode) const { return type(returnCode) == NxLibItemTypeArray;  }
		bool isObject(int* returnCode) const { return type(returnCode) == NxLibItemTypeObject; }
		bool isNull() const { 
			int result;
			bool res = isNull(&result);
			checkReturnCode(result);
			return res;
		}
		bool isString() const { 
			int result;
			bool res = isString(&result);
			checkReturnCode(result);
			return res;
		}
		bool isNumber() const { 
			int result;
			bool res = isNumber(&result);
			checkReturnCode(result);
			return res;
		}
		bool isBool() const { 
			int result;
			bool res = isBool(&result);
			checkReturnCode(result);
			return res;
		}
		bool isArray() const { 
			int result;
			bool res = isArray(&result);
			checkReturnCode(result);
			return res;
		}
		bool isObject() const { 
			int result;
			bool res = isObject(&result);
			checkReturnCode(result);
			return res;
		}

		int type(int* returnCode) const {
			NXLIBINT ret = nxLibGetType(returnCode, path.c_str());
			return ret;
		}
		int type() const {
			int result;
			int typeId = type(&result);
			checkReturnCode(result);
			return typeId;
		}

		bool exists() const {
			int result;
			bool res = exists(&result);
			checkReturnCode(result);
			return res;
		}
		bool exists(int* returnCode) const {
			NXLIBINT result;
			NXLIBINT ret = nxLibGetType(&result, path.c_str());
			if (result == NxLibOperationSucceeded) {
				assignIfGiven(returnCode, result);
				return ret != NxLibItemTypeInvalid;
			} else if (result == NxLibItemInexistent) {
				assignIfGiven(returnCode, NxLibOperationSucceeded);
			} else {
				assignIfGiven(returnCode, result);
			}
			return false;
		}

		void erase() const {
			int result;
			erase(&result);
			if (result == NxLibItemInexistent) return;
			checkReturnCode(result);
		}
		void erase(int* returnCode) const {
			nxLibErase(returnCode, path.c_str());
		}

		std::string name() const {
			int result;
			std::string res = name(&result);
			checkReturnCode(result);
			return res;
		}
		std::string name(int* returnCode) const {
			NXLIBSTR ret = nxLibGetName(returnCode, path.c_str());
			return ret? std::string((char const*) ret) : "";
		}

		int count() const {
			int result;
			int res = count(&result);
			checkReturnCode(result);
			return res;
		}
		int count(int* returnCode) const {
			NXLIBINT ret = nxLibGetCount(returnCode, path.c_str());
			return ret;
		}
		void getBinaryDataInfo(int* width, int* height, int* channels, int* bytesPerElement, bool* isFloat, double* timestamp) const {
			int result;
			getBinaryDataInfo(&result, width, height, channels, bytesPerElement, isFloat, timestamp);
			checkReturnCode(result);
		}
		void getBinaryDataInfo(int* returnCode, int* width, int* height, int* channels, int* bytesPerElement, bool* isFloat, double* timestamp) const {
			NXLIBBOOL isFloat_;
			nxLibGetBinaryInfo(returnCode, path.c_str(), width, height, channels, bytesPerElement, &isFloat_, timestamp);
			assignIfGiven(isFloat, isFloat_ != NXLIBFALSE);
		}
		void getBinaryData(void* bufferPointer, int bufferSize, int* numBytesCopied, double* timestamp) const {
			int result;
			getBinaryData(&result, bufferPointer, bufferSize, numBytesCopied, timestamp);
			checkReturnCode(result);
		}
		void getBinaryData(int* returnCode, void* bufferPointer, int bufferSize, int* numBytesCopied, double* timestamp) const {
			nxLibGetBinary(returnCode, path.c_str(), bufferPointer, bufferSize, numBytesCopied, timestamp);
		}
		void setBinaryData(void const* bufferPointer, int bufferSize) const {
			int result;
			setBinaryData(&result, bufferPointer, bufferSize);
			checkReturnCode(result);
		}
		void setBinaryData(int* returnCode, void const* bufferPointer, int bufferSize) const {
			nxLibSetBinary(returnCode, path.c_str(), bufferPointer, bufferSize);
		}
		template <typename T>
		void getBinaryData(std::vector<T>& bufferVector, double* timestamp) const {
			int result;
			getBinaryData(&result, bufferVector, timestamp);
			checkReturnCode(result);
		}
		template <typename T>
		void setBinaryData(std::vector<T> const& bufferVector) const {
			int result;
			setBinaryData(&result, bufferVector);
			checkReturnCode(result);
		}
		template <typename T>
		void setBinaryData(int* returnCode, std::vector<T> const& bufferVector) const {
			setBinaryData(returnCode, bufferVector.empty() ? 0 : &bufferVector[0], (int) bufferVector.size() * sizeof(T));
		}
		template <typename T>
		void setBinaryData(std::vector<T> const& bufferVector, int width, int height, int channelCount, bool isFloat) const {
			int result;
			setBinaryData(&result, bufferVector, width, height, channelCount, isFloat);
			checkReturnCode(result);
		}
		template <typename T>
		void setBinaryData(int* returnCode, std::vector<T> const& bufferVector, int width, int height, int channelCount, bool isFloat) const {
			int bytesPerElement = sizeof(T) / channelCount;
			assert((sizeof(T) % channelCount) == 0);
			assert(sizeof(T) == channelCount*bytesPerElement);
			assert(width*height == bufferVector.size());
			nxLibSetBinaryFormatted(returnCode, path.c_str(), bufferVector.empty() ? 0 : &bufferVector[0], width, height, channelCount, bytesPerElement, isFloat);
		}
#if CV_MAJOR_VERSION > 1
		void getBinaryData(cv::Mat& cvMat, double* timestamp) const {
			int result;
			getBinaryData(&result, cvMat, timestamp);
			checkReturnCode(result);
		}
		void getBinaryData(int* returnCode, cv::Mat& cvMat, double* timestamp) const {
			int width, height, type, channels, bpe;
			bool isFlt;
			getBinaryDataInfo(&width, &height, &channels, &bpe, &isFlt, 0);
			if (isFlt) {
				switch (channels) {
					case 1: type = (bpe == 4) ? CV_32FC1 : CV_64FC1; break;
					case 2: type = (bpe == 4) ? CV_32FC2 : CV_64FC2; break;
					case 3: type = (bpe == 4) ? CV_32FC3 : CV_64FC3; break;
					case 4: type = (bpe == 4) ? CV_32FC4 : CV_64FC4; break;
				}
			} else {
				switch (channels) {
					case 1:
						switch (bpe) {
							case 1: type = CV_8UC1; break;
							case 2: type = CV_16UC1; break;
							case 4: type = CV_32SC1; break;
						}
						break;
					case 2:
						switch (bpe) {
							case 1: type = CV_8UC2; break;
							case 2: type = CV_16UC2; break;
							case 4: type = CV_32SC2; break;
						}
						break;
					case 3:
						switch (bpe) {
							case 1: type = CV_8UC3; break;
							case 2: type = CV_16UC3; break;
							case 4: type = CV_32SC3; break;
						}
						break;
					case 4:
						switch (bpe) {
							case 1: type = CV_8UC4; break;
							case 2: type = CV_16UC4; break;
							case 4: type = CV_32SC4; break;
						}
						break;
				}
			}
			cvMat.create(height, width, type);
			int bytesCopied = 0;
			getBinaryData(returnCode, cvMat.ptr(), height*((int) cvMat.step), &bytesCopied, timestamp);
			assert(bytesCopied == height*((int) cvMat.step));
		}
		void setBinaryData(cv::Mat const& cvMat) const {
			int result;
			setBinaryData(&result, cvMat);
			checkReturnCode(result);
		}
		void setBinaryData(int* returnCode, cv::Mat const& cvMat) const {
			int channels, bpe; bool isFlt;
			int type = cvMat.type();
			switch(type) {
			case CV_8SC1:  case CV_8UC1:  channels = 1; bpe = 1; isFlt = false; break;
			case CV_8SC2:  case CV_8UC2:  channels = 2; bpe = 1; isFlt = false; break;
			case CV_8SC3:  case CV_8UC3:  channels = 3; bpe = 1; isFlt = false; break;
			case CV_8SC4:  case CV_8UC4:  channels = 4; bpe = 1; isFlt = false; break;
			case CV_16SC1: case CV_16UC1: channels = 1; bpe = 2; isFlt = false; break;
			case CV_16SC2: case CV_16UC2: channels = 2; bpe = 2; isFlt = false; break;
			case CV_16SC3: case CV_16UC3: channels = 3; bpe = 2; isFlt = false; break;
			case CV_16SC4: case CV_16UC4: channels = 4; bpe = 2; isFlt = false; break;
			case CV_32SC1:                channels = 1; bpe = 4; isFlt = false; break;
			case CV_32SC2:                channels = 2; bpe = 4; isFlt = false; break;
			case CV_32SC3:                channels = 3; bpe = 4; isFlt = false; break;
			case CV_32SC4:                channels = 4; bpe = 4; isFlt = false; break;
			case CV_32FC1:                channels = 1; bpe = 4; isFlt = true;  break;
			case CV_32FC2:                channels = 2; bpe = 4; isFlt = true;  break;
			case CV_32FC3:                channels = 3; bpe = 4; isFlt = true;  break;
			case CV_32FC4:                channels = 4; bpe = 4; isFlt = true;  break;
			case CV_64FC1:                channels = 1; bpe = 8; isFlt = true;  break;
			case CV_64FC2:                channels = 2; bpe = 8; isFlt = true;  break;
			case CV_64FC3:                channels = 3; bpe = 8; isFlt = true;  break;
			case CV_64FC4:                channels = 4; bpe = 8; isFlt = true;  break;
			}

			nxLibSetBinaryFormatted(returnCode, path.c_str(), cvMat.ptr(), cvMat.cols, cvMat.rows, channels, bpe, isFlt);
		}
#endif
		template <typename T>
		void getBinaryData(int* returnCode, std::vector<T>& bufferVector, double* timestamp) const {
			int result;
			int sizeNeeded = count(&result);
			if (result != NxLibOperationSucceeded) {
				assignIfGiven(returnCode, result);
				return;
			}
			NXLIBINT elementsNeeded = sizeNeeded/sizeof(T);
			if (sizeNeeded%sizeof(T) != 0) {
				assignIfGiven(returnCode, NxLibBufferNotDivisibleByElementSize);
				return;
			}
			bufferVector.resize(elementsNeeded);
			NXLIBINT bytesCopied;
			char fakeBuffer;
			getBinaryData(&result, bufferVector.size() > 0 ? (void*) &bufferVector[0] : (void*) &fakeBuffer, elementsNeeded*sizeof(T), &bytesCopied, timestamp);
			if ((result == NxLibOperationSucceeded) && (bytesCopied != sizeNeeded)) result = NxLibInternalError;
			assignIfGiven(returnCode, result);
		}

		void waitForChange() const {
			int result;
			waitForChange(&result);
			checkReturnCode(result);
		}
		void waitForChange(int* returnCode) const {
			nxLibWaitForChange(returnCode, path.c_str());
		}
		void waitForType(int type, bool waitForEqual) const {
			int result;
			waitForType(&result, type, waitForEqual);
			checkReturnCode(result);
		}
		void waitForType(int* returnCode, int type, bool waitForEqual) const {
			nxLibWaitForType(returnCode, path.c_str(), type, waitForEqual);
		}
		void waitForValue(int value, bool waitForEqual) const {
			int result;
			waitForValue(&result, value, waitForEqual);
			checkReturnCode(result);
		}
		void waitForValue(int* returnCode, int value, bool waitForEqual) const {
			nxLibWaitForIntValue(returnCode, path.c_str(), value, waitForEqual);
		}
		void waitForValue(double value, bool waitForEqual) const {
			int result;
			waitForValue(&result, value, waitForEqual);
			checkReturnCode(result);
		}
		void waitForValue(int* returnCode, double value, bool waitForEqual) const {
			nxLibWaitForDoubleValue(returnCode, path.c_str(), value, waitForEqual);
		}
		void waitForValue(bool value, bool waitForEqual) const {
			int result;
			waitForValue(&result, value, waitForEqual);
			checkReturnCode(result);
		}
		void waitForValue(int* returnCode, bool value, bool waitForEqual) const {
			nxLibWaitForBoolValue(returnCode, path.c_str(), value, waitForEqual);
		}
		void waitForValue(char const * const value, bool waitForEqual) const {
			int result;
			waitForValue(&result, value, waitForEqual);
			checkReturnCode(result);
		}
		void waitForValue(int* returnCode, char const * const value, bool waitForEqual) const {
			nxLibWaitForStringValue(returnCode, path.c_str(), value, waitForEqual);
		}
		void waitForValue(std::string const& value, bool waitForEqual) const {
			int result;
			waitForValue(&result, value, waitForEqual);
			checkReturnCode(result);
		}
		void waitForValue(int* returnCode, std::string const& value, bool waitForEqual) const {
			int result;
			waitForValue(&result, value.c_str(), waitForEqual);
			assignIfGiven(returnCode, result);
		}
	};
	template <> inline int         NxLibItem::compare<int>        (int* returnCode, int const&         value) const { return compare<double>(returnCode, value); }
	template <> inline std::string NxLibItem::as<std::string>     (int* returnCode) const { return asString(returnCode); }
	template <> inline char const* NxLibItem::as<char const*>     (int* returnCode) const { return asString(returnCode).c_str(); }
	template <> inline int         NxLibItem::as<int>             (int* returnCode) const { return asInt(returnCode); }
	template <> inline double      NxLibItem::as<double>          (int* returnCode) const { return asDouble(returnCode); }
	template <> inline bool        NxLibItem::as<bool>            (int* returnCode) const { return asBool(returnCode); }

	class NxLibCommand {
	public: 
		NxLibCommand(std::string const& commandName) { this->commandName = commandName; }
	protected:
		template <typename T> void assignIfGiven(T* ptr, T const& value) const { if (ptr) (*ptr) = value; }
		void checkReturnCode(int returnCode) const {
			if (returnCode != NxLibOperationSucceeded) {
				NxLibException exception(itmExecute, returnCode);
				throw exception;
			}
		}
	public:
		std::string commandName;

	public:
		NxLibItem const parameters() const { return NxLibItem()[itmExecute][itmParameters]; }
		NxLibItem const result()     const { return NxLibItem()[itmExecute][itmResult    ]; }
		
		void execute(bool wait = true) const {
			int result;
			execute(&result, wait);
			checkReturnCode(result);
		}
		void execute(int* returnCode, bool wait = true) const {
			int result;
			if (commandName != cmdBreak) {
				bool idle = finished(&result);
				assert(idle && (result == NxLibOperationSucceeded));
			}
			NxLibItem functionItem = NxLibItem()[itmExecute][itmCommand];
			functionItem.set(&result, commandName);
			if (result != NxLibOperationSucceeded) {
				assignIfGiven(returnCode, result);
				return;
			}
			if (wait) {
				functionItem.waitForType(&result, NxLibItemTypeNull, true);
				if (result != NxLibOperationSucceeded) {
					assignIfGiven(returnCode, result);
					return;
				}
				result = successful(returnCode) ? NxLibOperationSucceeded : NxLibExecutionFailed;
			}
			assignIfGiven(returnCode, result);
		}

		bool finished() const {
			int result;
			bool idle = finished(&result);
			checkReturnCode(result);
			return idle;
		}
		bool finished(int* returnCode) const {
			return NxLibItem()[itmExecute][itmCommand].type(returnCode) == NxLibItemTypeNull;
		}

		bool successful() const {
			int result;
			bool ok = successful(&result);
			checkReturnCode(result);
			return ok;
		}
		bool successful(int* returnCode) const {
			int err;
			bool res = !(result()[itmErrorSymbol].exists(&err));
			assignIfGiven(returnCode, err);
			return res;
		}
	};

	static void ATTR_UNUSED nxLibCheckReturnCode(int returnCode) {
		if (returnCode != NxLibOperationSucceeded) throw NxLibException("", returnCode);
	}
	static void ATTR_UNUSED nxLibWriteDebugMessage(std::string const& message) {
		int result;
		nxLibWriteDebugMessage(&result, message.c_str());
		nxLibCheckReturnCode(result);
	}
	static std::string ATTR_UNUSED nxLibGetDebugMessages() {
		int result;
		std::string messages;
		char const* str = nxLibGetDebugMessages(&result);
		nxLibCheckReturnCode(result);
		if (str) messages = std::string(str);
		return messages;
	}

#endif /* __cplusplus */

#endif /* __NXLIB_FUNCTIONS_H__ */
