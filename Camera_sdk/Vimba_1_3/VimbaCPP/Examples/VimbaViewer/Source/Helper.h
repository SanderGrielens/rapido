/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        Helper.h

  Description: A Common helper class. Mostly used to define any constants and common methods.
               

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


#ifndef HELPER_H
#define HELPER_H

#include <QString>
#include <VimbaCPP/Include/VimbaCPPCommon.h>
#include <QSharedPointer>

typedef struct 
{
    QSharedPointer<unsigned char>   m_pThreadFrame;
    VmbPixelFormatType              m_ThreadFormat;
    VmbUint32_t                     m_nThreadFrameWidth;
    VmbUint32_t                     m_nThreadFrameHeight; 
    VmbUint32_t                     m_nThreadFrameSize; 
    bool                            m_ColorInterpolation;

} tFrameInfo;

enum VimbaViewerLogCategory
{
    VimbaViewerLogCategory_OK      = 0,
    VimbaViewerLogCategory_WARNING = 1,
    VimbaViewerLogCategory_ERROR   = 2,
    VimbaViewerLogCategory_INFO    = 3,
};

class Helper
{
    public: 

    protected:

    private:

    public:
                Helper ( void );
               ~Helper ( void );

               QString parseToIPAddressLittleEndian      ( const QString sHexValue  );
               QString displayValueToIPv4             ( const QString sDecValue );
               QString mapReturnCodeToString          ( const VmbError_t &nReturnCode);
               QString convertFormatToString          ( const VmbPixelFormatType &format);
               /* constants */
               static const QString         m_GIGE_MULTICAST_IP_ADDRESS;
               static const QString         m_GIGE_CURRENT_IP_ADDRESS;
               static const QString         m_GIGE_CURRENT_SUBNET_MASK;
               static const QString         m_GIGE_CURRENT_DEFAULT_GATEWAY;
               static const QString         m_GIGE_PERSISTENT_DEFAULT_GATEWAY;
               static const QString         m_GIGE_PERSISTENT_IP_ADDRESS;
               static const QString         m_GIGE_PERSISTENT_SUBNET_MASK;
               static const QString         m_GIGE_DEVICE_MAC_ADDRESS;
               static const QString         m_GIGE_STAT_FRAME_DELIVERED;
               static const QString         m_GIGE_STAT_FRAME_DROPPED;
               static const QString         m_GIGE_STAT_FRAME_RATE;
               static const QString         m_GIGE_STAT_FRAME_RESCUED;
               static const QString         m_GIGE_STAT_FRAME_SHOVED;
               static const QString         m_GIGE_STAT_FRAME_UNDERRUN;
               static const QString         m_GIGE_STAT_LOCAL_RATE;
               static const QString         m_GIGE_STAT_PACKET_ERRORS;
               static const QString         m_GIGE_STAT_PACKET_MISSED;
               static const QString         m_GIGE_STAT_PACKET_RECEIVED;
               static const QString         m_GIGE_STAT_PACKET_REQUESTED;
               static const QString         m_GIGE_STAT_PACKET_RESENT;
               static const QString         m_GIGE_STAT_TIME_ELAPSED;

               static const QString         m_EXPOSURE_AUTO;
               static const QString         m_GAIN_AUTO;
               static const QString         m_BALANCE_WHITE_AUTO;

    protected:

    private:

};

#endif