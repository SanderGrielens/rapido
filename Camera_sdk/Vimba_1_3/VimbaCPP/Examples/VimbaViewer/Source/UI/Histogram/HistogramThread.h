/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        HistogramThread.h

  Description: a worker thread to  process histogram data

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


#ifndef HISTOGRAMTHREAD_H
#define HISTOGRAMTHREAD_H

#include <QTime>
#include <QThread>
#include <QVector>

#include "Helper.h"

#include <VimbaCPP/Include/IFrameObserver.h>
#include <VimbaCPP/Include/Frame.h>
#include <VimbaCPP/Include/Camera.h>



using namespace AVT::VmbAPI;

class HistogramThread : public QThread
{
    Q_OBJECT

    public:

    protected:
          tFrameInfo m_tFrameInfo;

    private:
          QVector<unsigned char*> m_HistFrame;
          QSharedPointer<unsigned char> m_pHistogramFrame;
          QString  m_sFormat;
    public:
          HistogramThread ( );
         ~HistogramThread ( void );

          void setThreadFrame ( QSharedPointer<unsigned char> pFrame, VmbUint32_t &nWidth, VmbUint32_t &nHeight, VmbUint32_t &nSize, VmbPixelFormatType m_Format );
    

    protected:
          virtual void run();
          void histogramMono8           ( void );
          void histogramRGB8            ( void );
          void histogramBGR8            ( void );
          void histogramYUV411          ( void );
          void histogramYUV422          ( void );
          void histogramYUV444          ( void );
          void histogramNotSupportedYet ( void );
          void histogramBayerRG12Packed ( void );
          void histogramBayerRG8        ( void );
          void histogramBayerRG12       ( void );

    private:
          VmbUint16_t calcWeightedMean( QVector<VmbUint32_t>& rValues );
          
    signals:
          void histogramDataFromThread ( const QVector<QVector <quint32> > &histData, 
                                         const QString &sHistogramTitle, 
                                         const double &nMaxHeight_YAxis, 
                                         const double &nMaxWidth_XAxis,
                                         const QVector <QStringList> &statistics);
};

#endif