/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FrameObserver.h

  Description: Frame callback.
               

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


#ifndef FRAMEOBSERVER_H
#define FRAMEOBSERVER_H

#include <QObject>
#include <QImage>
#include <QVector>
#include <QTime>
#include <QThread>
#include <QStringList>
#include "Helper.h"
#include "Histogram/HistogramThread.h"
#include <VimbaCPP/Include/IFrameObserver.h>
#include <VimbaCPP/Include/Frame.h>
#include <VimbaCPP/Include/Camera.h>

/* Number of frames in use to count the first FPS since start*/
const unsigned int MAX_FRAMES_TO_COUNT = 50;

using namespace AVT::VmbAPI;

class HelperThread : public QThread
{
    Q_OBJECT
    
    public:

    protected:
             tFrameInfo m_tFrameInfo;
    private:
             Helper   *m_Helper;
    public:
             HelperThread ( ){ m_Helper = new Helper(); };
            ~HelperThread ( void ){};

            void setThreadFrame ( QSharedPointer<unsigned char> pFrame, VmbUint32_t &nWidth, VmbUint32_t &nHeight, VmbPixelFormatType Format, bool ColorInterpolation  )
            {
                m_tFrameInfo.m_pThreadFrame         = pFrame;
                m_tFrameInfo.m_ThreadFormat         = Format;
                m_tFrameInfo.m_nThreadFrameWidth    = nWidth;
                m_tFrameInfo.m_nThreadFrameHeight   = nHeight;
                m_tFrameInfo.m_ColorInterpolation   = ColorInterpolation;
            }

    private:

    protected:
            virtual void run();

    private:

    signals:
            void frameReadyFromThread ( QImage image, const QString &sFormat, const QString &sHeight, const QString &sWidth );
};

class FrameObserver : public QObject, public AVT::VmbAPI::IFrameObserver
{
    Q_OBJECT

    public:

    protected:

    private:
            CameraPtr                           m_pCam;
            QTime                               m_FrameIntervalTimer;
            QTime                               m_FrameCounterTimer;
            Helper                             *m_Helper;

            QSharedPointer<unsigned char>       m_pFrame;
            QSharedPointer<HistogramThread>     m_pHistogramThread;

            QString                             m_sFormat;
            QString                             m_sHeight;
            QString                             m_sWidth;

            VmbPixelFormatType                  m_Format;
            VmbUint32_t                         m_nWidth;
            VmbUint32_t                         m_nHeight;
            VmbUint32_t                         m_nSize;

            unsigned int                        m_nFrames;
            unsigned int                        m_nFramesCounter;
            unsigned int                        m_DisplayInterval;
            unsigned int                        m_nNumberOfFPSToAvgCalc;
            double                              m_dAvgFPS;
            double                              m_dFramesPerSecond;
            double                              m_dFPS;
            double                              m_dFPS_NoImgProc;
            unsigned int                        m_nImageProcAverageDuration;
            unsigned int                        m_nImageProcDuration;
            unsigned int                        m_nMaximumFramesToCount;
            unsigned int                        m_nNumberOfFrames;
            unsigned int                        m_nCountPosition;

            QSharedPointer<HelperThread>        m_pHelperThread;
            bool                                m_bIsReset;

            /* Saving Raw Data */
            unsigned int                        m_nRawImagesToSave;
            unsigned int                        m_nRawImagesCounter;
            QString                             m_sPathDestination;
            QString                             m_sRawImagesName;

            /* Histogram */
            bool                                m_bIsHistogramEnabled;
            bool                                m_bColorInterpolation;
    public:
            FrameObserver ( CameraPtr pCam );
           ~FrameObserver ();

            virtual void FrameReceived          ( const AVT::VmbAPI::FramePtr frame );
            void resetFrameCounter              ( const bool &bIsRestart );
            void setDisplayInterval             ( const unsigned int &nInterval );
            void saveRawData                    ( const unsigned int &nNumberOfRawImagesToSave, const QString &sPath, const QString &sFileName);
            void enableHistogram                ( const bool &bIsHistogramEnabled );
            void setColorInterpolation          ( const bool &bState);
            bool getColorInterpolation          ( void );
            
    protected:
           
    private:
            void processFrame                   ( void );
            bool setFrame                       ( const AVT::VmbAPI::FramePtr &frame );
            void countFPS                       ( void );
            void imageProcessing                ( const AVT::VmbAPI::FramePtr &frame );
            void startHistogramThread           ( void );
            
    private slots:
            void getFrameFromThread             ( QImage image, const QString &sFormat, const QString &sHeight, const QString &sWidth );
            void getHistogramDataFromThread     ( const QVector<QVector <quint32> > &histData, const QString &sHistogramTitle, 
                                                  const double &nMaxHeight_YAxis, const double &nMaxWidth_XAxis, const QVector <QStringList> &statistics );
    signals:
            void frameReadyFromObserver         ( QImage image, const QString &sFormat, const QString &sHeight, const QString &sWidth );
            void setCurrentFPS                  ( const QString &sFPS );
            void setFrameCounter                ( const unsigned int &nFrame );
            void histogramDataFromObserver      ( const QVector<QVector <quint32> > &histData, const QString &sHistogramTitle, 
                                                  const double &nMaxHeight_YAxis, const double &nMaxWidth_XAxis, const QVector <QStringList> &statistics );
};


#endif