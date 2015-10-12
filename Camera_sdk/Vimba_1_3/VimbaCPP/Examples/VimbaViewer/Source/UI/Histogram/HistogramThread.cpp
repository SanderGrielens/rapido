/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        HistogramThread.cpp

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


#include "stdint.h"
#include <algorithm>
#include <exception>
#include "HistogramThread.h"
#include <QStringList>

#define MIN( a, b ) (((a) < (b)) ? (a) : (b))
#define MAX( a, b ) (((a) > (b)) ? (a) : (b))

template <class T>
void DeleteArray(T *pArray)
{
    delete [] pArray;
}

HistogramThread::HistogramThread ( )
{

}

HistogramThread::~HistogramThread ( )
{

}

void HistogramThread::setThreadFrame ( QSharedPointer<unsigned char> pFrame, VmbUint32_t &nWidth, VmbUint32_t &nHeight, VmbUint32_t &nSize, VmbPixelFormatType m_Format )
{
    m_pHistogramFrame = QSharedPointer<unsigned char>(new unsigned char[nSize], DeleteArray<unsigned char>);
    memcpy(m_pHistogramFrame.data(), pFrame.data(), nSize);

    m_tFrameInfo.m_pThreadFrame          = pFrame;
    m_tFrameInfo.m_ThreadFormat          = m_Format;
    m_tFrameInfo.m_nThreadFrameWidth     = nWidth;
    m_tFrameInfo.m_nThreadFrameHeight    = nHeight;
    m_tFrameInfo.m_nThreadFrameSize      = nSize;
}

void HistogramThread::run()
{
    switch(m_tFrameInfo.m_ThreadFormat)
    {
        /* Monochrome, 8 bits */
    case VmbPixelFormatMono8: 
        histogramMono8();
        break;

        // Monochrome, 10 bits in 16 bits
    case VmbPixelFormatMono10:
        histogramNotSupportedYet();
        break;

        // Monochrome, 12 bits in 16 bits
    case VmbPixelFormatMono12:
        histogramNotSupportedYet();
        break;

        // Monochrome, 2x12 bits in 24 bits
    case VmbPixelFormatMono12Packed:
        histogramNotSupportedYet();
        break;

        // Monochrome, 14 bits in 16 bits
    case VmbPixelFormatMono14:
        histogramNotSupportedYet();
        break;

        // Monochrome, 16 bits
    case VmbPixelFormatMono16:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 8 bits
    case VmbPixelFormatBayerGR8:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 8 bits
    case VmbPixelFormatBayerRG8:
        histogramBayerRG8();
        break;

        // Bayer-color, 8 bits
    case VmbPixelFormatBayerGB8:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 8 bits
    case VmbPixelFormatBayerBG8:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 10 bits in 16 bits
    case VmbPixelFormatBayerGR10:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 10 bits in 16 bits
    case VmbPixelFormatBayerRG10:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 10 bits in 16 bits
    case VmbPixelFormatBayerGB10:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 10 bits in 16 bits
    case VmbPixelFormatBayerBG10:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 12 bits in 16 bits
    case VmbPixelFormatBayerGR12:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 12 bits in 16 bits
    case VmbPixelFormatBayerRG12:
        histogramBayerRG12();
        break;

        // Bayer-color, 12 bits in 16 bits
    case VmbPixelFormatBayerGB12:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 12 bits in 16 bits
    case VmbPixelFormatBayerBG12:
        histogramNotSupportedYet();
        break;

        // Bayer-color, 12 bits in 16 bits
    case VmbPixelFormatBayerRG12Packed:
        histogramBayerRG12Packed();
        break;

    case VmbPixelFormatBayerGR12Packed:
        histogramNotSupportedYet();
        break;

    case VmbPixelFormatBayerGB12Packed:
        histogramNotSupportedYet();
        break;

    case VmbPixelFormatBayerBG12Packed:
        histogramNotSupportedYet();
        break;
        
    case VmbPixelFormatBayerGR16:
        histogramNotSupportedYet();
        break;

    case VmbPixelFormatBayerRG16:
        histogramNotSupportedYet();
        break;

    case VmbPixelFormatBayerGB16:
        histogramNotSupportedYet();
        break;

    case VmbPixelFormatBayerBG16:
        histogramNotSupportedYet();
        break;

        /* RGB, 8 bits x 3 */
    case VmbPixelFormatRgb8:
        m_sFormat = "RGB8";
        histogramRGB8();
        break;

        // BGR, 8 bits x 3
    case VmbPixelFormatBgr8:
        m_sFormat = "BGR8";
        histogramBGR8();
        break;

        // ARGB, 8 bits x 4
    case VmbPixelFormatArgb8:
        histogramNotSupportedYet();
        break;

        // RGBA, 8 bits x 4, legacy name
        /*
    case VmbPixelFormatRgba8:
        break;
        */
        // BGRA, 8 bits x 4
    case VmbPixelFormatBgra8:
        histogramNotSupportedYet();
        break;

        // RGB, 16 bits x 3
    case VmbPixelFormatRgb16:
        histogramNotSupportedYet();
        break;

        // YUV 411 with 8 bits
    case VmbPixelFormatYuv411:
        histogramYUV411();
        break;

        // YUV  422
    case VmbPixelFormatYuv422:
        histogramYUV422();
        break;

        // YUV 444
    case VmbPixelFormatYuv444:
        histogramYUV444();
        break;

    default:
        break;
    }
}

void HistogramThread::histogramMono8 ( void )
{
    QVector <QStringList> vStatistics;
    QStringList sStatistics;
    
    uint        nMin  = 255;
    uint        nMax  = 0;
    VmbUint16_t nMean = 0;

    QVector<QVector<quint32> > monoValues;
    QVector<quint32> values;
    values.resize(256);
    
    const unsigned char *lSrc = m_pHistogramFrame.data();
    const unsigned char *lDest = m_pHistogramFrame.data() + ( m_tFrameInfo.m_nThreadFrameWidth * m_tFrameInfo.m_nThreadFrameHeight);

    nMin = *std::min_element( lSrc, lDest );
    nMax = *std::max_element( lSrc, lDest );
    
    while ( lSrc < lDest )
    {
        ++values[ *lSrc ];
        ++lSrc;
    }

    monoValues.push_back(values);

    nMean = calcWeightedMean( values );

    sStatistics << "Y" <<  QString::number(nMin) << QString::number(nMax) << QString::number(nMean); 
    vStatistics.push_back(sStatistics);

    emit histogramDataFromThread ( monoValues, 
                                   QString("Mono8 (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
                                   " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
                                   (double) m_tFrameInfo.m_nThreadFrameSize,
                                   double(255),  vStatistics );
}

void HistogramThread::histogramRGB8 ( void )
{
    QVector <QStringList> vStatistics;
    QStringList redStatistics;
    QStringList blueStatistics;
    QStringList greenStatistics;

    uint        nRedMin  = 255;
    uint        nRedMax  = 0;
    VmbUint16_t nRedMean = 0;

    uint        nGreenMin  = 255;
    uint        nGreenMax  = 0;
    VmbUint16_t nGreenMean = 0;

    uint        nBlueMin  = 255;
    uint        nBlueMax  = 0;
    VmbUint16_t nBlueMean = 0;

    QVector<QVector<quint32> > RGBValues;
    QVector<quint32> redValues;
    QVector<quint32> greenValues;
    QVector<quint32> blueValues;

    redValues.resize(256);
    greenValues.resize(256);
    blueValues.resize(256);

    const unsigned char *lSrc = m_pHistogramFrame.data();
    const unsigned char *lDest = m_pHistogramFrame.data() + ( m_tFrameInfo.m_nThreadFrameSize);

    while ( lSrc < lDest )
    {
        // R
        ++redValues[ *lSrc ];
        nRedMin = MIN( nRedMin, *lSrc );
        nRedMax = MAX( nRedMax, *lSrc );

        // G
        ++lSrc;
        ++greenValues[ *lSrc ];
        nGreenMin = MIN( nGreenMin, *lSrc );
        nGreenMax = MAX( nGreenMax, *lSrc );

        // B
        ++lSrc;
        ++blueValues[ *lSrc ];
        nBlueMin = MIN( nBlueMin, *lSrc );
        nBlueMax = MAX( nBlueMax, *lSrc );

        // Move to Red
        ++lSrc;
    }

    RGBValues.push_back(redValues);
    RGBValues.push_back(greenValues);
    RGBValues.push_back(blueValues);

    nRedMean = calcWeightedMean( redValues );
    nGreenMean = calcWeightedMean( greenValues);
    nBlueMean = calcWeightedMean( blueValues );

    redStatistics   << "Red"   <<  QString::number(nRedMin)   << QString::number(nRedMax)   << QString::number(nRedMean); 
    greenStatistics << "Green" <<  QString::number(nGreenMin) << QString::number(nGreenMax) << QString::number(nGreenMean);
    blueStatistics  << "Blue"  <<  QString::number(nBlueMin)  << QString::number(nBlueMax)  << QString::number(nBlueMean);

    vStatistics.push_back(redStatistics);
    vStatistics.push_back(greenStatistics);
    vStatistics.push_back(blueStatistics);

    emit histogramDataFromThread ( RGBValues, 
                                   QString(m_sFormat + " (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
                                    " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
                                    (double) (m_tFrameInfo.m_nThreadFrameSize/3),
                                    double(255), vStatistics );
    
}

void HistogramThread::histogramBGR8 ( void )
{
    QVector <QStringList> vStatistics;
    QStringList redStatistics;
    QStringList blueStatistics;
    QStringList greenStatistics;

    uint        nRedMin  = 255;
    uint        nRedMax  = 0;
    VmbUint16_t nRedMean = 0;

    uint        nGreenMin  = 255;
    uint        nGreenMax  = 0;
    VmbUint16_t nGreenMean = 0;

    uint        nBlueMin  = 255;
    uint        nBlueMax  = 0;
    VmbUint16_t nBlueMean = 0;

    QVector<QVector<quint32> > RGBValues;
    QVector<quint32> redValues;
    QVector<quint32> greenValues;
    QVector<quint32> blueValues;

    redValues.resize(256);
    greenValues.resize(256);
    blueValues.resize(256);

    const unsigned char *lSrc = m_pHistogramFrame.data();
    const unsigned char *lDest = m_pHistogramFrame.data() + ( m_tFrameInfo.m_nThreadFrameSize);

    while ( lSrc < lDest )
    {
        // B
        ++blueValues[ *lSrc ];
        nBlueMin = MIN( nBlueMin, *lSrc );
        nBlueMax = MAX( nBlueMax, *lSrc );

        // G
        ++lSrc;
        ++greenValues[ *lSrc ];
        nGreenMin = MIN( nGreenMin, *lSrc );
        nGreenMax = MAX( nGreenMax, *lSrc );

        // R
        ++lSrc;
        ++redValues[ *lSrc ];
        nRedMin = MIN( nRedMin, *lSrc );
        nRedMax = MAX( nRedMax, *lSrc );

        // Move to Blue
        ++lSrc;
    }

    RGBValues.push_back(redValues);
    RGBValues.push_back(greenValues);
    RGBValues.push_back(blueValues);

    nRedMean = calcWeightedMean( redValues );
    nGreenMean = calcWeightedMean( greenValues);
    nBlueMean = calcWeightedMean( blueValues );

    redStatistics   << "Red"   <<  QString::number(nRedMin)   << QString::number(nRedMax)   << QString::number(nRedMean); 
    greenStatistics << "Green" <<  QString::number(nGreenMin) << QString::number(nGreenMax) << QString::number(nGreenMean);
    blueStatistics  << "Blue"  <<  QString::number(nBlueMin)  << QString::number(nBlueMax)  << QString::number(nBlueMean);

    vStatistics.push_back(redStatistics);
    vStatistics.push_back(greenStatistics);
    vStatistics.push_back(blueStatistics);

    emit histogramDataFromThread ( RGBValues, 
        QString(m_sFormat + " (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
        " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
        (double) (m_tFrameInfo.m_nThreadFrameSize/3),
        double(255), vStatistics );

}

void HistogramThread::histogramBayerRG8( void )
{
    QVector <QStringList> vStatistics;
    QStringList redStatistics;
    QStringList blueStatistics;
    QStringList greenStatistics;

    uint        nRedMin  = 255;
    uint        nRedMax  = 0;
    VmbUint16_t nRedMean = 0;

    uint        nGreenMin  = 255;
    uint        nGreenMax  = 0;
    VmbUint16_t nGreenMean = 0;

    uint        nBlueMin  = 255;
    uint        nBlueMax  = 0;
    VmbUint16_t nBlueMean  = 0;

    QVector<QVector<quint32> > RGBValues;
    QVector<quint32> redValues;
    QVector<quint32> greenValues;
    QVector<quint32> blueValues;

    redValues.resize(256);
    greenValues.resize(256);
    blueValues.resize(256);

    const unsigned char *lSrc = m_pHistogramFrame.data();

    for ( VmbUint32_t y=0; y<m_tFrameInfo.m_nThreadFrameHeight; y+=2 )
    {
        // RG Row
        for ( VmbUint32_t x=0; x<m_tFrameInfo.m_nThreadFrameWidth; x+=2 )
        {
            // Red
            ++redValues[ *lSrc ];
            nRedMin = MIN( nRedMin, *lSrc );
            nRedMax = MAX( nRedMax, *lSrc );

            // Green 1
            ++lSrc;
            ++greenValues [*lSrc ];
            nGreenMin = MIN( nGreenMin, *lSrc );
            nGreenMax = MAX( nGreenMax, *lSrc );

            // Move to Red
            ++lSrc;
        }
        
        // GB Row
        for ( VmbUint32_t x=0; x<m_tFrameInfo.m_nThreadFrameWidth; x+=2 )
        {
            // Green 2
            ++greenValues[ *lSrc ];
            nGreenMin = MIN( nGreenMin, *lSrc );
            nGreenMax = MAX( nGreenMax, *lSrc );

            // Blue
            ++lSrc;
            ++blueValues[ *lSrc ];
            nBlueMin = MIN( nBlueMin, *lSrc );
            nBlueMax = MAX( nBlueMax, *lSrc );

            // Move to Green
            ++lSrc;
        }
    }

    RGBValues.push_back(redValues);
    RGBValues.push_back(greenValues);
    RGBValues.push_back(blueValues);

    nRedMean = calcWeightedMean( redValues );
    nGreenMean = calcWeightedMean( greenValues);
    nBlueMean = calcWeightedMean( blueValues );

    redStatistics   << "Red"   <<  QString::number(nRedMin)   << QString::number(nRedMax)   << QString::number(nRedMean); 
    greenStatistics << "Green" <<  QString::number(nGreenMin) << QString::number(nGreenMax) << QString::number(nGreenMean);
    blueStatistics  << "Blue"  <<  QString::number(nBlueMin)  << QString::number(nBlueMax)  << QString::number(nBlueMean);

    vStatistics.push_back(redStatistics);
    vStatistics.push_back(greenStatistics);
    vStatistics.push_back(blueStatistics);

    emit histogramDataFromThread ( RGBValues, 
        QString("BayerRG8 (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
        " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
        (double) (m_tFrameInfo.m_nThreadFrameSize/3),
        double(255), vStatistics );
}

void HistogramThread::histogramBayerRG12( void )
{
    QVector <QStringList> vStatistics;
    QStringList redStatistics;
    QStringList blueStatistics;
    QStringList greenStatistics;

    uint        nRedMin  = 4095;
    uint        nRedMax  = 0;
    VmbUint16_t nRedMean = 0;

    uint        nGreenMin  = 4095;
    uint        nGreenMax  = 0;
    VmbUint16_t nGreenMean = 0;

    uint        nBlueMin  = 4095;
    uint        nBlueMax  = 0;
    VmbUint16_t nBlueMean = 0;

    QVector<QVector<quint32> > RGBValues;
    QVector<quint32> redValues;
    QVector<quint32> greenValues;
    QVector<quint32> blueValues;

    redValues.resize(4096);
    greenValues.resize(4096);
    blueValues.resize(4096);

    /*
        12 bit packed in 16 bit. Maximum: 0F FF (For safety we only use the low nibble of the MSB)
        Little Endian Layout for 16 bit Buffer RGGB macro pixel:    R_LSB | R_MSB | G_LSB | G_MSB
                                                                    G_LSB | G_MSB | B_LSB | B_MSB

        We work with 8 bit uchar pointers on 16 bit data. For reconstructing 16 bit shorts we 
        rather use bit wise operations than (short*)((void*)pCharBuffer).
    */

    uint16_t shortValue;
    const unsigned char *lSrc = m_pHistogramFrame.data();

    for ( VmbUint32_t y=0; y<m_tFrameInfo.m_nThreadFrameHeight; y+=2 )
    {
        // RG Row
        for ( VmbUint32_t x=0; x<m_tFrameInfo.m_nThreadFrameWidth; x+=2 )
        {
            // Red
            shortValue = ((*(lSrc+1) & 0x0F) << 8) + *lSrc;
            ++redValues[ shortValue ];
            nRedMin = MIN( nRedMin, shortValue );
            nRedMax = MAX( nRedMax, shortValue );

            // Green 1
            lSrc+=2;
            shortValue = ((*(lSrc+1) & 0x0F) << 8) + *lSrc;
            ++greenValues[ shortValue ];
            nGreenMin = MIN( nGreenMin, shortValue );
            nGreenMax = MAX( nGreenMax, shortValue );

            // Move to Red
            lSrc+=2;
        }

        // GB Row
        for ( VmbUint32_t x=0; x<m_tFrameInfo.m_nThreadFrameWidth; x+=2 )
        {
            // Green 2
            shortValue = ((*(lSrc+1) & 0x0F) << 8) + *lSrc;
            ++greenValues[ shortValue ];
            nGreenMin = MIN( nGreenMin, shortValue );
            nGreenMax = MAX( nGreenMax, shortValue );

            // Blue
            lSrc+=2;
            shortValue = ((*(lSrc+1) & 0x0F) << 8) + *lSrc;
            ++blueValues[ shortValue ];
            nBlueMin = MIN( nBlueMin, shortValue );
            nBlueMax = MAX( nBlueMax, shortValue );

            // Move to Green
            lSrc+=2;
        }
    }

    RGBValues.push_back(redValues);
    RGBValues.push_back(greenValues);
    RGBValues.push_back(blueValues);

    nRedMean = calcWeightedMean( redValues );
    nGreenMean = calcWeightedMean( greenValues);
    nBlueMean = calcWeightedMean( blueValues );

    redStatistics   << "Red"   <<  QString::number(nRedMin)   << QString::number(nRedMax)   << QString::number(nRedMean); 
    greenStatistics << "Green" <<  QString::number(nGreenMin) << QString::number(nGreenMax) << QString::number(nGreenMean);
    blueStatistics  << "Blue"  <<  QString::number(nBlueMin)  << QString::number(nBlueMax)  << QString::number(nBlueMean);

    vStatistics.push_back(redStatistics);
    vStatistics.push_back(greenStatistics);
    vStatistics.push_back(blueStatistics);

    emit histogramDataFromThread ( RGBValues, 
        QString("BayerRG12 (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
        " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
        (double) (m_tFrameInfo.m_nThreadFrameSize/4.5),
        double(4095), vStatistics );
}

void HistogramThread::histogramBayerRG12Packed( void )
{
    QVector <QStringList> vStatistics;
    QStringList redStatistics;
    QStringList blueStatistics;
    QStringList greenStatistics;

    uint        nRedMin  = 4095;
    uint        nRedMax  = 0;
    VmbUint16_t nRedMean = 0;

    uint        nGreenMin  = 4095;
    uint        nGreenMax  = 0;
    VmbUint16_t nGreenMean = 0;

    uint        nBlueMin  = 4095;
    uint        nBlueMax  = 0;
    VmbUint16_t nBlueMean = 0;

    QVector<QVector<quint32> > RGBValues;
    QVector<quint32> redValues;
    QVector<quint32> greenValues;
    QVector<quint32> blueValues;

    redValues.resize(4096);
    greenValues.resize(4096);
    blueValues.resize(4096);

    /*
        Buffer Layout in nibbles for RGGB macro pixel:      R_Hi R_Hi | G_Lo R_Lo | G_Hi G_Hi
                                                            G_Hi G_Hi | B_Lo G_Lo | B_Hi B_Hi

        Bit mask to get first nibble of 'mixed byte':       F0
        Bit mask to get second nibble of 'mixed byte':      0F

        First mask (if needed) then shift low nibbles to lsb, high nibbles to msb.
    */

    uint16_t unpackedValue;
    const unsigned char *lSrc = m_pHistogramFrame.data();

    for ( VmbUint32_t y=0; y<m_tFrameInfo.m_nThreadFrameHeight; y+=2 )
    {
        // RG Row
        for ( VmbUint32_t x=0; x<m_tFrameInfo.m_nThreadFrameWidth; x+=2 )
        {
            // Red
            unpackedValue = (*lSrc << 4) + (*(lSrc+1) & 0x0F);
            ++redValues[ unpackedValue ];
            nRedMin = MIN( nRedMin, unpackedValue );
            nRedMax = MAX( nRedMax, unpackedValue );

            // Green 1
            lSrc += 2;
            unpackedValue = (*lSrc << 4) + ((*(lSrc-1) & 0xF0) >> 4);
            ++greenValues[ unpackedValue ];
            nGreenMin = MIN( nGreenMin, unpackedValue );
            nGreenMax = MAX( nGreenMax, unpackedValue );

            // Move to Red
            ++lSrc;
        }
        
        // GB Row
        for ( VmbUint32_t x=0; x<m_tFrameInfo.m_nThreadFrameWidth; x+=2 )
        {
            // Green 2
            unpackedValue = (*lSrc << 4) + (*(lSrc+1) & 0x0F);
            ++greenValues[ unpackedValue ];
            nGreenMin = MIN( nGreenMin, unpackedValue );
            nGreenMax = MAX( nGreenMax, unpackedValue );

            // Blue
            lSrc += 2;
            unpackedValue = (*lSrc << 4) + ((*(lSrc-1) & 0xF0) >> 4);
            ++blueValues[ unpackedValue ];
            nBlueMin = MIN( nBlueMin, unpackedValue );
            nBlueMax = MAX( nBlueMax, unpackedValue );

            // Move to Green
            ++lSrc;
        }
    }

    RGBValues.push_back(redValues);
    RGBValues.push_back(greenValues);
    RGBValues.push_back(blueValues);

    nRedMean = calcWeightedMean( redValues );
    nGreenMean = calcWeightedMean( greenValues);
    nBlueMean = calcWeightedMean( blueValues );

    redStatistics   << "Red"   <<  QString::number(nRedMin)   << QString::number(nRedMax)   << QString::number(nRedMean); 
    greenStatistics << "Green" <<  QString::number(nGreenMin) << QString::number(nGreenMax) << QString::number(nGreenMean);
    blueStatistics  << "Blue"  <<  QString::number(nBlueMin)  << QString::number(nBlueMax)  << QString::number(nBlueMean);

    vStatistics.push_back(redStatistics);
    vStatistics.push_back(greenStatistics);
    vStatistics.push_back(blueStatistics);

    emit histogramDataFromThread ( RGBValues, 
        QString("BayerRG12Packed (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
        " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
        (double) (m_tFrameInfo.m_nThreadFrameSize / 4.5),
        double(4095), vStatistics );
}

void HistogramThread::histogramYUV411 ( void )
{
    QVector <QStringList> Statistics;
    QStringList yStatistics;
    QStringList uStatistics;
    QStringList vStatistics;

    uint        nYMin  = 255;
    uint        nYMax  = 0;
    VmbUint16_t nYMean = 0;

    uint        nUMin  = 255;
    uint        nUMax  = 0;
    VmbUint16_t nUMean = 0;

    uint        nVMin  = 255;
    uint        nVMax  = 0;
    VmbUint16_t nVMean = 255;


    QVector<QVector<quint32> > YUVValues;
    QVector<quint32> YValues;
    QVector<quint32> UValues;
    QVector<quint32> VValues;

    YValues.resize(256);
    UValues.resize(256);
    VValues.resize(256);

    /*
        6 Bytes per 4 pixel, buffer layout YUV Y Y Y macro pixel:  U | Y | Y | V | Y | Y
        12 bpp
    */

    const unsigned char *lSrc = m_pHistogramFrame.data();
    const unsigned char *lDest = m_pHistogramFrame.data() + ( m_tFrameInfo.m_nThreadFrameSize);

    while (lSrc < lDest)
    {
        // U
        ++UValues[ *lSrc ];
        nUMin = MIN( nUMin, *lSrc );
        nUMax = MAX( nUMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[ *lSrc ];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[ *lSrc ];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // V
        ++lSrc;
        ++VValues[ *lSrc ];
        nVMin = MIN( nVMin, *lSrc );
        nVMax = MAX( nVMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[ *lSrc ];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[ *lSrc ];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // Move to U
        ++lSrc;
    }

    YUVValues.push_back(YValues);
    YUVValues.push_back(UValues);
    YUVValues.push_back(VValues);

    nYMean = calcWeightedMean( YValues );
    nUMean = calcWeightedMean( UValues );
    nVMean = calcWeightedMean( VValues );

    yStatistics   << "Y" <<  QString::number(nYMin) << QString::number(nYMax) << QString::number(nYMean); 
    uStatistics   << "U" <<  QString::number(nUMin) << QString::number(nUMax) << QString::number(nUMean);
    vStatistics   << "V" <<  QString::number(nVMin) << QString::number(nVMax) << QString::number(nVMean);

    Statistics.push_back(yStatistics);
    Statistics.push_back(uStatistics);
    Statistics.push_back(vStatistics);

    emit histogramDataFromThread (    YUVValues, 
        QString("YUV411 (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
        " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
        (double) (m_tFrameInfo.m_nThreadFrameSize/1.5),
        double(255), Statistics);
}

void HistogramThread::histogramYUV422 ( void )
{
    QVector <QStringList> Statistics;
    QStringList yStatistics;
    QStringList uStatistics;
    QStringList vStatistics;

    uint        nYMin  = 255;
    uint        nYMax  = 0;
    VmbUint16_t nYMean = 0;

    uint        nUMin  = 255;
    uint        nUMax  = 0;
    VmbUint16_t nUMean = 0;

    uint        nVMin  = 255;
    uint        nVMax  = 0;
    VmbUint16_t nVMean = 255;


    QVector<QVector<quint32> > YUVValues;
    QVector<quint32> YValues;
    QVector<quint32> UValues;
    QVector<quint32> VValues;

    YValues.resize(256);
    UValues.resize(256);
    VValues.resize(256);

    /*
        8 Bytes per 4 pixel, buffer layout YUV Y YUV Y macro pixel:  U | Y | V | Y | U | Y | V | Y
        16 bpp
    */

    const unsigned char *lSrc = m_pHistogramFrame.data();
    const unsigned char *lDest = m_pHistogramFrame.data() + ( m_tFrameInfo.m_nThreadFrameSize);

    while ( lSrc < lDest )
    {
        // U
        ++UValues[ *lSrc ];
        nUMin = MIN( nUMin, *lSrc );
        nUMax = MAX( nUMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[*lSrc];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // V
        ++lSrc;
        ++VValues[*lSrc];
        nVMin = MIN( nVMin, *lSrc );
        nVMax = MAX( nVMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[*lSrc];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // U
        ++lSrc;
        ++UValues[*lSrc];
        nUMin = MIN( nUMin, *lSrc );
        nUMax = MAX( nUMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[*lSrc];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // V
        ++lSrc;
        ++VValues[*lSrc];
        nVMin = MIN( nVMin, *lSrc );
        nVMax = MAX( nVMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[*lSrc];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // Move to U
        ++lSrc;
    }

    YUVValues.push_back(YValues);
    YUVValues.push_back(UValues);
    YUVValues.push_back(VValues);

    nYMean = calcWeightedMean( YValues );
    nUMean = calcWeightedMean( UValues );
    nVMean = calcWeightedMean( VValues );

    yStatistics   << "Y" <<  QString::number(nYMin) << QString::number(nYMax) << QString::number(nYMean); 
    uStatistics   << "U" <<  QString::number(nUMin) << QString::number(nUMax) << QString::number(nUMean);
    vStatistics   << "V" <<  QString::number(nVMin) << QString::number(nVMax) << QString::number(nVMean);

    Statistics.push_back(yStatistics);
    Statistics.push_back(uStatistics);
    Statistics.push_back(vStatistics);

    emit histogramDataFromThread (    YUVValues, 
        QString("YUV422 (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
        " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
        (double) (m_tFrameInfo.m_nThreadFrameSize/2),
        double(255), Statistics);

}

void HistogramThread::histogramYUV444 ( void )
{
    QVector <QStringList> Statistics;
    QStringList yStatistics;
    QStringList uStatistics;
    QStringList vStatistics;

    uint        nYMin  = 255;
    uint        nYMax  = 0;
    VmbUint16_t nYMean = 0;

    uint        nUMin  = 255;
    uint        nUMax  = 0;
    VmbUint16_t nUMean = 0;

    uint        nVMin  = 255;
    uint        nVMax  = 0;
    VmbUint16_t nVMean = 255;


    QVector<QVector<quint32> > YUVValues;
    QVector<quint32> YValues;
    QVector<quint32> UValues;
    QVector<quint32> VValues;

    YValues.resize(256);
    UValues.resize(256);
    VValues.resize(256);

    /*
        3 Bytes per pixel, buffer layout YUV macro pixel:  U | Y | V
        24 bpp
    */

    const unsigned char *lSrc = m_pHistogramFrame.data();
    const unsigned char *lDest = m_pHistogramFrame.data() + ( m_tFrameInfo.m_nThreadFrameSize);

    while ( lSrc < lDest )
    {
        // U
        ++UValues[ *lSrc ];
        nUMin = MIN( nUMin, *lSrc );
        nUMax = MAX( nUMax, *lSrc );

        // Y
        ++lSrc;
        ++YValues[ *lSrc ];
        nYMin = MIN( nYMin, *lSrc );
        nYMax = MAX( nYMax, *lSrc );

        // V
        ++lSrc;
        ++VValues[ *lSrc ];
        nVMin = MIN( nVMin, *lSrc );
        nVMax = MAX( nVMax, *lSrc );

        // Move to U
        ++lSrc;
    }

    YUVValues.push_back(YValues);
    YUVValues.push_back(UValues);
    YUVValues.push_back(VValues);

    nYMean = calcWeightedMean( YValues );
    nUMean = calcWeightedMean( UValues );
    nVMean = calcWeightedMean( VValues );

    yStatistics   << "Y" <<  QString::number(nYMin) << QString::number(nYMax) << QString::number(nYMean); 
    uStatistics   << "U" <<  QString::number(nUMin) << QString::number(nUMax) << QString::number(nUMean);
    vStatistics   << "V" <<  QString::number(nVMin) << QString::number(nVMax) << QString::number(nVMean);

    Statistics.push_back(yStatistics);
    Statistics.push_back(uStatistics);
    Statistics.push_back(vStatistics);

    emit histogramDataFromThread (  YUVValues, 
                                    QString("YUV444 (" + QString::number(m_tFrameInfo.m_nThreadFrameHeight) + 
                                    " x " + QString::number(m_tFrameInfo.m_nThreadFrameWidth) + ")"  ),
                                    (double) (m_tFrameInfo.m_nThreadFrameSize/3),
                                    double(255), Statistics);

}

void HistogramThread::histogramNotSupportedYet ( void )
{
    QVector <QStringList> vStatistics;
    QVector<QVector<quint32> > noValues;
    QVector<quint32> values(256, 0);
    
    noValues.push_back(values);
    emit histogramDataFromThread (    noValues, 
                                    QString("No Histogram Support For This Format Yet"),
                                    (double) m_tFrameInfo.m_nThreadFrameSize,
                                    double(255), vStatistics );
}

VmbUint16_t HistogramThread::calcWeightedMean( QVector<VmbUint32_t>& rValues )
{
    if( rValues.empty() )
    {
        throw std::exception();
    }
    VmbUint32_t res = 0;
    VmbUint32_t nNumPixels = 0;

    for ( VmbUint32_t i=0; i<rValues.size(); ++i )
    {
        if( 0 < rValues.at( i ))
        {
            res += i * rValues.at( i );
            nNumPixels += rValues.at( i );
        }
    }

    return (VmbUint16_t)(res / nNumPixels);
}
