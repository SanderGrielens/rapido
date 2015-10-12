/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FeatureObserver.cpp

  Description: A notification whenever feature state or value has been changed
               

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


#include "FeatureObserver.h"


FeatureObserver::FeatureObserver ( CameraPtr pCam ) : m_bIsEventRunning ( false )
{ 
    m_pCam = pCam;
    m_Timer = new QTimer(this);
    connect(m_Timer, SIGNAL(timeout()), this, SLOT(checkSendEvent()));
}

FeatureObserver::~FeatureObserver ( void )
{

}

void FeatureObserver::startObserverTimer ( void )
{
    if( !m_Timer->isActive())
    {
        m_Timer->start(500);
    }
}

void FeatureObserver::stopObserverTimer ( void )
{
    if( m_Timer->isActive())
    {
        m_Timer->stop();
    }
}

void FeatureObserver::checkSendEvent ( void )
{
    emit setEventMessage(m_Message);
    m_Message.clear();
    stopObserverTimer();
    m_bIsEventRunning = false;
}

bool FeatureObserver::isEventRunning ( void )
{
    return m_bIsEventRunning;
}

void FeatureObserver::FeatureChanged ( const AVT::VmbAPI::FeaturePtr &feature )
{
    if ( feature != NULL )
    {
        std::string stdName("");

        VmbError_t error = feature->GetDisplayName(stdName);
        if(stdName.empty())
        {
            error = feature->GetName(stdName);
        }

        QString sName = QString::fromStdString(stdName);

        if(VmbErrorSuccess == error)    
        {
            VmbFeatureDataType pDataType = VmbFeatureDataUnknown;
            error = feature->GetDataType(pDataType);

            VmbInt64_t    nValue64    = 0;
            double        dValue        = 0;
            bool        bValue        = false;
            
            QString sValue("");
            std::string stdValue;

            if(VmbErrorSuccess == error)
            {
                switch(pDataType)
                {
                case VmbFeatureDataInt: 
                    if(VmbErrorSuccess == feature->GetValue(nValue64))
                    {
                        sValue = QString::number(nValue64);
                        isEventFeature(sName, nValue64);
                    }
                    break;

                case VmbFeatureDataFloat:
                    if(VmbErrorSuccess == feature->GetValue(dValue))
                        sValue = QString::number(dValue);
                    break;

                case VmbFeatureDataEnum:
                    if(VmbErrorSuccess == feature->GetValue(stdValue))
                        sValue = QString::fromStdString(stdValue);
                    break;

                case VmbFeatureDataString:
                    if(VmbErrorSuccess == feature->GetValue(stdValue))
                        sValue = QString::fromStdString (stdValue);
                    break;

                case VmbFeatureDataBool:
                    if(VmbErrorSuccess == feature->GetValue(bValue))
                        bValue ? sValue = "true" : sValue = "false";
                    break;

                case VmbFeatureDataCommand: 
                    sValue = "[COMMAND]";
                    break;

                case VmbFeatureDataRaw: 
                    sValue = "Click here to open";
                    break;

                default: break;

                }

                bool bIsWritable = false;
                if( VmbErrorSuccess == feature->IsWritable(bIsWritable) )
                {
                    emit setChangedFeature(sName, sValue, bIsWritable );
                }
            }
        }
    }
}

bool FeatureObserver::isEventFeature(const QString sName, const VmbInt64_t    nValue)
{
    if(0 ==sName.compare("EventAcquisitionEnd"))
    {
        sendEventMessage("EventAcquisitionEndTimestamp", "EventAcquisitionEndFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventAcquisitionRecordTrigger"))
    {
        sendEventMessage("EventAcquisitionRecordTriggerTimestamp", "EventAcquisitionRecordTriggerFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventAcquisitionStart"))
    {
        sendEventMessage("EventAcquisitionStartTimestamp", "EventAcquisitionStartFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventError"))
    {
        sendEventMessage("EventErrorTimestamp", "EventErrorFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventExposureEnd"))
    {
        sendEventMessage("EventExposureEndTimestamp", "EventExposureEndFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventFrameTrigger"))
    {
        sendEventMessage("EventFrameTriggerTimestamp", "EventFrameTriggerFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventFrameTriggerReady"))
    {
        sendEventMessage("EventFrameTriggerReadyTimestamp", "EventFrameTriggerReadyFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventLine1FallingEdge"))
    {
        sendEventMessage("EventLine1FallingEdgeTimestamp", "EventLine1FallingEdgeFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventLine1RisingEdge"))
    {
        sendEventMessage("EventLine1RisingEdgeTimestamp", "EventLine1RisingEdgeFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventLine2FallingEdge"))
    {
        sendEventMessage("EventLine2FallingEdgeTimestamp", "EventLine2FallingEdgeFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventLine2RisingEdge"))
    {
        sendEventMessage("EventLine2RisingEdgeTimestamp", "EventLine2RisingEdgeFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventLine3FallingEdge"))
    {
        sendEventMessage("EventLine3FallingEdgeTimestamp", "EventLine3FallingEdgeFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventLine3RisingEdge"))
    {
        sendEventMessage("EventLine3RisingEdgeTimestamp", "EventLine3RisingEdgeFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventLine4FallingEdge"))
    {
        sendEventMessage("EventLine4FallingEdgeTimestamp", "EventLine4FallingEdgeFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventLine4RisingEdge"))
    {
        sendEventMessage("EventLine4RisingEdgeTimestamp", "EventLine4RisingEdgeFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventOverflow"))
    {
        sendEventMessage("EventOverflowTimestamp", "EventOverflowFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventPtpSyncLocked"))
    {
        sendEventMessage("EventPtpSyncLockedTimestamp", "EventPtpSyncLockedFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    if(0 ==sName.compare("EventPtpSyncLost"))
    {
        sendEventMessage("EventPtpSyncLostTimestamp", "EventPtpSyncLostFrameID", 
        QString::number(nValue), sName);
        return true;
    }

    return false;
}

void FeatureObserver::sendEventMessage ( const QString &sFeatTimestamp, const QString &sFeatFrameID, 
                                         const QString &sValue, const QString &sFeature )
{
    AVT::VmbAPI::FeaturePtr pPtr1, pPtr2;
    VmbInt64_t nVal1 = 0, nVal2 = 0;
    static int i = 0;

    if( VmbErrorSuccess == m_pCam->GetFeatureByName(sFeatTimestamp.toAscii().data() ,pPtr1) &&
        VmbErrorSuccess == m_pCam->GetFeatureByName(sFeatFrameID.toAscii().data() ,pPtr2) )
    {
        if( VmbErrorSuccess == pPtr1->GetValue(nVal1) && VmbErrorSuccess == pPtr2->GetValue(nVal2) )
        {
            ++i;
            
            QString sMessage =  sFeature;
            sMessage.append(" (").append(sValue).append(")     ").append("\t").append("Timestamp: ").
                     append(QString::number(nVal1)).append(", FrameID: ").append(QString::number(nVal2));

            m_Message << sMessage;
            m_bIsEventRunning = true;

            if( MAX_EVENTS == i )
            {
                emit setEventMessage(m_Message);
                m_Message.clear();
                i = 0;
            }
        }
    }
}