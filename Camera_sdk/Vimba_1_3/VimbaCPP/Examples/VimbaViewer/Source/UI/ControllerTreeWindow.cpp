
/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ControllerTreeWindow.cpp

  Description: All about features control tree 

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

/* define this to use std::numeric_limits */
#define NOMINMAX

#include <limits>
#include <iostream>

#include "ControllerTreeWindow.h"
#include "Helper.h"
#include <FeatureObserver.h>

#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif

ControllerTreeWindow::ControllerTreeWindow ( QString sID, QWidget *parent, bool bAutoAdjustPacketSize, CameraPtr pCam ): QTreeView ( parent ), m_TreeDelegate ( NULL ),
    m_HBooleanLayout          ( NULL ), m_HBooleanLayout2          ( NULL ), m_BooleanWidget         ( NULL ), m_CheckBox_Bool   ( NULL ),
    m_HEditLayout             ( NULL ), m_HEditLayout2             ( NULL ), m_EditWidget            ( NULL ), m_TextEdit_String ( NULL ),
    m_HButtonLayout           ( NULL ), m_HButtonLayout2           ( NULL ), m_ButtonWidget          ( NULL ), m_CmdButton       ( NULL ),
    m_HComboLayout            ( NULL ), m_HComboLayout2            ( NULL ), m_ComboWidget           ( NULL ), m_EnumComboBox    ( NULL ),
    m_HLineEditLayout         ( NULL ), m_HLineEditLayout2         ( NULL ), m_LineEditWidget        ( NULL ), m_LineEdit        ( NULL ), m_ConfirmButton( NULL ), m_HexLabel ( NULL ),
    m_HSpinSliderLayout_Int   ( NULL ), m_HSpinSliderLayout_Int2   ( NULL ), m_IntSpinSliderWidget   ( NULL ), m_SpinBox_Int     ( NULL ), m_Slider_Int   ( NULL ),
    m_HSpinSliderLayout_Float ( NULL ), m_HSpinSliderLayout_Float2 ( NULL ), m_FloatSpinSliderWidget ( NULL ), m_SpinBox_Float   ( NULL ), m_Slider_Float ( NULL ),
    m_HexWindow               ( NULL ), m_LogSliderWidget          ( NULL ),
    m_nSliderStep ( 0 ), m_dMinimum ( 0 ), m_dMaximum ( 0 ), m_nIntSliderOldValue ( 0 ), m_bIsGigE ( false ), m_bIsTimeout ( false ),
    m_pFeatureObs( new FeatureObserver(pCam)), m_ListTreeError ( VmbErrorOther ), m_bIsTooltipOn ( true ), m_bIsJobDone ( true ), m_bIsMousePressed ( false ), m_bIsBusy ( false )
{
    m_pCam = pCam;                    
    m_sCameraID = sID;
    setupTree();    
    connect((FeatureObserver*)(SP_ACCESS(m_pFeatureObs)), SIGNAL(setEventMessage(const QStringList &)), this, SLOT(onSetEventMsg(const QStringList &)));

    m_Level.append(m_Level0Map);
    m_Level.append(m_Level1Map);
    m_Level.append(m_Level2Map);
    m_Level.append(m_Level3Map);
    m_Level.append(m_Level4Map);
    m_Level.append(m_Level5Map);
    m_Level.append(m_Level6Map);
    m_Level.append(m_Level7Map);
    m_Level.append(m_Level8Map);
    m_Level.append(m_Level9Map);
    
    QStringList CompletionList;

    VmbError_t error = m_pCam->GetFeatures(m_featPtrVec);
    if(VmbErrorSuccess == error)
    {
        for( unsigned int i=0; i < m_featPtrVec.size(); i++ )
        {
            VmbFeatureVisibilityType visibilityType;
            m_featPtrVec.at(i)->GetVisibility(visibilityType);

            std::string sDisplayName;
            m_featPtrVec.at(i)->GetDisplayName(sDisplayName);

            /* Save DisplayName for filter completer */
            CompletionList << QString::fromStdString(sDisplayName);

            if((VmbFeatureVisibilityBeginner == visibilityType) || (VmbFeatureVisibilityExpert == visibilityType) || (VmbFeatureVisibilityGuru == visibilityType))
            {
                sortCategoryAndAttribute(m_featPtrVec.at(i), m_ModelGuru);      
                mapInformation(QString::fromStdString(sDisplayName), m_featPtrVec.at(i));
                m_featPtrMap[QString::fromStdString(sDisplayName)] = m_featPtrVec.at(i);
                std::string sFeatureName;
                m_featPtrVec.at(i)->GetName(sFeatureName);
                m_DisplayFeatureNameMap[QString::fromStdString(sDisplayName)] = QString::fromStdString(sFeatureName);
            }
        }
    }
    
    /* Features string completer for filter */
    m_StringCompleter = new MultiCompleter(CompletionList, this);
    m_StringCompleter->setCaseSensitivity(Qt::CaseInsensitive);

    m_ListTreeError = error; 
    m_Model = m_ModelGuru;
   
    m_ProxyModel = new SortFilterProxyModel(this);
    m_ProxyModel->setDynamicSortFilter(true);
    setModel(m_ProxyModel);
    m_ProxyModel->setSourceModel(m_Model);

    resizeColumnToContents(0);
    /* used to set the maximum height of the row in tree */
    m_TreeDelegate = new ItemDelegate();
    m_TreeDelegate->setRowHeight(20);
    setItemDelegate(m_TreeDelegate);

    connect( this, SIGNAL( expanded(const QModelIndex &) ), this, SLOT( expand(const QModelIndex &) ) );  
    connect( this, SIGNAL( collapsed(const QModelIndex &) ), this, SLOT( collapse(const QModelIndex &) ) );
    connect((FeatureObserver*)(SP_ACCESS(m_pFeatureObs)), SIGNAL(setChangedFeature(const QString &, const QString &, const bool &)), 
            this, SLOT(onSetChangedFeature(const QString &, const QString &, const bool &)));

    sortByColumn(0, Qt::AscendingOrder);
    setSelectionMode(QAbstractItemView::SingleSelection);

    /* Auto Adjust Packet Size */
    m_bAutoAdjustPacketSize = bAutoAdjustPacketSize;
    if(m_bIsGigE && m_bAutoAdjustPacketSize)
    {
        FeaturePtr FeatPtr = getFeaturePtr("GVSP Adjust Packet Size");
        if( NULL == FeatPtr)
            return;

        error = FeatPtr->RunCommand();
        if(VmbErrorSuccess == error)
        {
            QTimer::singleShot(7000, this, SLOT(setAdjustPacketSizeTimeout()));
            
            bool bIsDone = false;
            while((!bIsDone) && (VmbErrorSuccess == error))
            {
                QCoreApplication::processEvents();
                if(m_bIsTimeout)
                {
                    m_ListTreeError = 7000; //7000: "Timeout"
                    return;
                }
                error = FeatPtr->IsCommandDone(bIsDone);
            }

            m_ListTreeError = error; 
        }
    }

    m_FeaturesPollingTimer = new QTimer ( this );
    connect (m_FeaturesPollingTimer, SIGNAL(timeout()), this, SLOT(pollingFeaturesValue()));
    m_FeaturesPollingTimer->start(500);
}

ControllerTreeWindow::~ControllerTreeWindow ( void )
{
    m_FeaturesPollingTimer->stop();
    resetControl();
}

void ControllerTreeWindow::saveFeaturesToTextFile ( const QString &sDestPathAndFileName )
{
    QFile file(sDestPathAndFileName);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream outputstream (&file);

    QMap<QString, FeaturePtr>::iterator i;
   
    for (i = m_featPtrMap.begin(); i != m_featPtrMap.end(); ++i)
    {
        QString sFeatureDisplayName = i.key();
        outputstream << sFeatureDisplayName << " = " << getFeatureValue ( i.value()) << "\n"; 
    }

    file.close();
}

void ControllerTreeWindow::setAdjustPacketSizeTimeout ( void )
{
    m_bIsTimeout = true;
}

void ControllerTreeWindow::pollingFeaturesValue ( void )
{
    for (unsigned int i = 0; i < m_FeaturesPollingName.size(); i++)
    {
        FeaturePtr FeatPtr = getFeaturePtr(m_FeaturesPollingName.at(i));
        
        VmbUint32_t nPollValue = 0;
        if (VmbErrorSuccess == FeatPtr->GetPollingTime( nPollValue ))
        {
            if(0 == nPollValue)
            {
                updateExpandedTreeValue(FeatPtr, m_FeaturesPollingName.at(i));
            }
        }
    }
}

bool ControllerTreeWindow::isGigE (void)
{
    return (m_bIsGigE && m_bAutoAdjustPacketSize);
}

VmbError_t ControllerTreeWindow::getTreeStatus( void )
{
    return m_ListTreeError;
}

MultiCompleter *ControllerTreeWindow::getListCompleter( void )
{
    return m_StringCompleter;
}

void ControllerTreeWindow::showTooltip ( const bool &bIsChecked )
{
    m_bIsTooltipOn = bIsChecked;
    m_bIsTooltipOn ? this->setToolTip(m_sTooltip):this->setToolTip("");
}

void ControllerTreeWindow::mapInformation(const QString sName, const FeaturePtr &featPtr)
{
    if( NULL == featPtr)
        return; 

    std::string sDescription;
    featPtr->GetDescription(sDescription);

    QString sQDescription = "<br/>";
    sQDescription.append(QString::fromStdString(sDescription));
    sQDescription = sQDescription.replace(".", ".<br/>");

    QString sInfo = getFeatureInformation(featPtr);
    if(sInfo.isEmpty())
    {
        m_DescriptonMap[sName] = sQDescription;
    }
    else
    {
        if(sDescription.empty())
            m_DescriptonMap[sName] = "<b>DESCRIPTION:</b> N/A<br/>" + sInfo;
        else
            m_DescriptonMap[sName] = "<b>DESCRIPTION:</b>" + sQDescription.append("<br/>").append(sInfo);
    }
}

QString ControllerTreeWindow::getFeatureInformation(const FeaturePtr &featPtr)
{
    QString sInformation;
    VmbInt64_t nMin = 0, nMax = 0, nInc = 0;
    double dMin = 0, dMax = 0;

    std::string sFeatureName;
    featPtr->GetName(sFeatureName);
    sInformation.append("<b>FEATURE NAME:</b> ").append(QString::fromStdString(sFeatureName)).append("<br/>");

    VmbFeatureVisibilityType visibility = VmbFeatureVisibilityUnknown;
    featPtr->GetVisibility(visibility);
    sInformation.append("<b>VISIBILITY:</b> ");
    switch(visibility)
    {
    case VmbFeatureVisibilityUnknown:   sInformation.append("UNKNOWN<br/>");   break;
    case VmbFeatureVisibilityBeginner:  sInformation.append("BEGINNER<br/>");  break;
    case VmbFeatureVisibilityExpert:    sInformation.append("EXPERT<br/>");    break;
    case VmbFeatureVisibilityGuru:      sInformation.append("GURU<br/>");      break;
    case VmbFeatureVisibilityInvisible: sInformation.append("INVISIBLE<br/>"); break;
    default: sInformation.append("N/A<br/>"); break;
    }
    
    /* get Feature Type */
    VmbFeatureDataType dataType = VmbFeatureDataUnknown;
    VmbError_t error = featPtr->GetDataType(dataType);
    if(VmbErrorSuccess == error)
    {
        switch(dataType)
        {
        case VmbFeatureDataInt: 
            if( VmbErrorSuccess == featPtr->GetRange(nMin, nMax))
                sInformation.append("<b>TYPE:</b> Integer<br/><b>MINIMUM:</b> ").append(QString::number(nMin)).append("<br/><b>MAXIMUM:</b> ").append(QString::number(nMax).append("<br/>"));

            if( VmbErrorSuccess == featPtr->GetIncrement(nInc))
                sInformation.append("<b>INTERVAL:</b> ").append(QString::number(nInc)).append("<br/>");
            break;
        case VmbFeatureDataFloat:
            if( VmbErrorSuccess == featPtr->GetRange(dMin, dMax))
                sInformation.append("<b>TYPE:</b> Float<br/><b>MINIMUM:</b> ").append(QString::number(dMin,'g',16)).append("<br/><b>MAXIMUM:</b> ").append(QString::number(dMax,'g',16)).append("<br/>");
            break;
        case VmbFeatureDataEnum:    sInformation.append("<b>TYPE:</b> Enumeration<br/>"); break;
        case VmbFeatureDataString:  sInformation.append("<b>TYPE:</b> String<br/>");      break;
        case VmbFeatureDataBool:    sInformation.append("<b>TYPE:</b> Boolean<br/>");     break;
        case VmbFeatureDataCommand: sInformation.append("<b>TYPE:</b> Command<br/>");     break;
        case VmbFeatureDataRaw:     sInformation.append("<b>TYPE:</b> DataRaw<br/>");     break;
        default: break;
        }
    }

    std::string sCategory;
    featPtr->GetCategory(sCategory);
    sInformation.append("<b>CATEGORY:</b> ").append(QString::fromStdString(sCategory)).append("<br/>");

    FeaturePtrVector           featPtrVec;

    sInformation.append("<br/><b>AFFECTED FEATURE(S):</b> ");
    featPtr->GetAffectedFeatures(featPtrVec);
    for(unsigned int i=0; i < featPtrVec.size(); i++)
    {
        std::string sName;
        featPtrVec.at(i)->GetName(sName);
        
        if(0 == i)
            sInformation.append("<br/>");

        sInformation.append(QString::fromStdString(sName));
        if(i+1 != featPtrVec.size())
        {
            sInformation.append(", ");
            if(0 == ((i+1) % 4) && (i != 0) )
                sInformation.append("<br/>");
        }
    }
    
    if(0 == featPtrVec.size())
        sInformation.append("N/A");

    sInformation.append("<br/>");
    return sInformation;
}

void ControllerTreeWindow::collapse ( const QModelIndex & index )
{ 
    QVariant varData = index.data();
    int nrow = 0;
    QVariant varChild = 1;

    while(0 != varChild.type())
    {
        QModelIndex child = index.child(nrow++, 0);

        if (0 == child.data().type())
            break;
        
        varChild = child.data();

        if(!isEventFeature(varChild.toString()))
        {
            unregisterFeatureObserver(varChild.toString());

            for (unsigned int i = 0; i < m_FeaturesPollingName.size(); i++)
            {
                if (m_FeaturesPollingName.at(i) == varChild.toString())
                {
                    m_FeaturesPollingName.removeAt(i);
                }
            }
        }

        collapse(child);
    }
}

void ControllerTreeWindow::expand ( const QModelIndex & index )
{ 
    QVariant varData = index.data();
    int nrow = 0;
    QVariant varChild = 1;

    while(0 != varChild.type())
    {
        QModelIndex child = index.child(nrow++, 0);
        if (0 == child.data().type())
            break;
        varChild = child.data();
        
        if(isExpanded(child))
            expand(child);

        if(!isEventFeature(varChild.toString()))
        {
            if(registerFeatureObserver (varChild.toString()))
                m_FeaturesPollingName.push_back(varChild.toString());
        }   
    }

    resizeColumnToContents(0);  
    resizeColumnToContents(1);  
}

void ControllerTreeWindow::updateRegisterFeature ( void )
{
    for( unsigned int i=0; i < model()->rowCount(); i++)
    {
        expand(model()->index(i, 0));
    }
}

void ControllerTreeWindow::updateUnRegisterFeature ( void )
{
    for( unsigned int i=0; i < model()->rowCount(); i++)
    {
        collapse(model()->index(i, 0));
    }
}

bool ControllerTreeWindow::isEventFeature (const QString &sFeature)
{
    if( 0 == sFeature.compare("EventAcquisitionEnd")   || 0 == sFeature.compare("EventAcquisitionRecordTrigger") ||
        0 == sFeature.compare("EventAcquisitionStart") || 0 == sFeature.compare("EventError")                    ||
        0 == sFeature.compare("EventExposureEnd")      || 0 == sFeature.compare("EventFrameTrigger")             ||
        0 == sFeature.compare("EventLine1FallingEdge") || 0 == sFeature.compare("EventLine1RisingEdge")          ||
        0 == sFeature.compare("EventLine2FallingEdge") || 0 == sFeature.compare("EventLine2RisingEdge")          ||
        0 == sFeature.compare("EventLine3FallingEdge") || 0 == sFeature.compare("EventLine3RisingEdge")          ||
        0 == sFeature.compare("EventLine4FallingEdge") || 0 == sFeature.compare("EventLine4RisingEdge")          ||
        0 == sFeature.compare("EventOverflow")         || 0 == sFeature.compare("EventPtpSyncLocked")            ||
        0 == sFeature.compare("EventPtpSyncLost")      || 0 == sFeature.compare("EventFrameTriggerReady") )
    {
        return true;
    }

    return false;
}

void ControllerTreeWindow::setupTree ( void )
{
    m_ModelGuru = new QStandardItemModel();
    m_Model = new QStandardItemModel();
    QStringList sHeader;
    sHeader << "Feature " << "Value ";
    m_Model->setHorizontalHeaderLabels( sHeader );
    m_ModelGuru->setHorizontalHeaderLabels( sHeader );
}

bool  ControllerTreeWindow::findCategory ( QMap <QString, QString> map, const QString sName )
{
    QMap<QString, QString>::const_iterator itr = map.find(sName);
    while (itr != map.constEnd()) 
    {
        if(0 == itr.key().compare(sName))
        {
            return true;
        }

        ++itr;
    }

    return false;
}

void ControllerTreeWindow::sortCategoryAndAttribute ( const FeaturePtr &featPtr, QStandardItemModel *Model )
{
    QFont categoryFont;
    categoryFont.setBold(true);

    QList<QStandardItem *> items;
    QList<QStandardItem *> pParent;
    QString sLastNode;
    bool bHasLastNode = false;
    unsigned int nLevel = 0;

    std::string sCategory, sFeatureName;
    QString sFeatureValue;
    bool bIsWritable = false;
    VmbFeatureVisibilityType visibilityType;
    VmbError_t error;
    
    error = featPtr->GetDisplayName(sFeatureName);

    if( VmbErrorSuccess == featPtr->GetCategory(sCategory) &&
        VmbErrorSuccess == error )
    {    
        /* TODO check return value */
        sFeatureValue = getFeatureValue(featPtr);
        error = featPtr->IsWritable(bIsWritable);
        featPtr->GetVisibility(visibilityType);
        
        /* is it a GigE? : temporarily use to handle floating gain */
        if(0 == sCategory.compare("/GigE"))
        {
            m_bIsGigE = true;
        }

        // Feature directly in root category
        if ( 0 == sCategory.length() )
        {
            sCategory = "/";
        }
    }
    else
    {
        return;
    }

    QString sPath = QString::fromStdString(sCategory);

    int nCount =  sPath.count('/');

    while((nLevel < nCount))
    {
        nLevel++;
        QString sPart = sPath.section('/', nLevel, nLevel);
        // Feature directly in root category
        if ( 0 == sPart.length() )
        {
            sPart = "/";
        }
        bool bIsNodeExist = false;
        if( 0 < sPart.length())
        {
            /* check if category in level already available */
            if(!findCategory ( m_Level.at(nLevel-1), sPart ))
            {
                bIsNodeExist = false; 
                switch(nLevel-1)
                {
                case 0: m_Level0Map[sPart] = "true";  /* TOP LEVEL */
                    m_Level[0] = m_Level0Map;
                    break;
                case 1:m_Level1Map[sPart] = "true"; 
                    m_Level[1] = m_Level1Map;
                    break;
                case 2:m_Level2Map[sPart] = "true"; 
                    m_Level[2] = m_Level2Map;
                    break;
                case 3:m_Level3Map[sPart] = "true"; 
                    m_Level[3] = m_Level3Map;
                    break;
                case 4:m_Level4Map[sPart] = "true"; 
                    m_Level[4] = m_Level4Map;
                    break;
                case 5:m_Level5Map[sPart] = "true";
                    m_Level[5] = m_Level5Map;
                    break;
                case 6:m_Level6Map[sPart] = "true";
                    m_Level[6] = m_Level6Map;
                    break;
                case 7:m_Level7Map[sPart] = "true";
                    m_Level[7] = m_Level7Map;
                    break;
                case 8:m_Level8Map[sPart] = "true";
                    m_Level[8] = m_Level8Map;
                    break;
                }
            }
            else
            {
                bIsNodeExist = true;
            }
            
            /* if category/feature not available in tree yet, then fill it */
            if( !bIsNodeExist ) 
            {
                QList<QStandardItem *> currentItem;
            
                currentItem << new QStandardItem(sPart) << new QStandardItem("");
                currentItem.at(0)->setEditable(false);
                currentItem.at(1)->setEditable(false);
                
                if(bHasLastNode)
                {
                    /* get the pointer of list items */
                    items = Model->findItems(sLastNode, Qt::MatchRecursive); 
                    if(0 != items.size())
                    {   
                        /* find the right parent level, and add the attribute item there*/
                        QString sGrandParent = sPath.section('/', nLevel-1, nLevel-1);
                        items.at(getGrandParentLevel(items, sGrandParent))->appendRow(currentItem);
                        sLastNode = sPart;
                        continue;
                    }
                }

                if(pParent.isEmpty())
                { 
                    pParent << new QStandardItem(sPart) << new QStandardItem("");
                    pParent.at(0)->setFont(categoryFont);
                    pParent.at(0)->setEditable(false);
                    pParent.at(1)->setEditable(false);

                    Model->appendRow(pParent);

                    sLastNode = sPart;          
                    continue;
                }
    
                /* use the last node as parent if available */
                items = Model->findItems(sLastNode, Qt::MatchRecursive); 
                if(0 != items.size())
                {       
                    /* find the right parent level, and add the attribute item there*/
                    QString sGrandParent = sPath.section('/', nLevel-1, nLevel-1);
                    items.at(getGrandParentLevel(items, sGrandParent))->appendRow(currentItem);
                }
                else
                {
                    pParent.at(0)->appendRow(currentItem);
                }
            }
            /* if already available use it as parent node */
            else
            {
                bHasLastNode = true;
            }

            sLastNode = sPart;          
        }
    }
    
    /* get the pointer of list items */
    items = Model->findItems(sLastNode, Qt::MatchWrap|Qt::MatchRecursive); 
    
    if(0 != items.size() && (VmbFeatureVisibilityInvisible != visibilityType))
    {
        QList<QStandardItem *> attributeItems;
        
        std::string sStdName;
        featPtr->GetName(sStdName);
        QString sName = QString::fromStdString(sStdName);

        if( 0 == sName.compare(Helper::m_GIGE_CURRENT_IP_ADDRESS)      || 
            0 == sName.compare(Helper::m_GIGE_CURRENT_SUBNET_MASK)     ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY) ||
            0 == sName.compare(Helper::m_GIGE_MULTICAST_IP_ADDRESS)    ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_IP_ADDRESS)   || 
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_SUBNET_MASK)  ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY))
        {
            Helper toolHelper;
            sFeatureValue = toolHelper.displayValueToIPv4(sFeatureValue);
        }
        
        attributeItems << new QStandardItem(QString::fromStdString(sFeatureName)) << new QStandardItem(sFeatureValue);
        
        attributeItems.at(0)->setEditable(false);
        attributeItems.at(1)->setEditable(false);

        bIsWritable ? attributeItems.at(0)->setForeground(QColor(0,128, 0))/* green */ : attributeItems.at(0)->setForeground(QColor(135,135, 135)) /* grey */;

        /* find the right parent level, and add the attribute item there*/
        QString sGrandParent = sPath.section('/', nCount-1, nCount-1);
        items.at(getGrandParentLevel(items, sGrandParent))->appendRow(attributeItems);

        /* find increment out */
        VmbFeatureDataType dataType = VmbFeatureDataUnknown;
        if( VmbErrorSuccess ==  featPtr->GetDataType(dataType) )
        {
            if( VmbFeatureDataFloat == dataType )
            {
                countFloatingIncrement(featPtr);
            }
        }

        /* register Events */
        if(isEventFeature(QString::fromStdString(sFeatureName)))
        {
            featPtr->RegisterObserver(m_pFeatureObs); 
        }   
    }
}

unsigned int ControllerTreeWindow::getGrandParentLevel( QList<QStandardItem *> items, const QString &sGrandParent)
{
    unsigned int nLevel = 0;
    for (unsigned int i=0; i<items.size(); i++ )
    {
        QStandardItem *parent = items.at(i)->parent();

        if(0 == parent)
            continue;

        if(0 == sGrandParent.compare(parent->text()) )
        {
            nLevel = i;
            break;
        }
    }

    return nLevel;
}

void ControllerTreeWindow::countFloatingIncrement ( const FeaturePtr &featPtr )
{
    std::string sFeatureName;
    featPtr->GetDisplayName(sFeatureName);

    if( 0 == sFeatureName.compare("Gamma"))
    { 
        m_FloatFeatureIncrementMap[QString::fromStdString(sFeatureName)] = 0.1;
        return;
    }

    double dMin = 0, dMax = 0;

    if(VmbErrorSuccess ==  featPtr->GetRange(dMin, dMax))
    {
        if( dMax < 0xFFFF )
        {
            double dCurrent = 0;
            bool bIsEverythingOk = false;

            /* keep the current value that will set back at the end*/
            if( VmbErrorSuccess == featPtr->GetValue(dCurrent))
            { 
                double dTmpValue = dMax - 0.1;
                if( VmbErrorSuccess == featPtr->SetValue(dTmpValue))
                {
                    double dTmpCurrent = 0;
                    if ( VmbErrorSuccess == featPtr->GetValue(dTmpCurrent))
                    {  
                        /* if rounded down */
                        if(dTmpCurrent < dTmpValue)
                        {
                            double dDelta = dMax - dTmpCurrent;
                            m_FloatFeatureIncrementMap[QString::fromStdString(sFeatureName)] = dDelta;
                            bIsEverythingOk = true;
                        }
                        /* if rounded up */
                        else
                        {
                            if( VmbErrorSuccess == featPtr->SetValue(dMin + 0.1))
                            {
                                dTmpCurrent = 0;
                                if( VmbErrorSuccess == featPtr->GetValue(dTmpCurrent))
                                {
                                    m_FloatFeatureIncrementMap[QString::fromStdString(sFeatureName)] = (dTmpCurrent-dMin);
                                    bIsEverythingOk = true;
                                }
                            }
                        }
                        /* set back current value  */
                        featPtr->SetValue(dCurrent);
                    }
                }
            }

            if(!bIsEverythingOk)
            { /* otherwise just set 0.1 as default */
                m_FloatFeatureIncrementMap[QString::fromStdString(sFeatureName)] = 0.1;
            }
        }
    }
}
            
bool ControllerTreeWindow::registerFeatureObserver ( const QString &sFeatureName )
{
    /* register all features to the observer, so you will get notification in case any features value has been changed */
    QMap<QString, FeaturePtr>::iterator i;
    bool bResult = false;

    for (i = m_featPtrMap.begin(); i != m_featPtrMap.end(); ++i)
    {   
        if( 0 == i.key().compare(sFeatureName))
        {
            VmbError_t error = i.value()->RegisterObserver(m_pFeatureObs); 
            if( VmbErrorSuccess != error)
            {
                emit logging( "ERROR Register Observer - Feature: " + i.key() + " returned "+QString::number(error)+
                              " "+ m_Helper.mapReturnCodeToString(error)) ;
            }
            else
            {
                bResult = true;
            }
            
            updateExpandedTreeValue(i.value(), sFeatureName);
            break;
        }
    }

    return bResult;
}

void ControllerTreeWindow::unregisterFeatureObserver ( const QString &sFeatureName )
{
    QMap<QString, FeaturePtr>::iterator i;

    for (i = m_featPtrMap.begin(); i != m_featPtrMap.end(); ++i)
    {
        if( 0 == i.key().compare(sFeatureName))
        {
            VmbError_t error = i.value()->UnregisterObserver(m_pFeatureObs); 
            if( VmbErrorSuccess != error)
            {
                if(VmbErrorNotFound != error )
                    emit logging( "ERROR Unregister Observer - Feature: " + i.key() + " returned "+QString::number(error)+" "+ 
                                   m_Helper.mapReturnCodeToString(error)) ;
            }

            break;
        }
    }
}

void ControllerTreeWindow::updateExpandedTreeValue ( const FeaturePtr &featPtr, const QString &sName )
{
    QString sValue = getFeatureValue(featPtr);
    bool bIsWritable = false;
    if( VmbErrorSuccess == featPtr->IsWritable(bIsWritable) )
        onSetChangedFeature(sName, sValue, bIsWritable);
}

void ControllerTreeWindow::onSetEventMsg ( const QStringList &sMsg )
{
    setEventMessage(sMsg);
}

/* synchronize the event information in tree and event window 
   make sure to collect last information for event window when acquisition stops */
void ControllerTreeWindow::synchronizeEventFeatures ( void )
{
    if(m_bIsGigE)
    {
        if(((FeatureObserver*)(SP_ACCESS(m_pFeatureObs)))->isEventRunning())
        {
            ((FeatureObserver*)(SP_ACCESS(m_pFeatureObs)))->startObserverTimer();
        }
    }
}

/* put the value to related features changed
   keep tree features value up-to-date  */
void ControllerTreeWindow::onSetChangedFeature ( const QString &sFeature, const QString &sValue, const bool &bIsWritable )
{   
    QModelIndexList currentItems = m_Model->match( m_Model->index(0,0), Qt::DisplayRole, QVariant::fromValue(sFeature), 1, Qt::MatchWrap|Qt::MatchRecursive);

    if( 0 == currentItems.size() )
        return;
    
    QString sFeatureValue = sValue;
  
    if( 0 == sFeature.compare("Multicast IP Address")      ||
        0 == sFeature.compare("Current Subnet Mask")       || 
        0 == sFeature.compare("Current IP Address")        ||
        0 == sFeature.compare("Current Default Gateway")   ||
        0 == sFeature.compare("Persistent IP Address")     ||
        0 == sFeature.compare("Persistent Subnet Mask")    || 
        0 == sFeature.compare("Persistent Default Gateway")||
        0 == sFeature.compare("GevCurrentIPAddress")       || 
        0 == sFeature.compare("GevCurrentSubnetMask")      ||
        0 == sFeature.compare("GevCurrentDefaultGateway")  ||
        0 == sFeature.compare("GevPersistentDefaultGateway") ||
        0 == sFeature.compare("GevPersistentIPAddress")    || 
        0 == sFeature.compare("GevPersistentSubnetMask") )
    {
        Helper toolHelper;
        sFeatureValue = toolHelper.displayValueToIPv4(sValue);
    }
  
    QModelIndex newIndex = m_Model->index( currentItems.at(0).row(), 1, currentItems.at(0).parent());
    m_Model->setData( newIndex, QVariant(sFeatureValue), Qt::EditRole );

    //TLParamsLock? 
    QList<QStandardItem *> itemsCol1 = m_Model->findItems(sFeature, Qt::MatchWrap|Qt::MatchRecursive, 0) ;

    if(!itemsCol1.empty())
    {
        bIsWritable ? itemsCol1.at(0)->setForeground(QColor(0,128, 0))/*green*/ : itemsCol1.at(0)->setForeground(QColor(135,135, 135))/*grey*/;     
        if( 0 == m_sCurrentSelectedFeature.compare(sFeature))
            enableWidget(bIsWritable);
    }

    /* update Exposure-/Gain-/BalanceWhite- Auto when "Once" clicked */
    if( ((0 == m_sCurrentSelectedFeature.compare(Helper::m_EXPOSURE_AUTO)) && (0 == sFeature.compare(Helper::m_EXPOSURE_AUTO)) && ( 0 == sFeatureValue.compare("Off"))) ||
        ((0 == m_sCurrentSelectedFeature.compare(Helper::m_GAIN_AUTO)) && (0 == sFeature.compare(Helper::m_GAIN_AUTO)) && ( 0 == sFeatureValue.compare("Off"))) ||
        ((0 == m_sCurrentSelectedFeature.compare(Helper::m_BALANCE_WHITE_AUTO)) && (0 == sFeature.compare(Helper::m_BALANCE_WHITE_AUTO)) && ( 0 == sFeatureValue.compare("Off")))||
        ((0 == m_sCurrentSelectedFeature.compare("BalanceWhiteAuto")) && (0 == sFeature.compare("BalanceWhiteAuto")) && ( 0 == sFeatureValue.compare("Off"))))
    {
        if( (NULL == m_FeaturePtr_EnumComboBox) || (NULL == m_EnumComboBox) )
            return;

        if (0 == m_EnumComboBox->currentText().compare("Once")) 
        {
            int nIndex = m_EnumComboBox->findText("Off");
            if(-1 != nIndex)
                m_EnumComboBox->setCurrentIndex(nIndex);
        }
    }

    if( (NULL != m_FeaturePtr_EnumComboBox) && (NULL != m_EnumComboBox) )
    {
        if(0 == m_sCurrentSelectedFeature.compare(sFeature) && sValue.isEmpty())
        {
            m_EnumComboBox->setItemText(m_EnumComboBox->count(), sValue);
            m_EnumComboBox->setCurrentIndex(m_EnumComboBox->count());
        }
    }
    
}

void ControllerTreeWindow::enableWidget ( const bool bIsWritable )
{
    if( NULL != m_IntSpinSliderWidget )
        m_IntSpinSliderWidget->setEnabled(bIsWritable);

    else if( NULL != m_ButtonWidget )
        m_ButtonWidget->setEnabled(bIsWritable);

    else if( NULL != m_ComboWidget )
        m_ComboWidget->setEnabled(bIsWritable);

    else if( NULL != m_FloatSpinSliderWidget )
        m_FloatSpinSliderWidget->setEnabled(bIsWritable);

    else if( NULL != m_BooleanWidget )
        m_BooleanWidget->setEnabled(bIsWritable);

    else if( NULL != m_EditWidget )
        m_EditWidget->setEnabled(bIsWritable);

    else if( NULL != m_LineEditWidget )
        m_LineEditWidget->setEnabled(bIsWritable);

    else if( NULL != m_LogSliderWidget )
        m_LogSliderWidget->setEnabled(bIsWritable);
}

QString ControllerTreeWindow::getFeatureValue ( const FeaturePtr &featPtr )
{
    VmbInt64_t  nValue64    = 0;
    double      dValue      = 0;
    bool        bValue      = false;
    
    std::string stdValue;
    QString     sValue("");
    
    VmbFeatureDataType dataType = VmbFeatureDataUnknown;

    VmbError_t error = featPtr->GetDataType(dataType);

    if(VmbErrorSuccess == error)
    {
        switch(dataType)
        {
            case VmbFeatureDataInt: 
                if(VmbErrorSuccess == featPtr->GetValue(nValue64))
                    sValue = QString::number(nValue64);
                break;

            case VmbFeatureDataFloat:
                if(VmbErrorSuccess == featPtr->GetValue(dValue))
                    sValue = QString::number(dValue);
                break;

            case VmbFeatureDataEnum:
                if(VmbErrorSuccess == featPtr->GetValue(stdValue))
                    sValue = QString::fromStdString(stdValue);
                break;

            case VmbFeatureDataString:
                if(VmbErrorSuccess == featPtr->GetValue(stdValue))
                    sValue = QString::fromStdString (stdValue);
                break;

            case VmbFeatureDataBool:
                if(VmbErrorSuccess == featPtr->GetValue(bValue))
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
    }
    return sValue;
}

QString ControllerTreeWindow::getFeatureName ( const QModelIndex item )
{
    QString sAttrName("");
    if(item.isValid())
    {
        QModelIndex indexFirstCol = item.sibling(item.row(), 0);
        sAttrName = indexFirstCol.model()->data( indexFirstCol, Qt::DisplayRole ).toString(); 
    }
    return sAttrName;
}

FeaturePtr ControllerTreeWindow::getFeaturePtr ( const QString &sFeature )
{
    QMap<QString, FeaturePtr>::iterator i = m_featPtrMap.find(sFeature);

    if ( m_featPtrMap.end() != i )
    {
        return i.value();
    }
    else
    {
        return FeaturePtr();
    }
}

double ControllerTreeWindow::getFloatFeatureIncrement ( const QString &sFeature )
{
    double dFloatValue = 0;
    QMap<QString, double>::const_iterator i = m_FloatFeatureIncrementMap.find(sFeature);

    while (i != m_FloatFeatureIncrementMap.constEnd()) 
    {
        if(0 == i.key().compare(sFeature))
        {
            dFloatValue = i.value();
            break;
        }

        ++i;
    }

    return dFloatValue;
}

void ControllerTreeWindow::currentChanged ( const QModelIndex & current, const QModelIndex & previous )
{  
    setCursor(Qt::ArrowCursor);
    scrollTo(current, QAbstractItemView::EnsureVisible);
    
    if( m_bIsBusy )
        return;

    m_bIsBusy = true;

    if( !current.isValid())
    {
        m_bIsBusy = false;
        return;
    }

    /* check what feature is that */
    FeaturePtr FeatPtr;
    QString sFeature = getFeatureName(current);
    FeatPtr = getFeaturePtr(sFeature);

    /* make sure to udpdate the info */
    mapInformation(sFeature, FeatPtr); 
    showIt(current, "Description");
    
    if (( NULL == FeatPtr) || !isFeatureWritable(sFeature) || 0 == current.column())
    {
        m_bIsBusy = false;
        m_sCurrentSelectedFeature = sFeature;
        resetControl();
        return;
    }

    VmbFeatureDataType dataType = VmbFeatureDataUnknown;
    if(VmbErrorSuccess == FeatPtr->GetDataType(dataType))
    {
        switch(dataType)
        {
        case VmbFeatureDataInt:     createIntSliderSpinBox(current);   break;
        case VmbFeatureDataFloat:   createFloatSliderSpinBox(current); break;
        case VmbFeatureDataEnum:    createEnumComboBox(current);       break;
        case VmbFeatureDataString:  createStringEditBox(current);      break;
        case VmbFeatureDataBool:    createBooleanCheckBox(current);    break;
        case VmbFeatureDataCommand: createCommandButton(current);      break;
        case VmbFeatureDataRaw:     createHexEditor(current);          break;
        default: break;
        }
    }

    m_bIsBusy = false;
}

void ControllerTreeWindow::showIt ( const QModelIndex item, const QString &sWhat )
{
    QString sAttrName = getFeatureName(item);

    QMap<QString, QString>::const_iterator i;

    if( 0 == sWhat.compare("Description"))
        i = m_DescriptonMap.find(sAttrName);

    while (i != m_DescriptonMap.constEnd()) 
    {
        if(0 == i.key().compare(sAttrName))
        {
            m_sTooltip = i.value();
            if(m_bIsTooltipOn)
            {
#ifdef WIN32
                this->setToolTip(i.value());
#else
                this->setToolTip( "<span style=\"background-color:black;color:white\">"+i.value()+"</span>" );
#endif
            }
            else
                this->setToolTip("");

            emit setDescription(i.value());
            return;
        }
        ++i;
    }
    /* no feature description has been found */
    this->setToolTip("");
    emit setDescription("");        
}

QString ControllerTreeWindow::getFeatureNameFromMap ( const QString &sDisplayName )
{
    QString sFeature;
    QMap<QString, QString>::const_iterator itr = m_DisplayFeatureNameMap.find(sDisplayName); 
    while (itr != m_DisplayFeatureNameMap.constEnd()) 
    {
        if(0 == itr.key().compare(sDisplayName))
        {
            sFeature = itr.value();
            break;
        }

        ++itr;
    }

    return sFeature;
}

bool ControllerTreeWindow::isFeatureWritable ( const QString & sFeature )
{
    bool bIsWritable = false;
    FeaturePtr FeatPtr;

    FeatPtr = getFeaturePtr(sFeature);
    if(FeaturePtr() == FeatPtr)
        return bIsWritable;

    FeatPtr->IsWritable(bIsWritable);

    return bIsWritable;
}

/* VmbFeatureDataRaw */
void ControllerTreeWindow::createHexEditor ( const QModelIndex item )
{
    FeaturePtr FeatPtr; 
    QString sFeature = getFeatureName(item);

    bool bIsReadOnly = true;
    if (isFeatureWritable(sFeature))
    {
        bIsReadOnly = false;
    }
    FeatPtr = getFeaturePtr(sFeature);
    if( NULL == FeatPtr)
        return;

    if( VmbErrorSuccess == FeatPtr->GetValue(m_RawData) )
    {
        m_HexWindow = new HexMainWindow(this, NULL, "Raw Data Editor", bIsReadOnly, FeatPtr);
        unsigned char * c = &(m_RawData[0]);
        QByteArray data = QByteArray::fromRawData((char*)c, m_RawData.size());
        m_HexWindow->setData(data);
    }
}

/* VmbFeatureDataCommand */
void ControllerTreeWindow::createCommandButton ( const QModelIndex item )
{
    QString sFeature_Command = getFeatureName(item);
   
    if (isFeatureWritable(sFeature_Command))
    {
        m_sCurrentSelectedFeature = m_sFeature_Command = sFeature_Command;

        m_FeaturePtr_Command = getFeaturePtr(sFeature_Command);
        if(NULL == m_FeaturePtr_Command)
            return;

        resetControl();

        m_ButtonWidget  = new QWidget(this);
        m_HButtonLayout = new QHBoxLayout(m_ButtonWidget); 
        m_CmdButton = new QPushButton(QIcon(":/VimbaViewer/Images/execute.png"), QString("Execute..."), m_ButtonWidget); 

        m_HButtonLayout2 = new QHBoxLayout(); 
        m_HButtonLayout2->addWidget(m_CmdButton);
        m_HButtonLayout->addLayout(m_HButtonLayout2);

        QPoint p = QCursor::pos();      
        m_ButtonWidget->setFixedSize(m_sCurrentSelectedFeature.length()*10, 60);
        m_ButtonWidget->setWindowFlags( Qt::Tool | Qt::WindowCloseButtonHint );
        m_ButtonWidget->setWindowTitle(m_sCurrentSelectedFeature);
        m_ButtonWidget->show();
 
        m_ButtonWidget->move(p.x() + 30, p.y() - (m_ButtonWidget->height() + 50));
        
        /* out of screen? */
        QDesktopWidget desktop;
        QRect rect = m_ButtonWidget->geometry();
        int rightPos = rect.right();

        if( (p.x() + 10 + (rightPos-p.x())) > desktop.width() )
        {
            m_ButtonWidget->move(p.x()-(30+rect.width()), p.y() - (m_ButtonWidget->height() + 50));
        }
        
        QObject::connect( m_CmdButton, SIGNAL(clicked()), this, SLOT(onCmdButtonClick()) );    
        setCursor(Qt::PointingHandCursor);
    }
}

/* run command when feature button clicked */
void ControllerTreeWindow::onCmdButtonClick ( void )
{
    VmbError_t error;

    if(0 == m_sFeature_Command.compare("AcquisitionStop") || 0 == m_sFeature_Command.compare("Acquisition Stop") || 0 == m_sFeature_Command.compare("AcquisitionAbort"))
    {
        error = m_FeaturePtr_Command->RunCommand();
        emit acquisitionStartStop("AcquisitionStopFreerun"); 
    }
    else if(0 == m_sFeature_Command.compare("AcquisitionStart") || 0 == m_sFeature_Command.compare("Acquisition Start"))
    {
        emit acquisitionStartStop("AcquisitionStartFreerun"); 
    }
    
    if(0 != m_sFeature_Command.compare("AcquisitionStop") && 0 != m_sFeature_Command.compare("Acquisition Stop") && 0 != m_sFeature_Command.compare("AcquisitionAbort"))
    {
        error = m_FeaturePtr_Command->RunCommand();
    }
    
    if( VmbErrorSuccess != error)
    {
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " returned "+QString::number(error)+
                        " "+ m_Helper.mapReturnCodeToString(error)) ;
    }
    else if( 0 == m_sFeature_Command.compare("GVSP Adjust Packet Size"))
    {
        bool bIsDone = false;
        this->setEnabled(false);
        emit enableViewerMenu (false);
        QPixmap Pixmap( ":/VimbaViewer/Images/refresh.png" );
        SplashScreen Splash(Pixmap, this, Qt::SplashScreen);
        int nW = ((this->width()/2) - Splash.width()/2);
        int nH = ((this->height()/2) - Splash.height()/2);
        Splash.setGeometry(nW,nH, Splash.width(),Splash.height());
        Splash.show();
        Splash.showMessage("Please wait ..." , Qt::AlignHCenter | Qt::AlignVCenter, Qt::red);

        while((!bIsDone) && (VmbErrorSuccess == error))
        {
            error = m_FeaturePtr_Command->IsCommandDone(bIsDone);
        }
        
        this->setEnabled(true);
        emit enableViewerMenu (true);
    }
}

/* VmbFeatureDataEnum */
void ControllerTreeWindow::createEnumComboBox ( const QModelIndex item )
{
    VmbError_t error;
    QString sFeature_EnumComboBox = getFeatureName(item);

    // Don't create drop down for read only features
    if (isFeatureWritable(sFeature_EnumComboBox))
    {
        m_sCurrentSelectedFeature = m_sFeature_EnumComboBox = sFeature_EnumComboBox;

        // Set FeaturePtr member to currently selected enum feature
        m_FeaturePtr_EnumComboBox = getFeaturePtr(m_sFeature_EnumComboBox);
        if(FeaturePtr() == m_FeaturePtr_EnumComboBox)
            return;

        resetControl();
        m_ComboWidget   = new QWidget(this);
        m_HComboLayout  = new QHBoxLayout(m_ComboWidget);
        m_EnumComboBox  = new ExComboBox( ); 
        m_HComboLayout2 = new QHBoxLayout(); 
        m_HComboLayout2->addWidget(m_EnumComboBox);
        m_HComboLayout->addLayout(m_HComboLayout2);

        // Get all possible enumerations as string
        StringVector sEnumEntries;
        error = m_FeaturePtr_EnumComboBox->GetValues(sEnumEntries);

        if(VmbErrorSuccess == error)
        {
            std::string sCurrentValue;
            unsigned int nPos = 0;
            // Get currently selected enumeration
            if(VmbErrorSuccess == m_FeaturePtr_EnumComboBox->GetValue(sCurrentValue) || VmbErrorSuccess == error)
            {
                for(unsigned int i=0; i < sEnumEntries.size(); i++)
                {
                    // Check each enumeration if it is currently applicable
                    bool bIsAvailable = false;
                    m_FeaturePtr_EnumComboBox->IsValueAvailable(sEnumEntries.at(i).c_str(), bIsAvailable );

                    if(bIsAvailable)
                    {
                        // Add all applicable enumerations to drop down
                        m_EnumComboBox->addItem(QString::fromStdString(sEnumEntries.at(i)));
                        // Is this currently selected enumeration?
                        if(0 == sEnumEntries.at(i).compare(sCurrentValue))
                            // Set drop down index
                            m_EnumComboBox->setCurrentIndex(nPos);
                        nPos++;
                    }
                }

                if(sCurrentValue.empty())
                {
                    m_EnumComboBox->setItemText(nPos, QString::fromStdString(sCurrentValue));
                    m_EnumComboBox->setCurrentIndex(nPos);
                }
            }

            QPoint p = QCursor::pos();      
            m_ComboWidget->setFixedSize(m_sCurrentSelectedFeature.length()*15, 60);
            m_ComboWidget->setWindowFlags( Qt::Tool | Qt::WindowCloseButtonHint );
            m_ComboWidget->setWindowTitle(m_sCurrentSelectedFeature);
            m_ComboWidget->show();
            m_ComboWidget->move(p.x()+30, p.y() - (m_ComboWidget->height() + 50));

            /* out of screen? */
            QDesktopWidget desktop;
            QRect rect = m_ComboWidget->geometry();
            int rightPos = rect.right();

            if( (p.x() + 10 + (rightPos-p.x())) > desktop.width() )
            {
                m_ComboWidget->move(p.x()-(30+rect.width()), p.y() - (m_ComboWidget->height() + 50));
            }

            QObject::connect( m_EnumComboBox, SIGNAL(activated(const QString &)), this, SLOT(onEnumComboBoxClick(const QString &)) );
            setCursor(Qt::PointingHandCursor);
        }
        else
        {
            emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetEnumRange) returned "+QString::number(error)+
                " "+ m_Helper.mapReturnCodeToString(error)) ;
        }
    }
}

void ControllerTreeWindow::onEnumComboBoxClick ( const QString &sSelected )
{
    std::string sCurrentValue;
    VmbError_t error;
    int nIndex = 0; 
    int nIndexOld = 0;

    if( VmbErrorSuccess == m_FeaturePtr_EnumComboBox->GetValue(sCurrentValue) )
    {
        nIndexOld = m_EnumComboBox->findText(QString::fromStdString(sCurrentValue));
    }
    else
    {
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetEnumValue) " + sSelected +" returned "+QString::number(error)+
                      " "+ m_Helper.mapReturnCodeToString(error)) ;
    }

    if( (0 == m_sFeature_EnumComboBox.compare("PixelFormat")) || (0 == m_sFeature_EnumComboBox.compare("Pixel Format")) ||
        (0 == m_sFeature_EnumComboBox.compare("AcquisitionMode")) || (0 == m_sFeature_EnumComboBox.compare("Acquisition Mode")) )
    {
        /* stop Acquisition */
        emit acquisitionStartStop("AcquisitionStop");
        
        /* give a bit time to make sure that camera is stopped */
#ifdef WIN32
        ::Sleep(5);
#else
        ::usleep(5000);
#endif

        error = m_FeaturePtr_EnumComboBox->SetValue(sSelected.toAscii().data()); 
        if(VmbErrorSuccess != error)
        {
            emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (SetEnumValue) " + sSelected +" returned "+QString::number(error)+
                          " "+ m_Helper.mapReturnCodeToString(error)) ;
        }
        
        if(VmbErrorSuccess == m_FeaturePtr_EnumComboBox->GetValue(sCurrentValue))
        {
            /* make sure to set back the valid value from the camera to combobox */
            nIndex = m_EnumComboBox->findText(QString::fromStdString(sCurrentValue));
            m_EnumComboBox->setCurrentIndex(nIndex);
        }
        else
        {
            m_EnumComboBox->setCurrentIndex(nIndexOld);
            {
                emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetEnumValue) " + sSelected +" returned "+QString::number(error)+
                              " "+ m_Helper.mapReturnCodeToString(error)) ;
            }
        }
            
#ifdef WIN32
        ::Sleep(5);
#else
        ::usleep(5000);
#endif

        /*start Acquisition */
        emit acquisitionStartStop("AcquisitionStart");
        return;
    }

    error = m_FeaturePtr_EnumComboBox->SetValue(sSelected.toAscii().data()); 
    
    if(VmbErrorSuccess != error)
    {
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (SetEnumValue) " + sSelected +" returned "+QString::number(error)+
                      " "+ m_Helper.mapReturnCodeToString(error)) ;
    }

    if( VmbErrorSuccess == m_FeaturePtr_EnumComboBox->GetValue(sCurrentValue) )
    {
        /* make sure to set back the valid value from the camera to combobox */
        nIndex = m_EnumComboBox->findText(QString::fromStdString(sCurrentValue));
        m_EnumComboBox->setCurrentIndex(nIndex);
    }
    else
    {
        m_EnumComboBox->setCurrentIndex(nIndexOld);
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetEnumValue) " + sSelected +" returned "+QString::number(error)+
                      " "+ m_Helper.mapReturnCodeToString(error)) ;
    }
}

/* VmbFeatureDataInt */
void ControllerTreeWindow::createIntSliderSpinBox ( const QModelIndex item )
{
    VmbError_t error;
    QString sFeature_IntSpinBox   = getFeatureName(item);

    if (isFeatureWritable(sFeature_IntSpinBox))
    {
        m_sCurrentSelectedFeature = m_sFeature_IntSpinBox = sFeature_IntSpinBox;
        
        m_FeaturePtr_IntSpinBox = getFeaturePtr(m_sFeature_IntSpinBox);

        std::string sInfo;
        m_FeaturePtr_IntSpinBox->GetRepresentation(sInfo);

        QString sRepresentation = QString::fromStdString(sInfo);

        VmbInt64_t nMin = 0, nMax = 0, nInc = 1, nValue = 0;
        error = m_FeaturePtr_IntSpinBox->GetRange(nMin, nMax);

        /* use a line edit to show reg add in Hex */
        QString sName = getFeatureNameFromMap(m_sCurrentSelectedFeature);
        if( VmbErrorSuccess == error &&
            (0 == sRepresentation.compare("HexNumber") ||
            0 == sName.compare(Helper::m_GIGE_MULTICAST_IP_ADDRESS) ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_IP_ADDRESS) ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_SUBNET_MASK) ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY) ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_IP_ADDRESS) ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_SUBNET_MASK) ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY) ||
            nMax > std::numeric_limits<long>::max()))
        {
            createLineEdit(item);
            return;
        }
            
        if(NULL == m_FeaturePtr_IntSpinBox)
            return;
        
        resetControl();
    
        m_IntSpinSliderWidget = new QWidget(this);
        m_HSpinSliderLayout_Int = new QHBoxLayout( m_IntSpinSliderWidget );

        if(VmbErrorSuccess == error)
        {
            if( nMin < std::numeric_limits<int>::min() )
            {
                nMin = std::numeric_limits<int>::min();
            }
            
            if( nMax > std::numeric_limits<int>::max() )
            {
                nMax = std::numeric_limits<int>::max();
                if(VmbErrorSuccess == m_FeaturePtr_IntSpinBox->GetValue(nValue))
                {
                    if( nMax < nValue )
                        m_FeaturePtr_IntSpinBox->SetValue(nMax);         
                }
            }

            m_SpinBox_Int = new IntSpinBox(0);
            m_Slider_Int = new QSlider(Qt::Horizontal);
            m_Slider_Int->installEventFilter(this);

            error = m_FeaturePtr_IntSpinBox->GetIncrement(nInc);
            if(VmbErrorSuccess == error)
            {
                m_SpinBox_Int->setSingleStep(nInc);
                m_Slider_Int->setTickInterval(nInc);
            }

            nMax = nMax - ((nMax-nMin) % nInc);
            m_SpinBox_Int->setRange(nMin, nMax);
            m_Slider_Int->setRange( nMin, nMax);
            
            m_nSliderStep = nInc;
    
            error = m_FeaturePtr_IntSpinBox->GetValue(nValue);           
            if(VmbErrorSuccess == error)
            {
                if( nValue > std::numeric_limits<int>::max() )
                {
                    nValue = std::numeric_limits<int>::max();
                }

                m_SpinBox_Int->setValue(nValue);
                m_Slider_Int->setValue(nValue);
                m_nIntSliderOldValue = nValue;
            }

            if( (nMax-nMin) > 0xFFFF )
            {
                m_Slider_Int->setPageStep(1000);
            }
            else
            {
                m_Slider_Int->setPageStep(nInc);
            }
        }
        else
        {
            emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetRange) returned "+QString::number(error)+
                          " "+ m_Helper.mapReturnCodeToString(error)) ;
        }

        m_HSpinSliderLayout_Int2 = new QHBoxLayout( );
        m_HSpinSliderLayout_Int2->addWidget(m_Slider_Int);
        m_HSpinSliderLayout_Int2->addWidget(m_SpinBox_Int);
        m_HSpinSliderLayout_Int->addLayout(m_HSpinSliderLayout_Int2);
        
        QPoint p = QCursor::pos();      
        m_IntSpinSliderWidget->setFixedHeight(60);
        m_IntSpinSliderWidget->setWindowFlags( Qt::Tool | Qt::WindowCloseButtonHint );
        m_IntSpinSliderWidget->setWindowTitle(m_sCurrentSelectedFeature);
        m_IntSpinSliderWidget->show();
        m_IntSpinSliderWidget->move(p.x()+30, p.y() - (m_IntSpinSliderWidget->height() + 50));

        /* out of screen? */
        QDesktopWidget desktop;
        QRect rect = m_IntSpinSliderWidget->geometry();
        int rightPos = rect.right();

        if( (p.x() + 10 + (rightPos-p.x())) > desktop.width() )
        {
            m_IntSpinSliderWidget->move(p.x()-(30+rect.width()), p.y() - (m_IntSpinSliderWidget->height() + 50));
        }
    
        connect( m_Slider_Int,  SIGNAL(valueChanged(int)), this, SLOT(onIntSliderChanged(int)) );
        connect( m_SpinBox_Int, SIGNAL(editingFinished()), this, SLOT(onIntSpinBoxClick()) );       
        setCursor(Qt::PointingHandCursor);  
    }
}

void ControllerTreeWindow::onIntSliderReleased ( void )
{
    onIntSliderChanged(m_Slider_Int->value());
}

void ControllerTreeWindow::onIntSliderChanged ( int nValue )
{
    /* ignore it while pressing mouse and when slider still busy */
    if(m_bIsMousePressed || !m_bIsJobDone )
      return;

    m_bIsJobDone = false;
    unsigned int nCurrentValue = nValue;

    if( (m_nIntSliderOldValue > nValue) && (m_Slider_Int->minimum() != nValue) )
    {
        int nMod = nValue%m_nSliderStep;
        nValue = nValue - nMod;
    }
    
    if( (0 != (nValue%m_nSliderStep)) && ( m_Slider_Int->minimum() != nValue) && ( m_Slider_Int->maximum() != nValue) )
    {
        nValue = nValue + (m_nSliderStep - (nValue%m_nSliderStep));
    }
    
    if(m_Slider_Int->hasFocus())
    {
        setIntegerValue (nValue);
    }

    m_nIntSliderOldValue = nCurrentValue;
    m_bIsJobDone = true;
}

void ControllerTreeWindow::onIntSpinBoxClick ( void )
{   
    int nValue = m_SpinBox_Int->value();

    if( (0 != (nValue%m_nSliderStep)) && ( m_SpinBox_Int->minimum() != nValue) && ( m_SpinBox_Int->maximum() != nValue) )
    {
        nValue = (nValue + (m_nSliderStep - (nValue%m_nSliderStep)) );
    }

    setIntegerValue (nValue);
}

void ControllerTreeWindow::setIntegerValue ( const int &nValue )
{
    VmbError_t error;
    
    if( 0 == m_sFeature_IntSpinBox.compare("Width") ||0 == m_sFeature_IntSpinBox.compare("Height") )
    {
        /* stop Acquisition */
        emit acquisitionStartStop("AcquisitionStopWidthHeight");
        
        error = m_FeaturePtr_IntSpinBox->SetValue(nValue);
        if( VmbErrorSuccess != error )
        {
            emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (SetValue) "+ QString::number(nValue)+" returned "+QString::number(error)+
                          " "+ m_Helper.mapReturnCodeToString(error)) ;
        }
    
        updateCurrentIntValue();

        /*start Acquisition */
        emit acquisitionStartStop("AcquisitionStart");
        return;
    }

    error = m_FeaturePtr_IntSpinBox->SetValue(nValue);
    if( VmbErrorSuccess != error )
    {
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (SetValue) "+ QString::number(nValue)+" returned "+QString::number(error)+
                      " "+ m_Helper.mapReturnCodeToString(error)) ;
    }
    
    updateCurrentIntValue();
}

void ControllerTreeWindow::updateCurrentIntValue ( void )
{
    VmbInt64_t nCurrentValue = 0;
 
    VmbError_t error = m_FeaturePtr_IntSpinBox->GetValue(nCurrentValue);
    if(VmbErrorSuccess == error)
    {
        /* make sure to set back the valid value from the camera to slider and spinbox */
        m_SpinBox_Int->setValue(nCurrentValue);
        m_Slider_Int->setValue(nCurrentValue);
    }
    else
    {
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetValue) "+ QString::number(nCurrentValue)+" returned "+QString::number(error)+
                      " "+ m_Helper.mapReturnCodeToString(error)) ;
    }
}


/* VmbFeatureDataFloat */
void ControllerTreeWindow::createFloatSliderSpinBox ( const QModelIndex item )
{
    VmbError_t error;
    QString sFeature_FloatSliderSpinBox   = getFeatureName(item);

    if (isFeatureWritable(sFeature_FloatSliderSpinBox))
    {
        m_sCurrentSelectedFeature = m_sFeature_FloatSliderSpinBox = sFeature_FloatSliderSpinBox;
        m_FeaturePtr_FloatSliderSpinBox = getFeaturePtr(sFeature_FloatSliderSpinBox);
        if(NULL == m_FeaturePtr_FloatSliderSpinBox)
            return;

        resetControl();
    
        /* use a logarithmic slider for exposure */
        if( 0 == sFeature_FloatSliderSpinBox.compare("Exposure Time") || 0 == sFeature_FloatSliderSpinBox.compare("ExposureTimeAbs"))
        {
            createLogarithmicSLider(item);
            return;
        }

        m_FloatSpinSliderWidget = new QWidget(this);
        m_HSpinSliderLayout_Float = new QHBoxLayout( m_FloatSpinSliderWidget );

        error = m_FeaturePtr_FloatSliderSpinBox->GetRange(m_dMinimum, m_dMaximum);
        
        if(VmbErrorSuccess == error)
        {
            /* Spinbox */
            m_SpinBox_Float = new FloatSpinBox(0);
            m_SpinBox_Float->setRange( m_dMinimum, m_dMaximum);
            
            /* GigE: Gain, Hue 
            *  1394: Trigger Delay
            *  They will be treated as Floating in fact they are int 
            */
            
            if( (0 == m_sFeature_FloatSliderSpinBox.compare("Exposure Time")   && !m_bIsGigE) || 
                (0 == m_sFeature_FloatSliderSpinBox.compare("ExposureTimeAbs") &&  m_bIsGigE)  ||
                (0 == m_sFeature_FloatSliderSpinBox.compare("Trigger Delay")   && !m_bIsGigE))
            {
                m_SpinBox_Float->setSingleStep(1.0);
            }
            else if( m_dMaximum < 0xFFFF )
            {
                double dIncrement = getFloatFeatureIncrement(sFeature_FloatSliderSpinBox);
                m_SpinBox_Float->setSingleStep(dIncrement);
            }
    
            double dValue = 0;
            error = m_FeaturePtr_FloatSliderSpinBox->GetValue(dValue);

            if(VmbErrorSuccess == error)
            {
                m_SpinBox_Float->setValue( dValue );
            }
            else
            {
                emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetValue) "+ QString::number(dValue,'g',16)+" returned "+QString::number(error)+
                              " "+ m_Helper.mapReturnCodeToString(error)) ;
            }
        
            /* Slider */
            m_Slider_Float = new QSlider(Qt::Horizontal);
            m_Slider_Float->installEventFilter(this);

            if(  m_dMaximum <= 10)
            {
                m_Slider_Float->setRange( (int) (m_dMinimum * 100), (int) (m_dMaximum * 100) );
                m_Slider_Float->setValue( (int) (dValue * 100) );
            }
            else
            {
                m_Slider_Float->setRange( (int) m_dMinimum, (int) m_dMaximum );
                m_Slider_Float->setValue( (int) dValue );
            }
            
            if( (m_dMaximum-m_dMinimum) > 0xFFFF )
            {
                m_Slider_Float->setPageStep(1000);
            }
            else
            {
                if((m_dMaximum > 10) && (m_dMaximum <= 100) )
                    m_Slider_Float->setPageStep(2);
                else
                    m_Slider_Float->setPageStep(10);
            }
        
            m_HSpinSliderLayout_Float2 = new QHBoxLayout();
            m_HSpinSliderLayout_Float2->addWidget(m_Slider_Float);
            m_HSpinSliderLayout_Float2->addWidget(m_SpinBox_Float);
            m_HSpinSliderLayout_Float->addLayout(m_HSpinSliderLayout_Float2);
        }
        else
        {
            emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetRange) "+" returned "+QString::number(error)+
                          " "+ m_Helper.mapReturnCodeToString(error)) ;
        }

        QPoint p = QCursor::pos();      
        m_FloatSpinSliderWidget->setFixedHeight(60);
        m_FloatSpinSliderWidget->setWindowFlags( Qt::Tool | Qt::WindowCloseButtonHint );
        m_FloatSpinSliderWidget->setWindowTitle(m_sCurrentSelectedFeature);
        m_FloatSpinSliderWidget->show();
        m_FloatSpinSliderWidget->move(p.x()+30, p.y() - (m_FloatSpinSliderWidget->height() + 50));

        /* out of screen? */
        QDesktopWidget desktop;
        QRect rect = m_FloatSpinSliderWidget->geometry();
        int rightPos = rect.right();

        if( (p.x() + 10 + (rightPos-p.x())) > desktop.width() )
        {
            m_FloatSpinSliderWidget->move(p.x()-(30+rect.width()), p.y() - (m_FloatSpinSliderWidget->height() + 50));
        }
       
        connect( m_Slider_Float,  SIGNAL(valueChanged(int)), this, SLOT(onFloatSliderChanged(int)) );
        connect( m_SpinBox_Float, SIGNAL(editingFinished()), this, SLOT(onFloatSpinBoxClick()) );       
        setCursor(Qt::PointingHandCursor);  
    }
}

void ControllerTreeWindow::onFloatSliderReleased ( void )
{
    onFloatSliderChanged(m_Slider_Float->value());
}

void ControllerTreeWindow::onFloatSliderChanged ( int nValue )
{
    /* ignore it while pressing mouse and when slider still busy */
    if(m_bIsMousePressed || !m_bIsJobDone )
        return; 

    m_bIsJobDone = false;

    double dSpinBoxValue = m_SpinBox_Float->value();
    int    nSpinBoxValue = m_SpinBox_Float->value();
    double dDelta = dSpinBoxValue - nSpinBoxValue;
   
    double dValue = 0;
    
    if( 10.0 >= m_dMaximum)
    {
        dValue = double (nValue/100.0);
    }
    else
    {
        dValue = nValue + dDelta;
    }

    if(m_Slider_Float->hasFocus())
    {
        if( m_Slider_Float->maximum() == nValue)
        {   
            m_SpinBox_Float->setValue(m_dMaximum);
        }
        else if( m_Slider_Float->minimum() == nValue)
        {
            m_SpinBox_Float->setValue(m_dMinimum);
        }
        else
        {
            m_SpinBox_Float->setValue(dValue);
        }
        
        onFloatSpinBoxClick();
    }

    m_bIsJobDone = true;
}

void ControllerTreeWindow::onFloatSpinBoxClick ( void )
{
    double dValue = m_SpinBox_Float->value();
    setFloatingValue (dValue);
}

void ControllerTreeWindow::setFloatingValue ( const double &dValue )
{
    double dCurrentValue = 0;
    VmbError_t error;

    if( m_dMaximum < dValue )
    {
        error = m_FeaturePtr_FloatSliderSpinBox->SetValue(m_dMaximum);
    }
    else
    {
        error = m_FeaturePtr_FloatSliderSpinBox->SetValue(dValue);
    }
    
    if(VmbErrorSuccess == error)
    {
        error = m_FeaturePtr_FloatSliderSpinBox->GetValue(dCurrentValue);

        if(VmbErrorSuccess == error)
        {
            m_SpinBox_Float->setValue(dCurrentValue);
            
            if(!m_Slider_Float->hasFocus() || (!m_bIsGigE) || ( 0 == m_sCurrentSelectedFeature.compare("ColorTransformationValue")) )
            { /* Not GigE (except ColorTransformationValue), especially to handle Gamma feature. Make sure to set the current feature value back to slider */
                if( 10.0 >= m_dMaximum)
                {
                    m_Slider_Float->setValue(dCurrentValue * 100);
                }
                else
                {
                    m_Slider_Float->setValue(dCurrentValue);
                }
            }
        }

        /* reset all fps counters whenever shutter value has been changed */
        if( 0 == m_sFeature_FloatSliderSpinBox.compare("Exposure Time") ||
            0 == m_sFeature_FloatSliderSpinBox.compare("ExposureTimeAbs"))
        {
            emit resetFPS(); 
        }
    }
    else
    {
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (SetValue) "+ QString::number(dValue)+" returned "+QString::number(error)+
                      " "+ m_Helper.mapReturnCodeToString(error)) ;
    }
}

/* Logarithmic slider */
void ControllerTreeWindow::createLogarithmicSLider ( const QModelIndex &item )
{
    QString sFeature_FloatSliderSpinBox = getFeatureName(item);
    
    if (isFeatureWritable(sFeature_FloatSliderSpinBox))
    {
        m_sFeature_FloatSliderSpinBox = sFeature_FloatSliderSpinBox;

        m_LogSliderWidget  = new QWidget(this);
        m_HLogSliderLayout = new QHBoxLayout( m_LogSliderWidget );
        m_LogSliderWidget->setFixedHeight(80);
        m_LogSliderWidget->setMinimumWidth(400);

        /* LogSlider */
        m_LogSlider = new QwtSlider( m_LogSliderWidget, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough | QwtSlider::Groove );
        m_LogSlider->setScaleEngine( new QwtLog10ScaleEngine );
        m_LogSlider->setHandleSize( 20, 20 );
        m_LogSlider->setBorderWidth( 1 );

        /*
        *   |-----------------------------------------------------------------|
        *  Min                                                                Max
        *  m_dMinimum                                                         m_dMaximum
        */
        VmbError_t error = m_FeaturePtr_FloatSliderSpinBox->GetRange(m_dMinimum, m_dMaximum);
        if(VmbErrorSuccess == error)
        {
            m_LogSlider->setRange( log10(m_dMinimum), log10(m_dMaximum), 0.1 );
            m_LogSlider->setScale( m_dMinimum, m_dMaximum );
        }
        else
        {
            m_LogSlider->setRange( 0.0, 8.0, 0.1 );
            m_LogSlider->setScale( 1.0, (1.0e8) );
        }
        
        m_LogSlider->setScaleMaxMinor( 10 );
        m_HLogSliderLayout->addWidget(m_LogSlider);
        m_LogSlider->setFocusPolicy( Qt::StrongFocus);
        connect( m_LogSlider, SIGNAL( valueChanged( double ) ), this, SLOT( setLogarithmicSliderValue( double ) ) );

        /* SpinBox  */
        m_SpinBox_Float = new FloatSpinBox(0);
        m_HLogSliderLayout->addWidget(m_SpinBox_Float);
        m_SpinBox_Float->setFocusPolicy( Qt::StrongFocus);
        connect( m_SpinBox_Float, SIGNAL(editingFinished()), this, SLOT(onLogarithmicFloatSpinBoxClick()) );    

        if(VmbErrorSuccess == error)
        {
            m_SpinBox_Float->setRange( m_dMinimum, m_dMaximum);
            m_SpinBox_Float->setSingleStep(1.0);
        }

        double dCurrentValue = 0;
        m_FeaturePtr_FloatSliderSpinBox->GetValue(dCurrentValue);
        m_SpinBox_Float->setValue(dCurrentValue);
        m_LogSlider->setValue(log10(dCurrentValue));
        m_LogSlider->installEventFilter(this);

        m_LogSliderWidget->setWindowTitle( "Exposure (Min: " + QString::number(m_dMinimum,'g',16) + ", Max: " + QString::number(m_dMaximum,'g', 16) + ") "+ m_sCameraID );
        m_LogSliderWidget->setWindowFlags(Qt::WindowTitleHint);
        QPoint p = QCursor::pos();      
        m_LogSliderWidget->setWindowFlags( Qt::Tool );
        m_LogSliderWidget->show();
        m_LogSliderWidget->move(p.x()+30, p.y() - (m_LogSliderWidget->height()+50));

        /* out of screen? */
        QDesktopWidget desktop;
        QRect rect = m_LogSliderWidget->geometry();
        int rightPos = rect.right();

        if( (p.x() + 10 + (rightPos-p.x())) > desktop.width() )
        {
            m_LogSliderWidget->move(p.x()-(30+rect.width()), p.y() - (m_LogSliderWidget->height()+50));
        }
    }
}

void ControllerTreeWindow::onLogarithmicFloatSpinBoxClick ( void )
{
    setLogarithmicFloatingValue ( m_SpinBox_Float->value() );
}

void ControllerTreeWindow::onLogarithmicSliderReleased( void )
{  
    setLogarithmicSliderValue(m_LogSlider->value());
}

void ControllerTreeWindow::setLogarithmicSliderValue( double v )
{
    /* ignore it while pressing mouse and when slider still busy */
    if(m_bIsMousePressed || !m_bIsJobDone )
        return; 

    m_bIsJobDone = false;

    double dValue = qPow(10.0, v);
    dValue = floor(dValue);
    
    if( dValue > (m_dMaximum * 0.8) )
        dValue = m_dMaximum;
        
    if(m_LogSlider->hasFocus())
        setLogarithmicFloatingValue ( dValue );

    m_bIsJobDone = true;
}

void ControllerTreeWindow::setLogarithmicFloatingValue ( const double &dValue )
{
    if( (m_dMaximum >= dValue ) && (dValue>= m_dMinimum))
    {
        m_FeaturePtr_FloatSliderSpinBox->SetValue(dValue);
    }

    else if(dValue < m_dMinimum)
    {
        m_FeaturePtr_FloatSliderSpinBox->SetValue(m_dMinimum);
    }
    
    else if(dValue > m_dMaximum)
    {
        m_FeaturePtr_FloatSliderSpinBox->SetValue(m_dMaximum);
    }

    double dCurrentValue = 0;
    m_FeaturePtr_FloatSliderSpinBox->GetValue(dCurrentValue);
    
    double dLog = log10(dCurrentValue);
    double dRounded = static_cast<double>(static_cast<int>(dLog*100+0.5))/100.0;
    m_LogSlider->setValue(dRounded);
    
    m_SpinBox_Float->setValue(dCurrentValue);
    m_LogSlider->setToolTip(QString::number(dCurrentValue,'g',16));
    
    emit resetFPS(); 
}

/* VmbFeatureDataBool */
void ControllerTreeWindow::createBooleanCheckBox ( const QModelIndex item )
{
    QString sFeature_CheckBox   = getFeatureName(item);

    if (isFeatureWritable(sFeature_CheckBox))
    {
        m_sCurrentSelectedFeature = m_sFeature_CheckBox = sFeature_CheckBox;

        m_FeaturePtr_CheckBox = getFeaturePtr(m_sFeature_CheckBox);
        if(NULL == m_FeaturePtr_CheckBox)
            return;

        resetControl();

        m_BooleanWidget  = new QWidget(this);
        m_HBooleanLayout = new QHBoxLayout (m_BooleanWidget);
        m_CheckBox_Bool = new QCheckBox (QString(""));
        m_CheckBox_Bool->setMaximumHeight(20);
        m_HBooleanLayout2 = new QHBoxLayout ();
        m_HBooleanLayout2->addWidget(m_CheckBox_Bool);
        m_HBooleanLayout->addLayout(m_HBooleanLayout2);

        bool bValue = false;
        VmbError_t error = m_FeaturePtr_CheckBox->GetValue(bValue);
        if (VmbErrorSuccess == error)
        {
            m_CheckBox_Bool->setChecked(bValue);
        }
        else
        {
            m_CheckBox_Bool->setChecked(false); 
            emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetValue) "+ " returned "+QString::number(error)+
                " "+ m_Helper.mapReturnCodeToString(error)) ;
        }

        QPoint p = QCursor::pos();      
        m_BooleanWidget->setFixedHeight(60);
        m_CheckBox_Bool->setText(m_sCurrentSelectedFeature);

        m_BooleanWidget->setWindowFlags( Qt::Tool | Qt::WindowCloseButtonHint );
        m_BooleanWidget->setWindowTitle(m_sCurrentSelectedFeature);
        m_BooleanWidget->show();
        m_BooleanWidget->move(p.x()+30, p.y() - (m_BooleanWidget->height() + 50));

        /* out of screen? */
        QDesktopWidget desktop;
        QRect rect = m_BooleanWidget->geometry();
        int rightPos = rect.right();

        if( (p.x() + 10 + (rightPos-p.x())) > desktop.width() )
        {
            m_BooleanWidget->move(p.x()-(30+rect.width()), p.y() - (m_BooleanWidget->height() + 50));
        }

        connect( m_CheckBox_Bool, SIGNAL(clicked(bool)), this, SLOT(onBoolCheckBoxClick(bool)) );
        setCursor(Qt::PointingHandCursor);
    }
}

void ControllerTreeWindow::onBoolCheckBoxClick ( bool bValue )
{ 
    bool bCurrentValue = false;
    
    VmbError_t error = m_FeaturePtr_CheckBox->SetValue(bValue); 
    if(VmbErrorSuccess == error)
    {
        error = m_FeaturePtr_CheckBox->GetValue(bCurrentValue); 
        if(VmbErrorSuccess == error)
        {
            QString sValue = "false";
            (true == bCurrentValue) ? sValue = "true" : sValue = "false";
            m_CheckBox_Bool->setChecked(bCurrentValue);
        }
        else
        {
            emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetValue) "+ " returned "+QString::number(error)+
                          " "+ m_Helper.mapReturnCodeToString(error)) ;
        }
    }
    else
    {
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (SetValue) "+ QString::number(bValue) + " returned "+QString::number(error)+
                      " "+ m_Helper.mapReturnCodeToString(error)) ;
    }
}

/* VmbFeatureDataString */
void ControllerTreeWindow::createStringEditBox ( const QModelIndex item )
{
    QString sFeature_StringEditBox = getFeatureName(item);

    if (isFeatureWritable(sFeature_StringEditBox))
    {
        m_sCurrentSelectedFeature = m_sFeature_StringEditBox = sFeature_StringEditBox;

        m_FeaturePtr_StringEditBox = getFeaturePtr(m_sFeature_StringEditBox);
        if(NULL == m_FeaturePtr_StringEditBox)
            return;

        resetControl();

        m_EditWidget = new QWidget (this);
        m_HEditLayout = new QHBoxLayout(m_EditWidget);
        m_TextEdit_String = new QLineEdit ();
        m_TextEdit_String->setMaximumHeight(20);
        m_HEditLayout2 = new QHBoxLayout();
        m_HEditLayout2->addWidget(m_TextEdit_String);
        m_HEditLayout->addLayout(m_HEditLayout2);

        std::string sValue;
        VmbError_t error = m_FeaturePtr_StringEditBox->GetValue(sValue);

        if(VmbErrorSuccess == error)
            m_TextEdit_String->setText(QString::fromStdString(sValue));
        else
            m_TextEdit_String->setText(QString::fromStdString(" "));

        QPoint p = QCursor::pos();      
        m_EditWidget->setFixedHeight(60);
        m_EditWidget->setWindowFlags( Qt::Tool | Qt::WindowCloseButtonHint );
        m_EditWidget->setWindowTitle(m_sCurrentSelectedFeature);
        m_EditWidget->show();
        m_EditWidget->move(p.x()+30, p.y() - (m_EditWidget->height() + 50));

        /* out of screen? */
        QDesktopWidget desktop;
        QRect rect = m_EditWidget->geometry();
        int rightPos = rect.right();

        if( (p.x() + 10 + (rightPos-p.x())) > desktop.width() )
        {
            m_EditWidget->move(p.x()-(30+rect.width()), p.y() - (m_EditWidget->height() + 50));
        }
        connect( m_TextEdit_String, SIGNAL(returnPressed()), this, SLOT(onEditText()) );
        setCursor(Qt::PointingHandCursor);
    }
}

void ControllerTreeWindow::onEditText ( void )
{
    QString sText = m_TextEdit_String->text();
    VmbError_t error = m_FeaturePtr_StringEditBox->SetValue((const char*)sText.toAscii());
    
    if(VmbErrorSuccess == error)
    {
        std::string sCurrentValue;
        error = m_FeaturePtr_StringEditBox->GetValue(sCurrentValue); 
        if(VmbErrorSuccess == error)
        {
            m_TextEdit_String->setText(QString::fromStdString(sCurrentValue));
        }
        else
        {
            m_TextEdit_String->setText("");
            emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (GetStringValue) "+ " returned "+QString::number(error)+
                          " "+ m_Helper.mapReturnCodeToString(error)) ;
        }
    }
    else
    {
        emit logging( "ERROR Feature: " + m_sCurrentSelectedFeature + " (SetStringValue) "+ sText  + " returned "+QString::number(error)+
            " "+ m_Helper.mapReturnCodeToString(error)) ;
    }
}

void ControllerTreeWindow::createLineEdit ( const QModelIndex &item )
{
    QString sFeature = getFeatureName(item);

    if (isFeatureWritable(sFeature))
    {
        m_sCurrentSelectedFeature = m_sFeature_StringLineEdit = sFeature;
        m_FeaturePtr_LineEdit = getFeaturePtr(m_sFeature_StringLineEdit);
        if(NULL == m_FeaturePtr_LineEdit)
            return;

        resetControl();

        QString sName = getFeatureNameFromMap(m_sCurrentSelectedFeature);
        std::string sRepresentation;
        m_FeaturePtr_IntSpinBox->GetRepresentation( sRepresentation );
        
        m_LineEditWidget = new QWidget(this);
        m_HLineEditLayout = new QHBoxLayout( m_LineEditWidget );

        /* add LineEdit that allows 0-255 */
        /* add Hex Label */
        m_LineEdit = new QLineEdit (m_LineEditWidget);

        if( 0 == sName.compare(Helper::m_GIGE_MULTICAST_IP_ADDRESS)    ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_IP_ADDRESS)      ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_SUBNET_MASK)     ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY) ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_IP_ADDRESS)   ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_SUBNET_MASK)  ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY))
        {
            m_HexLabel = new QLabel ("", m_LineEditWidget);
            m_LineEdit->setInputMask("000.000.000.000;0");
            QRegExp rxIp("(([01]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])\\.){3}([01]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])");
            m_LineEdit->setValidator(new QRegExpValidator(rxIp, m_LineEdit)); 
            m_LineEdit->setMinimumWidth(20);
        }
        else if ( !strcmp( "HexNumber", sRepresentation.c_str() ))
        {   
            m_HexLabel = new QLabel (" 0x", m_LineEditWidget);
            /* 1394 direct register access */
            m_LineEdit->setInputMask("HHHHHHHH");
        }
        else // Int64
        {
            m_HexLabel = new QLabel ("", m_LineEditWidget);
            QRegExp rxIp("-?[0-9]*");
            m_LineEdit->setValidator(new QRegExpValidator(rxIp, m_LineEdit)); 
            m_LineEdit->setMinimumWidth(20);
        }

        m_HexLabel->setMaximumHeight(20);
        m_LineEdit->setMaximumHeight(20);

        /* add confirm button */
        m_ConfirmButton = new QPushButton("Set", m_LineEditWidget);
        m_ConfirmButton->setMaximumHeight(20);

        m_HLineEditLayout2 = new QHBoxLayout( );
        m_HLineEditLayout2->addWidget(m_HexLabel);
        m_HLineEditLayout2->addWidget(m_LineEdit);
        m_HLineEditLayout2->addWidget(m_ConfirmButton);
        m_HLineEditLayout->addLayout(m_HLineEditLayout2);

        VmbInt64_t nValue64 = 0;
        VmbError_t error = m_FeaturePtr_LineEdit->GetValue(nValue64);

        QString sHexValue("");
        if(VmbErrorSuccess == error)
        {
            sHexValue = QString::number(nValue64, 16);
            /* Handling of Multicast IP Address */
            if( 0 == sName.compare(Helper::m_GIGE_MULTICAST_IP_ADDRESS)    ||
                0 == sName.compare(Helper::m_GIGE_CURRENT_IP_ADDRESS)      ||
                0 == sName.compare(Helper::m_GIGE_CURRENT_SUBNET_MASK)     ||
                0 == sName.compare(Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY) ||
                0 == sName.compare(Helper::m_GIGE_PERSISTENT_IP_ADDRESS)   ||
                0 == sName.compare(Helper::m_GIGE_PERSISTENT_SUBNET_MASK)  ||
                0 == sName.compare(Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY))
            {
                Helper toolHelper;
                m_LineEdit->insert(toolHelper.parseToIPAddressLittleEndian(sHexValue));
            }
            /* Handling of DeviceAccessRegisterValue or DeviceAccessRegisterAddress */
            else if ( !strcmp( "HexNumber", sRepresentation.c_str() ))
            {
                m_LineEdit->insert(sHexValue);
            }
            else
            {
                m_LineEdit->insert(QString::number( nValue64 ));
            }
        }
        else
        {
            m_LineEdit->insert("Read Error");
        }

        QPoint p = QCursor::pos();      
        m_LineEditWidget->setFixedHeight(60);
        m_LineEditWidget->setWindowFlags( Qt::Tool | Qt::WindowCloseButtonHint );
        m_LineEditWidget->setWindowTitle(m_sCurrentSelectedFeature);
        m_LineEditWidget->show();
        m_LineEditWidget->move(p.x()+30, p.y() - (m_LineEditWidget->height() + 50));

        /* out of screen? */
        QDesktopWidget desktop;
        QRect rect = m_LineEditWidget->geometry();
        int rightPos = rect.right();

        if( (p.x() + 10 + (rightPos-p.x())) > desktop.width() )
        {
            m_LineEditWidget->move(p.x()-(30+rect.width()), p.y() - (m_LineEditWidget->height() + 50));
        }

        connect( m_ConfirmButton, SIGNAL(clicked()), this, SLOT(onConfirmClick()) );
        setCursor(Qt::PointingHandCursor);
    }
}

void ControllerTreeWindow::onConfirmClick ( void )
{
    m_HexLabel->clear();
    VmbError_t error;
    bool bOk;

    qlonglong lValue = 0;
    QString sName = getFeatureNameFromMap(m_sCurrentSelectedFeature);

    std::string sRepresentation;
    m_FeaturePtr_IntSpinBox->GetRepresentation( sRepresentation );

    if( 0 == sName.compare(Helper::m_GIGE_MULTICAST_IP_ADDRESS)    ||
        0 == sName.compare(Helper::m_GIGE_CURRENT_IP_ADDRESS)      ||
        0 == sName.compare(Helper::m_GIGE_CURRENT_SUBNET_MASK)     ||
        0 == sName.compare(Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY) ||
        0 == sName.compare(Helper::m_GIGE_PERSISTENT_IP_ADDRESS)   ||
        0 == sName.compare(Helper::m_GIGE_PERSISTENT_SUBNET_MASK)  ||
        0 == sName.compare(Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY)) 
    {
        m_HexLabel->setText("");
        /* read the value in byte */
        QString sIP = m_LineEdit->displayText();
        uint nSub0 = sIP.section('.', 0,0).toInt();
        uint nSub1 = sIP.section('.', 1,1).toInt();
        uint nSub2 = sIP.section('.', 2,2).toInt();
        uint nSub3 = sIP.section('.', -1,-1).toInt();

        /* convert to hex */
        QString sHexIP("");
        QString sHexSub("");

        /* set back to big endian, starting from nSub3 to nSub0 */
        sHexSub = sHexSub.setNum(nSub3,16);
        if(15 >= nSub3)
           sHexIP.append( "0"+ sHexSub );
        else
            sHexIP.append( sHexSub );

        sHexSub = sHexSub.setNum(nSub2,16);
        if(15 >= nSub2)
            sHexIP.append( "0"+ sHexSub );
        else
            sHexIP.append( sHexSub );

        sHexSub = sHexSub.setNum(nSub1,16);
        if(15 >= nSub1)
            sHexIP.append( "0"+ sHexSub );
        else
            sHexIP.append( sHexSub );

        sHexSub = sHexSub.setNum(nSub0,16);
        if(15 >= nSub0)
            sHexIP.append( "0"+ sHexSub );
        else
            sHexIP.append( sHexSub );

        lValue = sHexIP.toULongLong(&bOk, 16);

    }
    else if ( !strcmp( "HexNumber", sRepresentation.c_str() ))
    {
        m_HexLabel->setText(" 0x");
        /* convert hex to qlonglong */
        lValue = m_LineEdit->text().toLongLong(&bOk, 16);
    }
    else
    {
        m_HexLabel->setText("");
        lValue = m_LineEdit->text().toLongLong(&bOk);
        if ( !bOk && !lValue )
        {
            lValue = m_LineEdit->text().contains('-') ? std::numeric_limits<long long>::min() : std::numeric_limits<long long>::max();
        }
    }

    VmbInt64_t nCurrentValue64 = 0, nMin = 0, nMax = 0;
    VmbInt64_t nValue64 = (VmbInt64_t)lValue;
    error = m_FeaturePtr_IntSpinBox->GetRange(nMin, nMax);
    if ( VmbErrorSuccess == error )
    {
        if ( nMin > nValue64 )
        {
            nValue64 = nMin;
        }
        if ( nMax < nValue64 )
        {
            nValue64 = nMax;
        }
        error = m_FeaturePtr_LineEdit->SetValue(nValue64);
    }
    if(VmbErrorSuccess == error)
    {
        /* if set is ok, read the value again and put it to GUI */
        error = m_FeaturePtr_LineEdit->GetValue(nCurrentValue64);
        if(VmbErrorSuccess != error)
        {
            if( 0 == sName.compare(Helper::m_GIGE_MULTICAST_IP_ADDRESS)    ||
                0 == sName.compare(Helper::m_GIGE_CURRENT_IP_ADDRESS)      ||
                0 == sName.compare(Helper::m_GIGE_CURRENT_SUBNET_MASK)     ||
                0 == sName.compare(Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY) ||
                0 == sName.compare(Helper::m_GIGE_PERSISTENT_IP_ADDRESS)   ||
                0 == sName.compare(Helper::m_GIGE_PERSISTENT_SUBNET_MASK)  ||
                0 == sName.compare(Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY)) 
            {
                m_HexLabel->setText("READ ERROR!!!");
            }
            else if ( !strcmp( "HexNumber", sRepresentation.c_str() ))
            {
                m_HexLabel->setText("READ ERROR!!! 0x");
            }
            else
            {
                m_HexLabel->setText("READ ERROR!!!");
            }

            return;
        }

        m_LineEdit->clear();

        if( 0 == sName.compare(Helper::m_GIGE_MULTICAST_IP_ADDRESS)    ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_IP_ADDRESS)      ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_SUBNET_MASK)     ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY) ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_IP_ADDRESS)   ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_SUBNET_MASK)  ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY)) 
        {
            Helper toolHelper;
            m_LineEdit->insert(toolHelper.parseToIPAddressLittleEndian(QString::number(nCurrentValue64, 16)));
        }
        else if ( !strcmp( "HexNumber", sRepresentation.c_str() ))
        {
            m_LineEdit->insert(QString::number(nCurrentValue64, 16));
        }
        else
        {
            m_LineEdit->insert(QString::number(nCurrentValue64));
        }
    }
    else
    {
        if( 0 == sName.compare(Helper::m_GIGE_MULTICAST_IP_ADDRESS)    ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_IP_ADDRESS)      ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_SUBNET_MASK)     ||
            0 == sName.compare(Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY) ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_IP_ADDRESS)   ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_SUBNET_MASK)  ||
            0 == sName.compare(Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY)) 
        {
            m_HexLabel->setText("WRITE ERROR!!!");
        }
        else if ( !strcmp( "HexNumber", sRepresentation.c_str() ))
        {
            m_HexLabel->setText("WRITE ERROR!!! 0x");
        }
        else
        {
            m_HexLabel->setText("WRITE ERROR!!!");
        }
    }
}

bool ControllerTreeWindow::eventFilter(QObject *object, QEvent *event)
{
    /* set the slider focus back when loosing on scrolling, e.g scrolling right after changing spinbox */
    if (event->type() == QEvent::Wheel)
    {
        if (object == m_Slider_Int)
        {
            if(!m_Slider_Int->hasFocus())
                m_Slider_Int->setFocus();
        }

        if (object == m_Slider_Float)
        {
            if(!m_Slider_Float->hasFocus())
                m_Slider_Float->setFocus();
        }

        if (object == m_LogSlider)
        {
            if(!m_LogSlider->hasFocus())
                m_LogSlider->setFocus();
        }
    }

    if (event->type() == QEvent::MouseButtonPress)
    {
        m_bIsMousePressed = true;
    }

    if (event->type() == QEvent::MouseButtonRelease)
    {
        m_bIsMousePressed = false;

        if (object == m_Slider_Int)
            onIntSliderReleased();

        if (object == m_Slider_Float)
            onFloatSliderReleased();

        if (object == m_LogSlider)
            onLogarithmicSliderReleased();    
    }

    return false;        
}

void ControllerTreeWindow::resetControl ( void )
{
    /* delete button */
    if( NULL != m_ButtonWidget )
    {
        disconnect( m_CmdButton, SIGNAL(clicked()), this, SLOT(onCmdButtonClick()) );
        delete m_ButtonWidget;

        m_ButtonWidget   = NULL;
        m_CmdButton      = NULL;
        m_HButtonLayout  = NULL;
        m_HButtonLayout2 = NULL;
    }

    /* delete ComboBox */
    if( NULL != m_EnumComboBox )
    {
        disconnect( m_EnumComboBox, SIGNAL(activated(const QString &)), this, SLOT(onEnumComboBoxClick(const QString &)) );
        delete m_ComboWidget;        
        
        m_ComboWidget   = NULL;
        m_EnumComboBox  = NULL;
        m_HComboLayout  = NULL;
        m_HComboLayout2 = NULL;
    }

    /* delete SpinBox+Slider Int */
    if( ( NULL != m_SpinBox_Int ) && ( NULL != m_Slider_Int ) && 
        ( NULL != m_HSpinSliderLayout_Int ) && ( NULL != m_IntSpinSliderWidget))
    {   
        removeEventFilter(m_Slider_Int);
        disconnect( m_Slider_Int,  SIGNAL(valueChanged(int)), this, SLOT(onIntSliderChanged(int)) );
        disconnect( m_SpinBox_Int, SIGNAL(editingFinished()), this, SLOT(onIntSpinBoxClick()) );    
        delete m_IntSpinSliderWidget;

        m_IntSpinSliderWidget    = NULL;
        m_SpinBox_Int            = NULL;
        m_Slider_Int             = NULL;
        m_HSpinSliderLayout_Int  = NULL;
        m_HSpinSliderLayout_Int2 = NULL;
    }

    /* delete SpinBox+Slider Float */
    if( ( NULL != m_SpinBox_Float ) && ( NULL != m_Slider_Float ) && 
        ( NULL != m_HSpinSliderLayout_Float ) && ( NULL != m_FloatSpinSliderWidget))
    {
        removeEventFilter(m_Slider_Float);
        disconnect( m_Slider_Float,  SIGNAL(valueChanged(int)), this, SLOT(onFloatSliderChanged(int)) );
        disconnect( m_SpinBox_Float, SIGNAL(editingFinished()), this, SLOT(onFloatSpinBoxClick()) );        
        delete m_FloatSpinSliderWidget;
        
        m_FloatSpinSliderWidget    = NULL;
        m_SpinBox_Float            = NULL;
        m_Slider_Float             = NULL;
        m_HSpinSliderLayout_Float  = NULL;
        m_HSpinSliderLayout_Float2 = NULL;
    }

    /* delete bool CheckBox  */
    if( NULL != m_CheckBox_Bool )
    {
        disconnect( m_CheckBox_Bool, SIGNAL(clicked(bool)), this, SLOT(onBoolCheckBoxClick(bool)) );
        delete m_BooleanWidget;

        m_BooleanWidget   = NULL;
        m_CheckBox_Bool   = NULL;
        m_HBooleanLayout  = NULL;
        m_HBooleanLayout2 = NULL;
    }
    
    /* delete TextEdit */
    if( NULL != m_TextEdit_String )
    {
        disconnect( m_TextEdit_String, SIGNAL(returnPressed()), this, SLOT(onEditText()) );
        delete m_EditWidget;

        m_EditWidget      = NULL;
        m_TextEdit_String = NULL;
        m_HEditLayout     = NULL;
        m_HEditLayout2    = NULL;
    }

    /* delete LineEdit for REG ADD or Multicast IP Address*/
    if( NULL != m_LineEdit )    
    {
        disconnect( m_ConfirmButton, SIGNAL(clicked()), this, SLOT(onConfirmClick()) );
        delete m_LineEditWidget;

        m_LineEditWidget   = NULL;
        m_LineEdit         = NULL;
        m_ConfirmButton    = NULL;
        m_HexLabel         = NULL;
        m_HLineEditLayout  = NULL;
        m_HLineEditLayout2 = NULL;
    }

    if ( NULL != m_HexWindow )
    {
        /* windows has been closed , just reset it */
        m_HexWindow = NULL;
        m_RawData.clear();
    }

    /* delete Logarithmic Slider */
    if( NULL != m_LogSliderWidget )
    {
        removeEventFilter(m_LogSlider);
        disconnect( m_LogSlider, SIGNAL( valueChanged( double ) ), this, SLOT( setLogarithmicSliderValue( double ) ) );
        delete m_LogSliderWidget;
        
        m_LogSliderWidget   = NULL;
        m_LogSlider         = NULL;
        m_HLogSliderLayout  = NULL;
        m_SpinBox_Float     = NULL;
    }
}

void ControllerTreeWindow::showControl( const bool &bIsShow )
{
  if( bIsShow )
  {
      if( NULL != m_IntSpinSliderWidget )
          m_IntSpinSliderWidget->show();

      else if( NULL != m_ButtonWidget )
          m_ButtonWidget->show();

      else if( NULL != m_ComboWidget )
          m_ComboWidget->show();

      else if( NULL != m_FloatSpinSliderWidget )
          m_FloatSpinSliderWidget->show();

      else if( NULL != m_BooleanWidget )
          m_BooleanWidget->show();

      else if( NULL != m_EditWidget )
          m_EditWidget->show();

      else if( NULL != m_LineEditWidget )
          m_LineEditWidget->show();

      else if( NULL != m_LogSliderWidget )
          m_LogSliderWidget->show();
  }
  else
  {
      if( NULL != m_IntSpinSliderWidget )
          m_IntSpinSliderWidget->hide();

      else if( NULL != m_ButtonWidget )
          m_ButtonWidget->hide();

      else if( NULL != m_ComboWidget )
          m_ComboWidget->hide();

      else if( NULL != m_FloatSpinSliderWidget )
          m_FloatSpinSliderWidget->hide();

      else if( NULL != m_BooleanWidget )
          m_BooleanWidget->hide();

      else if( NULL != m_EditWidget )
          m_EditWidget->hide();

      else if( NULL != m_LineEditWidget )
          m_LineEditWidget->hide();

      else if( NULL != m_LogSliderWidget )
          m_LogSliderWidget->hide();
  }
  
}