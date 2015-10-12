/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        MainWindow.cpp

  Description: The main window framework. This contains of camera tree, a toolbar and logging
               

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

#include "MainWindow.h"
#include "CameraObserver.h"
#include "SplashScreen.h"
#include <QSplitter>
#include "Version.h"


MainWindow::MainWindow ( QWidget *parent, Qt::WFlags flags ): QMainWindow ( parent, flags ), m_bIsCurrentModelChecked ( false ),
m_Logger( NULL ), m_VimbaSystem(AVT::VmbAPI::VimbaSystem::GetInstance()), m_bIsRightMouseClicked ( false ), m_nOpenState ( 0 ), 
m_bIsOpenByRightMouseClick ( false ), m_bIsAutoAdjustPacketSize ( false )
{
    /* create the main window */
    m_MainWindow.setupUi(this);
    connect(m_MainWindow.actionAbout, SIGNAL(triggered()), this, SLOT(about()));
    connect(m_MainWindow.actionAbout_Qt, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
    connect(m_MainWindow.actionExit, SIGNAL(triggered()), this, SLOT(close()));
    /* create splitter 
    *     _______________
    *   |        |        |
    *    |       |        |
    *    |        |        |
    *    |_______|_______|
    */

    QSplitter *splitter_H = new QSplitter(Qt::Horizontal, m_MainWindow.centralWidget);    
    splitter_H->setChildrenCollapsible(false);
    
    m_CameraTree = new CameraTreeWindow(splitter_H);
    connect(m_CameraTree, SIGNAL(cameraClicked(const QString &, const bool &)), this, SLOT(onCameraClicked(const QString &, const bool &)));
    connect(m_CameraTree, SIGNAL(rightMouseClicked(const bool &)), this, SLOT(onRightMouseClicked(const bool &)));

    m_Logger = new LoggerWindow(splitter_H);
    m_MainWindow.layout_H->addWidget(splitter_H);

    /* use setting position and geometry from the last session*/
    QSettings settings("Allied Vision Technologies", "AVT VimbaViewer");
    this->restoreGeometry(settings.value("geometrymainwindow").toByteArray());
    this->restoreState(settings.value("state").toByteArray(), 0);    

    /* get VmbAPI version */
    VmbVersionInfo_t version;
    VmbErrorType error = m_VimbaSystem.QueryVersion( version );
    m_sAPIVersion = "VimbaAPI Version: " + QString::number(version.major) + "." + QString::number(version.minor) + "." + QString::number(version.patch);
    ( VmbErrorSuccess != error) ?    m_Logger->logging(" QueryVersion failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR ) :
                                    m_Logger->logging(m_sAPIVersion, VimbaViewerLogCategory_INFO);

    /*Start Option (auto adjust packet size) */
    m_StartOptionDialog = new QDialog(this);
    m_StartOption.setupUi(m_StartOptionDialog);
    m_MainWindow.ActionStartOptions->setEnabled(false);
    /* read the last saved value */
    QVariant result = settings.value("autoenable", true);
    m_bIsAutoAdjustPacketSize = result.toBool();
    m_StartOption.AutoAdjustPacketSize_CheckBox->setChecked(m_bIsAutoAdjustPacketSize); 

    /* Initialize VmbAPI */
    error = m_VimbaSystem.Startup();
    if(VmbErrorSuccess != error)
    {
        m_Logger->logging(" Startup failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR );
        return;
    }
                                
    try
    {
        searchCameras( );
       // m_CameraTree->expandAll();
    }
    catch(const std::exception &e)
    {
        m_Logger->logging("MainWindow <constructor> Exception: "+QString::fromStdString(e.what()), VimbaViewerLogCategory_ERROR); 
    }

    /*    register the camera observer to get notification when camera list has been updated, e.g:
    *     - plug / unplug camera(s)
    *     - bus reset occurs 
    */
    CameraObserver *camObserver = new CameraObserver();
    ICameraListObserverPtr pDeviceObs(camObserver);
    
    error = m_VimbaSystem.RegisterCameraListObserver(pDeviceObs);
    if(VmbErrorSuccess == error)
    {
        connect(camObserver, SIGNAL(updateDeviceList( )), this, SLOT(onUpdateDeviceList()));
    }
    else
    {
        m_Logger->logging("RegisterCameraListObserver Failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR); 
    }

}

MainWindow::~MainWindow ( void )
{
    /* save setting position and geometry */
    QSettings settings("Allied Vision Technologies", "AVT VimbaViewer");
    settings.setValue("geometrymainwindow", saveGeometry());
    settings.setValue("state", saveState(0));
    settings.setValue("autoenable", m_bIsAutoAdjustPacketSize);

    /* release API */
    m_VimbaSystem.Shutdown();
}

void MainWindow::about ( void )
{
    QString sAbout(QString("VimbaViewer is an application example using VimbaAPI\n\nVersion: ") + QString(VIMBAVIEWER_VERSION) + QString("\n"));
    sAbout.append(m_sAPIVersion).append("\n").append("\nAllied Vision Technologies GmbH\nCopyright(C) 2012-2014");
    QMessageBox::about(this, tr("About AVT VimbaViewer"), sAbout);
}

/* this will be called on destruction of mainwindow in case at least one viewerwindow is open */
void MainWindow::closeEvent ( QCloseEvent *event )
{
    if(!m_Viewer.empty())
    {
        QVector<ViewerWindow*>::iterator it = m_Viewer.begin();
        while(it != m_Viewer.end())
        {
            delete m_Viewer[0];
            m_Viewer[0] = NULL;
            m_Viewer.erase(it);
        }
    }
}

void MainWindow::onUpdateDeviceList ( void )
{
    /*
    QPixmap pixmap( ":/VimbaViewer/Images/refresh.png" );
    SplashScreen splashScreen(pixmap, this, Qt::SplashScreen);
    int nW = ((this->width()/2) - splashScreen.width()/2);
    int nH = ((this->height()/2) - splashScreen.height()/2);
    splashScreen.setGeometry(nW,nH, splashScreen.width(),splashScreen.height());
    splashScreen.show();
    splashScreen.showMessage("Please wait..." , Qt::AlignHCenter | Qt::AlignVCenter, Qt::red);
    */
    std::string sDisconnectedModel;
    CameraPtrVector currentListedCameras;
    bool bIsFound = false;
    
    VmbErrorType error;
    error = m_VimbaSystem.GetCameras(currentListedCameras);
    
    if(VmbErrorSuccess == error)
    {
        /* disconnect */
        if(currentListedCameras.size() < m_rCameras.size())
        {
            /* find out what camera is not available */
            for (unsigned int i = 0; i < m_rCameras.size(); i++)
            {
                std::string sModelFromCurrentList;
                std::string sIDFromCurrentList;
                std::string sModelFromOldList;
                std::string sIDFromOldList;

                for (unsigned int y = 0; y < currentListedCameras.size(); y++ )
                {
                    m_rCameras.at(i)->GetModel(sModelFromOldList);
                    m_rCameras.at(i)->GetID(sIDFromOldList);
                    currentListedCameras.at(y)->GetModel(sModelFromCurrentList);
                    currentListedCameras.at(y)->GetID(sIDFromCurrentList);
                    
                    sModelFromOldList.append("-");
                    sModelFromOldList.append(sIDFromOldList);
                    sModelFromCurrentList.append("-");
                    sModelFromCurrentList.append(sIDFromCurrentList);

                    if(0 == sModelFromOldList.compare(sModelFromCurrentList))
                    {
                        bIsFound = true;
                        break;
                    }
                    else
                    {
                        sDisconnectedModel = sModelFromOldList;
                        bIsFound = false;
                    }
                }
                
                if(!bIsFound)
                {
                    /* get camera model and SN when last camera disconnected before closing the viewerwindow */
                    if( (sDisconnectedModel.empty()) && (1 == m_rCameras.size()) )
                    {
                        sIDFromOldList.clear();
                        m_rCameras.at(0)->GetModel(sDisconnectedModel);
                        m_rCameras.at(0)->GetID(sIDFromOldList);
                        sDisconnectedModel.append("-");
                        sDisconnectedModel.append(sIDFromOldList);
                    }

                    break;
                }
            }

            if(!bIsFound)
            {
                for (unsigned int x= 0; x < m_Viewer.size(); x++)
                {
                    /* no camera available, close all open viewer if there are any */
                    if(sDisconnectedModel.empty())
                    {
                        for( unsigned int nPos=0; nPos < m_rCameras.size(); nPos++ )
                        {
                            std::string sID;
                            m_rCameras.at(nPos)->GetModel(sDisconnectedModel);
                            m_rCameras.at(nPos)->GetID(sID);
                            sDisconnectedModel.append("-");
                            sDisconnectedModel.append(sID);
                            closeViewer ( QString::fromStdString(sDisconnectedModel) );    
                        }
                    }
                    else
                        closeViewer ( QString::fromStdString(sDisconnectedModel) );    
                }
            }
        }
    }
    else
    {
        m_Logger->logging("GetCameras Failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR); 
    }

    //m_CameraTree->clear();
    m_rCameras.clear();
    searchCameras();
 //   m_CameraTree->expandAll();

    /* update tree */
    for(unsigned int i = 0; i<m_Viewer.size(); i++)
    {
        for(unsigned int x=0; x<m_GigE.size(); x++)
        {
            if(0 == m_GigE.at(x)->text(0).compare(m_Viewer.at(i)->getCameraID()))
                m_GigE.at(x)->setCheckState(0, Qt::Checked);
        }

        for(unsigned int x=0; x<m_1394.size(); x++)
        {
            if(0 == m_1394.at(x)->text(0).compare(m_Viewer.at(i)->getCameraID()))
                m_1394.at(x)->setCheckState(0, Qt::Checked);
        }

        for(unsigned int x=0; x<m_USB.size(); x++)
        {
            if(0 == m_USB.at(x)->text(0).compare(m_Viewer.at(i)->getCameraID()))
                m_USB.at(x)->setCheckState(0, Qt::Checked);
        }
    }
}

void MainWindow::searchCameras ( void )
{
    /* If update device list callback triggered: Recopy before clearing. If the viewer is already open, use the access list from CopyMap,
       this will avoid the change of menu access entry  info.
       Access information the viewer got when camera is opened will be kept.*/
    rightMouseMenu.close();
    m_CameraTree->clear();
    QMap <QString, QStringList> CamerasPermittedAccessCopyMap     = m_CamerasPermittedAccessMap; 
    QMap <QString, QStringList> PermittedAccessStateListCopyMap   = m_PermittedAccessStateListMap; 

    m_CamerasPermittedAccessMap.clear();
    m_PermittedAccessStateListMap.clear();
    
    QTreeWidgetItem *itemGigERoot = NULL;
    QTreeWidgetItem *item1394Root = NULL;
    QTreeWidgetItem *itemUSBRoot  = NULL;

    m_GigE.clear();
    m_1394.clear();
    m_USB.clear();

    QMap <QString, QString>    ifTypeMap;
    InterfacePtrVector ifPtrVec;
    std::string sInterfaceID;
    VmbError_t error;

    /* list all interfaces found by VmbAPI */
    error = m_VimbaSystem.GetInterfaces(ifPtrVec);
    
    if(VmbErrorSuccess != error)
    {
        m_Logger->logging("GetInterfaces Failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR); 
        return;
    }

    /* check type of Interfaces */
    for(unsigned int i=0; i<ifPtrVec.size(); i++)
    {
        error = ifPtrVec.at(i)->GetID(sInterfaceID);
        if(VmbErrorSuccess != error)
        {
            m_Logger->logging("GetID <Interface "+ QString::number(i)+" Failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR); 
            continue;
        }
        
        VmbInterfaceType ifType = VmbInterfaceUnknown;
        VmbError_t errorGetType = ifPtrVec.at(i)->GetType(ifType) ;
        if(VmbErrorSuccess != errorGetType)
        {
            m_Logger->logging("GetType <Interface "+ QString::number(i)+" Failed, Error: "+QString::number(errorGetType)+" "+ m_Helper.mapReturnCodeToString(errorGetType), VimbaViewerLogCategory_ERROR); 
            continue;
        }

        if((VmbErrorSuccess == errorGetType) && (VmbErrorSuccess == error))
        {
            switch(ifType)
            {
            case VmbInterfaceEthernet: ifTypeMap[QString::fromStdString(sInterfaceID)] = "GigE";
                if(NULL == itemGigERoot)
                {
                    itemGigERoot = new QTreeWidgetItem(m_CameraTree);
                    m_CameraTree->setText(itemGigERoot, "GigE");
                    m_MainWindow.ActionStartOptions->setEnabled(true);
                }
                break;

            case VmbInterfaceFirewire: ifTypeMap[QString::fromStdString(sInterfaceID)] = "1394";
                if(NULL == item1394Root)
                {
                    item1394Root = new QTreeWidgetItem(m_CameraTree);
                    m_CameraTree->setText(item1394Root, "1394");
                }
                break;

            case VmbInterfaceUsb: ifTypeMap[QString::fromStdString(sInterfaceID)] = "USB";
                if(NULL == itemUSBRoot)
                {
                    itemUSBRoot = new QTreeWidgetItem(m_CameraTree);
                    m_CameraTree->setText(itemUSBRoot, "USB");
                }
                break;

            default: break;
            }
        }
    } /* for */

    /* sort the cameras */
    error = m_VimbaSystem.GetCameras(m_rCameras);
    if(VmbErrorSuccess != error)
    {
        m_Logger->logging("GetCameras Failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR); 
        return;
    }

    if(VmbErrorSuccess == error)
    {
        if(0 < m_rCameras.size())
            m_CameraTree->setCursor(Qt::PointingHandCursor);
        else
            m_CameraTree->setCursor(Qt::ArrowCursor);

        for(unsigned int i=0; i<m_rCameras.size(); i++)
        {
            error = m_rCameras.at(i)->GetInterfaceID(sInterfaceID);
            if(VmbErrorSuccess != error)
            {
                m_Logger->logging("GetInterfaceID <Camera "+ QString::number(i)+" Failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR); 
                continue;
            }

            /* get and sort the camera interface */
            QMapIterator <QString, QString> it(ifTypeMap);
            while(it.hasNext())
            {
                it.next();
                if( 0 == it.key().compare(QString::fromStdString(sInterfaceID)))
                {
                    std::string sCamModel;
                    std::string sID;

                    error = m_rCameras.at(i)->GetModel(sCamModel);
                    if(VmbErrorSuccess != error)
                    {
                        m_Logger->logging("GetModel Failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR); 
                        break;
                    }

                    error = m_rCameras.at(i)->GetID(sID);
                    if(VmbErrorSuccess != error)
                    {
                        m_Logger->logging("GetID Failed, Error: "+QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error), VimbaViewerLogCategory_ERROR); 
                        break;
                    }

                    sCamModel.append("-");
                    sCamModel.append(sID);

                    QString sAccess;
                    QStringList sListPermittedAccess;
                    QStringList sListPermittedAccessState;

                    /* Check if viewer already open */
                    bool bIsopen = false;
                    
                    QMap <QString, QString>::iterator itOpenstate;
                    for(itOpenstate = m_IsCameraOpenMap.begin(); itOpenstate != m_IsCameraOpenMap.end(); ++itOpenstate) 
                    {
                        if (0 == itOpenstate.key().compare(QString::fromStdString(sCamModel)))
                        {
                            bIsopen = true;
                            break;
                        }
                    }

                    /*
                    for (unsigned int nPos=0; nPos<m_Viewer.size(); nPos++)
                    {
                        if( 0 == m_Viewer.at(nPos)->getCameraID().compare(QString::fromStdString(sCamModel)) )
                        {
                            bIsopen = true;
                            break;
                        }
                    }
                    */

                    /* camera is open, so keep the information */
                    if(bIsopen)
                    {
                        /* keep the last information */
                        sListPermittedAccess      = getStringListInfo(CamerasPermittedAccessCopyMap   , QString::fromStdString(sCamModel));
                        sListPermittedAccessState = getStringListInfo(PermittedAccessStateListCopyMap , QString::fromStdString(sCamModel));
                        
                        if(sListPermittedAccess.contains("Open FULL ACCESS") && sListPermittedAccess.contains("Open READ ONLY"))
                        {
                            sAccess = "FULL ACCESS";
                        }
                        else if(sListPermittedAccess.contains("Open READ ONLY"))
                        {
                            sAccess = "READ ONLY";
                        }
                    }
                    else /* camera is not open, so get the current supported access */ 
                    {
                        /* check access */
                        VmbAccessModeType accessType = VmbAccessModeFull;
                        if( VmbErrorSuccess == m_rCameras.at(i)->GetPermittedAccess( accessType ))
                        {
                            if( accessType & VmbAccessModeFull)
                            {
                                sAccess = "FULL ACCESS";
                                if(0 == it.value().compare("GigE"))
                                {
                                    sListPermittedAccess      << "Open FULL ACCESS" << "Open READ ONLY" << "Open CONFIG";
                                    sListPermittedAccessState << "false"            << "false"          << "false";
                                }
                                else if(0 == it.value().compare("1394"))
                                {
                                    sListPermittedAccess      << "Open FULL ACCESS" << "Open READ ONLY" ;
                                    sListPermittedAccessState << "false"            << "false"          ;
                                }
                                else if(0 == it.value().compare("USB"))
                                {
                                    sListPermittedAccess      << "Open FULL ACCESS" << "Open READ ONLY" ;
                                    sListPermittedAccessState << "false"            << "false"          ;
                                }
                                
                            }
                            else if( accessType & VmbAccessModeRead)
                            {
                                sAccess = "READ ONLY";
                                if(0 == it.value().compare("GigE"))
                                {
                                    //sListPermittedAccess      << "Open READ ONLY" << "Open CONFIG";
                                    sListPermittedAccess      << "Open READ ONLY";
                                    sListPermittedAccessState << "false"          ;//<< "false";
                                }
                                else if(0 == it.value().compare("1394"))
                                {
                                    sListPermittedAccess      << "Open READ ONLY" ;
                                    sListPermittedAccessState << "false"          ;
                                }
                                else if(0 == it.value().compare("USB"))
                                {
                                    sListPermittedAccess      << "Open READ ONLY" ;
                                    sListPermittedAccessState << "false"          ;
                                }
                                
                            }
                            else if( accessType & VmbAccessModeConfig)
                            {
                                sAccess = "DEVICE CONFIGURATION";
                                if(0 == it.value().compare("GigE"))
                                {
                                    sListPermittedAccess << "Open CONFIG" ;
                                    sListPermittedAccessState << "false"  ;
                                }
                                
                            }
                            else if( accessType & VmbAccessModeLite)
                            {
                                sAccess = "RW WITH NO FEATURE ACCESS";
                                sListPermittedAccess << "Open LITE";
                                sListPermittedAccessState << "false";
                            }
                            else
                            {
                                sAccess = "NO ACCESS";
                            }
                        }
                    }

                    m_CamerasPermittedAccessMap  [QString::fromStdString(sCamModel)]   = sListPermittedAccess;
                    m_PermittedAccessStateListMap[QString::fromStdString(sCamModel)]   = sListPermittedAccessState;

                    if(0 == it.value().compare("GigE"))
                    {
                        QTreeWidgetItem *itemIF_GigE_ID = m_CameraTree->createItem(itemGigERoot, true);
                        itemIF_GigE_ID->setText(0, QString::fromStdString(sCamModel) );
                        itemIF_GigE_ID->setIcon(0, QIcon(":/VimbaViewer/Images/stripes_256.png"));
                    //    if(!bIsopen)
                        {
                            if(   (true)
                               && ( 0 == sAccess.compare("READ ONLY") || 0 == sAccess.compare("NO ACCESS")))
                            {
                                itemIF_GigE_ID->setBackgroundColor(0, Qt::black);
                                itemIF_GigE_ID->setForeground(0, Qt::white);
                                itemIF_GigE_ID->setIcon(0, QIcon(":/VimbaViewer/Images/lock.png"));
                            }
                        }
                    //    else
                        if(bIsopen)
                            itemIF_GigE_ID->setCheckState(0, Qt::Checked);

                        m_GigE.append(itemIF_GigE_ID);
                    }
                    
                    if(0 == it.value().compare("1394"))
                    {
                        QTreeWidgetItem *itemIF_1394_ID = m_CameraTree->createItem(item1394Root, true);
                        itemIF_1394_ID->setText(0, QString::fromStdString(sCamModel) );
                        itemIF_1394_ID->setIcon(0, QIcon(":/VimbaViewer/Images/stripes_256.png"));
                        if(!bIsopen)
                        {
                            if( 0 == sAccess.compare("READ ONLY") || 0 == sAccess.compare("NO ACCESS"))
                            {
                                itemIF_1394_ID->setBackgroundColor(0, Qt::black);
                                itemIF_1394_ID->setForeground(0, Qt::white);
                                itemIF_1394_ID->setIcon(0, QIcon(":/VimbaViewer/Images/lock.png"));
                                if(0 == sAccess.compare("NO ACCESS"))
                                    itemIF_1394_ID->setDisabled(true);
                            }
                        }
                        
                        m_1394.append(itemIF_1394_ID);
                    }
                    
                    if(0 == it.value().compare("USB"))
                    {
                        QTreeWidgetItem *itemIF_USB_ID = m_CameraTree->createItem(itemUSBRoot, true);
                        itemIF_USB_ID->setText(0, QString::fromStdString(sCamModel) );
                        itemIF_USB_ID->setIcon(0, QIcon(":/VimbaViewer/Images/stripes_256.png"));
                        if(!bIsopen)
                        {
                            if( 0 == sAccess.compare("READ ONLY") || 0 == sAccess.compare("NO ACCESS"))
                            {
                                itemIF_USB_ID->setBackgroundColor(0, Qt::black);
                                itemIF_USB_ID->setForeground(0, Qt::white);
                                itemIF_USB_ID->setIcon(0, QIcon(":/VimbaViewer/Images/lock.png"));
                                if(0 == sAccess.compare("NO ACCESS"))
                                    itemIF_USB_ID->setDisabled(true);
                            }
                        }

                        m_USB.append(itemIF_USB_ID);
                    }
                    break;
                }
            }
        }
    }

    m_CameraTree->expandAll();
}

void MainWindow::onRightMouseClicked ( const bool &bIsClicked )
{
    m_bIsRightMouseClicked = bIsClicked;
}

void MainWindow::rightMouseOpenCamera ( bool bOpenAccesState )
{
    QString sStatus;
    QObject *sender = QObject::sender();
    QString sSenderName = sender->objectName();
    m_sOpenAccessType = sSenderName;

    QStringList sAccessList = getStringListInfo(m_PermittedAccessStateListMap , m_sCurrentModel);
    unsigned int nPosition  = getAccessListPosition(sSenderName);
    QString sCurrentState = sAccessList.at(nPosition);
    bool bCurrentState = false;

    if(0 == sCurrentState.compare("false"))
        bCurrentState = false;

    if(0 == sCurrentState.compare("true"))
        bCurrentState = true;

    if(bOpenAccesState == bCurrentState)
    {
        return;
    }
    else
    {
        if(bCurrentState)
        {
            m_CameraTree->setCheckCurrentItem(false);
            sStatus = "false";
        }
        else
        {
            m_CameraTree->setCheckCurrentItem(true);
            sStatus = "true";
        }
    }

    updateAccessStateListMap(nPosition, sStatus, false); 
    m_bIsOpenByRightMouseClick = true;
    onCameraClicked(m_sCurrentModel, m_bIsCurrentModelChecked);
}

/* Find the camera and read all string in StringList within the map */
QStringList MainWindow::getStringListInfo ( const QMap<QString, QStringList> mInfoMap, const QString &sCamID )
{
    QStringList sListInfo;
    QMap<QString, QStringList>::const_iterator itr = mInfoMap.find(sCamID);
    
    /* keep the last information */
    while (itr != mInfoMap.constEnd()) 
    {
        if(0 == itr.key().compare(sCamID))
        {
            sListInfo = itr.value();
            break;
        }

        ++itr;
    }

    return sListInfo;
}

/* get the best access */
QString  MainWindow::getBestAccess ( const QString &sCamID )
{
    QString sBestAccess;
    QStringList sBestList;
    
    QMap<QString, QStringList>::const_iterator itr = m_CamerasPermittedAccessMap.find(sCamID);
    while (itr != m_CamerasPermittedAccessMap.constEnd()) 
    {
        if(0 == itr.key().compare(sCamID))
        {
            sBestList = itr.value();
            break;
        }

        ++itr;
    }

    if(0 != sBestList.size())
        sBestAccess = sBestList.at(0);

    return sBestAccess;
}
/* get the string position of the access in the list */
unsigned int  MainWindow::getAccessListPosition ( const QString &sAccessName )
{
    /* get position */
    QMap<QString, QStringList>::const_iterator itr = m_CamerasPermittedAccessMap.find(m_sCurrentModel);
    QStringList sPermittedAccess;

    while (itr != m_CamerasPermittedAccessMap.constEnd()) 
    {
        if(0 == itr.key().compare(m_sCurrentModel))
        {
            sPermittedAccess = itr.value();
            break;
        }

        ++itr;
    }

    unsigned int nPosition = 0;

    for (unsigned int i = 0; i < sPermittedAccess.size(); i++)
    {
        if(0 == sPermittedAccess.at(i).compare(sAccessName))
        {
            nPosition = i;
            break;
        }    
    }

    return nPosition;

}

/* updating the access list value (true or false) */
void MainWindow::updateAccessStateListMap ( const unsigned int &nPosition, const QString sStatus, const bool &bIsResetAll )
{
    QStringList sPermittedAccess;
    QMap<QString, QStringList>::const_iterator itr = m_PermittedAccessStateListMap.find(m_sCurrentModel);
    while (itr != m_PermittedAccessStateListMap.constEnd()) 
    {
        if(0 == itr.key().compare(m_sCurrentModel))
        {
            sPermittedAccess = itr.value();
            break;
        }

        ++itr;
    }

    QStringList sUpdateList;
    for (unsigned int i = 0; i < sPermittedAccess.size(); i++)
    {
        if(bIsResetAll)
        {
            sUpdateList << "false";
        }
        else
        {
            if( nPosition == i)
            {
                sUpdateList << sStatus;
                continue;
            }

            sUpdateList << sPermittedAccess.at(i);
        }    
    }

    m_PermittedAccessStateListMap[m_sCurrentModel] = sUpdateList;
}

void MainWindow::onCameraClicked ( const QString &sModel, const bool &bIsChecked )
{
    m_bIsCurrentModelChecked = bIsChecked;
    m_sCurrentModel = sModel;
 
    if( m_bIsRightMouseClicked)
    {
        m_bIsRightMouseClicked = false;
        /* show permitted access */
        if( !m_RightMouseAction.isEmpty())
        {
            for (unsigned int nPos = 0; nPos < m_RightMouseAction.size(); nPos++)
            {
                delete m_RightMouseAction.at(nPos);
            }

            m_RightMouseAction.clear();
        }
        
        QStringList sPermittedAccess = getStringListInfo(m_CamerasPermittedAccessMap, sModel);
        
        for (unsigned int i = 0; i < sPermittedAccess.size(); i++)
        {
            m_RightMouseAction.push_back(new QAction(this));
            m_RightMouseAction.last()->setObjectName(sPermittedAccess.at(i));
            m_RightMouseAction.last()->setCheckable(true);
            m_RightMouseAction.last()->setText(sPermittedAccess.at(i));
    
            connect(m_RightMouseAction.last(), SIGNAL(toggled(bool)), this, SLOT(rightMouseOpenCamera(bool)) );
            rightMouseMenu.addAction(m_RightMouseAction.last());
        }

        bool bIsOneOfThemChecked = false;
        QMap<QString, QStringList>::const_iterator it = m_PermittedAccessStateListMap.find(sModel);
        while (it != m_PermittedAccessStateListMap.constEnd()) 
        {
            QStringList sValueList = it.value();
            for(unsigned int i = 0; i < sValueList.size(); i++)
            {
                if(0 == sValueList.at(i).compare("true"))
                {
                    m_RightMouseAction.at(i)->setChecked(true);
                    bIsOneOfThemChecked = true;
                }

                else if(0 == sValueList.at(i).compare("false"))
                    m_RightMouseAction.at(i)->setChecked(false);
            }

            /* disabling other menus entries if one of already selected */
            if(bIsOneOfThemChecked)
            {
                for(unsigned int i = 0; i < sValueList.size(); i++)
                {
                    if(0 != sValueList.at(i).compare("true"))
                    {
                        m_RightMouseAction.at(i)->setEnabled(false);
                    }    
                }
            }

            break;
        }

        rightMouseMenu.exec(QCursor::pos());
    }
    else
    {
        try
        {
            bIsChecked ? openViewer(sModel):closeViewer(sModel);
        }
        catch(const std::exception &e)
        {
            m_Logger->logging("MainWindow <onCameraClicked> Exception: "+QString::fromStdString(e.what()), VimbaViewerLogCategory_ERROR); 
        }
    }

    if(!m_CameraTree->isEnabled())
        m_CameraTree->setEnabled(true);
    
}

void MainWindow::openViewer ( const QString &sCamID )
{
    VmbError_t error;

    try
    {
        /* find out what cam it is */
        for(unsigned int i=0; i<m_rCameras.size(); i++)
        {
            std::string sCamModel;
            std::string sUniqueId;

            error = m_rCameras.at(i)->GetModel(sCamModel);
            if(VmbErrorSuccess != error)
                continue;

            error = m_rCameras.at(i)->GetID(sUniqueId);
            if(VmbErrorSuccess != error)
                continue;

            sCamModel.append("-");
            sCamModel.append(sUniqueId);
            if(0 == sCamID.compare(QString::fromStdString(sCamModel)))
            {  
                /* if it´s not triggered from right mouse click menu, open with best case */
                if(!m_bIsOpenByRightMouseClick )
                    m_sOpenAccessType = getBestAccess(QString::fromStdString(sCamModel));
                
                //keep in mind what viewer is open
                m_IsCameraOpenMap.insert(QString::fromStdString(sCamModel), "true");

                m_Viewer.append(new ViewerWindow(this,0, sCamID, m_sOpenAccessType, m_bIsAutoAdjustPacketSize, m_rCameras.at((i))));

                if(!connect(m_Viewer.back(), SIGNAL(closeViewer (const QString &)), this, SLOT(onCloseFromViewer(const QString &))))
                    m_Logger->logging("MainWindow: Failed connecting SIGNAL (<CameraTreeWindow>cameraClicked) to SLOT(<MainWindow>onCameraClicked)", VimbaViewerLogCategory_ERROR);

                if(!m_CameraTree->isEnabled())
                    m_CameraTree->setDisabled(false);

                QString sBestAccess = getBestAccess(sCamID);
                unsigned int nPos = 0;

                if(m_Viewer.back()->getCamOpenStatus())
                {
                    m_Logger->logging("Opening The Viewer:\t"+sCamID, VimbaViewerLogCategory_INFO);
                    /*Use best case in case open camera directly with no  right mouse click */
                    if(!m_bIsOpenByRightMouseClick )
                    {
                        updateAccessStateListMap(0, "true", false);
                    }

                    QString sAdjustMsg;
                    if(m_Viewer.back()->getAdjustPacketSizeMessage (sAdjustMsg))
                    {
                        sAdjustMsg.contains("Failed") ? m_Logger->logging(sAdjustMsg+sCamID, VimbaViewerLogCategory_INFO) : m_Logger->logging(sAdjustMsg+sCamID, VimbaViewerLogCategory_OK); 
                    }
                }
                else
                {
                    onCloseFromViewer ( sCamID );
                    nPos = getAccessListPosition(sBestAccess);
                    updateAccessStateListMap(nPos, "false", true); /* reset All */

                    if( VmbErrorInvalidAccess == m_Viewer.back()->getOpenError() )
                    {
                        m_Logger->logging("Opening The Viewer:\t"+ 
                                           sCamID + 
                                           " failed! <Cannot open the camera because it is already controlled by another application>", VimbaViewerLogCategory_WARNING); 
                    }
                    else
                    {
                        m_Logger->logging("Opening The Viewer:\t" + 
                                           sCamID + 
                                           " failed! - Error: " +
                                           QString::number(m_Viewer.back()->getOpenError()) +
                                           m_Helper.mapReturnCodeToString(m_Viewer.back()->getOpenError()), VimbaViewerLogCategory_WARNING); 
                    }

                    if (m_sOpenAccessType.isEmpty())
                    {
                        if(!m_Viewer.empty())
                            m_Viewer.pop_back();
                    }
                }
            }
        }
    }
    catch(const std::exception &e)
    {
        m_Logger->logging("MainWindow <openViewer> Exception: " + QString::fromStdString(e.what()), VimbaViewerLogCategory_ERROR); 
    }

    if(m_bIsOpenByRightMouseClick )
        m_bIsOpenByRightMouseClick = false;
}

void MainWindow::closeViewer ( const QString &sCamID )
{
    QMap <QString, QString>::iterator itOpenstate;
    for(itOpenstate = m_IsCameraOpenMap.begin(); itOpenstate != m_IsCameraOpenMap.end(); ++itOpenstate) 
    {
        if (0 == itOpenstate.key().compare(sCamID))
        {
            itOpenstate = m_IsCameraOpenMap.erase(itOpenstate);
            break;
        }
    }

    /* copy ID for logging because sCamID will be deleted (ref) when m_Viewer destroyed */
    QString sID = sCamID;
    m_sCurrentModel = sCamID;
    int nPosition = 0;
    for(QVector<ViewerWindow*>::iterator it = m_Viewer.begin(); it != m_Viewer.end(); ++it)
    {
        if( (0==sCamID.compare((*it)->getCameraID())) )
        { 
            /* delete the object and remove entry */
            delete m_Viewer[nPosition];
            m_Viewer[nPosition] = NULL;
            m_Viewer.erase(it);
            m_Logger->logging("Closing   The Viewer:\t"+sID, VimbaViewerLogCategory_INFO); 
            updateAccessStateListMap(0, "false", true); /* reset all */ 
            break;
        }

        nPosition++;
    }

    if(!m_CameraTree->isEnabled())
        m_CameraTree->setDisabled(false);

    searchCameras();
}

void MainWindow::onCloseFromViewer ( const QString &sModel )
{
    QTreeWidgetItemIterator it(  m_CameraTree );

    /* iterate the tree and find the position of the string, if found uncheck the checkbox */
    while( *it )
    {
        if( (*it)->text(0) == sModel )
        {
            if( Qt::Checked == (*it)->checkState(false) )
            {
                if( 0 == (*it)->text(0).compare(sModel) )
                {
                    (*it)->setSelected(true);
                    (*it)->setCheckState(0, Qt::Unchecked); 
                    //closeViewer(sModel);
                    break;
                }
            }    
        }

        ++it;
    }

    closeViewer(sModel);
}

void MainWindow::on_ActionDiscover_triggered ( void )
{
    for (unsigned int i=0; i < m_Viewer.size(); i++)
    {
        if(m_Viewer.at(i)->getCamOpenStatus())
        {
            closeViewer (m_Viewer.at(i)->getCameraID());
            i--;
        }
    }

    onUpdateDeviceList();
}

void MainWindow::on_ActionClear_triggered ( void )
{
    if(0 != m_Logger->count())
        m_Logger->clear();
}

void MainWindow::on_ActionStartOptions_triggered ( void )
{
    if( QDialog::Accepted == m_StartOptionDialog->exec())
    {
        m_bIsAutoAdjustPacketSize = m_StartOption.AutoAdjustPacketSize_CheckBox->isChecked();
    }
    else
    {
        m_StartOption.AutoAdjustPacketSize_CheckBox->setChecked(m_bIsAutoAdjustPacketSize);
    }
}


