/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ViewerWindow.cpp

  Description: The viewer window framework. 
               This contains of dock widgets like camera feature tree, a histogram, a toolbar and MDI window
               

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


#include "ViewerWindow.h"
#include "SplashScreen.h"
#ifndef _WIN32
    #include <unistd.h>
#endif

#define BUFFER_COUNT 3
#define IMAGES_COUNT 50

ViewerWindow::ViewerWindow ( QWidget *parent, Qt::WFlags flag, QString sID, QString sAccess, bool bAutoAdjustPacketSize, CameraPtr pCam ) : QMainWindow( 0, flag ), 
                                                                               m_DockController( NULL ), 
                                                                               m_DockInformation( NULL ),
                                                                               m_Controller( NULL ),
                                                                               m_ScreenViewer( NULL ),
                                                                               m_InformationWindow( NULL ),
                                                                               m_bHasJustStarted ( false ),
                                                                               m_bIsFirstStart   ( true ),
                                                                               m_bIsCameraRunning ( false ),
                                                                               m_bIsCamOpen ( false ),
                                                                               m_bIsRedHighlighted ( false ),
                                                                               m_bIsDisplayEveryFrame ( false ),
                                                                               m_ImageOptionDialog ( NULL ),
                                                                               m_saveFileDialog ( NULL ),
                                                                               m_getDirDialog ( NULL ),
                                                                               m_bIsTriggeredByMultiSaveBtn (false),
                                                                               m_nNumberOfFramesToSave (0)
{  
    m_pCam = pCam;
    
    bool bIsModelFound = true;
    QPixmap pixmap;
    if(sID.contains("Manta"))
        pixmap.load( ":/VimbaViewer/Images/manta.png" );
    else if(sID.contains("GT"))
        pixmap.load( ":/VimbaViewer/Images/prosilicagt.png" );
    else if(sID.contains("GC"))
        pixmap.load( ":/VimbaViewer/Images/prosilicagc.png" );
    else if(sID.contains("GX"))
        pixmap.load( ":/VimbaViewer/Images/prosilicagx.png" );
    else if(sID.contains("GE"))
        pixmap.load( ":/VimbaViewer/Images/prosilicage.png" );
    else if(sID.contains("Marlin"))
        pixmap.load( ":/VimbaViewer/Images/marlin.png" );
    else if(sID.contains("Guppy"))
        pixmap.load( ":/VimbaViewer/Images/guppy.png" );
    else if(sID.contains("Oscar"))
        pixmap.load( ":/VimbaViewer/Images/oscar.png" );
    else if(sID.contains("Pike"))
        pixmap.load( ":/VimbaViewer/Images/pike.png" );
    else if(sID.contains("Stingray"))
        pixmap.load( ":/VimbaViewer/Images/stingray.png" );
    else if(sID.contains("Bigeye"))
        pixmap.load( ":/VimbaViewer/Images/bigeye.png" );
    else if(sID.contains("Goldeye P"))
        pixmap.load( ":/VimbaViewer/Images/goldeye.png" );
    else if(sID.contains("Goldeye G"))
        pixmap.load( ":/VimbaViewer/Images/goldeye-g.png" );
    else
    {
        pixmap.load( ":/VimbaViewer/Images/stripes_256.png" );
        bIsModelFound = false;
    }

    SplashScreen splashScreen(pixmap, parent, Qt::SplashScreen);
    int nW = ((this->width()/2) - splashScreen.width()/2);
    int nH = ((this->height()/2) - splashScreen.height()/2);
    splashScreen.setGeometry(nW,nH, splashScreen.width(),splashScreen.height());
    splashScreen.show();

    if(bIsModelFound)
        splashScreen.showMessage("" , Qt::AlignHCenter | Qt::AlignVCenter, Qt::white);
    else
        splashScreen.showMessage("" , Qt::AlignHCenter | Qt::AlignVCenter, Qt::black);
    
    VmbError_t errorType;
    QTime openTimer;
    openTimer.start();

    if( 0 == sAccess.compare("Open FULL ACCESS"))
    {
        errorType = m_pCam->Open(VmbAccessModeFull);
        m_sAccessMode = " (FULL ACCESS)";
    }
    else if( 0 == sAccess.compare("Open READ ONLY"))
    {
        errorType = m_pCam->Open(VmbAccessModeRead);
        m_sAccessMode = " (READ ONLY)";
        bAutoAdjustPacketSize = false;
    }
    else if( 0 == sAccess.compare("Open CONFIG"))
    {
        errorType = m_pCam->Open(VmbAccessModeConfig);
        m_sAccessMode = " (CONFIG MODE)";
        bAutoAdjustPacketSize = false;
    }
    else if( 0 == sAccess.compare("Open LITE"))
    {
        errorType = m_pCam->Open(VmbAccessModeLite);
        m_sAccessMode = " (LITE)";
        bAutoAdjustPacketSize = false;
    }
    else
    {
        errorType = VmbErrorInvalidAccess;
        bAutoAdjustPacketSize = false;
    }

    m_OpenError = errorType;

    if( VmbErrorSuccess != errorType )
    {
        openTimer.elapsed();
        return;
    }
    
    m_sCameraID = sID;
    
    /* create ViewerWindow */
    setupUi(this);
    if(!m_sAccessMode.isEmpty())
        sID.append(m_sAccessMode);
    this->setWindowTitle(sID);

    if( 0 == sAccess.compare("Open CONFIG"))
        ActionFreerun->setEnabled(false);
    
    
    /* add Viewer Widget to ViewerWindow*/
    m_pScene        = QSharedPointer<QGraphicsScene>(new QGraphicsScene());
    m_PixmapItem    = new QGraphicsPixmapItem();
    m_ScreenViewer  = new Viewer(Ui::ViewerWindow::centralWidget);
    m_ScreenViewer->setScene(m_pScene.data());
    m_pScene->addItem(m_PixmapItem);    
    this->setCentralWidget( m_ScreenViewer );
    QPixmap image( ":/VimbaViewer/Images/stripes_256.png" );
    m_PixmapItem->setPixmap(image);
    m_ScreenViewer->show();
    m_bIsCamOpen = true;
    
    /* add DockWidgets: Controller and Information */
    m_DockController   = new DockWidgetWindow("Controller  ("+sID+")", this);
    m_DockInformation  = new DockWidgetWindow("Information ("+sID+")", this);
    m_DockHistogram    = new DockWidgetWindow("Histogram   ("+sID+")", this);
    m_DockController->setObjectName("Controller");
    m_DockInformation->setObjectName("Information");
    m_DockHistogram->setObjectName("Histogram");

    this->addDockWidget(static_cast<Qt::DockWidgetArea>(2), m_DockController);
    this->addDockWidget(static_cast<Qt::DockWidgetArea>(8), m_DockInformation);
    this->addDockWidget(static_cast<Qt::DockWidgetArea>(1), m_DockHistogram);
    m_DockHistogram->hide();
        
    /* add Controller Tree */
    
    QWidget     *dockWidgetContents   = new QWidget();
    QVBoxLayout *verticalLayout2      = new QVBoxLayout(dockWidgetContents); 
    QSplitter   *splitter             = new QSplitter(dockWidgetContents);
    QWidget     *verticalLayoutWidget = new QWidget(splitter);
    QVBoxLayout *verticalLayout       = new QVBoxLayout(verticalLayoutWidget); 
    QTabWidget    *tabWidget          = new QTabWidget(verticalLayoutWidget);
    QWidget        *widgetTree        = new QWidget(); 
    QVBoxLayout *verticalLayout3      = new QVBoxLayout(widgetTree);
    
    m_Description = new QTextEdit();
    m_Controller = new ControllerTreeWindow(m_sCameraID, widgetTree, bAutoAdjustPacketSize, m_pCam);
   
    m_Description->setLineWrapMode(QTextEdit::NoWrap); 
    m_Description->setReadOnly(true);
    m_Description->setStyleSheet("font: 12px;\n" "font-family: Verdana;\n");
    
    QList<int> listSize;
    listSize << 5000;
    splitter->setChildrenCollapsible(false);
    splitter->setOrientation(Qt::Vertical);
    splitter->addWidget(verticalLayoutWidget);
    splitter->addWidget(m_Description);
    splitter->setSizes(listSize);
    
    /* Filter Pattern */
    QHBoxLayout *pattern_HLayout = new QHBoxLayout();
    m_FilterPatternLineEdit = new LineEditCompleter(this);
    m_FilterPatternLineEdit->setText("Example: Gain|Width");
    m_FilterPatternLineEdit->setToolTip("To filter multiple features, e.g: Width|Gain|xyz|etc");
    m_FilterPatternLineEdit->setCompleter(m_Controller->getListCompleter());
    m_FilterPatternLineEdit->setMinimumWidth(150);
    QLabel *filterPatternLabel = new QLabel(tr("Filter pattern:"));
    filterPatternLabel->setStyleSheet("font-weight: bold;");
    QPushButton *patternButton = new QPushButton(tr("Search"));
    
    pattern_HLayout->addWidget(filterPatternLabel);
    pattern_HLayout->addWidget(m_FilterPatternLineEdit);
    pattern_HLayout->addWidget(patternButton);
    verticalLayout3->addLayout(pattern_HLayout);
   
    connect(m_FilterPatternLineEdit, SIGNAL(returnPressed()), this, SLOT(textFilterChanged()));
    connect(patternButton, SIGNAL(clicked(bool)), this, SLOT(textFilterChanged()));

    verticalLayout->setContentsMargins(0, 0, 0, 0);
    verticalLayout->addWidget(tabWidget);
    verticalLayout2->addWidget(splitter);
    verticalLayout3->addWidget(m_Controller);
    
    tabWidget->addTab(widgetTree, QString());
    tabWidget->setTabText(0,"Properties");
    tabWidget->setStyleSheet("color: rgb(0, 0, 0);");
    m_DockController->setWidget(dockWidgetContents);
    
    /* tooltip checkbox */
    m_ToolTipCheckBox     = new QCheckBox();
    m_ToolTipCheckBox->setText("Tooltip ON");
    m_ToolTipCheckBox->setChecked(true);
    verticalLayout3->addWidget(m_ToolTipCheckBox);
    connect( m_ToolTipCheckBox, SIGNAL(clicked(bool)), this, SLOT(onTooltipCheckBoxClick(bool)) );
    

    connect(m_DockController,  SIGNAL(topLevelChanged  (bool)), this, SLOT(onfloatingDockChanged(bool)));
    connect(m_DockInformation, SIGNAL(topLevelChanged  (bool)), this, SLOT(onfloatingDockChanged(bool)));
    connect(m_DockHistogram,   SIGNAL(topLevelChanged  (bool)), this, SLOT(onfloatingDockChanged(bool)));

    connect(m_DockController,  SIGNAL(visibilityChanged(bool)), this, SLOT(onVisibilityChanged(bool)));
    connect(m_DockInformation, SIGNAL(visibilityChanged(bool)), this, SLOT(onVisibilityChanged(bool)));
    connect(m_DockHistogram,   SIGNAL(visibilityChanged(bool)), this, SLOT(onVisibilityChanged(bool)));

    connect(m_Controller,      SIGNAL(setDescription(const QString &)),  this, SLOT(onSetDescription(const QString &)));
    connect(m_Controller,      SIGNAL(setEventMessage(const QStringList &)), this, SLOT(onSetEventMessage(const QStringList &)), Qt::QueuedConnection);
    connect(m_Controller,      SIGNAL(acquisitionStartStop(const QString &)), this, SLOT(onAcquisitionStartStop(const QString &)));
    connect(m_Controller,      SIGNAL(updateBufferSize(void)), this, SLOT(onPrepareCapture(void)));
    connect(m_Controller,      SIGNAL(resetFPS(void)), this, SLOT(onResetFPS(void)));
    connect(m_Controller,      SIGNAL(logging(const QString &)), this, SLOT(onFeedLogger( const QString &)));
    connect(m_ScreenViewer,    SIGNAL(savingImage(void)), this, SLOT(on_ActionSaveAs_triggered(void)));
    connect(m_ScreenViewer,    SIGNAL(setColorInterpolationState(const bool &)), this, SLOT(onSetColorInterpolation(const bool &)));
    
    /* create FrameObserver to get frames from camera */
    SP_SET( m_pFrameObs, new FrameObserver(m_pCam) );

    connect(SP_ACCESS( m_pFrameObs ), SIGNAL(frameReadyFromObserver (QImage , const QString&, const QString&, const QString&)), 
        this, SLOT(onimageReady(QImage , const QString&, const QString&, const QString&)));

    connect(SP_ACCESS( m_pFrameObs ), SIGNAL(setCurrentFPS (const QString &)), 
        this, SLOT(onSetCurrentFPS(const QString &)));

    connect(SP_ACCESS( m_pFrameObs ), SIGNAL(setFrameCounter (const unsigned int &)), 
        this, SLOT(onSetFrameCounter(const unsigned int &)));

    /* HISTOGRAM: We need to register QVector<QVector <quint32> > because it is not known to Qt's meta-object system */
    qRegisterMetaType< QVector<QVector <quint32> > >("QVector<QVector <quint32> >");
    qRegisterMetaType< QVector <QStringList> >("QVector <QStringList>");

    connect(m_pFrameObs.get(), SIGNAL(histogramDataFromObserver ( const QVector<QVector <quint32> > &, const QString &, const double &, const double &, const QVector <QStringList>& )), 
        this, SLOT(onSetHistogramData( const QVector<QVector <quint32> > &, const QString &, const double &, const double & , const QVector <QStringList>&)));

    /* setup information window */
    m_InformationWindow = new MainInformationWindow(m_DockInformation, 0, m_pCam);
    m_InformationWindow->openLoggingWindow();
    
    m_DockInformation->setWidget(m_InformationWindow);
    int openElapsedTime = openTimer.elapsed();
    if( openElapsedTime <= 2500 )
    {
#ifdef WIN32
        Sleep(2500-openElapsedTime);
#else
        usleep( (2500-openElapsedTime) * 1000 );
#endif
    }
    
    /* use default setting position and geometry */
    QSettings settings("Allied Vision Technologies", "AVT VimbaViewer");
    this->restoreGeometry(settings.value("geometry").toByteArray());
    this->restoreState( settings.value("state").toByteArray(), 0 );    
    this->show();

    (!m_DockController->isFloating() && !m_DockInformation->isFloating() && m_DockController->isVisible() && m_DockInformation->isVisible()) ? ActionResetPosition->setEnabled(false) : ActionResetPosition->setEnabled(true);
    
    if( VmbErrorSuccess != m_Controller->getTreeStatus())
    {
        onFeedLogger("ERROR: GetFeatures returned: "+QString::number(m_Controller->getTreeStatus()) + " " +  m_Helper.mapReturnCodeToString(m_Controller->getTreeStatus()));
    }

    /* Histogram */
    m_HistogramWindow = new HistogramWindow(this);
    m_DockHistogram->setWidget(m_HistogramWindow);
    m_HistogramWindow->createGraphWidgets();
    
    m_DockHistogram->isVisible() ? ActionHistogram->setChecked(true) : ActionHistogram->setChecked(false);

    /* Statusbar */
    m_OperatingStatusLabel = new QLabel(" Ready ");
    m_FormatLabel          = new QLabel;
    m_ImageSizeLabel       = new QLabel;
    m_FramesLabel          = new QLabel;
    m_FramerateLabel       = new QLabel;

    statusbar->addWidget(m_OperatingStatusLabel);
    statusbar->addWidget(m_ImageSizeLabel);
    statusbar->addWidget(m_FormatLabel);
    statusbar->addWidget(m_FramesLabel);
    statusbar->addWidget(m_FramerateLabel);

    m_OperatingStatusLabel->setStyleSheet("background-color: rgb(0,0, 0); color: rgb(255,255,255)");

    m_TextItem =  new QGraphicsTextItem;
    QFont serifFont("Arial", 12, QFont::Bold);
    m_TextItem->setFont(serifFont);
    m_TextItem->setDefaultTextColor(Qt::red);

    /*Save Images Option */
    m_ImageOptionDialog = new QDialog(this);
    m_SaveImageOption.setupUi(m_ImageOptionDialog);
    if(!settings.value(m_sCameraID).toString().isEmpty())
        m_SaveImageOption.ImageDestination_Edit->setText (settings.value(m_sCameraID).toString());
    else
    {
        m_SaveImageOption.ImageDestination_Edit->setText( QString(   
#ifdef WIN32
            "C:\\"
#else
            "/home/"
#endif
            ));
    }

    connect(m_SaveImageOption.ImageDestinationButton, SIGNAL(clicked()),this, SLOT(getSaveDestinationPath()));
    connect(m_ImageOptionDialog, SIGNAL(accepted()),this, SLOT(acceptSaveImagesDlg()));
    connect(m_ImageOptionDialog, SIGNAL(rejected()),this, SLOT(rejectSaveImagesDlg()));

    if(!settings.value(m_sCameraID+"SaveImageName").toString().isEmpty())
        m_SaveImageOption.ImageName_Edit->setText (settings.value(m_sCameraID+"SaveImageName").toString());
    else
        m_SaveImageOption.ImageName_Edit->setText("VimbaImage");
    
    /* Direct Access */
    m_AccessDialog = new QDialog(this);
    m_DirectAccess.setupUi(m_AccessDialog);    
    m_DirectAccess.RegAdd_Edit->setMaxLength(8);
    m_DirectAccess.RegData_Edit->setMaxLength(8);
    m_DirectAccess.RegAdd_Edit->setText("FFFFFFFF");
    m_DirectAccess.RegData_Edit->setText("00000000");
    m_DirectAccess.RegDataDec_Edit->setText("0");
    m_AccessDialog->setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint & ~Qt::WindowMinimizeButtonHint); 
    QRegExp rxHex("[0-9A-Fa-f]{1,8}");
    m_DirectAccess.RegAdd_Edit->setValidator(new QRegExpValidator(rxHex, m_DirectAccess.RegAdd_Edit)); 
    m_DirectAccess.RegData_Edit->setValidator(new QRegExpValidator(rxHex, m_DirectAccess.RegData_Edit)); 

    QRegExp rxDec("[0-9]{1,10}");
    m_DirectAccess.RegDataDec_Edit->setValidator(new QRegExpValidator(rxDec, m_DirectAccess.RegDataDec_Edit)); 

    connect(m_DirectAccess.RegWrite_Button, SIGNAL(clicked()),this, SLOT(writeRegisterData()));
    connect(m_DirectAccess.RegRead_Button,  SIGNAL(clicked()),this, SLOT(readRegisterData()));
    connect(m_DirectAccess.RegData_Edit, SIGNAL(textChanged(const QString&)),this, SLOT(directAccessHexTextChanged(const QString&)));
    connect(m_DirectAccess.RegDataDec_Edit, SIGNAL(textChanged(const QString&)),this, SLOT(directAccessDecTextChanged(const QString&)));

    /*Viewer Option */
    m_ViewerOptionDialog = new QDialog(this);
    m_ViewerOption.setupUi(m_ViewerOptionDialog);
    connect( m_ViewerOption.DisplayInterval_CheckBox, SIGNAL(clicked(bool)), this, SLOT(displayEveryFrameClick(bool)));

    if (ActionHistogram->isChecked())
    {
        m_HistogramWindow->initializeStatistic();
        m_pFrameObs->enableHistogram(true);
    }
    else
        m_pFrameObs->enableHistogram(false);

    m_Timer = new QTimer(this);
    connect(m_Timer, SIGNAL(timeout()), this, SLOT(updateColorInterpolationState()));

    /* enable/disable menu */
    connect(m_Controller, SIGNAL(enableViewerMenu(bool)), this, SLOT(enableMenuAndToolbar(bool)));

    /* saving images */
    m_SaveImageThread = QSharedPointer<SaveImageThread>(new SaveImageThread());
    connect(m_SaveImageThread.data(), SIGNAL(setPosition(unsigned int)), this, SLOT(onSaving(unsigned int)));
    m_SaveImageThread->m_nImagePosition = 0;

    m_FilterPatternLineEdit->setFocus();
    m_FilterPatternLineEdit->selectAll();

#ifdef WIN32
    m_SelectedExtension = ".bmp";
#else
    m_SelectedExtension = ".png";
#endif
}

ViewerWindow::~ViewerWindow ( void )
{
    /* save setting position and geometry from last session */
    QSettings settings("Allied Vision Technologies", "AVT VimbaViewer");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("state", saveState(0));
    
    /* If cam is open */
    if(!m_sCameraID.isEmpty())
    {
        settings.setValue(m_sCameraID,  m_SaveImageOption.ImageDestination_Edit->text());
        if(!m_SaveName.isEmpty())
            settings.setValue(m_sCameraID +"SaveImageName", m_SaveName+"_"+QString::number(m_SaveImageThread->m_nImagePosition));

        if( NULL != m_saveFileDialog )
        {
            delete m_saveFileDialog;
            m_saveFileDialog = NULL;
        }

        VmbError_t error = m_pCam->EndCapture();
        if( VmbErrorSuccess == error )
            error = m_pCam->FlushQueue();
        if( VmbErrorSuccess == error ) 
            error = m_pCam->RevokeAllFrames();

        m_pCam->Close();
    }
}

void ViewerWindow::textFilterChanged ( void )
{
    QPixmap pixmap( ":/VimbaViewer/Images/refresh.png" );
    SplashScreen splashScreen(pixmap, m_Controller, Qt::SplashScreen);
    int nW = ((m_Controller->width()/2) - splashScreen.width()/2);
    int nH = ((m_Controller->height()/2) - splashScreen.height()/2);
    splashScreen.setGeometry(nW,nH, splashScreen.width(),splashScreen.height());
    splashScreen.show();
    splashScreen.showMessage("Please wait..." , Qt::AlignHCenter | Qt::AlignVCenter, Qt::red);

    QRegExp::PatternSyntax syntax = QRegExp::PatternSyntax(0);
    Qt::CaseSensitivity caseSensitivity = Qt::CaseInsensitive;
        
    QRegExp regExp(m_FilterPatternLineEdit->text(), caseSensitivity, syntax);
    m_Controller->m_ProxyModel->setFilterRegExp(regExp);
    m_Controller->expandAll();
    m_Controller->updateUnRegisterFeature();
    m_Controller->updateRegisterFeature();
    m_FilterPatternLineEdit->setFocus();
    m_FilterPatternLineEdit->selectAll();
}

void ViewerWindow::enableMenuAndToolbar ( bool bValue )
{
    menuBarMainWindow->setEnabled(bValue);
    toolBar->setEnabled(bValue);
}

void ViewerWindow::onTooltipCheckBoxClick ( bool bValue )
{
    m_Controller->showTooltip ( bValue );
}

void ViewerWindow::updateColorInterpolationState ( void )
{
    /* update interpolation state after start */
    m_ScreenViewer->updateInterpolationState(m_pFrameObs->getColorInterpolation());
    m_Timer->stop();
}

void ViewerWindow::onSetColorInterpolation ( const bool &bState )
{
    m_pFrameObs->setColorInterpolation(bState);
}

bool ViewerWindow::getCamOpenStatus ( void )
{
    return m_bIsCamOpen;
}

bool  ViewerWindow::getAdjustPacketSizeMessage ( QString &sMessage )
{
    if(m_Controller->isGigE())
    {
        if(VmbErrorSuccess == m_Controller->getTreeStatus())
        {
            sMessage = "Packet Size Adjusted:\t";
        }
        else
        {
            sMessage = "Failed To Adjust Packet Size!";
            sMessage.append(" Reason: " + m_Helper.mapReturnCodeToString(m_Controller->getTreeStatus()));
        }

        return true;
    }

    return false;
}

VmbError_t ViewerWindow::getOpenError ( void )
{
    return m_OpenError;
}

QString ViewerWindow::getCameraID( void ) 
{ 
    return m_sCameraID; 
}

void ViewerWindow::closeEvent ( QCloseEvent *event ) 
{
    emit closeViewer( m_sCameraID );   
}

void ViewerWindow::displayEveryFrameClick ( bool bValue )
{
    m_ViewerOption.Note_Label->setEnabled(bValue);
    m_bIsDisplayEveryFrame = bValue;    
}

/* Viewer Option */
void ViewerWindow::on_ActionDisplayOptions_triggered( void )
{
    m_ViewerOptionDialog->exec();
}

void ViewerWindow::on_ActionResetPosition_triggered ( void )
{
    m_DockController ->setFloating(false);
    m_DockInformation->setFloating(false);
    m_DockHistogram  ->setFloating(false);
    m_DockController ->show();
    m_DockInformation->show();
    m_DockHistogram  ->hide();
    ActionHistogram  ->setChecked(false);    
    ActionResetPosition->setEnabled(false);
}

/* Direct Register Access */
void ViewerWindow::on_ActionRegister_triggered ( void )
{
    m_DirectAccess.RegAccessError_Label->clear();
    m_DirectAccess.RegAccessError_Label->setStyleSheet("");
    m_DirectAccess.RegAdd_Edit->setFocus();
    m_DirectAccess.RegAdd_Edit->selectAll();
    m_AccessDialog->show();
}

void ViewerWindow::directAccessHexTextChanged ( const QString &sText )
{
    bool bOk;
    unsigned int sDec = sText.toUInt(&bOk, 16);

    if(0 == QString::number(sDec).compare(m_DirectAccess.RegDataDec_Edit->text()))
        return;
    else
        m_DirectAccess.RegDataDec_Edit->setText(QString::number(sDec));
}

void ViewerWindow::directAccessDecTextChanged ( const QString &sText )
{
    qlonglong lDecValue = sText.toLongLong();
    QString sHex;
    sHex.setNum(lDecValue,16);

    if(0 == sHex.compare(m_DirectAccess.RegData_Edit->text()))
        return;
    else
        m_DirectAccess.RegData_Edit->setText(sHex);
}

void ViewerWindow::writeRegisterData ( void )
{
    bool bOk;
    qlonglong lRegAddress = m_DirectAccess.RegAdd_Edit->text().toLongLong(&bOk, 16);
    qlonglong lRegData    = m_DirectAccess.RegData_Edit->text().toLongLong(&bOk, 16);

    m_DirectAccess.RegAccessError_Label->clear();
    m_DirectAccess.RegAccessError_Label->setStyleSheet("background-color: rgb(240,240,240); color: rgb(0,0,0)");

    m_DirectAccess.RegDataDec_Edit->setFocus();
    m_DirectAccess.RegDataDec_Edit->selectAll();

    std::vector<VmbUint64_t> address;
    address.push_back((VmbUint64_t)lRegAddress);
    std::vector<VmbUint64_t> data;
    data.push_back((VmbUint64_t)lRegData);
    VmbError_t errorType = m_pCam->WriteRegisters(address, data);
    if( VmbErrorSuccess != errorType )
    {
        m_DirectAccess.RegAccessError_Label->setStyleSheet("background-color: rgb(0,0,0); color: rgb(255,0,0)");
        m_DirectAccess.RegAccessError_Label->setText(" Write Register Failed! <Error: " + QString::number(errorType) + ">");
    }
}

void ViewerWindow::readRegisterData ( void )
{
    bool bOk;
    qlonglong lRegAddress = m_DirectAccess.RegAdd_Edit->text().toLongLong(&bOk, 16);

    m_DirectAccess.RegAccessError_Label->clear();
    m_DirectAccess.RegAccessError_Label->setStyleSheet("background-color: rgb(240,240,240); color: rgb(0,0,0)");

    std::vector<VmbUint64_t> address;
    address.push_back((VmbUint64_t)lRegAddress);

    std::vector<VmbUint64_t> data;    
    data.resize(1);
    VmbError_t errorType = m_pCam->ReadRegisters(address, data);
    if( VmbErrorSuccess != errorType )
    {
        m_DirectAccess.RegAccessError_Label->setStyleSheet("background-color: rgb(0,0,0); color: rgb(255,0,0)");
        m_DirectAccess.RegAccessError_Label->setText(" Read Register Failed! <Error: " + QString::number(errorType) + ">");
        return;
    }

    QString sData = QString("%1").arg(data[0], 0, 16);
    m_DirectAccess.RegData_Edit->setText(sData);
}

/* Saving an image */
void ViewerWindow::on_ActionSaveAs_triggered ( void )
{
    QPixmap image = m_PixmapItem->pixmap();
    QString fileExtension;

    /* Get all inputformats */
    unsigned int nFilterSize = QImageReader::supportedImageFormats().count();
    for (int i = nFilterSize-1; i >= 0; i--) 
    {
        fileExtension += "."; /* Insert wildcard */
        fileExtension += QString(QImageReader::supportedImageFormats().at(i)).toLower(); /* Insert the format */
        if(0 != i)
            fileExtension += ";;"; /* Insert a space */
    }

     if( NULL != m_saveFileDialog )
     {
         delete m_saveFileDialog;
         m_saveFileDialog = NULL;
     }

    m_saveFileDialog = new QFileDialog ( this, tr("Save Image"), m_SaveFileDir, fileExtension );
    m_saveFileDialog->selectFilter(m_SelectedExtension);
    m_saveFileDialog->setAcceptMode(QFileDialog::AcceptSave);
    
    if(m_saveFileDialog->exec())
    {   //OK
       m_SelectedExtension = m_saveFileDialog->selectedNameFilter();
       m_SaveFileDir = m_saveFileDialog->directory().absolutePath();
       QStringList files = m_saveFileDialog->selectedFiles();

       if(!files.isEmpty())
       {
           QString fileName = files.at(0);
           
           if(!fileName.endsWith(m_SelectedExtension))
               fileName.append(m_SelectedExtension);
           
           if(image.save(fileName))
           {
               QMessageBox::warning( this, tr("AVT Vimba SampleViewer"), "Saving image: " + fileName + " is DONE!" );
           }
           else
           {
               QMessageBox::warning( this, tr("AVT Vimba SampleViewer"), "FAILED TO SAVE IMAGE!" );
           }   
       }    
    }
}

/* Saving multiple images */
void ViewerWindow::on_ActionSaveOptions_triggered ( void )
{
    QStringList sListFormat;

    for (int i = 0; i < QImageReader::supportedImageFormats().count(); i++) 
    {
        QString sTemp;
        sTemp.append("."); /* Insert wildcard */
        sTemp.append(QString(QImageReader::supportedImageFormats().at(i)).toLower()); /* Insert the format */
        sListFormat << sTemp;
    }

    sListFormat << ".bin";

    m_SaveImageOption.ImageFormat_ComboBox->clear();
    m_SaveImageOption.ImageFormat_ComboBox->addItems(sListFormat);
    m_ImageOptionDialog->setModal(true);
    m_ImageOptionDialog->show();
}

void ViewerWindow::acceptSaveImagesDlg ( void )
{
    if( !m_SaveImageOption.ImageDestination_Edit->text().isEmpty() && 
        !m_SaveImageOption.ImageName_Edit->text().isEmpty() )
    {            
        m_ImagePath =  m_SaveImageOption.ImageDestination_Edit->text();
    }
    else
    {
        if(m_SaveImageOption.ImageDestination_Edit->text().isEmpty())
            QMessageBox::warning( this, tr("AVT Vimba SampleViewer"), "<Save Image Options> Please choose your destination path!" );
    
        if(m_SaveImageOption.ImageName_Edit->text().isEmpty())
            QMessageBox::warning( this, tr("AVT Vimba SampleViewer"), "<Save Image Options> Please give a name!" );

        m_ImageOptionDialog->setModal(true);
        m_ImageOptionDialog->show();
    }
    
    /* name check existing files */
    QDir destDir( m_SaveImageOption.ImageDestination_Edit->text());
    QStringList filter;
    
    for (int i = 0; i < QImageReader::supportedImageFormats().count(); i++) 
    {
        QString sTemp;
        sTemp.append("*."); /* Insert wildcard */
        sTemp.append(QString(QImageReader::supportedImageFormats().at(i)).toLower()); /* Insert the format */
        filter << sTemp;
    }
    
    destDir.setNameFilters(filter);
    QStringList files = destDir.entryList();
    
    bool bRes = true;
    while(bRes)
    {
        bRes = checkUsedName(files);
    }   

    if(0 < m_SaveImageOption.NumberOfFrames_SpinBox->value())
        ActionSaveImages->setEnabled(true);
    else
        ActionSaveImages->setEnabled(false);
}

void ViewerWindow::rejectSaveImagesDlg ( void )
{
    QString sPathBefore = m_SaveImageOption.ImageDestination_Edit->text();
    m_SaveImageOption.ImageDestination_Edit->setText(sPathBefore);
}

bool ViewerWindow::checkUsedName ( const QStringList &files )
{
    for( int nPos = 0; nPos < files.size(); nPos++ )
    {
        if(0 == files.at(nPos).compare(m_SaveImageOption.ImageName_Edit->text()+"_1"+m_SaveImageOption.ImageFormat_ComboBox->currentText()))
        {
            m_SaveImageOption.ImageName_Edit->setText(m_SaveImageOption.ImageName_Edit->text()+"_New");
            return true;
        }
    }

    return false;
}

bool ViewerWindow::isDestPathWritable  ( void )
{
    /* check permission of destination */
    QString sTmpPath = m_SaveImageOption.ImageDestination_Edit->text().append("/").append("fileTmp.dat");
    QFile fileTmp(sTmpPath);

    if(!fileTmp.open( QIODevice::ReadWrite|QIODevice::Text))
    {
        QMessageBox::warning( this, tr("AVT Vimba SampleViewer"), "<Save Image Options> No permission to write to destination path. Please select another one! " );
        return false;
    }

    fileTmp.close();
    QFile::remove(sTmpPath);
    return true;
}

/* confirm saving from toolbar */
void ViewerWindow::on_ActionSaveImages_triggered ( void )
{

#ifdef _WIN32 // Do the check always on Windows
    if (!isDestPathWritable())
    {
        ActionSaveImages->setEnabled(false);
        return;
    }
#else
    #ifndef WIN32
        if (!isDestPathWritable())
        {
            ActionSaveImages->setEnabled(false);
            return;
        }
    #endif
#endif

    //overwrite existing name
    if(0 != m_SaveImageOption.ImageName_Edit->text().compare(m_SaveImageName))
    {
        m_SaveImageThread->m_nImagePosition = 0;
        m_SaveImageName = m_SaveImageOption.ImageName_Edit->text();
    }
  
    m_nImageCounter = 0;
    m_nNumberOfFramesToSave = m_SaveImageOption.NumberOfFrames_SpinBox->value();
    m_SaveFormat = m_SaveImageOption.ImageFormat_ComboBox->currentText();
    m_SaveName = m_SaveImageOption.ImageName_Edit->text();

    if( 0 < m_nNumberOfFramesToSave )
    {
        if( 0 == m_SaveImageOption.ImageFormat_ComboBox->currentText().compare(".bin"))
            m_pFrameObs->saveRawData(m_nNumberOfFramesToSave, m_ImagePath, m_SaveName);

        m_bIsTriggeredByMultiSaveBtn = true;
        ActionFreerun->setChecked(true);
        on_ActionFreerun_triggered();
        ActionFreerun->setEnabled(false);
    }
}

void ViewerWindow::getSaveDestinationPath ( void )
{
    if( NULL != m_getDirDialog )
    {
        delete m_getDirDialog;
        m_getDirDialog = NULL;
    }

    m_getDirDialog = new QFileDialog ( this, tr("Destination"), m_SaveImageOption.ImageDestination_Edit->text());
    QFileDialog::Options options = QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly |QFileDialog::DontUseNativeDialog;
    m_getDirDialog->setOptions(options);
    m_getDirDialog->setLabelText(QFileDialog::LookIn, "Destination Path");
    m_getDirDialog->setLabelText(QFileDialog::FileType, "Type"); 
    m_getDirDialog->setFileMode( QFileDialog::Directory);
  
    QString sDir;
    if( m_getDirDialog->exec() )
    {   //OK
        sDir = m_getDirDialog->directory().absolutePath();
    }
  
    if(!sDir.isEmpty())
        m_SaveImageOption.ImageDestination_Edit->setText(sDir);
}

void ViewerWindow::on_ActionLeftRotation_triggered ( void )
{
    m_ScreenViewer->rotate(-90);
}

void ViewerWindow::on_ActionRightRotation_triggered ( void )
{
    m_ScreenViewer->rotate(90);
}

void ViewerWindow::on_ActionZoomOut_triggered ( void )
{
    m_ScreenViewer->zoomOut();
}

void ViewerWindow::on_ActionZoomIn_triggered ( void )
{
    m_ScreenViewer->zoomIn();
}

void ViewerWindow::on_ActionOriginalSize_triggered ( void )
{
    m_ScreenViewer->setDefaultSize();
}

void ViewerWindow::on_ActionFitToWindow_triggered ( void )
{
    m_ScreenViewer->setDefaultSize();

    if (ActionFitToWindow->isChecked())
    {
        m_ScreenViewer->setToolTip(tr(""));
        m_ScreenViewer->fitInView(m_pScene->itemsBoundingRect(), Qt::IgnoreAspectRatio);            
        m_ScreenViewer->enableFitToWindow(true);
        ActionLeftRotation->setEnabled(false);
        ActionRightRotation->setEnabled(false);
        ActionOriginalSize->setEnabled(false);
        ActionZoomIn->setEnabled(false);
        ActionZoomOut->setEnabled(false);
    }
    else
    {    
        m_ScreenViewer->enableFitToWindow(false);
        ActionLeftRotation->setEnabled(true);
        ActionRightRotation->setEnabled(true);
        ActionOriginalSize->setEnabled(true);
        ActionZoomIn->setEnabled(true);
        ActionZoomOut->setEnabled(true);
    }
}

void ViewerWindow::on_ActionHistogram_triggered ( void )
{
    m_HistogramWindow->deinitializeStatistic();

    if (ActionHistogram->isChecked())
    {
        m_HistogramWindow->initializeStatistic();
        m_DockHistogram->show();
        m_pFrameObs->enableHistogram(true);
    }
    else
    {
        m_DockHistogram->hide();
        m_pFrameObs->enableHistogram(false);
    }
      
    if(!ActionHistogram->isChecked() && m_DockController->isVisible() && m_DockInformation->isVisible())
        ActionResetPosition->setEnabled(false);
}

void ViewerWindow::onfloatingDockChanged ( bool bIsFloating )
{
    if( (m_DockController->isFloating()  || (false == m_DockController->isVisible())) ||
        (m_DockInformation->isFloating() || (false == m_DockInformation->isVisible())) )
        ActionResetPosition->setEnabled(true);
    else
        ActionResetPosition->setEnabled(false);

    if(m_DockHistogram->isVisible())
        ActionHistogram->setChecked(true);
}

void ViewerWindow::onVisibilityChanged ( bool bIsVisible )
{
    if( m_DockController->isVisible()  && !m_DockController->isFloating() && 
        m_DockInformation->isVisible() && !m_DockInformation->isFloating() )
    {
        ActionResetPosition->setEnabled(false);
    }
    else
    {
        ActionResetPosition->setEnabled(true);
    }

    if(!m_DockHistogram->isVisible())
    {
        if (ActionHistogram->isChecked() )
        {
            ActionHistogram->setChecked(false);
        }
    }
    else
    {
        ActionHistogram->setChecked(true);
    }
}

void ViewerWindow::onResetFPS ( void )
{
    SP_ACCESS( m_pFrameObs )->resetFrameCounter(false);
}

void ViewerWindow::onSetCurrentFPS( const QString &sFPS )
{
    m_FramerateLabel->setText( QString::fromStdString(" Current FPS: ")+ sFPS + " " );
}

void ViewerWindow::onSetFrameCounter( const unsigned int &nNumberOfFrames )
{
    m_FramesLabel->setText( "Frames: " + QString::number(nNumberOfFrames) + " " );
}

void ViewerWindow::onSetEventMessage ( const QStringList &sMsg )
{
    m_InformationWindow->setEventMessage(sMsg);
}

void ViewerWindow::onSetDescription ( const QString &sDesc )
{
    m_Description->setText(sDesc);
}

void ViewerWindow::onSetHistogramData ( const QVector<QVector <quint32> > &histData, const QString &sHistogramTitle, 
                                        const double &nMaxHeight_YAxis, const double &nMaxWidth_XAxis, const QVector <QStringList> &statistics )
{
    if( ActionHistogram->isChecked() )
    {
        QStringList ColorComponentList;
        QStringList MinimumValueList;
        QStringList MaximumValueList;
        QStringList AverageValueList;
        QString sFormat;

        if(sHistogramTitle.contains("Mono8"))
        {
            sFormat = "Mono8";
            ColorComponentList << statistics.at(0).at(0);
            MinimumValueList << statistics.at(0).at(1);
            MaximumValueList << statistics.at(0).at(2);
            AverageValueList << statistics.at(0).at(3);
            m_HistogramWindow->setStatistic(ColorComponentList, MinimumValueList, MaximumValueList, AverageValueList, sFormat);
        }

        if(sHistogramTitle.contains("RGB8") || sHistogramTitle.contains("BGR8") || sHistogramTitle.contains("YUV") || sHistogramTitle.contains("Bayer"))
        {
            if(sHistogramTitle.contains("RGB8") || sHistogramTitle.contains("BGR8"))
                sFormat = "RGB";
            if(sHistogramTitle.contains("Bayer"))
                sFormat = "Bayer";
            if(sHistogramTitle.contains("YUV"))
                sFormat = "YUV";

            ColorComponentList << statistics.at(0).at(0) << statistics.at(1).at(0) << statistics.at(2).at(0);
            MinimumValueList << statistics.at(0).at(1) << statistics.at(1).at(1) << statistics.at(2).at(1);
            MaximumValueList << statistics.at(0).at(2) << statistics.at(1).at(2) << statistics.at(2).at(2);
            AverageValueList << statistics.at(0).at(3) << statistics.at(1).at(3) << statistics.at(2).at(3);
            m_HistogramWindow->setStatistic(ColorComponentList, MinimumValueList, MaximumValueList, AverageValueList, sFormat);
        }

        m_HistogramWindow->updateHistogram(histData, sHistogramTitle, nMaxHeight_YAxis, nMaxWidth_XAxis);
    }
    else
         m_pFrameObs->enableHistogram(false);
}

/* display frames on viewer */
void ViewerWindow::onimageReady ( QImage image, const QString &sFormat, const QString &sHeight, const QString &sWidth )
{    
    m_FormatLabel->setText("Pixel Format: " + sFormat + " ");
    m_ImageSizeLabel->setText("Size H: " + sHeight + " ,W: "+ sWidth + " ");

    if(m_bHasJustStarted)
    {
        foreach( QGraphicsItem *item, m_pScene->items() )
        {
            if( item->type() == QGraphicsTextItem::Type )
            {
                 m_pScene->removeItem(m_TextItem);
                 m_FormatLabel->setStyleSheet("background-color: rgb(195,195,195); color: rgb(0,0,0)");
                 m_bIsRedHighlighted = false;
                 continue;
            }

            m_pScene->removeItem(m_PixmapItem);
        }

        m_pScene->addItem(m_PixmapItem);    
        m_bHasJustStarted = false;
    }

    if( (sFormat.contains("Convert Error")) && ((sFormat.contains("Height") || sFormat.contains("Width"))) )
    {
        if( false == m_bIsRedHighlighted )
        {
            m_bIsRedHighlighted = true;
            m_FormatLabel->setStyleSheet("background-color: rgb(196,0, 0); color: rgb(255,255,255)");
            
            if(sFormat.contains("Height"))
            {
                m_TextItem->setPlainText("The Resolution you set is not supported by AVTImageTransform.\n"
                                         "Please change the Height !");
            }
            else if(sFormat.contains("Width"))
            {
                m_TextItem->setPlainText("The Resolution you set is not supported by AVTImageTransform.\n"
                                         "Please change the Width !");
            }
            
            m_TextItem->setPos(m_pScene->width()/6, m_pScene->height()/2);
            m_pScene->addItem(m_TextItem);        
        }
    }
    else
    {
        if( m_bIsRedHighlighted )
        {
            m_FormatLabel->setStyleSheet("background-color: rgb(195,195,195); color: rgb(0,0,0)");
            m_bIsRedHighlighted = false;

            if (!m_pScene->items().isEmpty())
            {
                m_pScene->removeItem(m_TextItem);
            }
        }
    }

    /* display it at centre whenever changed */
    m_pScene->setSceneRect(0, 0, image.width(), image.height() );
    m_PixmapItem->setPixmap(QPixmap::fromImage(image));
    m_ScreenViewer->show();

    /* saving images */
    if( (0 < m_nNumberOfFramesToSave) && m_bIsTriggeredByMultiSaveBtn )
    {
        ++m_nImageCounter;

        if(m_nImageCounter <= m_nNumberOfFramesToSave)
            m_SaveImageThread->m_Image.append(image);

        m_SaveImageThread->m_sImagePath = m_ImagePath;
        m_SaveImageThread->m_sSaveFormat = m_SaveFormat;
        m_SaveImageThread->m_sSaveName = m_SaveName;
        m_SaveImageThread->m_nTImageCounter = m_nImageCounter ;
        m_SaveImageThread->m_nTNumber2Save = m_nNumberOfFramesToSave;

        if(IMAGES_COUNT == m_nImageCounter)
            m_SaveImageThread->start();
        
        if( m_nImageCounter == m_nNumberOfFramesToSave)
        {
            if(!m_SaveImageThread->isRunning() && (!m_SaveImageThread->m_Image.isEmpty()))
               m_SaveImageThread->start();

            return;
        }
    }
}

void SaveImageThread::run ( void )
{
   while(m_nImagePosition != m_nTNumber2Save)
   {
        while (!m_Image.isEmpty())
        {       
            emit setPosition (++m_nImagePosition);
            QString sFullPath = m_sImagePath;
            sFullPath.append("//").append(m_sSaveName);
            sFullPath.append("_"+QString::number(m_nImagePosition)).append(m_sSaveFormat);
            m_Image.first().save(sFullPath);
            m_Image.pop_front();
        }   
   }
 
   exec();
}

void ViewerWindow:: onSaving ( unsigned int nPos )
{ 
    if( 1 == nPos )
    {
        //Start Progressbar
        m_SaveImagesDialog = new QDialog(0, Qt::CustomizeWindowHint|Qt::WindowTitleHint);
        m_SaveImagesUIDialog.setupUi(m_SaveImagesDialog);
        m_SaveImagesUIDialog.saveImagesProgress->setMaximum(m_nNumberOfFramesToSave);
        m_SaveImagesUIDialog.saveImagesProgress->setMinimum(1);
        m_SaveImagesDialog->show();
    }

    m_SaveImagesUIDialog.saveImagesProgress->setValue(nPos);

    if (m_nNumberOfFramesToSave == nPos)
    {
        m_FormatLabel->setStyleSheet("background-color: rgb(195,195,195); color: rgb(0,0,0)");
        m_bIsRedHighlighted = false;

        ActionFreerun->setChecked(false);
        on_ActionFreerun_triggered();
        ActionFreerun->setEnabled(true);
        
        foreach( QGraphicsItem *item, m_pScene->items() )
        {
            if( item->type() == QGraphicsTextItem::Type )
            {
                m_pScene->removeItem(m_TextItem);
                break;
            }
        }

        m_SaveImagesDialog->hide();
        delete m_SaveImagesDialog;
        m_SaveImagesDialog = NULL;
        m_SaveImageThread->quit();
        m_nImageCounter = 0;
        m_SaveImageThread->m_nImagePosition = 0;
        m_bIsTriggeredByMultiSaveBtn = false;
    }

    QCoreApplication::processEvents();
}

void ViewerWindow::onAcquisitionStartStop ( const QString &sThisFeature )
{
    QIcon icon;

    /* this is intended to stop and start the camera again since PixelFormat, Height and Width have been changed while camera running
    *  ignore this when the changing has been made while camera not running 
    */
    if( ((0 == sThisFeature.compare("AcquisitionStart")) && (m_bIsCameraRunning)) )
    {
        ActionFreerun->setChecked(true);
        on_ActionFreerun_triggered();
    }
    else if( sThisFeature.contains("AcquisitionStartFreerun") )
    {
        SP_ACCESS( m_pFrameObs )->resetFrameCounter(true);
        if(!m_bIsCameraRunning)
        {
            icon.addFile(QString::fromUtf8(":/VimbaViewer/Images/stop.png"), QSize(), QIcon::Normal, QIcon::On);
            ActionFreerun->setIcon(icon);
            checkDisplayInterval();
            releaseBuffer();
            onPrepareCapture();
            ActionFreerun->setChecked(true);
            m_bIsCameraRunning = true;
            m_bHasJustStarted = true;

            if(ActionDisplayOptions->isEnabled())
                ActionDisplayOptions->setEnabled(false);

            if(ActionSaveOptions->isEnabled())
                ActionSaveOptions->setEnabled(false);

            /* if save images settings set, and acquisition starts */
            if( (0 < m_SaveImageOption.NumberOfFrames_SpinBox->value()) && ActionSaveImages->isEnabled() )
            {
                ActionSaveImages->setEnabled(false);
                m_nImageCounter = 0;
                m_nNumberOfFramesToSave = 0;
            }

            m_OperatingStatusLabel->setText( " Running... " );
            m_OperatingStatusLabel->setStyleSheet("background-color: rgb(0,128, 0); color: rgb(255,255,255)");
        }
    }
    else if( sThisFeature.contains("AcquisitionStopFreerun") )
    {
        if(m_bIsCameraRunning)
        {
            icon.addFile(QString::fromUtf8(":/VimbaViewer/Images/play.png"), QSize(), QIcon::Normal, QIcon::Off);
            ActionFreerun->setIcon(icon);
            releaseBuffer();
            ActionFreerun->setChecked(false);
            m_bIsCameraRunning = false;

            if(!ActionSaveOptions->isEnabled())
                ActionSaveOptions->setEnabled(true);

            if(!ActionFreerun->isEnabled())
                ActionFreerun->setEnabled(true);

            if(!ActionDisplayOptions->isEnabled())
                ActionDisplayOptions->setEnabled(true);

            /* if save images running, and acquisition stops */
            if( (0 < m_SaveImageOption.NumberOfFrames_SpinBox->value()) && !ActionSaveImages->isEnabled() )
            {
                ActionSaveImages->setEnabled(true);
            }

            m_Controller->synchronizeEventFeatures();
        }

        m_OperatingStatusLabel->setText( " Ready " );
        m_OperatingStatusLabel->setStyleSheet("background-color: rgb(0,0, 0); color: rgb(255,255,255)"); 
    }
    else if( ((0 == sThisFeature.compare("AcquisitionStop")) && (m_bIsCameraRunning)) ||
              (sThisFeature.contains("AcquisitionStopWidthHeight")))
    {
        if(m_bIsCameraRunning)
        {
            ActionFreerun->setChecked(false);
            on_ActionFreerun_triggered();
        
            /* use this for GigE, so you can change the W/H "on the fly" */
            if(0 == sThisFeature.compare("AcquisitionStopWidthHeight"))
                m_bIsCameraRunning = true;
        }
    }
}

void ViewerWindow::checkDisplayInterval( void )
{
    FeaturePtr pFeatMode;

    if(VmbErrorSuccess == m_pCam->GetFeatureByName( "AcquisitionMode", pFeatMode ))
    {
        std::string sValue("");
        if( VmbErrorSuccess == pFeatMode->GetValue(sValue) )
        {
            /* display all received frames for SingleFrame and MultiFrame mode or if the user wants to have it */
            if( 0 == sValue.compare("SingleFrame") || 0 == sValue.compare("MultiFrame") || m_bIsDisplayEveryFrame )
                SP_ACCESS( m_pFrameObs )->setDisplayInterval( 0 ); 
            /* display frame in a certain interval to save CPU consumption for continuous mode */
            else
                SP_ACCESS( m_pFrameObs )->setDisplayInterval( 1 ); 
        }
    }
}

void ViewerWindow::on_ActionFreerun_triggered( void )
{
    VmbError_t error;
    FeaturePtr pFeat;
    
    checkDisplayInterval();

     /* update interpolation state after start */
    if( !m_Timer->isActive())
    {
        m_Timer->start(200);
    }

    QIcon icon;
    /* ON */
    if( ActionFreerun->isChecked() )
    {
        icon.addFile(QString::fromUtf8(":/VimbaViewer/Images/stop.png"), QSize(), QIcon::Normal, QIcon::On);
        ActionFreerun->setIcon(icon);
    
        error = onPrepareCapture();
        if( VmbErrorSuccess != error )
        {
            m_bIsCameraRunning = false;
            m_OperatingStatusLabel->setText( " Failed to start! Error: " + QString::number(error)+" "+m_Helper.mapReturnCodeToString(error) );
            m_OperatingStatusLabel->setStyleSheet("background-color: rgb(196,0, 0); color: rgb(255,255,255)"); 
            icon.addFile(QString::fromUtf8(":/VimbaViewer/Images/play.png"), QSize(), QIcon::Normal, QIcon::Off);
            ActionFreerun->setIcon(icon);
            ActionFreerun->setChecked(false);
            return;
        }

        error = m_pCam->GetFeatureByName( "AcquisitionStart", pFeat );
        int nResult = m_sAccessMode.compare(" (READ ONLY)") ;
        if ( (VmbErrorSuccess == error) && ( 0 != nResult ) ) 
        {
            SP_ACCESS( m_pFrameObs )->resetFrameCounter(true);
            error = pFeat->RunCommand();
 
            if(VmbErrorSuccess == error)
            {
                if(m_bIsFirstStart)
                {  
                    m_bIsFirstStart = false;
                }

                m_bHasJustStarted = true;
                m_bIsCameraRunning = true;
                m_OperatingStatusLabel->setText( " Running... " );
                m_OperatingStatusLabel->setStyleSheet("background-color: rgb(0,128, 0); color: rgb(255,255,255)");

                if(ActionDisplayOptions->isEnabled())
                    ActionDisplayOptions->setEnabled(false);

                if(ActionSaveOptions->isEnabled())
                    ActionSaveOptions->setEnabled(false);

                if( ActionSaveImages->isEnabled() && (0 < m_SaveImageOption.NumberOfFrames_SpinBox->value()) )
                    ActionSaveImages->setEnabled(false);
            }
            else
            {
                m_bIsCameraRunning = false;
                m_OperatingStatusLabel->setText( " Failed to execute AcquisitionStart! Error: " + QString::number(error)+" "+m_Helper.mapReturnCodeToString(error) );
                m_OperatingStatusLabel->setStyleSheet("background-color: rgb(196,0, 0); color: rgb(255,255,255)"); 

                m_InformationWindow->feedLogger("Logging", QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + 
                                                " RunCommand [AcquisitionStart] Failed! Error: " + QString::number(error)+" "+
                                                m_Helper.mapReturnCodeToString(error)), VimbaViewerLogCategory_ERROR);
            }
        }
    }
    /* OFF */
    else
    {
        error = m_pCam->GetFeatureByName("AcquisitionStop", pFeat);
        if ( (VmbErrorSuccess == error) )
        {
            icon.addFile(QString::fromUtf8(":/VimbaViewer/Images/play.png"), QSize(), QIcon::Normal, QIcon::Off);
            ActionFreerun->setIcon(icon);
            
            if(0 != m_sAccessMode.compare(" (READ ONLY)"))
                error = pFeat->RunCommand();
            
            if(VmbErrorSuccess == error)
            {  
                releaseBuffer();
                m_bIsCameraRunning = false;
                m_OperatingStatusLabel->setText( " Ready " );
                m_OperatingStatusLabel->setStyleSheet("background-color: rgb(0,0, 0); color: rgb(255,255,255)"); 
            }
            else
            {
                m_InformationWindow->feedLogger("Logging", QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + 
                                                " RunCommand [AcquisitionStop] Failed! Error: " + QString::number(error) + " " +
                                                m_Helper.mapReturnCodeToString(error) ), VimbaViewerLogCategory_ERROR);
            }
        }


        if(!ActionDisplayOptions->isEnabled())
            ActionDisplayOptions->setEnabled(true);

        if(!ActionSaveOptions->isEnabled())
            ActionSaveOptions->setEnabled(true);

        if( !ActionSaveImages->isEnabled() && (0 < m_SaveImageOption.NumberOfFrames_SpinBox->value()))
            ActionSaveImages->setEnabled(true);

        m_Controller->synchronizeEventFeatures();
    }
}

VmbError_t ViewerWindow::releaseBuffer ( void )
{
    VmbError_t error = m_pCam->EndCapture();
    if( VmbErrorSuccess == error )
        error = m_pCam->FlushQueue();
    if( VmbErrorSuccess == error ) 
        error = m_pCam->RevokeAllFrames();
 
    return error;
}

VmbError_t ViewerWindow::onPrepareCapture ( void )
{
    FeaturePtr pFeature;
    VmbInt64_t nPayload = 0;
    QVector <FramePtr> frames;
    VmbError_t error = m_pCam->GetFeatureByName("PayloadSize", pFeature);
    VmbUint32_t nCounter = 0;
    if( VmbErrorSuccess == error )
    {
        error = pFeature->GetValue(nPayload);
        if(VmbErrorSuccess == error)
        {
            frames.resize(BUFFER_COUNT);
        
            for (unsigned int i=0; i<frames.size(); i++)
            {
                try
                {
                    frames[i] = FramePtr(new Frame(nPayload));
                    nCounter++;
                }
                catch(std::bad_alloc& ba)
                {
                     frames.resize((VmbInt64_t) (nCounter * 0.7));
                     break;
                }

                error = frames[i]->RegisterObserver(m_pFrameObs);
                if( VmbErrorSuccess != error )
                {
                    m_InformationWindow->feedLogger("Logging", 
                                                    QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + " RegisterObserver frame["+ QString::number(i)+ "] Failed! Error: " + QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error)), 
                                                    VimbaViewerLogCategory_ERROR);
                    return error;
                }
            }

            for (size_t    i=0; i<frames.size(); i++)
            {
                error = m_pCam->AnnounceFrame( frames[i] );
                if( VmbErrorSuccess != error )
                {
                    m_InformationWindow->feedLogger("Logging", 
                                                    QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + " AnnounceFrame ["+ QString::number(i)+ "] Failed! Error: " + QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error)), 
                                                    VimbaViewerLogCategory_ERROR);
                    return error;
                }
            }
            
            if(VmbErrorSuccess == error)
            {
                error = m_pCam->StartCapture();
                if( VmbErrorSuccess != error )
                {
                    QString sMessage = " StartCapture Failed! Error: ";
                    
                    if(0 != m_sAccessMode.compare(" (READ ONLY)"))
                        m_InformationWindow->feedLogger("Logging", 
                                                         QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + sMessage + QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error)), 
                                                         VimbaViewerLogCategory_ERROR);
                    return error;
                }
            }

            for (size_t    i=0; i<frames.size(); i++)
            {
                error = m_pCam->QueueFrame( frames[i] );
                if( VmbErrorSuccess != error )
                {
                    m_InformationWindow->feedLogger("Logging", 
                                                    QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + " QueueFrame ["+ QString::number(i)+ "] Failed! Error: " + QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error)), 
                                                    VimbaViewerLogCategory_ERROR);
                    return error;
                }
            }
        }
        else
        {
            m_InformationWindow->feedLogger("Logging", 
                                            QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + " GetValue [PayloadSize] Failed! Error: " + QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error)), 
                                            VimbaViewerLogCategory_ERROR);
            return error;
        }
    }
    else
    {
        m_InformationWindow->feedLogger("Logging", 
                                        QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + " GetFeatureByName [PayloadSize] Failed! Error: " + QString::number(error)+" "+ m_Helper.mapReturnCodeToString(error)), 
                                        VimbaViewerLogCategory_ERROR);
        return error;
    }

    return error;
}

void ViewerWindow::onFeedLogger    ( const QString &sMessage )
{
    m_InformationWindow->feedLogger("Logging", QString(QTime::currentTime().toString("hh:mm:ss:zzz")+"\t" + sMessage), VimbaViewerLogCategory_ERROR);
}

void ViewerWindow::changeEvent ( QEvent * event )
{
    if( event->type() == QEvent::WindowStateChange )
    {
        if( isMinimized() )
            m_Controller->showControl(false);
        else if( isMaximized() )
            m_Controller->showControl(true);
    }
}