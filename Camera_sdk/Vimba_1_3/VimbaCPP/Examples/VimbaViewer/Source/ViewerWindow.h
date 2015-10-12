/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ViewerWindow.h

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


#ifndef VIEWERWINDOW_H
#define VIEWERWINDOW_H

#include <QtGui/QtGui>
#include <QQueue>
#include <QTimer>
#include "ui_ViewerWindow.h"
#include "ui_SaveImageOption.h"
#include "ui_DirectAccess.h"
#include "ui_ViewerOption.h"
#include "ui_SavingProgress.h"
#include "LineEditCompleter.h"
#include "Helper.h"
#include "UI/DockWidgetWindow.h"    
#include "UI/ControllerTreeWindow.h"
#include "UI/Viewer.h"
#include "UI/MainInformationWindow.h"
#include "UI/Histogram/HistogramWindow.h"
#include "FrameObserver.h"
#include "ImageSaver.h"
#include <VimbaCPP/Include/VimbaSystem.h>

using namespace AVT::VmbAPI;

class SaveImageThread : public QThread
{
    Q_OBJECT

    public:
              unsigned int      m_nTImageCounter;
              unsigned int      m_nTNumber2Save;
              unsigned int      m_nImagePosition;
              
              QString           m_sImagePath;
              QString           m_sSaveFormat;
              QString           m_sSaveName;
              QVector <QImage>  m_Image;
    public:
              SaveImageThread ( ){};
             ~SaveImageThread ( void ){};
    
    private:
              
    protected:
              virtual void run();

    private slots:

    signals:
              void setPosition ( unsigned int nPos );
};



class ViewerWindow : public QMainWindow, private Ui::ViewerWindow
{
    Q_OBJECT
    public: 
               
    protected:
                
    private:    
                QTimer                             *m_Timer;
                QString                             m_sCameraID;
                VmbError_t                          m_OpenError;
                bool                                m_bIsCameraRunning;
                bool                                m_bHasJustStarted;
                bool                                m_bIsCamOpen;
                bool                                m_bIsFirstStart;
                bool                                m_bIsRedHighlighted;

                QString                             m_sAccessMode;
                QAction                            *m_ResetPosition;
                QCheckBox                          *m_ToolTipCheckBox;

                DockWidgetWindow                   *m_DockController;
                DockWidgetWindow                   *m_DockInformation;
                DockWidgetWindow                   *m_DockHistogram;

                Viewer                             *m_ScreenViewer;
                MainInformationWindow              *m_InformationWindow;
                ControllerTreeWindow               *m_Controller;
                HistogramWindow                    *m_HistogramWindow;

                CameraPtr                           m_pCam;
                QTextEdit                          *m_Description;
            
                QLabel                             *m_OperatingStatusLabel;
                QLabel                             *m_FormatLabel;
                QLabel                             *m_ImageSizeLabel;
                QLabel                             *m_FramerateLabel;
                QLabel                             *m_FramesLabel;

                QSharedPointer<QGraphicsScene>      m_pScene;
                QGraphicsPixmapItem                *m_PixmapItem;
                QGraphicsTextItem                  *m_TextItem; 

                SP_DECL( FrameObserver )            m_pFrameObs;
                Helper                              m_Helper;

                QString                             m_SaveFileDir;
                QString                             m_SelectedExtension;
                QString                             m_SaveImageName;
                /* Save Image Option */
                Ui::Dialog                          m_SaveImageOption;
                QDialog                            *m_ImageOptionDialog;
                QFileDialog                        *m_getDirDialog; // mulitiple images
                QFileDialog                        *m_saveFileDialog; // save an image

                /* Direct Access */
                Ui::DirectAccessDialog              m_DirectAccess;
                QDialog                            *m_AccessDialog;

                /*Viewer Option */
                Ui::DisplayOptionsDialog            m_ViewerOption;
                QDialog                            *m_ViewerOptionDialog;
                bool                                m_bIsDisplayEveryFrame;

                /* Filter Pattern */
                LineEditCompleter                  *m_FilterPatternLineEdit;

                /*  Saving images progress */
                QSharedPointer<SaveImageThread>     m_SaveImageThread;
                Ui::SaveImagesProgressDialog        m_SaveImagesUIDialog;
                QDialog                            *m_SaveImagesDialog;

                unsigned int                        m_nNumberOfFramesToSave;
                unsigned int                        m_nImageCounter;

                QString                             m_SaveName;
                QString                             m_SaveFormat;
                QString                             m_ImagePath;
                bool                                m_bIsTriggeredByMultiSaveBtn; // see where the start comes from
                
                
    public:
                ViewerWindow ( QWidget *parent = 0, Qt::WFlags flag = 0,  QString sID = " ", QString sAccess = " ", bool bAutoAdjustPacketSize = false, CameraPtr pCam = CameraPtr() );
               ~ViewerWindow ( void );

                bool        getCamOpenStatus            ( void );
                VmbError_t  getOpenError                ( void );
                QString     getCameraID                 ( void );
                bool        getAdjustPacketSizeMessage  ( QString &sMessage );
                
    protected:
                virtual void closeEvent                 ( QCloseEvent *event );                

    private:
                VmbError_t    releaseBuffer             ( void );
                void checkDisplayInterval               ( void );
                void changeEvent                        ( QEvent * event );
                bool isDestPathWritable                 ( void );
                bool checkUsedName                      ( const QStringList &files );

    private slots:

                /* when you use this std convention, you don't need any "connect..." */
                void on_ActionFreerun_triggered         ( void );
                void on_ActionResetPosition_triggered   ( void );
                void on_ActionHistogram_triggered       ( void );
                void on_ActionOriginalSize_triggered    ( void );
                void on_ActionSaveAs_triggered          ( void );
                void on_ActionSaveOptions_triggered     ( void );
                void on_ActionSaveImages_triggered      ( void );
                void on_ActionRegister_triggered        ( void );
                void on_ActionDisplayOptions_triggered  ( void );
                void on_ActionFitToWindow_triggered     ( void );
                void on_ActionLeftRotation_triggered    ( void );
                void on_ActionRightRotation_triggered   ( void );
                void on_ActionZoomOut_triggered         ( void );
                void on_ActionZoomIn_triggered          ( void );

                /* custom */
                void onAcquisitionStartStop             ( const QString &sThisFeature );
                void onfloatingDockChanged              ( bool bIsFloating );
                void onVisibilityChanged                ( bool bIsVisible );
                void displayEveryFrameClick             ( bool bValue );
                void onSetDescription                   ( const QString &sDesc );
                void onimageReady                       ( QImage image, const QString &sFormat, const QString &sHeight, const QString &sWidth );
                void onSaving                           ( unsigned int nPos );
                void onSetEventMessage                  ( const QStringList &sMsg );
                void onSetCurrentFPS                    ( const QString &sFPS );
                void onSetFrameCounter                  ( const unsigned int &nNumberOfFrames );
                void onFeedLogger                       ( const QString &sMessage );
                void onResetFPS                         ( void );
                void getSaveDestinationPath             ( void );
                void acceptSaveImagesDlg                ( void );
                void rejectSaveImagesDlg                ( void );
                void writeRegisterData                  ( void );
                void readRegisterData                   ( void );
                VmbError_t onPrepareCapture             ( void );
                void onSetColorInterpolation            ( const bool &bState );
                void onSetHistogramData                 ( const QVector<QVector <quint32> > &histData, const QString &sHistogramTitle, 
                                                          const double &nMaxHeight_YAxis, const double &nMaxWidth_XAxis, const QVector <QStringList> &statistics );
                void updateColorInterpolationState      ( void );
                void onTooltipCheckBoxClick             ( bool bValue );
                void enableMenuAndToolbar               ( bool bValue );
                void directAccessHexTextChanged         ( const QString &sText );
                void directAccessDecTextChanged         ( const QString &sText );
                void textFilterChanged                  ( void );

    signals:
                void closeViewer                        ( const QString &sID );
};

#endif