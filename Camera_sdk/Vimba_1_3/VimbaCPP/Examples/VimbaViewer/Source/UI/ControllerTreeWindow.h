/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ControllerTreeWindow.h

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

#ifndef CONTROLLERTREEWINDOW_H
#define CONTROLLERTREEWINDOW_H


#include <QTreeView>
#include <QGridLayout>
#include <QDockWidget>
#include <QStandardItemModel>
#include <QItemDelegate>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QTextEdit>
#include <QLineEdit>
#include <QLabel>
#include "MultiCompleter.h"
#include "Helper.h"
#include "IntSpinBox.h"
#include "FloatSpinBox.h"
#include "ExComboBox.h"
#include "SplashScreen.h"
#include "SortFilterProxyModel.h"
#include "HexEditor/HexMainWindow.h"
#include <VimbaCPP/Include/VimbaSystem.h>
#include <VimbaCPP/Include/IFeatureObserver.h>

#include "../ExternLib/qwt/qwt_slider.h"
#include "../ExternLib/qwt/qwt_scale_engine.h"
#include "../ExternLib/qwt/qwt_scale_map.h"

using namespace AVT::VmbAPI;

/* controlling row/height size in tree */
class ItemDelegate : public QItemDelegate
{
public:
    ItemDelegate(){}
    QSize sizeHint ( const QStyleOptionViewItem & option, const QModelIndex & index ) const
    {
        QSize s = QItemDelegate::sizeHint(option, index);
        s.setHeight(row_height);
        return s;
    }

    void setRowHeight(const unsigned char height)
    {
        row_height = height;
    }

protected:
    unsigned char row_height;
};


class ControllerTreeWindow : public QTreeView
{
    Q_OBJECT

    public:
                /* Filter Search Proxy */
                SortFilterProxyModel       *m_ProxyModel;        
    protected:

    private:    
                FeaturePtrVector                    m_featPtrVec;
                CameraPtr                           m_pCam;
                MultiCompleter                     *m_StringCompleter;
                ItemDelegate                       *m_TreeDelegate;
                IFeatureObserverPtr                 m_pFeatureObs;
                QString                             m_sCameraID;
                VmbError_t                          m_ListTreeError;

                QMap <QString, QString>             m_DescriptonMap;
                QMap <QString, QString>             m_DisplayFeatureNameMap;
                QMap <QString, FeaturePtr>          m_featPtrMap;
                QMap <QString, double>              m_FloatFeatureIncrementMap;

                QVector < QMap <QString, QString> > m_Level;
                QMap <QString, QString>             m_Level0Map;
                QMap <QString, QString>             m_Level1Map;
                QMap <QString, QString>             m_Level2Map;
                QMap <QString, QString>             m_Level3Map;
                QMap <QString, QString>             m_Level4Map;
                QMap <QString, QString>             m_Level5Map;
                QMap <QString, QString>             m_Level6Map;
                QMap <QString, QString>             m_Level7Map;
                QMap <QString, QString>             m_Level8Map;
                QMap <QString, QString>             m_Level9Map;
           
                FeaturePtr                          m_FeaturePtr_Command;
                FeaturePtr                          m_FeaturePtr_EnumComboBox;
                FeaturePtr                          m_FeaturePtr_CheckBox;
                FeaturePtr                          m_FeaturePtr_IntSpinBox;
                FeaturePtr                          m_FeaturePtr_FloatSliderSpinBox;
                FeaturePtr                          m_FeaturePtr_StringEditBox;
                FeaturePtr                          m_FeaturePtr_LineEdit;

                QString                             m_sFeature_Command;
                QString                             m_sFeature_EnumComboBox;
                QString                             m_sFeature_CheckBox;
                QString                             m_sFeature_IntSpinBox;
                QString                             m_sFeature_FloatSliderSpinBox;
                QString                             m_sFeature_StringEditBox;
                QString                             m_sFeature_StringLineEdit;
                QString                             m_sCurrentSelectedFeature;

                QStandardItemModel                 *m_Model;
                QStandardItemModel                 *m_ModelGuru;

                unsigned int                        m_nIntSliderOldValue;
                unsigned int                        m_nSliderStep;
                double                              m_dMinimum;
                double                              m_dMaximum;
                
                Helper                              m_Helper;

                HexMainWindow                      *m_HexWindow;
                std::vector <VmbUchar_t>            m_RawData;

                /* Logarithmic Slider */
                QWidget                            *m_LogSliderWidget;
                QwtSlider                          *m_LogSlider;
                QHBoxLayout                        *m_HLogSliderLayout;

                /* Button */
                QPushButton                        *m_CmdButton;
                QWidget                            *m_ButtonWidget;
                QHBoxLayout                        *m_HButtonLayout;
                QHBoxLayout                        *m_HButtonLayout2;

                /* ComboBox */
                ExComboBox                         *m_EnumComboBox;
                QWidget                            *m_ComboWidget;
                QHBoxLayout                        *m_HComboLayout;
                QHBoxLayout                        *m_HComboLayout2;

                /* Integer Feature Slider-SpinBoxes */
                QWidget                            *m_IntSpinSliderWidget;
                QHBoxLayout                        *m_HSpinSliderLayout_Int;
                QHBoxLayout                        *m_HSpinSliderLayout_Int2;
                IntSpinBox                         *m_SpinBox_Int;
                QSlider                            *m_Slider_Int;

                /* Float Feature Slider-SpinBoxes */
                QWidget                            *m_FloatSpinSliderWidget;
                QHBoxLayout                        *m_HSpinSliderLayout_Float;
                QHBoxLayout                        *m_HSpinSliderLayout_Float2;
                FloatSpinBox                       *m_SpinBox_Float;
                QSlider                            *m_Slider_Float;

                /* String Feature EditBox */
                QLineEdit                          *m_TextEdit_String;
                QWidget                            *m_EditWidget;
                QHBoxLayout                        *m_HEditLayout;
                QHBoxLayout                        *m_HEditLayout2;

                /* Boolean Feature CheckBox */
                QCheckBox                          *m_CheckBox_Bool;
                QWidget                            *m_BooleanWidget;
                QHBoxLayout                        *m_HBooleanLayout;
                QHBoxLayout                        *m_HBooleanLayout2;

                /* IP Address or Direct Access */
                QLineEdit                          *m_LineEdit;
                QPushButton                        *m_ConfirmButton;
                QLabel                             *m_HexLabel;
                QWidget                            *m_LineEditWidget;
                QHBoxLayout                        *m_HLineEditLayout;
                QHBoxLayout                        *m_HLineEditLayout2;
            
                bool                                m_bIsTooltipOn;
                QString                             m_sTooltip;
                
                bool                                m_bIsJobDone;
                bool                                m_bIsBusy;
                bool                                m_bIsMousePressed;

                /* GigE Auto adjust PacketSize */
                bool                                m_bIsGigE;
                bool                                m_bAutoAdjustPacketSize;
                bool                                m_bIsTimeout;

                QTimer                             *m_FeaturesPollingTimer;
                QList <QString>                     m_FeaturesPollingName;            

    public:
                ControllerTreeWindow ( QString sID = " ", QWidget *parent = 0, bool bAutoAdjustPacketSize = false, CameraPtr pCam = CameraPtr() );
                ~ControllerTreeWindow ( void );

                void setupTree( void );
                void synchronizeEventFeatures   ( void );
                VmbError_t getTreeStatus        ( void );
                void showTooltip ( const bool &bIsChecked );
                void showControl( const bool &bIsShow );
                void updateRegisterFeature ( void );
                void updateUnRegisterFeature ( void );
                bool isGigE (void);
                MultiCompleter *getListCompleter( void );
                void   saveFeaturesToTextFile ( const QString &sDestPathAndFileName );
                
    protected:
        
    private:    
                void mapInformation(const QString sName, const FeaturePtr &featPtr);
                QString getFeatureNameFromMap ( const QString &sDisplayName );
                bool    findCategory ( QMap <QString, QString> map, const QString sName);
                unsigned int getGrandParentLevel( QList<QStandardItem *> items, const QString &sGrandParent);

                bool    isEventFeature        (const QString &sFeature);
                QString getFeatureValue       ( const FeaturePtr &featPtr );
                QString getFeatureName        ( const QModelIndex item );

                void updateExpandedTreeValue   ( const FeaturePtr &featPtr, const QString &sName );

                bool registerFeatureObserver   ( const QString &sFeatureName );
                void unregisterFeatureObserver ( const QString &sFeatureName );

                void resetControl              ( void );
                void sortCategoryAndAttribute ( const FeaturePtr &featPtr , QStandardItemModel *Model );

                QString getFeatureInformation (const FeaturePtr &featPtr);
                void createCommandButton      ( const QModelIndex item );  /*  maps to VmbFeatureDataCommand */
                void createEnumComboBox       ( const QModelIndex item );  /*  maps to VmbFeatureDataEnum */
                void createIntSliderSpinBox   ( const QModelIndex item );  /*  maps to VmbFeatureDataInt */
                void createFloatSliderSpinBox ( const QModelIndex item );  /*  maps to VmbFeatureDataFloat */
                void createBooleanCheckBox    ( const QModelIndex item );  /*  maps to VmbFeatureDataBool */
                void createStringEditBox      ( const QModelIndex item );  /*  maps to VmbFeatureDataString */
                void createHexEditor          ( const QModelIndex item );  /*  maps to VmbFeatureDataRaw*/
                void createLineEdit              ( const QModelIndex &item ); /*  maps to register stuffs*/
                void createLogarithmicSLider  ( const QModelIndex &item ); /*  maps to Exposure*/
                void showIt                   ( const QModelIndex item, const QString &sWhat );

                bool isFeatureWritable        ( const QString &sFeature );
                FeaturePtr getFeaturePtr      ( const QString &sFeature );

                void setIntegerValue          ( const int &nValue );
                void updateCurrentIntValue    ( void );
                void setFloatingValue         ( const double &dValue );
                void setLogarithmicFloatingValue ( const double &dValue );

                void   countFloatingIncrement   ( const FeaturePtr &featPtr );
                double getFloatFeatureIncrement ( const QString &sFeature );

                bool   eventFilter             (QObject *object, QEvent *event);

                void   onFloatSliderReleased   ( void );
                void   onIntSliderReleased     ( void );
                void   enableWidget            ( const bool bIsWritable );
                void   onLogarithmicSliderReleased( void );

    protected slots:
                    virtual void currentChanged   ( const QModelIndex & current, const QModelIndex & previous );
                    void setAdjustPacketSizeTimeout ( void );
                
    private Q_SLOTS:
                    void setLogarithmicSliderValue ( double v );
                    
    private slots:
                void onCmdButtonClick        ( void );
                void onConfirmClick          ( void );
                void onIntSpinBoxClick       ( void );
                void onFloatSpinBoxClick     ( void );
                void onIntSliderChanged      ( int nValue );
                void onFloatSliderChanged    ( int nValue );
                void onBoolCheckBoxClick     ( bool bValue );
                void onSetChangedFeature     ( const QString &sFeature, const QString &sValue, const bool &bIsWritable );
                void onSetEventMsg           ( const QStringList &sMsg );
                void onEnumComboBoxClick     ( const QString &sSelected );
                void expand                     ( const QModelIndex & index );
                void collapse                 ( const QModelIndex & index );
                void onLogarithmicFloatSpinBoxClick  ( void );
                void onEditText              ( void );
                void pollingFeaturesValue    ( void );

    signals:
                void setDescription           ( const QString &sDesc );
                void acquisitionStartStop     ( const QString &sThisFeature );
                void setEventMessage          ( const QStringList &sMsg );
                void logging                  ( const QString &sMessage );
                void updateBufferSize         ( void );
                void resetFPS                  ( void );                
                void enableViewerMenu         ( bool bIsEnabled);
};

#endif