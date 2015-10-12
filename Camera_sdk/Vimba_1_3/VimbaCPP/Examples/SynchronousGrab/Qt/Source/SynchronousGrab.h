/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        SynchronousGrab.cpp

  Description: Qt dialog class for the GUI of the SynchronousGrab example of
               VimbaCPP.

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

#ifndef SYNCHRONOUSGRABQT_H
#define SYNCHRONOUSGRABQT_H

#include <QtGui/QMainWindow>

#include "ui_SynchronousGrab.h"

#include "ApiController.h"

using namespace AVT::VmbAPI;
using namespace AVT::VmbAPI::Examples;

class SynchronousGrab : public QMainWindow
{
    Q_OBJECT

public:
    SynchronousGrab(QWidget *parent = 0, Qt::WFlags flags = 0);
    ~SynchronousGrab();

private:
    Ui::SynchronousGrabClass    ui;
    // Our controller that wraps API access
    ApiController *             m_pApiController;
    // A list of known camera IDs
    std::vector<std::string>    m_cameras;
    // Are we streaming?
    bool                        m_bIsStreaming;
    // Our Qt image to display
    QImage *                    m_pImage;

    // Queries and lists all known camera
    void UpdateCameraListBox();
    // Prints out some logging
    void Log( std::string strMsg, VmbErrorType eErr );
    // Copies the content of a byte buffer to a MFC image
    void CopyToImage( VmbUchar_t *pInBuffer, QImage *pOutImage );

private slots:
    void OnBnClickedButtonStartstop();
    // The event handler for the camera list changed event
    void OnCameraListChanged( int reason );    
};

#endif // SYNCHRONOUSGRABQT_H
