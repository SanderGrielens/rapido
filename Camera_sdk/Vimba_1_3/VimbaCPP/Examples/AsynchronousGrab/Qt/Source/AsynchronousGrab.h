/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        AsynchronousGrab.h

  Description: Qt dialog class for the GUI of the AsynchronousGrab example of
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

#ifndef ASYNCHRONOUSGRABQT_H
#define ASYNCHRONOUSGRABQT_H

#include <QtGui/QMainWindow>
#include "ui_AsynchronousGrab.h"

#include <ApiController.h>

using namespace AVT::VmbAPI;
using namespace AVT::VmbAPI::Examples;

class AsynchronousGrab : public QMainWindow
{
    Q_OBJECT

public:
    AsynchronousGrab( QWidget *parent = 0, Qt::WFlags flags = 0 );
    ~AsynchronousGrab();

private:
    // The Qt GUI
    Ui::AsynchronousGrabClass ui;
    // Our controller that wraps API access
    ApiController m_ApiController;
    // A list of known camera IDs
    std::vector<std::string> m_cameras;
    // Are we streaming?
    bool m_bIsStreaming;
    // Our Qt image to display
    QImage *m_pImage;

    // Queries and lists all known cameras
    void UpdateCameraListBox();
    // Prints out some logging
    void Log( std::string strMsg, VmbErrorType eErr );
    // Copies the content of a byte buffer to a MFC image
    VmbErrorType CopyToImage( VmbUchar_t *pInBuffer, VmbPixelFormat_t ePixelFormat, QImage &pOutImage, const float *Matrix = NULL );

private slots:
    // The event handler for starting / stopping acquisition
    void OnBnClickedButtonStartstop();
    // The event handler for the frame received event
    void OnFrameReady( int status );
    // The event handler for the camera list changed event
    void OnCameraListChanged( int reason );
};

#endif // ASYNCHRONOUSGRABQT_H
