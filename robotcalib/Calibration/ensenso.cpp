#include "calibration.hpp"
#include "calib_en/nxLib.h"
void get_en_image()
{
    try {
		// Initialize NxLib and enumerate cameras
		nxLibInitialize(true);

		// Reference to the first camera in the node BySerialNo
		NxLibItem root;
		NxLibItem camera = root[itmCameras][itmBySerialNo][0];

		// Open the Ensenso
		NxLibCommand open(cmdOpen);
		open.parameters()[itmCameras] = camera[itmSerialNumber].asString();
		open.execute();

		// Capture an image
		NxLibCommand (cmdCapture).execute();

		NxLibCommand (cmdRectifyImages).execute();

		// Save images
		NxLibCommand saveImage(cmdSaveImage);
		//   raw left
		saveImage.parameters()[itmNode] = camera[itmImages][itmRaw][itmLeft].path;
		saveImage.parameters()[itmFilename] = "raw_left.png";
		saveImage.execute();
		//   raw right
		saveImage.parameters()[itmNode] = camera[itmImages][itmRaw][itmRight].path;
		saveImage.parameters()[itmFilename] = "raw_right.png";
		saveImage.execute();
		//   rectified left
		saveImage.parameters()[itmNode] = camera[itmImages][itmRectified][itmLeft].path;
		saveImage.parameters()[itmFilename] = "rectified_left.png";
		saveImage.execute();
		//   rectified right
		saveImage.parameters()[itmNode] = camera[itmImages][itmRectified][itmRight].path;
		saveImage.parameters()[itmFilename] = "rectified_right.png";
		saveImage.execute();
	} catch (NxLibException& e) { // Display NxLib API exceptions, if any
		printf("An NxLib API error with code %d (%s) occurred while accessing item %s.\n", e.getErrorCode(), e.getErrorText().c_str(), e.getItemPath().c_str());
		if (e.getErrorCode() == NxLibExecutionFailed) printf("/Execute:\n%s\n", NxLibItem(itmExecute).asJson(true).c_str());
	} catch (...) { // Display other exceptions
		printf("Something, somewhere went terribly wrong!\n");
	}
}

