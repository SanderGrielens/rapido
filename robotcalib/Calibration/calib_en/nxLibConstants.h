#ifndef __NXLIB_CONSTANTS_H__
#define __NXLIB_CONSTANTS_H__

#ifndef __cli
	#if defined (_WIN32) || defined (__WINDOWS__)
		#define NXLIBERR  __int32
		#define NXLIBBOOL __int32
		#define NXLIBINT  __int32
		#define NXLIBDOUBLE double
	#else /* defined (_WIN32) */
		#include <stdint.h>
		#define NXLIBERR  int32_t
		#define NXLIBBOOL int32_t
		#define NXLIBINT  int32_t
		#define NXLIBDOUBLE double
	#endif

	#define NXLIBSTR const char*
	#define NXLIBFALSE 0 
	#define NXLIBTRUE  1

	#define NxLibItemSeparator '/'
	#define NxLibIndexEscapeChar '\\'
	#define NxLibItemForbiddenChars "\r\n\"/\\\0"
#endif /* __cli */

#ifdef __cli
	#undef NxLibItemSeparator
	#undef NxLibIndexEscapeChar
	#undef NxLibItemForbiddenChars
	#define NxLibItemSeparator "/"
	#define NxLibIndexEscapeChar "\\"
	#define NxLibItemForbiddenChars "\r\n\"/\\\0"

	#undef NXLIB_ITEM
	#undef NXLIB_COMMAND
	#undef NXLIB_VALUE
	#undef NXLIB_ERROR
	#undef NXLIB_INT_CONST
	#define NXLIB_ITEM(NAME)    static String^ itm ## NAME = #NAME
	#define NXLIB_COMMAND(NAME) static String^ cmd ## NAME = #NAME
	#define NXLIB_VALUE(NAME)   static String^ val ## NAME = #NAME
	#define NXLIB_ERROR(NAME)   static String^ err ## NAME = #NAME
	#define NXLIB_INT_CONST(NAME, VALUE) static int NAME = VALUE
	#ifdef NXLIB_APIERROR
		#undef NXLIB_APIERROR
	#endif
	#define NXLIB_APIERROR(NAME, VALUE) static int const NAME = VALUE

	static short const InvalidDisparityScaled = (short)0x8000;
#else
	#define NXLIB_ITEM(NAME)    static char const * const itm ## NAME = #NAME
	#define NXLIB_COMMAND(NAME) static char const * const cmd ## NAME = #NAME
	#define NXLIB_VALUE(NAME)   static char const * const val ## NAME = #NAME
	#define NXLIB_ERROR(NAME)   static char const * const err ## NAME = #NAME
	#define NXLIB_VALUE_(NAME, VALUE)     static char const * const val ## NAME = VALUE
	#define NXLIB_ERROR_(NAME, VALUE)     static char const * const err ## NAME = VALUE
	#define NXLIB_INT_CONST(NAME, VALUE) static int const NxLib ## NAME = VALUE
	#ifdef NXLIB_APIERROR
		#undef NXLIB_APIERROR
	#endif
	#define NXLIB_APIERROR(NAME, VALUE) static int const NxLib ## NAME = VALUE
	
	static short const NxLibInvalidDisparityScaled = (short)0x8000;
#endif /* __cli */

NXLIB_ITEM(Value);
NXLIB_ITEM(Protection);
NXLIB_ITEM(ExtendedType);

NXLIB_ITEM(Debug);
	NXLIB_ITEM(Destination);
	NXLIB_ITEM(Modules);
	NXLIB_ITEM(Locking);
	//NXLIB_ITEM(Camera);
	NXLIB_ITEM(StereoCamera);
	NXLIB_ITEM(Matching);
	//NXLIB_ITEM(Calibration);
	NXLIB_ITEM(Commands);
	NXLIB_ITEM(Rendering);

NXLIB_ITEM(Version);
	NXLIB_ITEM(Year);
	NXLIB_ITEM(Month);
	NXLIB_ITEM(Day);
	NXLIB_ITEM(Major);
	NXLIB_ITEM(Minor);
	NXLIB_ITEM(Build);
	NXLIB_ITEM(Hash);
	NXLIB_ITEM(UEye);

NXLIB_ITEM(Cameras);
	NXLIB_ITEM(ByEepromId);
	NXLIB_ITEM(BySerialNo);
		NXLIB_ITEM(SerialNumber);
		NXLIB_ITEM(ModelName);
		NXLIB_ITEM(EepromId);
		NXLIB_ITEM(Status);
		NXLIB_ITEM(Temperature);
		NXLIB_ITEM(Type);
		NXLIB_ITEM(Sensor);
			//NXLIB_ITEM(Size);
		NXLIB_ITEM(Port);
			NXLIB_ITEM(Bandwidth);
			//NXLIB_ITEM(Type);

NXLIB_COMMAND(Break);
NXLIB_COMMAND(Open);
	NXLIB_ITEM(LoadCalibration);
	NXLIB_ITEM(AllowFirmwareUpload);
NXLIB_COMMAND(Close);
NXLIB_COMMAND(Capture);
	NXLIB_ITEM(InitialTrigger);
	NXLIB_ITEM(FinalTrigger);
	NXLIB_ITEM(WaitFor);
	NXLIB_ITEM(Timeout);
	NXLIB_ITEM(Operation);
	NXLIB_ITEM(WaitForProjector);
NXLIB_COMMAND(RectifyImages);
NXLIB_COMMAND(ComputeDisparityMap);
	NXLIB_ITEM(MarkFilterRegions);
NXLIB_COMMAND(ComputePointMap);
NXLIB_COMMAND(ConvertTransformation);
	NXLIB_ITEM(SplitRotation);
		NXLIB_VALUE(XYZ);
NXLIB_COMMAND(ChainTransformations);
	NXLIB_ITEM(Transformations);
	NXLIB_ITEM(Transformation);
NXLIB_COMMAND(ComputeImageContrast);
	NXLIB_ITEM(Region);
NXLIB_COMMAND(SaveModel);
	NXLIB_ITEM(Texture);
NXLIB_COMMAND(SaveImage);
	NXLIB_ITEM(Node);
	NXLIB_ITEM(Filename);
NXLIB_COMMAND(LoadImage);
	NXLIB_ITEM(ForceGrayscale);
NXLIB_COMMAND(DiscardPatterns);
NXLIB_COMMAND(ReducePatterns);
	NXLIB_ITEM(DrawOnly);
	NXLIB_ITEM(ShowPattern);
NXLIB_COMMAND(LoadCalibration);
	NXLIB_ITEM(EepromFormat);
NXLIB_COMMAND(StoreCalibration);
	NXLIB_ITEM(Force);
	NXLIB_ITEM(OverwriteWithDynamic);
NXLIB_COMMAND(LoadUEyeParameterSet);
NXLIB_COMMAND(Calibrate);
NXLIB_COMMAND(GetPatternBuffers);
NXLIB_COMMAND(SetPatternBuffers);
NXLIB_COMMAND(ProjectPattern);
NXLIB_COMMAND(CollectPattern);
	NXLIB_ITEM(DecodeData);
	NXLIB_ITEM(MarkOnly);
	NXLIB_ITEM(Buffer);
	NXLIB_ITEM(Brightness);
NXLIB_COMMAND(CalibrateWorkspace);
	NXLIB_ITEM(AlignAxis);
NXLIB_COMMAND(CalibrateHandEye);
	NXLIB_ITEM(Iterations);
	NXLIB_ITEM(Tolerance);
	NXLIB_ITEM(Setup);
		NXLIB_VALUE(Moving);
		NXLIB_VALUE(Fixed);
NXLIB_COMMAND(EstimateDisparitySettings);
NXLIB_COMMAND(EstimatePatternPose);
	NXLIB_ITEM(ReprojectionErrorScale);
	NXLIB_ITEM(Average);
	NXLIB_ITEM(Index);
	NXLIB_ITEM(Recalibrate);
NXLIB_COMMAND(MeasureCalibration);
NXLIB_COMMAND(RenderView);
	NXLIB_ITEM(ColorRepetitionDistance);
	NXLIB_ITEM(ColorOffset);
	NXLIB_ITEM(UseStereoTextures);
NXLIB_COMMAND(RenderDepthMap); // Deprecated since 1.2; renamed to RenderPointMap
NXLIB_COMMAND(RenderPointMap);
NXLIB_COMMAND(ClearOverlay);
	NXLIB_ITEM(ShowRectifiedArea);
NXLIB_COMMAND(GetRawCalibrationData);
NXLIB_COMMAND(GetModelInfo);
	NXLIB_ITEM(WorldCoordinates);
	NXLIB_ITEM(Disparity);
	NXLIB_ITEM(Distance);
	NXLIB_ITEM(Blur);
	NXLIB_ITEM(Focus);
	NXLIB_ITEM(FocalLength);
	NXLIB_ITEM(PixelPitch);
	NXLIB_ITEM(DisparityAccuracy);
	//NXLIB_ITEM(Pattern);
		NXLIB_ITEM(OuterSize);
		//NXLIB_ITEM(GridSpacing);
		//NXLIB_ITEM(GridSize);
		//NXLIB_ITEM(Thickness);
NXLIB_COMMAND(FitPrimitive);
	//NXLIB_ITEM(Points);
	NXLIB_ITEM(Primitive);
	NXLIB_ITEM(InlierThreshold);
	//NXLIB_ITEM(Iterations);
	NXLIB_ITEM(BoundingBox);
	NXLIB_ITEM(InlierFraction);
	NXLIB_ITEM(InlierCount);
	NXLIB_ITEM(Score);
	NXLIB_ITEM(FailureProbability);
	NXLIB_ITEM(Center);
	NXLIB_ITEM(Normal);
	NXLIB_ITEM(Count);
	NXLIB_ITEM(Radius);
		NXLIB_ITEM(Min);
		NXLIB_ITEM(Max);
NXLIB_COMMAND(EthernetConfiguration);
NXLIB_COMMAND(AdjustExposureAndGain);
	NXLIB_ITEM(AdjustExposure);
	NXLIB_ITEM(AdjustGain);
NXLIB_COMMAND(GenerateCalibrationPattern);
	//NXLIB_ITEM(Filename);
	NXLIB_ITEM(Encoding);
	NXLIB_ITEM(Patterns);
		//NXLIB_ITEM(GridSpacing);
		//NXLIB_ITEM(Thickness);
		NXLIB_ITEM(Text);

NXLIB_ITEM(Calibration);
NXLIB_ITEM(Monocular);
	NXLIB_ITEM(Camera);
	NXLIB_ITEM(Distortion);
NXLIB_ITEM(Stereo);
	NXLIB_ITEM(Rectification); // Hidden node to disable rectification; this allows to load and use rectified images in the Raw image nodes. Introduced for compatibility with versions <1.2 which allowed saving of rectified images.
	NXLIB_ITEM(Baseline);
	NXLIB_ITEM(Reprojection);
	//NXLIB_ITEM(Rotation);
	//NXLIB_ITEM(Angle);
		NXLIB_ITEM(Vergence);
		NXLIB_ITEM(Epipolar);
		NXLIB_ITEM(OpticalAxis);
		NXLIB_ITEM(Skew);
NXLIB_ITEM(Dynamic);
	//NXLIB_ITEM(Monocular);
	//NXLIB_ITEM(Stereo);
		//NXLIB_ITEM(Vergence);
		//NXLIB_ITEM(Epipolar);

NXLIB_ITEM(ViewPose);
NXLIB_ITEM(PixelSize);
NXLIB_ITEM(ZBufferOnly);
NXLIB_ITEM(FillXYCoordinates);
NXLIB_ITEM(PatternPose);
NXLIB_ITEM(DefinedPose);
NXLIB_ITEM(Offset);
NXLIB_ITEM(Inverse);
NXLIB_ITEM(Rotation);
	NXLIB_ITEM(Axis);
	NXLIB_ITEM(Angle);
NXLIB_ITEM(Translation);

NXLIB_ITEM(Execute);
	NXLIB_ITEM(Command);
	NXLIB_ITEM(Parameters);
	NXLIB_ITEM(Result);
		NXLIB_ITEM(Time);
		NXLIB_ITEM(ErrorSymbol);
		NXLIB_ITEM(ErrorText);
		NXLIB_ITEM(Progress);
			NXLIB_ITEM(MonocularCalibration);
			NXLIB_ITEM(StereoCalibration);

NXLIB_ITEM(PixelClock);
NXLIB_ITEM(Projector);
NXLIB_ITEM(FrontLight);
NXLIB_ITEM(Hdr);
NXLIB_ITEM(HardwareGamma);
NXLIB_ITEM(GainBoost);
NXLIB_ITEM(Exposure);
NXLIB_ITEM(Gain);
NXLIB_ITEM(MaxFlashTime);
NXLIB_ITEM(MaxGain);
//NXLIB_ITEM(Focus);
NXLIB_ITEM(AutoExposure);
NXLIB_ITEM(AutoGain);
NXLIB_ITEM(AutoFocus);
NXLIB_ITEM(TriggerMode);
NXLIB_ITEM(FlexView);
NXLIB_ITEM(IO);
NXLIB_ITEM(Output);
NXLIB_ITEM(Input);
NXLIB_ITEM(TargetBrightness);
NXLIB_ITEM(BlackLevelOffset);
NXLIB_ITEM(AbsoluteBlackLevelOffset);
NXLIB_ITEM(AutoBlackLevel);
NXLIB_ITEM(UseDisparityMapAreaOfInterest);
NXLIB_ITEM(FlashDelay);

NXLIB_ITEM(Size);
NXLIB_ITEM(Scaling);
NXLIB_ITEM(Binning);

NXLIB_ITEM(Thickness);

NXLIB_ITEM(ReprojectionError);
NXLIB_ITEM(EpipolarError);
NXLIB_ITEM(Contrast);
NXLIB_ITEM(Pattern);
	NXLIB_ITEM(Points);
	NXLIB_ITEM(GridSpacing);
	NXLIB_ITEM(GridSize);

NXLIB_ITEM(Threads);
NXLIB_ITEM(Capture);
NXLIB_ITEM(SurfaceConnectivity);
NXLIB_ITEM(UseOpenGL);
NXLIB_ITEM(RenderView);
	NXLIB_ITEM(ShowSurface);
NXLIB_ITEM(RenderDepthMap); // Deprecated since 1.2, renamed to RenderPointMap.
NXLIB_ITEM(RenderPointMap);
NXLIB_ITEM(RenderPointMapTexture);
NXLIB_ITEM(PointMap);
NXLIB_ITEM(DisparityMap);
NXLIB_ITEM(Links);
NXLIB_ITEM(Link);
	NXLIB_ITEM(Target);
NXLIB_ITEM(ComputeDepth);
	NXLIB_ITEM(StereoMatching);
		NXLIB_ITEM(MinDisparity); // Hidden link to MinimumDisparity (renamed from Version 1.0 to 1.1)
		NXLIB_ITEM(MinimumDisparity);
		NXLIB_ITEM(NumberOfDisparities);
		NXLIB_ITEM(ScaledMinimumDisparity);
		NXLIB_ITEM(ScaledNumberOfDisparities);
		NXLIB_ITEM(DepthChangeCost);
		NXLIB_ITEM(DepthStepCost);
		NXLIB_ITEM(PropagationDecay);
		NXLIB_ITEM(CostScale);
		NXLIB_ITEM(OptimizationProfile);
		NXLIB_ITEM(ShadowingThreshold);
	NXLIB_ITEM(PostProcessing);
		NXLIB_ITEM(UniquenessRatio);
		NXLIB_ITEM(UniquenessOffset);
		NXLIB_ITEM(SpeckleRemoval);
			NXLIB_ITEM(RegionSize);
			NXLIB_ITEM(ComponentThreshold);
		NXLIB_ITEM(Filling);
			//NXLIB_ITEM(RegionSize);
			NXLIB_ITEM(BorderSpread);
		NXLIB_ITEM(MedianFilterRadius);
	NXLIB_ITEM(MeasurementVolume);
		NXLIB_ITEM(Near);
		NXLIB_ITEM(Far);
			NXLIB_ITEM(DisparityStep);
			NXLIB_ITEM(LeftTop);
			NXLIB_ITEM(RightTop);
			NXLIB_ITEM(RightBottom);
			NXLIB_ITEM(LeftBottom);
	NXLIB_ITEM(ValidRegion);
	NXLIB_ITEM(AreaOfInterest);

NXLIB_ITEM(Overlay);
	NXLIB_ITEM(Font); // Deprecated
	//NXLIB_ITEM(Text);
		NXLIB_ITEM(Color);
		//NXLIB_ITEM(Angle);
		NXLIB_ITEM(Mirror);
			NXLIB_ITEM(Vertical);
			NXLIB_ITEM(Horizontal);

NXLIB_ITEM(Images);
	NXLIB_ITEM(Raw);
	NXLIB_ITEM(WithOverlay);
	NXLIB_ITEM(Rectified);
		NXLIB_ITEM(Left);
		NXLIB_ITEM(Right);

NXLIB_ITEM(PatternCount);
NXLIB_ITEM(MonocularPatternCount);
NXLIB_ITEM(Background);

NXLIB_ITEM(Check);

NXLIB_VALUE(Available);
NXLIB_VALUE(InUse);
NXLIB_VALUE(Open);
NXLIB_VALUE(Successful);

NXLIB_VALUE(None);
NXLIB_VALUE(All);
NXLIB_VALUE(Triggered);
NXLIB_VALUE(Untriggered);

NXLIB_VALUE(New);
NXLIB_VALUE(Add);

NXLIB_VALUE(Software);
NXLIB_VALUE(RisingEdge);
NXLIB_VALUE(FallingEdge);

NXLIB_VALUE(Stereo);
NXLIB_VALUE(Monocular);
NXLIB_VALUE(Item);

NXLIB_VALUE(Workspace);
NXLIB_VALUE(Hand);

NXLIB_VALUE(Origin);
NXLIB_VALUE(Axis);

NXLIB_VALUE(X);
NXLIB_VALUE(Y);
NXLIB_VALUE(Z);

NXLIB_VALUE(Auto);

NXLIB_VALUE(Aligned);
NXLIB_VALUE(Diagonal);
NXLIB_VALUE(AlignedAndDiagonal);

NXLIB_VALUE(Console);
NXLIB_VALUE(Buffer);
NXLIB_VALUE(DebugOut);

NXLIB_VALUE(Locked);
NXLIB_VALUE(StructureLocked);
NXLIB_VALUE(NotSpecified);

NXLIB_VALUE(Standard);
NXLIB_VALUE(Link);
NXLIB_VALUE(Binary);
NXLIB_VALUE(LinkHidden);
NXLIB_VALUE(Hidden);

NXLIB_VALUE(USB);
NXLIB_VALUE(Ethernet);
NXLIB_VALUE(Unknown);

NXLIB_VALUE(Plane);
NXLIB_VALUE(Sphere);
NXLIB_VALUE(Cylinder);

NXLIB_ERROR(PatternNotFound);
NXLIB_ERROR(PatternNotDecodable);
NXLIB_ERROR(CommandUnknown);
NXLIB_ERROR(CommandNotAllowed);
NXLIB_ERROR(UnhandledException);
NXLIB_ERROR(OperationCanceled);
NXLIB_ERROR(ParametersInvalid);
NXLIB_ERROR(InvalidPatternBuffer);
NXLIB_ERROR(GLDriver);
NXLIB_ERROR(NoWorkspaceLink);
NXLIB_ERROR(CalibrationFailed);
NXLIB_ERROR(CaptureTimeout);
NXLIB_ERROR(EepromUpgradeNeeded);
NXLIB_ERROR(CameraNotFound);
NXLIB_ERROR(ImageTransferFailed);
NXLIB_ERROR(InvalidCalibrationData);
NXLIB_ERROR(InvalidPairingData);
NXLIB_ERROR(NotEnoughPointsForPrimitive);

NXLIB_ERROR_(FlexViewAvailableImageCountMismatch, "FlexView/AvailableImageCountMismatch");

NXLIB_VALUE_(FilePrefix, "file://");
NXLIB_VALUE_(KitModelTag, "Kit");

#include "nxLibApiErrors.h"

// Tree item types

NXLIB_INT_CONST(ItemTypeInvalid , 0);
NXLIB_INT_CONST(ItemTypeNull    , 1);
NXLIB_INT_CONST(ItemTypeNumber  , 2);
NXLIB_INT_CONST(ItemTypeString  , 3);
NXLIB_INT_CONST(ItemTypeBool    , 4);
NXLIB_INT_CONST(ItemTypeArray   , 5);
NXLIB_INT_CONST(ItemTypeObject  , 6);

#endif /* __NXLIB_CONSTANTS_H__ */
