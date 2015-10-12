PROJECT_NAME            = VimbaViewer

PROJECT_DIR             = ../..
EXAMPLES_DIR            = $(PROJECT_DIR)/..
VIMBASDK_DIR			= $(EXAMPLES_DIR)/../..
MAKE_INCLUDE_DIR        = $(CURDIR)/$(EXAMPLES_DIR)/Build/Make

include $(MAKE_INCLUDE_DIR)/Common.mk

CONFIG_DIR          = $(ARCH)_$(WORDSIZE)bit
BIN_FILE            = $(PROJECT_NAME)
BIN_DIR             = binary/$(CONFIG_DIR)
OBJ_DIR             = object/$(CONFIG_DIR)
BIN_PATH            = $(BIN_DIR)/$(BIN_FILE)

all: $(BIN_PATH)

include $(MAKE_INCLUDE_DIR)/VimbaCPP.mk
include $(MAKE_INCLUDE_DIR)/AVTImageTransform.mk
include $(MAKE_INCLUDE_DIR)/Qt.mk

SOURCE_DIR          = $(PROJECT_DIR)/Source
INCLUDE_DIRS        = -I$(SOURCE_DIR) \
					  -I$(SOURCE_DIR)/UI \
					  -I$(SOURCE_DIR)/UI/Histogram \
					  -I$(SOURCE_DIR)/UI/HexEditor \
					  -I$(SOURCE_DIR)/UI/HexEditor/ExternLib/qwt \
					  -I$(OBJ_DIR)

LIBS                = $(VIMBACPP_LIBS) \
					  $(AVTIMAGETRANSFORM_LIBS) \
					  $(QTCORE_LIBS) \
					  $(QTGUI_LIBS) \
					  $(QTSVG_LIBS) \
					  -lrt

DEFINES             =

CFLAGS              = $(COMMON_CFLAGS) \
					  $(AVTIMAGETRANSFORM_CFLAGS) \
                      $(VIMBACPP_CFLAGS) \
                      $(QTCORE_CFLAGS) \
                      $(QTGUI_CFLAGS) \
                      $(QTSVG_CFLAGS)

OBJ_FILES			= $(OBJ_DIR)/CameraObserver.o \
					  $(OBJ_DIR)/FeatureObserver.o \
					  $(OBJ_DIR)/FrameObserver.o \
					  $(OBJ_DIR)/Helper.o \
					  $(OBJ_DIR)/Main.o \
					  $(OBJ_DIR)/MainWindow.o \
					  $(OBJ_DIR)/SplashScreen.o \
					  $(OBJ_DIR)/ViewerWindow.o \
					  $(OBJ_DIR)/ImageSaver.o \
					  $(OBJ_DIR)/qwt_abstract_scale.o \
					  $(OBJ_DIR)/qwt_abstract_scale_draw.o \
					  $(OBJ_DIR)/qwt_abstract_slider.o \
					  $(OBJ_DIR)/qwt_analog_clock.o \
					  $(OBJ_DIR)/qwt_arrow_button.o \
					  $(OBJ_DIR)/qwt_clipper.o \
					  $(OBJ_DIR)/qwt_color_map.o \
					  $(OBJ_DIR)/qwt_column_symbol.o \
					  $(OBJ_DIR)/qwt_compass.o \
					  $(OBJ_DIR)/qwt_compass_rose.o \
					  $(OBJ_DIR)/qwt_counter.o \
					  $(OBJ_DIR)/qwt_curve_fitter.o \
					  $(OBJ_DIR)/qwt_dial.o \
					  $(OBJ_DIR)/qwt_dial_needle.o \
					  $(OBJ_DIR)/qwt_double_range.o \
					  $(OBJ_DIR)/qwt_dyngrid_layout.o \
					  $(OBJ_DIR)/qwt_event_pattern.o \
					  $(OBJ_DIR)/qwt_interval.o \
					  $(OBJ_DIR)/qwt_interval_symbol.o \
					  $(OBJ_DIR)/qwt_knob.o \
					  $(OBJ_DIR)/qwt_legend.o \
					  $(OBJ_DIR)/qwt_legend_item.o \
					  $(OBJ_DIR)/qwt_magnifier.o \
					  $(OBJ_DIR)/qwt_math.o \
					  $(OBJ_DIR)/qwt_matrix_raster_data.o \
					  $(OBJ_DIR)/qwt_null_paintdevice.o \
					  $(OBJ_DIR)/qwt_painter.o \
					  $(OBJ_DIR)/qwt_panner.o \
					  $(OBJ_DIR)/qwt_picker.o \
					  $(OBJ_DIR)/qwt_picker_machine.o \
					  $(OBJ_DIR)/qwt_plot_axis.o \
					  $(OBJ_DIR)/qwt_plot_canvas.o \
					  $(OBJ_DIR)/qwt_plot.o \
					  $(OBJ_DIR)/qwt_plot_curve.o \
					  $(OBJ_DIR)/qwt_plot_dict.o \
					  $(OBJ_DIR)/qwt_plot_directpainter.o \
					  $(OBJ_DIR)/qwt_plot_grid.o \
					  $(OBJ_DIR)/qwt_plot_histogram.o \
					  $(OBJ_DIR)/qwt_plot_intervalcurve.o \
					  $(OBJ_DIR)/qwt_plot_item.o \
					  $(OBJ_DIR)/qwt_plot_layout.o \
					  $(OBJ_DIR)/qwt_plot_magnifier.o \
					  $(OBJ_DIR)/qwt_plot_marker.o \
					  $(OBJ_DIR)/qwt_plot_panner.o \
					  $(OBJ_DIR)/qwt_plot_picker.o \
					  $(OBJ_DIR)/qwt_plot_rasteritem.o \
					  $(OBJ_DIR)/qwt_plot_renderer.o \
					  $(OBJ_DIR)/qwt_plot_rescaler.o \
					  $(OBJ_DIR)/qwt_plot_scaleitem.o \
					  $(OBJ_DIR)/qwt_plot_seriesitem.o \
					  $(OBJ_DIR)/qwt_plot_spectrocurve.o \
					  $(OBJ_DIR)/qwt_plot_spectrogram.o \
					  $(OBJ_DIR)/qwt_plot_svgitem.o \
					  $(OBJ_DIR)/qwt_plot_xml.o \
					  $(OBJ_DIR)/qwt_plot_zoomer.o \
					  $(OBJ_DIR)/qwt_point_3d.o \
					  $(OBJ_DIR)/qwt_point_polar.o \
					  $(OBJ_DIR)/qwt_raster_data.o \
					  $(OBJ_DIR)/qwt_round_scale_draw.o \
					  $(OBJ_DIR)/qwt_sampling_thread.o \
					  $(OBJ_DIR)/qwt_scale_div.o \
					  $(OBJ_DIR)/qwt_scale_draw.o \
					  $(OBJ_DIR)/qwt_scale_engine.o \
					  $(OBJ_DIR)/qwt_scale_map.o \
					  $(OBJ_DIR)/qwt_scale_widget.o \
					  $(OBJ_DIR)/qwt_series_data.o \
					  $(OBJ_DIR)/qwt_slider.o \
					  $(OBJ_DIR)/qwt_spline.o \
					  $(OBJ_DIR)/qwt_symbol.o \
					  $(OBJ_DIR)/qwt_system_clock.o \
					  $(OBJ_DIR)/qwt_text.o \
					  $(OBJ_DIR)/qwt_text_engine.o \
					  $(OBJ_DIR)/qwt_text_label.o \
					  $(OBJ_DIR)/qwt_thermo.o \
					  $(OBJ_DIR)/qwt_wheel.o \
					  $(OBJ_DIR)/CameraTreeWindow.o \
					  $(OBJ_DIR)/ControllerTreeWindow.o \
					  $(OBJ_DIR)/LineEditCompleter.o \
					  $(OBJ_DIR)/MultiCompleter.o \
					  $(OBJ_DIR)/DockWidgetWindow.o \
					  $(OBJ_DIR)/ExComboBox.o \
					  $(OBJ_DIR)/FloatSpinBox.o \
					  $(OBJ_DIR)/IntSpinBox.o \
					  $(OBJ_DIR)/LoggerWindow.o \
					  $(OBJ_DIR)/MainInformationWindow.o \
					  $(OBJ_DIR)/MdiChild.o \
					  $(OBJ_DIR)/SortFilterProxyModel.o \
					  $(OBJ_DIR)/Viewer.o \
					  $(OBJ_DIR)/Commands.o \
					  $(OBJ_DIR)/HexMainWindow.o \
					  $(OBJ_DIR)/HexOptionDialog.o \
					  $(OBJ_DIR)/QHexEdit.o \
					  $(OBJ_DIR)/QHexEdit_p.o \
					  $(OBJ_DIR)/XByteArray.o \
					  $(OBJ_DIR)/Histogram.o \
					  $(OBJ_DIR)/HistogramThread.o \
					  $(OBJ_DIR)/HistogramWindow.o \
					  $(OBJ_DIR)/moc_CameraObserver.o \
					  $(OBJ_DIR)/moc_FeatureObserver.o \
					  $(OBJ_DIR)/moc_FrameObserver.o \
					  $(OBJ_DIR)/moc_MainWindow.o \
					  $(OBJ_DIR)/qrc_VimbaViewer.o \
					  $(OBJ_DIR)/moc_ViewerWindow.o \
					  $(OBJ_DIR)/moc_ImageSaver.o \
					  $(OBJ_DIR)/moc_CameraTreeWindow.o \
					  $(OBJ_DIR)/moc_ControllerTreeWindow.o \
					  $(OBJ_DIR)/moc_LineEditCompleter.o \
					  $(OBJ_DIR)/moc_MultiCompleter.o \
					  $(OBJ_DIR)/moc_DockWidgetWindow.o \
					  $(OBJ_DIR)/moc_ExComboBox.o \
					  $(OBJ_DIR)/moc_FloatSpinBox.o \
					  $(OBJ_DIR)/moc_IntSpinBox.o \
					  $(OBJ_DIR)/moc_LoggerWindow.o \
					  $(OBJ_DIR)/moc_MainInformationWindow.o \
					  $(OBJ_DIR)/moc_MdiChild.o \
					  $(OBJ_DIR)/moc_Viewer.o \
					  $(OBJ_DIR)/moc_HexMainWindow.o \
					  $(OBJ_DIR)/moc_HexOptionDialog.o \
					  $(OBJ_DIR)/moc_QHexEdit.o \
					  $(OBJ_DIR)/moc_QHexEdit_p.o \
					  $(OBJ_DIR)/moc_HistogramThread.o \
					  $(OBJ_DIR)/moc_HistogramWindow.o \
					  $(OBJ_DIR)/moc_qwt_abstract_slider.o \
					  $(OBJ_DIR)/moc_qwt_analog_clock.o \
					  $(OBJ_DIR)/moc_qwt_compass.o \
					  $(OBJ_DIR)/moc_qwt_counter.o \
					  $(OBJ_DIR)/moc_qwt_dial.o \
					  $(OBJ_DIR)/moc_qwt_dyngrid_layout.o \
					  $(OBJ_DIR)/moc_qwt_knob.o \
					  $(OBJ_DIR)/moc_qwt_legend.o \
					  $(OBJ_DIR)/moc_qwt_legend_item.o \
					  $(OBJ_DIR)/moc_qwt_magnifier.o \
					  $(OBJ_DIR)/moc_qwt_panner.o \
					  $(OBJ_DIR)/moc_qwt_picker.o \
					  $(OBJ_DIR)/moc_qwt_plot_canvas.o \
					  $(OBJ_DIR)/moc_qwt_plot.o \
					  $(OBJ_DIR)/moc_qwt_plot_magnifier.o \
					  $(OBJ_DIR)/moc_qwt_plot_panner.o \
					  $(OBJ_DIR)/moc_qwt_plot_picker.o \
					  $(OBJ_DIR)/moc_qwt_plot_renderer.o \
					  $(OBJ_DIR)/moc_qwt_plot_zoomer.o \
					  $(OBJ_DIR)/moc_qwt_sampling_thread.o \
					  $(OBJ_DIR)/moc_qwt_scale_widget.o \
					  $(OBJ_DIR)/moc_qwt_slider.o \
					  $(OBJ_DIR)/moc_qwt_text_label.o \
					  $(OBJ_DIR)/moc_qwt_thermo.o \
					  $(OBJ_DIR)/moc_qwt_wheel.o

GEN_HEADERS			= $(OBJ_DIR)/ui_DirectAccess.h \
					  $(OBJ_DIR)/ui_MainWindow.h \
					  $(OBJ_DIR)/ui_SaveImageOption.h \
					  $(OBJ_DIR)/ui_ViewerOption.h \
					  $(OBJ_DIR)/ui_ViewerWindow.h \
					  $(OBJ_DIR)/ui_OptionDialog.h \
					  $(OBJ_DIR)/ui_SavingProgress.h \
					  $(OBJ_DIR)/ui_StartOption.h

DEPENDENCIES        = VimbaCPP \
					  AVTImageTransform \
					  QtCore \
					  QtGui \
					  QtSvg
					  
$(OBJ_DIR)/moc_%.cpp: $(SOURCE_DIR)/%.h $(OBJ_DIR)
	$(MOC) -o $@ $<

$(OBJ_DIR)/moc_%.cpp: $(SOURCE_DIR)/ExternLib/qwt/%.h $(OBJ_DIR)
	$(MOC) -o $@ $<

$(OBJ_DIR)/moc_%.cpp: $(SOURCE_DIR)/UI/%.h $(OBJ_DIR)
	$(MOC) -o $@ $<

$(OBJ_DIR)/moc_%.cpp: $(SOURCE_DIR)/UI/HexEditor/%.h $(OBJ_DIR)
	$(MOC) -o $@ $<

$(OBJ_DIR)/moc_%.cpp: $(SOURCE_DIR)/UI/Histogram/%.h $(OBJ_DIR)
	$(MOC) -o $@ $<

$(OBJ_DIR)/ui_%.h: $(SOURCE_DIR)/%.ui $(OBJ_DIR)
	$(UIC) -o $@ $<

$(OBJ_DIR)/ui_%.h: $(SOURCE_DIR)/UI/HexEditor/%.ui $(OBJ_DIR)
	$(UIC) -o $@ $<

$(OBJ_DIR)/qrc_%.cpp: $(SOURCE_DIR)/%.qrc $(OBJ_DIR)
	$(RCC) -o $@ $<

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp $(OBJ_DIR) $(GEN_HEADERS)
	$(CXX) -c $(INCLUDE_DIRS) $(DEFINES) $(CFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/ExternLib/qwt/%.cpp $(OBJ_DIR) $(GEN_HEADERS)
	$(CXX) -c $(INCLUDE_DIRS) $(DEFINES) $(CFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/UI/%.cpp $(OBJ_DIR) $(GEN_HEADERS)
	$(CXX) -c $(INCLUDE_DIRS) $(DEFINES) $(CFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/UI/HexEditor/%.cpp $(OBJ_DIR) $(GEN_HEADERS)
	$(CXX) -c $(INCLUDE_DIRS) $(DEFINES) $(CFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/UI/Histogram/%.cpp $(OBJ_DIR) $(GEN_HEADERS)
	$(CXX) -c $(INCLUDE_DIRS) $(DEFINES) $(CFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(OBJ_DIR)/%.cpp $(OBJ_DIR) $(GEN_HEADERS)
	$(CXX) -c $(INCLUDE_DIRS) $(DEFINES) $(CFLAGS) -o $@ $<

$(BIN_PATH): $(DEPENDENCIES) $(OBJ_FILES) $(BIN_DIR)
	$(CXX) $(ARCH_CFLAGS) -o $(BIN_PATH) $(OBJ_FILES) $(LIBS) -Wl,-rpath,'$$ORIGIN'

clean:
	$(RM) binary -r -f
	$(RM) object -r -f

$(OBJ_DIR):
	$(MKDIR) -p $(OBJ_DIR)

$(BIN_DIR):
	$(MKDIR) -p $(BIN_DIR)