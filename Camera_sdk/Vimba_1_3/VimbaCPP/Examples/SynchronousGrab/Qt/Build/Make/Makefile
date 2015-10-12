PROJECT_NAME            = SynchronousGrabQt

PROJECT_DIR             = ../..
EXAMPLES_DIR            = $(PROJECT_DIR)/../..
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
include $(MAKE_INCLUDE_DIR)/Qt.mk

SOURCE_DIR          = $(PROJECT_DIR)/Source
INCLUDE_DIRS        = -I$(SOURCE_DIR) \
                      -I$(EXAMPLES_DIR) \
                      -I$(OBJ_DIR)

LIBS                = $(VIMBACPP_LIBS) \
                      $(QTCORE_LIBS) \
                      $(QTGUI_LIBS)

DEFINES             =

CFLAGS              = $(COMMON_CFLAGS) \
                      $(VIMBACPP_CFLAGS) \
                      $(QTCORE_CFLAGS) \
                      $(QTGUI_CFLAGS)

OBJ_FILES           = $(OBJ_DIR)/ApiController.o \
                      $(OBJ_DIR)/SynchronousGrab.o \
                      $(OBJ_DIR)/CameraObserver.o \
                      $(OBJ_DIR)/main.o \
                      $(OBJ_DIR)/moc_SynchronousGrab.o \
                      $(OBJ_DIR)/moc_CameraObserver.o \
                      $(OBJ_DIR)/qrc_SynchronousGrab.o

GEN_HEADERS			= $(OBJ_DIR)/ui_SynchronousGrab.h

DEPENDENCIES        = VimbaCPP \
                      QtCore \
                      QtGui

$(OBJ_DIR)/moc_%.cpp: $(SOURCE_DIR)/%.h $(OBJ_DIR)
	$(MOC) -o $@ $<

$(OBJ_DIR)/ui_%.h: $(SOURCE_DIR)/res/%.ui $(OBJ_DIR)
	$(UIC) -o $@ $<

$(OBJ_DIR)/qrc_%.cpp: $(SOURCE_DIR)/res/%.qrc $(OBJ_DIR)
	$(RCC) -o $@ $<

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp $(OBJ_DIR) $(GEN_HEADERS)
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