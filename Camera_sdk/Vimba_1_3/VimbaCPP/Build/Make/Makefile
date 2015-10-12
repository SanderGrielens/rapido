PROJECT_NAME            = VimbaCPP

PROJECT_DIR             = ../..
VIMBASDK_DIR            = $(PROJECT_DIR)/..
MAKE_INCLUDE_DIR        = $(CURDIR)

include $(MAKE_INCLUDE_DIR)/Common.mk

CONFIG_DIR          = $(ARCH)_$(WORDSIZE)bit
BIN_FILE            = lib$(PROJECT_NAME).so
BIN_DIR             = dynamic/$(CONFIG_DIR)
OBJ_DIR             = object/$(CONFIG_DIR)
BIN_PATH            = $(BIN_DIR)/$(BIN_FILE)

all: $(BIN_PATH)

include $(MAKE_INCLUDE_DIR)/VimbaC.mk

SOURCE_DIR          = $(PROJECT_DIR)/Source
INCLUDE_DIRS        = -I$(PROJECT_DIR)/..

LIBS                = $(VIMBAC_LIBS)

DEFINES             =

CFLAGS              = $(COMMON_CFLAGS) \
                      $(VIMBAC_CFLAGS)

OBJ_FILES			= $(OBJ_DIR)/AncillaryData.o \
					  $(OBJ_DIR)/BaseFeature.o \
					  $(OBJ_DIR)/BasicLockable.o \
					  $(OBJ_DIR)/BoolFeature.o \
					  $(OBJ_DIR)/Camera.o \
					  $(OBJ_DIR)/Clock.o \
					  $(OBJ_DIR)/CommandFeature.o \
					  $(OBJ_DIR)/Condition.o \
					  $(OBJ_DIR)/ConditionHelper.o \
					  $(OBJ_DIR)/DefaultCameraFactory.o \
					  $(OBJ_DIR)/EnumEntry.o \
					  $(OBJ_DIR)/EnumFeature.o \
					  $(OBJ_DIR)/FeatureContainer.o \
					  $(OBJ_DIR)/Feature.o \
					  $(OBJ_DIR)/FileLogger.o \
					  $(OBJ_DIR)/FloatFeature.o \
					  $(OBJ_DIR)/Frame.o \
					  $(OBJ_DIR)/FrameHandler.o \
					  $(OBJ_DIR)/Interface.o \
					  $(OBJ_DIR)/IntFeature.o \
					  $(OBJ_DIR)/Mutex.o \
					  $(OBJ_DIR)/MutexGuard.o \
					  $(OBJ_DIR)/RawFeature.o \
					  $(OBJ_DIR)/Semaphore.o \
					  $(OBJ_DIR)/StringFeature.o \
					  $(OBJ_DIR)/VimbaSystem.o

DEPENDENCIES        = VimbaC

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp $(OBJ_DIR)
	$(CXX) -c $(INCLUDE_DIRS) $(DEFINES) $(CFLAGS) -o $@ $<

$(BIN_PATH): $(DEPENDENCIES) $(OBJ_FILES) $(BIN_DIR)
	$(CXX) -shared $(ARCH_CFLAGS) -o $(BIN_PATH) $(OBJ_FILES) $(LIBS) -Wl,-rpath,'$$ORIGIN'

clean:
	$(RM) dynamic -r -f
	$(RM) object -r -f

$(OBJ_DIR):
	$(MKDIR) -p $(OBJ_DIR)

$(BIN_DIR):
	$(MKDIR) -p $(BIN_DIR)