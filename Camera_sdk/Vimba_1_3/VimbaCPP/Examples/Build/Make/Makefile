EXAMPLES_DIR            = ../..
MAKE_INCLUDE_DIR        = $(CURDIR)

include $(MAKE_INCLUDE_DIR)/Common.mk

QT_SUPPORT			= true

ifeq ($(QT_SUPPORT),true)
QT_EXAMPLES			= VimbaViewer

QT_SUB_EXAMPLES			= AsynchronousGrab \
				  SynchronousGrab			  
endif		  

CONSOLE_EXAMPLES		= BandwidthHelper \
				  CameraFactory \
				  ListCameras \
				  ListFeatures \
				  LookUpTable \
				  ShadingData \
				  UserSet
				  
CONSOLE_SUB_EXAMPLES		= AsynchronousGrab \
				  SynchronousGrab			  

make_%: $(EXAMPLES_DIR)/%/Build/Make/Makefile
	$(MAKE) -C $(EXAMPLES_DIR)/$*/Build/Make
	
clean_%: $(EXAMPLES_DIR)/%/Build/Make/Makefile
	$(MAKE) -C $(EXAMPLES_DIR)/$*/Build/Make clean

make_%Console: $(EXAMPLES_DIR)/%/Console/Build/Make/Makefile
	$(MAKE) -C $(EXAMPLES_DIR)/$*/Console/Build/Make

clean_%Console: $(EXAMPLES_DIR)/%/Console/Build/Make/Makefile
	$(MAKE) -C $(EXAMPLES_DIR)/$*/Console/Build/Make clean
	
make_%Qt: $(EXAMPLES_DIR)/%/Qt/Build/Make/Makefile
	$(MAKE) -C $(EXAMPLES_DIR)/$*/Qt/Build/Make

clean_%Qt: $(EXAMPLES_DIR)/%/Qt/Build/Make/Makefile
	$(MAKE) -C $(EXAMPLES_DIR)/$*/Qt/Build/Make clean

all: $(foreach example,$(CONSOLE_EXAMPLES),make_$(example)) \
     $(foreach example,$(CONSOLE_SUB_EXAMPLES),make_$(example)Console) \
     $(foreach example,$(QT_EXAMPLES),make_$(example)) \
     $(foreach example,$(QT_SUB_EXAMPLES),make_$(example)Qt)
     
clean: $(foreach example,$(CONSOLE_EXAMPLES),clean_$(example)) \
       $(foreach example,$(CONSOLE_SUB_EXAMPLES),clean_$(example)Console) \
       $(foreach example,$(QT_EXAMPLES),clean_$(example)) \
       $(foreach example,$(QT_SUB_EXAMPLES),clean_$(example)Qt)
