EXAMPLES_DIR            = ../..
MAKE_INCLUDE_DIR        = $(CURDIR)

include $(MAKE_INCLUDE_DIR)/Common.mk

EXAMPLES		= AsynchronousGrab \
			  ForceIP \
			  ListCameras \
			  ListFeatures \
			  SynchronousGrab

make_%: $(EXAMPLES_DIR)/%/Build/Make/Makefile
	$(MAKE) -C $(EXAMPLES_DIR)/$*/Build/Make
	
clean_%: $(EXAMPLES_DIR)/%/Build/Make/Makefile
	$(MAKE) -C $(EXAMPLES_DIR)/$*/Build/Make clean

all: $(foreach example,$(EXAMPLES),make_$(example))

clean: $(foreach example,$(EXAMPLES),clean_$(example))
