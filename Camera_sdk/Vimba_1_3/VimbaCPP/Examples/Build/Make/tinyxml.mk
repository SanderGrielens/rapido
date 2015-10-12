include $(MAKE_INCLUDE_DIR)/Common.mk

PKGCFG_TINYXML_LIBS		= $(shell $(PKGCFG) --libs tinyxml)

#Compile options needed for tinyxml
TINYXML_CFLAGS	= $(shell $(PKGCFG) --cflags tinyxml)

#Linker options needed for tinyxml
TINYXML_LIBS	= $(if $(PKGCFG_TINYXML_LIBS),$(PKGCFG_TINYXML_LIBS),-ltinyxml)

#Operations we have to do in order to prepare tinyxml
tinyxml: