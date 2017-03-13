#
# OMNeT++/OMNEST Makefile for multiUAV
#
# This file was generated with the command:
#  opp_makemake -f --deep -O out -DWITH_OSG
#

# Name of target to be created (-o option)
TARGET = multiUAV$(EXE_SUFFIX)

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(QTENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)
#USERIF_LIBS = $(QTENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I. \
    -Idata \
    -Idata/resources \
    -Idata/resources/textures_us \
    -Idata/resources/textures_us/barriers \
    -Idata/resources/textures_us/commercial \
    -Idata/resources/textures_us/commercial/Tiles \
    -Idata/resources/textures_us/misc \
    -Idata/resources/textures_us/residential \
    -Idata/resources/textures_us/residential/tiles \
    -Idata/resources/textures_us/rooftop \
    -Idata/resources/textures_us/rooftop/tiled

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS =

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc, .msg and .sm files
OBJS = \
    $O/Battery.o \
    $O/ChannelController.o \
    $O/ChargingNode.o \
    $O/Command.o \
    $O/CommandExecEngine.o \
    $O/fallback.o \
    $O/GenericNode.o \
    $O/MissionControl.o \
    $O/MobileNode.o \
    $O/OsgEarthScene.o \
    $O/UAVNode.o \
    $O/CmdCompletedMsg_m.o \
    $O/MissionMsg_m.o

# Message files
MSGFILES = \
    CmdCompletedMsg.msg \
    MissionMsg.msg

# SM files
SMFILES =

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppmain$D $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS) -DWITH_OSG $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)
SMCOPTS =

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# inserted from file 'makefrag':
# add required libraries for OpenSceneGraph and osgEarth
ifneq ($(OSG_LIBS),)
LIBS += $(OSG_LIBS) -losgDB -losgAnimation
endif
ifneq ($(OSGEARTH_LIBS),)
LIBS += $(OSGEARTH_LIBS) -losgEarthFeatures -losgEarthSymbology -losgEarthAnnotation
endif

# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

.PHONY: all clean cleanall depend msgheaders smheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

%_sm.cc %_sm.h: %.sm
	$(qecho) SMC: $<
	$(Q)$(SMC) -c++ -suffix cc $(SMCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

smheaders: $(SMFILES:.sm=_sm.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f multiUAV multiUAV.exe libmultiUAV.so libmultiUAV.a libmultiUAV.dll libmultiUAV.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h ./*_sm.cc ./*_sm.h
	$(Q)-rm -f data/*_m.cc data/*_m.h data/*_sm.cc data/*_sm.h
	$(Q)-rm -f data/resources/*_m.cc data/resources/*_m.h data/resources/*_sm.cc data/resources/*_sm.h
	$(Q)-rm -f data/resources/textures_us/*_m.cc data/resources/textures_us/*_m.h data/resources/textures_us/*_sm.cc data/resources/textures_us/*_sm.h
	$(Q)-rm -f data/resources/textures_us/barriers/*_m.cc data/resources/textures_us/barriers/*_m.h data/resources/textures_us/barriers/*_sm.cc data/resources/textures_us/barriers/*_sm.h
	$(Q)-rm -f data/resources/textures_us/commercial/*_m.cc data/resources/textures_us/commercial/*_m.h data/resources/textures_us/commercial/*_sm.cc data/resources/textures_us/commercial/*_sm.h
	$(Q)-rm -f data/resources/textures_us/commercial/Tiles/*_m.cc data/resources/textures_us/commercial/Tiles/*_m.h data/resources/textures_us/commercial/Tiles/*_sm.cc data/resources/textures_us/commercial/Tiles/*_sm.h
	$(Q)-rm -f data/resources/textures_us/misc/*_m.cc data/resources/textures_us/misc/*_m.h data/resources/textures_us/misc/*_sm.cc data/resources/textures_us/misc/*_sm.h
	$(Q)-rm -f data/resources/textures_us/residential/*_m.cc data/resources/textures_us/residential/*_m.h data/resources/textures_us/residential/*_sm.cc data/resources/textures_us/residential/*_sm.h
	$(Q)-rm -f data/resources/textures_us/residential/tiles/*_m.cc data/resources/textures_us/residential/tiles/*_m.h data/resources/textures_us/residential/tiles/*_sm.cc data/resources/textures_us/residential/tiles/*_sm.h
	$(Q)-rm -f data/resources/textures_us/rooftop/*_m.cc data/resources/textures_us/rooftop/*_m.h data/resources/textures_us/rooftop/*_sm.cc data/resources/textures_us/rooftop/*_sm.h
	$(Q)-rm -f data/resources/textures_us/rooftop/tiled/*_m.cc data/resources/textures_us/rooftop/tiled/*_m.h data/resources/textures_us/rooftop/tiled/*_sm.cc data/resources/textures_us/rooftop/tiled/*_sm.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES) $(SM_CC_FILES)  ./*.cc data/*.cc data/resources/*.cc data/resources/textures_us/*.cc data/resources/textures_us/barriers/*.cc data/resources/textures_us/commercial/*.cc data/resources/textures_us/commercial/Tiles/*.cc data/resources/textures_us/misc/*.cc data/resources/textures_us/residential/*.cc data/resources/textures_us/residential/tiles/*.cc data/resources/textures_us/rooftop/*.cc data/resources/textures_us/rooftop/tiled/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/Battery.o: Battery.cc \
	Battery.h
$O/ChannelController.o: ChannelController.cc \
	ChannelController.h \
	CmdCompletedMsg_m.h \
	Command.h \
	CommandExecEngine.h \
	GenericNode.h \
	MissionMsg_m.h \
	OsgEarthScene.h
$O/ChargingNode.o: ChargingNode.cc \
	Battery.h \
	ChargingNode.h \
	CmdCompletedMsg_m.h \
	Command.h \
	CommandExecEngine.h \
	GenericNode.h \
	MissionMsg_m.h \
	MobileNode.h \
	OsgEarthScene.h
$O/CmdCompletedMsg_m.o: CmdCompletedMsg_m.cc \
	CmdCompletedMsg_m.h
$O/Command.o: Command.cc \
	Command.h
$O/CommandExecEngine.o: CommandExecEngine.cc \
	Battery.h \
	ChargingNode.h \
	CmdCompletedMsg_m.h \
	Command.h \
	CommandExecEngine.h \
	GenericNode.h \
	MissionMsg_m.h \
	MobileNode.h \
	OsgEarthScene.h \
	UAVNode.h
$O/GenericNode.o: GenericNode.cc \
	ChannelController.h \
	CmdCompletedMsg_m.h \
	Command.h \
	CommandExecEngine.h \
	GenericNode.h \
	MissionMsg_m.h \
	OsgEarthScene.h
$O/MissionControl.o: MissionControl.cc \
	Battery.h \
	ChargingNode.h \
	CmdCompletedMsg_m.h \
	Command.h \
	CommandExecEngine.h \
	GenericNode.h \
	MissionControl.h \
	MissionMsg_m.h \
	MobileNode.h \
	OsgEarthScene.h \
	UAVNode.h
$O/MissionMsg_m.o: MissionMsg_m.cc \
	Command.h \
	MissionMsg_m.h
$O/MobileNode.o: MobileNode.cc \
	Battery.h \
	ChargingNode.h \
	CmdCompletedMsg_m.h \
	Command.h \
	CommandExecEngine.h \
	GenericNode.h \
	MissionMsg_m.h \
	MobileNode.h \
	OsgEarthScene.h
$O/OsgEarthScene.o: OsgEarthScene.cc \
	OsgEarthScene.h
$O/UAVNode.o: UAVNode.cc \
	Battery.h \
	ChannelController.h \
	ChargingNode.h \
	CmdCompletedMsg_m.h \
	Command.h \
	CommandExecEngine.h \
	GenericNode.h \
	MissionMsg_m.h \
	MobileNode.h \
	OsgEarthScene.h \
	UAVNode.h
$O/fallback.o: fallback.cc

