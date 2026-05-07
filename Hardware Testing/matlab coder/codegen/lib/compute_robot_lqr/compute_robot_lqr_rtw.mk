###########################################################################
## Makefile generated for component 'compute_robot_lqr'. 
## 
## Makefile     : compute_robot_lqr_rtw.mk
## Generated on : Thu Apr 30 20:32:57 2026
## Final product: ./compute_robot_lqr.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file
# MODELLIB                Static library target

PRODUCT_NAME              = compute_robot_lqr
MAKEFILE                  = compute_robot_lqr_rtw.mk
MATLAB_ROOT               = D:/programs/matlab 2025b/matlab
MATLAB_BIN                = D:/programs/matlab 2025b/matlab/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
START_DIR                 = C:/Users/hassa/OneDrive/Desktop/GIU/git/WHEELE~1/HARDWA~1/MATLAB~1
TGT_FCN_LIB               = ISO_C
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = ../../..
COMPILER_COMMAND_FILE     = compute_robot_lqr_rtw_comp.rsp
CMD_FILE                  = compute_robot_lqr_rtw.rsp
C_STANDARD_OPTS           = -fwrapv
CPP_STANDARD_OPTS         = -fwrapv
EMPTY                     = 
SPACE                     = $(EMPTY) $(EMPTY)
SPACE_TO_QUESTION         = $(subst $(SPACE),?,$1)
ESCAPE_SPACES             = $(subst $(SPACE),\ ,$1)
SUBSTITUTE_ESCAPED_SPACES = $(subst \ ,__<SPACE>__,$1)
ADD_QUOTES                = $(foreach aPath,$1,"$(aPath)")
REVERT_SPACES             = $(subst __<SPACE>__,$(SPACE),$1)
CONVERT_ESCAPED_SPACES_TO_QUOTES = $(call REVERT_SPACES,$(call ADD_QUOTES,$(call SUBSTITUTE_ESCAPED_SPACES,$1)))
MODELLIB                  = compute_robot_lqr.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          MinGW64 | gmake (64-bit Windows)
# Supported Version(s):    8.x
# ToolchainInfo Version:   2025b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS
# MINGW_ROOT
# MINGW_C_STANDARD_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS            = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align -Wno-stringop-overflow
WARN_FLAGS_MAX        = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS        = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align -Wno-stringop-overflow
CPP_WARN_FLAGS_MAX    = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow
MW_EXTERNLIB_DIR      = $(MATLAB_ROOT)/extern/lib/win64/mingw64
SHELL                 = %SystemRoot%/system32/cmd.exe

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lws2_32

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC_PATH = $(MINGW_ROOT)
CC = "$(CC_PATH)/gcc"

# Linker: GNU Linker
LD_PATH = $(MINGW_ROOT)
LD = "$(LD_PATH)/g++"

# C++ Compiler: GNU C++ Compiler
CPP_PATH = $(MINGW_ROOT)
CPP = "$(CPP_PATH)/g++"

# C++ Linker: GNU C++ Linker
CPP_LD_PATH = $(MINGW_ROOT)
CPP_LD = "$(CPP_LD_PATH)/g++"

# Archiver: GNU Archiver
AR_PATH = $(MINGW_ROOT)
AR = "$(AR_PATH)/ar"

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = $(MINGW_ROOT)
MAKE = "$(MAKE_PATH)/mingw32-make.exe"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @del
ECHO                = @echo
MV                  = @move
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c $(MINGW_C_STANDARD_OPTS) -m64 \
                       -O3
CPPFLAGS             = -c $(CPP_STANDARD_OPTS) -m64 \
                       -O3
CPP_LDFLAGS          =  -static -m64
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,--no-undefined \
                         -Wl,--out-implib,$(basename $(PRODUCT)).lib
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              =  -static -m64
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -j $(MAX_MAKE_JOBS) -l $(MAX_MAKE_LOAD_AVG) -Oline -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared -Wl,--no-undefined \
                       -Wl,--out-implib,$(basename $(PRODUCT)).lib



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./compute_robot_lqr.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__USE_MINGW_ANSI_STDIO=1
DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=compute_robot_lqr

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/codegen/lib/compute_robot_lqr/rt_nonfinite.c $(START_DIR)/codegen/lib/compute_robot_lqr/rtGetNaN.c $(START_DIR)/codegen/lib/compute_robot_lqr/rtGetInf.c $(START_DIR)/codegen/lib/compute_robot_lqr/compute_robot_lqr_initialize.c $(START_DIR)/codegen/lib/compute_robot_lqr/compute_robot_lqr_terminate.c $(START_DIR)/codegen/lib/compute_robot_lqr/compute_robot_lqr.c $(START_DIR)/codegen/lib/compute_robot_lqr/diag.c $(START_DIR)/codegen/lib/compute_robot_lqr/mldivide.c $(START_DIR)/codegen/lib/compute_robot_lqr/eig.c $(START_DIR)/codegen/lib/compute_robot_lqr/xnrm2.c $(START_DIR)/codegen/lib/compute_robot_lqr/xdlaev2.c $(START_DIR)/codegen/lib/compute_robot_lqr/xzlartg.c $(START_DIR)/codegen/lib/compute_robot_lqr/eigStandard.c $(START_DIR)/codegen/lib/compute_robot_lqr/xdladiv.c $(START_DIR)/codegen/lib/compute_robot_lqr/xzlarfg.c $(START_DIR)/codegen/lib/compute_robot_lqr/xzlarf.c $(START_DIR)/codegen/lib/compute_robot_lqr/xzsteqr.c $(START_DIR)/codegen/lib/compute_robot_lqr/xzgehrd.c $(START_DIR)/codegen/lib/compute_robot_lqr/xdlahqr.c $(START_DIR)/codegen/lib/compute_robot_lqr/xdlanv2.c $(START_DIR)/codegen/lib/compute_robot_lqr/xdtrevc3.c $(START_DIR)/codegen/lib/compute_robot_lqr/xaxpy.c $(START_DIR)/codegen/lib/compute_robot_lqr/xgemv.c $(START_DIR)/codegen/lib/compute_robot_lqr/mrdivide_helper.c $(START_DIR)/codegen/lib/compute_robot_lqr/xdsterf.c $(START_DIR)/codegen/lib/compute_robot_lqr/xzlascl.c $(START_DIR)/codegen/lib/compute_robot_lqr/xzunghr.c $(START_DIR)/codegen/lib/compute_robot_lqr/xzgebal.c $(START_DIR)/codegen/lib/compute_robot_lqr/xdlaln2.c $(START_DIR)/codegen/lib/compute_robot_lqr/insertionsort.c $(START_DIR)/codegen/lib/compute_robot_lqr/compute_robot_lqr_rtwutil.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = rt_nonfinite.obj rtGetNaN.obj rtGetInf.obj compute_robot_lqr_initialize.obj compute_robot_lqr_terminate.obj compute_robot_lqr.obj diag.obj mldivide.obj eig.obj xnrm2.obj xdlaev2.obj xzlartg.obj eigStandard.obj xdladiv.obj xzlarfg.obj xzlarf.obj xzsteqr.obj xzgehrd.obj xdlahqr.obj xdlanv2.obj xdtrevc3.obj xaxpy.obj xgemv.obj mrdivide_helper.obj xdsterf.obj xzlascl.obj xzunghr.obj xzgebal.obj xdlaln2.obj insertionsort.obj compute_robot_lqr_rtwutil.obj

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_TFL = -msse2 -fno-predictive-commoning
CFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CFLAGS += $(CFLAGS_TFL) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_TFL = -msse2 -fno-predictive-commoning
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS += $(CPPFLAGS_TFL) $(CPPFLAGS_BASIC)

#---------------------
# MEX C++ Compiler
#---------------------

MEX_CPP_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CPPFLAGS += $(MEX_CPP_Compiler_BASIC)

#-----------------
# MEX Compiler
#-----------------

MEX_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CFLAGS += $(MEX_Compiler_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################


MINGW_C_STANDARD_OPTS = $(C_STANDARD_OPTS)


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) @$(CMD_FILE)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.obj : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rt_nonfinite.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetNaN.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/rtGetNaN.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetInf.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/rtGetInf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


compute_robot_lqr_initialize.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/compute_robot_lqr_initialize.c
	$(CC) $(CFLAGS) -o "$@" "$<"


compute_robot_lqr_terminate.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/compute_robot_lqr_terminate.c
	$(CC) $(CFLAGS) -o "$@" "$<"


compute_robot_lqr.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/compute_robot_lqr.c
	$(CC) $(CFLAGS) -o "$@" "$<"


diag.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/diag.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mldivide.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/mldivide.c
	$(CC) $(CFLAGS) -o "$@" "$<"


eig.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/eig.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xnrm2.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xnrm2.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xdlaev2.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xdlaev2.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzlartg.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xzlartg.c
	$(CC) $(CFLAGS) -o "$@" "$<"


eigStandard.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/eigStandard.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xdladiv.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xdladiv.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzlarfg.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xzlarfg.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzlarf.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xzlarf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzsteqr.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xzsteqr.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzgehrd.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xzgehrd.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xdlahqr.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xdlahqr.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xdlanv2.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xdlanv2.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xdtrevc3.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xdtrevc3.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xaxpy.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xaxpy.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xgemv.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xgemv.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mrdivide_helper.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/mrdivide_helper.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xdsterf.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xdsterf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzlascl.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xzlascl.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzunghr.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xzunghr.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzgebal.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xzgebal.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xdlaln2.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/xdlaln2.c
	$(CC) $(CFLAGS) -o "$@" "$<"


insertionsort.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/insertionsort.c
	$(CC) $(CFLAGS) -o "$@" "$<"


compute_robot_lqr_rtwutil.obj : $(START_DIR)/codegen/lib/compute_robot_lqr/compute_robot_lqr_rtwutil.c
	$(CC) $(CFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(subst /,\,$(PRODUCT))
	$(RM) $(subst /,\,$(ALL_OBJS))
	$(ECHO) "### Deleted all derived files."


