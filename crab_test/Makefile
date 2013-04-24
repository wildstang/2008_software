##############################################################################
# Application specific info.  Specify the object target basename, object
# files to be created, and any precompiled libraries here.
##############################################################################

# must make clean after changing RC version
RC_VERSION = 'old'
#RC_VERSION = 'new'

OBJS = ifi_startup.o \
       ifi_utilities.o \
       pwm.o \
       main.o \
       user_routines.o \
       user_routines_fast.o \
       serial_ports.o \
       eeprom.o \
       ws_io.o \
       ws_pid.o \
       ws_general.o \
       ws_calibrate.o \
       ws_feedback.o \
       ws_drive_input.o \
       ws_cc.o \
       ws_trig.o

ifeq ($(RC_VERSION), 'new')
  LIBS = FRC_library.lib
  TARGET = WsCode_2007
else
  LIBS = FRC_library_8520.lib
  TARGET = WsCode_2007
endif

##############################################################################
# Compiler settings.  Specify compiler/linker flags here, as well as default
# directories for system headers/libraries.
##############################################################################
ifndef MCC18PATH
MCC18PATH = /home/WS/mcc18
endif
# Compile for pre-2006 controller
# Must also:
# - replace FRC_Library.lib with FRC_Library_8520.lib
# - replace FRC_alltimers.lib with FRC_alltimers_8520.lib

ifeq ($(RC_VERSION), 'new')
  # Compile for 2006 controller
  CFLAGS = -p=18F8722 -mL -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa- -nw=2066
  LINKER = 18f8722.lkr
else
  # Compile for pre-2006 controller
  CFLAGS = -p=18F8520 -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa- -nw=2066
  LINKER = 18f8520.lkr
endif

COPTS = -D_FRC_BOARD -DREAL_ROBOT -DUSE_ARCTAN
#COPTS = -D_FRC_BOARD -DPROTO
LDFLAGS = -m$(TARGET).map -o$(TARGET).cof
LIBDIRS = -L`cygpath -w $(MCC18PATH)/lib`
INCLUDEDIRS = -I`cygpath -w $(MCC18PATH)/include`
PATH += :$(MCC18PATH)/bin

##############################################################################
# Executable defines & rules - shouldn't need to change anything below here.
##############################################################################
CC  = mcc18
LD  = mplink
RM  = rm
PCC = cpp18

.SUFFIXES: .o .c .i

all: $(TARGET).hex

$(TARGET).hex: $(OBJS) $(LIBS)
	$(LD) $(LINKER) $(OBJS) $(LIBS) $(LDFLAGS) $(LIBDIRS)

.c.o:
	$(CC) $*.c -fo $@ $(INCLUDEDIRS) $(COPTS) $(CFLAGS)

.c.i:
	$(PCC) $*.c $(INCLUDEDIRS) > $@

dl: $(TARGET).hex
	./picloader.exe $(TARGET).hex COM1
	cat COM1


clean:
	@echo "Cleaning build directory..."
	-@$(RM) *.o *.hex *.cof *.cod *.lst *.map *.err
