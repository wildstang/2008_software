##############################################################################
# Application specific info.  Specify the object target basename, object
# files to be created, and any precompiled libraries here.
##############################################################################

# must make clean after changing RC version
#RC_VERSION = 'old'
RC_VERSION = 'new'

OBJS = ifi_code.o \
       ifi_frc.o \
       gyro.o \
       pwm.o \
       serial_ports.o \
       disabled.o \
       autonomous.o \
       teleop.o \

ifeq ($(RC_VERSION), 'new')
  OBJS += adc.o
  LIBS = ifi_frc_8722_30.lib
  TARGET = ifi_frc_gyro
else
  OBJS += adc_8520.o
  LIBS = ifi_frc_8520_30.lib
  TARGET = ifi_frc_gyro
endif

##############################################################################
# Compiler settings.  Specify compiler/linker flags here, as well as default
# directories for system headers/libraries.
##############################################################################
ifndef MCC18PATH_30
MCC18PATH_30 = /home/WS/mcc18
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

COPTS = -D_FRC_BOARD -DREAL_ROBOT
#COPTS = -D_FRC_BOARD -DPROTO
LDFLAGS = -m$(TARGET).map -o$(TARGET).cof
LIBDIRS = -L`cygpath -w $(MCC18PATH_30)/lib`
INCLUDEDIRS = -I`cygpath -w $(MCC18PATH_30)/include`
PATH += :$(MCC18PATH_30)/bin

##############################################################################
# Executable defines & rules - shouldn't need to change anything below here.
##############################################################################
CC  = mcc18
LD  = mplink
PCC = cpp18
RM  = rm

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

