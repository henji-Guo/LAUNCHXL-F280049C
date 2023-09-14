# Project Name #
NAME = demo

# TOOLCHAIN #

TOOLCHAIN = /tools/ti-cgt-c2000_22.6.0.LTS

CC      = $(TOOLCHAIN)/bin/cl2000
AR      = $(TOOLCHAIN)/bin/ar2000
LD      = $(TOOLCHAIN)/bin/lnk2000
DUMP    = $(TOOLCHAIN)/bin/dis2000

# root directory #
ROOTDIR = $(shell pwd)

# Include  #
INCDIR  = --include_path=$(TOOLCHAIN)/include \
		  --include_path=$(ROOTDIR)/ \
		  --include_path=$(ROOTDIR)/include \
          --include_path=$(ROOTDIR)/device_support/ \
          --include_path=$(ROOTDIR)/device_support/driverlib/ \
          --include_path=$(ROOTDIR)/device_support/driverlib/inc \
          --include_path=$(ROOTDIR)/device_support/headers/include \
		  --include_path=$(ROOTDIR)/src_control/ \
		  --include_path=${ROOTDIR}/libraries/ \
		  --include_path=${ROOTDIR}/libraries/control/pi \
		  --include_path=${ROOTDIR}/libraries/control/mtpa \
		  --include_path=${ROOTDIR}/libraries/control/vs_freq \
		  --include_path=${ROOTDIR}/libraries/control/vib_comp \
		  --include_path=${ROOTDIR}/libraries/control/dclink_ss \
		  --include_path=${ROOTDIR}/libraries/filter/filter_fo \
		  --include_path=${ROOTDIR}/libraries/filter/filter_so \
		  --include_path=${ROOTDIR}/libraries/filter/offset \
		  --include_path=${ROOTDIR}/libraries/filter/notch \
		  --include_path=${ROOTDIR}/libraries/observers/est_lib \
		  --include_path=${ROOTDIR}/libraries/observers/esmo \
		  --include_path=${ROOTDIR}/libraries/observers/speedfr \
		  --include_path=${ROOTDIR}/libraries/observers/ssipd \
		  --include_path=${ROOTDIR}/libraries/transforms/clarke \
		  --include_path=${ROOTDIR}/libraries/transforms/ipark \
		  --include_path=${ROOTDIR}/libraries/transforms/park \
		  --include_path=${ROOTDIR}/libraries/transforms/svgen \
		  --include_path=${ROOTDIR}/libraries/transforms/volts \
		  --include_path=${ROOTDIR}/libraries/utilities/angle_gen \
		  --include_path=${ROOTDIR}/libraries/utilities/cpu_time \
		  --include_path=${ROOTDIR}/libraries/utilities/datalog \
		  --include_path=${ROOTDIR}/libraries/utilities/traj \
		  --include_path=${ROOTDIR}/libraries/utilities/types/include \
		  --include_path=${ROOTDIR}/libraries/utilities/power_measurement \
		  --include_path=${ROOTDIR}/libraries/utilities/spll \
		  --include_path=${ROOTDIR}/libraries/sfra \
		  --include_path=${ROOTDIR}/libraries/dacs/dac128s085 \
		  --include_path=${ROOTDIR}/libraries/control/DCL/c28 \



# C flags # 
CFLAGS  = -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0
CFLAGS += -O2 -g --opt_for_speed=2 --fp_mode=relaxed --asm_listing --gen_func_subsections=on --c99
CFLAGS += --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi
CFLAGS += $(INCDIR)

# C Define #
CFLAGS += --define=_FLASH 
# CFLAGS += --define=_RAM 
CFLAGS += --define=_INLINE \
          --define=_F28004x \
          --define=_PMSM_FAST_LIB \
          --define=SYSCONFIG_EN_N \
          --define=NEST_INT_ENABLE \
          --define=DMCPFC_REV3P2 \
          --define=PFC_DISABLE \
          --define=MOTOR1_DISABLE_N \
          --define=MOTOR2_DISABLE_N \
          --define=ILPFC_DCH_EN \
          --define=ILPFC_SWM_EN_N \
          --define=SPPFC_CHA_EN_N \
          --define=SPPFC_CHB_EN_N \
          --define=MOTOR1_FAST \
          --define=MOTOR1_ESMO_N \
          --define=MOTOR2_FAST \
          --define=MOTOR2_ESMO_N \
          --define=MOTOR1_SSIPD_N \
          --define=MOTOR1_MTPA_N \
          --define=MOTOR1_FWC_N \
          --define=MOTOR1_VIBCOMPA_N \
          --define=MOTOR1_VIBCOMPT_N \
          --define=DATALOGI4_EN_N \
          --define=DATALOGF2_EN_N \
          --define=DATALOG_PFC_N \
          --define=DATALOG_MOTOR1_N \
          --define=DATALOG_MOTOR2_N \
          --define=DAC128S_ENABLE \
          --define=DAC128S_SPIA \
          --define=SFRA_ENABLE_N \
          --define=CPUTIME_ENABLE_N \
          --define=GPTEST_ENABLE_N \
          --define=DEBUG_MONITOR_EN_N



# Link Flags #
LDFLAGS  = --stack_size=0x300 --warn_sections
LDFLAGS += --entry_point=code_start --rom_model --reread_libs
LINKSCIRTS += f28004x_headers_nonbios.cmd 
LINKSCIRTS += 28004x_launchpad_flash_lnk.cmd

# Library PATH  #
LIB_PATH = -i$(ROOTDIR)/ \
		   -i$(TOOLCHAIN)/lib \
		   -i$(ROOTDIR)/libraries/sfra/ \
		   -i$(ROOTDIR)/libraries/observers/est_lib/ \
		   
# Library #
# LIBS = -llibc.a
LIBS = -lrts2800_fpu32_eabi.lib \
       -lsfra_f32_tmu_eabi.lib \
       -lfast_full_lib_eabi.lib \

# Debug directory #
OBJ_PATH = $(BUILD_DIR)/Debug

# source directory #
SRC_PATH = $(ROOTDIR)/ \
		   $(ROOTDIR)/device_support/ \
           $(ROOTDIR)/device_support/driverlib \
		   ${ROOTDIR}/libraries/control/pi \
		   ${ROOTDIR}/libraries/control/mtpa \
		   ${ROOTDIR}/libraries/control/vs_freq \
		   ${ROOTDIR}/libraries/control/vib_comp \
		   ${ROOTDIR}/libraries/control/dclink_ss \
		   ${ROOTDIR}/libraries/filter/filter_fo \
		   ${ROOTDIR}/libraries/filter/filter_so \
		   ${ROOTDIR}/libraries/filter/offset \
		   ${ROOTDIR}/libraries/filter/notch \
		   ${ROOTDIR}/libraries/observers/est_lib \
		   ${ROOTDIR}/libraries/observers/esmo \
		   ${ROOTDIR}/libraries/observers/speedfr \
		   ${ROOTDIR}/libraries/observers/ssipd \
		   ${ROOTDIR}/libraries/transforms/clarke \
		   ${ROOTDIR}/libraries/transforms/ipark \
		   ${ROOTDIR}/libraries/transforms/park \
		   ${ROOTDIR}/libraries/transforms/svgen \
		   ${ROOTDIR}/libraries/transforms/volts \
		   ${ROOTDIR}/libraries/utilities/angle_gen \
		   ${ROOTDIR}/libraries/utilities/cpu_time \
		   ${ROOTDIR}/libraries/utilities/datalog \
		   ${ROOTDIR}/libraries/utilities/traj \
		   ${ROOTDIR}/libraries/utilities/types/include \
		   ${ROOTDIR}/libraries/utilities/power_measurement \
		   ${ROOTDIR}/libraries/utilities/spll \
		   ${ROOTDIR}/libraries/sfra \
		   ${ROOTDIR}/libraries/dacs/dac128s085 \
		   ${ROOTDIR}/libraries/control/DCL/c28 \

# Gen OBJ #
SRC = $(foreach x,$(SRC_PATH), $(wildcard $(addprefix $(x)/*, .c .S .asm)))
OBJ = $(addprefix $(OBJ_PATH)/, $(addsuffix .obj, $(notdir $(basename $(SRC)))))

ifeq ($(BUILD_DIR),)
BUILD_DIR := $(shell /bin/pwd)
endif
saved-output := $(BUILD_DIR)

# Attemp to create a output directory.
$(shell [ -d ${BUILD_DIR} ] || mkdir -p ${BUILD_DIR})
# Verify if it was successful.
BUILD_DIR := $(shell cd $(BUILD_DIR) && /bin/pwd)
$(if $(BUILD_DIR),,$(error output directory "$(saved-output)" does not exist))

$(shell [ -d ${BUILD_DIR}/Debug ] || mkdir -p ${OBJ_PATH})

OUT_NAME = $(OBJ_PATH)/$(NAME)

vpath %.c $(SRC_PATH)
vpath %.S $(SRC_PATH)
vpath %.asm $(SRC_PATH)

all: $(OUT_NAME).out

$(OUT_NAME).out:$(OBJ)

	$(CC) $(CFLAGS) -z $(LIB_PATH) -o "$(OUT_NAME).out" $(LDFLAGS) --xml_link_info="$(OBJ_PATH)/$(NAME)_linkInfo.xml" -m"$(OBJ_PATH)/$(NAME).map" $(OBJ) $(LINKSCIRTS) $(LIBS)
	@$(DUMP) -all $(OUT_NAME).out > $(OUT_NAME).asm
	@echo -e "------------ \033[0;32mBUILD SUCCESS\033[0m ------------"
	@echo "Generate $(OUT_NAME).out"
	@echo "Generate $(OUT_NAME).asm"
	@echo "Generate $(OBJ_PATH)/$(NAME).map"
	@echo "Generate $(OBJ_PATH)/$(NAME)_linkInfo.xml"
	
$(OBJ_PATH)/%obj:%c
	@$(CC) $(CFLAGS) --preproc_with_compile --preproc_dependency="$(OBJ_PATH)/$(basename $(<F)).d_raw" --obj_directory="$(OBJ_PATH)" $<
	@echo "Finished building: $<"
$(OBJ_PATH)/%obj:%S
	@$(CC) $(CFLAGS) --preproc_with_compile --preproc_dependency="$(OBJ_PATH)/$(basename $(<F)).d_raw" --obj_directory="$(OBJ_PATH)" $<
	@echo "Finished building: $<"
$(OBJ_PATH)/%obj:%asm
	@$(CC) $(CFLAGS) --preproc_with_compile --preproc_dependency="$(OBJ_PATH)/$(basename $(<F)).d_raw" --obj_directory="$(OBJ_PATH)" $<
	@echo "Finished building: $<"

.PHONY: tags
tags:
	ctags -R .

.PHONY: clean
clean:
	rm -rf $(OBJ_PATH)/*
	rm -rf tags
	@echo -e "------------ \033[0;32mFinished clean\033[0m ------------"



