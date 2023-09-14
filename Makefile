# Project Name #
NAME = f280049c_demo

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

# C flags # 
CFLAGS  = -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0
CFLAGS += -O2 -g --opt_for_speed=2 --fp_mode=relaxed --asm_listing --gen_func_subsections=on --c99
CFLAGS += --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi
CFLAGS += $(INCDIR)

# C Define #
CFLAGS += --define=_FLASH 
# CFLAGS += --define=_RAM 
CFLAGS += 

# Link Flags #
LDFLAGS  = --stack_size=0x300 --warn_sections
LDFLAGS += --entry_point=code_start --rom_model --reread_libs
LINKSCIRTS += f28004x_headers_nonbios.cmd 
LINKSCIRTS += 28004x_launchpad_flash_lnk.cmd

# Library PATH  #
LIB_PATH = -i$(ROOTDIR)/ \
		   -i$(TOOLCHAIN)/lib
		   
# Library #
# LIBS = -llibc.a
LIBS = -lrts2800_fpu32_eabi.lib

# Debug directory #
OBJ_PATH = $(BUILD_DIR)/Debug

# source directory #
SRC_PATH = $(ROOTDIR)/ \
		   $(ROOTDIR)/device_support/ \
           $(ROOTDIR)/device_support/driverlib \

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



