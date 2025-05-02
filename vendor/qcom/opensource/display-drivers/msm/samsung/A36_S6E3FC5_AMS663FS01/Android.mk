XXD := /usr/bin/xxd
SED := /bin/sed

#Translate .dat file to .h to cover the case which can not use request_firmware(Recovery Mode)
CLEAR_TMP := $(shell rm -f A36_S6E3FC5_AMS663FS01_PDF_DATA)
CLEAR_CURRENT := $(shell rm -f $(DISPLAY_BLD_DIR)/msm/samsung/A36_S6E3FC5_AMS663FS01/A36_S6E3FC5_AMS663FS01_PDF.h)
COPY_TO_HERE := $(shell cp -vf $(DISPLAY_BLD_DIR)/msm/samsung/panel_data_file/A36_S6E3FC5_AMS663FS01.dat A36_S6E3FC5_AMS663FS01_PDF_DATA)
DATA_TO_HEX := $(shell $(XXD) -i A36_S6E3FC5_AMS663FS01_PDF_DATA > $(DISPLAY_BLD_DIR)/msm/samsung/A36_S6E3FC5_AMS663FS01/A36_S6E3FC5_AMS663FS01_PDF.h)
ADD_NULL_CHR := $(shell $(SED) -i -e 's/\([0-9a-f]\)$$/\0, 0x00/' $(DISPLAY_BLD_DIR)/msm/samsung/A36_S6E3FC5_AMS663FS01/A36_S6E3FC5_AMS663FS01_PDF.h)
