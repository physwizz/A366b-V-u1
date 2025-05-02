XXD := /usr/bin/xxd
SED := /bin/sed

#Translate .dat file to .h to cover the case which can not use request_firmware(Recovery Mode)
CLEAR_TMP := $(shell rm -f A36_ANA6710_AMS663FS02_PDF_DATA)
CLEAR_CURRENT := $(shell rm -f $(DISPLAY_BLD_DIR)/msm/samsung/A36_ANA6710_AMS663FS02/A36_ANA6710_AMS663FS02_PDF.h)
COPY_TO_HERE := $(shell cp -vf $(DISPLAY_BLD_DIR)/msm/samsung/panel_data_file/A36_ANA6710_AMS663FS02.dat A36_ANA6710_AMS663FS02_PDF_DATA)
DATA_TO_HEX := $(shell $(XXD) -i A36_ANA6710_AMS663FS02_PDF_DATA > $(DISPLAY_BLD_DIR)/msm/samsung/A36_ANA6710_AMS663FS02/A36_ANA6710_AMS663FS02_PDF.h)
ADD_NULL_CHR := $(shell $(SED) -i -e 's/\([0-9a-f]\)$$/\0, 0x00/' $(DISPLAY_BLD_DIR)/msm/samsung/A36_ANA6710_AMS663FS02/A36_ANA6710_AMS663FS02_PDF.h)
