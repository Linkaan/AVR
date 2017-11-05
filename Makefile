BOARD_TAG    = gert328
BOARDS_TXT   = boards.txt

USER_LIB_PATH += ..
ARDUINO_LIBS = Serializer

ARDMK_VENDOR = archlinux-arduino
ARDUINO_DIR = /usr/share/arduino
include ~/Arduino-Makefile/Arduino.mk
