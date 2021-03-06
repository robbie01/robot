TARGET = Proteus
FIRMWARE = fehproteusfirmware

all:
ifeq ($(OS), Windows_NT)
	mingw32-make -C $(FIRMWARE) all TARGET=$(TARGET)
	mingw32-make -C $(FIRMWARE) deploy TARGET=$(TARGET)
else
	make -C $(FIRMWARE) all TARGET=$(TARGET)
	@echo
	@./copy_to_sd.sh
endif

clean:
ifeq ($(OS), Windows_NT)
	mingw32-make -C $(FIRMWARE) clean TARGET=$(TARGET)
else
	make -C $(FIRMWARE) clean TARGET=$(TARGET)
endif

run:
ifeq ($(OS), Windows_NT)
	mingw32-make -C $(FIRMWARE) run TARGET=$(TARGET)
else
	make -C $(FIRMWARE) run TARGET=$(TARGET)
endif