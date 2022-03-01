TARGET = Proteus
FIRMWARE = fehproteusfirmware

all:
	make -C $(FIRMWARE) all TARGET=$(TARGET)

clean:
	make -C $(FIRMWARE) clean TARGET=$(TARGET)

run:
	make -C $(FIRMWARE) run TARGET=$(TARGET)