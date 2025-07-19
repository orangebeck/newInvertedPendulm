CC = gcc
CFLAGS = -Wall -g -Iinclude -Wno-unused-variable -DDEBUG_MODE
ifeq ($(TARGET_ARCH), arm)
    CC = arm-linux-gnueabihf-gcc
	CFLAGS = -Wall -g -Iinclude -Wno-unused-variable
endif

LDFLAGS = -lpthread -lrt
TARGET = bin/app
SRC = $(wildcard src/*.c)
OBJ = $(patsubst src/%, build/%, $(SRC:.c=.o))

all: $(TARGET)
ifeq ($(TARGET_ARCH), arm)
	cp ./$(TARGET) ~/workdir/
endif
	
$(TARGET): $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

build/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@ $(LDFLAGS)

clean:
	rm -rf bin/* build/* ./*.csv