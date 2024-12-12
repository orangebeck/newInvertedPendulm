CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -g -Iinclude -Wno-unused-variable
LDFLAGS = -lpthread -lrt
TARGET = bin/app
SRC = $(wildcard src/**/*.c)
OBJ = $(patsubst src/%, bin/%, $(SRC:.c=.o))

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

bin/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@ $(LDFLAGS)

clean:
	rm -rf bin/*