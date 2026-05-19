CC = gcc
CFLAGS = -Wall -Wextra -O2 -D USE_DEV_LIB -I$(LIBDIR)
DISTDIR = dist
LIBDIR = lib
LIBS = -lpigpio -lrt -lpthread -lm

TARGET = main
SRCS =  main.c \
        line_controller.c \
        MotorDriver.c \
        $(LIBDIR)/DEV_Config.c \
        $(LIBDIR)/PCA9685.c \
        $(LIBDIR)/dev_hardware_i2c.c \
        $(LIBDIR)/sysfs_gpio.c \
        $(LIBDIR)/ColorLib.c
OBJS = $(patsubst %.c,$(DISTDIR)/%.o,$(notdir $(SRCS)))

all: $(DISTDIR) $(TARGET)

run: $(TARGET)
	sudo ./$(TARGET)

$(DISTDIR):
	mkdir -p $(DISTDIR)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

$(DISTDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(DISTDIR)/%.o: $(LIBDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(DISTDIR) $(TARGET)
