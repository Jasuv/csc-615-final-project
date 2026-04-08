CC = gcc

CFLAGS = -Wall -Wextra -O2 -D USE_DEV_LIB -Ilib
LIBS = -lpigpio -lrt -lpthread -lm

TARGET_AI = motor_ai
DIST = dist
LIBDIR = lib

#Motor control driven by AI vision feedback via named pipe
SRCS_AI = main_ai_integrated.c \
          MotorDriver.c \
          $(LIBDIR)/DEV_Config.c \
          $(LIBDIR)/PCA9685.c \
          $(LIBDIR)/dev_hardware_i2c.c \
          $(LIBDIR)/sysfs_gpio.c

OBJS = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS)))
OBJS_CONCURRENT = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_CONCURRENT)))
OBJS_AI = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_AI)))

all: dirs $(TARGET) $(TARGET_CONCURRENT) $(TARGET_AI)

run: motor_ai
	sudo ./motor_ai

dirs:
	mkdir -p $(DIST)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LIBS)

$(TARGET_CONCURRENT): $(OBJS_CONCURRENT)
	$(CC) $(OBJS_CONCURRENT) -o $(TARGET_CONCURRENT) $(LIBS)

$(TARGET_AI): $(OBJS_AI)
	$(CC) $(OBJS_AI) -o $(TARGET_AI) $(LIBS)

$(DIST)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(DIST)/%.o: $(LIBDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(DIST) $(TARGET) $(TARGET_CONCURRENT) $(TARGET_AI)
