CC = gcc

CFLAGS = -Wall -Wextra -O2 -D USE_DEV_LIB -Ilib
LIBS = -lpigpio -lrt -lpthread -lm

TARGET_AI = motor_ai
TARGET_MOTOR = line_motor
TARGET_MAIN = latest_ai_motor
DIST = dist
LIBDIR = lib

# Motor control driven by AI vision feedback via named pipe
SRCS_AI = ai_camera.c \
          MotorDriver.c \
          $(LIBDIR)/DEV_Config.c \
          $(LIBDIR)/PCA9685.c \
          $(LIBDIR)/dev_hardware_i2c.c \
          $(LIBDIR)/sysfs_gpio.c

SRCS_MOTOR = line_moto_main.c \
			MotorDriver.c \
          $(LIBDIR)/DEV_Config.c \
          $(LIBDIR)/PCA9685.c \
          $(LIBDIR)/dev_hardware_i2c.c \
          $(LIBDIR)/sysfs_gpio.c

SRCS_MAIN = latest_ai_motor.c \
			MotorDriver.c \
		  $(LIBDIR)/DEV_Config.c \
		  $(LIBDIR)/PCA9685.c \
		  $(LIBDIR)/dev_hardware_i2c.c \
		  $(LIBDIR)/sysfs_gpio.c

		  
OBJS = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS)))
OBJS_CONCURRENT = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_CONCURRENT)))
OBJS_AI = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_AI)))
OBJS_MOTOR = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_MOTOR)))
OBJS_MAIN = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_MAIN)))

all: dirs $(TARGET_AI) $(TARGET_MOTOR) $(TARGET_MAIN)

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

$(TARGET_MOTOR): $(OBJS_MOTOR)
	$(CC) $(OBJS_MOTOR) -o $(TARGET_MOTOR) $(LIBS)

$(TARGET_MAIN): $(OBJS_MAIN)
	$(CC) $(OBJS_MAIN) -o $(TARGET_MAIN) $(LIBS)

$(DIST)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(DIST)/%.o: $(LIBDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(DIST) $(TARGET_AI) $(TARGET_MOTOR) $(TARGET_MAIN)
