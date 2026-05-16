CC = gcc
CXX = g++

DIST = dist
LIBDIR = lib

# Compilation flags
CFLAGS = -Wall -Wextra -O2 -D USE_DEV_LIB -I$(LIBDIR)
CXXFLAGS = -Wall -Wextra -O2 $(shell pkg-config --cflags opencv4) -I$(LIBDIR)

# Libraries
LIBS = -lpigpio -lrt -lpthread -lm
OPENCV_LIBS = $(shell pkg-config --libs opencv4)

# Targets
TARGET_AI = motor_ai
TARGET_MOTOR = line_motor
TARGET_MAIN = latest_ai_motor

# --- 1. AI Motor (Mix of C++ and C) ---
SRCS_AI_CPP = ai_camera.cpp
SRCS_AI_C = MotorDriver.c \
            $(LIBDIR)/DEV_Config.c \
            $(LIBDIR)/PCA9685.c \
            $(LIBDIR)/dev_hardware_i2c.c \
            $(LIBDIR)/sysfs_gpio.c

# Process C++ and C files into separate object lists, then combine them
OBJS_AI = $(patsubst %.cpp,$(DIST)/%.o,$(notdir $(SRCS_AI_CPP))) \
          $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_AI_C)))


# --- 2. Line Motor (Pure C) ---
SRCS_MOTOR = line_moto_main.c \
             MotorDriver.c \
             $(LIBDIR)/DEV_Config.c \
             $(LIBDIR)/PCA9685.c \
             $(LIBDIR)/dev_hardware_i2c.c \
             $(LIBDIR)/sysfs_gpio.c

OBJS_MOTOR = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_MOTOR)))


# --- 3. Main Motor (Pure C) ---
SRCS_MAIN = main.c \
            camera_control.c \
            line_control.c \
            MotorDriver.c \
            $(LIBDIR)/DEV_Config.c \
            $(LIBDIR)/PCA9685.c \
            $(LIBDIR)/dev_hardware_i2c.c \
            $(LIBDIR)/sysfs_gpio.c \
            $(LIBDIR)/ColorLib.c

OBJS_MAIN = $(patsubst %.c,$(DIST)/%.o,$(notdir $(SRCS_MAIN)))

# Tell Make where to search for source files
vpath %.c . $(LIBDIR)
vpath %.cpp .

.PHONY: all clean run

all: $(TARGET_AI) $(TARGET_MOTOR) $(TARGET_MAIN)

run: $(TARGET_AI)
	sudo ./$(TARGET_AI)

# Output directory creation
$(DIST):
	mkdir -p $(DIST)

# Executable linking
# TARGET_AI must be linked with $(CXX) because it includes C++ OpenCV code
$(TARGET_AI): $(OBJS_AI)
	$(CXX) $^ -o $@ $(LIBS) $(OPENCV_LIBS)

$(TARGET_MOTOR): $(OBJS_MOTOR)
	$(CC) $^ -o $@ $(LIBS)

$(TARGET_MAIN): $(OBJS_MAIN)
	$(CC) $^ -o $@ $(LIBS)

# Object file compilation rules
$(DIST)/%.o: %.c | $(DIST)
	$(CC) $(CFLAGS) -c $< -o $@

$(DIST)/%.o: %.cpp | $(DIST)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(DIST) $(TARGET_AI) $(TARGET_MOTOR) $(TARGET_MAIN)
