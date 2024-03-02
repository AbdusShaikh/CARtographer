CC = g++
CFLAGS = -Wall -Wextra -Iinc -Ilib/rplidar/inc -g
LDFLAGS = -Llib/rplidar -l:libsl_lidar_sdk.a -lpigpio -lopencv_core -lopencv_highgui -lopencv_imgproc

SRC_DIR = src
INC_DIR = inc

SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.cpp=%.o)
EXECUTABLE = slam_car

.PHONY: all clean

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS)