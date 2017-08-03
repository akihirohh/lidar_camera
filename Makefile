TARGET = lidarExample

SRC := src
INCLUDE = -Iinclude -I/usr/local/include/urg_cpp
CC = g++ -std=c++11
CFLAGS = -Wno-deprecated-declarations -fpermissive `pkg-config --cflags opencv`
LIBS = -pthread -lrt /usr/local/lib/liburg_cpp.a `pkg-config --libs opencv` 
AUX = $(SRC)/*.cpp

all: $(TARGET)

$(TARGET): $(TARGET).cpp $(AUX)
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).cpp $(AUX) $(INCLUDE) $(LIBS) 
	
clean:
	$(RM) $(TARGET)
