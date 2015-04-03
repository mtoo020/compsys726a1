# Makefile for player c++ clients

#Specify your target and your cc source file name below
TARGET = pioneer
SRC = pioneer.cc

# Pick up the necessary options, directories and options for compiling and linking with player 
# Add a warning on all errors, and -g3 for debugging information 
CPPFLAGS = `pkg-config --cflags playerc++` -Wall -g3
LDFLAGS = `pkg-config --libs playerc++` -lpthread -lGL -lGLU -lglut

#Target and command below builds the target output executable file
$(TARGET): $(SRC)
	make clean
	g++ $(SRC) -o $(TARGET) $(CPPFLAGS) $(LDFLAGS) -std=c++11

clean:
	rm -f $(TARGET) *.o