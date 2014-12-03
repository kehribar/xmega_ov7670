###############################################################################
# Example makefile for openCV projects.
# ihsan@kehribar.me wrote this file.
###############################################################################
CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

TARGET = main

.PHONY: clean

$(TARGET):
	g++ $(TARGET).cpp serial_lib.cpp -o $(TARGET) $(CFLAGS) $(LIBS)

clean:
	rm -f *.o *~ ./$(TARGET) 
