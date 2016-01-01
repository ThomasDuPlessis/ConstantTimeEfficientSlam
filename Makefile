
CC          := g++
MAKE        := make
CFLAGS      := -Wall -lm -g -std=c++11
OPEN_CV     := `pkg-config opencv --cflags --libs`
MRPT        := -lmrpt-base -lmrpt-opengl -lmrpt-gui -lmrpt-slam -lmrpt-tfest
LIB_FLAGS   := $(OPEN_CV) $(MRPT) -Llib/lib
INCLUDE     := -I../../lib/include
S           := src
B           := bin
OUT         := SLAM

.PHONY: all
all: $(B)/Main.o
	$(CC) $(CFLAGS) $(B)/Slam.o $(B)/KeyFrameState.o $(B)/ObservationSet.o \
		$(B)/Utilities.o $(B)/StereoRig.o $(B)/Main.o -o $(OUT) $(LIB_FLAGS)

$(B)/Main.o: $(B)/Slam.o $(S)/Main.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $(S)/Main.cpp -o $(B)/Main.o

$(B)/Slam.o: $(B)/KeyFrameState.o $(B)/StereoRig.o $(S)/Slam.h $(S)/Slam.cpp $(S)/Types.h \
		$(S)/SearchSpaceConstructor.h
	$(CC) $(CFLAGS) $(INCLUDE) -c $(S)/Slam.cpp -o $(B)/Slam.o

$(B)/KeyFrameState.o: $(B)/ObservationSet.o $(B)/StereoRig.o $(B)/Utilities.o $(S)/KeyFrameState.h \
		$(S)/KeyFrameState.cpp $(S)/Types.h $(S)/Observation.h $(S)/LandmarkSet.h \
		$(S)/SearchSpaceConstructor.h
	$(CC) $(CFLAGS) $(INCLUDE) -c $(S)/KeyFrameState.cpp -o $(B)/KeyFrameState.o

$(B)/ObservationSet.o: $(B)/StereoRig.o $(B)/Utilities.o $(S)/ObservationSet.h $(S)/ObservationSet.cpp \
		$(S)/Types.h $(S)/Observation.h 
	$(CC) $(CFLAGS) $(INCLUDE) -c $(S)/ObservationSet.cpp -o $(B)/ObservationSet.o

$(B)/StereoRig.o: $(S)/StereoRig.h $(S)/StereoRig.cpp $(S)/Types.h $(S)/Camera.h
	$(CC) $(CFLAGS) $(INCLUDE) -c $(S)/StereoRig.cpp -o $(B)/StereoRig.o

$(B)/Utilities.o: $(S)/Utilities.h $(S)/Utilities.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $(S)/Utilities.cpp -o $(B)/Utilities.o

clean:
	rm -rf $(B)/* test testvideo
	rm -rf $(OUT)