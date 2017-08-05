HEADERS = packet.h util.h
OBJECTS = util.o alsa_audio.o
CXXFLAGS = --std=c++11 -O3

default: all

all: audio_server audio_client

%.o: %.cpp $(HEADERS)
	g++ $(CXXFLAGS) -c $< -o $@

audio_server: $(OBJECTS) audio_server.o
	g++ $(CXXFLAGS) $(OBJECTS) audio_server.o -lopus -lpthread -lasound -o $@

audio_client.o: audio_client.cpp linear_resampler.h
	g++ $(CXXFLAGS) -c audio_client.cpp -o $@

audio_client: $(OBJECTS) audio_client.o
	g++ $(CXXFLAGS) $(OBJECTS) audio_client.o -lopus -lasound -o $@

clean:
	rm *.o
	rm audio_server audio_client

