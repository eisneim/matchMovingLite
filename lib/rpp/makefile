CFLAGS=-O2 -Wall `pkg-config --cflags opencv`
# CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`


demo: Rpoly.o RPP.o demo.o
	g++ $(CFLAGS) Rpoly.o RPP.o demo.o $(LIBS) -o demo

Rpoly.o: Rpoly.cpp
	g++ $(CFLAGS) -c Rpoly.cpp

RPP.o: RPP.cpp
	g++ $(CFLAGS) -c RPP.cpp

demo.o: demo.cpp
	g++ $(CFLAGS) -c demo.cpp

clean:
	rm -f *.o demo

