#@Makefile for VANET Simulator

OS_TYPE=-D_LINUX_
#CC= gcc
#CC= g++ -Wno-write-strings
CC= g++ -Wno-write-strings -Wno-format-security

#PG_FLAG=-pg
PG_FLAG=

GSL_INCLUDE= -I ~/usr/include
#GSL_INCLUDE= -I /home/bc1/jeongjh/usr/include
#GSL_INCLUDE=

GSL_LIBRARY= -L ~/usr/lib
#GSL_LIBRARY= -L /home/bc1/jeongjh/usr/lib
#GSL_LIBRARY=

GSL_SOURCE = gsl-util.c
GSL_OBJECT = gsl-util.o
GSL_DEBUG_OBJECT = gsl-util.do

CFLAG1= -c -O $(PG_FLAG)  $(GSL_INCLUDE)
CFLAG2= -c -g $(PG_FLAG) $(GSL_INCLUDE)
CFLAG3= -o $@ $(OS_TYPE) $(PG_FLAG)
CFLAG4= -o $@ $(PG_FLAG) $(GSL_LIBRARY)
CFLAG5= -o $@ -g $(PG_FLAG) $(GSL_LIBRARY)

#etags: Exuberant ctags
ETAGS=etags

CTAGS=ctags

#Release version
all: vanet

vanet: main.o all-pairs-shortest-paths.o heap.o mst.o param.o queue.o quick-sort.o rand.o random-path.o schedule.o shortest-path.o smpl.o util.o vadd.o linear-algebra.o access-point-model.o mobility.o tpd.o epidemic.o $(GSL_OBJECT)
#	$(CC) $(CFLAG4) main.o all-pairs-shortest-paths.o heap.o mst.o param.o queue.o quick-sort.o rand.o random-path.o schedule.o shortest-path.o smpl.o util.o vadd.o linear-algebra.o access-point-model.o mobility.o -lm
	$(CC) $(CFLAG4) main.o all-pairs-shortest-paths.o heap.o mst.o param.o queue.o quick-sort.o rand.o random-path.o schedule.o shortest-path.o smpl.o util.o vadd.o linear-algebra.o access-point-model.o mobility.o tpd.o epidemic.o $(GSL_OBJECT) -lm -lgsl -lgslcblas

main.o: main.c
	$(CC) $(CFLAG3) $(CFLAG1) main.c

all-pairs-shortest-paths.o: all-pairs-shortest-paths.c
	$(CC) $(CFLAG3) $(CFLAG1) all-pairs-shortest-paths.c

heap.o: heap.c
	$(CC) $(CFLAG3) $(CFLAG1) heap.c

mst.o: mst.c
	$(CC) $(CFLAG3) $(CFLAG1) mst.c

param.o: param.c
	$(CC) $(CFLAG3) $(CFLAG1) param.c

queue.o: queue.c
	$(CC) $(CFLAG3) $(CFLAG1) queue.c

quick-sort.o: quick-sort.c
	$(CC) $(CFLAG3) $(CFLAG1) quick-sort.c

rand.o: rand.c
	$(CC) $(CFLAG3) $(CFLAG1) rand.c

random-path.o: random-path.c
	$(CC) $(CFLAG3) $(CFLAG1) random-path.c

schedule.o: schedule.c
	$(CC) $(CFLAG3) $(CFLAG1) schedule.c

shortest-path.o: shortest-path.c
	$(CC) $(CFLAG3) $(CFLAG1) shortest-path.c

smpl.o: smpl.c
	$(CC) $(CFLAG3) $(CFLAG1) smpl.c

util.o: util.c
	$(CC) $(CFLAG3) $(CFLAG1) util.c

vadd.o: vadd.c
	$(CC) $(CFLAG3) $(CFLAG1) vadd.c

linear-algebra.o: linear-algebra.c
	$(CC) $(CFLAG3) $(CFLAG1) linear-algebra.c

access-point-model.o: access-point-model.c
	$(CC) $(CFLAG3) $(CFLAG1) access-point-model.c

mobility.o: mobility.c
	$(CC) $(CFLAG3) $(CFLAG1) mobility.c

gsl-util.o: gsl-util.c
	$(CC) $(CFLAG3) $(CFLAG1) gsl-util.c 

tpd.o: tpd.c
	$(CC) $(CFLAG3) $(CFLAG1) tpd.c

epidemic.o: epidemic.c
	$(CC) $(CFLAG3) $(CFLAG1) epidemic.c

#Debugging version
all.db: vanet.db
 
vanet.db: main.do all-pairs-shortest-paths.do heap.do mst.do param.do queue.do quick-sort.do rand.do random-path.do schedule.do shortest-path.do smpl.do util.do vadd.do linear-algebra.do access-point-model.do mobility.do tpd.do epidemic.do $(GSL_DEBUG_OBJECT)
#	$(CC) $(CFLAG5) main.do all-pairs-shortest-paths.do heap.do mst.do param.do queue.do quick-sort.do rand.do random-path.do schedule.do shortest-path.do smpl.do util.do vadd.do linear-algebra.do access-point-model.do mobility.do -lm
	$(CC) $(CFLAG5) main.do all-pairs-shortest-paths.do heap.do mst.do param.do queue.do quick-sort.do rand.do random-path.do schedule.do shortest-path.do smpl.do util.do vadd.do linear-algebra.do access-point-model.do mobility.do tpd.do epidemic.do $(GSL_DEBUG_OBJECT) -lm -lgsl -lgslcblas

main.do: main.c
	$(CC) $(CFLAG3) $(CFLAG2) main.c

all-pairs-shortest-paths.do: all-pairs-shortest-paths.c
	$(CC) $(CFLAG3) $(CFLAG2) all-pairs-shortest-paths.c

heap.do: heap.c
	$(CC) $(CFLAG3) $(CFLAG2) heap.c

mst.do: mst.c
	$(CC) $(CFLAG3) $(CFLAG2) mst.c

param.do: param.c
	$(CC) $(CFLAG3) $(CFLAG2) param.c

queue.do: queue.c
	$(CC) $(CFLAG3) $(CFLAG2) queue.c

quick-sort.do: quick-sort.c
	$(CC) $(CFLAG3) $(CFLAG2) quick-sort.c

rand.do: rand.c
	$(CC) $(CFLAG3) $(CFLAG2) rand.c

random-path.do: random-path.c
	$(CC) $(CFLAG3) $(CFLAG2) random-path.c

schedule.do: schedule.c
	$(CC) $(CFLAG3) $(CFLAG2) schedule.c

shortest-path.do: shortest-path.c
	$(CC) $(CFLAG3) $(CFLAG2) shortest-path.c

smpl.do: smpl.c
	$(CC) $(CFLAG3) $(CFLAG2) smpl.c

util.do: util.c
	$(CC) $(CFLAG3) $(CFLAG2) util.c

vadd.do: vadd.c
	$(CC) $(CFLAG3) $(CFLAG2) vadd.c

linear-algebra.do: linear-algebra.c
	$(CC) $(CFLAG3) $(CFLAG2) linear-algebra.c

access-point-model.do: access-point-model.c
	$(CC) $(CFLAG3) $(CFLAG2) access-point-model.c

mobility.do: mobility.c
	$(CC) $(CFLAG3) $(CFLAG2) mobility.c

gsl-util.do: gsl-util.c
	$(CC) $(CFLAG3) $(CFLAG2) gsl-util.c 

tpd.do: tpd.c
	$(CC) $(CFLAG3) $(CFLAG2) tpd.c 

epidemic.do: epidemic.c
	$(CC) $(CFLAG3) $(CFLAG2) epidemic.c 

#Clean binary and object files
clean:
	rm -f *.o *.do *~ vanet vanet.db .nfs* 1
	rm -r ./output
	mkdir output 
	rm -f TASK.sh submit_task 
	rm -f xa* task-xa* *.e* *.o*
	rm -f tags
	rm -f vehicle-trajectory.txt

#etags operation
etag:
	rm -f TAGS
#   $(ETAGS) --append --language-force=c *.c *.h $(K_SHORTEST_PATHS)/*.cpp $(K_SHORTEST_PATHS)/*.h
	$(ETAGS) --append --lang=c --declarations *.c *.h 
#   find . -o -name '*.cc' -o -name '*.h' -o -name '*.c' -print0 \
#| xargs $(ETAGS) * --append

ctag:
	rm -f tags
	$(CTAGS) --append --language-force=c *.c *.h *.cpp
#   $(CTAGS) --append --lang=c --declarations *.c *.h $(K_SHORTEST_PATHS)/*.cpp $(K_SHORTEST_PATHS)/*.h



