all:
	gcc -I../../include -c PSOPID.c -o PSOPID.o
	gcc -I../../include -c pid_core.c -o pid_core.o
	gcc -I../../include -o main PSOPID.o pid_core.o

clean:
	rm *.o main