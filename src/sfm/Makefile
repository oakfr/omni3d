##
## Makefile for the SFM code.
##

CC = gcc
CFLAGS = -O2

SRCS = LocalNewton.c quaternions.c LieCalculus.c polygon.c \
       algorithm.c  noise.c utils.c stageAB.c stageCD.c \
       LogFile.c check_solns.c sfm.c 

ALGORITHM_MODULES = LocalNewton.o quaternions.o LieCalculus.o \
	            algorithm.o utils.o stageAB.o stageCD.o \
	            LogFile.o check_solns.o polygon.o noise.o

#
# Library of program modules
#

algorithm.a : $(ALGORITHM_MODULES)
	rm -f algorithm.a
	ar cq algorithm.a $(ALGORITHM_MODULES)
	ranlib algorithm.a

## N.B. The ranlib command is only needed on Suns running SunOS 4.1.3
## and earlier.

#
# Executables : sfm, calibrate_edges and expt7
#

sfm : sfm.o algorithm.a
	$(CC) -o $@ sfm.o algorithm.a -lm

expt7 : expt7.o algorithm.a LH88.o mat3x3.o
	$(CC) -o $@ expt7.o algorithm.a LH88.o mat3x3.o \
	LINPACK-EISPACK-ROUTINES/svd.a LINPACK-EISPACK-ROUTINES/ev.a -lm

depend :
	makedepend $(SRCS)

