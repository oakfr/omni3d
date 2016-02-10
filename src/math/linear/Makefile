#
# This makefile will produce the svd.a and ev.a archives
#

CC = gcc
CFLAGS = -O2

SVD_OBJS = daxpy.o ddot.o dnrm2.o drot.o drotg.o dscal.o dsvdc.o dswap.o \
           d_sign.o svd.o

EV_OBJS = pythag.o rs.o tql1.o tql2.o tred1.o tred2.o d_sign.o ev.o

svd.a : $(SVD_OBJS)
	rm -f $@
	ar cq $@ $(SVD_OBJS)
	ranlib $@

ev.a : $(EV_OBJS)
	rm -f $@
	ar cq $@ $(EV_OBJS)
	ranlib $@

## N.B. The ranlib command is only needed on Suns running SunOS 4.1.3
## and earlier.
