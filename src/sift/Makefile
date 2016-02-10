
# ------------------ Compilation options ------------------------

# Loads math library.  
LIBS = -lm

# Flags for the C compiler:
#   -Wall for strict gcc warnings (requires prototypes for all functions).
#   -g to produce debug data for gdb
#   -O for optimization
CFLAGS = -Wall -g -O

CC = gcc

# --------------------- Code modules ----------------------------

# Object files
OBJ = main.o key.o util.o

# Definitions
DEFS = key.h

# ------------------------ Rules --------------------------------

keypoints: ${OBJ}
	${CC} -o $@ ${CFLAGS} ${OBJ} ${LIBS}

# Implicit rule used by Gnu Make: $(CC) -c $(CPPFLAGS) $(CFLAGS)
${OBJ}: ${DEFS}
