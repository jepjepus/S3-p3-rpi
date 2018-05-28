CC=gcc
LIBS=-lm -lrt

ODIR=.

BIN=./bin
SOURCE=.

PROG=experiment0-usleep experiment0-nanosleep experiment1-nanosleep experiment2-nanosleep
LIST=$(addprefix $(BIN)/, $(PROG))

.PHONY: all

all: $(LIST)

$(BIN)/%: $(SOURCE)/%.c
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm -f $(ODIR)/*.o *~ $(BIN)/* 
