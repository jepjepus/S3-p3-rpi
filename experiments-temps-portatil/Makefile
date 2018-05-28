CC=gcc
LIBS=-lm -lrt

ODIR=.

BIN=./bin
SOURCE=.

PROG=experiment0-usleep experiment0-nanosleep experiment1-nanosleep experiment2-nanosleep
LIST=$(addprefix $(BIN)/, $(PROG))

dir_guard=@mkdir -p $(@D)

.PHONY: all

all: $(LIST)

$(BIN)/%: $(SOURCE)/%.c
	$(dir_guard)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm -f $(ODIR)/*.o *~ $(BIN)/* 
