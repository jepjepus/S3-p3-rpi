CC=gcc
LIBS=-lm -lrt

ODIR=.
BIN=./bin
SOURCE=.

PROG=main
LIST=$(addprefix $(BIN)/, $(PROG))

.PHONY: all

all: $(LIST)

$(BIN)/%: $(SOURCE)/%.c
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm -f $(ODIR)/*.o *~  $(BIN)/*
