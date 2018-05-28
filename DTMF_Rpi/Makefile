CC=gcc
LIBS=-lm -lrt

ODIR=.
BIN=./bin
SOURCE=.

PROG=main
LIST=$(addprefix $(BIN)/, $(PROG))

dir_guard=@mkdir -p $(@D)

.PHONY: all

all: $(LIST)

$(BIN)/%: $(SOURCE)/%.c
	$(dir_guard)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm -f $(ODIR)/*.o *~  $(BIN)/*
