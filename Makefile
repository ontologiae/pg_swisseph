EXTENSION  = swisseph_planets
MODULE_big = swisseph_planets
OBJS       = swisseph_planets.o

# Fichier SQL à installer dans share/extension/
DATA       = $(EXTENSION)--1.0.sql

PG_CONFIG ?= /usr/lib/postgresql/15/bin/pg_config
PGXS       := $(shell $(PG_CONFIG) --pgxs)
include $(PGXS)

CFLAGS     += -I/usr/include
SHLIB_LINK += -lswe -lm
