EXTENSION  = swisseph_planets
MODULE_big = swisseph_planets
OBJS       = swisseph_planets.o

# Fichier SQL à installer
DATA       = $(EXTENSION)--1.0.sql

# Répertoire cible pour les fichiers .se1 (modifiable via make EPHEMERIDES_DIR=/chemin)
EPHEMERIDES_DIR ?= /usr/share/libswe/ephe
EPHEMERIDES_FILES = $(wildcard *.se1)

PG_CONFIG ?= /usr/lib/postgresql/15/bin/pg_config
PGXS       := $(shell $(PG_CONFIG) --pgxs)
include $(PGXS)

# Inclure le chemin dans le code C
CFLAGS     += -I/usr/include -DSE_EPHE_PATH="\"$(EPHEMERIDES_DIR)\""
SHLIB_LINK += -lswe -lm

# Cible d'installation personnalisée
install: all install-data

install-data:
	install -d $(DESTDIR)$(EPHEMERIDES_DIR)
	install -m 644 $(EPHEMERIDES_FILES) $(DESTDIR)$(EPHEMERIDES_DIR)



