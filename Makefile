# ====================================================================
# Makefile fuer simplephys
#
#   make lib         Statische Bibliothek lib/libphysics.a bauen
#   make             Bibliothek + alle Apps bauen (Alias: make apps)
#   make apps        Alle Apps bauen und linken
#   make bin/<name>  Einzelne App bauen/linken (z. B. make bin/beispiel)
#   make run APP=x   App x bauen und starten (x aus: $(APP_NAMES))
#   make run-x       Kurzform zum Starten (z. B. make run-beispiel)
#   make clean       Build-Artefakte loeschen
#
# Verzeichnisse:
#   lib/src     Quellen (*.cpp) der Physik-Engine
#   lib/header  Header (*.hpp) der Engine + raylib-Header
#   lib         Ziel der statischen Bibliothek (libphysics.a) + raylib
#   apps        Quellen der Anwendungen (je App ein Unterordner mit main.cpp)
#   build       Objektdateien
#   bin         Fertige Programme
# ====================================================================

CXX      ?= g++
AR       ?= ar

# -MMD -MP erzeugt je Objektdatei eine .d-Datei mit den Header-Abhaengigkeiten.
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -MMD -MP
CPPFLAGS += -Ilib/header

# raylib liegt als statische Bibliothek in lib/ (libraylib.a); dazu die
# noetigen Systembibliotheken fuer den Desktop-OpenGL-Build.
LDLIBS := -Llib -lraylib -lGL -lm -lpthread -ldl -lrt -lX11

BUILD := build
BIN   := bin
LIB   := lib/libphysics.a

# Objektdateien nicht als Zwischendateien loeschen (sonst Rebuild jede Runde).
.SECONDARY:

# --- Bibliothek -----------------------------------------------------
LIB_SRCS := $(wildcard lib/src/*.cpp)
LIB_OBJS := $(patsubst lib/src/%.cpp,$(BUILD)/lib/%.o,$(LIB_SRCS))

# --- Apps -----------------------------------------------------------
# Jede App ist ein Unterordner apps/<name>/ und darf mehrere .cpp enthalten;
# alle werden gebaut und zusammen gelinkt.
APP_NAMES := beispiel
APP_BINS  := $(addprefix $(BIN)/,$(APP_NAMES))
APP_SRCS  := $(wildcard apps/*/*.cpp)
APP_OBJS  := $(patsubst apps/%.cpp,$(BUILD)/apps/%.o,$(APP_SRCS))

# --- Header-Abhaengigkeiten -----------------------------------------
DEPS := $(LIB_OBJS:.o=.d) $(APP_OBJS:.o=.d)

.PHONY: all apps lib clean run $(addprefix run-,$(APP_NAMES))

all: apps

apps: $(APP_BINS)

lib: $(LIB)

# --- Objektdateien der Bibliothek -----------------------------------
$(BUILD)/lib/%.o: lib/src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

# --- Statische Bibliothek -------------------------------------------
$(LIB): $(LIB_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

# --- Objektdateien der Apps -----------------------------------------
$(BUILD)/apps/%.o: apps/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

# --- Linken ---------------------------------------------------------
# Pro App eine explizite Regel: Objektdateien aller Quellen der App zusammen
# mit der Bibliothek und raylib linken.
define APP_LINK_RULE
$(BIN)/$(1): $$(patsubst apps/%.cpp,$(BUILD)/apps/%.o,$$(wildcard apps/$(1)/*.cpp)) $(LIB)
	@mkdir -p $$(dir $$@)
	$$(CXX) $$^ -o $$@ $$(LDLIBS)
endef

$(foreach app,$(APP_NAMES),$(eval $(call APP_LINK_RULE,$(app))))

# --- Starten --------------------------------------------------------
# make run APP=<name>
run:
	@test -n "$(APP)" || { \
		echo "Fehler: APP fehlt."; \
		echo "Aufruf: make run APP=<name>   (verfuegbar: $(APP_NAMES))"; \
		exit 2; \
	}
	@$(MAKE) --no-print-directory $(BIN)/$(APP)
	@$(BIN)/$(APP)

# make run-<name>  (Kurzform)
$(addprefix run-,$(APP_NAMES)): run-%: $(BIN)/%
	$(BIN)/$*

# --- Aufraeumen -----------------------------------------------------
clean:
	rm -rf $(BUILD) $(BIN) $(LIB)

# Eingebundene Header-Abhaengigkeiten (fehlende .d-Dateien werden ignoriert).
-include $(DEPS)
