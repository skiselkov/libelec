# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Copyright 2023 Saso Kiselkov. All rights reserved.

ACFUTILS=../../libacfutils

OS := $(shell uname)

ifeq ($(OS),Darwin)
	PLAT_LONG := mac-64
	PLAT_SHORT = mac64
	PLAT_FLAGS := -DIBM=0 -DAPL=1 -DLIN=0
else
	PLAT_LONG := linux-64
	PLAT_SHORT = lin64
	PLAT_FLAGS := -DIBM=0 -DAPL=0 -DLIN=1
endif

CFLAGS = -D_GNU_SOURCE -DDEBUG -g -O0 $(PLAT_FLAGS) -D_LACF_WITHOUT_XPLM \
    -fPIC \
    -W -Wall -Werror -Wno-missing-field-initializers \
    -I../../abus/src -I../src -I$(ACFUTILS)/src \
    $(shell $(ACFUTILS)/pkg-config-deps $(PLAT_LONG) --cflags)
LDFLAGS = -fPIC -L$(ACFUTILS)/qmake/$(PLAT_SHORT) -lacfutils \
     $(shell $(ACFUTILS)/pkg-config-deps $(PLAT_LONG) --libs) \
     -lm -lpthread -lstdc++

OBJS = test.o ../src/libelec.o ../src/libelec_drawing.o

ifeq ($(slow),yes)
    CFLAGS += -DLIBELEC_SLOW_DEBUG
endif
ifeq ($(profile),yes)
    CFLAGS += -pg
    LDFLAGS += -pg
endif

all : test

test : $(OBJS)
	$(CC) $^ $(LDFLAGS) -o $@

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

clean :
	rm -f $(OBJS) test
