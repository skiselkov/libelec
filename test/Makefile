# CONFIDENTIAL
#
# Copyright 2018 Saso Kiselkov. All rights reserved.
#
# NOTICE:  All information contained herein is, and remains the property
# of Saso Kiselkov. The intellectual and technical concepts contained
# herein are proprietary to Saso Kiselkov and may be covered by U.S. and
# Foreign Patents, patents in process, and are protected by trade secret
# or copyright law. Dissemination of this information or reproduction of
# this material is strictly forbidden unless prior written permission is
# obtained from Saso Kiselkov.

ACFUTILS=/home/diablos/NextDev/libacfutils

CFLAGS = -D_GNU_SOURCE -DDEBUG -g -O0 -DIBM=0 -DAPL=0 -DLIN=1 \
    -DLIBELEC_SLOW_DEBUG \
    -W -Wall -Werror -Wno-missing-field-initializers \
    -I../src -I$(ACFUTILS)/src \
    $(shell $(ACFUTILS)/pkg-config-deps linux-64 --cflags)
LDFLAGS = -L$(ACFUTILS)/qmake/lin64 -lacfutils \
     $(shell $(ACFUTILS)/pkg-config-deps linux-64 --libs) \
     -lm -lpthread -lstdc++

OBJS = test.o ../src/libelec.o

all : test

test : $(OBJS)
	$(CC) $^ $(LDFLAGS) -o $@

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

clean :
	rm -f $(OBJS) test
