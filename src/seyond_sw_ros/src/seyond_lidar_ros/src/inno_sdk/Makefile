###
#  Copyright (C) 2023 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
.PHONY: build all version src apps tarball_public tarball_internal clean

RM ?= rm -rf

sinclude src/sdk_common/version_gen.mk
VER=$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD).$(shell date +%Y%m%d%H%M%S)$(DYNA_LIB_DEV)

build: src apps

all: build

version:
	printf $(VER)"\n" > SDK_VERSION

src:
	$(MAKE) -C $@ all

apps: src
	$(MAKE) -C $@ all

clean:
	$(RM) SDK_VERSION
	$(MAKE) -C src clean
	$(MAKE) -C apps clean

export CUSTOMER_RELEASE
export TARBALL_PUBLIC
export TARBALL_INTERNAL

tarball_public: CUSTOMER_RELEASE = ON
tarball_public: TARBALL_PUBLIC = YES
tarball_public: TARBALL_INTERNAL = NO
tarball_public: build
tarball_public: version
	./build/tarball.bash

tarball_internal: CUSTOMER_RELEASE = OFF
tarball_internal: TARBALL_PUBLIC = NO
tarball_internal: TARBALL_INTERNAL = YES
tarball_internal: build
tarball_internal: version
	./build/tarball.bash
