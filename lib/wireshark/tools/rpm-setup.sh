#!/bin/bash
# Setup development environment for RPM based systems such as Red Hat, Centos, Fedora, openSUSE
#
# Wireshark - Network traffic analyzer
# By Gerald Combs <gerald@wireshark.org>
# Copyright 1998 Gerald Combs
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# We drag in tools that might not be needed by all users; it's easier
# that way.
#

if [ "$1" = "--help" ]
then
	echo "\nUtility to setup a rpm-based system for Wireshark Development.\n"
	echo "The basic usage installs the needed software\n\n"
	echo "Usage: $0 [--install-optional] [...other options...]\n"
	echo "\t--install-optional: install optional software as well"
	echo "\t--install-rpm-deps: install packages required to build the .rpm file\\n"
	echo "\t[other]: other options are passed as-is to the packet manager\n"
	exit 1
fi

# Check if the user is root
if [ $(id -u) -ne 0 ]
then
	echo "You must be root."
	exit 1
fi

ADDITIONAL=0
RPMDEPS=0
for arg; do
	case $arg in
		--install-optional)
			ADDITIONAL=1
			;;
		--install-rpm-deps)
			RPMDEPS=1
			;;
		*)
			OPTIONS="$OPTIONS $arg"
			;;
	esac
done

BASIC_LIST="cmake \
	gcc \
	gcc-c++ \
	flex \
	python3 \
	perl \
	desktop-file-utils \
	git \
	glib2-devel \
	libpcap-devel \
	zlib-devel \
	libgcrypt-devel"

ADDITIONAL_LIST="libcap-devel \
	libssh-devel \
	krb5-devel \
	perl-Parse-Yapp \
	snappy-devel \
	minizip-devel \
	lz4 \
	libxml2-devel \
	spandsp-devel \
	systemd-devel"

# Uncomment to add PNG compression utilities used by compress-pngs:
# ADDITIONAL_LIST="$ADDITIONAL_LIST \
#	advancecomp \
#	optipng \
#	oxipng \
#	pngcrush"

# XXX
RPMDEPS_LIST="rpm-build"

# Guess which package manager we will use
for PM in zypper dnf yum ''; do
	if type "$PM" >/dev/null 2>&1; then
		break
	fi
done

if [ -z $PM ]
then
	echo "No package managers found, exiting"
	exit 1
fi

case $PM in
	zypper)
		PM_OPT="--non-interactive"
		PM_SEARCH="search -x --provides"
		;;
	dnf)
		PM_SEARCH="info"
		;;
	yum)
		PM_SEARCH="info"
		;;
esac

echo "Using $PM ($PM_SEARCH)"

# Adds package $2 to list variable $1 if the package is found
add_package() {
	local list="$1" pkgname="$2"

	# fail if the package is not known
	$PM $PM_SEARCH "$pkgname" &> /dev/null || return 1

	# package is found, append it to list
	eval "${list}=\"\${${list}} \${pkgname}\""
}

# Adds packages $2-$n to list variable $1 if all the packages are found
add_packages() {
	local list="$1" pkgnames="${@:2}"

	# fail if any package is not known
	for pkgname in $pkgnames; do
		$PM $PM_SEARCH "$pkgname" &> /dev/null || return 1
	done

	# all packages are found, append it to list
	eval "${list}=\"\${${list}} \${pkgnames}\""
}

add_package BASIC_LIST cmake3 || add_package BASIC_LIST cmake ||
echo "cmake is unavailable" >&2

add_package BASIC_LIST glib2 || add_package BASIC_LIST libglib-2_0-0 ||
echo "glib2 is unavailable" >&2

# lua51, lua51-devel: OpenSUSE Leap 42.3 (lua would be fine too, as it installs lua52), OpenSUSE Leap 15.0 (lua installs lua53, so it wouldn't work)
# compat-lua, compat-lua-devel: Fedora 28, Fedora 29, CentOS 8
# lua, lua-devel: CentOS 7
add_package BASIC_LIST lua51-devel || add_package BASIC_LIST compat-lua-devel || add_package BASIC_LIST lua-devel ||
echo "lua devel is unavailable" >&2

add_package BASIC_LIST lua51 || add_package BASIC_LIST compat-lua || add_package BASIC_LIST lua ||
echo "lua is unavailable" >&2

add_package BASIC_LIST libpcap || add_package BASIC_LIST libpcap1 ||
echo "libpcap is unavailable" >&2

add_package BASIC_LIST zlib || add_package BASIC_LIST libz1 ||
echo "zlib is unavailable" >&2

add_package BASIC_LIST c-ares-devel || add_package BASIC_LIST libcares-devel ||
echo "libcares-devel is unavailable" >&2

# qt5-linguist: CentOS, Fedora
# libqt5-linguist-devel: OpenSUSE
add_package BASIC_LIST qt5-linguist ||
add_package BASIC_LIST libqt5-linguist-devel ||
echo "Qt5 linguist is unavailable" >&2

# qt5-qtmultimedia: CentOS, Fedora, pulls in qt5-qtbase-devel (big dependency list!)
# libqt5-qtmultimedia-devel: OpenSUSE, pulls in Core, Gui, Multimedia, Network, Widgets
# OpenSUSE additionally has a separate Qt5PrintSupport package.
add_package BASIC_LIST qt5-qtmultimedia-devel ||
add_packages BASIC_LIST libqt5-qtmultimedia-devel libQt5PrintSupport-devel ||
echo "Qt5 is unavailable" >&2

# This in only required (and available) on OpenSUSE
add_package BASIC_LIST update-desktop-files ||
echo "update-desktop-files is unavailable" >&2

add_package BASIC_LIST perl-podlators ||
echo "perl-podlators unavailable" >&2

# rubygem-asciidoctor.noarch: Centos 7, Fedora
# ruby2.5-rubygem-asciidoctor: openSUSE 15.2
# You will get nothing and you will like it: CentOS 8
add_package RPMDEPS_LIST rubygem-asciidoctor.noarch || add_package RPMDEPS_LIST ruby2.5-rubygem-asciidoctor ||
echo "asciidoctor is unavailable" >&2


# libcap: CentOS 7, Fedora 28, Fedora 29
# libcap2: OpenSUSE Leap 42.3, OpenSUSE Leap 15.0
add_package ADDITIONAL_LIST libcap || add_package ADDITIONAL_LIST libcap2 ||
echo "libcap is unavailable" >&2

add_package ADDITIONAL_LIST nghttp2-devel || add_package ADDITIONAL_LIST libnghttp2-devel ||
echo "nghttp2 is unavailable" >&2

add_package ADDITIONAL_LIST snappy || add_package ADDITIONAL_LIST libsnappy1 ||
echo "snappy is unavailable" >&2

add_package ADDITIONAL_LIST libzstd-devel || echo "zstd is unavailable" >&2

add_package ADDITIONAL_LIST lz4-devel || add_package ADDITIONAL_LIST liblz4-devel ||
echo "lz4 devel is unavailable" >&2

add_package ADDITIONAL_LIST libcap-progs || echo "cap progs are unavailable" >&2

add_package ADDITIONAL_LIST libmaxminddb-devel ||
echo "MaxMind DB devel is unavailable" >&2

add_package ADDITIONAL_LIST gnutls-devel || add_package ADDITIONAL_LIST libgnutls-devel ||
echo "gnutls devel is unavailable" >&2

add_package ADDITIONAL_LIST gettext-devel || add_package ADDITIONAL_LIST gettext-tools ||
echo "Gettext devel is unavailable" >&2

add_package ADDITIONAL_LIST perl-Pod-Html ||
echo "perl-Pod-Html is unavailable" >&2

add_package ADDITIONAL_LIST ninja || add_package ADDITIONAL_LIST ninja-build ||
echo "ninja is unavailable" >&2

add_package ADDITIONAL_LIST libxslt || add_package ADDITIONAL_LIST libxslt1 ||
echo "xslt is unavailable" >&2

add_package ADDITIONAL_LIST brotli-devel || add_packages ADDITIONAL_LIST libbrotli-devel libbrotlidec1 ||
echo "brotli is unavailable" >&2

add_package ADDITIONAL_LIST git-review ||
echo "git-review is unavailabe" >&2

add_package ADDITIONAL_LIST speexdsp-devel || add_package ADDITIONAL_LIST speex-devel ||
echo "speex is unavailable" >&2

add_package ADDITIONAL_LIST libnl3-devel || add_package ADDITIONAL_LIST libnl-devel ||
echo "libnl3/libnl are unavailable" >&2

add_package ADDITIONAL_LIST ilbc-devel ||
echo "ilbc is unavailable" >&2

# opus-devel: RHEL/CentOS, Fedora
# libopus-devel: OpenSUSE
add_package ADDITIONAL_LIST opus-devel || add_package ADDITIONAL_LIST libopus-devel ||
echo "opus is unavailable" >&2

add_package ADDITIONAL_LIST bcg729-devel ||
echo "bcg729 is unavailable" >&2

# RHEL 8 / CentOS 8 are missing the -devel packages for sbc and libsmi due to
# RH deciding not to ship all -devel packages.
# https://wiki.centos.org/FAQ/CentOS8/UnshippedPackages
# There are CentOS bugs filed to add them to the Devel repository and eventually
# RHEL 8 CRB / CentOS PowerTools, but make them optional for now.
# https://bugs.centos.org/view.php?id=16504
# https://bugs.centos.org/view.php?id=17824
add_package ADDITIONAL_LIST sbc-devel ||
echo "sbc is unavailable"

add_package ADDITIONAL_LIST libsmi-devel ||
echo "libsmi is unavailable"

ACTUAL_LIST=$BASIC_LIST

# Now arrange for optional support libraries
if [ $ADDITIONAL -ne 0 ]
then
	ACTUAL_LIST="$ACTUAL_LIST $ADDITIONAL_LIST"
fi

if [ $RPMDEPS -ne 0 ]
then
	ACTUAL_LIST="$ACTUAL_LIST $RPMDEPS_LIST"
fi

$PM $PM_OPT install $ACTUAL_LIST $OPTIONS

if [ $ADDITIONAL -eq 0 ]
then
	echo -e "\n*** Optional packages not installed. Rerun with --install-optional to have them.\n"
fi

if [ $RPMDEPS -eq 0 ]
then
	printf "\n*** RPM packages build deps not installed. Rerun with --install-rpm-deps to have them.\n"
fi
