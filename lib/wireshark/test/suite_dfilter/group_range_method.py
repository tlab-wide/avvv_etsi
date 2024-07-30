# Copyright (c) 2013 by Gilbert Ramirez <gram@alumni.rice.edu>
#
# SPDX-License-Identifier: GPL-2.0-or-later

import unittest
import fixtures
from suite_dfilter.dfiltertest import *


@fixtures.uses_fixtures
class case_range(unittest.TestCase):
    trace_file = "ipx_rip.pcap"

    def test_slice_1_pos(self, checkDFilterCount):
        dfilter = "ipx.src.node[1] == aa"
        checkDFilterCount(dfilter, 1)

    def test_slice_1_neg(self, checkDFilterCount):
        dfilter = "ipx.src.node[1] == bb"
        checkDFilterCount(dfilter, 0)

    def test_slice_1_hex_pos(self, checkDFilterCount):
        dfilter = "ipx.src.node[1] == 0xaa"
        checkDFilterCount(dfilter, 1)

    def test_slice_1_hex_neg(self, checkDFilterCount):
        dfilter = "ipx.src.node[1] == 0xbb"
        checkDFilterCount(dfilter, 0)

    def test_slice_2_pos(self, checkDFilterCount):
        dfilter = "ipx.src.node[3:2] == a3:e3"
        checkDFilterCount(dfilter, 1)

    def test_slice_2_neg(self, checkDFilterCount):
        dfilter = "ipx.src.node[3:2] == cc:dd"
        checkDFilterCount(dfilter, 0)

    def test_slice_string_1(self, checkDFilterFail):
        dfilter = "frame == \"00\"[1]"
        checkDFilterFail(dfilter, "Range is not supported for entity 00 of type STRING")

    def test_slice_unparsed_1(self, checkDFilterFail):
        dfilter = "a == b[1]"
        checkDFilterFail(dfilter, "Range is not supported for entity b of type UNPARSED")

    def test_slice_func_1(self, checkDFilterSucceed):
        dfilter = "string(ipx.src.node)[3:2] == \"cc:dd\""
        checkDFilterSucceed(dfilter)
