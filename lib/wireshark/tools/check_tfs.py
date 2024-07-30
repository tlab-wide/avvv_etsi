#!/usr/bin/env python3
# Wireshark - Network traffic analyzer
# By Gerald Combs <gerald@wireshark.org>
# Copyright 1998 Gerald Combs
#
# SPDX-License-Identifier: GPL-2.0-or-later

import os
import re
import subprocess
import argparse
import signal

# This utility scans for tfs items, and works out if standard ones
# could have been used intead (from epan/tfs.c)

# TODO:
# - check how many of the definitions in epan/tfs.c are used in other dissectors
# - see if there are other values that should be in epan/tfs.c and shared


# Try to exit soon after Ctrl-C is pressed.
should_exit = False

def signal_handler(sig, frame):
    global should_exit
    should_exit = True
    print('You pressed Ctrl+C - exiting')

signal.signal(signal.SIGINT, signal_handler)


# Keep track of custom entries that might appear in multiple dissectors,
# so we can consider adding them to tfs.c
custom_tfs_entries = {}
def AddCustomEntry(val1, val2, file):
    global custom_tfs_entries
    if (val1, val2) in custom_tfs_entries:
        custom_tfs_entries[(val1, val2)].append(file)
    else:
        custom_tfs_entries[(val1, val2)] = [file]



class TFS:
    def __init__(self, file, name, val1, val2):
        self.file = file
        self.name = name
        self.val1 = val1
        self.val2 = val2

        # Do some extra checks on values.
        if val1.startswith(' ') or val1.endswith(' '):
            print('N.B.: file=' + self.file + ' ' + self.name + ' - false val begins or ends with space \"' + self.val1 + '\"')
        if val2.startswith(' ') or val2.endswith(' '):
            print('N.B.: file=' + self.file + ' ' + self.name + ' - true val begins or ends with space \"' + self.val2 + '\"')

    def __str__(self):
        return '{' + '"' + self.val1 + '", "' + self.val2 + '"}'


def removeComments(code_string):
    code_string = re.sub(re.compile(r"/\*.*?\*/",re.DOTALL ) ,"" ,code_string) # C-style comment
    code_string = re.sub(re.compile(r"//.*?\n" ) ,"" ,code_string)             # C++-style comment
    return code_string


# Look for hf items in a dissector file.
def findItems(filename):
    items = {}

    with open(filename, 'r') as f:
        contents = f.read()
        # Example: const true_false_string tfs_true_false = { "True", "False" };

        # Remove comments so as not to trip up RE.
        contents = removeComments(contents)

        matches =   re.finditer(r'.*const\s*true_false_string\s*([a-z_]*)\s*=\s*{\s*\"([a-zA-Z_ ]*)\"\s*,\s*\"([a-zA-Z_ ]*)\"', contents)
        for m in matches:
            name = m.group(1)
            val1 = m.group(2)
            val2 = m.group(3)
            # Store this entry.
            items[name] = TFS(filename, name, val1, val2)

    return items



def is_dissector_file(filename):
    p = re.compile(r'.*packet-.*\.c')
    return p.match(filename)

def findDissectorFilesInFolder(folder):
    # Look at files in sorted order, to give some idea of how far through is.
    files = []

    for f in sorted(os.listdir(folder)):
        if should_exit:
            return
        if is_dissector_file(f):
            filename = os.path.join(folder, f)
            files.append(filename)
    return files

warnings_found = 0
errors_found = 0

# Check the given dissector file.
def checkFile(filename, tfs_items, look_for_common=False):
    global warnings_found
    global errors_found

    # Check file exists - e.g. may have been deleted in a recent commit.
    if not os.path.exists(filename):
        print(filename, 'does not exist!')
        return

    # Find items.
    items = findItems(filename)

    # See if any of these items already existed in tfs.c
    for i in items:
        for t in tfs_items:
            found = False

            #
            # Do not do this check for plugins; plugins cannot import
            # data values from libwireshark (functions, yes; data
            # values, no).
            #
            # Test whether there's a common prefix for the file name
            # and "plugin/epan/"; if so, this is a plugin, and there
            # is no common path and os.path.commonprefix returns an
            # empty string, otherwise it returns the common path, so
            # we check whether the common path is an empty string.
            #
            if os.path.commonprefix([filename, 'plugin/epan/']) == '':
                exact_case = False
                if tfs_items[t].val1 == items[i].val1 and tfs_items[t].val2 == items[i].val2:
                    found = True
                    exact_case = True
                elif tfs_items[t].val1.upper() == items[i].val1.upper() and tfs_items[t].val2.upper() == items[i].val2.upper():
                    found = True

                if found:
                    print(filename, i, "- could have used", t, 'from tfs.c instead: ', tfs_items[t],
                          '' if exact_case else '  (capitalisation differs)')
                    if exact_case:
                        errors_found += 1
                    else:
                        warnings_found += 1
                    break
        if not found:
            if look_for_common:
                AddCustomEntry(items[i].val1, items[i].val2, filename)


#################################################################
# Main logic.

# command-line args.  Controls which dissector files should be checked.
# If no args given, will just scan epan/dissectors folder.
parser = argparse.ArgumentParser(description='Check calls in dissectors')
parser.add_argument('--file', action='store', default='',
                    help='specify individual dissector file to test')
parser.add_argument('--commits', action='store',
                    help='last N commits to check')
parser.add_argument('--open', action='store_true',
                    help='check open files')
parser.add_argument('--common', action='store_true',
                    help='check for potential new entries for tfs.c')


args = parser.parse_args()


# Get files from wherever command-line args indicate.
files = []
if args.file:
    # Add single specified file..
    if not args.file.startswith('epan'):
        files.append(os.path.join('epan', 'dissectors', args.file))
    else:
        files.append(args.file)
elif args.commits:
    # Get files affected by specified number of commits.
    command = ['git', 'diff', '--name-only', 'HEAD~' + args.commits]
    files = [f.decode('utf-8')
             for f in subprocess.check_output(command).splitlines()]
    # Will examine dissector files only
    files = list(filter(lambda f : is_dissector_file(f), files))
elif args.open:
    # Unstaged changes.
    command = ['git', 'diff', '--name-only']
    files = [f.decode('utf-8')
             for f in subprocess.check_output(command).splitlines()]
    # Only interested in dissector files.
    files = list(filter(lambda f : is_dissector_file(f), files))
    # Staged changes.
    command = ['git', 'diff', '--staged', '--name-only']
    files_staged = [f.decode('utf-8')
                    for f in subprocess.check_output(command).splitlines()]
    # Only interested in dissector files.
    files_staged = list(filter(lambda f : is_dissector_file(f), files_staged))
    for f in files_staged:
        if not f in files:
            files.append(f)
else:
    # Find all dissector files from folder.
    files = findDissectorFilesInFolder(os.path.join('epan', 'dissectors'))


# If scanning a subset of files, list them here.
print('Examining:')
if args.file or args.commits or args.open:
    if files:
        print(' '.join(files), '\n')
    else:
        print('No files to check.\n')
else:
    print('All dissector modules\n')


# Get standard/ shared ones.
tfs_entries = findItems(os.path.join('epan', 'tfs.c'))

# Now check the files to see if they could have used shared ones instead.
for f in files:
    if should_exit:
        exit(1)
    checkFile(f, tfs_entries, look_for_common=args.common)


# Show summary.
print(warnings_found, 'warnings found')
if errors_found:
    print(errors_found, 'errors found')
    exit(1)

if args.common:
    # Looking for items that could potentially be moved to tfs.c
    for c in custom_tfs_entries:
        # Only want to see items that have 3 or more occurrences.
        # Even then, probably only want to consider ones that sound generic.
        if len(custom_tfs_entries[c]) > 2:
            print(c, 'appears', len(custom_tfs_entries[c]), 'times, in: ', custom_tfs_entries[c])
