#!/usr/bin/env python

# Run clang-tidy recursively and parallel on directory
# Usage: run-clang-tidy sourcedir builddir excludedirs extensions
# extensions and excludedirs are specified as comma-separated 
# string without dot, e.g. 'c,cpp' 
# e.g. run-clang-tidy . build test,other c,cpp file

import os, sys, subprocess, multiprocessing
import json

manager = multiprocessing.Manager()
failedfiles = manager.list()

# Get absolute paths from arguments
print("Arguments: " + str(sys.argv))
sourcedir = os.path.abspath(sys.argv[1])
print("Source directory: " + sourcedir)
builddir = os.path.abspath(sys.argv[2])
print("Build directory: " + builddir)
# If exclude dirs is not empty, split it into a tuple
excludedirs = ()
if len(sys.argv) > 3 and sys.argv[3]:
    excludedirs = tuple([os.path.join(sourcedir, s) for s in sys.argv[3].split(',')])
# If the build directory is not the same as the source directory, exclude it
if not sourcedir == builddir:
    excludedirs = excludedirs + (builddir,)

print("Exclude directories: " + str(excludedirs))
# Split extensions into a tuple
extensions = ()
if(len(sys.argv) >= 4):
    extensions = tuple([("." + s) for s in sys.argv[4].split(',')])
print("Extensions: " + str(extensions))

def runclangtidy(filepath):
    print("Checking: " + filepath)
    proc = subprocess.Popen(f'clang-tidy -warnings-as-errors=* -p {builddir} {filepath}', shell=True)
    if proc.wait() != 0:
        failedfiles.append(filepath)

def collectfiles(dir, exclude, exts):
    collectedfiles = []
    for root, dirs, files in os.walk(dir):
        root = os.path.abspath(root)
        for file in files:
            filepath = os.path.join(root, file)
            if (len(exclude) == 0 or not filepath.startswith(exclude)) and filepath.endswith(exts):
                collectedfiles.append(filepath)
    return collectedfiles

def collectcompiledfiles(dir, builddir):
    collectedfiles = []
    if(builddir):
        compile_file = open(builddir + '/compile_commands.json')
        if compile_file is not None:
            compile_data = json.load(compile_file)
            for line in compile_data:
                filepath = line["file"]
                if dir in filepath and not builddir in filepath:
                    collectedfiles.append(filepath)

    return collectedfiles

# Define the pool AFTER the global variables and subprocess function because WTF python
# See: https://stackoverflow.com/questions/41385708/multiprocessing-example-giving-attributeerror
pool = multiprocessing.Pool()
files = collectcompiledfiles(sourcedir, builddir)
if len(files) == 0:
    files = collectfiles(sourcedir, excludedirs, extensions)
pool.map(runclangtidy, files)
pool.close()
pool.join()
if len(failedfiles) > 0:
    print("Errors in " + str(len(failedfiles)) + " files")
    sys.exit(1)
print("No errors found")
sys.exit(0)