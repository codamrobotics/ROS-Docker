#!/bin/sh

#
## Returns absolute path of a folder
# should be reasonable cross posix fun
# source: http://stackoverflow.com/a/18443300/441757
# Edited by Stan Verschuuren for POSIX-compliancy

realpath() {                                                                       
    OURPWD=$PWD                                                                 
    cd "$(dirname "$1")" || exit 1
    LINK=$(readlink "$(basename "$1")")                                         
    while [ "$LINK" ]; do                                                       
      cd "$(dirname "$LINK")" || exit 1
      LINK=$(readlink "$(basename "$1")")                                       
    done                                                                        
    REALPATH="$PWD/$(basename "$1")"                                            
    [ "${REALPATH: -1}" = "." ] && [ ! ${#REALPATH} -eq 1 ] && REALPATH=$(echo $REALPATH | sed 's/.$//')
    [ "${REALPATH: -1}" = "/" ] && [ ! ${#REALPATH} -eq 1 ] && REALPATH=$(echo $REALPATH | sed 's/\/$//')
    cd "$OURPWD" || exit 1
    echo "$REALPATH"                                                            
} # End of realpath()
