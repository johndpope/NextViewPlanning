# - Try to find ANN
# Once done this will define
#
#  ANN_FOUND        - system has ANN
#  ANN_INCLUDE_DIR  - the ANN include directory
#  ANN_LIBRARY_DIR  - Link these to use ANN
#

IF (ANN_INCLUDE_DIRS)
    # Already in cache, be silent
    SET(ANN_FIND_QUIETLY TRUE)
ENDIF (ANN_INCLUDE_DIRS)

FIND_PATH( ANN_INCLUDE_DIR ANN/ANN.h
        PATHS "/Users/ucaHome/libraries/ann_1.1.2/include")

FIND_LIBRARY( ANN_LIBRARY
        NAMES ann ANN
        PATHS "/Users/ucaHome/libraries/ann_1.1.2/lib")

GET_FILENAME_COMPONENT( ANN_LIBRARY_DIR ${ANN_LIBRARY} PATH )


IF (ANN_INCLUDE_DIR AND ANN_LIBRARY)
    SET(ANN_FOUND TRUE)
ELSE (ANN_INCLUDE_DIR AND ANN_LIBRARY)
    SET( ANN_FOUND FALSE )
ENDIF (ANN_INCLUDE_DIR AND ANN_LIBRARY)
