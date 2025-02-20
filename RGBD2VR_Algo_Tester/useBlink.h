//include Blink core objects and API.
#include "blink/api/bl_face_detector.h"
#include "blink/api/bl_headpose_detector.h"
#include "blink/api/bl_3d_landmark_detector.h"
#include "blink/api/bl_exception.h"
#include "blink/api/bl_context.h"
#include "blink/api/bl_vector.h"

// Sample common
#include "blink_types.h"
#include "blink_utils.h"
#include "blink_capture.h"
//#include "blink_ui.h"
#include "blink_input.h"
#include "blink_output.h"

constexpr uint32_t NUM_OF_SOURCES = 1;       // In this sample we use one camera hence the number of inputs is one
constexpr uint32_t SOURCE_INDEX = 0;         // Camera source index                                                          
constexpr uint32_t MAX_NUM_OF_FACES = 3;     // The face detector will scan the frame to detect up to this number of faces   
constexpr uint32_t WAIT_KEY_MS = 1;          // Amount of milliseconds to wait between frames                                                          
#pragma once
